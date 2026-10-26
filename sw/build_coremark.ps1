<#
=============================================================================
 build_coremark.ps1 — CoreMark image for the RISCV_1_pango port
-----------------------------------------------------------------------------
 Produces, in .\build\:
     coremark.elf      linked ELF
     coremark.map      linker map
     coremark.lst      disassembly
     firmware.hex      flat $readmemh image (one 32-bit word per line)

 Defaults are for the Pango remote-lab board:
     system clock   27 MHz   (core-board FPGA_GCLK_27M on ball D18)
     UART divisor   233      -> 27e6/234 = 115385 baud (+0.16%)
     iterations     700      -> ~11.3 s at 27 MHz (CoreMark requires >= 10 s)

 The RISCV_1 port measures time with mcycle and its core_portme.h hardcodes
 CLOCKS_PER_SEC = 50000000, so CLOCKS_PER_SEC and the UART divisor MUST be
 overridden here — otherwise the printed score would be 50/27 too high and
 the serial output would be at 62 kbaud.

 Usage:
   .\build_coremark.ps1                   # 700 iterations (valid report)
   .\build_coremark.ps1 -Iterations 1     # quick image, report in ~16 ms
   .\build_coremark.ps1 -SlowPrint 20000  # spread report for the platform log

 -SlowPrint is optional and off by default; it only affects printing after
 stop_time(), so the score is unchanged.
=============================================================================#>

[CmdletBinding()]
param(
    [ValidateSet('-O1', '-O2', '-O3', '-Os')]
    [string] $Opt = '-O2',

    [int]    $Iterations    = 700,
    [int]    $TotalDataSize = 2000,
    [long]   $ClockHz       = 27000000,
    [int]    $UartClkDiv    = 233,

    # Busy-wait loop ticks after each transmitted character. 0 = normal speed.
    [int]    $SlowPrint     = 0,

    [string] $Toolchain = 'D:\UESTC\2026\tools\xpack-riscv-none-elf-gcc-15.2.0-1'
)

$ErrorActionPreference = 'Stop'

$root     = $PSScriptRoot
$buildDir = Join-Path $root 'build'
$bin      = Join-Path $Toolchain 'bin'
$CC       = Join-Path $bin 'riscv-none-elf-gcc.exe'

if (-not (Test-Path -LiteralPath $CC)) {
    throw "riscv-none-elf-gcc not found at $CC - pass -Toolchain <dir>."
}
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$cflags = @(
    '-march=rv32imc_zicsr', '-mabi=ilp32', '-mcmodel=medlow',
    '-mno-relax',
    $Opt,
    '-ffunction-sections', '-fdata-sections', '-ffreestanding', '-fno-common',
    '-nostartfiles', '-nostdlib',
    '-I.',
    "-DITERATIONS=$Iterations",
    "-DTOTAL_DATA_SIZE=$TotalDataSize",
    '-DPERFORMANCE_RUN=1',
    "-DCLOCKS_PER_SEC=$ClockHz",
    "-DUART_DIV_50MHZ_115200=${UartClkDiv}u"
)
if ($SlowPrint -gt 0) { $cflags += "-DUART_SLOW_PRINT=$SlowPrint" }

$srcs = @(
    'start.S',
    'core_main.c',
    'core_list_join.c',
    'core_matrix.c',
    'core_state.c',
    'core_util.c',
    'core_portme.c',
    'ee_printf.c',
    'cvt.c',
    'uart.c'
)

Push-Location $root
try {
    Write-Host '== RISCV_1 CoreMark build (Pango 27 MHz) ==' -ForegroundColor Cyan
    Write-Host "   arch      : rv32imc_zicsr ($Opt)"
    Write-Host "   clock     : $ClockHz Hz   uart div $UartClkDiv"
    Write-Host "   coremark  : ITERATIONS=$Iterations  TOTAL_DATA_SIZE=$TotalDataSize"
    if ($SlowPrint -gt 0) { Write-Host "   slowprint : $SlowPrint ticks/char" }

    $elf = Join-Path $buildDir 'coremark.elf'
    $map = Join-Path $buildDir 'coremark.map'

    & $CC @cflags @srcs `
        "-Wl,-T,$(Join-Path $root 'board.ld')" `
        "-Wl,-Map,$map" `
        '-Wl,--gc-sections' `
        '-o' $elf '-lm' '-lgcc'
    if ($LASTEXITCODE -ne 0) { throw "link failed (exit $LASTEXITCODE)" }

    & (Join-Path $bin 'riscv-none-elf-size.exe') $elf

    $secs = & (Join-Path $bin 'riscv-none-elf-objdump.exe') -h $elf
    $imageBytes = 0
    $bssBytes   = 0
    foreach ($line in $secs) {
        if ($line -match '^\s*\d+\s+(\.text|\.rodata|\.data)\s+([0-9a-f]+)') {
            $imageBytes += [Convert]::ToInt64($Matches[2], 16)
        } elseif ($line -match '^\s*\d+\s+\.bss\s+([0-9a-f]+)') {
            $bssBytes = [Convert]::ToInt64($Matches[1], 16)
        }
    }
    $sramBytes = 32768
    Write-Host ('   image     : {0} bytes  (+ bss {1} bytes)' -f $imageBytes, $bssBytes)
    Write-Host ('   SRAM      : {0} of {1} bytes used, {2} left for the stack' -f `
                    ($imageBytes + $bssBytes), $sramBytes, ($sramBytes - $imageBytes - $bssBytes))

    & (Join-Path $bin 'riscv-none-elf-objdump.exe') -d $elf |
        Set-Content -LiteralPath (Join-Path $buildDir 'coremark.lst')

    # ---- flat hex image for $readmemh (little-endian words, one per line) --
    $binFile = Join-Path $buildDir 'coremark.bin'
    & (Join-Path $bin 'riscv-none-elf-objcopy.exe') -O binary $elf $binFile
    if ($LASTEXITCODE -ne 0) { throw 'objcopy failed' }

    $bytes = [IO.File]::ReadAllBytes($binFile)
    $sb    = New-Object System.Text.StringBuilder
    for ($i = 0; $i -lt $bytes.Length; $i += 4) {
        $b0 = [uint32]$bytes[$i]
        $b1 = if ($i + 1 -lt $bytes.Length) { [uint32]$bytes[$i + 1] } else { 0 }
        $b2 = if ($i + 2 -lt $bytes.Length) { [uint32]$bytes[$i + 2] } else { 0 }
        $b3 = if ($i + 3 -lt $bytes.Length) { [uint32]$bytes[$i + 3] } else { 0 }
        $word = $b0 -bor ($b1 -shl 8) -bor ($b2 -shl 16) -bor ($b3 -shl 24)
        [void]$sb.AppendLine($word.ToString('X8'))
    }
    $hexPath = Join-Path $buildDir 'firmware.hex'
    [IO.File]::WriteAllText($hexPath, $sb.ToString())
    $words = [int][Math]::Ceiling($bytes.Length / 4)
    Write-Host ("   hex       : build\firmware.hex  ($words words, $($bytes.Length) bytes)")

    Write-Host '   OK' -ForegroundColor Green
}
finally {
    Pop-Location
}