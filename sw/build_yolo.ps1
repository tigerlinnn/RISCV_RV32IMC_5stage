# ============================================================================
# build_yolo.ps1 — Phase A YOLO test image for the RISCV_1_pango_yolo port
# ----------------------------------------------------------------------------
# Compiles sw/start.S + sw/yolo_main.c + sw/uart.c against a generated
# yolo_vectors.h, links with sw/board.ld, and emits sw/build/firmware.hex
# (32-bit words, one per line) for $readmemh / PDS INIT_FILE.
#
# Usage:
#   .\build_yolo.ps1                      # uses ..\sim\vec\C
#   .\build_yolo.ps1 -VectorDir ..\sim\vec\B
# ============================================================================
[CmdletBinding()]
param(
    [string] $VectorDir = '..\sim\vec\C',
    [ValidateSet('-O1', '-O2', '-O3', '-Os')]
    [string] $Opt = '-O2',
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
$vec = Resolve-Path (Join-Path $root $VectorDir) -ErrorAction Stop
$hdr = Join-Path $vec 'vectors.h'
if (-not (Test-Path -LiteralPath $hdr)) {
    throw "vectors.h not found in $vec - run tools/gen_vectors.py first."
}
Copy-Item -LiteralPath $hdr -Destination (Join-Path $root 'yolo_vectors.h') -Force
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$cflags = @(
    '-march=rv32imc_zicsr', '-mabi=ilp32', '-mcmodel=medlow',
    '-mno-relax',
    $Opt,
    '-ffunction-sections', '-fdata-sections', '-ffreestanding', '-fno-common',
    '-nostartfiles', '-nostdlib',
    '-I.',
    '-DUART_DIV_50MHZ_115200=233u'
)

$srcs = @('start.S', 'yolo_main.c', 'uart.c')

Push-Location $root
try {
    Write-Host '== YOLO Phase A firmware build (Pango 27 MHz) ==' -ForegroundColor Cyan
    Write-Host "   vectors : $vec"
    Write-Host "   arch    : rv32imc_zicsr ($Opt)"

    $elf = Join-Path $buildDir 'yolo.elf'
    $map = Join-Path $buildDir 'yolo.map'

    & $CC @cflags @srcs `
        "-Wl,-T,$(Join-Path $root 'board.ld')" `
        "-Wl,-Map,$map" `
        '-Wl,--gc-sections' `
        '-o' $elf '-lgcc'
    if ($LASTEXITCODE -ne 0) { throw "link failed (exit $LASTEXITCODE)" }

    & (Join-Path $bin 'riscv-none-elf-size.exe') $elf
    & (Join-Path $bin 'riscv-none-elf-objdump.exe') -d $elf |
        Set-Content -LiteralPath (Join-Path $buildDir 'yolo.lst')

    $binFile = Join-Path $buildDir 'yolo.bin'
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
    Write-Host ("   hex     : build\firmware.hex  ($words words, $($bytes.Length) bytes)")
    Write-Host '   OK' -ForegroundColor Green
}
finally {
    Pop-Location
}