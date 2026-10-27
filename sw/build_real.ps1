# ============================================================================
# build_real.ps1 — real YOLOv3-tiny layer-0 image for the Pango port
# ----------------------------------------------------------------------------
# Compiles sw/start.S + sw/yolo_real.c + sw/uart.c against a generated
# yolo_real_m3a.h / yolo_real_m3b.h, links with sw/board.ld, and emits
# sw/build/firmware_real.hex (one 32-bit word per line).
#
# Usage:
#   .\build_real.ps1                                  # full pooled layer, 43 chunks
#   .\build_real.ps1 -Header ..\sim\vec_real\yolo_real_m3a.h
#   .\build_real.ps1 -Limit 3                         # reduced image for SoC sim
# ============================================================================
[CmdletBinding()]
param(
    [string] $Header = '..\sim\vec_real\yolo_real_m3b.h',
    [int]    $Limit = 0,
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
$hdrPath = if ([IO.Path]::IsPathRooted($Header)) { $Header } else { Join-Path $root $Header }
$hdr = Resolve-Path $hdrPath -ErrorAction Stop
Copy-Item -LiteralPath $hdr -Destination (Join-Path $root 'yolo_real_vectors.h') -Force
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
if ($Limit -gt 0) { $cflags += "-DRL_LIMIT=$Limit" }

$srcs = @('start.S', 'yolo_real.c', 'uart.c')

Push-Location $root
try {
    Write-Host '== YOLO real layer-0 firmware build ==' -ForegroundColor Cyan
    Write-Host "   header : $hdr"
    Write-Host "   arch   : rv32imc_zicsr ($Opt)   chunk limit: $(if ($Limit -gt 0) { $Limit } else { 'all' })"

    $elf = Join-Path $buildDir 'yolo_real.elf'
    $map = Join-Path $buildDir 'yolo_real.map'

    & $CC @cflags @srcs `
        "-Wl,-T,$(Join-Path $root 'board.ld')" `
        "-Wl,-Map,$map" `
        '-Wl,--gc-sections' `
        '-o' $elf '-lgcc'
    if ($LASTEXITCODE -ne 0) { throw "link failed (exit $LASTEXITCODE)" }

    & (Join-Path $bin 'riscv-none-elf-size.exe') $elf
    & (Join-Path $bin 'riscv-none-elf-objdump.exe') -d $elf |
        Set-Content -LiteralPath (Join-Path $buildDir 'yolo_real.lst')

    $binFile = Join-Path $buildDir 'yolo_real.bin'
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
    $hexPath = Join-Path $buildDir 'firmware_real.hex'
    [IO.File]::WriteAllText($hexPath, $sb.ToString())
    $words = [int][Math]::Ceiling($bytes.Length / 4)
    Write-Host ("   hex    : build\firmware_real.hex  ($words words, $($bytes.Length) bytes)")
    Write-Host '   OK' -ForegroundColor Green
}
finally {
    Pop-Location
}