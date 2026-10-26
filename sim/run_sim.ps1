# ============================================================================
# run_sim.ps1 — 用 Vivado XSim 跑一次 CoreMark（验证 CRC，不出比特流）
# ----------------------------------------------------------------------------
# 用法:  先 build_coremark.ps1 -Iterations 5，把 build\firmware.hex 复制到
#        本目录，然后运行本脚本。tb_coremark.v 监听 UART 发送寄存器写，
#        直接还原串口字符流；看到 "Correct operation validated." 即通过。
# ============================================================================
$ErrorActionPreference = 'Stop'

$V    = 'D:\SOFTWARE\Xilinx\Vivado\2018.3\bin'
$root = Split-Path (Split-Path $PSScriptRoot)   # coremark\sim -> coremark -> 工程根

$rtl = @(
    "$root\rtl\core\alu.v",
    "$root\rtl\core\branch_predictor.v",
    "$root\rtl\core\brv32p_core.v",
    "$root\rtl\core\compressed_decoder.v",
    "$root\rtl\core\csr.v",
    "$root\rtl\core\decoder.v",
    "$root\rtl\core\hazard_unit.v",
    "$root\rtl\core\muldiv.v",
    "$root\rtl\core\regfile.v",
    "$root\rtl\cache\dcache.v",
    "$root\rtl\cache\icache.v",
    "$root\rtl\bus\axi_interconnect.v",
    "$root\rtl\bus\axi_periph_bridge.v",
    "$root\rtl\bus\axi_sram.v",
    "$root\rtl\periph\gpio.v",
    "$root\rtl\periph\timer.v",
    "$root\rtl\periph\uart.v",
    "$root\rtl\brv32p_soc.v"
)

Push-Location $PSScriptRoot
try {
    Write-Host '== xvlog ==' -ForegroundColor Cyan
    & "$V\xvlog.bat" -i "$root\rtl\pkg" @rtl '.\tb_coremark.v'
    if ($LASTEXITCODE -ne 0) { throw 'xvlog failed' }

    Write-Host '== xelab ==' -ForegroundColor Cyan
    & "$V\xelab.bat" tb_coremark -s cm_sim --timescale 1ns/1ps
    if ($LASTEXITCODE -ne 0) { throw 'xelab failed' }

    Write-Host '== xsim (这是模拟时间，不是墙钟时间) ==' -ForegroundColor Cyan
    $out = & "$V\xsim.bat" cm_sim -runall 2>&1
    $out | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'sim_output.txt')

    $flat = ($out -join "`n") -replace '\s', ''
    if ($flat -match 'Correctoperationvalidated' -or
        ($flat -match 'seedcrc:0xe9f5' -and $flat -match 'crclist:0xe714' -and
         $flat -match 'crcmatrix:0x1fd7' -and $flat -match 'crcstate:0x8e3a')) {
        Write-Host ' CoreMark : PASS (CRC 与官方参考值一致)' -ForegroundColor Green
    } else {
        Write-Host ' CoreMark : FAIL — 见 sim_output.txt' -ForegroundColor Red
    }
    Write-Host " 原始输出 : $PSScriptRoot\sim_output.txt"
}
finally {
    Pop-Location
}