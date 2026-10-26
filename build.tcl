# ============================================================================
# build.tcl — RISCV_1 (BRV32P SoC) -> Pango PG2L200H, full implementation flow
# ----------------------------------------------------------------------------
# Do NOT run PDS on this file directly: pds_shell appends every command it
# executes to the script it was given, so running it in place would rewrite it
# on every run.  Use ./build.sh, which runs a throwaway copy inside prj/ and
# then adopts the project file PDS writes as prj/riscv1_pango.pds.
#
# Flow:  add_design -> set_arch -> compile -> add_constraint -> synthesize
#        -> dev_map -> pnr -> report_timing -> gen_bit_stream
# (same order as the vendor's LCD_DEMO/prj/impl.tcl for this board, and as the
#  hardware-verified BRV32P_pango/build.tcl)
#
# rtl/ is a verbatim copy of RISCV_1's sources_1/new tree (only the directory
# layout is preserved); the .vh lives in rtl/pkg so -include_path points there.
# The only new source file is top/riscv1_pango_top.v, the board wrapper.
#
# -loop_limit 10000 : axi_sram.v zero-fills its 8192-word `mem` in an `initial`
#   for-loop; ADS's default loop limit is 2000, so the front-end aborts with
#   "Verilog-4092 Loop limit 2000 exceeded" unless this is raised.
# ============================================================================

set root {D:/UESTC/2026/QS/project/RISCV_1_pango_yolo}

# ── design sources ──────────────────────────────────────────────────────────
foreach f [list \
    rtl/core/alu.v \
    rtl/core/branch_predictor.v \
    rtl/core/brv32p_core.v \
    rtl/core/compressed_decoder.v \
    rtl/core/csr.v \
    rtl/core/decoder.v \
    rtl/core/hazard_unit.v \
    rtl/core/muldiv.v \
    rtl/core/regfile.v \
    rtl/cache/dcache.v \
    rtl/cache/icache.v \
    rtl/bus/axi_interconnect.v \
    rtl/bus/axi_periph_bridge.v \
    rtl/bus/axi_sram.v \
    rtl/periph/gpio.v \
    rtl/periph/timer.v \
    rtl/periph/uart.v \
    rtl/periph/accel_mmio.v \
    rtl/brv32p_soc.v \
    top/riscv1_pango_top.v \
] {
    add_design "$root/$f"
}

# CNN accelerator (pure Verilog, copied from the Zynq YOLOv3-tiny project)
foreach f [glob -nocomplain "$root/rtl/accel/*.v"] {
    add_design $f
}

# ── device ──────────────────────────────────────────────────────────────────
set_arch -family Logos2 -device PG2L200H -speedgrade -6 -package FBB676

# ── elaborate ───────────────────────────────────────────────────────────────
compile -include_path $root/rtl/pkg -loop_limit 10000 -top_module riscv1_pango_top

# ── pin + timing constraints ────────────────────────────────────────────────
add_constraint "$root/constr/riscv1_pango.fdc"

# ── synthesis (ADS) and implementation ──────────────────────────────────────
synthesize -ads -selected_syn_tool_opt 2

# SYNTH_ONLY=1 stops after elaboration + synthesis (fast smoke test, no PnR).
if {![info exists ::env(SYNTH_ONLY)] || $::env(SYNTH_ONLY) ne "1"} {
    dev_map
    pnr
    report_timing
    gen_bit_stream -master_configuration_clock_frequency {25M}
}

puts "============================================================="
puts " RISCV_1 -> PG2L200H flow complete"
puts " bitstream : $root/prj/generate_bitstream/riscv1_pango_top.sbit"
puts " timing    : $root/prj/report_timing/riscv1_pango_top.rtr"
puts "============================================================="