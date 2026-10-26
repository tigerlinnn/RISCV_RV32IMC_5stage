#!/usr/bin/env bash
# ============================================================================
# build.sh — implement RISCV_1 on the Pango PG2L200H and produce a bitstream
# ----------------------------------------------------------------------------
# Wraps pds_shell so the canonical flow script (build.tcl, next to this file)
# is never touched by the tool (pds_shell appends every executed command back
# into the script it was given, and names the generated project after it).
# We hand it a throwaway copy inside prj/ and adopt the project file it wrote
# as prj/riscv1_pango.pds.
#
# FIRMWARE: prj/firmware.hex is the active SRAM image ($readmemh's INIT_FILE
# is the relative name "firmware.hex" and PDS resolves it against the work
# dir).  By default this script copies firmware/firmware.hex (27 MHz build,
# UART divider 233).  Override with:
#     FIRMWARE=/path/to/other.hex ./build.sh
#
# Outputs (in prj/):
#     generate_bitstream/riscv1_pango_top.sbit   bitstream
#     report_timing/riscv1_pango_top.rtr         post-PnR timing
#     pds_run.log                                full console log
#
# Override the toolchain with  PDS=/path/to/pds_shell.exe ./build.sh
# ============================================================================
set -euo pipefail

PDS=${PDS:-D:/SOFTWARE/PDS_2022.2/bin/pds_shell.exe}
FIRMWARE=${FIRMWARE:-}
here=$(cd "$(dirname "$0")" && pwd)

cd "$here/prj"
if [ -n "$FIRMWARE" ]; then
    cp -f "$FIRMWARE" firmware.hex
else
    cp -f "$here/firmware/firmware.hex" firmware.hex
fi
cp -f ../build.tcl _run.tcl
"$PDS" -file _run.tcl -work_dir . 2>&1 | tee pds_run.log
cp -f _run.pds riscv1_pango.pds

echo
echo "bitstream : $here/prj/generate_bitstream/riscv1_pango_top.sbit"
echo "timing    : $here/prj/report_timing/riscv1_pango_top.rtr"
echo "project   : $here/prj/riscv1_pango.pds  (refreshed)"
echo "Check WNS >= 0 and 'All Constraints Met' in the timing report."