#!/usr/bin/env python3
"""Generate real-layer (upstream YOLOv3-tiny layer 0) vectors + golden.

Uses the trained model in the upstream Vitis/Python project:
    python_prj/infer_bin/{FW0,FB0,FR0,FCG}.bin

Cases:
    m1  6x15, 3->16 (2 OC groups), conv only, real weights/quant/LUT
    m2  8x418 (sel=5), one OC group, conv only, full-width single chunk
    m3  (later) chunked 416x256 layer 0 with pooling

Golden arithmetic mirrors the RTL exactly:
  * products are exact int8*int8
  * per input channel: 18-bit wrapping tree (p0+p1+p2),(p3+p4+p5),(p6+p7+p8)
  * 8 channels + bias:    18-bit wrapping tree (c0+c1+c2),(c3+c4+c5),(c6+c7+bias)
  * quant: sat18->16, *mult, >>>(15+shift), +bit(14+shift), sat8, +zp_out (8-bit)
  * activation: LUT[quant_result] (layer 0 uses leaky LUT) or ReLU
  * border: act(quant(0)) -- bias/acc are zeroed by the global_zero path

Usage:
    python gen_real_layer.py m1 [outdir]
    python gen_real_layer.py m2 [outdir]
"""
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_vectors as gv  # noqa: E402

UPSTREAM = ("D:/UESTC/2026/QS/fpga_accelerator_yolov3tiny-main/"
            "fpga_accelerator_yolov3tiny-main")
CONV_COL = {0: 15, 1: 28, 2: 54, 3: 106, 4: 210, 5: 418}

WRAP18 = 1 << 18
HALF18 = 1 << 17


def wrap18(x):
    return ((x + HALF18) & (WRAP18 - 1)) - HALF18


def load_upstream(root):
    def read(name):
        with open(os.path.join(root, "python_prj", "infer_bin", name), "rb") as f:
            return f.read()

    fcg = list(struct.unpack("<100I", read("FCG.bin")[:400]))
    cfg = {
        "ifm": fcg[0 * 10 + 0],
        "ofm": fcg[1 * 10 + 0],
        "mult": fcg[2 * 10 + 0],
        "shift": fcg[3 * 10 + 0],
        "zp_in": fcg[4 * 10 + 0],
        "zp_out": fcg[5 * 10 + 0],
        "zp_act": fcg[6 * 10 + 0],
        "sel": fcg[7 * 10 + 0],
        "pool": fcg[8 * 10 + 0],
        "stride": fcg[9 * 10 + 0],
    }
    wgt = np.frombuffer(read("FW0.bin"), dtype=np.int8).reshape(2, 8, 9, 8)
    bias = np.frombuffer(read("FB0.bin"), dtype=np.int64)
    lut64 = np.frombuffer(read("FR0.bin"), dtype=np.uint64)
    lut = [int(x) & 0xFF for x in lut64]
    return cfg, wgt, bias, lut


def ifm_pattern(r, c):
    """IFM generator mirrored in sw/yolo_real.c (3 real channels + 5 zeros)."""
    return [((5 * r + 3 * c + 7) & 0x7F),
            ((2 * r + 11 * c + 1) & 0x7F),
            ((13 * r + 7 * c + 9) & 0x7F), 0, 0, 0, 0, 0]


def golden_group(ifm, wg, bias_g, rows, cols, cfg, lut):
    """One 8-output-channel group over the full bordered grid."""
    zp_in = cfg["zp_in"]
    border = gv.activate(gv.quantize(0, cfg["mult"], cfg["shift"],
                                     cfg["zp_out"]),
                         cfg["relu_type"], cfg["zp_out"], lut)
    out = []
    for r in range(rows):
        for c in range(cols):
            if r == 0 or r == rows - 1 or c == 0 or c == cols - 1:
                out.append([border] * 8)
                continue
            beat = []
            for oc in range(8):
                ch_acc = []
                for ic in range(8):
                    p = []
                    for tap in range(9):
                        dr, dc = divmod(tap, 3)
                        qin = gv.wrap8(ifm[r - 1 + dr][c - 1 + dc][ic] -
                                       zp_in)
                        p.append(qin * int(wg[ic][tap][oc]))
                    b1 = wrap18(p[0] + p[1] + p[2])
                    b2 = wrap18(p[3] + p[4] + p[5])
                    b3 = wrap18(p[6] + p[7] + p[8])
                    ch_acc.append(wrap18(b1 + b2 + b3))
                ob1 = wrap18(ch_acc[0] + ch_acc[1] + ch_acc[2])
                ob2 = wrap18(ch_acc[3] + ch_acc[4] + ch_acc[5])
                ob3 = wrap18(ch_acc[6] + ch_acc[7] + int(bias_g[oc]))
                acc = wrap18(ob1 + ob2 + ob3)
                q = gv.quantize(acc, cfg["mult"], cfg["shift"], cfg["zp_out"])
                beat.append(gv.activate(q, cfg["relu_type"], cfg["zp_out"], lut))
            out.append(beat)
    return out


def weight_beats(wgt, groups):
    beats = []
    for g in range(groups):
        for ic in range(8):
            for tap in range(9):
                beats.append([int(wgt[g][ic][tap][oc]) for oc in range(8)])
    return beats


# ── vectorised golden model (numpy) ────────────────────────────────────────
def _wrap18(a):
    return ((a + HALF18) & (WRAP18 - 1)) - HALF18


def _quant_act_np(acc, cfg, lut):
    v = np.clip(acc, -32768, 32767).astype(np.int64)
    prod = v * int(cfg["mult"])
    shifted = prod >> (15 + int(cfg["shift"]))
    rnd = (prod >> (14 + int(cfg["shift"]))) & 1
    q = np.clip(shifted + rnd, -128, 127)
    q = (q + int(cfg["zp_out"])) & 0xFF
    lut_arr = np.asarray(lut, dtype=np.int64)
    return lut_arr[q]


def golden_group_np(ifm, wg, bias_g, rows, cols, cfg, lut):
    """Exact RTL-arithmetic golden for one 8-OC group; returns (rows*cols, 8)."""
    ifm_arr = np.asarray(ifm, dtype=np.int64)          # (rows, cols, 8)
    wg_np = np.asarray(wg, dtype=np.int64)             # (8 ic, 9 tap, 8 oc)
    bias = np.asarray([int(b) for b in bias_g], dtype=np.int64)
    border = int(_quant_act_np(np.array([0], dtype=np.int64), cfg, lut)[0])
    out = np.full((rows, cols, 8), border, dtype=np.int64)
    if rows <= 2 or cols <= 2:
        return out.reshape(-1, 8)
    win = np.lib.stride_tricks.sliding_window_view(ifm_arr, (3, 3),
                                                   axis=(0, 1))
    # win: (rows-2, cols-2, 8, 3, 3) -> (P, 8 ic, 9 tap)
    x = win.reshape(rows - 2, cols - 2, 8, 9)
    x = x.reshape(-1, 8, 9)
    prod = x[:, :, :, None] * wg_np[None, :, :, :]     # (P, 8, 9, 8)
    b1 = _wrap18(prod[:, :, 0:3, :].sum(axis=2))
    b2 = _wrap18(prod[:, :, 3:6, :].sum(axis=2))
    b3 = _wrap18(prod[:, :, 6:9, :].sum(axis=2))
    c = _wrap18(b1 + b2 + b3)                          # (P, 8 ic, 8 oc)
    ob1 = _wrap18(c[:, 0:3, :].sum(axis=1))
    ob2 = _wrap18(c[:, 3:6, :].sum(axis=1))
    ob3 = _wrap18(c[:, 6:8, :].sum(axis=1) + bias[None, :])
    acc = _wrap18(ob1 + ob2 + ob3)                     # (P, 8 oc)
    q = _quant_act_np(acc, cfg, lut)
    out[1:rows - 1, 1:cols - 1, :] = q.reshape(rows - 2, cols - 2, 8)
    return out.reshape(-1, 8)


def pool_grid_np(conv, rows, cols, zp_act):
    """2x2 stride-2 maxpool with the hardware's 1-cell zero border."""
    conv = np.asarray(conv, dtype=np.int64).reshape(rows, cols, 8)
    pr = ((rows - 2) >> 1) + 2
    pc = ((cols - 2) >> 1) + 2
    out = np.full((pr, pc, 8), int(zp_act), dtype=np.int64)
    for p in range(1, pr - 1):
        for q in range(1, pc - 1):
            out[p, q, :] = conv[2 * p - 1:2 * p + 1,
                                2 * q - 1:2 * q + 1, :].max(axis=(0, 1))
    return out.reshape(-1, 8)


def sent_rows(grid, rows, cols, send_sel, pooled):
    """Rows of the local grid that leave the chip (ofm_send_sel)."""
    pr = (((rows - 2) >> 1) + 2) if pooled else rows
    pc = (((cols - 2) >> 1) + 2) if pooled else cols
    r0, r1 = 0, pr
    if send_sel == 1:        # NO_HEAD
        r0 = 1
    elif send_sel == 2:      # NO_TAIL
        r1 = pr - 1
    elif send_sel == 3:      # REDUCE
        r0, r1 = 1, pr - 1
    return grid.reshape(pr, pc, 8)[r0:r1].reshape(-1, 8)


def layer_schedule(fm_row=258, conv_col=418):
    """Upstream chunking of one full layer (see accel.c Initiate/Set_Reg)."""
    fm_div = 4096 // conv_col - 2
    if fm_div % 2 != 0:
        fm_div -= 1
    fm_res = fm_row % fm_div
    if fm_res == 0:
        fm_div_cnt = fm_row // fm_div
        fm_res = fm_div
    else:
        fm_div_cnt = fm_row // fm_div + 1
        if fm_res < 6:
            fm_res = 6
    chunks = []
    for d in range(fm_div_cnt):
        if d == 0:
            off, sel = 0, 2
        elif d == fm_div_cnt - 1:
            off, sel = fm_row - fm_res - 2, 1
        else:
            off, sel = d * fm_div, 3
        chunks.append({"div": d, "row_off": off, "send_sel": sel,
                       "row": (fm_div + 2) if d != fm_div_cnt - 1
                              else (fm_res + 2)})
    return chunks


def build_m3(kind, outroot, root=UPSTREAM):
    """kind = m3a (conv only) or m3b (conv + 2x2/s2 pool)."""
    cfg, wgt, bias, lut = load_upstream(root)
    cfg["relu_type"] = 0
    cfg["sel"] = 5
    cfg["pool"] = 0 if kind == "m3a" else 1
    cols = CONV_COL[5]
    chunks = layer_schedule()
    wb = weight_beats(wgt, 2)
    wgt_words = gv.pack_beats(wb)
    bia_words = [int(b) & 0xFFFFFFFFFFFFFFFF for b in bias[:16]]
    lut_words = [int(x) for x in lut]

    fw_gold = []
    fw_beats = []
    fw_sel = []
    per_chunk = {}
    for ch in chunks:
        off = ch["row_off"]
        sel = ch["send_sel"] if cfg["pool"] else 0
        row = ch["row"]
        ifm = [[ifm_pattern(off + r, c) for c in range(cols)]
               for r in range(row)]
        g_gold = []
        for g in range(2):
            conv = golden_group_np(ifm, wgt[g], bias[8 * g:8 * g + 8],
                                   row, cols, cfg, lut)
            if cfg["pool"]:
                grid = pool_grid_np(conv, row, cols, cfg["zp_act"])
                sent = sent_rows(grid, row, cols, sel, True)
            else:
                sent = sent_rows(conv, row, cols, 0, False)
            g_gold.append(gv.pack_beats(sent.tolist()))
        fw_gold.append([gv.beat_checksum(g_gold[0]),
                        gv.beat_checksum(g_gold[1])])
        fw_beats.append(len(g_gold[0]))
        fw_sel.append(sel)
        per_chunk[ch["div"]] = (ifm, g_gold, row, off, sel)

    # firmware header with all per-chunk checksums
    first = chunks[0]
    hdr = ["// Auto-generated by tools/gen_real_layer.py -- do not edit.",
           "#ifndef YOLO_REAL_H", "#define YOLO_REAL_H", "#include <stdint.h>",
           "#define RL_ROW          %d" % first["row"],
           "#define RL_SEL          5",
           "#define RL_POOL         %d" % cfg["pool"],
           "#define RL_RELU_TYPE    0",
           "#define RL_SCALE        %d" % cfg["mult"],
           "#define RL_SHIFT        %d" % cfg["shift"],
           "#define RL_ZP_IN        %d" % cfg["zp_in"],
           "#define RL_ZP_OUT       %d" % cfg["zp_out"],
           "#define RL_ZP_ACT       %d" % cfg["zp_act"],
           "#define RL_CHUNKS       %d" % len(chunks),
           "#define RL_COLS         %d" % cols,
           "#define RL_IFM_BEATS    %d" % (first["row"] * cols),
           "#define RL_WGT_BEATS    %d" % len(wgt_words),
           "#define RL_BIA_BEATS    %d" % len(bia_words),
           "#define RL_LUT_BEATS    %d" % len(lut_words),
           ""]

    def carr(name, ctype, fmt, words, perline=4):
        hdr.append("static const %s %s[%d] = {" % (ctype, name, len(words)))
        for i in range(0, len(words), perline):
            hdr.append("    " + ", ".join(fmt % w for w in words[i:i + perline]) + ",")
        hdr.append("};")
        hdr.append("")

    carr("rl_lut", "uint64_t", "0x%016Xull", lut_words)
    carr("rl_bia", "uint64_t", "0x%016Xull", bia_words)
    carr("rl_wgt", "uint64_t", "0x%016Xull", wgt_words)
    carr("rl_row_off", "uint16_t", "%d", [c["row_off"] for c in chunks])
    carr("rl_send_sel", "uint8_t", "%d", fw_sel)
    carr("rl_beats", "uint32_t", "%du", fw_beats, perline=8)
    hdr.append("static const uint32_t rl_gold[%d][2] = {" % len(fw_gold))
    for pair in fw_gold:
        hdr.append("    {0x%08Xu, 0x%08Xu}," % tuple(pair))
    hdr.append("};")
    hdr.append("")
    hdr.append("#endif  // YOLO_REAL_H")
    with open(os.path.join(outroot, "yolo_real_%s.h" % kind), "w") as f:
        f.write("\n".join(hdr) + "\n")

    # per-chunk sim dirs: c0, c20, c42 (+ c0_full pooled probe)
    picks = [0, 20, 42]
    for d in picks:
        ifm, g_gold, row, off, sel = per_chunk[d]
        sels = [sel]
        names = ["c%d" % d]
        if kind == "m3b" and d == 0:
            sels.append(0)
            names.append("c0_full")
        for name, s_sel in zip(names, sels):
            odir = os.path.join(outroot, kind, name)
            os.makedirs(odir, exist_ok=True)
            ifm_words = gv.pack_beats([[ifm[r][c][ch] for ch in range(8)]
                                       for r in range(row) for c in range(cols)])
            gv.write_hex(os.path.join(odir, "ifm.hex"), ifm_words)
            gv.write_hex(os.path.join(odir, "wgt.hex"), wgt_words)
            gv.write_hex(os.path.join(odir, "bia.hex"), bia_words)
            gv.write_hex(os.path.join(odir, "lky.hex"), lut_words)
            if s_sel == sel:
                g0, g1 = g_gold
                srow = row
            else:
                conv = None
                bg = []
                for g in range(2):
                    cv = golden_group_np(ifm, wgt[g], bias[8 * g:8 * g + 8],
                                         row, cols, cfg, lut)
                    grid = pool_grid_np(cv, row, cols, cfg["zp_act"])
                    bg.append(gv.pack_beats(grid.tolist()))
                g0, g1 = bg
                srow = row
            gv.write_hex(os.path.join(odir, "gold_g0.hex"), g0)
            gv.write_hex(os.path.join(odir, "gold_g1.hex"), g1)
            txt = """// Auto-generated chunk sim defines
`ifndef REAL_DEFS_VH
`define REAL_DEFS_VH
`define R_ROW          %d
`define R_COLS         %d
`define R_SEL          5
`define R_POOL         %d
`define R_RELU_TYPE    0
`define R_SCALE        %d
`define R_SHIFT        %d
`define R_ZP_IN        %d
`define R_ZP_OUT       %d
`define R_ZP_ACT       %d
`define R_IFM_BEATS    %d
`define R_WGT_BEATS    %d
`define R_BIA_BEATS    %d
`define R_LUT_BEATS    %d
`define R_SEND_SEL     %d
`define R_GOLD_N0      %d
`define R_GOLD_N1      %d
`define R_OG_N         2
`endif
""" % (row, cols, cfg["pool"], cfg["mult"], cfg["shift"], cfg["zp_in"],
       cfg["zp_out"], cfg["zp_act"], row * cols, len(wgt_words),
       len(bia_words), len(lut_words), s_sel, len(g0), len(g1))
            with open(os.path.join(odir, "real_defs.vh"), "w") as f:
                f.write(txt)
            print("  %s/%s: chunk %d off=%d send_sel=%d pool=%d gold=%d/%d"
                  % (kind, name, d, off, s_sel, cfg["pool"], len(g0), len(g1)))
    print("%s -> %s (43 chunks, fw header yolo_real_%s.h)"
          % (kind, outroot, kind))


def write_defs(path, cfg, rows, cols, og_n, ifm_beats, wgt_beats_n,
               bia_beats, lut_beats, per_og, total_gold):
    txt = """// Auto-generated by tools/gen_real_layer.py -- do not edit.
`ifndef REAL_DEFS_VH
`define REAL_DEFS_VH
`define R_ROW          %d
`define R_COLS         %d
`define R_SEL          %d
`define R_POOL         %d
`define R_RELU_TYPE    %d
`define R_SCALE        %d
`define R_SHIFT        %d
`define R_ZP_IN        %d
`define R_ZP_OUT       %d
`define R_ZP_ACT       %d
`define R_IFM_BEATS    %d
`define R_WGT_BEATS    %d
`define R_BIA_BEATS    %d
`define R_LUT_BEATS    %d
`define R_OG_N         %d
`define R_GOLD_PER_OG  %d
`define R_GOLD_BEATS   %d
`endif
""" % (rows, cols, cfg["sel"], cfg["pool"], cfg["relu_type"],
       cfg["mult"], cfg["shift"], cfg["zp_in"], cfg["zp_out"], cfg["zp_act"],
       ifm_beats, wgt_beats_n, bia_beats, lut_beats, og_n, per_og, total_gold)
    with open(path, "w") as f:
        f.write(txt)


def build(case, outdir, root=UPSTREAM):
    cfg, wgt, bias, lut = load_upstream(root)
    cfg["relu_type"] = 0          # layer 0 uses the leaky LUT
    if case == "m1":
        cfg["sel"] = 0
        cfg["pool"] = 0
        rows, cols, og_n = 6, CONV_COL[0], 2
    elif case == "m2":
        cfg["sel"] = 5
        cfg["pool"] = 0
        rows, cols, og_n = 8, CONV_COL[5], 1
    else:
        raise SystemExit("unknown case: " + case)

    cols = CONV_COL[cfg["sel"]]
    ifm = [[ifm_pattern(r, c) for c in range(cols)] for r in range(rows)]
    ifm_words = gv.pack_beats([[ifm[r][c][ch] for ch in range(8)]
                               for r in range(rows) for c in range(cols)])
    wb = weight_beats(wgt, og_n)
    wgt_words = gv.pack_beats(wb)
    bia_words = [int(b) & 0xFFFFFFFFFFFFFFFF for b in bias[:8 * og_n]]
    lut_words = [int(x) for x in lut]

    gold_words = []
    for g in range(og_n):
        gold = golden_group(ifm, wgt[g], bias[8 * g:8 * g + 8], rows, cols,
                            cfg, lut)
        gsw = gv.pack_beats(gold)
        gnw = gv.pack_beats(golden_group_np(ifm, wgt[g],
                                           bias[8 * g:8 * g + 8],
                                           rows, cols, cfg, lut).tolist())
        if gsw != gnw:
            raise SystemExit("numpy golden mismatch (case %s group %d)"
                             % (case, g))
        gold_words += gsw

    os.makedirs(outdir, exist_ok=True)
    gv.write_hex(os.path.join(outdir, "ifm.hex"), ifm_words)
    gv.write_hex(os.path.join(outdir, "wgt.hex"), wgt_words)
    gv.write_hex(os.path.join(outdir, "bia.hex"), bia_words)
    gv.write_hex(os.path.join(outdir, "lky.hex"), lut_words)
    gv.write_hex(os.path.join(outdir, "ofm_gold.hex"), gold_words)
    per_og = rows * cols
    write_defs(os.path.join(outdir, "real_defs.vh"), cfg, rows, cols, og_n,
               len(ifm_words), len(wgt_words), len(bia_words), len(lut_words),
               per_og, len(gold_words))

    meta = {
        "case": case, "rows": rows, "cols": cols, "sel": cfg["sel"],
        "pool": cfg["pool"], "relu_type": cfg["relu_type"],
        "mult": cfg["mult"], "shift": cfg["shift"],
        "zp_in": cfg["zp_in"], "zp_out": cfg["zp_out"],
        "zp_act": cfg["zp_act"], "oc_groups": og_n,
        "ifm_beats": len(ifm_words), "wgt_beats": len(wgt_words),
        "bia_beats": len(bia_words), "lut_beats": len(lut_words),
        "gold_beats": len(gold_words), "gold_per_og": per_og,
        "gold_checksum": "0x%08X" % gv.beat_checksum(gold_words),
        "note": ("M1/M2 are unit-level cases; M3 firmware uses 416x256 "
                 "chunking with pooling."),
    }
    with open(os.path.join(outdir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    print("%-3s -> %s  (IFM %d, WGT %d, BIA %d, LUT %d, GOLD %d beats)"
          % (case, outdir, len(ifm_words), len(wgt_words), len(bia_words),
             len(lut_words), len(gold_words)))


if __name__ == "__main__":
    c = sys.argv[1] if len(sys.argv) > 1 else "m1"
    if c in ("m3a", "m3b"):
        out = sys.argv[2] if len(sys.argv) > 2 else os.path.join("sim", "vec_real")
        build_m3(c, out)
    else:
        out = (sys.argv[2] if len(sys.argv) > 2
               else os.path.join("sim", "vec_real", c))
        build(c, out)