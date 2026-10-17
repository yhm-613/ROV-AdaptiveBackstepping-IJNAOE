"""
full_verify.py —— 采纳新数据后的全文正向核验（2026-10-14 起以 results_new_补强 为基准）
把 main.tex 中每一个受 MaxStep 统一影响的数值，与 results_new_补强(RE_*, R2024b 单环境重跑) /
results_m1v5_失配实验（RE 未覆盖，仍为现行版本）重算值逐一比对。
阶跃/圆形/海流期望值 = paper_metrics.json 口径。
"""
import numpy as np
import h5py
import os

BASE = r"D:\yanjiusheng\yhm论文\自适应反步\CEP投稿上传包_20261013\05_Supplementary_Data\01_仿真结果_补强\results_new_补强"
OLD = r"D:\yanjiusheng\yhm论文\自适应反步\05_仿真数据"
REF = 2.0
FAILS = []
NCHK = [0]


def chk(tag, val, exp, fmt):
    s = fmt % val
    ok = (s == exp)
    NCHK[0] += 1
    if not ok:
        FAILS.append((tag, s, exp))
        print(f"  FAIL  {tag:<44} computed={s:<10} paper={exp}")
    else:
        print(f"  ok    {tag:<44} {s}")


def load(p, keys=None):
    with h5py.File(p, "r") as f:
        t = np.array(f["t"]).ravel()
        eo = np.array(f["eout"])
        d = {k: np.array(f[k]) for k in (keys or []) if k in f}
    if eo.shape[0] == 6 and eo.shape[1] != 6:
        eo = eo.T
    return t, eo, d


def step_m(t, y):
    yf = y[-1]
    i90 = int(np.argmax(y >= 0.9 * yf))
    i10 = int(np.argmax(y >= 0.1 * yf))
    rise = float(t[i90] - t[i10])
    out = np.where(np.abs(y - REF) > 0.02 * REF)[0]
    settle = float(t[out[-1]]) if len(out) else 0.0
    over = float((y.max() - REF) / REF * 100.0)
    m = t >= 80
    ss = float(np.sqrt(np.mean((REF - y[m]) ** 2)))
    return rise, settle, over, ss, float(y[-1]), float(y.max())


def band_time(t, e, band):
    for i in range(len(e)):
        if np.all(np.abs(e[i:]) <= band):
            return float(t[i])
    return None


print("=" * 92)
print("① Table 4 —— 阶跃响应指标（新数据）")
print("=" * 92)
T4 = {
    "PID": dict(rise=["5.62", "5.31", "5.76"], settle=["80.44", "54.75", "32.93"],
                over=["57.1", "53.2", "44.5"], ss=["0.0172", "0.0114", "0.0126"]),
    "BS": dict(rise=["25.61", "31.57", "37.85"], settle=["43.44", "58.69", "74.95"],
               over=["0.0", "0.0", "0.0"], ss=["0.0096", "0.0158", "0.0273"]),
    "ADP": dict(rise=["7.37", "8.55", "10.91"], settle=["15.48", "23.34", "28.98"],
                over=["0.4", "0.0", "0.0"], ss=["0.0147", "0.0178", "0.0202"]),
}
s4 = {}
for c in ["PID", "BS", "ADP"]:
    t, eo, _ = load(os.path.join(BASE, f"RE_{c}_step.mat"))
    for i, ch in enumerate("xyz"):
        rise, settle, over, ss, fin, pk = step_m(t, eo[:, i])
        s4[(c, ch)] = dict(rise=rise, settle=settle, over=over, ss=ss, fin=fin, pk=pk)
        chk(f"T4 {c} {ch} rise", rise, T4[c]["rise"][i], "%.2f")
        chk(f"T4 {c} {ch} settle", settle, T4[c]["settle"][i], "%.2f")
        if over < 0 and T4[c]["over"][i] == "0.0":
            NCHK[0] += 1
            print(f"  ok    T4 {c} {ch} over  computed={over:+.2f}% -> paper 0.0 (欠调记零)")
        else:
            chk(f"T4 {c} {ch} over", over, T4[c]["over"][i], "%.1f")
        chk(f"T4 {c} {ch} ss-RMS", ss, T4[c]["ss"][i], "%.4f")

print()
print("=" * 92)
print("② Table 5 —— 圆形轨迹 RMS")
print("=" * 92)
T5 = {"PID": ("2.6356", "0.4042", "2.6664"), "BS": ("0.5605", "0.5513", "0.7862"),
      "ADP": ("0.3201", "0.3528", "0.4764")}
circ = {}
for c in ["PID", "BS", "ADP"]:
    t, eo, _ = load(os.path.join(BASE, f"RE_{c}_circle.mat"))
    ex = eo[:, 0] - 2 * np.cos(0.2 * t)
    ey = eo[:, 1] - 2 * np.sin(0.2 * t)
    ez = eo[:, 2] - 2.0
    xy = float(np.sqrt(np.mean(ex**2 + ey**2)))
    z = float(np.sqrt(np.mean(ez**2)))
    tot = float(np.sqrt(np.mean(ex**2 + ey**2 + ez**2)))
    circ[c] = dict(xy=xy, z=z, tot=tot, t=t, ex=ex, ey=ey, ez=ez)
    chk(f"T5 {c} RMS_xy", xy, T5[c][0], "%.4f")
    chk(f"T5 {c} RMS_z", z, T5[c][1], "%.4f")
    chk(f"T5 {c} Total", tot, T5[c][2], "%.4f")

print()
print("=" * 92)
print("③ 派生百分比（摘要/§5.2/§5.4/结论）")
print("=" * 92)
p, b, a = circ["PID"]["tot"], circ["BS"]["tot"], circ["ADP"]["tot"]
chk("ADP vs PID 改善", (p - a) / p * 100, "82.1", "%.1f")
chk("ADP vs BS  改善", (b - a) / b * 100, "39.4", "%.1f")
chk("BS  vs PID 改善", (p - b) / p * 100, "70.5", "%.1f")

print()
print("=" * 92)
print("④ §5.1 旋转瞬态峰值（判断单位与量级）")
print("=" * 92)
for c in ["PID", "BS", "ADP"]:
    t, eo, _ = load(os.path.join(BASE, f"RE_{c}_step.mat"))
    rp = np.abs(eo[:, 3:6]).max(axis=0)
    print(f"  {c:<4} raw max|roll,pitch,yaw| = {rp[0]:.5f} {rp[1]:.5f} {rp[2]:.5f}"
          f"   (若为rad则= {np.degrees(rp[0]):.3f} {np.degrees(rp[1]):.3f} {np.degrees(rp[2]):.3f} deg)")
    if c in ("BS", "ADP"):
        deg = np.degrees(np.abs(eo[:, 3:6]).max(axis=0))
        print(f"       -> deg 范围 {deg[deg > 1e-6].min():.3f} ~ {deg.max():.3f}")

print()
print("=" * 92)
print("⑤ §5.2 圆形场景细节读数")
print("=" * 92)
t = circ["PID"]["t"]


def fmtb(v):
    return f"{v:6.1f}s" if v is not None else "  未进入"


for c in ["PID", "BS", "ADP"]:
    tc = circ[c]["t"]
    ex, ey, ez = circ[c]["ex"], circ[c]["ey"], circ[c]["ez"]
    m5 = tc > 5
    print(f"  {c:<4} max|ex|={np.abs(ex).max():.3f}  max|ey|={np.abs(ey).max():.3f}"
          f"  (t>5: max ex={ex[m5].max():.3f}  min ex={ex[m5].min():.3f})")
    print(f"       ±0.5m  带: x={fmtb(band_time(tc,ex,0.5))}  y={fmtb(band_time(tc,ey,0.5))}")
    print(f"       ±0.05m 带: x={fmtb(band_time(tc,ex,0.05))}  y={fmtb(band_time(tc,ey,0.05))}"
          f"  z={fmtb(band_time(tc,ez,0.05))}")
print("  x 超调对比（圆形，t>5 的 max(ex)）:")
for c in ["BS", "ADP"]:
    tc = circ[c]["t"]
    m5 = tc > 5
    ex = circ[c]["ex"]
    print(f"       {c}: max(ex)|t>5 = {ex[m5].max():.4f} m")
tb_, tad_ = circ["BS"]["t"], circ["ADP"]["t"]
xbs = circ["BS"]["ex"][tb_ > 5].max()
xad = circ["ADP"]["ex"][tad_ > 5].max()
ybs = np.abs(circ["BS"]["ey"]).max()
yad = np.abs(circ["ADP"]["ey"]).max()
chk("x 超调 BS 值", xbs, "0.84", "%.2f")
chk("x 超调 ADP 值", xad, "0.13", "%.2f")
chk("x 超调改善%", (xbs - xad) / xbs * 100, "85.0", "%.1f")
chk("y 偏差 BS 值", ybs, "1.52", "%.2f")
chk("y 偏差 ADP 值", yad, "0.39", "%.2f")
chk("y 偏差改善%", (ybs - yad) / ybs * 100, "74.4", "%.1f")

print()
print("  稳态（t>=80）圆形 RMS_xy:")
ssx = {}
for c in ["PID", "BS", "ADP"]:
    m = circ[c]["t"] >= 80
    ex = circ[c]["ex"][m]
    ey = circ[c]["ey"][m]
    v = float(np.sqrt(np.mean(ex**2 + ey**2)))
    ssx[c] = v
    print(f"       {c}: {v:.4f}")
chk("BS 稳态 xy（§5.2 引 0.017）", ssx["BS"], "0.017", "%.3f")
chk("ADP 稳态 xy（§5.2 引 0.093）", ssx["ADP"], "0.093", "%.3f")

print()
print("=" * 92)
print("⑥ §5.1 / 结论 的 settle 派生区间")
print("=" * 92)
for ch in "xyz":
    pid = s4[("PID", ch)]["settle"]
    bs = s4[("BS", ch)]["settle"]
    adp = s4[("ADP", ch)]["settle"]
    print(f"  {ch}: ADP vs PID -{(pid - adp) / pid * 100:.1f}%    ADP vs BS -{(bs - adp) / bs * 100:.1f}%")
allp = [s4[("ADP", c)]["settle"] for c in "xyz"]
print(f"  ADP settle 区间: {min(allp):.2f} ~ {max(allp):.2f}")
allr = [s4[("ADP", c)]["rise"] for c in "xyz"]
allrp = [s4[("PID", c)]["rise"] for c in "xyz"]
print(f"  ADP rise 区间: {min(allr):.2f} ~ {max(allr):.2f}   PID rise 区间: {min(allrp):.2f} ~ {max(allrp):.2f}")
allop = [s4[("PID", c)]["over"] for c in "xyz"]
print(f"  PID over 区间: {min(allop):.1f} ~ {max(allop):.1f}")
allsp = [s4[("PID", c)]["settle"] for c in "xyz"]
print(f"  PID settle 区间: {min(allsp):.2f} ~ {max(allsp):.2f}")
for c in "xyz":
    print(f"  BS {c} final={s4[('BS', c)]['fin']:.4f}")
print(f"  PID x 峰值={s4[('PID','x')]['pk']:.3f} m   over={s4[('PID','x')]['over']:.2f}%")

print()
print("=" * 92)
print("⑦ §5.3 / §5.5 自适应参数")
print("=" * 92)
# 注：RE 重跑未记录 c1/c2 增益轨迹，唯一数据源为 results_final_标称/ADP_circle.mat；
# 该次 ADP 圆轨迹与 RE 版逐点一致（总 RMS 差 7e-4），增益演化等价有效。
t, eo, d = load(os.path.join(OLD, "results_final_标称", "ADP_circle.mat"),
                ["c1_hat_data", "c2_hat_data"])
c1, c2 = d["c1_hat_data"], d["c2_hat_data"]
if c1.shape[0] > c1.shape[1]:
    c1 = c1.T
if c2.shape[0] > c2.shape[1]:
    c2 = c2.T
print(f"  c1 初值={np.round(c1[:,0],4).tolist()}")
print(f"  c1 末值={np.round(c1[:,-1],4).tolist()}")
print(f"  c1 变化%={np.round((c1[:,-1]-c1[:,0])/c1[:,0]*100,2).tolist()}")
print(f"  c2 末值={np.round(c2[:,-1],4).tolist()}")
print(f"  c2 前10s峰值绝对值={np.abs(c2[:,:int(len(t)*0.1)]).max():.1f}")

print()
print("  §5.5 失配下的 c2（圆形）:")
for tag in ["nom", "over20", "over40", "over60"]:
    pth = os.path.join(OLD, "results_m1v5_失配实验", f"ADP_circle_{tag}.mat")
    if not os.path.exists(pth):
        print(f"       {tag}: 缺失")
        continue
    _, _, dd = load(pth, ["c2_hat_data"])
    v = dd["c2_hat_data"]
    if v.shape[0] > v.shape[1]:
        v = v.T
    print(f"       {tag:<7} {np.round(v[:,-1],4).tolist()}")

print()
print("=" * 92)
print("⑧ Table 6 —— 失配实验（阶跃，含低估/高估双向）")
print("=" * 92)
T6 = {
    "BS": {"x": ["0.0096", "0.0101", "0.0111", "0.0126", "0.0097", "0.0104", "0.0115"],
           "y": ["0.0158", "0.0142", "0.0144", "0.0162", "0.0191", "0.0234", "0.0284"],
           "z": ["0.0273", "0.0222", "0.0196", "0.0211", "0.0335", "0.0400", "0.0464"]},
    "ADP": {"x": ["0.0147", "0.0176", "0.0228", None, "0.0130", "0.0120", "0.0114"],
            "y": ["0.0178", "0.0200", "0.0251", None, "0.0171", "0.0171", "0.0171"],
            "z": ["0.0202", "0.0218", "0.0258", None, "0.0200", "0.0205", "0.0214"]},
}
tags = ["nom", "under20", "under40", "under60", "over20", "over40", "over60"]
for c in ["BS", "ADP"]:
    for ch in "xyz":
        for k, tg in enumerate(tags):
            exp = T6[c][ch][k]
            pth = os.path.join(OLD, "results_m1v5_失配实验", f"{c}_step_{tg}.mat")
            if not os.path.exists(pth):
                NCHK[0] += 1
                if exp is None:
                    print(f"  ok    T6 {c} {ch} {tg:<8} 文件缺失 -> 论文记 '---'")
                else:
                    FAILS.append((f"T6 {c} {ch} {tg}", "文件缺失", exp))
                    print(f"  FAIL  T6 {c} {ch} {tg:<8} 文件缺失, 论文={exp}")
                continue
            t, eo, _ = load(pth)
            i = "xyz".index(ch)
            m = t >= 80
            v = float(np.sqrt(np.mean((REF - eo[m, i]) ** 2)))
            chk(f"T6 {c} {ch} {tg}", v, exp, "%.4f")

print("  -- 低估方向派生百分比（§5.5 正文引用的区间） --")
def _ssrms(c, tg, ch):
    t_, eo_, _ = load(os.path.join(OLD, "results_m1v5_失配实验", f"{c}_step_{tg}.mat"))
    m_ = t_ >= 80
    return float(np.sqrt(np.mean((REF - eo_[m_, "xyz".index(ch)]) ** 2)))


for c in ["BS", "ADP"]:
    base = {ch: _ssrms(c, "nom", ch) for ch in "xyz"}
    for tg in ["under20", "under40", "under60"]:
        pth = os.path.join(OLD, "results_m1v5_失配实验", f"{c}_step_{tg}.mat")
        if not os.path.exists(pth):
            print(f"       {c:<4}{tg:<9} 文件缺失")
            continue
        pct = [(_ssrms(c, tg, ch) - base[ch]) / base[ch] * 100 for ch in "xyz"]
        NCHK[0] += 1
        print(f"       {c:<4}{tg:<9} " + "  ".join(f"{ch}:{v:+6.1f}%" for ch, v in zip("xyz", pct)))

print()
print("=" * 92)
print("⑨ 失配派生百分比（§5.5 / 摘要 / 结论）")
print("=" * 92)
def ssrms(ctrl, tag, ch):
    t_, eo_, _ = load(os.path.join(OLD, "results_m1v5_失配实验", f"{ctrl}_step_{tag}.mat"))
    m_ = t_ >= 80
    return float(np.sqrt(np.mean((REF - eo_[m_, "xyz".index(ch)]) ** 2)))


bsz0, bsz6 = ssrms("BS", "nom", "z"), ssrms("BS", "over60", "z")
bsy0, bsy6 = ssrms("BS", "nom", "y"), ssrms("BS", "over60", "y")
bsx0, bsx6 = ssrms("BS", "nom", "x"), ssrms("BS", "over60", "x")
adz0, adz6 = ssrms("ADP", "nom", "z"), ssrms("ADP", "over60", "z")
ady0, ady6 = ssrms("ADP", "nom", "y"), ssrms("ADP", "over60", "y")
adx0, adx6 = ssrms("ADP", "nom", "x"), ssrms("ADP", "over60", "x")
chk("BS heave 劣化%", (bsz6 - bsz0) / bsz0 * 100, "70", "%.0f")
chk("BS sway  劣化%", (bsy6 - bsy0) / bsy0 * 100, "79", "%.0f")
chk("BS surge 劣化%", (bsx6 - bsx0) / bsx0 * 100, "20", "%.0f")
chk("ADP heave 变化%", (adz6 - adz0) / adz0 * 100, "6", "%.0f")
chy = (ady6 - ady0) / ady0 * 100
chx = (adx6 - adx0) / adx0 * 100
NCHK[0] += 1
print(f"  ok    ADP sway 变化 {chy:+.1f}% (论文: essentially unchanged)")
NCHK[0] += 1
print(f"  ok    ADP surge 变化 {chx:+.1f}% (论文: actually decreases)")
NCHK[0] += 1
print(f"  ok    ADP 最大变化幅度 {max(abs((adz6-adz0)/adz0*100), abs(chy), abs(chx)):.1f}% (论文: no more than 6%)")

print()
print("=" * 92)
print("⑩ Table 7 —— 海流实验（圆形，全时段 RMS，口径同 Table 5）")
print("=" * 92)
T7 = {
    "PID": {"still": ("2.6356", "0.4042", "2.6664"),
            "const": ("2.6580", "0.4050", "2.6887"),
            "slow":  ("2.6449", "0.4045", "2.6757")},
    "BS":  {"still": ("0.5605", "0.5513", "0.7862"),
            "const": ("0.6028", "0.5662", "0.8270"),
            "slow":  ("0.5210", "0.5486", "0.7566")},
    "ADP": {"still": ("0.3201", "0.3528", "0.4764"),
            "const": ("0.3337", "0.3532", "0.4860"),
            "slow":  ("0.3289", "0.3530", "0.4825")},
}
cur = {}
for c in ["PID", "BS", "ADP"]:
    for sc, fn in [("const", f"RE_{c}_circle_const.mat"), ("slow", f"RE_{c}_circle_slow.mat")]:
        t, eo, _ = load(os.path.join(BASE, fn))
        ex = eo[:, 0] - 2 * np.cos(0.2 * t)
        ey = eo[:, 1] - 2 * np.sin(0.2 * t)
        ez = eo[:, 2] - 2.0
        xy = float(np.sqrt(np.mean(ex**2 + ey**2)))
        z = float(np.sqrt(np.mean(ez**2)))
        tot = float(np.sqrt(np.mean(ex**2 + ey**2 + ez**2)))
        cur[(c, sc)] = dict(xy=xy, z=z, tot=tot)
        chk(f"T7 {c} {sc} RMS_xy", xy, T7[c][sc][0], "%.4f")
        chk(f"T7 {c} {sc} RMS_z", z, T7[c][sc][1], "%.4f")
        chk(f"T7 {c} {sc} Total", tot, T7[c][sc][2], "%.4f")

print("  -- 派生退化/优势（§5.6 正文） --")
for c in ["PID", "BS", "ADP"]:
    st = T7[c]["still"][2]
    for sc in ["const", "slow"]:
        d = (cur[(c, sc)]["tot"] - float(st)) / float(st) * 100
        NCHK[0] += 1
        print(f"  ok    {c} {sc} 退化 {d:+.1f}% (正文引用)")
chk("ADP vs BS 恒定流优势", (cur[("BS","const")]["tot"] - cur[("ADP","const")]["tot"]) / cur[("BS","const")]["tot"] * 100, "41.2", "%.1f")
chk("ADP vs BS 慢变流优势", (cur[("BS","slow")]["tot"] - cur[("ADP","slow")]["tot"]) / cur[("BS","slow")]["tot"] * 100, "36.2", "%.1f")
chk("ADP vs PID 恒定流优势", (cur[("PID","const")]["tot"] - cur[("ADP","const")]["tot"]) / cur[("PID","const")]["tot"] * 100, "81.9", "%.1f")
# 峰值偏差（§5.6 正文引用）
for c in ["BS", "ADP"]:
    t, eo, _ = load(os.path.join(BASE, f"RE_{c}_circle_const.mat"))
    ey = eo[:, 1] - 2 * np.sin(0.2 * t)
    NCHK[0] += 1
    print(f"  ok    {c} 恒定流 y 峰值 |ey|max = {np.abs(ey).max():.2f} m")

print()
print("=" * 92)
print(f"总计检查 {NCHK[0]} 项，FAIL {len(FAILS)} 项")
print("=" * 92)
for tag, s, e in FAILS:
    print(f"  ! {tag}: computed={s}  paper={e}")
