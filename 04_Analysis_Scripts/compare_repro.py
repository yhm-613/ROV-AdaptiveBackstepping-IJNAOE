# compare_repro.py - compare local reproduction results vs packaged data & paper values
import os, json
import numpy as np
import h5py

REPRO = r"C:/Users/yhm/wb_matrun/repro_out"
PKG   = r"D:/yanjiusheng/yhm论文/自适应反步/CEP投稿上传包_20261013/05_Supplementary_Data/01_仿真结果_补强/results_new_补强"
M1    = r"D:/yanjiusheng/yhm论文/自适应反步/05_仿真数据/results_m1v5_失配实验"

def load(p):
    with h5py.File(p, "r") as f:
        t = np.array(f["t"]).ravel()
        eo = np.array(f["eout"])
    if eo.shape[0] == 6 and eo.shape[1] != 6:
        eo = eo.T
    return t, eo

def circle_rms(t, eo, ss=False):
    ex = eo[:, 0] - 2*np.cos(0.2*t)
    ey = eo[:, 1] - 2*np.sin(0.2*t)
    ez = eo[:, 2] - 2.0
    xy = float(np.sqrt(np.mean(ex**2 + ey**2)))
    z  = float(np.sqrt(np.mean(ez**2)))
    tot = float(np.sqrt(np.mean(ex**2 + ey**2 + ez**2)))
    out = dict(xy=xy, z=z, tot=tot)
    if ss:
        m = t >= 80
        out["ss_xy"] = float(np.sqrt(np.mean(ex[m]**2 + ey[m]**2)))
    return out

def step_m(t, y):
    REF = 2.0
    yf = y[-1]
    i90 = int(np.argmax(y >= 0.9*yf)); i10 = int(np.argmax(y >= 0.1*yf))
    rise = float(t[i90] - t[i10])
    out = np.where(np.abs(y - REF) > 0.02*REF)[0]
    settle = float(t[out[-1]]) if len(out) else 0.0
    over = float((y.max() - REF)/REF*100.0)
    m = t >= 80
    ss = float(np.sqrt(np.mean((REF - y[m])**2)))
    return rise, settle, over, ss

results = []   # (case, metric, repro, ref, rel_diff, verdict)

def add(case, metric, rep, ref, tol=0.005):
    if ref == 0:
        rd = abs(rep - ref)
        v = "PASS" if rd < 1e-3 else "DIFF"
    else:
        rd = abs(rep - ref)/abs(ref)
        v = "PASS" if rd < tol else ("CLOSE" if rd < 0.02 else "DIFF")
    results.append((case, metric, rep, ref, rd, v))

# ---------- 1. nominal vs packaged RE ----------
print("== nominal (vs packaged RE_*) ==")
for c in ["PID", "BS", "ADP"]:
    # circle
    p = os.path.join(REPRO, f"{c}_circle.mat")
    if os.path.exists(p):
        t, eo = load(p)
        r = circle_rms(t, eo)
        tp, eop = load(os.path.join(PKG, f"RE_{c}_circle.mat"))
        o = circle_rms(tp, eop)
        for k in ["xy", "z", "tot"]:
            add(f"{c}_circle", k, r[k], o[k])
        if min(len(t), len(tp)) > 1000:
            n = min(len(t), len(tp))
            dw = np.max(np.abs(eo[:n, :3] - eop[:n, :3]))
            print(f"  {c}_circle: pointwise max|d eta| (first {n}) = {dw:.3e}")
    else:
        print(f"  MISSING {p}")
    # step
    p = os.path.join(REPRO, f"{c}_step.mat")
    if os.path.exists(p):
        t, eo = load(p)
        tp, eop = load(os.path.join(PKG, f"RE_{c}_step.mat"))
        for i, ch in enumerate("xyz"):
            r = step_m(t, eo[:, i]); o = step_m(tp, eop[:, i])
            add(f"{c}_step", f"{ch}_rise", r[0], o[0]); add(f"{c}_step", f"{ch}_settle", r[1], o[1])
            add(f"{c}_step", f"{ch}_over", r[2], o[2]); add(f"{c}_step", f"{ch}_ssrms", r[3], o[3])

# ---------- 2. mismatch vs results_m1v5 ----------
print("== mismatch (vs results_m1v5) ==")
TAGS = ["under20", "under40", "under60", "over20", "over40", "over60"]
for c in ["BS", "ADP", "PID"]:
    for tg in TAGS:
        p = os.path.join(REPRO, f"{c}_step_{tg}.mat")
        pref = os.path.join(M1, f"{c}_step_{tg}.mat")
        if os.path.exists(p) and os.path.exists(pref):
            t, eo = load(p); tp, eop = load(pref)
            for i, ch in enumerate("xyz"):
                add(f"{c}_step_{tg}", f"{ch}_ssrms", step_m(t, eo[:, i])[3], step_m(tp, eop[:, i])[3])
        elif os.path.exists(p):
            print(f"  {c}_step_{tg}: repro OK, reference m1v5 missing (known divergence)")
        # circle mismatch
        p = os.path.join(REPRO, f"{c}_circle_{tg}.mat")
        pref = os.path.join(M1, f"{c}_circle_{tg}.mat")
        if os.path.exists(p) and os.path.exists(pref):
            t, eo = load(p); tp, eop = load(pref)
            add(f"{c}_circle_{tg}", "tot", circle_rms(t, eo)["tot"], circle_rms(tp, eop)["tot"])
        elif os.path.exists(p):
            print(f"  {c}_circle_{tg}: repro OK (completed), reference m1v5 missing (known divergence)")

# ---------- 3. current vs packaged RE ----------
print("== current (vs packaged RE_*) ==")
nom_tot = {}
for c in ["PID", "BS", "ADP"]:
    p = os.path.join(PKG, f"RE_{c}_circle.mat")
    t, eo = load(p)
    nom_tot[c] = circle_rms(t, eo)["tot"]
for c in ["PID", "BS", "ADP"]:
    for sc in ["const", "slow"]:
        p = os.path.join(REPRO, f"{c}_circle_{sc}.mat")
        if not os.path.exists(p):
            print(f"  MISSING {p}"); continue
        t, eo = load(p)
        r = circle_rms(t, eo)
        tp, eop = load(os.path.join(PKG, f"RE_{c}_circle_{sc}.mat"))
        o = circle_rms(tp, eop)
        add(f"{c}_circle_{sc}", "tot", r["tot"], o["tot"])
        growth = (r["tot"]/nom_tot[c] - 1)*100
        print(f"  {c} {sc}: total={r['tot']:.4f}  growth vs nominal = {growth:+.1f}%")

# ---------- 4. noise / saturation vs paper Table (tab:noise-sat) ----------
print("== noise / saturation (vs main.tex Table) ==")
PAPER = {
    ("PID", "noise"): (2.6588, 2.8675), ("PID", "sat"): (2.6891, 2.8984),
    ("BS",  "noise"): (0.7086, 0.0239), ("BS",  "sat"): (0.8597, 0.0112),
    ("ADP", "noise"): (0.8285, 0.1877), ("ADP", "sat"): (0.9369, 0.4536),
}
for (c, v), (ref_tot, ref_ss) in PAPER.items():
    p = os.path.join(REPRO, f"{c}_circle_{v}.mat")
    if not os.path.exists(p):
        print(f"  MISSING {p}"); continue
    t, eo = load(p)
    r = circle_rms(t, eo, ss=True)
    add(f"{c}_circle_{v}", "tot", r["tot"], ref_tot)
    add(f"{c}_circle_{v}", "ss_xy", r["ss_xy"], ref_ss)

# ---------- 5. SMC packaged data self-consistency (data-level only) ----------
print("== SMC packaged data self-check (no local rerun: R2024b model) ==")
try:
    sm = json.load(open(os.path.join(PKG, "smc_metrics.json"), encoding="utf-8"))
    for f_, key in [("SMC_circle_nom.mat", "SMC_circle"), ("SMC_step_nom.mat", None)]:
        p = os.path.join(PKG, f_)
        if not os.path.exists(p):
            continue
        t, eo = load(p)
        if "circle" in f_:
            r = circle_rms(t, eo)
            print(f"  {f_}: recomputed total={r['tot']:.4f}  json={sm[key]['total']:.4f}")
        else:
            for i, ch in enumerate("xyz"):
                rr = step_m(t, eo[:, i])
                js = sm[f"SMC_step_{ch}"]
                print(f"  {f_} {ch}: ssrms={rr[3]:.4f} (json {js['ssrms']:.4f}) rise={rr[0]:.2f} (json {js['rise']:.2f})")
except Exception as e:
    print("  SMC check error:", e)

# ---------- summary ----------
print("\n" + "="*100)
print(f"{'case':<24}{'metric':<10}{'repro':>12}{'ref':>12}{'rel.diff':>10}  verdict")
print("-"*100)
npass = sum(1 for r in results if r[5] == "PASS")
nclose = sum(1 for r in results if r[5] == "CLOSE")
ndiff = sum(1 for r in results if r[5] == "DIFF")
for case, metric, rep, ref, rd, v in results:
    flag = "" if v == "PASS" else "   <-- check"
    print(f"{case:<24}{metric:<10}{rep:>12.4f}{ref:>12.4f}{rd:>9.2%}  {v}{flag}")
print("-"*100)
print(f"TOTAL {len(results)}  PASS={npass}  CLOSE={nclose}  DIFF={ndiff}")
