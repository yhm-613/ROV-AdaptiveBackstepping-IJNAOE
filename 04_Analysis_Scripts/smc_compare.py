# smc_compare.py - compare SMC R2023b reproduction vs packaged RE data & metrics
import os, json
import numpy as np
import h5py

REPRO = r"C:/Users/yhm/wb_matrun/repro_out"
PKG   = r"D:/yanjiusheng/yhm论文/自适应反步/CEP投稿上传包_20261013/05_Supplementary_Data/01_仿真结果_补强/results_new_补强"

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

results = []
def add(case, metric, rep, ref, tol=0.005):
    if ref == 0:
        rd = abs(rep - ref)
        v = "PASS" if rd < 1e-3 else "DIFF"
    else:
        rd = abs(rep - ref)/abs(ref)
        v = "PASS" if rd < tol else ("CLOSE" if rd < 0.02 else "DIFF")
    results.append((case, metric, rep, ref, rd, v))

print("== step cases (vs packaged SMC_step_*.mat) ==")
for tg in ["over60", "over40", "over20", "nom", "under20", "under40", "under60"]:
    p = os.path.join(REPRO, f"SMC_step_{tg}.mat")
    q = os.path.join(PKG, f"SMC_step_{tg}.mat")
    if not os.path.exists(p):
        print(f"  MISSING repro {tg}"); continue
    t, eo = load(p)
    tp, eop = load(q)
    for i, ch in enumerate("xyz"):
        r = step_m(t, eo[:, i]); o = step_m(tp, eop[:, i])
        add(f"SMC_step_{tg}", f"{ch}_rise", r[0], o[0])
        add(f"SMC_step_{tg}", f"{ch}_settle", r[1], o[1])
        add(f"SMC_step_{tg}", f"{ch}_over", r[2], o[2])
        add(f"SMC_step_{tg}", f"{ch}_ssrms", r[3], o[3])

print("== circle cases (vs packaged SMC_circle_*.mat) ==")
for tg in ["nom", "over60", "const", "slow", "noise", "sat"]:
    p = os.path.join(REPRO, f"SMC_circle_{tg}.mat")
    q = os.path.join(PKG, f"SMC_circle_{tg}.mat")
    if not os.path.exists(p):
        print(f"  MISSING repro {tg}"); continue
    t, eo = load(p)
    r = circle_rms(t, eo, ss=(tg in ("noise", "sat")))
    tp, eop = load(q)
    o = circle_rms(tp, eop, ss=(tg in ("noise", "sat")))
    for k in ["xy", "z", "tot"]:
        add(f"SMC_circle_{tg}", k, r[k], o[k])
    if tg in ("noise", "sat"):
        add(f"SMC_circle_{tg}", "ss_xy", r["ss_xy"], o["ss_xy"])
    if tg == "nom":
        add(f"SMC_circle_{tg}", "tot_vs_json", r["tot"],
            json.load(open(os.path.join(PKG, "smc_metrics.json"), encoding="utf-8"))["SMC_circle"]["total"])

# secondary MaxStep exploratory runs (no packaged reference; report only)
print("\n== MaxStep 1e-3 exploratory (current, no verdict) ==")
for tg in ["const", "slow"]:
    p = os.path.join(REPRO, f"SMC_circle_{tg}_ms1e3.mat")
    if os.path.exists(p):
        t, eo = load(p)
        r = circle_rms(t, eo)
        print(f"  SMC_circle_{tg} @1e-3: tot={r['tot']:.4f}  (5e-4 primary compared above)")

print("\n== growth vs SMC nominal (paper-derivable quantities) ==")
tn, eon = load(os.path.join(REPRO, "SMC_circle_nom.mat"))
nom = circle_rms(tn, eon)["tot"]
for tg in ["const", "slow", "noise", "sat"]:
    p = os.path.join(REPRO, f"SMC_circle_{tg}.mat")
    if os.path.exists(p):
        t, eo = load(p)
        r = circle_rms(t, eo, ss=(tg in ("noise","sat")))
        g = (r["tot"]/nom - 1)*100
        ss = r.get("ss_xy", float("nan"))
        print(f"  SMC {tg}: tot={r['tot']:.4f} growth={g:+.1f}%  ss_xy={ss:.4f}")

print("\n" + "="*100)
print(f"{'case':<22}{'metric':<12}{'repro':>12}{'ref(pkg)':>12}{'rel.diff':>10}  verdict")
print("-"*100)
npass = sum(1 for r in results if r[5] == "PASS")
nclose = sum(1 for r in results if r[5] == "CLOSE")
ndiff = sum(1 for r in results if r[5] == "DIFF")
for case, metric, rep, ref, rd, v in results:
    flag = "" if v == "PASS" else "   <-- check"
    print(f"{case:<22}{metric:<12}{rep:>12.4f}{ref:>12.4f}{rd:>9.2%}  {v}{flag}")
print("-"*100)
print(f"TOTAL {len(results)}  PASS={npass}  CLOSE={nclose}  DIFF={ndiff}")
