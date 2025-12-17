import csv
import matplotlib.pyplot as plt

csv_path = "mpc_log.csv"   # change if needed
Ts = 0.05                  # used only if t_rel is not present in the CSV

rows = []
with open(csv_path, newline="") as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

if not rows:
    raise SystemExit(f"No data in {csv_path}")

# Build time and signals (prefer t_rel if present; else use k*Ts)
use_trel = ("t_rel" in rows[0]) and (rows[0]["t_rel"] not in ("", None))
t, u = [], []
pos, th, dpos, dth = [], [], [], []

for r in rows:
    if use_trel:
        t.append(float(r["t_rel"]))
    else:
        k = int(float(r.get("k", 0)))
        t.append(k * Ts)
    u.append(float(r["u0"]))
    pos.append(float(r["x0"]))
    th.append(float(r["x1"]))
    dpos.append(float(r["x2"]))
    dth.append(float(r["x3"]))

# Figure 1: Control
plt.figure()
plt.step(t, u, where="post")
plt.title("Control signal")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid(True)

# Figure 2: States
plt.figure()
plt.plot(t, pos,  marker="x", label="pos")
plt.plot(t, th,   marker="x", label="theta")
plt.plot(t, dpos, marker="x", label="dpos")
plt.plot(t, dth,  marker="x", label="dtheta")
plt.title("States")
plt.xlabel("Time [s]")
plt.ylabel("States")
plt.legend()
plt.grid(True)

plt.show()