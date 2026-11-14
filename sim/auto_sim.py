"""2D simulation of Car1AutoV2Base.goTo() — tunes Car1 auto waypoints tonight.

Faithful port of the control law in TeamCode/.../Car1AutoV2Base.java:
same gains, caps, floors, tolerances, timeouts. Robot is a kinematic
body with first-order speed response (tune VFULL/TAU to the real robot).

Output: per-segment timing table + sim/auto_path.svg trajectory plot.
Run:  py sim/auto_sim.py
"""
import math

# ---- control law constants (verbatim from Car1AutoV2Base.java) ----
KP_XY = 0.004
MAX_P = 0.4
MIN_P = 0.08
KP_H = 0.9
TOL_MM = 25.0
TOL_DEG = 3.0
SEG_TIMEOUT_S = 6.0
AUTO_END_S = 29.0

# ---- drivetrain model (TUNE THESE to the real robot) ----
VFULL = 2000.0   # mm/s at power 1.0  (0.4 power -> 800 mm/s)
TAU = 0.30       # s, how fast speed responds to command

# ---- the path under test (from Car1AutoShoot.java) ----
WAYPOINTS = [(-300, 0, 0), (-560, -790, 0), (-2450, -790, 0), (-2480, -1130, 0)]

# ---- shooting-phase estimate (before driving; for the budget check) ----
SPINUP_S = 3.0           # worst case per the code
SHOTS = 4
GATE_MS = 350
SHOT_GAP_S = 1.5         # worst case per shot
SHOOT_TIME = SPINUP_S + SHOTS * (GATE_MS / 1000.0 + SHOT_GAP_S)

DT = 0.005

def clamp(v, lo, hi): return max(lo, min(hi, v))

def simulate_segment(x, y, tx, ty, tdeg, timeout):
    """Port of goTo(): returns (reached, t, end_err, path[])."""
    vx_body, vy_body = 0.0, 0.0      # current body velocity (fwd, right) mm/s
    path = [(x, y)]
    t = 0.0
    h = 0.0                            # heading stays ~0 for this path (all targets 0)
    while t < timeout:
        ex, ey = tx - x, ty - y
        err = math.hypot(ex, ey)
        if err < TOL_MM:
            return True, t, err, path
        # goTo: fwd/left projections (heading 0 => identity)
        fwd, left = ex, ey
        f, l = KP_XY * fwd, KP_XY * left
        m = math.hypot(f, l)
        if m > MAX_P:
            f *= MAX_P / m; l *= MAX_P / m
        elif m > 0.001 and m < MIN_P:
            f *= MIN_P / m; l *= MIN_P / m
        # commanded body velocity (drive() maps: forward=f, lateralRight=-l)
        cmd_f, cmd_r = f * VFULL, -l * VFULL
        # first-order response
        k = DT / TAU
        vx_body += (cmd_f - vx_body) * k
        vy_body += (cmd_r - vy_body) * k
        # integrate (heading 0: body fwd = +x field; body RIGHT = -y field,
        # because the Pinpoint frame is y-LEFT-positive — do not flip this!)
        x += vx_body * DT
        y += -vy_body * DT
        path.append((x, y))
        t += DT
    return False, t, math.hypot(tx - x, ty - y), path

def run():
    x, y = 0.0, 0.0
    total = SHOOT_TIME
    all_path = [(0.0, 0.0)]
    print(f"{'seg':>3} {'target (mm)':>18} {'dist':>6} {'time':>6} {'result':>9} {'end err':>8}")
    for i, (tx, ty, td) in enumerate(WAYPOINTS):
        reached, t, err, path = simulate_segment(x, y, tx, ty, td, SEG_TIMEOUT_S)
        dist = math.hypot(tx - x, ty - y)
        total += t
        print(f"{i+1:>3} ({tx:>7.0f},{ty:>7.0f}) {dist:>6.0f} {t:>6.2f} {('REACHED' if reached else 'TIMEOUT'):>9} {err:>7.1f}mm")
        x, y = tx, ty
        all_path += path[1:]
    print(f"\nshooting phase est : {SHOOT_TIME:.1f}s (worst case: spinup {SPINUP_S}s + {SHOTS} shots)")
    print(f"driving total       : {total - SHOOT_TIME:.1f}s")
    print(f"AUTO TOTAL est      : {total:.1f}s   budget {AUTO_END_S:.0f}s -> "
          + ("FITS with %.1fs spare" % (AUTO_END_S - total) if total < AUTO_END_S else "OVER BUDGET!"))
    return all_path

def write_svg(path, fname="sim/auto_path.svg"):
    # scale: 1px = 5mm; canvas covers the path + margin
    S = 1/5.0
    xs = [p[0] for p in path]; ys = [p[1] for p in path]
    x0, x1 = min(xs)-400, max(xs)+400
    y0, y1 = min(ys)-400, max(ys)+400
    W, H = (x1-x0)*S, (y1-y0)*S
    def px(p): return ((p[0]-x0)*S, (p[1]-y0)*S)
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" style="background:#fafafa">']
    # tile grid (609.6mm) for scale
    TILE = 609.6
    s.append('<g stroke="#ddd" stroke-width="1">')
    gx = math.floor(x0/TILE)*TILE
    while gx < x1:
        a,_=px((gx,0)); s.append(f'<line x1="{a:.0f}" y1="0" x2="{a:.0f}" y2="{H:.0f}"/>'); gx+=TILE
    gy = math.floor(y0/TILE)*TILE
    while gy < y1:
        _,b=px((0,gy)); s.append(f'<line x1="0" y1="{b:.0f}" x2="{W:.0f}" y2="{b:.0f}"/>'); gy+=TILE
    s.append('</g>')
    # trajectory
    pts = " ".join(f"{px(p)[0]:.1f},{px(p)[1]:.1f}" for p in path)
    s.append(f'<polyline points="{pts}" fill="none" stroke="#1a7a1a" stroke-width="2.5"/>')
    # waypoints + start
    for i,(tx,ty,_) in enumerate(WAYPOINTS):
        a,b = px((tx,ty))
        s.append(f'<circle cx="{a:.0f}" cy="{b:.0f}" r="6" fill="none" stroke="#a11a1a" stroke-width="2"/>')
        s.append(f'<text x="{a+10:.0f}" y="{b-8:.0f}" font-size="13" fill="#a11a1a">W{i+1} ({tx:.0f},{ty:.0f})</text>')
    a,b = px((0,0))
    s.append(f'<circle cx="{a:.0f}" cy="{b:.0f}" r="6" fill="#1a4fa1"/>')
    s.append(f'<text x="{a+10:.0f}" y="{b+18:.0f}" font-size="13" fill="#1a4fa1">START (0,0), nose +x</text>')
    s.append(f'<text x="8" y="18" font-size="13" fill="#555">Car1 AutoShoot path — grid = 609.6mm tiles</text>')
    s.append('</svg>')
    open(fname,"w").write("\n".join(s))
    print(f"plot -> {fname}")

if __name__ == "__main__":
    write_svg(run())
