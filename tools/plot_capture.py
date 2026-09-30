"""Turn a per-move capture from the gate controller into an annotated SVG.

At the end of every move the MSP430 dumps what it recorded (motor current and
supply voltage every 25 ms) between CAP_BEGIN and CAP_END lines on its console.
Save the console to a text file from the bridge's web UI ("Download log"), then:

    python tools/plot_capture.py --from-log console.txt --csv media/data/move.csv
    python tools/plot_capture.py --csv media/data/move.csv --out media/charts/move

The second command writes <out>-light.svg and <out>-dark.svg. The duty-cycle
panel is rebuilt from the motion parameters (defaults = firmware defaults), so
pass --soft-end-ms etc. if your gate is tuned differently.

No third-party packages needed.
"""
import argparse
import csv
import io
import re
import sys

THEMES = {
    "light": dict(surface="#fcfcfb", ink="#0b0b0b", ink2="#52514e", muted="#898781",
                  grid="#e1e0d9", axis="#c3c2b7", band="#0b0b0b", band_op="0.05",
                  pwm="#2a78d6", relay="#eb6834"),
    "dark": dict(surface="#1a1a19", ink="#ffffff", ink2="#c3c2b7", muted="#898781",
                 grid="#2c2c2a", axis="#383835", band="#ffffff", band_op="0.07",
                 pwm="#3987e5", relay="#d95926"),
}


# ---------------------------------------------------------------------------
# Input
# ---------------------------------------------------------------------------
def parse_log(path, which=-1):
    """Pull every CAP_BEGIN..CAP_END block out of a console log; return one."""
    blocks, cur, label = [], None, None
    for raw in io.open(path, encoding="utf-8", errors="replace"):
        line = raw.split("|", 1)[1].strip() if re.match(r"^\s*\d+\s*\|", raw) else raw.strip()
        if line.startswith("CAP_BEGIN"):
            cur, label = [], line
        elif line.startswith("CAP_END") and cur is not None:
            blocks.append((label, cur))
            cur = None
        elif cur is not None and line[:1].isdigit():
            for rec in line.split("~"):
                parts = rec.split(",")
                if len(parts) == 3 and all(p.strip().isdigit() for p in parts):
                    cur.append(tuple(int(p) for p in parts))
    if not blocks:
        sys.exit("no CAP_BEGIN..CAP_END block found in " + path)
    label, rows = blocks[which]
    print("using %s (%d samples)" % (label, len(rows)))
    return rows


def read_csv(path):
    with io.open(path, encoding="utf-8") as f:
        return [(int(r["t_ms"]), int(r["mA"]), int(r["mV"])) for r in csv.DictReader(f)]


def write_csv(path, rows):
    with io.open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "mA", "mV"])
        w.writerows(rows)


# ---------------------------------------------------------------------------
# Commanded duty, rebuilt from the firmware's motion rules (gate_fsm.c/drv8262.c)
# ---------------------------------------------------------------------------
def pwm_profile(a, travel_s):
    ramp_s = a.ramp_ms / 1000.0
    soft_s = a.soft_end_ms / 1000.0
    ease_s = (a.run_duty - a.soft_end_duty) / 100.0 * ramp_s   # full-range slew
    stop_s = a.soft_end_duty / 100.0 * (a.brake_ramp_ms / 1000.0)
    pts = [(0, 0), (0, a.ramp_start), (ramp_s, a.run_duty), (soft_s, a.run_duty),
           (soft_s + ease_s, a.soft_end_duty), (travel_s, a.soft_end_duty),
           (travel_s + stop_s, 0), (travel_s + stop_s + a.brake_ms / 1000.0, 0)]
    return pts, soft_s, ramp_s


# ---------------------------------------------------------------------------
# SVG
# ---------------------------------------------------------------------------
def fmt(v):
    return ("%.1f" % v).rstrip("0").rstrip(".")


def build_svg(rows, a, theme):
    c = THEMES[theme]
    W, H = 880, 496
    L, R = 58, 24
    phase_y = 84                       # phase-name row
    ax_title_y, ax_top, ax_bot = 112, 124, 300     # current panel
    bx_title_y, bx_top, bx_bot = 338, 350, 436     # duty panel
    t = [r[0] / 1000.0 for r in rows]
    amps = [r[1] / 1000.0 for r in rows]
    travel_s = t[-1]
    pwm, soft_s, ramp_s = pwm_profile(a, travel_s)
    x_max = max(16.0, pwm[-1][0] + 0.4)
    y_max = 9.0

    def X(s):
        return L + (W - L - R) * s / x_max

    def YA(v):
        return ax_bot - (ax_bot - ax_top) * v / y_max

    def YB(v):
        return bx_bot - (bx_bot - bx_top) * v / 100.0

    def line(x1, y1, x2, y2, color, w=1):
        return ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="%s"/>'
                % (x1, y1, x2, y2, color, w))

    def text(x, y, body, size=12, fill=None, anchor="start", weight=None, extra=""):
        return ('<text x="%.1f" y="%.1f" font-size="%s" fill="%s" text-anchor="%s"%s%s>%s</text>'
                % (x, y, size, fill or c["ink2"], anchor,
                   ' font-weight="%s"' % weight if weight else "", extra, body))

    num = ' style="font-variant-numeric:tabular-nums"'
    o = []
    add = o.append
    add('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" width="%d" height="%d" '
        'font-family="system-ui, -apple-system, \'Segoe UI\', sans-serif" role="img" '
        'aria-labelledby="t d">' % (W, H, W, H))
    add('<title id="t">Motor current and commanded duty cycle during one real gate close</title>')
    add('<desc id="d">Current peaks at %s A as the motor launches at %d%% duty, settles near '
        '2.5 to 3 A at full speed, then drops to about 1 A when the firmware eases the duty to '
        '%d%% at %s s, so the gate reaches the closed limit slowly and is then held by a 1 s '
        'short-brake. A relay drives 100%% the whole way and cuts off at the limit.</desc>'
        % (fmt(max(amps)), a.ramp_start, a.soft_end_duty, fmt(soft_s)))
    add('<rect width="%d" height="%d" rx="12" fill="%s"/>' % (W, H, c["surface"]))

    # Title
    add(text(L, 30, "One real close, recorded by the controller itself", 16, c["ink"], weight=600))
    add(text(L, 50, "Motor current sampled every 25 ms, and the PWM duty cycle the firmware "
                    "commanded over the same move", 12.5))

    # Phase bands + markers
    stop_t = travel_s
    brake_end = pwm[-1][0]
    add('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="%s" fill-opacity="%s"/>'
        % (X(stop_t), phase_y + 8, X(brake_end) - X(stop_t), bx_bot - phase_y - 8,
           c["band"], c["band_op"]))
    for s in (ramp_s, soft_s, stop_t):
        add(line(X(s), phase_y + 8, X(s), bx_bot, c["axis"]))
    phases = [((0, ramp_s), "Soft start"), ((ramp_s, soft_s), "Full speed"),
              ((soft_s, stop_t), "Soft end at %d%%" % a.soft_end_duty), ((stop_t, brake_end), "Brake")]
    for (s0, s1), label in phases:
        add(text((X(s0) + X(s1)) / 2, phase_y, label, 12, anchor="middle", weight=600))

    # ---- Panel A: motor current ------------------------------------------
    add(text(L, ax_title_y, "Motor current (A)", 12, weight=600))
    for v in range(0, int(y_max), 2):
        y = YA(v)
        if v:
            add(line(L, y, W - R, y, c["grid"]))
        add(text(L - 10, y + 4, "%d" % v, 11.5, c["muted"], "end", extra=num))
    add(line(L, ax_bot, W - R, ax_bot, c["axis"]))
    pts = " ".join("%.1f,%.1f" % (X(s), YA(v)) for s, v in zip(t, amps))
    add('<polygon points="%.1f,%d %s %.1f,%d" fill="%s" fill-opacity="0.10"/>'
        % (X(t[0]), ax_bot, pts, X(t[-1]), ax_bot, c["pwm"]))
    add('<polyline points="%s" fill="none" stroke="%s" stroke-width="2" '
        'stroke-linejoin="round" stroke-linecap="round"/>' % (pts, c["pwm"]))

    # breakaway peak
    pk = max(range(len(amps)), key=lambda i: amps[i])
    px, py = X(t[pk]), YA(amps[pk])
    add('<circle cx="%.1f" cy="%.1f" r="4.5" fill="%s" stroke="%s" stroke-width="2"/>'
        % (px, py, c["pwm"], c["surface"]))
    add(text(px + 12, YA(8.2), '<tspan font-weight="600" fill="%s">%s A</tspan> breakaway peak '
             'at launch (%d%% duty, not 100%%)' % (c["ink"], fmt(amps[pk]), a.ramp_start)))

    # soft-end dip
    win = [i for i in range(len(t)) if soft_s + 0.3 <= t[i] <= soft_s + 1.6]
    if win:
        lo = min(win, key=lambda i: amps[i])
        lx, ly = X(t[lo]), YA(amps[lo])
        tx, ty = lx + 10, YA(7.3)
        add(line(lx, ty - 10, lx, ly - 7, c["muted"]))
        add('<circle cx="%.1f" cy="%.1f" r="4.5" fill="%s" stroke="%s" stroke-width="2"/>'
            % (lx, ly, c["pwm"], c["surface"]))
        add(text(tx, ty, '<tspan font-weight="600" fill="%s">Eased to %d%%:</tspan>'
                 '<tspan x="%.1f" dy="15">the gate sheds its momentum</tspan>'
                 '<tspan x="%.1f" dy="15">before the limit, not after</tspan>'
                 % (c["ink"], a.soft_end_duty, tx, tx)))

    # ---- Panel B: commanded duty -----------------------------------------
    add(text(L, bx_title_y, "Commanded duty (%)", 12, weight=600))
    lgx = W - R - 352
    add(line(lgx, bx_title_y - 4, lgx + 18, bx_title_y - 4, c["pwm"], 2.5))
    add(text(lgx + 24, bx_title_y, "DRV8262 PWM (this controller)"))
    lgx2 = lgx + 214
    add(line(lgx2, bx_title_y - 4, lgx2 + 18, bx_title_y - 4, c["relay"], 2.5))
    add(text(lgx2 + 24, bx_title_y, "Relay, for comparison"))
    for v in (0, 50, 100):
        y = YB(v)
        if v:
            add(line(L, y, W - R, y, c["grid"]))
        add(text(L - 10, y + 4, "%d" % v, 11.5, c["muted"], "end", extra=num))
    add(line(L, bx_bot, W - R, bx_bot, c["axis"]))
    relay = [(0, 0), (0, 100), (stop_t, 100), (stop_t, 0)]
    for series, color in ((relay, c["relay"]), (pwm, c["pwm"])):
        add('<polyline points="%s" fill="none" stroke="%s" stroke-width="2" '
            'stroke-linejoin="round" stroke-linecap="round"/>'
            % (" ".join("%.1f,%.1f" % (X(s), YB(v)) for s, v in series), color))
    mid = X((soft_s + 1.0 + stop_t) / 2)
    add(text(mid, YB(100) + 15, "relay: full power, then off at the limit", 11.5, anchor="middle"))
    add(text(mid, YB(a.soft_end_duty) + 15, "PWM: %d%%, then a 1 s short-brake" % a.soft_end_duty,
             11.5, anchor="middle"))
    add(text(X(0) + 8, YB(a.ramp_start) + 15, "launch at %d%%" % a.ramp_start, 11.5))

    # ---- X axis ----------------------------------------------------------
    for s in range(0, int(x_max) + 1, 2):
        add(text(X(s), bx_bot + 20, "%d" % s, 11.5, c["muted"], "middle", extra=num))
    add(text((L + W - R) / 2, bx_bot + 44, "seconds since the motor started", 12, c["muted"], "middle"))
    add('</svg>')
    return "\n".join(o) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from-log", help="console log containing a CAP_BEGIN..CAP_END dump")
    ap.add_argument("--block", type=int, default=-1, help="which capture in the log (default: last)")
    ap.add_argument("--csv", help="CSV to write (with --from-log) or read")
    ap.add_argument("--out", help="output prefix; writes <out>-light.svg and <out>-dark.svg")
    ap.add_argument("--ramp-start", type=int, default=75, help="RAMP_START_DUTY, %%")
    ap.add_argument("--run-duty", type=int, default=100, help="RUN_DUTY, %%")
    ap.add_argument("--ramp-ms", type=int, default=2500, help="RAMP_TIME, ms")
    ap.add_argument("--soft-end-ms", type=int, default=9800, help="SOFT_END_OPEN/CLOSE, ms")
    ap.add_argument("--soft-end-duty", type=int, default=60, help="SOFT_END_DUTY, %%")
    ap.add_argument("--brake-ms", type=int, default=1000, help="BRAKE_TIME, ms")
    ap.add_argument("--brake-ramp-ms", type=int, default=100, help="BRAKE_RAMP_MS (gate_config.h)")
    a = ap.parse_args()

    if a.from_log:
        rows = parse_log(a.from_log, a.block)
        if a.csv:
            write_csv(a.csv, rows)
    elif a.csv:
        rows = read_csv(a.csv)
    else:
        ap.error("need --from-log or --csv")

    if a.out:
        for theme in THEMES:
            path = "%s-%s.svg" % (a.out, theme)
            io.open(path, "w", encoding="utf-8", newline="\n").write(build_svg(rows, a, theme))
            print("wrote", path)


if __name__ == "__main__":
    main()
