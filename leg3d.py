# Continues milk streams from the torso texture onto the thighs by tracing them over the real 3BA body surface
# and baking them back into the texture by 3D distance, so both sides of every UV seam get exactly the same milk.
import numpy as np, random
from scipy.spatial import cKDTree

_D = np.load('/home/claude/work/body/posmap.npz')
P, N, I = _D['P'], _D['N'], _D['I']
S = P.shape[0]
_ys, _xs = np.where(I >= 0)
_pts = P[_ys, _xs].astype(np.float64)
_nrm = N[_ys, _xs].astype(np.float64)
_nrm /= np.linalg.norm(_nrm, axis=1, keepdims=True) + 1e-9
_tree = cKDTree(_pts)
_tpts, _tnrm = _pts, _nrm


def uv_to_3d(x, y):
    xi, yi = int(round(x)), int(round(y))
    for r in range(0, 12):
        win = P[max(0, yi - r):yi + r + 1, max(0, xi - r):xi + r + 1].reshape(-1, 3)
        ok = win[np.isfinite(win[:, 0])]
        if len(ok):
            return ok.mean(0).astype(np.float64)
    raise ValueError('no surface near %s,%s' % (x, y))


def units_per_px(x, y):
    xi, yi = int(round(x)), int(round(y))
    a, b, c = P[yi, xi], P[yi, xi + 2], P[yi + 2, xi]
    v = [np.linalg.norm(b - a) / 2, np.linalg.norm(c - a) / 2]
    v = [q for q in v if np.isfinite(q)]
    return float(np.mean(v)) if v else 0.03


def snap(p):
    d, i = _tree.query(p)
    return _tpts[i], _tnrm[i], d


def trace(p0, d0, length, outward, rng, ds=0.1, gravity=1.0, decay=2.5):
    """Gravity-driven run over the surface. outward: +1/-1 = push toward that thigh (3D x) at the start."""
    p, n, _ = snap(p0)
    d = d0 / (np.linalg.norm(d0) + 1e-9)
    pts = [p.copy()]
    ph = [rng.uniform(0, 6.28) for _ in range(3)]
    fr = [rng.uniform(0.35, 0.9), rng.uniform(0.9, 1.8), rng.uniform(2.0, 3.2)]
    am = [rng.uniform(0.05, 0.12), rng.uniform(0.03, 0.07), rng.uniform(0.01, 0.03)]
    travelled = 0.0
    while travelled < length:
        g = np.array([0.0, 0.0, -1.0])
        gp = g - n * g.dot(n)
        side = np.cross(n, gp if np.linalg.norm(gp) > 1e-3 else d)
        side /= np.linalg.norm(side) + 1e-9
        push = np.array([outward * 1.0, 0.0, 0.0]) * np.exp(-travelled / decay)
        wiggle = sum(a * np.cos(f * travelled + q) for a, f, q in zip(am, fr, ph))
        want = gp * gravity + push + side * wiggle
        want = want - n * want.dot(n)
        if np.linalg.norm(want) < 1e-4:
            want = d
        want /= np.linalg.norm(want) + 1e-9
        d = d * 0.82 + want * 0.18
        d = d - n * d.dot(n)
        d /= np.linalg.norm(d) + 1e-9
        q, n2, gap = snap(p + d * ds)
        if np.linalg.norm(q - p) > ds * 4 + 0.15:   # jumped to another body part (e.g. the other thigh): stop
            break
        p, n = q, n2
        if np.linalg.norm(p - pts[-1]) >= ds * 0.5:
            pts.append(p.copy())
        travelled += ds
    return np.array(pts)


def smooth(pts, win=17):
    if len(pts) < 5:
        return pts
    k = np.exp(-np.linspace(-2, 2, win) ** 2); k /= k.sum()
    pad = win // 2
    ext = np.concatenate([np.repeat(pts[:1], pad, 0), pts, np.repeat(pts[-1:], pad, 0)])
    out = np.stack([np.convolve(ext[:, j], k, mode='valid') for j in range(3)], 1)
    # pull the smoothed line back onto the surface
    _, i = _tree.query(out)
    surf, nrm = _tpts[i], _tnrm[i]
    out = out - nrm * np.sum((out - surf) * nrm, axis=1, keepdims=True)
    out[0] = pts[0]
    return out


def bake(streams, length, seed=0, stage=8):
    rng = random.Random(seed * 977 + 13)
    samples, radius, value, halo = [], [], [], []
    cur_halo = [0.3]

    def add_tube(pts, r0, r1, v):
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0], np.cumsum(seg)])
        total = max(s[-1], 1e-6)
        # dense resampling so the tube is smooth
        k = max(2, int(total / 0.02))
        t = np.linspace(0, total, k)
        dense = np.stack([np.interp(t, s, pts[:, j]) for j in range(3)], 1)
        u = t / total
        bulge = 1 + 0.12 * np.sin(u * rng.uniform(9, 15) + rng.uniform(0, 6.28))
        samples.append(dense)
        radius.append((r0 + (r1 - r0) * u) * bulge)
        value.append(np.full(k, v))
        halo.append(np.full(k, cur_halo[0]))

    seen_side = {}
    for st in streams:
        ex, ey = st['end']
        px, py = st['prev']
        p_end = uv_to_3d(ex, ey)
        p_prev = uv_to_3d(px, py)
        d0 = p_end - p_prev
        if np.linalg.norm(d0) < 1e-4:
            d0 = np.array([0, 0, -1.0])
        upp = units_per_px(ex, ey)
        r = max(st['w'] * upp / 2, 0.08)
        cur_halo[0] = 6.5 * upp
        tail3 = np.array([uv_to_3d(x, y) for (x, y) in st['tail'][::3]]) if len(st.get('tail', [])) > 6 else None
        outward = 1.0 if st['side'] == 0 else -1.0       # UV left half = +x in 3D
        if st['kind'] == 'reach':
            L = length * rng.uniform(0.75, 1.15)
        else:
            L = length * rng.uniform(0.6, 1.0)
            outward *= 0.25
        # start a little back up the torso stroke so the two overlap seamlessly
        start = p_end - d0 / (np.linalg.norm(d0) + 1e-9) * 0.25
        if st['kind'] == 'reach':
            def over_vulva(scale):
                # down the outer lip, then curving in along the rim of the vaginal opening, gathering there in a drop,
                # with a little running on below it. (The labia/opening itself is a separate mesh the overlay cannot
                # paint, so the strand follows the body skin right up to its rim.)
                o = outward
                lip = rng.uniform(1.1, 1.45)
                guide = [(o * lip, start[2] - 0.2),
                         (o * lip * 0.98, 66.6),
                         (o * rng.uniform(1.0, 1.2), 66.15),
                         (o * rng.uniform(0.7, 0.85), 65.8),
                         (o * rng.uniform(0.38, 0.5), 65.5),      # rim of the opening
                         (o * rng.uniform(0.5, 0.65), 65.2),
                         (o * rng.uniform(0.62, 0.8), 64.95)]     # runs on below the opening
                ytab = {67.6: 3.9, 67.0: 3.6, 66.6: 3.3, 66.15: 3.0, 65.8: 2.6, 65.5: 2.3, 65.2: 1.95, 64.95: 1.5}
                def yfor(z):
                    ks = sorted(ytab)
                    return float(np.interp(z, ks, [ytab[k] for k in ks]))
                pts = []
                for (x0, z0), (x1, z1) in zip(guide[:-1], guide[1:]):
                    n = max(2, int(np.hypot(x1 - x0, z1 - z0) / 0.04))
                    for t in np.linspace(0, 1, n, endpoint=False):
                        x = x0 + (x1 - x0) * t; z = z0 + (z1 - z0) * t
                        q, _, _ = snap(np.array([x, yfor(z) + 0.35, z]))
                        pts.append(q)
                pts = np.array(pts)
                if len(pts) < 6:
                    return
                pts = smooth(pts, 9)
                rim = int(len(pts) * 0.78)
                add_tube(pts[:rim + 1], r * scale, r * scale * 0.9, 0.92)
                add_tube(pts[rim:], r * scale * 0.85, r * scale * 0.5, 0.9)
                # milk gathering on the rim of the opening, and a drop hanging where it runs off
                k = int(len(pts) * 0.7)
                samples.append(pts[k][None, :]); radius.append(np.array([r * scale * 1.5])); value.append(np.array([0.95])); halo.append(np.array([cur_halo[0]]))
                samples.append(pts[-1][None, :]); radius.append(np.array([r * scale * 1.3])); value.append(np.array([0.95])); halo.append(np.array([cur_halo[0]]))
            if not seen_side and stage >= 6:
                # down the middle: over the clitoral hood into the top of the slit, then along both inner edges of the
                # slit down into the vaginal opening
                ytab = {68.2: 4.15, 67.6: 3.9, 67.0: 3.65, 66.6: 3.45, 66.3: 3.2, 66.0: 2.95, 65.75: 2.6, 65.5: 2.2}
                def yfor(z):
                    ks = sorted(ytab)
                    return float(np.interp(z, ks, [ytab[k] for k in ks]))
                def along(guide):
                    pts = []
                    for (x0, z0), (x1, z1) in zip(guide[:-1], guide[1:]):
                        n = max(2, int(np.hypot(x1 - x0, z1 - z0) / 0.03))
                        for t in np.linspace(0, 1, n, endpoint=False):
                            x = x0 + (x1 - x0) * t; z = z0 + (z1 - z0) * t
                            q, _, _ = snap(np.array([x, yfor(z) + 0.3, z]))
                            pts.append(q)
                    return smooth(np.array(pts), 7) if len(pts) > 6 else None
                rc = r * 0.85
                top = start[2] - 0.1
                hood = along([(start[0], start[2] + 0.35), (start[0] * 0.5, top - 0.3), (0.02, 67.4), (0.0, 66.8), (0.0, 66.42)])
                if hood is not None:
                    add_tube(hood, rc, rc * 1.05, 0.93)
                    # milk gathering on the clitoral hood
                    samples.append(hood[-1][None, :]); radius.append(np.array([rc * 1.45])); value.append(np.array([0.95])); halo.append(np.array([cur_halo[0]]))
                for o in (1.0, -1.0):
                    edge = along([(0.0, 66.42), (o * 0.27, 66.2), (o * 0.33, 65.95), (o * 0.36, 65.7), (o * 0.4, 65.5)])
                    if edge is not None:
                        add_tube(edge, rc * 0.8, rc * 0.6, 0.9)
                        samples.append(edge[-1][None, :]); radius.append(np.array([rc * 1.1])); value.append(np.array([0.95])); halo.append(np.array([cur_halo[0]]))
            first = seen_side.get(st['side'], 0) == 0
            seen_side[st['side']] = seen_side.get(st['side'], 0) + 1
            if first:
                # the first stream on each side flows into the vulva itself
                over_vulva(1.0)
                continue
            # the rest spread along the groin crease onto the inner thigh; a thinner part still runs over the vulva
            off = start + np.array([outward * rng.uniform(0.6, 1.2), 0.0, 0.0])
            bridge = trace(off, np.array([outward * 0.35, 0.0, -1.0]), rng.uniform(3.5, 5.0), outward * 0.55, rng, gravity=1.0, decay=60.0)
            rest = trace(bridge[-1], bridge[-1] - bridge[max(0, len(bridge) - 6)], L, outward * 0.3, rng)
            pts = np.concatenate([bridge, rest[1:]]) if len(rest) > 1 else bridge
            if rng.random() < 0.6:
                over_vulva(0.65)
        else:
            pts = trace(start, d0, L, outward, rng)
        if len(pts) < 4:
            continue
        if tail3 is not None and len(tail3) > 3:
            # run the 3D stroke over the end of the torso stroke so the two blend instead of meeting edge to edge
            pts = np.concatenate([tail3, pts])
        pts = smooth(smooth(pts))
        v = 0.62 if st['dried'] else 0.9
        add_tube(pts, r, r * (0.55 if st['dried'] else 0.75), v)
        # drop where it stops (dried: a flat ring, wet: a hanging bead)
        end = pts[-1]
        rb = r * (1.25 if not st['dried'] else 1.1)
        samples.append(end[None, :]); radius.append(np.array([rb])); value.append(np.array([0.95 if not st['dried'] else 0.75])); halo.append(np.array([cur_halo[0]]))
        # a thinner branch now and then, splitting off lower down the thigh
        if not st['dried'] and len(pts) > 40 and rng.random() < 0.45 + 0.05 * stage:
            j = rng.randint(len(pts) // 3, 2 * len(pts) // 3)
            bd = pts[min(j + 3, len(pts) - 1)] - pts[j]
            sub = trace(pts[j], bd + np.array([0, 0, -0.2]), L * rng.uniform(0.25, 0.5), -outward * 0.6, rng)
            if len(sub) > 4:
                sub = smooth(smooth(sub))
                add_tube(sub, r * 0.6, r * 0.35, 0.85)
                samples.append(sub[-1][None, :]); radius.append(np.array([r * 0.8])); value.append(np.array([0.92])); halo.append(np.array([cur_halo[0]]))

    out = np.zeros((S, S), np.float32)
    if not samples:
        return out
    sp = np.concatenate(samples); sr = np.concatenate(radius); sv = np.concatenate(value); sh = np.concatenate(halo)
    tree = cKDTree(sp)
    reach = max(sr.max() * 2.6, sh.max() * 2.2)
    lo, hi = sp.min(0) - reach, sp.max(0) + reach
    m = np.all((_pts >= lo) & (_pts <= hi), axis=1)
    q = _pts[m]
    dist, idx = tree.query(q, k=24, distance_upper_bound=reach)
    ok_any = np.isfinite(dist).any(1)
    dist, idx, q_ok = dist[ok_any], idx[ok_any], None
    valid = np.isfinite(dist)
    idx_c = np.where(valid, idx, 0)
    rr, vv, hh = sr[idx_c], sv[idx_c], sh[idx_c]
    dd = np.where(valid, dist, 1e9)
    core = np.clip((rr - dd) / (0.3 * rr), 0, 1) * vv                              # the stream itself, soft edge
    halo = 0.38 * np.exp(-((dd / np.maximum(1.8 * rr, hh)) ** 2)) * (vv > 0.8)        # wet sheen around it
    a = np.maximum(core, halo).max(1)
    ok = ok_any
    ys, xs = _ys[m][ok], _xs[m][ok]
    np.maximum.at(out, (ys, xs), a.astype(np.float32))
    return out
