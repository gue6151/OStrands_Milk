# Small software renderer: the 3BA body with the milk overlay, lit roughly like the game.
import sys, numpy as np
from PIL import Image, ImageFilter

def load_mesh(name):
    d = np.load(f'/home/claude/work/body/fb1_{name}.npz')
    return d['pos'].astype(np.float64), d['uv'].astype(np.float64), d['tris'].astype(np.int64)

def normals(pos, tris):
    fn = np.cross(pos[tris[:, 1]] - pos[tris[:, 0]], pos[tris[:, 2]] - pos[tris[:, 0]])
    vn = np.zeros_like(pos)
    for k in range(3):
        np.add.at(vn, tris[:, k], fn)
    return vn / (np.linalg.norm(vn, axis=1, keepdims=True) + 1e-12)

def render(alpha_img, yaw_deg, out, W=900, H=1300, pitch_deg=4, zoom=1.0, center_z=78.0, center_x=0.0):
    A = np.asarray(alpha_img, np.float32) / 255.0
    TS = A.shape[0]
    meshes = [load_mesh('3BA'), load_mesh('3BA_Vagina')]
    yaw, pitch = np.radians(yaw_deg), np.radians(pitch_deg)
    # camera looks along -y toward the body front (+y) by default; rotate the body instead
    Ry = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, np.cos(pitch), -np.sin(pitch)], [0, np.sin(pitch), np.cos(pitch)]])
    R = Rx @ Ry
    scale = 13.0 * zoom
    color = np.zeros((H, W, 3), np.float32)
    depth = np.full((H, W), -1e9, np.float32)
    bg_top, bg_bot = np.array([0.20, 0.17, 0.15]), np.array([0.10, 0.085, 0.075])
    t = np.linspace(0, 1, H)[:, None, None]
    color[:] = bg_top * (1 - t) + bg_bot * t
    light = np.array([-0.35, 0.75, 0.55]); light /= np.linalg.norm(light)
    rim = np.array([0.6, -0.3, 0.4]); rim /= np.linalg.norm(rim)
    view = np.array([0, 1.0, 0])
    skin = np.array([0.93, 0.80, 0.76])
    milk = np.array([0.99, 0.985, 0.97])
    for mi, (pos, uv, tris) in enumerate(meshes):
        vn = normals(pos, tris)
        p = (pos - np.array([center_x, 0, center_z])) @ R.T
        n = vn @ R.T
        sx = W / 2 + p[:, 0] * scale * -1  # body +x appears on the viewer's left (character's left side)
        sy = H / 2 - p[:, 2] * scale
        sz = p[:, 1]
        for t3 in tris:
            xs, ys = sx[t3], sy[t3]
            x0, x1 = int(max(0, np.floor(xs.min()))), int(min(W - 1, np.ceil(xs.max())))
            y0, y1 = int(max(0, np.floor(ys.min()))), int(min(H - 1, np.ceil(ys.max())))
            if x1 < x0 or y1 < y0:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
            a, b, c = np.stack([xs, ys], 1)
            den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(den) < 1e-9:
                continue
            l0 = ((b[1] - c[1]) * (gx - c[0]) + (c[0] - b[0]) * (gy - c[1])) / den
            l1 = ((c[1] - a[1]) * (gx - c[0]) + (a[0] - c[0]) * (gy - c[1])) / den
            l2 = 1 - l0 - l1
            m = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            if not m.any():
                continue
            L = np.stack([l0[m], l1[m], l2[m]], 1)
            z = L @ sz[t3]
            yy, xx = (gy[m] - 0.5).astype(int), (gx[m] - 0.5).astype(int)
            front = z > depth[yy, xx]
            if not front.any():
                continue
            L, z, yy, xx = L[front], z[front], yy[front], xx[front]
            nn = L @ n[t3]
            nn /= np.linalg.norm(nn, axis=1, keepdims=True) + 1e-9
            if (nn @ view).mean() < 0:      # back face: light it as seen from behind
                nn = -nn
            u = L @ uv[t3]
            if mi == 0:
                tx = np.clip((u[:, 0] * TS).astype(int), 0, TS - 1)
                ty = np.clip((u[:, 1] * TS).astype(int), 0, TS - 1)
                al = A[ty, tx]
            else:
                al = np.zeros(len(L))
            diff = np.clip(nn @ light, 0, 1)
            amb = 0.42 + 0.18 * np.clip(nn[:, 2], -1, 1)
            rimv = np.clip(nn @ rim, 0, 1) ** 2 * 0.25
            base = skin[None, :] * (amb + 0.62 * diff + rimv)[:, None]
            h = light + view; h /= np.linalg.norm(h)
            spec_skin = np.clip(nn @ h, 0, 1) ** 30 * 0.08
            spec_milk = np.clip(nn @ h, 0, 1) ** 70 * 0.9 + np.clip(nn @ h, 0, 1) ** 12 * 0.18
            milk_lit = milk[None, :] * (0.62 + 0.42 * diff + 0.1)[:, None]
            al3 = (np.clip(al * 1.15, 0, 1))[:, None]
            col = base * (1 - al3) + milk_lit * al3 + (spec_skin + spec_milk * al3[:, 0])[:, None]
            color[yy, xx] = col
            depth[yy, xx] = z
    img = Image.fromarray((np.clip(color, 0, 1) ** (1 / 1.05) * 255).astype(np.uint8))
    img = img.resize((W * 2 // 3, H * 2 // 3), Image.LANCZOS)
    img.save(out)

if __name__ == '__main__':
    tex = Image.open(sys.argv[1]).convert('L')
    for yaw, name in [(0, 'front'), (35, 'left'), (-35, 'right'), (180, 'back')]:
        render(tex, yaw, f'{sys.argv[2]}_{name}.png')
