# core/collision.py
# 2.5D 충돌 감지 및 보정
from __future__ import annotations

import math
import numpy as np
from core.shape import Sphere, Box
from core.event import Event

EPS = 1e-12

def _v2(x) -> np.ndarray:
    v = np.asarray(x, dtype=float)
    return v.reshape(2)

def _norm(v: np.ndarray) -> float:
    return float(np.hypot(v[0], v[1]))

def _unit(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(2)
    if not np.all(np.isfinite(v)):
        return np.array([1.0, 0.0], dtype=float)
    n = float(np.hypot(v[0], v[1]))
    if (not math.isfinite(n)) or n < EPS:
        return np.array([1.0, 0.0], dtype=float)
    return v / n


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0]*b[1] - a[1]*b[0])

def _rot_axes(theta: float):
    if not math.isfinite(float(theta)):
        ux = np.array([1.0, 0.0], dtype=float)
        uy = np.array([0.0, 1.0], dtype=float)
        return ux, uy
    c, s = math.cos(theta), math.sin(theta)
    ux = np.array([c, s], dtype=float)
    uy = np.array([-s, c], dtype=float)
    return ux, uy


def _half_extents(box: Box):
    return 0.5*float(box.w), 0.5*float(box.h)

def _proj_radius(axis: np.ndarray, ux: np.ndarray, uy: np.ndarray, hx: float, hy: float) -> float:
    return hx*abs(float(np.dot(axis, ux))) + hy*abs(float(np.dot(axis, uy)))

def collide(a, b) -> Event | None:
    sa, sb = a.shape, b.shape
    if isinstance(sa, Sphere) and isinstance(sb, Sphere):
        return _ss(a, b)
    if isinstance(sa, Sphere) and isinstance(sb, Box):
        return _so(a, b)          # normal: A(sphere)->B(box)
    if isinstance(sa, Box) and isinstance(sb, Sphere):
        e = _so(b, a)             # normal: B(sphere)->A(box)
        if e is None: return None
        e.body_a, e.body_b = a, b
        e.normal = (-e.normal[0], -e.normal[1])  # A(box)->B(sphere)
        return e
    if isinstance(sa, Box) and isinstance(sb, Box):
        return _oo(a, b)
    return None

def _ss(a, b) -> Event | None:
    ca = _v2(a.r[:2]); cb = _v2(b.r[:2])
    ra = float(a.shape.R); rb = float(b.shape.R)

    d = cb - ca
    dist = _norm(d)
    pen = (ra + rb) - dist
    if pen <= 0.0:
        return None

    n = _unit(d)  # A->B
    contact = ca + n*(ra - 0.5*pen)

    e = Event(a, b)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(pen)
    e.contacts = [(float(contact[0]), float(contact[1]))]
    return e

def _so(sphere_body, box_body) -> Event | None:
    """Sphere vs OBB (2D), normal points sphere->box (A->B)."""
    c = _v2(sphere_body.r[:2])
    r = float(sphere_body.shape.R)

    bc = _v2(box_body.r[:2])
    hx, hy = _half_extents(box_body.shape)
    ux, uy = _rot_axes(float(box_body.theta))

    rel = c - bc
    lx = float(np.dot(rel, ux))
    ly = float(np.dot(rel, uy))

    inside = (-hx <= lx <= hx) and (-hy <= ly <= hy)

    cx = max(-hx, min(hx, lx))
    cy = max(-hy, min(hy, ly))
    closest = bc + cx*ux + cy*uy

    if not inside:
        d = closest - c                 # sphere->box
        dist = _norm(d)
        pen = r - dist
        if pen <= 0.0:
            return None
        n = _unit(d)
        contact = closest
    else:
        dx = hx - abs(lx)
        dy = hy - abs(ly)
        if dx < dy:
            sgn = 1.0 if lx >= 0.0 else -1.0
            n = -sgn * ux              # sphere->box
            contact = bc + (sgn*hx)*ux + ly*uy
            pen = r + dx
        else:
            sgn = 1.0 if ly >= 0.0 else -1.0
            n = -sgn * uy              # sphere->box
            contact = bc + lx*ux + (sgn*hy)*uy
            pen = r + dy

    e = Event(sphere_body, box_body)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(pen)
    e.contacts = [(float(contact[0]), float(contact[1]))]
    return e

def _oo(a, b) -> Event | None:
    ca = _v2(a.r[:2]); cb = _v2(b.r[:2])
    hxA, hyA = _half_extents(a.shape)
    hxB, hyB = _half_extents(b.shape)
    uxA, uyA = _rot_axes(float(a.theta))
    uxB, uyB = _rot_axes(float(b.theta))

    d = cb - ca
    axes = (uxA, uyA, uxB, uyB)

    min_overlap = float("inf")
    best_axis = None
    best_from_A = True
    best_is_x = True

    for idx, axis0 in enumerate(axes):
        axis = _unit(axis0)
        ra = _proj_radius(axis, uxA, uyA, hxA, hyA)
        rb = _proj_radius(axis, uxB, uyB, hxB, hyB)
        dist = abs(float(np.dot(d, axis)))
        overlap = (ra + rb) - dist
        if overlap <= 0.0:
            return None
        if overlap < min_overlap:
            min_overlap = overlap
            n = axis if float(np.dot(d, axis)) >= 0.0 else -axis  # A->B
            best_axis = n
            best_from_A = (idx < 2)
            best_is_x = (idx % 2 == 0)

    n = best_axis if best_axis is not None else np.array([1.0, 0.0], dtype=float)

    # manifold (max 2 contacts) via clipping
    if best_from_A:
        ref_c, inc_c = ca, cb
        ref_ux, ref_uy = uxA, uyA
        inc_ux, inc_uy = uxB, uyB
        ref_hx, ref_hy = hxA, hyA
        inc_hx, inc_hy = hxB, hyB
        normal = n                  # ref->inc
    else:
        ref_c, inc_c = cb, ca
        ref_ux, ref_uy = uxB, uyB
        inc_ux, inc_uy = uxA, uyA
        ref_hx, ref_hy = hxB, hyB
        inc_hx, inc_hy = hxA, hyA
        normal = -n                 # ref->inc

    if best_is_x:
        ref_face_n = ref_ux if float(np.dot(normal, ref_ux)) >= 0.0 else -ref_ux
        side = ref_uy
        extent_n, extent_t = ref_hx, ref_hy
    else:
        ref_face_n = ref_uy if float(np.dot(normal, ref_uy)) >= 0.0 else -ref_uy
        side = ref_ux
        extent_n, extent_t = ref_hy, ref_hx

    ref_face_center = ref_c + ref_face_n * extent_n
    v1 = ref_face_center - side * extent_t
    v2 = ref_face_center + side * extent_t

    cand = [inc_ux, -inc_ux, inc_uy, -inc_uy]
    dots = [float(np.dot(ref_face_n, c)) for c in cand]
    inc_face_n = cand[int(np.argmin(dots))]

    if abs(float(np.dot(inc_face_n, inc_ux))) > abs(float(np.dot(inc_face_n, inc_uy))):
        inc_side = inc_uy
        inc_extent_n, inc_extent_t = inc_hx, inc_hy
        inc_n_dir = inc_ux if float(np.dot(inc_face_n, inc_ux)) >= 0.0 else -inc_ux
    else:
        inc_side = inc_ux
        inc_extent_n, inc_extent_t = inc_hy, inc_hx
        inc_n_dir = inc_uy if float(np.dot(inc_face_n, inc_uy)) >= 0.0 else -inc_uy

    inc_face_center = inc_c + inc_n_dir * inc_extent_n
    i1 = inc_face_center - inc_side * inc_extent_t
    i2 = inc_face_center + inc_side * inc_extent_t

    def clip(p1, p2, n_clip, c_clip):
        out = []
        d1 = float(np.dot(n_clip, p1)) - c_clip
        d2 = float(np.dot(n_clip, p2)) - c_clip
        if d1 <= 0.0: out.append(p1)
        if d2 <= 0.0: out.append(p2)
        if d1*d2 < 0.0:
            t = d1 / (d1 - d2)
            out.append(p1 + t*(p2 - p1))
        if len(out) == 0:
            return None
        if len(out) == 1:
            return (out[0], out[0])
        return (out[0], out[1])

    seg = clip(i1, i2,  side, float(np.dot(side, v2)))
    if seg is None: return None
    seg = clip(seg[0], seg[1], -side, float(np.dot(-side, v1)))
    if seg is None: return None
    c1, c2 = seg

    contacts = []
    for p in (c1, c2):
        sep = float(np.dot(ref_face_n, p - ref_face_center))
        if sep <= 1e-9:
            contacts.append((float(p[0]), float(p[1])))
    if not contacts:
        mid = 0.5*(c1+c2)
        contacts = [(float(mid[0]), float(mid[1]))]

    e = Event(a, b)
    e.normal = (float(n[0]), float(n[1]))
    e.penetration = float(min_overlap)
    e.contacts = contacts
    return e

def resolve(event: Event, iterations: int = 1, slop: float = 1e-3, percent: float = 0.6) -> None:
    a = event.body_a
    b = event.body_b

    e = min(float(getattr(a, "restitution", 0.2)), float(getattr(b, "restitution", 0.2)))
    mu = math.sqrt(max(float(getattr(a, "friction", 0.5)), 0.0) * max(float(getattr(b, "friction", 0.5)), 0.0))

    n = _unit(_v2(event.normal))
    contacts = [_v2(c) for c in event.contacts] or [0.5*(_v2(a.r[:2]) + _v2(b.r[:2]))]
    m = len(contacts)

    ma = float(getattr(a, "mass", 0.0)); mb = float(getattr(b, "mass", 0.0))
    Ia = float(getattr(a, "Izz", 0.0));  Ib = float(getattr(b, "Izz", 0.0))
    static_a = bool(getattr(a, "is_static", False))
    static_b = bool(getattr(b, "is_static", False))

    inv_ma = 0.0 if static_a or ma <= 0.0 or math.isinf(ma) else 1.0/ma
    inv_mb = 0.0 if static_b or mb <= 0.0 or math.isinf(mb) else 1.0/mb
    inv_Ia = 0.0 if static_a or Ia <= 0.0 or math.isinf(Ia) else 1.0/Ia
    inv_Ib = 0.0 if static_b or Ib <= 0.0 or math.isinf(Ib) else 1.0/Ib

    def w_cross_r(w, r):
        return np.array([-w*r[1], w*r[0]], dtype=float)

    for _ in range(max(1, int(iterations))):
        for c in contacts:
            ra = c - _v2(a.r[:2])
            rb = c - _v2(b.r[:2])

            va = _v2(a.v[:2]); vb = _v2(b.v[:2])
            wa = float(a.omega[2]); wb = float(b.omega[2])

            rv = (vb + w_cross_r(wb, rb)) - (va + w_cross_r(wa, ra))  # vB - vA
            vel_n = float(np.dot(rv, n))
            if vel_n > 0.0:
                continue

            ra_cn = _cross2(ra, n)
            rb_cn = _cross2(rb, n)
            denom = inv_ma + inv_mb + (ra_cn*ra_cn)*inv_Ia + (rb_cn*rb_cn)*inv_Ib
            if denom < EPS:
                continue

            j = -(1.0 + e) * vel_n / denom
            j /= m
            impulse = j * n

            va -= inv_ma * impulse
            vb += inv_mb * impulse
            wa -= inv_Ia * _cross2(ra, impulse)
            wb += inv_Ib * _cross2(rb, impulse)

            rv2 = (vb + w_cross_r(wb, rb)) - (va + w_cross_r(wa, ra))
            t = rv2 - float(np.dot(rv2, n)) * n
            tmag = _norm(t)
            if tmag > EPS:
                t = t / tmag
                ra_ct = _cross2(ra, t)
                rb_ct = _cross2(rb, t)
                denom_t = inv_ma + inv_mb + (ra_ct*ra_ct)*inv_Ia + (rb_ct*rb_ct)*inv_Ib
                if denom_t > EPS:
                    jt = -float(np.dot(rv2, t)) / denom_t
                    jt = max(-j*mu, min(j*mu, jt))
                    imp_t = jt * t

                    va -= inv_ma * imp_t
                    vb += inv_mb * imp_t
                    wa -= inv_Ia * _cross2(ra, imp_t)
                    wb += inv_Ib * _cross2(rb, imp_t)

            a.v[0], a.v[1] = float(va[0]), float(va[1])
            b.v[0], b.v[1] = float(vb[0]), float(vb[1])
            a.omega[2] = float(wa)
            b.omega[2] = float(wb)

    _positional_correction(a, b, n, float(event.penetration), inv_ma, inv_mb, slop, percent)
    _enforce_2p5d(a); _enforce_2p5d(b)

def _positional_correction(a, b, n, penetration, inv_ma, inv_mb, slop, percent):
    if penetration <= 0.0:
        return
    denom = inv_ma + inv_mb
    if denom < EPS:
        return
    corr_mag = max(penetration - slop, 0.0) / denom * percent
    corr = corr_mag * n
    a.r[0] -= inv_ma * float(corr[0]); a.r[1] -= inv_ma * float(corr[1])
    b.r[0] += inv_mb * float(corr[0]); b.r[1] += inv_mb * float(corr[1])

def _enforce_2p5d(body):
    body.r[2] = 0.0
    body.v[2] = 0.0
    body.omega[0] = 0.0
    body.omega[1] = 0.0
