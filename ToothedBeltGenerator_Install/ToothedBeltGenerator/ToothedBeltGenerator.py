import adsk.core
import adsk.fusion
import traceback
import math
import json

handlers = []
is_updating_belts = False
active_editing_base_feat = None
cached_selected_entities = []

CMD_ID = 'ToothedBeltGeneratorCmdId'
EDIT_CMD_ID = 'ToothedBeltEditCmdId'
CMD_NAME = 'Toothed Belt Generator'
EDIT_CMD_NAME = 'Edit Toothed Belt'
CMD_DESC = 'Generate parametric GT1.5, GT2, HTD 5M, and HTD 8M timing belts along a 2D sketch path.'
EDIT_CMD_DESC = 'Edit parameters and profile settings of an existing Toothed Belt body.'
PANEL_ID = 'SolidCreatePanel'
ATTR_GROUP = 'ToothedBeltGenerator'
ATTR_CONFIG_KEY = 'beltConfig'
ATTR_SIG_KEY = 'pathSignature'

BELT_PROFILES_MM = {
    'Gates GT1.5 (1.5GT)': {
        'pitch': 1.50, 'total_h': 1.00, 'tooth_h': 0.56, 'backing_h': 0.44,
        'tip_r': 0.415, 'flank_r': 0.75, 'flank_offset': 0.30, 'root_r': 0.115, 'pld': 0.230
    },
    'Gates GT2 (2GT)': {
        'pitch': 2.00, 'total_h': 1.38, 'tooth_h': 0.75, 'backing_h': 0.63,
        'tip_r': 0.555, 'flank_r': 1.00, 'flank_offset': 0.40, 'root_r': 0.150, 'pld': 0.254
    },
    'HTD 5M': {
        'pitch': 5.00, 'total_h': 3.80, 'tooth_h': 2.06, 'backing_h': 1.74,
        'tip_r': 1.49, 'flank_r': 2.50, 'flank_offset': 0.83, 'root_r': 0.43, 'pld': 0.5715
    },
    'HTD 8M': {
        'pitch': 8.00, 'total_h': 6.00, 'tooth_h': 3.38, 'backing_h': 2.62,
        'tip_r': 2.46, 'flank_r': 4.10, 'flank_offset': 1.36, 'root_r': 0.68, 'pld': 0.686
    }
}


def ensure_no_active_base_feature_edit():
    try:
        app = adsk.core.Application.get()
        active_bf = adsk.fusion.BaseFeature.cast(app.activeEditObject)
        if active_bf and active_bf.isValid:
            active_bf.finishEdit()
    except:
        pass


def clear_visualizer_graphics(design):
    if not design:
        return
    try:
        root_cg = design.rootComponent.customGraphicsGroups
        for i in range(root_cg.count - 1, -1, -1):
            grp = root_cg.item(i)
            if grp and grp.isValid:
                grp.deleteMe()
    except:
        pass


def evaluate_belt_width_cm(design, config):
    expr = config.get('belt_width_expr', '').strip()
    if expr and design:
        if design.allParameters:
            param = design.allParameters.itemByName(expr)
            if param and param.isValid and param.value > 0:
                return param.value
        if design.unitsManager:
            try:
                val_cm = design.unitsManager.evaluateExpression(expr, 'mm')
                if val_cm > 0:
                    return val_cm
            except:
                pass
    return float(config.get('belt_width_cm', 0.6))


def get_belt_base_feature_and_config(entity):
    if not entity or not entity.isValid:
        return None, None
    body = adsk.fusion.BRepBody.cast(entity)
    if body:
        cfg_attr = body.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
        if body.baseFeature and body.baseFeature.isValid:
            bf_attr = body.baseFeature.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
            use_attr = bf_attr if bf_attr else cfg_attr
            if use_attr:
                return body.baseFeature, json.loads(use_attr.value)
    bf = adsk.fusion.BaseFeature.cast(entity)
    if bf and bf.isValid:
        cfg_attr = bf.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
        if cfg_attr:
            return bf, json.loads(cfg_attr.value)
    return None, None


def offset_point(pt, vec, dist):
    return adsk.core.Point3D.create(pt.x + vec.x * dist, pt.y + vec.y * dist, pt.z + vec.z * dist)


def midpoint(p1, p2):
    return adsk.core.Point3D.create(0.5 * (p1.x + p2.x), 0.5 * (p1.y + p2.y), 0.5 * (p1.z + p2.z))


def intersect_two_circles_positive_u(x1, y1, r1, x2, y2, r2):
    dx, dy = x2 - x1, y2 - y1
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return None
    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h_sq = r1 * r1 - a * a
    if h_sq < 0.0:
        return None
    h = math.sqrt(h_sq)
    xm, ym = x1 + a * (dx / d), y1 + a * (dy / d)
    rx, ry = -dy * (h / d), dx * (h / d)
    p1, p2 = (xm + rx, ym + ry), (xm - rx, ym - ry)
    cands = [p for p in (p1, p2) if p[0] > 0.0]
    if not cands:
        return p1 if p1[0] >= p2[0] else p2
    cands.sort(key=lambda pt: pt[0])
    return cands[0]


def solve_side_root_fillet(kappa_side, pitch_cm, tooth_h_cm, pld_cm, tip_r_cm, flank_r_cm, flank_offset_cm, root_r_cm):
    y_ct = tooth_h_cm - tip_r_cm
    dr = max(1e-6, flank_r_cm - tip_r_cm)
    dy_cf = math.sqrt(max(0.0, dr * dr - flank_offset_cm * flank_offset_cm))
    y_cf = y_ct - dy_cf
    d_flank = flank_r_cm + root_r_cm

    y_cr_flat = root_r_cm
    u_cr_flat = -flank_offset_cm + math.sqrt(max(1e-8, d_flank * d_flank - (y_cr_flat - y_cf) ** 2))
    pinched = False
    alpha = 0.0

    if abs(kappa_side) < 1e-4:
        u_cr, y_cr, u_surf_tan, y_surf_tan = u_cr_flat, y_cr_flat, u_cr_flat, 0.0
    elif kappa_side < 0.0:
        r_path = 1.0 / abs(kappa_side)
        r_surf = r_path + pld_cm
        d_surf = r_surf + root_r_cm
        sol = intersect_two_circles_positive_u(-flank_offset_cm, y_cf, d_flank, 0.0, -r_surf, d_surf)
        if sol is not None:
            u_cr, y_cr = sol
            u_surf_tan = u_cr * (r_surf / d_surf)
            y_surf_tan = -r_surf + (y_cr + r_surf) * (r_surf / d_surf)
        else:
            u_cr, y_cr, u_surf_tan, y_surf_tan = u_cr_flat, y_cr_flat, u_cr_flat, 0.0
    else:
        r_path = max(pld_cm + 2.2 * root_r_cm, 1.0 / kappa_side)
        r_surf = max(1.5 * root_r_cm, r_path - pld_cm)
        d_surf = max(0.2 * root_r_cm, r_surf - root_r_cm)
        alpha = min(math.pi * 0.36, 0.5 * pitch_cm / r_path)
        sol = intersect_two_circles_positive_u(-flank_offset_cm, y_cf, d_flank, 0.0, r_surf, d_surf)
        need_pinch = True
        if sol is not None:
            u_cr_try, y_cr_try = sol
            if u_cr_try <= (r_surf - y_cr_try) * math.tan(alpha):
                u_cr, y_cr = u_cr_try, y_cr_try
                u_surf_tan = u_cr * (r_surf / d_surf)
                y_surf_tan = r_surf - (r_surf - y_cr) * (r_surf / d_surf)
                need_pinch = False
        if need_pinch:
            pinched = True
            dy0 = r_surf - y_cf
            sin_a, cos_a = math.sin(alpha), math.cos(alpha)
            b_q = flank_offset_cm * sin_a - dy0 * cos_a
            c_q = flank_offset_cm * flank_offset_cm + dy0 * dy0 - d_flank * d_flank
            tau = -b_q + math.sqrt(max(0.0, b_q * b_q - c_q))
            u_cr, y_cr = tau * sin_a, r_surf - tau * cos_a
            tau_v = tau + root_r_cm
            u_surf_tan, y_surf_tan = tau_v * sin_a, r_surf - tau_v * cos_a

    vx, vy = u_cr + flank_offset_cm, y_cr - y_cf
    v_len = max(1e-8, math.hypot(vx, vy))
    y_flank_tan = y_cf + flank_r_cm * (vy / v_len)
    return {
        'u_cr': u_cr, 'y_cr': y_cr, 'u_surf_tan': u_surf_tan, 'y_surf_tan': y_surf_tan,
        'y_flank_tan': y_flank_tan, 'pinched': pinched, 'alpha': alpha
    }


def create_canonical_tooth_transient_body(
    pitch_cm, tooth_h_cm, backing_h_cm, pld_cm, tip_r_cm, flank_r_cm, flank_offset_cm,
    root_r_cm, belt_width_cm, is_double_sided, flip_side, symmetric_width,
    include_root_fillet, kappa_left=0.0, kappa_right=0.0
):
    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    half_hb = 0.5 * backing_h_cm
    z0 = -0.5 * belt_width_cm if symmetric_width else 0.0
    z1 = 0.5 * belt_width_cm if symmetric_width else belt_width_cm
    z_mid = 0.5 * (z0 + z1)
    x_axis = adsk.core.Vector3D.create(1.0, 0.0, 0.0)
    y_axis = adsk.core.Vector3D.create(0.0, 1.0, 0.0)

    y_ct = tooth_h_cm - tip_r_cm
    dr = max(1e-6, flank_r_cm - tip_r_cm)
    dy_cf = math.sqrt(max(0.0, dr * dr - flank_offset_cm * flank_offset_cm))
    y_cf = y_ct - dy_cf
    y_tan = y_ct + tip_r_cm * (dy_cf / dr)

    def build_single_side_tooth(s_sign, k_left_signed, k_right_signed):
        y_cf_w = s_sign * (half_hb + y_cf)
        cyl_r = temp_mgr.createCylinderOrCone(
            adsk.core.Point3D.create(-flank_offset_cm, y_cf_w, z0), flank_r_cm,
            adsk.core.Point3D.create(-flank_offset_cm, y_cf_w, z1), flank_r_cm
        )
        cyl_l = temp_mgr.createCylinderOrCone(
            adsk.core.Point3D.create(flank_offset_cm, y_cf_w, z0), flank_r_cm,
            adsk.core.Point3D.create(flank_offset_cm, y_cf_w, z1), flank_r_cm
        )
        temp_mgr.booleanOperation(cyl_r, cyl_l, adsk.fusion.BooleanTypes.IntersectionBooleanType)

        clip_h = half_hb + y_tan
        clip_obb = adsk.core.OrientedBoundingBox3D.create(
            adsk.core.Point3D.create(0.0, 0.5 * s_sign * clip_h, z_mid),
            x_axis, y_axis, 2.5 * flank_r_cm, clip_h, belt_width_cm
        )
        tooth_part = temp_mgr.createBox(clip_obb)
        temp_mgr.booleanOperation(tooth_part, cyl_r, adsk.fusion.BooleanTypes.IntersectionBooleanType)

        y_ct_w = s_sign * (half_hb + y_ct)
        tip_cyl = temp_mgr.createCylinderOrCone(
            adsk.core.Point3D.create(0.0, y_ct_w, z0), tip_r_cm,
            adsk.core.Point3D.create(0.0, y_ct_w, z1), tip_r_cm
        )
        temp_mgr.booleanOperation(tooth_part, tip_cyl, adsk.fusion.BooleanTypes.UnionBooleanType)

        if half_hb + y_ct - tip_r_cm < 0.0:
            total_h_box = half_hb + tooth_h_cm + 0.05
            bound_obb = adsk.core.OrientedBoundingBox3D.create(
                adsk.core.Point3D.create(0.0, 0.5 * s_sign * total_h_box, z_mid),
                x_axis, y_axis, 3.0 * flank_r_cm, total_h_box, belt_width_cm
            )
            temp_mgr.booleanOperation(tooth_part, temp_mgr.createBox(bound_obb), adsk.fusion.BooleanTypes.IntersectionBooleanType)

        if include_root_fillet and root_r_cm > 1e-5:
            for side_x_sign, k_val in ((1.0, k_right_signed), (-1.0, k_left_signed)):
                sol = solve_side_root_fillet(
                    k_val, pitch_cm, tooth_h_cm, pld_cm, tip_r_cm, flank_r_cm, flank_offset_cm, root_r_cm
                )
                u_surf, y_surf = sol['u_surf_tan'], sol['y_surf_tan']
                y_bot_rel = max(-half_hb * 0.92, y_surf - 0.42 * backing_h_cm)
                y_top_rel = max(sol['y_flank_tan'], y_surf + 1e-4)
                y_bot_w = s_sign * (half_hb + y_bot_rel)
                y_top_w = s_sign * (half_hb + y_top_rel)
                flare_h = abs(y_top_w - y_bot_w)

                if flare_h > 1e-5 and u_surf > 1e-5:
                    flare_obb = adsk.core.OrientedBoundingBox3D.create(
                        adsk.core.Point3D.create(0.5 * side_x_sign * u_surf, 0.5 * (y_bot_w + y_top_w), z_mid),
                        x_axis, y_axis, u_surf, flare_h, belt_width_cm
                    )
                    flare_box = temp_mgr.createBox(flare_obb)
                    z_pad = 0.02
                    y_cr_w = s_sign * (half_hb + sol['y_cr'])
                    c_rf0 = adsk.core.Point3D.create(side_x_sign * sol['u_cr'], y_cr_w, z0 - z_pad)
                    c_rf1 = adsk.core.Point3D.create(side_x_sign * sol['u_cr'], y_cr_w, z1 + z_pad)
                    rf_cyl = temp_mgr.createCylinderOrCone(c_rf0, root_r_cm, c_rf1, root_r_cm)
                    temp_mgr.booleanOperation(flare_box, rf_cyl, adsk.fusion.BooleanTypes.DifferenceBooleanType)

                    if sol['pinched'] and sol['alpha'] > 1e-4:
                        a = sol['alpha']
                        n_vec = adsk.core.Vector3D.create(side_x_sign * math.cos(a), s_sign * math.sin(a), 0.0)
                        t_vec = adsk.core.Vector3D.create(-side_x_sign * math.sin(a), s_sign * math.cos(a), 0.0)
                        bis_pt = adsk.core.Point3D.create(side_x_sign * u_surf, s_sign * (half_hb + y_surf), z_mid)
                        cut_sz = 4.0 * pitch_cm
                        bis_obb = adsk.core.OrientedBoundingBox3D.create(
                            offset_point(bis_pt, n_vec, 0.5 * cut_sz), t_vec, n_vec, cut_sz, cut_sz, 2.0 * belt_width_cm
                        )
                        temp_mgr.booleanOperation(flare_box, temp_mgr.createBox(bis_obb), adsk.fusion.BooleanTypes.DifferenceBooleanType)

                    temp_mgr.booleanOperation(tooth_part, flare_box, adsk.fusion.BooleanTypes.UnionBooleanType)
                    rf_cyl2 = temp_mgr.createCylinderOrCone(c_rf0, root_r_cm, c_rf1, root_r_cm)
                    temp_mgr.booleanOperation(tooth_part, rf_cyl2, adsk.fusion.BooleanTypes.DifferenceBooleanType)
        return tooth_part

    if not is_double_sided:
        s = -1.0 if flip_side else 1.0
        return build_single_side_tooth(s, s * kappa_left, s * kappa_right)
    else:
        pos_t = build_single_side_tooth(1.0, kappa_left, kappa_right)
        neg_t = build_single_side_tooth(-1.0, -kappa_left, -kappa_right)
        temp_mgr.booleanOperation(pos_t, neg_t, adsk.fusion.BooleanTypes.UnionBooleanType)
        return pos_t


def collect_chained_sketch_curves(seed_curves, auto_chain):
    valid_seeds = [c for c in seed_curves if c and c.isValid]
    if not valid_seeds or not auto_chain:
        return valid_seeds
    sketch = valid_seeds[0].parentSketch
    candidates = [crv for crv in sketch.sketchCurves if crv and crv.isValid and not crv.isConstruction]
    tol = 1e-3

    def endpoints_of(sc):
        ev = sc.worldGeometry.evaluator
        _, sp, ep = ev.getParameterExtents()
        _, ps = ev.getPointAtParameter(sp)
        _, pe = ev.getPointAtParameter(ep)
        return ps, pe

    chained = list(valid_seeds)
    tokens = set(c.entityToken for c in chained)
    pts = []
    for c in chained:
        pts.extend(endpoints_of(c))

    added = True
    while added:
        added = False
        for cand in candidates:
            tok = cand.entityToken
            if tok in tokens:
                continue
            ps, pe = endpoints_of(cand)
            if any(ps.distanceTo(ep) < tol or pe.distanceTo(ep) < tol for ep in pts):
                chained.append(cand)
                tokens.add(tok)
                pts.extend([ps, pe])
                added = True
    return chained


def build_ordered_curve_chain_from_sketch_curves(sketch_curves):
    raw = []
    for sc in sketch_curves:
        if not sc or not sc.isValid or not sc.worldGeometry:
            continue
        crv = sc.worldGeometry
        ev = crv.evaluator
        _, sp, ep = ev.getParameterExtents()
        _, length = ev.getLengthAtParameter(sp, ep)
        _, ps = ev.getPointAtParameter(sp)
        _, pe = ev.getPointAtParameter(ep)
        raw.append({'sketch_curve': sc, 'curve': crv, 'evaluator': ev, 's_prm': sp, 'e_prm': ep, 'length': length, 'p_start': ps, 'p_end': pe})

    if not raw:
        return [], 0.0, False
    tol = 1e-3
    if len(raw) == 1:
        seg = dict(raw[0])
        seg['reversed'] = False
        return [seg], seg['length'], (seg['p_start'].distanceTo(seg['p_end']) < tol)

    used = [False] * len(raw)
    first = dict(raw[0]); first['reversed'] = False; used[0] = True
    ordered = [first]
    c_start, c_end = first['p_start'], first['p_end']

    progress = True
    while progress:
        progress = False
        for i, seg in enumerate(raw):
            if used[i]:
                continue
            if c_end.distanceTo(seg['p_start']) < tol:
                s = dict(seg); s['reversed'] = False; ordered.append(s); c_end = seg['p_end']; used[i] = True; progress = True; break
            elif c_end.distanceTo(seg['p_end']) < tol:
                s = dict(seg); s['reversed'] = True; ordered.append(s); c_end = seg['p_start']; used[i] = True; progress = True; break
            elif c_start.distanceTo(seg['p_end']) < tol:
                s = dict(seg); s['reversed'] = False; ordered.insert(0, s); c_start = seg['p_start']; used[i] = True; progress = True; break
            elif c_start.distanceTo(seg['p_start']) < tol:
                s = dict(seg); s['reversed'] = True; ordered.insert(0, s); c_start = seg['p_end']; used[i] = True; progress = True; break

    for i, seg in enumerate(raw):
        if not used[i]:
            s = dict(seg); s['reversed'] = False; ordered.append(s)
    return ordered, sum(s['length'] for s in ordered), (c_start.distanceTo(c_end) < tol)


def compute_chain_signature(ordered_chain, total_length_cm, config=None, eval_width_cm=None):
    parts = ['v24_vis', f'{total_length_cm:.5f}']
    if config:
        w_val = eval_width_cm if eval_width_cm is not None else float(config.get('belt_width_cm', 0.6))
        parts.append(
            f"{config.get('profile_name', '')}:{config.get('is_double_sided', False)}:"
            f"{config.get('flip_side', False)}:{w_val:.5f}:{config.get('symmetric_width', True)}:"
            f"{config.get('auto_chain', True)}:{config.get('include_root_fillet', True)}:"
            f"{config.get('belt_width_expr', '')}"
        )
    for seg in ordered_chain:
        _, pm = seg['evaluator'].getPointAtParameter(0.5 * (seg['s_prm'] + seg['e_prm']))
        ps, pe = seg['p_start'], seg['p_end']
        parts.append(f"{seg['length']:.5f}:{ps.x:.4f},{ps.y:.4f}:{pm.x:.4f},{pm.y:.4f}:{pe.x:.4f},{pe.y:.4f}")
    return '|'.join(parts)


def evaluate_chain_at_distance(ordered_chain, dist_cm):
    accum = 0.0
    for seg in ordered_chain:
        seg_len = seg['length']
        if dist_cm <= accum + seg_len or seg is ordered_chain[-1]:
            local_d = max(0.0, min(seg_len, dist_cm - accum))
            ev, s_prm, e_prm = seg['evaluator'], seg['s_prm'], seg['e_prm']
            eval_len = max(0.0, min(seg_len, (seg_len - local_d) if seg['reversed'] else local_d))
            if eval_len <= 1e-6:
                prm = s_prm
            elif eval_len >= seg_len - 1e-6:
                prm = e_prm
            else:
                ok, prm = ev.getParameterAtLength(s_prm, eval_len)
                if not ok:
                    prm = s_prm + (e_prm - s_prm) * (eval_len / max(seg_len, 1e-6))
            _, pt = ev.getPointAtParameter(prm)
            _, deriv = ev.getFirstDerivative(prm)
            if seg['reversed']:
                deriv.scaleBy(-1.0)
            deriv.normalize()
            return pt, deriv
        accum += seg_len

    last = ordered_chain[-1]
    prm = last['s_prm'] if last['reversed'] else last['e_prm']
    _, pt = last['evaluator'].getPointAtParameter(prm)
    _, deriv = last['evaluator'].getFirstDerivative(prm)
    if last['reversed']:
        deriv.scaleBy(-1.0)
    deriv.normalize()
    return pt, deriv


def estimate_local_side_curvatures(ordered_chain, total_length_cm, is_closed_path, dist_cm, pt_center, normal_in_plane, sample_offset_cm):
    def clamp_d(d):
        return (d % total_length_cm) if is_closed_path else max(0.0, min(total_length_cm, d))

    pt_l, _ = evaluate_chain_at_distance(ordered_chain, clamp_d(dist_cm - sample_offset_cm))
    pt_r, _ = evaluate_chain_at_distance(ordered_chain, clamp_d(dist_cm + sample_offset_cm))
    chord_l = pt_center.distanceTo(pt_l)
    kl = (2.0 * pt_center.vectorTo(pt_l).dotProduct(normal_in_plane) / (chord_l * chord_l)) if chord_l > 1e-5 else 0.0
    chord_r = pt_center.distanceTo(pt_r)
    kr = (2.0 * pt_center.vectorTo(pt_r).dotProduct(normal_in_plane) / (chord_r * chord_r)) if chord_r > 1e-5 else 0.0
    ql = 0.0 if abs(kl) < 0.01 else round(kl * 50.0) / 50.0
    qr = 0.0 if abs(kr) < 0.01 else round(kr * 50.0) / 50.0
    return ql, qr


def draw_tooth_side_visualizer(design, seed_curves, auto_chain, profile_name, is_double_sided, flip_side, belt_width_cm, symmetric_width):
    clear_visualizer_graphics(design)
    chained_curves = collect_chained_sketch_curves(seed_curves, auto_chain)
    if not chained_curves:
        return

    sketch = chained_curves[0].parentSketch
    sketch_normal = sketch.xDirection.crossProduct(sketch.yDirection)
    sketch_normal.normalize()

    ordered_chain, total_length_cm, is_closed_path = build_ordered_curve_chain_from_sketch_curves(chained_curves)
    if total_length_cm <= 1e-5 or not ordered_chain:
        return

    spec_mm = BELT_PROFILES_MM.get(profile_name, BELT_PROFILES_MM['Gates GT2 (2GT)'])
    pitch_cm = spec_mm['pitch'] * 0.1
    tooth_h_cm = spec_mm['tooth_h'] * 0.1
    backing_h_cm = spec_mm['backing_h'] * 0.1
    pld_cm = spec_mm['pld'] * 0.1
    half_hb = 0.5 * backing_h_cm

    s_sign = -1.0 if flip_side else 1.0
    y_shift_cm = 0.0 if is_double_sided else s_sign * (pld_cm - half_hb)
    active_sides = (1.0, -1.0) if is_double_sided else (s_sign,)

    w_cm = max(0.05, float(belt_width_cm))
    z0 = -0.5 * w_cm if symmetric_width else 0.0
    z1 = 0.5 * w_cm if symmetric_width else w_cm
    z_mid = 0.5 * (z0 + z1)

    cg_group = design.rootComponent.customGraphicsGroups.add()
    cg_group.depthPriority = 10

    if is_closed_path:
        num_teeth = max(1, int(round(total_length_cm / pitch_cm)))
        eff_step = total_length_cm / num_teeth
        start_off = 0.0
    else:
        num_teeth = max(1, int(math.floor(total_length_cm / pitch_cm)))
        eff_step = pitch_cm
        start_off = 0.5 * (total_length_cm - (num_teeth - 1) * pitch_cm)

    comb_stride = max(1, int(math.ceil(num_teeth / 500.0)))
    half_tooth_span = 0.36 * pitch_cm
    comb_segments = []

    for side in active_sides:
        prev_right_pt = None
        for k in range(0, num_teeth, comb_stride):
            dist = start_off + k * eff_step
            pt, tangent = evaluate_chain_at_distance(ordered_chain, dist)
            n_in_plane = sketch_normal.crossProduct(tangent)
            n_in_plane.normalize()

            root_center = offset_point(pt, n_in_plane, y_shift_cm + side * half_hb)
            root_center = offset_point(root_center, sketch_normal, z_mid)

            tip_pt = offset_point(root_center, n_in_plane, side * tooth_h_cm)
            left_pt = offset_point(root_center, tangent, -half_tooth_span)
            right_pt = offset_point(root_center, tangent, half_tooth_span)

            if prev_right_pt is not None and comb_stride == 1:
                comb_segments.extend([
                    prev_right_pt.x, prev_right_pt.y, prev_right_pt.z,
                    left_pt.x, left_pt.y, left_pt.z
                ])

            comb_segments.extend([
                left_pt.x, left_pt.y, left_pt.z,
                tip_pt.x, tip_pt.y, tip_pt.z,
                tip_pt.x, tip_pt.y, tip_pt.z,
                right_pt.x, right_pt.y, right_pt.z
            ])
            prev_right_pt = right_pt

    if comb_segments:
        comb_coords = adsk.fusion.CustomGraphicsCoordinates.create(comb_segments)
        comb_lines = cg_group.addLines(comb_coords, [], False)
        comb_lines.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
            adsk.core.Color.create(0, 230, 110, 255)
        )
        comb_lines.weight = 2.0
        comb_lines.isSelectable = False
        comb_lines.depthPriority = 10

    rail_steps = max(24, min(140, int(round(total_length_cm / max(0.25, 1.5 * pitch_cm)))))
    rail_segments = []
    back_offset = y_shift_cm - s_sign * half_hb if not is_double_sided else -half_hb

    prev_z0 = None
    prev_z1 = None
    for i in range(rail_steps + 1):
        dist = (i / float(rail_steps)) * total_length_cm
        pt, tangent = evaluate_chain_at_distance(ordered_chain, dist)
        n_in_plane = sketch_normal.crossProduct(tangent)
        n_in_plane.normalize()

        back_pt = offset_point(pt, n_in_plane, back_offset)
        p_z0 = offset_point(back_pt, sketch_normal, z0)
        p_z1 = offset_point(back_pt, sketch_normal, z1)

        if prev_z0 is not None:
            rail_segments.extend([
                prev_z0.x, prev_z0.y, prev_z0.z, p_z0.x, p_z0.y, p_z0.z,
                prev_z1.x, prev_z1.y, prev_z1.z, p_z1.x, p_z1.y, p_z1.z
            ])
        if i % 8 == 0 or i == rail_steps:
            rail_segments.extend([
                p_z0.x, p_z0.y, p_z0.z, p_z1.x, p_z1.y, p_z1.z
            ])
        prev_z0 = p_z0
        prev_z1 = p_z1

    if rail_segments:
        rail_coords = adsk.fusion.CustomGraphicsCoordinates.create(rail_segments)
        rail_lines = cg_group.addLines(rail_coords, [], False)
        rail_lines.color = adsk.fusion.CustomGraphicsSolidColorEffect.create(
            adsk.core.Color.create(0, 190, 255, 255)
        )
        rail_lines.weight = 1.5
        rail_lines.isSelectable = False
        rail_lines.depthPriority = 9

    try:
        adsk.core.Application.get().activeViewport.refresh()
    except:
        pass


def update_visualizer_from_inputs(inputs, is_edit):
    global active_editing_base_feat, cached_selected_entities
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design or not inputs:
            return

        sel_in = adsk.core.SelectionCommandInput.cast(inputs.itemById('entitySelection'))
        raw_entities = [
            sel_in.selection(i).entity
            for i in range(sel_in.selectionCount)
            if sel_in.selection(i).entity and sel_in.selection(i).entity.isValid
        ]
        if not raw_entities and cached_selected_entities:
            raw_entities = [e for e in cached_selected_entities if e and e.isValid]

        auto_chain_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('autoChain')).value
        profile_name_val = adsk.core.DropDownCommandInput.cast(inputs.itemById('beltProfile')).selectedItem.name
        is_dbl_val = (adsk.core.DropDownCommandInput.cast(inputs.itemById('toothSidedness')).selectedItem.name == 'Double-Sided Teeth')
        flip_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('flipSide')).value
        width_input = adsk.core.ValueCommandInput.cast(inputs.itemById('beltWidth'))
        w_cm = width_input.value if width_input.isValidExpression and width_input.value > 0 else 0.6
        sym_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('symmetricWidth')).value

        seed_curves = []
        if is_edit:
            target_bf = active_editing_base_feat
            for ent in raw_entities:
                bf, _ = get_belt_base_feature_and_config(ent)
                if bf:
                    target_bf = bf
                    break
            if target_bf and target_bf.isValid:
                cfg_attr = target_bf.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
                if cfg_attr:
                    cfg = json.loads(cfg_attr.value)
                    seed_curves = resolve_seed_curves_for_feature(design, cfg)
        else:
            seed_curves = [
                adsk.fusion.SketchCurve.cast(ent)
                for ent in raw_entities
                if adsk.fusion.SketchCurve.cast(ent)
            ]

        if seed_curves:
            draw_tooth_side_visualizer(
                design=design,
                seed_curves=seed_curves,
                auto_chain=auto_chain_val,
                profile_name=profile_name_val,
                is_double_sided=is_dbl_val,
                flip_side=flip_val,
                belt_width_cm=w_cm,
                symmetric_width=sym_val
            )
        else:
            clear_visualizer_graphics(design)
    except:
        pass


def create_arc_subsegment_body(temp_mgr, center_pt, radius, prm1, prm2, ev, sketch_normal, backing_h_cm, belt_width_cm, z0, z1, z_mid):
    half_hb = 0.5 * backing_h_cm
    r_out = radius + half_hb
    r_in = max(1e-4, radius - half_hb)
    c_z0 = offset_point(center_pt, sketch_normal, z0)
    c_z1 = offset_point(center_pt, sketch_normal, z1)
    ring_body = temp_mgr.createCylinderOrCone(c_z0, r_out, c_z1, r_out)
    if radius > half_hb + 1e-4:
        inner_cyl = temp_mgr.createCylinderOrCone(c_z0, r_in, c_z1, r_in)
        temp_mgr.booleanOperation(ring_body, inner_cyl, adsk.fusion.BooleanTypes.DifferenceBooleanType)

    _, p1 = ev.getPointAtParameter(prm1)
    _, pm = ev.getPointAtParameter(0.5 * (prm1 + prm2))
    _, p2 = ev.getPointAtParameter(prm2)
    u1 = center_pt.vectorTo(p1); u1.normalize()
    u2 = center_pt.vectorTo(p2); u2.normalize()

    n1 = sketch_normal.crossProduct(u1); n1.normalize()
    if n1.dotProduct(p1.vectorTo(pm)) > 0:
        n1.scaleBy(-1.0)
    n2 = sketch_normal.crossProduct(u2); n2.normalize()
    if n2.dotProduct(p2.vectorTo(pm)) > 0:
        n2.scaleBy(-1.0)

    box_sz = 4.0 * r_out
    cut_h = 2.0 * belt_width_cm
    c_mid = offset_point(center_pt, sketch_normal, z_mid)
    obb1 = adsk.core.OrientedBoundingBox3D.create(offset_point(c_mid, n1, 0.5 * box_sz), u1, n1, box_sz, box_sz, cut_h)
    temp_mgr.booleanOperation(ring_body, temp_mgr.createBox(obb1), adsk.fusion.BooleanTypes.DifferenceBooleanType)
    obb2 = adsk.core.OrientedBoundingBox3D.create(offset_point(c_mid, n2, 0.5 * box_sz), u2, n2, box_sz, box_sz, cut_h)
    temp_mgr.booleanOperation(ring_body, temp_mgr.createBox(obb2), adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return ring_body


def create_backing_transient_body(ordered_chain, is_closed_path, sketch_normal, backing_h_cm, y_shift_cm, belt_width_cm, symmetric_width):
    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    backing_body = None
    half_hb = 0.5 * backing_h_cm
    z0 = -0.5 * belt_width_cm if symmetric_width else 0.0
    z1 = 0.5 * belt_width_cm if symmetric_width else belt_width_cm
    z_mid = 0.5 * (z0 + z1)

    def union_part(part):
        nonlocal backing_body
        if part is not None:
            if backing_body is None:
                backing_body = part
            else:
                temp_mgr.booleanOperation(backing_body, part, adsk.fusion.BooleanTypes.UnionBooleanType)

    def oriented_normal_at(seg, prm):
        _, d = seg['evaluator'].getFirstDerivative(prm)
        if seg['reversed']:
            d.scaleBy(-1.0)
        d.normalize()
        n = sketch_normal.crossProduct(d)
        n.normalize()
        return n

    def make_joint_cyl(pt, n_vec):
        pt_s = offset_point(pt, n_vec, y_shift_cm)
        return temp_mgr.createCylinderOrCone(
            offset_point(pt_s, sketch_normal, z0), half_hb,
            offset_point(pt_s, sketch_normal, z1), half_hb
        )

    def discretize_segment(seg):
        ev, s_prm, e_prm, seg_len = seg['evaluator'], seg['s_prm'], seg['e_prm'], seg['length']
        steps = max(6, int(math.ceil(seg_len / min(0.08, 0.4 * backing_h_cm))))
        step_len = seg_len / steps
        for k in range(steps):
            la, lb = k * step_len, min(seg_len, (k + 1) * step_len)
            ok_a, pa_prm = ev.getParameterAtLength(s_prm, la)
            ok_b, pb_prm = ev.getParameterAtLength(s_prm, lb)
            if not ok_a:
                pa_prm = s_prm + (e_prm - s_prm) * (la / max(seg_len, 1e-6))
            if not ok_b:
                pb_prm = s_prm + (e_prm - s_prm) * (lb / max(seg_len, 1e-6))
            _, pa = ev.getPointAtParameter(pa_prm)
            _, pb = ev.getPointAtParameter(pb_prm)
            na, nb = oriented_normal_at(seg, pa_prm), oriented_normal_at(seg, pb_prm)
            pa_s, pb_s = offset_point(pa, na, y_shift_cm), offset_point(pb, nb, y_shift_cm)
            chord = pa_s.distanceTo(pb_s)
            if chord > 1e-5:
                t_vec = pa_s.vectorTo(pb_s); t_vec.normalize()
                n_vec = sketch_normal.crossProduct(t_vec); n_vec.normalize()
                mid_pt = offset_point(midpoint(pa_s, pb_s), sketch_normal, z_mid)
                obb = adsk.core.OrientedBoundingBox3D.create(mid_pt, t_vec, n_vec, chord, backing_h_cm, belt_width_cm)
                union_part(temp_mgr.createBox(obb))
                if k > 0:
                    union_part(make_joint_cyl(pa, na))

    for idx, seg in enumerate(ordered_chain):
        crv, ev, s_prm, e_prm, seg_len = seg['curve'], seg['evaluator'], seg['s_prm'], seg['e_prm'], seg['length']
        ctype = crv.curveType

        if ctype == adsk.core.Curve3DTypes.Line3DCurveType and seg_len > 1e-5:
            p_s, p_e = seg['p_start'], seg['p_end']
            n_chain = oriented_normal_at(seg, s_prm)
            p_s_shifted = offset_point(p_s, n_chain, y_shift_cm)
            p_e_shifted = offset_point(p_e, n_chain, y_shift_cm)
            t_vec = p_s_shifted.vectorTo(p_e_shifted); t_vec.normalize()
            n_vec = sketch_normal.crossProduct(t_vec); n_vec.normalize()
            mid_pt = offset_point(midpoint(p_s_shifted, p_e_shifted), sketch_normal, z_mid)
            obb = adsk.core.OrientedBoundingBox3D.create(mid_pt, t_vec, n_vec, seg_len, backing_h_cm, belt_width_cm)
            union_part(temp_mgr.createBox(obb))

        elif ctype == adsk.core.Curve3DTypes.Circle3DCurveType:
            circ = adsk.core.Circle3D.cast(crv)
            c_pt = circ.center
            _, p_test = ev.getPointAtParameter(s_prm)
            n_test = oriented_normal_at(seg, s_prm)
            rad_sign = 1.0 if n_test.dotProduct(c_pt.vectorTo(p_test)) >= 0.0 else -1.0
            r_eff = max(half_hb + 1e-4, circ.radius + rad_sign * y_shift_cm)
            c_z0, c_z1 = offset_point(c_pt, sketch_normal, z0), offset_point(c_pt, sketch_normal, z1)
            ring = temp_mgr.createCylinderOrCone(c_z0, r_eff + half_hb, c_z1, r_eff + half_hb)
            if r_eff > half_hb + 1e-4:
                inner = temp_mgr.createCylinderOrCone(c_z0, r_eff - half_hb, c_z1, r_eff - half_hb)
                temp_mgr.booleanOperation(ring, inner, adsk.fusion.BooleanTypes.DifferenceBooleanType)
            union_part(ring)

        elif ctype == adsk.core.Curve3DTypes.Arc3DCurveType and seg_len > 1e-5:
            arc = adsk.core.Arc3D.cast(crv)
            c_pt = arc.center
            m_prm = 0.5 * (s_prm + e_prm)
            _, p_test = ev.getPointAtParameter(m_prm)
            n_test = oriented_normal_at(seg, m_prm)
            rad_sign = 1.0 if n_test.dotProduct(c_pt.vectorTo(p_test)) >= 0.0 else -1.0
            r_eff = arc.radius + rad_sign * y_shift_cm
            sweep_angle = seg_len / max(arc.radius, 1e-5)

            if r_eff > half_hb + 1e-4 and sweep_angle < 2.0 * math.pi - 1e-3:
                n_sub = max(1, int(math.ceil(sweep_angle / (0.42 * math.pi))))
                d_prm = (e_prm - s_prm) / n_sub
                for k in range(n_sub):
                    sp1, sp2 = s_prm + k * d_prm, s_prm + (k + 1) * d_prm
                    union_part(create_arc_subsegment_body(temp_mgr, c_pt, r_eff, sp1, sp2, ev, sketch_normal, backing_h_cm, belt_width_cm, z0, z1, z_mid))
                    if k > 0:
                        _, pj = ev.getPointAtParameter(sp1)
                        union_part(make_joint_cyl(pj, oriented_normal_at(seg, sp1)))
            else:
                discretize_segment(seg)
        else:
            discretize_segment(seg)

        if idx > 0:
            j_prm = seg['e_prm'] if seg['reversed'] else seg['s_prm']
            j_pt = seg['p_end'] if seg['reversed'] else seg['p_start']
            union_part(make_joint_cyl(j_pt, oriented_normal_at(seg, j_prm)))

    if is_closed_path and len(ordered_chain) > 1:
        f_seg = ordered_chain[0]
        s_prm0 = f_seg['e_prm'] if f_seg['reversed'] else f_seg['s_prm']
        s_pt0 = f_seg['p_end'] if f_seg['reversed'] else f_seg['p_start']
        union_part(make_joint_cyl(s_pt0, oriented_normal_at(f_seg, s_prm0)))

    return backing_body


def build_belt_transient_body_from_curves(design, seed_curves, config):
    auto_chain = bool(config.get('auto_chain', True))
    profile_name = config.get('profile_name', 'Gates GT2 (2GT)')
    is_double_sided = bool(config.get('is_double_sided', False))
    flip_side = bool(config.get('flip_side', False))
    belt_width_cm = evaluate_belt_width_cm(design, config)
    symmetric_width = bool(config.get('symmetric_width', True))
    include_root_fillet = bool(config.get('include_root_fillet', True))

    chained_curves = collect_chained_sketch_curves(seed_curves, auto_chain)
    if not chained_curves:
        return None, '', '', []

    sketch = chained_curves[0].parentSketch
    sketch_normal = sketch.xDirection.crossProduct(sketch.yDirection)
    sketch_normal.normalize()

    ordered_chain, total_length_cm, is_closed_path = build_ordered_curve_chain_from_sketch_curves(chained_curves)
    if total_length_cm <= 1e-5 or not ordered_chain:
        return None, '', '', []

    signature = compute_chain_signature(ordered_chain, total_length_cm, config, belt_width_cm)
    spec_mm = BELT_PROFILES_MM[profile_name]
    pitch_cm = spec_mm['pitch'] * 0.1
    tooth_h_cm = spec_mm['tooth_h'] * 0.1
    backing_h_cm = spec_mm['backing_h'] * 0.1
    pld_cm = spec_mm['pld'] * 0.1
    tip_r_cm = spec_mm['tip_r'] * 0.1
    flank_r_cm = spec_mm['flank_r'] * 0.1
    flank_offset_cm = spec_mm['flank_offset'] * 0.1
    root_r_cm = spec_mm['root_r'] * 0.1

    s_sign = -1.0 if flip_side else 1.0
    y_shift_cm = 0.0 if is_double_sided else s_sign * (pld_cm - 0.5 * backing_h_cm)

    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    belt_body = create_backing_transient_body(
        ordered_chain, is_closed_path, sketch_normal, backing_h_cm, y_shift_cm, belt_width_cm, symmetric_width
    )

    if is_closed_path:
        num_teeth = max(1, int(round(total_length_cm / pitch_cm)))
        effective_step = total_length_cm / num_teeth
        start_offset = 0.0
    else:
        num_teeth = max(1, int(math.floor(total_length_cm / pitch_cm)))
        effective_step = pitch_cm
        start_offset = 0.5 * (total_length_cm - (num_teeth - 1) * pitch_cm)

    tooth_cache = {}
    sample_offset_cm = 0.35 * pitch_cm

    for k in range(num_teeth):
        dist = start_offset + k * effective_step
        pt, tangent = evaluate_chain_at_distance(ordered_chain, dist)
        normal_in_plane = sketch_normal.crossProduct(tangent)
        normal_in_plane.normalize()

        if include_root_fillet:
            q_kl, q_kr = estimate_local_side_curvatures(
                ordered_chain, total_length_cm, is_closed_path, dist, pt, normal_in_plane, sample_offset_cm
            )
        else:
            q_kl, q_kr = 0.0, 0.0

        cache_key = (q_kl, q_kr)
        if cache_key not in tooth_cache:
            tooth_cache[cache_key] = create_canonical_tooth_transient_body(
                pitch_cm, tooth_h_cm, backing_h_cm, pld_cm, tip_r_cm, flank_r_cm, flank_offset_cm,
                root_r_cm, belt_width_cm, is_double_sided, flip_side, symmetric_width,
                include_root_fillet, q_kl, q_kr
            )

        pt_shifted = offset_point(pt, normal_in_plane, y_shift_cm)
        mat = adsk.core.Matrix3D.create()
        mat.setWithCoordinateSystem(pt_shifted, tangent, normal_in_plane, sketch_normal)
        tooth_inst = temp_mgr.copy(tooth_cache[cache_key])
        temp_mgr.transform(tooth_inst, mat)
        temp_mgr.booleanOperation(belt_body, tooth_inst, adsk.fusion.BooleanTypes.UnionBooleanType)

    side_tag = 'DoubleSided' if is_double_sided else 'SingleSided'
    body_name = f'{profile_name}_{side_tag}_{belt_width_cm * 10.0:.1f}mm_{num_teeth}T'
    return belt_body, body_name, signature, chained_curves


def resolve_seed_curves_for_feature(design, config):
    seed_curves = []
    for tok in config.get('seed_tokens', []):
        ents = design.findEntityByToken(tok)
        if ents:
            sc = adsk.fusion.SketchCurve.cast(ents[0])
            if sc and sc.isValid:
                seed_curves.append(sc)
    if not seed_curves and config.get('sketch_token', ''):
        ents = design.findEntityByToken(config['sketch_token'])
        if ents:
            sk = adsk.fusion.Sketch.cast(ents[0])
            if sk and sk.isValid:
                for crv in sk.sketchCurves:
                    if crv and crv.isValid and not crv.isConstruction:
                        seed_curves.append(crv)
                        break
    return seed_curves


def update_single_belt_base_feature(design, base_feat, force_rebuild=False):
    if not base_feat or not base_feat.isValid:
        return False
    cfg_attr = base_feat.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
    if not cfg_attr:
        return False

    config = json.loads(cfg_attr.value)
    seed_curves = resolve_seed_curves_for_feature(design, config)
    if not seed_curves:
        return False

    chained_curves = collect_chained_sketch_curves(seed_curves, bool(config.get('auto_chain', True)))
    if not chained_curves:
        return False
    ordered_chain, total_length_cm, _ = build_ordered_curve_chain_from_sketch_curves(chained_curves)
    if total_length_cm <= 1e-5:
        return False

    eval_width_cm = evaluate_belt_width_cm(design, config)
    new_sig = compute_chain_signature(ordered_chain, total_length_cm, config, eval_width_cm)
    sig_attr = base_feat.attributes.itemByName(ATTR_GROUP, ATTR_SIG_KEY)

    if not force_rebuild and sig_attr and sig_attr.value == new_sig:
        return False

    config['belt_width_cm'] = eval_width_cm
    belt_body, body_name, signature, _ = build_belt_transient_body_from_curves(design, seed_curves, config)
    if not belt_body:
        return False

    ensure_no_active_base_feature_edit()
    comp = base_feat.parentComponent
    timeline_pos = design.timeline.markerPosition if design.timeline else None

    try:
        base_feat.startEdit()
        try:
            if base_feat.bodies.count > 0:
                target_body = base_feat.bodies.item(0)
                base_feat.updateBody(target_body, belt_body)
                target_body.name = body_name
                target_body.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))
            else:
                created = comp.bRepBodies.add(belt_body, base_feat)
                created.name = body_name
                created.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))
        finally:
            try:
                base_feat.finishEdit()
            except:
                pass
    except:
        pass

    base_feat.name = body_name
    base_feat.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))
    base_feat.attributes.add(ATTR_GROUP, ATTR_SIG_KEY, signature)
    if base_feat.bodies.count > 0:
        base_feat.bodies.item(0).attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))

    if timeline_pos is not None and design.timeline.markerPosition != timeline_pos:
        design.timeline.markerPosition = timeline_pos
    return True


def refresh_all_parametric_belts():
    global is_updating_belts
    if is_updating_belts:
        return
    try:
        is_updating_belts = True
        app = adsk.core.Application.get()
        if adsk.fusion.Sketch.cast(app.activeEditObject) is not None:
            return
        if adsk.fusion.BaseFeature.cast(app.activeEditObject) is not None:
            return
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            return
        seen = set()
        for attr in design.findAttributes(ATTR_GROUP, ATTR_CONFIG_KEY):
            ent = attr.parent
            base_feat = adsk.fusion.BaseFeature.cast(ent)
            if not base_feat:
                body = adsk.fusion.BRepBody.cast(ent)
                if body and body.baseFeature:
                    base_feat = body.baseFeature
            if base_feat and base_feat.isValid and base_feat.entityToken not in seen:
                seen.add(base_feat.entityToken)
                update_single_belt_base_feature(design, base_feat, False)
    except:
        pass
    finally:
        is_updating_belts = False


class BeltWorkspaceWatcherHandler(adsk.core.ApplicationCommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd_id = adsk.core.ApplicationCommandEventArgs.cast(args).commandId
            if cmd_id in (
                'SelectCommand', CMD_ID, EDIT_CMD_ID, 'Undo', 'Redo',
                'BaseFeatureEdit', 'BaseFeatureFinishEdit', 'ScriptsAndAddinsCmd'
            ):
                return
            refresh_all_parametric_belts()
        except:
            pass


class BeltCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
            clear_visualizer_graphics(design)
        except:
            pass


class BeltCommandPreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self, is_edit=False):
        super().__init__()
        self.is_edit = is_edit

    def notify(self, args):
        try:
            event_args = adsk.core.CommandEventArgs.cast(args)
            event_args.isValidResult = False
            update_visualizer_from_inputs(event_args.command.commandInputs, self.is_edit)
        except:
            pass


class BeltInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self, is_edit=False):
        super().__init__()
        self.is_edit = is_edit

    def notify(self, args):
        global active_editing_base_feat, cached_selected_entities
        try:
            event_args = adsk.core.InputChangedEventArgs.cast(args)
            changed_id = event_args.input.id
            inputs = event_args.inputs
            design = adsk.fusion.Design.cast(app = adsk.core.Application.get().activeProduct) # type: ignore

            # Simple guard check for active design
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)

            if changed_id == 'toothSidedness':
                side_dd = adsk.core.DropDownCommandInput.cast(inputs.itemById('toothSidedness'))
                flip_in = adsk.core.BoolValueCommandInput.cast(inputs.itemById('flipSide'))
                flip_in.isEnabled = (side_dd.selectedItem.name != 'Double-Sided Teeth')

            elif changed_id == 'entitySelection':
                sel_in = adsk.core.SelectionCommandInput.cast(inputs.itemById('entitySelection'))
                if sel_in.selectionCount > 0:
                    cached_selected_entities = [
                        sel_in.selection(i).entity for i in range(sel_in.selectionCount)
                        if sel_in.selection(i).entity and sel_in.selection(i).entity.isValid
                    ]
                    bf, cfg = get_belt_base_feature_and_config(sel_in.selection(0).entity)
                    if bf and cfg:
                        active_editing_base_feat = bf
                        adsk.core.BoolValueCommandInput.cast(inputs.itemById('autoChain')).value = bool(cfg.get('auto_chain', True))
                        for item in adsk.core.DropDownCommandInput.cast(inputs.itemById('beltProfile')).listItems:
                            item.isSelected = (item.name == cfg.get('profile_name', 'Gates GT2 (2GT)'))
                        is_dbl = bool(cfg.get('is_double_sided', False))
                        for item in adsk.core.DropDownCommandInput.cast(inputs.itemById('toothSidedness')).listItems:
                            item.isSelected = (item.name == ('Double-Sided Teeth' if is_dbl else 'Single-Sided Teeth'))
                        flip_in = adsk.core.BoolValueCommandInput.cast(inputs.itemById('flipSide'))
                        flip_in.value = bool(cfg.get('flip_side', False))
                        flip_in.isEnabled = not is_dbl
                        adsk.core.BoolValueCommandInput.cast(inputs.itemById('includeRootFillet')).value = bool(cfg.get('include_root_fillet', True))

                        w_in = adsk.core.ValueCommandInput.cast(inputs.itemById('beltWidth'))
                        expr = cfg.get('belt_width_expr', '').strip()
                        try:
                            if expr:
                                w_in.expression = expr
                            else:
                                w_in.value = evaluate_belt_width_cm(design, cfg)
                        except:
                            w_in.value = evaluate_belt_width_cm(design, cfg)

                        adsk.core.BoolValueCommandInput.cast(inputs.itemById('symmetricWidth')).value = bool(cfg.get('symmetric_width', True))
                else:
                    cached_selected_entities = []

            update_visualizer_from_inputs(inputs, self.is_edit)
        except:
            pass


class BeltCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self, is_edit=False):
        super().__init__()
        self.is_edit = is_edit

    def notify(self, args):
        global active_editing_base_feat, cached_selected_entities
        ui = None
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox('An active Fusion Design is required.')
                return

            clear_visualizer_graphics(design)

            inputs = adsk.core.CommandEventArgs.cast(args).command.commandInputs
            sel_in = adsk.core.SelectionCommandInput.cast(inputs.itemById('entitySelection'))

            raw_entities = [
                sel_in.selection(i).entity
                for i in range(sel_in.selectionCount)
                if sel_in.selection(i).entity and sel_in.selection(i).entity.isValid
            ]
            if not raw_entities and cached_selected_entities:
                raw_entities = [e for e in cached_selected_entities if e and e.isValid]

            width_input = adsk.core.ValueCommandInput.cast(inputs.itemById('beltWidth'))
            target_width_cm = width_input.value
            target_width_expr = width_input.expression.strip()
            auto_chain_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('autoChain')).value
            profile_name_val = adsk.core.DropDownCommandInput.cast(inputs.itemById('beltProfile')).selectedItem.name
            is_dbl_val = (adsk.core.DropDownCommandInput.cast(inputs.itemById('toothSidedness')).selectedItem.name == 'Double-Sided Teeth')
            flip_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('flipSide')).value
            fillet_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('includeRootFillet')).value
            sym_val = adsk.core.BoolValueCommandInput.cast(inputs.itemById('symmetricWidth')).value

            if target_width_cm <= 0:
                ui.messageBox('Belt width must be greater than zero.')
                return

            if self.is_edit:
                for ent in raw_entities:
                    bf, _ = get_belt_base_feature_and_config(ent)
                    if bf:
                        active_editing_base_feat = bf
                        break

                if not active_editing_base_feat or not active_editing_base_feat.isValid:
                    ui.messageBox('No valid belt feature selected for editing.')
                    return

                cfg_attr = active_editing_base_feat.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
                existing_config = json.loads(cfg_attr.value) if cfg_attr else {}

                new_config = {
                    'seed_tokens': existing_config.get('seed_tokens', []),
                    'sketch_token': existing_config.get('sketch_token', ''),
                    'auto_chain': auto_chain_val,
                    'profile_name': profile_name_val,
                    'is_double_sided': is_dbl_val,
                    'flip_side': flip_val,
                    'include_root_fillet': fillet_val,
                    'belt_width_cm': target_width_cm,
                    'belt_width_expr': target_width_expr,
                    'symmetric_width': sym_val
                }
                active_editing_base_feat.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(new_config))
                update_single_belt_base_feature(design, active_editing_base_feat, True)
                return

            # CREATE MODE
            selected_curves = [
                adsk.fusion.SketchCurve.cast(ent)
                for ent in raw_entities
                if adsk.fusion.SketchCurve.cast(ent)
            ]
            if not selected_curves:
                ui.messageBox('Please select at least one valid 2D sketch curve.')
                return

            seed_tokens = [c.entityToken for c in selected_curves]
            sketch_token = selected_curves[0].parentSketch.entityToken
            comp = selected_curves[0].parentSketch.parentComponent

            config = {
                'seed_tokens': seed_tokens,
                'sketch_token': sketch_token,
                'auto_chain': auto_chain_val,
                'profile_name': profile_name_val,
                'is_double_sided': is_dbl_val,
                'flip_side': flip_val,
                'include_root_fillet': fillet_val,
                'belt_width_cm': target_width_cm,
                'belt_width_expr': target_width_expr,
                'symmetric_width': sym_val
            }

            belt_body, body_name, signature, _ = build_belt_transient_body_from_curves(design, selected_curves, config)
            if not belt_body:
                ui.messageBox('Could not generate belt geometry from the selected path.')
                return

            ensure_no_active_base_feature_edit()
            base_feat = comp.features.baseFeatures.add()
            base_feat.startEdit()
            try:
                created_body = comp.bRepBodies.add(belt_body, base_feat)
                created_body.name = body_name
                created_body.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))
            finally:
                try:
                    base_feat.finishEdit()
                except:
                    pass

            base_feat.name = body_name
            base_feat.attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))
            base_feat.attributes.add(ATTR_GROUP, ATTR_SIG_KEY, signature)
            if base_feat.bodies.count > 0:
                base_feat.bodies.item(0).attributes.add(ATTR_GROUP, ATTR_CONFIG_KEY, json.dumps(config))

        except:
            if ui:
                ui.messageBox('Belt Operation Failed:\n{}'.format(traceback.format_exc()))


class BeltCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self, is_edit=False):
        super().__init__()
        self.is_edit = is_edit

    def notify(self, args):
        global active_editing_base_feat, cached_selected_entities
        try:
            cached_selected_entities = []
            app = adsk.core.Application.get()
            ui = app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            cmd = adsk.core.CommandCreatedEventArgs.cast(args).command
            inputs = cmd.commandInputs

            if self.is_edit:
                active_editing_base_feat = None

                if ui.activeSelections.count > 0:
                    ent = ui.activeSelections.item(0).entity
                    bf, _ = get_belt_base_feature_and_config(ent)
                    if bf:
                        active_editing_base_feat = bf

                if not active_editing_base_feat and design:
                    for attr in design.findAttributes(ATTR_GROUP, ATTR_CONFIG_KEY):
                        bf = adsk.fusion.BaseFeature.cast(attr.parent)
                        if bf and bf.isValid:
                            active_editing_base_feat = bf
                            break

                default_config = {}
                if active_editing_base_feat:
                    cfg_attr = active_editing_base_feat.attributes.itemByName(ATTR_GROUP, ATTR_CONFIG_KEY)
                    if cfg_attr:
                        default_config = json.loads(cfg_attr.value)

                sel_in = inputs.addSelectionInput('entitySelection', 'Belt Body to Edit', 'Select the existing belt body.')
                sel_in.addSelectionFilter('Bodies')
                sel_in.setSelectionLimits(1, 1)
                if active_editing_base_feat and active_editing_base_feat.bodies.count > 0:
                    body0 = active_editing_base_feat.bodies.item(0)
                    sel_in.addSelection(body0)
                    cached_selected_entities = [body0]

                inputs.addBoolValueInput('autoChain', 'Auto-Chain Connected Curves', True, '', bool(default_config.get('auto_chain', True)))
                profile_dd = inputs.addDropDownCommandInput('beltProfile', 'Belt Profile', adsk.core.DropDownStyles.TextListDropDownStyle)
                saved_prof = default_config.get('profile_name', 'Gates GT2 (2GT)')
                for p_name in BELT_PROFILES_MM.keys():
                    profile_dd.listItems.add(p_name, p_name == saved_prof)

                is_dbl = bool(default_config.get('is_double_sided', False))
                side_dd = inputs.addDropDownCommandInput('toothSidedness', 'Tooth Configuration', adsk.core.DropDownStyles.TextListDropDownStyle)
                side_dd.listItems.add('Single-Sided Teeth', not is_dbl)
                side_dd.listItems.add('Double-Sided Teeth', is_dbl)

                flip_in = inputs.addBoolValueInput('flipSide', 'Flip Tooth Side', True, '', bool(default_config.get('flip_side', False)))
                flip_in.isEnabled = not is_dbl
                inputs.addBoolValueInput('includeRootFillet', 'Include Root Fillet Radius', True, '', bool(default_config.get('include_root_fillet', True)))

                saved_expr = default_config.get('belt_width_expr', '').strip()
                cur_width_cm = evaluate_belt_width_cm(design, default_config)
                val_in = adsk.core.ValueInput.createByString(saved_expr) if saved_expr else adsk.core.ValueInput.createByReal(cur_width_cm)
                w_input = inputs.addValueInput('beltWidth', 'Belt Width (Extrusion)', 'mm', val_in)
                if not w_input.isValidExpression:
                    w_input.value = cur_width_cm

                inputs.addBoolValueInput('symmetricWidth', 'Symmetric Extrusion Width', True, '', bool(default_config.get('symmetric_width', True)))

            else:
                pre_selected_curves = []
                for i in range(ui.activeSelections.count):
                    sc = adsk.fusion.SketchCurve.cast(ui.activeSelections.item(i).entity)
                    if sc and sc.isValid:
                        pre_selected_curves.append(sc)

                sel_in = inputs.addSelectionInput('entitySelection', '2D Sketch Path (Pitch Line)', 'Select a sketch curve.')
                sel_in.addSelectionFilter('SketchCurves')
                sel_in.setSelectionLimits(1, 0)
                for sc in pre_selected_curves:
                    sel_in.addSelection(sc)
                cached_selected_entities = list(pre_selected_curves)

                inputs.addBoolValueInput('autoChain', 'Auto-Chain Connected Curves', True, '', True)
                profile_dd = inputs.addDropDownCommandInput('beltProfile', 'Belt Profile', adsk.core.DropDownStyles.TextListDropDownStyle)
                for p_name in BELT_PROFILES_MM.keys():
                    profile_dd.listItems.add(p_name, p_name == 'Gates GT2 (2GT)')

                side_dd = inputs.addDropDownCommandInput('toothSidedness', 'Tooth Configuration', adsk.core.DropDownStyles.TextListDropDownStyle)
                side_dd.listItems.add('Single-Sided Teeth', True)
                side_dd.listItems.add('Double-Sided Teeth', False)

                inputs.addBoolValueInput('flipSide', 'Flip Tooth Side', True, '', False)
                inputs.addBoolValueInput('includeRootFillet', 'Include Root Fillet Radius', True, '', True)
                inputs.addValueInput('beltWidth', 'Belt Width (Extrusion)', 'mm', adsk.core.ValueInput.createByReal(0.6))
                inputs.addBoolValueInput('symmetricWidth', 'Symmetric Extrusion Width', True, '', True)

            on_preview = BeltCommandPreviewHandler(self.is_edit)
            cmd.executePreview.add(on_preview)
            handlers.append(on_preview)

            on_execute = BeltCommandExecuteHandler(self.is_edit)
            cmd.execute.add(on_execute)
            handlers.append(on_execute)

            on_input_changed = BeltInputChangedHandler(self.is_edit)
            cmd.inputChanged.add(on_input_changed)
            handlers.append(on_input_changed)

            on_destroy = BeltCommandDestroyHandler()
            cmd.destroy.add(on_destroy)
            handlers.append(on_destroy)

            update_visualizer_from_inputs(inputs, self.is_edit)

        except:
            adsk.core.Application.get().userInterface.messageBox('Command Created Error:\n{}'.format(traceback.format_exc()))


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cmd_defs = ui.commandDefinitions

        for old_ev in ('ToothedBeltSyncParamEventId', 'ToothedBeltParamSyncCustomEventId', 'ToothedBeltDeferredBuildEventId'):
            try:
                app.unregisterCustomEvent(old_ev)
            except:
                pass
        old_sync_cmd = cmd_defs.itemById('ToothedBeltParamSyncCmdId')
        if old_sync_cmd:
            try:
                old_sync_cmd.deleteMe()
            except:
                pass

        on_cmd_terminated = BeltWorkspaceWatcherHandler()
        ui.commandTerminated.add(on_cmd_terminated)
        handlers.append(on_cmd_terminated)

        existing_def = cmd_defs.itemById(CMD_ID)
        if existing_def:
            existing_def.deleteMe()
        # Loads icons from resources/BeltGenerator/16x16.png, 32x32.png, 64x64.png
        cmd_def = cmd_defs.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESC, 'resources/BeltGenerator')
        on_cmd_created = BeltCommandCreatedHandler(False)
        cmd_def.commandCreated.add(on_cmd_created)
        handlers.append(on_cmd_created)

        existing_edit_def = cmd_defs.itemById(EDIT_CMD_ID)
        if existing_edit_def:
            existing_edit_def.deleteMe()
        # Loads icons from resources/BeltEdit/16x16.png, 32x32.png, 64x64.png
        edit_cmd_def = cmd_defs.addButtonDefinition(EDIT_CMD_ID, EDIT_CMD_NAME, EDIT_CMD_DESC, 'resources/BeltEdit')
        on_edit_created = BeltCommandCreatedHandler(True)
        edit_cmd_def.commandCreated.add(on_edit_created)
        handlers.append(on_edit_created)

        modeling_ws = ui.workspaces.itemById('FusionSolidEnvironment')
        if modeling_ws:
            panel = modeling_ws.toolbarPanels.itemById(PANEL_ID)
            if panel:
                for cid in (CMD_ID, EDIT_CMD_ID):
                    ctrl = panel.controls.itemById(cid)
                    if ctrl:
                        ctrl.deleteMe()
                ctrl1 = panel.controls.addCommand(cmd_def)
                ctrl1.isPromoted = True
                ctrl2 = panel.controls.addCommand(edit_cmd_def)
                ctrl2.isPromoted = True

    except:
        if ui:
            ui.messageBox('Add-In Start Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        clear_visualizer_graphics(design)

        for old_ev in ('ToothedBeltSyncParamEventId', 'ToothedBeltParamSyncCustomEventId', 'ToothedBeltDeferredBuildEventId'):
            try:
                app.unregisterCustomEvent(old_ev)
            except:
                pass
        modeling_ws = ui.workspaces.itemById('FusionSolidEnvironment')
        if modeling_ws:
            panel = modeling_ws.toolbarPanels.itemById(PANEL_ID)
            if panel:
                for cid in (CMD_ID, EDIT_CMD_ID):
                    ctrl = panel.controls.itemById(cid)
                    if ctrl:
                        ctrl.deleteMe()
        for cid in (CMD_ID, EDIT_CMD_ID, 'ToothedBeltParamSyncCmdId'):
            cd = ui.commandDefinitions.itemById(cid)
            if cd:
                cd.deleteMe()
        handlers.clear()
    except:
        if ui:
            ui.messageBox('Add-In Stop Failed:\n{}'.format(traceback.format_exc()))
# === END OF FILE ===