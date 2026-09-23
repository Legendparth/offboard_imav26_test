"""
Takeoff -> find the tube obstacle -> fit it -> line up on the big gap -> fly
through it -> step left past the back tube -> fly clear -> land. ARK Flow
localisation, ZED as a camera only.

Flown as its own mission, one obstacle at a time, like bar_cross.

    arm -> climb -> hold -> SEARCH (stare ahead for the uprights) -> LOCK
    (stand still, fit the obstacle, choose the gap) -> ALIGN (fly to the
    entry point on the gap centreline at the gap altitude, refining the
    lateral offset while the uprights are still in view) -> PASS (committed,
    blind, through the gap) -> SHIFT (sideways, left by default) -> EXIT
    (straight on, past the back tube) -> CLEAR -> land.

    The gap is the BIG cell of the front structure -- the trapezium under the
    high end of the diagonal -- and the aircraft crosses at its centre of
    area, not at the middle of the two uprights and not at the middle of the
    altitude band. Both of those sit low and off to one side of the opening,
    which is how you end up skimming the diagonal. tube_detect measures the
    cell directly (/tube_hole), and that decides which pair of uprights to fly
    between and at what height; the template is only the fallback.

    q -> abort into a controlled descent.   k -> force-disarm.

THE OBSTACLE (front view, as the aircraft approaches; mm)

        L         M         R           uprights 500 apart, 2000 tall
        |\\        |         |
        | \\       |         |           diagonal: top of L (2000) down to
        |  \\      |         |           R at 922, crossing M at about 1461
        |   \\     |         |
        |    \\    |         |
        | GAP \\   |         |           THE GAP: between L and M, above
        |      \\  |         |           the cross tube, below the diagonal
        |       \\ |         |
        |        \\|         |
        |         |\\        |
        |         |  \\      |
        |         |    \\    |
        |         |      \\  |
        |         |        \\|  922
        |=========|=========|  461      cross tube
        |         |         |
       ---------------------------      floor

    plus one free-standing upright about 1 m BEHIND the plane, off to the
    right. The largest hole is the L-M one: 500 mm between centrelines, less
    a tube, so about 450 mm for a 260 mm airframe -- under 10 cm a side.

WHY IT IS MEASURED AND NOT FLOWN BLIND
    Ten centimetres a side is less than ARK Flow drifts over a few metres, and
    less than the error in where the aircraft was put down. So the uprights
    are measured: each is placed on the ground plane in NED, exactly as
    bar_cross places the bar, points are clustered into tubes, and the known
    layout -- three uprights on a line, tube_spacing apart -- is fitted to
    them. The crossing point is the MEASURED hole centre, clamped to stay
    half_airframe off the two measured uprights either side of it, so neither
    a spacing error in the build nor a bad frame from the camera can put the
    aircraft into a tube.

    Matching three tubes to the template is what decides which tube is the
    middle one. With only two in view the answer is ambiguous (L+M or M+R),
    so min_matched_tubes defaults to 3 and the node waits rather than guess.

    What the template CANNOT decide is which side of the middle tube the big
    cell is on: three evenly spaced uprights look the same either way round,
    so it depends on how the obstacle was built and which way the aircraft
    came at it. /tube_hole answers that from the image. Without it, gap_side
    ('auto' at start-up, resolved from the diagonal in the parameters) decides
    and the crossing is flown blind to which cell is really the big one.

    The back upright is 1 m behind the plane; plane_band keeps it out of the
    fit.

    assume_gap_distance > 0 skips all of this and flies the gap from the
    parameters, bar_cross style. With the margins above, only do that if you
    have put the aircraft down on the gap centreline with a tape measure.

THE ALTITUDE
    Solved for the airframe, across its whole width:

        floor  = cross_bar_height + tube_radius + clearance + body_below
        roof   = diagonal height at the inner edge of the airframe
                 - tube_radius - clearance - body_above

    The diagonal slopes, so its lowest point over the airframe is at the edge
    nearest the middle tube. If floor > roof the gap does not fit this
    aircraft with this clearance and the node refuses at start-up.

    Inside that band the aircraft flies at the centre of AREA of the opening
    -- from /tube_hole when the camera has it, from the template trapezium
    otherwise. With the launch defaults (cam_z -0.04, clearance 0.12): floor
    0.77, roof 1.35, template centre of area 1.11, so 1.11 m.

THE COMMIT
    Like the window traverse, PASS commits: the target is frozen at the entry
    and the camera stops steering, because the uprights leave the field of
    view in the last metre.

    If flow drops out BEFORE the gap, the aircraft lands where it is -- in
    front of the obstacle, straight down is clear. If it drops out at or past
    the plane, it pushes on open-loop for exactly the distance left to the
    pass exit and lands there: descending inside the gap lands on the cross
    tube. Losing flow during SHIFT or EXIT lands immediately.

THE LIDAR OVER THE CROSS TUBE
    The cross tube passes 0.6 m under the ARK Flow rangefinder. It is thin and
    crossed quickly, but it is the same step change in dist_bottom that
    tripped cs_rng_kin_consistent over the red bar (see course_fsm). During
    the tube stages horizontal hold is judged on flow fusion alone, as
    course_fsm does. If the height estimate itself jumps when crossing, the
    thing to look at is EKF2_RNG_K_GATE / EKF2_HGT_REF, not this node.
"""

import collections
import math
import threading
import time

import numpy as np

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.offboard_sequence import OffboardSequence, spin_node, wrap_pi
from drone_testing.tube_detect import HOLE_STRIDE, STRIDE
from drone_testing.window_traverse import quat_rotate, rpy_to_matrix_frd


class TubeEstimator:
    """Camera-frame upright measurements -> clustered ground points in NED.

    Free of ROS and the flight node, like BarEstimator, so the rejection and
    clustering logic can be exercised on the bench.
    """

    def __init__(self, depth_min, depth_max, tube_radius, min_top_height,
                 max_bottom_height, buffer_seconds, buffer_max, cluster_radius,
                 min_samples, min_hole_width=0.30, min_hole_height=0.50,
                 max_hole_floor=None):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.tube_radius = tube_radius
        self.min_top_height = min_top_height
        self.max_bottom_height = max_bottom_height
        self.buffer_seconds = buffer_seconds
        self.cluster_radius = cluster_radius
        self.min_samples = min_samples
        self.min_hole_width = min_hole_width
        self.min_hole_height = min_hole_height
        # Every cell the aircraft may fly SITS ON the cross tube. Sealing the
        # top of the frame so the tall cell is visible at all also makes the
        # space ABOVE the diagonal enclosed, and that space is bounded below
        # by the diagonal rather than by the cross tube -- high, narrowing,
        # and roofed by a top bar nobody has measured. This is where that gets
        # thrown out: a floor well above the cross tube is not a cell of the
        # gate the aircraft goes through.
        self.max_hole_floor = max_hole_floor

        self._lock = threading.RLock()
        self.samples = collections.deque(maxlen=buffer_max)
        self.hole_samples = collections.deque(maxlen=buffer_max)
        self.rejections = {}
        self.accepted_total = 0
        self.holes_total = 0

    def add(self, row, q_att, p_ned, r_cam, t_cam, home_z, now):
        with self._lock:
            return self._add(row, q_att, p_ned, r_cam, t_cam, home_z, now)

    def _to_ned(self, depth, az_deg, el_deg, q_att, p_ned, r_cam, t_cam):
        az = math.radians(float(az_deg))
        el = math.radians(float(el_deg))
        cam = np.array([depth, depth * math.tan(az), -depth * math.tan(el)])
        body = r_cam @ cam + t_cam
        return np.asarray(quat_rotate(q_att, body)) + p_ned

    def _add(self, row, q_att, p_ned, r_cam, t_cam, home_z, now):
        depth, az, el, el_top, el_bot, trunc_top, trunc_bot, _ = row
        if not np.isfinite(depth) or not (self.depth_min <= depth <= self.depth_max):
            return self._reject('depth out of range')

        point = self._to_ned(depth, az, el, q_att, p_ned, r_cam, t_cam)
        top = self._to_ned(depth, az, el_top, q_att, p_ned, r_cam, t_cam)
        bottom = self._to_ned(depth, az, el_bot, q_att, p_ned, r_cam, t_cam)

        # Heights above the arming plane, positive up. An upright stands on
        # the floor and reaches up past the aircraft; a cut-off end only
        # tells us it goes at least that far.
        top_h = home_z - top[2]
        bottom_h = home_z - bottom[2]
        if top_h < self.min_top_height and trunc_top < 0.5:
            return self._reject('too short to be an upright')
        if bottom_h > self.max_bottom_height and trunc_bot < 0.5:
            return self._reject('does not reach the floor')

        # Depth is to the near SURFACE; the centreline is one radius further
        # along the horizontal ray.
        ray = point[:2] - p_ned[:2]
        norm = float(np.linalg.norm(ray))
        if norm < 1e-6:
            return self._reject('degenerate ray')
        xy = point[:2] + ray / norm * self.tube_radius

        self.accepted_total += 1
        self.samples.append({'t': now, 'xy': xy})
        return True, ''

    def add_hole(self, row, q_att, p_ned, r_cam, t_cam, home_z, now):
        """One /tube_hole row -> the cell, placed in NED.

        The depth is the depth of the UPRIGHTS: the ray through the centroid
        goes through the hole and out the other side, so there is nothing
        there for the camera to range on. The point wanted is where that ray
        crosses the plane of the obstacle, which is exactly what this gives.

        What comes back with it is the CEILING and the FLOOR of the cell over
        that same ray -- the diagonal and the cross tube where the aircraft
        will pass them, measured rather than taken from the template. Nothing
        else in the node knows which way the diagonal really slopes.
        """
        with self._lock:
            (az, el, az_l, az_r, el_top, el_bot, el_top_l, el_top_r,
             depth, _, sides, roof_cut) = row
            if not np.isfinite(depth) or not (self.depth_min <= depth <= self.depth_max):
                return self._reject('hole depth out of range')
            p = self._to_ned(depth, az, el, q_att, p_ned, r_cam, t_cam)
            height = home_z - p[2]
            if not (self.max_bottom_height <= height <= self.min_top_height + 1.0):
                return self._reject(f'hole centre at {height:.2f} m is implausible')

            width = depth * (math.tan(math.radians(az_r)) - math.tan(math.radians(az_l)))
            if width < self.min_hole_width:
                # Half a cell, most likely: something was standing in it.
                return self._reject(f'hole only {width:.2f} m wide')
            ceiling = home_z - self._to_ned(depth, az, el_top, q_att, p_ned,
                                            r_cam, t_cam)[2]
            floor = home_z - self._to_ned(depth, az, el_bot, q_att, p_ned,
                                          r_cam, t_cam)[2]
            if ceiling - floor < self.min_hole_height:
                return self._reject(f'hole only {ceiling - floor:.2f} m tall')
            if self.max_hole_floor is not None and floor > self.max_hole_floor:
                return self._reject(
                    f'hole floor at {floor:.2f} m is above the cross tube '
                    f'(max {self.max_hole_floor:.2f} m) -- above the diagonal, '
                    'not a cell of the gate')

            self.holes_total += 1
            # Which way the roof slopes, in metres of height per metre of
            # lateral, + meaning it rises towards the aircraft's LEFT. This is
            # the sign the template gets wrong.
            shoulder = lambda e: home_z - self._to_ned(depth, az, e, q_att,
                                                       p_ned, r_cam, t_cam)[2]
            span = depth * (math.tan(math.radians(az_r))
                            - math.tan(math.radians(az_l))) * 0.5
            rise = ((shoulder(el_top_l) - shoulder(el_top_r)) / span
                    if span > 1e-3 else 0.0)
            self.hole_samples.append({'t': now, 'xy': p[:2].copy(), 'h': height,
                                      'ceiling': ceiling, 'floor': floor,
                                      'width': width, 'sides': sides,
                                      'rise': rise, 'cut': float(roof_cut)})
            return True, ''

    def holes(self, now, buffer_seconds=None):
        """Every cell seen lately, biggest OPENING first.

        Clustered by where they are in NED, not averaged together. During a
        yaw scan the camera sees both cells of the obstacle, and sometimes a
        cell of something else entirely; taking the median of all of it puts
        the answer in the tube between them. Each cluster is one cell, and
        what ranks them is the measured area of the opening -- width times
        height, in metres, at the place they actually are.

        [{xy, height, ceiling, floor, width, rise, area, count, cut}, ...]

        cut  the roof of this cell was above the top of the frame, so its
             ceiling -- and therefore its area -- is a LOWER bound. It is the
             tall cell of the obstacle seen from close in, and it still
             outranks the short one on the clipped numbers.
        """
        with self._lock:
            cutoff = now - (self.buffer_seconds if buffer_seconds is None
                            else buffer_seconds)
            fresh = [s for s in self.hole_samples if s['t'] >= cutoff]
        groups = []
        for s in fresh:
            for g in groups:
                if float(np.linalg.norm(s['xy'] - g['mean'])) <= self.cluster_radius:
                    g['members'].append(s)
                    g['mean'] = np.mean([m['xy'] for m in g['members']], axis=0)
                    break
            else:
                groups.append({'members': [s], 'mean': s['xy'].copy()})

        out = []
        for g in groups:
            members = g['members']
            if len(members) < self.min_samples:
                continue
            med = lambda key: float(np.median([m[key] for m in members]))
            cell = {'xy': np.median(np.array([m['xy'] for m in members]), axis=0),
                    'height': med('h'), 'ceiling': med('ceiling'),
                    'floor': med('floor'), 'width': med('width'),
                    'rise': med('rise'), 'count': len(members),
                    'cut': med('cut') >= 0.5}
            cell['area'] = cell['width'] * max(0.0, cell['ceiling'] - cell['floor'])
            out.append(cell)
        out.sort(key=lambda c: -c['area'])
        return out

    def hole(self, now):
        """The biggest cell as measured, in the old tuple form, or None."""
        found = self.holes(now)
        if not found:
            return None
        c = found[0]
        return (c['xy'], c['height'], c['ceiling'], c['floor'], c['width'],
                c['rise'], c['count'])

    def _reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return False, reason

    def clusters(self, now):
        """[(xy, count)] for every upright seen often enough, most-seen first."""
        with self._lock:
            cutoff = now - self.buffer_seconds
            pts = [s['xy'] for s in self.samples if s['t'] >= cutoff]
        groups = []
        for p in pts:
            for g in groups:
                if float(np.linalg.norm(p - g['mean'])) <= self.cluster_radius:
                    g['pts'].append(p)
                    g['mean'] = np.mean(g['pts'], axis=0)
                    break
            else:
                groups.append({'pts': [p], 'mean': p.copy()})
        out = [(np.median(np.array(g['pts']), axis=0), len(g['pts']))
               for g in groups if len(g['pts']) >= self.min_samples]
        out.sort(key=lambda c: -c[1])
        return out

    def rotate(self, pivot, delta):
        c, s = math.cos(delta), math.sin(delta)
        with self._lock:
            for sample in list(self.samples) + list(self.hole_samples):
                dx = sample['xy'][0] - pivot[0]
                dy = sample['xy'][1] - pivot[1]
                sample['xy'] = np.array([pivot[0] + c * dx - s * dy,
                                         pivot[1] + s * dx + c * dy])

    def rejection_summary(self, limit=3):
        with self._lock:
            if not self.rejections:
                return 'none'
            worst = sorted(self.rejections.items(), key=lambda kv: -kv[1])[:limit]
        return ', '.join(f"{name} x{count}" for name, count in worst)


def front_plane(clusters, vehicle_xy, heading, plane_band, max_plane_yaw):
    """The plane the nearest uprights stand on, and what is behind it.

    This is the part of the fit that does NOT need to know the layout: which
    uprights are in the front plane, which way that plane faces, and what is
    standing behind it. Pulled out of solve_gap because the crossing can be
    flown from a measured opening alone, and when the layout will not fit --
    two uprights in view instead of three, or an obstacle that simply is not
    three evenly spaced tubes -- this is still everything the aircraft needs.

    Returns (plane, reason). plane is a dict with 'front' (the xy of the
    uprights in it), 'u' (unit, along the plane, + to the aircraft's LEFT),
    'normal' (unit, through the obstacle), 'heading', 'along' (m from the
    aircraft to the plane) and 'behind' [(along, xy), ...] sorted near first.
    """
    v = np.asarray(vehicle_xy, dtype=float)
    fwd = np.array([math.cos(heading), math.sin(heading)])
    left = np.array([math.sin(heading), -math.cos(heading)])

    ahead = []
    for xy, count in clusters:
        along = float(np.dot(xy - v, fwd))
        if along > 0.3:
            ahead.append((xy, along))
    if not ahead:
        return None, 'no uprights ahead'

    nearest = min(a for _, a in ahead)
    front = np.array([xy for xy, a in ahead if a <= nearest + plane_band])
    behind = sorted(((a, xy) for xy, a in ahead if a > nearest + plane_band),
                    key=lambda t: t[0])

    # The line through the front uprights. With one tube there is no line, so
    # the course heading stands in for it.
    u = left.copy()
    if len(front) >= 2:
        centred = front - front.mean(axis=0)
        _, _, vt = np.linalg.svd(centred, full_matrices=False)
        u = vt[0] / np.linalg.norm(vt[0])
        if float(np.dot(u, left)) < 0.0:
            u = -u
        if math.acos(min(1.0, abs(float(np.dot(u, left))))) > max_plane_yaw:
            return None, 'tube plane is too far off square to the heading'
    normal = np.array([-u[1], u[0]])
    if float(np.dot(normal, fwd)) < 0.0:
        normal = -normal

    return {'front': front, 'u': u, 'normal': normal,
            'heading': math.atan2(normal[1], normal[0]),
            'along': float(np.mean([np.dot(p - v, normal) for p in front])),
            'behind': behind}, ''


def structure_centre(plane, vehicle_xy, u, lateral):
    """Where the MIDDLE of the obstacle is, relative to the crossing point.

    + is the aircraft's left, as everywhere else. The midpoint of the
    outermost uprights of the front plane, which is the one thing that says
    which side of the structure the aircraft is actually crossing on -- the
    cell it flew is not the cell the template picked, and anything that is
    placed off gap_side alone (the upright behind, and the sideways step that
    dodges it) ends up on the wrong side when those two disagree.

    None when there are not two uprights to take a midpoint of.
    """
    front = plane.get('front')
    if front is None or len(front) < 2:
        return None
    across = [float(np.dot(p - np.asarray(vehicle_xy), u)) for p in front]
    return 0.5 * (min(across) + max(across)) - lateral


def solve_from_hole(plane, vehicle_xy, cell_xy, cell_width, half_airframe,
                    lean=0.0):
    """A crossing built from a MEASURED opening, with no layout fit at all.

    The opening is the thing the aircraft has to fit through, and the camera
    has measured where it is, how wide it is and what its roof and floor are
    doing. Three evenly spaced uprights are a way of guessing at that from a
    drawing; where the drawing does not match what is really there -- a gate
    with two posts instead of three, a build that is not to spec, a third
    upright out of frame -- the opening is still right and the drawing is
    still wrong.

    The lateral clamp is the opening's OWN measured half-width less half an
    airframe, so it does not matter that there is no upright position to
    clamp against.
    """
    v = np.asarray(vehicle_xy, dtype=float)
    u = plane['u']
    centre = float(np.dot(np.asarray(cell_xy) - v, u))
    free = max(0.0, 0.5 * cell_width - half_airframe)
    lateral = min(max(centre + lean, centre - free), centre + free)
    return {
        'point': v + u * lateral + plane['normal'] * plane['along'],
        'centre_offset': structure_centre(plane, v, u, lateral),
        'normal': plane['normal'],
        'left': u,
        'heading': plane['heading'],
        'width': cell_width,
        'offset': lateral - centre,
        'side': 'left' if centre >= 0.0 else 'right',
        'matched': 0,
        'residual': 0.0,
        'source': 'hole',
        'back': (None if not plane['behind'] else
                 {'along': plane['behind'][0][0] - plane['along'],
                  'lateral': float(np.dot(plane['behind'][0][1] - v, u)) - lateral}),
    }


def solve_gap(clusters, vehicle_xy, heading, spacing, plane_band, match_tol,
              min_matched, max_plane_yaw, gap_side, prefer_lateral=None,
              lateral_offset=0.0, half_airframe=0.0):
    """Fit the three-upright layout to tube clusters and return the gap.

    Returns (solution, reason). solution is a dict with 'point' (the crossing
    point, NED xy on the tube plane), 'normal' (unit, pointing through the
    obstacle), 'left' (unit, the aircraft's left when flying along normal),
    'heading', 'width' (measured between the gap's two uprights), 'matched',
    and 'back' -- the upright behind the obstacle as {'along', 'lateral'}
    relative to the crossing point, or None if it was not in view.

    prefer_lateral is where the camera says the middle of the big cell is, as
    a lateral coordinate in the same frame as the clusters (+ LEFT of the
    aircraft). Given one, it picks WHICH pair of uprights to fly between --
    which is the only reliable way to tell the big cell from the small one,
    since the layout of three evenly spaced tubes is the same either way round
    -- and aims at it rather than at the midpoint, held to within
    half_airframe of the uprights either side. Without one, gap_side decides
    the pair and the crossing point is their midpoint plus lateral_offset --
    where the template says the centre of area of that cell is.
    """
    v = np.asarray(vehicle_xy, dtype=float)
    plane, why = front_plane(clusters, vehicle_xy, heading, plane_band, max_plane_yaw)
    if plane is None:
        return None, why
    front, u, normal, behind = (plane['front'], plane['u'], plane['normal'],
                                plane['behind'])

    lat = [float(np.dot(p - v, u)) for p in front]
    slots = (1, 0, -1)        # left, middle, right: + is LEFT

    best = None
    for li in lat:
        for k in slots:
            m = li - k * spacing
            used = {}
            residual = 0.0
            for lj in lat:
                errs = [(abs(lj - (m + s * spacing)), s) for s in slots]
                err, s = min(errs)
                if err <= match_tol and (s not in used or err < used[s][0]):
                    used[s] = (err, lj)
            residual = sum(e for e, _ in used.values())
            key = (len(used), -residual)
            if best is None or key > best[0]:
                best = (key, m, used)
            elif key[0] == best[0][0] and abs(m - best[1]) > 0.5 * spacing:
                # Equally good fit with a different middle tube: ambiguous,
                # unless something later beats both.
                best = (key, m, used, 'ambiguous')

    if best is None:
        return None, 'no layout fit'
    if len(best) == 4:
        return None, 'two layouts fit equally well (which tube is the middle?)'
    _, m, used = best
    if len(used) < min_matched:
        return None, f'only {len(used)}/{min_matched} uprights fit the layout'

    pairs = [p for p in ((1, 0), (0, -1)) if p[0] in used and p[1] in used]
    if not pairs:
        return None, 'the uprights either side of the gap are not both matched'
    if prefer_lateral is None:
        want = (1, 0) if gap_side == 'left' else (0, -1)
        if want not in pairs:
            return None, 'the uprights either side of the gap are not both matched'
        pair = want
    else:
        pair = min(pairs, key=lambda p: abs(
            0.5 * (used[p[0]][1] + used[p[1]][1]) - prefer_lateral))
    l_a, l_b = used[pair[0]][1], used[pair[1]][1]

    mid = 0.5 * (l_a + l_b)
    free = max(0.0, 0.5 * abs(l_a - l_b) - half_airframe)
    want = mid + lateral_offset if prefer_lateral is None else prefer_lateral
    lateral = min(max(want, mid - free), mid + free)
    along = plane['along']
    point = v + u * lateral + normal * along

    # The free-standing upright behind the obstacle, if it is in the buffer.
    # Measuring it is what stops the run ending in a hover in front of it:
    # its distance behind the plane is a build number nobody has checked, and
    # its lateral is the difference between stepping clear of it and stepping
    # into it.
    back = None
    if behind:
        back_along, back_xy = behind[0]
        back = {'along': back_along - along,
                'lateral': float(np.dot(back_xy - v, u)) - lateral}
    return {
        'point': point,
        'centre_offset': structure_centre(
            {'front': front}, v, u, lateral),
        'source': 'layout',
        'back': back,
        'side': 'left' if pair == (1, 0) else 'right',
        'offset': lateral - mid,
        'normal': normal,
        'left': u,
        'heading': math.atan2(normal[1], normal[0]),
        'width': abs(l_a - l_b),
        'middle_lateral': m,
        'matched': len(used),
        'residual': sum(e for e, _ in used.values()),
    }, ''


class TubeCross(OffboardSequence):

    SEARCH = "SEARCH"
    LOCK = "LOCK"
    ALIGN = "ALIGN"
    PASS = "PASS"
    SHIFT = "SHIFT"
    EXIT = "EXIT"
    CLEAR = "CLEAR"

    TUBE_STAGES = (SEARCH, LOCK, ALIGN, PASS, SHIFT, EXIT, CLEAR)
    FLOW_ONLY_STAGES = (PASS, SHIFT, EXIT, CLEAR)

    # ---- the obstacle (rules drawing, metres) -----------------------------
    TUBE_SPACING = 0.50
    TUBE_RADIUS = 0.025
    CROSS_BAR_HEIGHT = 0.461
    DIAGONAL_LEFT_HEIGHT = 2.000    # where the diagonal meets the LEFT upright
    DIAGONAL_RIGHT_HEIGHT = 0.922   # ... and the RIGHT one
    GAP_SIDE = 'auto'               # auto = the cell with the bigger opening,
                                    # which is the one under the high end of
                                    # the diagonal. 'left'/'right' force it.

    # ---- the path ---------------------------------------------------------
    STANDOFF_DISTANCE = 1.20    # m before the plane the pass starts from
    PASS_EXIT_DISTANCE = 0.50   # m past the plane before stepping sideways
    SHIFT_LEFT = 0.40           # m, + = left. The SMALLEST step sideways
                                # after the gap; where the back upright was
                                # measured, whatever it takes to clear it.
    BACK_TUBE_DISTANCE = 1.00   # m the back upright stands behind the plane
    BACK_TUBE_CLEAR = 1.00      # m to be past it before the run is over
    MAX_SHIFT = 1.20            # m sideways after the gap before going the
                                # other way round the back upright instead
    EXIT_DISTANCE = 0.0         # m from the shift point; 0 = work it out from
                                # back_tube_distance + back_tube_clear
    CROSS_DROP = 0.15           # m BELOW the centre of the opening to aim.
                                # The roof of the cell is the diagonal and the
                                # floor is a single horizontal tube: dropping
                                # buys headroom against the thing that is
                                # actually in the way. Clamped off the floor.
    CROSS_LEFT = 0.05           # m LEFT of the centre of the opening to aim,
                                # towards the high end of the diagonal and
                                # towards the side the aircraft leaves on.
                                # Clamped off the uprights.
    MERGE_SHIFT = True          # fly the gap and the step round the back
                                # upright as ONE diagonal leg
    CLEARANCE = 0.12            # m wanted between airframe and tube, vertically
    LATERAL_MARGIN = 0.02       # m kept between airframe and upright when the
                                # camera's hole centre is followed sideways

    # ---- the airframe (same numbers as window_traverse / bar_cross) -------
    GEAR_BELOW_CAMERA = 0.120
    DRONE_HEIGHT = 0.260
    DRONE_WIDTH = 0.260

    APPROACH_SPEED = 0.30
    PASS_SPEED = 0.30
    SHIFT_SPEED = 0.25

    ALIGN_CROSS_TOLERANCE = 0.05    # m off the gap centreline. Tight on
                                    # purpose: there is under 10 cm a side.
    ALIGN_ALONG_TOLERANCE = 0.15
    ALIGN_YAW_TOLERANCE = math.radians(5.0)
    ALT_TOLERANCE = 0.06
    SETTLE_SECONDS = 1.5
    ARRIVE_TOLERANCE = 0.10
    CROSS_CLEAR = 0.30          # m off the exit line that still counts as
                                # having gone round the back upright
    ARRIVE_SETTLE_SECONDS = 0.5
    REFINE_MIN_DISTANCE = 1.00      # m to the plane below which the camera
                                    # stops steering the entry point
    REFINE_MAX_JUMP = 0.20          # m a refinement may move the gap by
    MIN_HOLE_WIDTH = 0.30       # m. Under this it is not a whole cell --
                                # something was standing in front of it.
    MIN_HOLE_HEIGHT = 0.50      # m, measured over the aircraft's own track
    MIN_GAP_WIDTH = 0.40
    MAX_GAP_WIDTH = 0.60

    SEARCH_TIMEOUT = 45.0
    LOCK_SECONDS = 2.0
    LOCK_TIMEOUT = 20.0
    ALIGN_TIMEOUT = 40.0
    PASS_TIMEOUT = 20.0
    MOVE_STAGE_TIMEOUT = 20.0
    CLEAR_SECONDS = 2.0
    SOLUTION_LOST_TIMEOUT = 6.0

    YAW_CONE_DEG = 30.0

    # ---- the estimate -----------------------------------------------------
    GEOMETRY_TOPIC = 'tube_geometry'
    DETECT_TOPIC = 'tubes_detected'
    HOLE_TOPIC = 'tube_hole'
    ATTITUDE_MAX_HZ = 30.0
    DEPTH_MIN = 0.40
    DEPTH_MAX = 6.00
    MIN_TOP_HEIGHT = 1.20
    MAX_BOTTOM_HEIGHT = 0.40
    BUFFER_SECONDS = 2.5
    BUFFER_MAX = 400
    CLUSTER_RADIUS = 0.15
    MIN_SAMPLES = 6
    PLANE_BAND = 0.40
    MATCH_TOLERANCE = 0.12
    MIN_MATCHED_TUBES = 3
    MAX_PLANE_YAW_DEG = 30.0

    TAKEOFF_ALTITUDE = 1.00
    MAX_ALTITUDE = 2.00
    FLIGHT_SECONDS = 150.0

    def __init__(self, node_name='tube_cross'):
        super().__init__(node_name)
        n = self._declare_number

        self.TUBE_SPACING = float(n('tube_spacing', self.TUBE_SPACING))
        self.TUBE_RADIUS = float(n('tube_radius', self.TUBE_RADIUS))
        self.CROSS_BAR_HEIGHT = float(n('cross_bar_height', self.CROSS_BAR_HEIGHT))
        self.DIAGONAL_LEFT_HEIGHT = float(n('diagonal_left_height', self.DIAGONAL_LEFT_HEIGHT))
        self.DIAGONAL_RIGHT_HEIGHT = float(n('diagonal_right_height', self.DIAGONAL_RIGHT_HEIGHT))
        side = str(self.declare_parameter('gap_side', self.GAP_SIDE).value).strip().lower()
        self.gap_side = side if side in ('left', 'right', 'auto') else self.GAP_SIDE

        self.STANDOFF_DISTANCE = float(n('standoff_distance', self.STANDOFF_DISTANCE))
        self.PASS_EXIT_DISTANCE = float(n('pass_exit_distance', self.PASS_EXIT_DISTANCE))
        self.SHIFT_LEFT = float(n('shift_left', self.SHIFT_LEFT))
        self.BACK_TUBE_DISTANCE = float(n('back_tube_distance', self.BACK_TUBE_DISTANCE))
        self.BACK_TUBE_CLEAR = float(n('back_tube_clear', self.BACK_TUBE_CLEAR))
        self.MAX_SHIFT = float(n('max_shift', self.MAX_SHIFT))
        self.ALLOW_SHIFT_RIGHT = bool(self.declare_parameter(
            'allow_shift_right', False).value)
        self.EXIT_DISTANCE = float(n('exit_distance', self.EXIT_DISTANCE))
        self.CROSS_DROP = float(n('cross_drop', self.CROSS_DROP))
        self.CROSS_LEFT = float(n('cross_left', self.CROSS_LEFT))
        self.MERGE_SHIFT = bool(self.declare_parameter(
            'merge_shift', self.MERGE_SHIFT).value)
        self.CLEARANCE = float(n('clearance', self.CLEARANCE))
        self.LATERAL_MARGIN = float(n('lateral_margin', self.LATERAL_MARGIN))
        self.cross_altitude_override = float(n('cross_altitude', 0.0))

        self.GEAR_BELOW_CAMERA = float(n('gear_below_camera', self.GEAR_BELOW_CAMERA))
        self.DRONE_HEIGHT = float(n('drone_height', self.DRONE_HEIGHT))
        self.DRONE_WIDTH = float(n('drone_width', self.DRONE_WIDTH))

        self.APPROACH_SPEED = float(n('approach_speed', self.APPROACH_SPEED))
        self.PASS_SPEED = float(n('pass_speed', self.PASS_SPEED))
        self.SHIFT_SPEED = float(n('shift_speed', self.SHIFT_SPEED))
        self.ALIGN_CROSS_TOLERANCE = float(n('align_cross_tolerance', self.ALIGN_CROSS_TOLERANCE))
        self.ALIGN_ALONG_TOLERANCE = float(n('align_along_tolerance', self.ALIGN_ALONG_TOLERANCE))
        self.ALIGN_YAW_TOLERANCE = math.radians(float(n(
            'align_yaw_tolerance_deg', math.degrees(self.ALIGN_YAW_TOLERANCE))))
        self.ALT_TOLERANCE = float(n('alt_tolerance', self.ALT_TOLERANCE))
        self.SETTLE_SECONDS = float(n('settle_seconds', self.SETTLE_SECONDS))
        self.REFINE_MIN_DISTANCE = float(n('refine_min_distance', self.REFINE_MIN_DISTANCE))
        self.REFINE_MAX_JUMP = float(n('refine_max_jump', self.REFINE_MAX_JUMP))
        self.SEARCH_TIMEOUT = float(n('search_timeout', self.SEARCH_TIMEOUT))
        self.LOCK_SECONDS = float(n('lock_seconds', self.LOCK_SECONDS))
        self.FLIGHT_SECONDS = float(n('flight_seconds', self.FLIGHT_SECONDS))
        self.YAW_CONE = math.radians(float(n('yaw_cone_deg', self.YAW_CONE_DEG)))

        self.PLANE_BAND = float(n('plane_band', self.PLANE_BAND))
        self.MATCH_TOLERANCE = float(n('match_tolerance', self.MATCH_TOLERANCE))
        self.MIN_MATCHED_TUBES = int(n('min_matched_tubes', self.MIN_MATCHED_TUBES))
        self.MAX_PLANE_YAW = math.radians(float(n('max_plane_yaw_deg', self.MAX_PLANE_YAW_DEG)))

        # Blind: gap assumed this far ahead of where the aircraft is at the
        # end of the hold, offset sideways by assume_gap_left.
        self.ASSUME_GAP_DISTANCE = float(n('assume_gap_distance', 0.0))
        self.ASSUME_GAP_LEFT = float(n('assume_gap_left', 0.0))
        self.flying_blind = self.ASSUME_GAP_DISTANCE > 0.0

        cam_x = float(n('cam_x', 0.0))
        cam_y = float(n('cam_y', 0.0))
        cam_z = float(n('cam_z', 0.0))
        self.r_cam = rpy_to_matrix_frd(float(n('cam_roll', 0.0)),
                                       float(n('cam_pitch', 0.0)),
                                       float(n('cam_yaw', 0.0)))
        self.t_cam = np.array([cam_x, -cam_y, -cam_z])
        self.body_below = self.GEAR_BELOW_CAMERA - cam_z
        self.body_above = self.DRONE_HEIGHT - self.body_below

        self.estimator = TubeEstimator(
            depth_min=float(n('depth_min', self.DEPTH_MIN)),
            depth_max=float(n('depth_max', self.DEPTH_MAX)),
            tube_radius=self.TUBE_RADIUS,
            min_top_height=float(n('min_tube_top_height', self.MIN_TOP_HEIGHT)),
            max_bottom_height=float(n('max_tube_bottom_height', self.MAX_BOTTOM_HEIGHT)),
            buffer_seconds=float(n('buffer_seconds', self.BUFFER_SECONDS)),
            buffer_max=self.BUFFER_MAX,
            cluster_radius=float(n('cluster_radius', self.CLUSTER_RADIUS)),
            min_samples=int(n('pose_min_samples', self.MIN_SAMPLES)),
            min_hole_width=float(n('min_hole_width', self.MIN_HOLE_WIDTH)),
            min_hole_height=float(n('min_hole_height', self.MIN_HOLE_HEIGHT)),
            max_hole_floor=float(n('max_hole_floor',
                                   self._default_hole_floor())),
        )

        self.cross_altitude, self.gap_floor, self.gap_roof = self._solve_altitude()
        self.problems = self._check_geometry()
        for problem in self.problems:
            self.get_logger().error(f"TUBES NOT FLYABLE: {problem}")

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.attitude = None
        self.attitude_time = None
        self._attitude_min_interval = 1.0 / self.ATTITUDE_MAX_HZ
        self.create_subscription(VehicleAttitude, '/uav_1/fmu/out/vehicle_attitude',
                                 self.attitude_callback, sensor_qos,
                                 callback_group=self.sensor_cbg)

        self.geometry_topic = str(self.declare_parameter(
            'geometry_topic', self.GEOMETRY_TOPIC).value)
        detect_topic = str(self.declare_parameter('detect_topic', self.DETECT_TOPIC).value)
        self.create_subscription(Float32MultiArray, self.geometry_topic,
                                 self.geometry_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Bool, detect_topic, self.detected_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.hole_topic = str(self.declare_parameter('hole_topic', self.HOLE_TOPIC).value)
        self.create_subscription(Float32MultiArray, self.hole_topic,
                                 self.hole_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.gap_pub = self.create_publisher(String, 'tube_gap', 10)

        self.tubes_flag = False
        self.geometry_seen = 0
        self.solution = None
        self.solution_reason = 'no data yet'
        self.solution_ok_since = None
        self.solution_lost_since = None

        self.hole_hint = None
        self.back_along = self.BACK_TUBE_DISTANCE
        self.back_lateral = 0.0
        self.shift = self.SHIFT_LEFT
        self.gap_point = None
        self.gap_normal = None
        self.gap_left = None
        self.gap_heading = None
        self.entry = None
        self.pass_exit = None
        self.shift_point = None
        self.final_point = None
        self.stage_target = None
        self.settle_since = None
        self.push_since = None
        self.push_seconds = 0.0
        self.flight_start = None
        self.outcome = 'not attempted'

        self.get_logger().warning(
            f"Tube crossing on ARK FLOW{', FLYING BLIND' if self.flying_blind else ''}: "
            f"through the {self.gap_side.upper()} gap at {self.cross_altitude:.2f} m "
        f"(aiming {self.CROSS_DROP:.2f} m under and {self.CROSS_LEFT:.2f} m "
        f"left of the middle of the opening) "
            f"(fits {self.gap_floor:.2f}-{self.gap_roof:.2f} m), then "
            f"at least {abs(self.SHIFT_LEFT):.2f} m left of it and on past "
            f"the back upright. Point the aircraft at the "
            "obstacle before you arm. Press q to abort, k to force-disarm. "
            + ("READY." if not self.problems else "WILL NOT ATTEMPT -- see errors."))

    # ------------------------------------------------------------ geometry

    def _default_hole_floor(self):
        """The highest a flyable cell's FLOOR can be: halfway between the
        cross tube and the LOW end of the diagonal.

        Every cell the aircraft may cross rests on the cross tube. The cells
        ABOVE the diagonal -- which exist as soon as the top of the frame is
        sealed so the tall cell can be seen at all -- rest on the diagonal,
        and the lowest the diagonal ever gets is its low end. Halfway between
        the two separates them on this obstacle and on a scaled copy of it
        alike, with no number to keep in step by hand.
        """
        low = min(self.DIAGONAL_LEFT_HEIGHT, self.DIAGONAL_RIGHT_HEIGHT)
        return 0.5 * (self.CROSS_BAR_HEIGHT + max(low, self.CROSS_BAR_HEIGHT))

    def _diagonal_height(self, lateral):
        """Diagonal height at `lateral` m LEFT of the middle upright."""
        mid = 0.5 * (self.DIAGONAL_LEFT_HEIGHT + self.DIAGONAL_RIGHT_HEIGHT)
        slope = ((self.DIAGONAL_LEFT_HEIGHT - self.DIAGONAL_RIGHT_HEIGHT)
                 / (2.0 * self.TUBE_SPACING))
        return mid + slope * lateral

    def _cell(self, side):
        """(area, lateral, height) of the open cell on one side of the middle.

        Columns across the cell: the floor is the cross tube, the roof is the
        sloping diagonal, and a tube radius comes off every edge. The lateral
        and height returned are its centre of AREA -- the middle of the
        trapezium, not the middle of the two uprights, which sits well off to
        the low side of it.
        """
        sign = 1.0 if side == 'left' else -1.0
        lo, hi = sorted((self.TUBE_RADIUS * sign, sign * (self.TUBE_SPACING - self.TUBE_RADIUS)))
        bottom = self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS
        steps = 60
        dx = (hi - lo) / steps
        area = lat = hgt = 0.0
        for i in range(steps):
            x = lo + (i + 0.5) * dx
            top = self._diagonal_height(x) - self.TUBE_RADIUS
            column = max(0.0, top - bottom) * dx
            area += column
            lat += x * column
            hgt += 0.5 * (top + bottom) * column
        if area <= 0.0:
            return 0.0, sign * 0.5 * self.TUBE_SPACING, bottom
        return area, lat / area, hgt / area

    def _solve_altitude(self):
        """The default crossing height: the centre of area of the big cell.

        Held inside the band the airframe actually fits in. The band is worked
        out across the whole width of the aircraft, and the diagonal slopes,
        so the binding point is the airframe edge nearest the middle upright.
        """
        if self.gap_side == 'auto':
            self.gap_side = max(('left', 'right'), key=lambda c: self._cell(c)[0])
            self.get_logger().info(
                f"gap_side=auto -> the {self.gap_side.upper()} cell is the "
                f"bigger opening ({self._cell(self.gap_side)[0]:.2f} m2).")
        _, self.cell_lateral, self.cell_height = self._cell(self.gap_side)
        edges = (self.cell_lateral - 0.5 * self.DRONE_WIDTH,
                 self.cell_lateral + 0.5 * self.DRONE_WIDTH)
        roof_tube = min(self._diagonal_height(e) for e in edges)
        floor = (self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS + self.CLEARANCE
                 + self.body_below)
        roof = roof_tube - self.TUBE_RADIUS - self.CLEARANCE - self.body_above
        altitude = min(max(self.cell_height - self.CROSS_DROP, floor), roof)
        if self.cross_altitude_override > 0.0:
            altitude = self.cross_altitude_override
        return altitude, floor, roof

    def _check_geometry(self):
        problems = []
        if self.gap_floor > self.gap_roof:
            problems.append(
                f"the gap leaves no altitude band with {self.CLEARANCE:.2f} m "
                f"clearance (floor {self.gap_floor:.2f} > roof {self.gap_roof:.2f})")
        elif not (self.gap_floor <= self.cross_altitude <= self.gap_roof):
            problems.append(
                f"cross_altitude {self.cross_altitude:.2f} m is outside the "
                f"band {self.gap_floor:.2f}-{self.gap_roof:.2f} m")
        side = 0.5 * (self.TUBE_SPACING - 2.0 * self.TUBE_RADIUS - self.DRONE_WIDTH)
        if side < 0.05:
            problems.append(f"only {side:.3f} m a side between airframe and uprights")
        if not (self.MIN_ALTITUDE <= self.cross_altitude <= self.MAX_ALTITUDE):
            problems.append(
                f"cross altitude {self.cross_altitude:.2f} m outside "
                f"[{self.MIN_ALTITUDE:.2f}, {self.MAX_ALTITUDE:.2f}]")
        if self.cross_altitude - self.body_below < self.FLOW_MIN_AGL + 0.2:
            problems.append("cross altitude is too close to the optical-flow floor")
        return problems

    # ------------------------------------------------------------------ subs

    def attitude_callback(self, msg):
        now = time.monotonic()
        if (self.attitude_time is not None
                and now - self.attitude_time < self._attitude_min_interval):
            return
        self.attitude = msg
        self.attitude_time = now

    def detected_callback(self, msg):
        self.tubes_flag = bool(msg.data)

    def geometry_callback(self, msg):
        self.geometry_seen += 1
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0 or data.size % STRIDE != 0:
            return
        lp = self.local_position
        if (lp is None or not lp.xy_valid or not lp.z_valid
                or self.home_z is None or self.attitude is None):
            return
        q = np.asarray(self.attitude.q, dtype=float)
        p = np.array([lp.x, lp.y, lp.z])
        now = time.monotonic()
        for row in data.reshape(-1, STRIDE):
            self.estimator.add(row, q, p, self.r_cam, self.t_cam, self.home_z, now)

    def hole_callback(self, msg):
        """The camera's hole candidates -> the first plausible one, buffered."""
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0 or data.size % HOLE_STRIDE != 0:
            return
        lp = self.local_position
        if (lp is None or not lp.xy_valid or not lp.z_valid
                or self.home_z is None or self.attitude is None):
            return
        q = np.asarray(self.attitude.q, dtype=float)
        p = np.array([lp.x, lp.y, lp.z])
        now = time.monotonic()
        rows = list(data.reshape(-1, HOLE_STRIDE))
        # A cell the detector could fit a clean quadrilateral to first: that
        # is the trapezium of the obstacle, and its edges are the ones worth
        # measuring a ceiling from.
        for row in sorted(rows, key=lambda r: -r[8]):
            ok, _ = self.estimator.add_hole(row, q, p, self.r_cam, self.t_cam,
                                            self.home_z, now)
            if ok:
                return

    def _hole_hint(self, u, vehicle_xy):
        """The measured cell: dict of lateral, height, ceiling, floor, or None."""
        found = self.estimator.hole(time.monotonic())
        if found is None:
            return None
        xy, height, ceiling, floor, width, rise, count = found
        return {'lateral': float(np.dot(xy - np.asarray(vehicle_xy), u)),
                'height': height, 'ceiling': ceiling, 'floor': floor,
                'width': width, 'rise': rise, 'count': count}

    def template_offset(self):
        """Centre of area of the template cell, off the midpoint, + LEFT."""
        sign = 1.0 if self.gap_side == 'left' else -1.0
        return self.cell_lateral - sign * 0.5 * self.TUBE_SPACING

    def _lean(self, hint):
        """How far to aim off the middle of the opening, + LEFT.

        cross_left metres TOWARDS THE HIGH END of the diagonal, not towards
        the aircraft's left. On this obstacle those are the same thing, but
        only because of how it happens to be built and which way it is
        approached, and leaning the wrong way is worse than not leaning at
        all: on the real course a centimetre of lean costs a centimetre of the
        eight there are to an upright and buys a centimetre of headroom under
        a diagonal that is half a metre clear. Worth it towards the high side;
        a way to hit two things at once towards the low side.

        Measured from the roof at the two shoulders of the cell when the
        camera has it. From the template -- which is to say, from whichever
        cell gap_side picked -- when it does not.
        """
        if hint is not None and abs(hint['rise']) > 1e-3:
            return math.copysign(self.CROSS_LEFT, hint['rise'])
        return math.copysign(self.CROSS_LEFT,
                             1.0 if self.gap_side == 'left' else -1.0)

    @property
    def half_airframe(self):
        return 0.5 * self.DRONE_WIDTH + self.TUBE_RADIUS + self.LATERAL_MARGIN

    # --------------------------------------------------------------- the gap

    def _update_solution(self):
        lp = self.local_position
        now = time.monotonic()
        if lp is None or self.home_z is None:
            self.solution = None
            return
        heading = self.gap_heading if self.gap_heading is not None else self.home_yaw
        u = np.array([math.sin(heading), -math.cos(heading)])
        hint = self._hole_hint(u, (lp.x, lp.y))
        self.hole_hint = hint
        clusters = self.estimator.clusters(now)
        sol, reason = solve_gap(
            clusters, (lp.x, lp.y), heading,
            self.TUBE_SPACING, self.PLANE_BAND, self.MATCH_TOLERANCE,
            self.MIN_MATCHED_TUBES, self.MAX_PLANE_YAW, self.gap_side,
            prefer_lateral=(None if hint is None
                            else hint['lateral'] + self._lean(hint)),
            lateral_offset=self.template_offset() + self._lean(None),
            half_airframe=self.half_airframe)
        if sol is not None:
            sol['hole'] = hint
        if sol is not None and not (self.MIN_GAP_WIDTH <= sol['width'] <= self.MAX_GAP_WIDTH):
            sol, reason = None, f"measured gap {sol['width']:.2f} m is implausible"
        if sol is None and hint is not None:
            # No layout fit, but the opening itself is measured. Fly that --
            # see solve_from_hole. Refusing it means hovering in front of a
            # hole the camera can see perfectly well.
            sol, reason = self._solve_from_cell(hint, clusters, heading,
                                                (lp.x, lp.y), reason)
        self.solution = sol
        self.solution_reason = reason
        if sol is not None:
            self.solution_lost_since = None
            if self.solution_ok_since is None:
                self.solution_ok_since = now
        else:
            self.solution_ok_since = None
            if self.solution_lost_since is None:
                self.solution_lost_since = now

    def _solve_from_cell(self, hint, clusters, heading, vehicle_xy, why):
        """A crossing from the measured opening, when the layout will not fit."""
        tall = hint['ceiling'] - hint['floor']
        if (hint['width'] < self.DRONE_WIDTH + 2.0 * self.LATERAL_MARGIN
                or tall < self.DRONE_HEIGHT + 2.0 * self.CLEARANCE):
            return None, (f"{why}; and the measured opening "
                          f"({hint['width']:.2f} x {tall:.2f} m) is too small "
                          "for the airframe")
        plane, plane_why = front_plane(clusters, vehicle_xy, heading,
                                       self.PLANE_BAND, self.MAX_PLANE_YAW)
        if plane is None:
            return None, f"{why}; and no tube plane either ({plane_why})"
        found = self.estimator.hole(time.monotonic())
        if found is None:
            return None, f"{why}; and the opening went stale"
        sol = solve_from_hole(plane, vehicle_xy, found[0], hint['width'],
                              self.half_airframe, lean=self._lean(hint))
        self.get_logger().warning(
            "Flying the MEASURED opening, not the template: %s. %.2f x %.2f m, "
            "%d uprights in the plane." % (why, hint['width'], tall,
                                           len(plane['front'])),
            throttle_duration_sec=5.0)
        return sol, ''

    def gap_summary(self):
        if self.solution is None:
            if self.geometry_seen == 0:
                return f"nothing on /{self.geometry_topic} -- is tube_detect running?"
            clusters = self.estimator.clusters(time.monotonic())
            return (f"no gap: {self.solution_reason} ({len(clusters)} tube "
                    f"clusters, {self.estimator.accepted_total} samples accepted; "
                    f"rejections: {self.estimator.rejection_summary()})")
        s = self.solution
        h = s.get('hole')
        hole = (f"hole centre {s['offset']:+.2f} m off centre at "
                f"{h['height']:.2f} m, ceiling {h['ceiling']:.2f}, floor "
                f"{h['floor']:.2f}, {h['width']:.2f} m wide, roof rising "
                f"{h['rise']:+.2f} m/m to the left" if h is not None
                else "no hole in view; aiming at the template centre")
        return (f"{s['side'].upper()} gap at ({s['point'][0]:+.2f}, {s['point'][1]:+.2f}), "
                f"{s['width']:.2f} m wide, heading {math.degrees(s['heading']):+.0f} deg, "
                f"{s['matched']} uprights matched, residual {s['residual']:.3f} m, "
                f"{hole}")

    def publish_gap(self):
        msg = String()
        s = self.solution
        msg.data = '' if s is None else "|".join([
            f"{s['point'][0]:.3f}", f"{s['point'][1]:.3f}",
            f"{math.degrees(s['heading']):.1f}", f"{s['width']:.3f}",
            f"{s['matched']}", f"{s['residual']:.3f}"])
        self.gap_pub.publish(msg)

    def assumed_solution(self):
        lp = self.local_position
        if lp is None:
            return None
        h = self.home_yaw
        fwd = np.array([math.cos(h), math.sin(h)])
        left = np.array([math.sin(h), -math.cos(h)])
        point = (np.array([lp.x, lp.y]) + fwd * self.ASSUME_GAP_DISTANCE
                 + left * self.ASSUME_GAP_LEFT)
        return {'point': point, 'normal': fwd, 'left': left, 'heading': h,
                'width': self.TUBE_SPACING, 'matched': 0, 'residual': 0.0,
                'side': self.gap_side, 'offset': 0.0, 'hole': None, 'back': None}

    def _freeze_path(self, sol):
        """Every waypoint of the crossing, from one gap solution."""
        self.gap_point = np.array(sol['point'], dtype=float)
        self.gap_normal = np.array(sol['normal'], dtype=float)
        self.gap_left = np.array(sol['left'], dtype=float)
        self.gap_heading = sol['heading']
        self.entry = self.gap_point - self.gap_normal * self.STANDOFF_DISTANCE
        self.pass_exit = self.gap_point + self.gap_normal * self.PASS_EXIT_DISTANCE
        self.back_along, self.back_lateral, back_measured = self._back_tube(
            sol.get('back'), sol.get('centre_offset'))
        self.shift = self._shift_offset(self.back_lateral, back_measured)
        self.shift_point = self.pass_exit + self.gap_left * self.shift
        self.final_point = (self.gap_point
                            + self.gap_normal * (self.back_along + self.BACK_TUBE_CLEAR)
                            + self.gap_left * self.shift)
        self.cross_altitude = self._crossing_altitude(sol.get('hole'))
        self._log_plan(sol)

    def _log_plan(self, sol):
        """Every direction of the crossing, in words, before it is flown."""
        hole = sol.get('hole')
        side = lambda v: 'LEFT' if v >= 0 else 'RIGHT'
        if sol.get('source') == 'hole':
            where = ("the MEASURED opening, %.2f m wide, aiming %.2f m to its %s"
                     % (sol['width'], abs(sol['offset']), side(sol['offset'])))
        else:
            where = ("the %s cell, %.2f m to the %s of the middle upright"
                     % (self.gap_side.upper(),
                        abs(sol['offset'] + 0.5 * self.TUBE_SPACING),
                        side(1.0 if self.gap_side == 'left' else -1.0)))
        self.get_logger().warning(
            "TUBE PLAN: cross %s, at %.2f m. %s the roof: %s. Then %.2f m to "
            "the %s and %.2f m on, passing the back upright %.2f m to our %s."
            % (where, self.cross_altitude,
               'MEASURED' if hole is not None else 'TEMPLATE',
               (f"rises {abs(hole['rise']):.2f} m/m to the {side(hole['rise'])}"
                if hole is not None else
                f"assumed high on the {self.gap_side.upper()}"),
               abs(self.shift), side(self.shift),
               self.back_along + self.BACK_TUBE_CLEAR,
               abs(self.back_lateral), side(self.back_lateral)))

    def _back_tube(self, back, centre_offset=None):
        """(along, lateral) of the upright behind the plane, relative to the
        crossing point. Measured where it was seen, from the parameters if not.

        When it was not seen, it is assumed to stand on the CENTRELINE of the
        structure, and where that centreline is comes from the uprights the
        camera actually measured -- not from gap_side. The two disagree
        whenever the cell flown is not the cell the template picked, and this
        is the number the sideways step afterwards is built on, so taking it
        off gap_side is how the aircraft steps back across the centreline
        into the upright it is meant to be dodging.
        """
        if back is None:
            if centre_offset is not None:
                self.get_logger().warning(
                    f"The back upright was not in view; assuming it stands "
                    f"{self.BACK_TUBE_DISTANCE:.2f} m behind the plane, on the "
                    f"centreline of the structure -- {abs(centre_offset):.2f} m "
                    f"to our {'LEFT' if centre_offset >= 0 else 'RIGHT'}, "
                    "measured off the uprights.")
                return self.BACK_TUBE_DISTANCE, centre_offset, True
            self.get_logger().warning(
                f"The back upright was not in view and neither were two "
                f"uprights to take a centreline from; assuming it stands "
                f"{self.BACK_TUBE_DISTANCE:.2f} m behind the plane, on the "
                f"{self.gap_side.upper()} cell's inner edge.")
            sign = 1.0 if self.gap_side == 'left' else -1.0
            return self.BACK_TUBE_DISTANCE, -sign * 0.5 * self.TUBE_SPACING, False
        self.get_logger().warning(
            f"Back upright measured {back['along']:.2f} m behind the plane, "
            f"{back['lateral']:+.2f} m off the track (+ = left).")
        return back['along'], back['lateral'], True

    def _shift_offset(self, back_lateral, measured=False):
        """How far sideways to step after the gap, + LEFT.

        Left by preference, as far as it takes to have half an airframe plus
        the clearance between a prop tip and the back upright, and never less
        than shift_left. Right instead, but only if going left would mean an
        absurd step -- which happens when the back upright is off to the left
        already.
        """
        need = self.half_airframe + self.CLEARANCE
        least = abs(self.SHIFT_LEFT)
        go_left = back_lateral + need
        go_right = back_lateral - need
        if go_left <= self.MAX_SHIFT:
            return max(least, go_left)
        if not measured and not self.ALLOW_SHIFT_RIGHT:
            self.get_logger().error(
                f"The back upright measures {back_lateral:+.2f} m to the LEFT "
                f"of the track, which would take a {go_left:.2f} m step to go "
                f"round on the left (max {self.MAX_SHIFT:.2f}). That is "
                "probably a bad measurement. Stepping "
                f"{self.MAX_SHIFT:.2f} m LEFT anyway; allow_shift_right:=true "
                "to let it go round the other side.")
            return self.MAX_SHIFT
        self.get_logger().warning(
            f"Stepping {abs(go_right):.2f} m RIGHT after the gap: the back "
            f"upright is {back_lateral:+.2f} m to the LEFT of the track and "
            f"going round it on the left would take {go_left:.2f} m. This is "
            "the correct side when the cell crossed was the right-hand one -- "
            "stepping left there walks back across the centreline into it.")
        return min(-least, go_right)

    def exit_distance(self):
        """How far on from the shift point the run ends."""
        if self.EXIT_DISTANCE > 0.0:
            return self.EXIT_DISTANCE
        return max(0.0, self.back_along + self.BACK_TUBE_CLEAR
                   - self.PASS_EXIT_DISTANCE)

    def _crossing_altitude(self, hole):
        """The crossing height, from the MEASURED ceiling and floor of the cell.

        The template band is not used here, and deliberately so: it is built
        from diagonal_left_height and diagonal_right_height, and if those are
        the wrong way round -- the obstacle built mirrored, or the aircraft
        coming at it from the other side -- the band says there is room where
        the diagonal actually is. The camera measures the ceiling over the
        aircraft's own track. Where the two disagree, the camera wins, and the
        disagreement is logged because it means the parameters are wrong.
        """
        if hole is None or self.cross_altitude_override > 0.0:
            return self.cross_altitude
        lo = hole['floor'] + self.CLEARANCE + self.body_below
        hi = hole['ceiling'] - self.CLEARANCE - self.body_above
        wanted = hole['height'] - self.CROSS_DROP
        if hi < lo:
            middle = 0.5 * (hole['floor'] + hole['ceiling'])
            self.get_logger().error(
                f"The measured cell ({hole['floor']:.2f}-{hole['ceiling']:.2f} m) "
                f"is under {self.DRONE_HEIGHT + 2 * self.CLEARANCE:.2f} m tall. "
                f"Threading the middle of it at {middle:.2f} m.")
            altitude = middle
        else:
            altitude = min(max(wanted, lo), hi)
        altitude = min(max(altitude, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if not (self.gap_floor - 0.05 <= altitude <= self.gap_roof + 0.05):
            self.get_logger().error(
                f"MEASURED crossing height {altitude:.2f} m is outside the "
                f"TEMPLATE band {self.gap_floor:.2f}-{self.gap_roof:.2f} m. "
                "Trusting the camera, but check cross_bar_height and "
                "diagonal_left_height/diagonal_right_height -- left and right "
                "are as the APPROACHING AIRCRAFT sees them, and having them "
                "the wrong way round is what flies it into the diagonal.")
        self.get_logger().warning(
            f"Crossing at {altitude:.2f} m: the camera puts the cell at "
            f"{hole['floor']:.2f}-{hole['ceiling']:.2f} m over the track, "
            f"centre of area {hole['height']:.2f} m (template said "
            f"{self.cross_altitude:.2f} m).")
        return altitude

    def _path_frame(self, target):
        """(along, cross) from the aircraft to target, in the gap frame."""
        lp = self.local_position
        if lp is None or target is None or self.gap_normal is None:
            return None, None
        e = np.asarray(target) - np.array([lp.x, lp.y])
        return float(np.dot(e, self.gap_normal)), float(np.dot(e, self.gap_left))

    def _distance_past_plane(self):
        lp = self.local_position
        if lp is None or self.gap_point is None:
            return -1e9
        return float(np.dot(np.array([lp.x, lp.y]) - self.gap_point, self.gap_normal))

    # ---------------------------------------------------------------- moves

    def _set_target(self, xy, altitude=None):
        self.stage_target = np.array(xy, dtype=float)
        self.move_target_x = float(xy[0])
        self.move_target_y = float(xy[1])
        self.moving = True
        if altitude is not None:
            self.commanded_altitude = float(altitude)
            self.target_z = self.home_z - float(altitude)

    def _clamp_to_cone(self, heading):
        if self.YAW_CONE <= 0.0 or self.home_z is None:
            return heading
        off = wrap_pi(heading - self.home_yaw)
        if abs(off) <= self.YAW_CONE:
            return heading
        return wrap_pi(self.home_yaw + math.copysign(self.YAW_CONE, off))

    def _aim_yaw_at(self, heading):
        self.yaw_remaining = wrap_pi(self._clamp_to_cone(heading) - self.yaw_setpoint)

    def _heading_error(self, heading):
        lp = self.local_position
        return math.pi if lp is None else abs(wrap_pi(heading - lp.heading))

    def _arrived(self, target):
        """True once within ARRIVE_TOLERANCE for ARRIVE_SETTLE_SECONDS."""
        along, cross = self._path_frame(target)
        if (along is not None and abs(along) <= self.ARRIVE_TOLERANCE
                and abs(cross) <= self.ARRIVE_TOLERANCE):
            now = time.monotonic()
            if self.settle_since is None:
                self.settle_since = now
            return now - self.settle_since >= self.ARRIVE_SETTLE_SECONDS
        self.settle_since = None
        return False

    def _enter_tube_stage(self, stage):
        self._enter_stage(stage)
        self.settle_since = None
        self.push_since = None

    # ---------------------------------------------------------- EKF2 resets

    def _on_heading_reset(self, delta):
        super()._on_heading_reset(delta)
        lp = self.local_position
        if lp is None:
            self.estimator.samples.clear()
            return
        pivot = (lp.x, lp.y)
        self.estimator.rotate(pivot, delta)
        c, s = math.cos(delta), math.sin(delta)

        def turn(p):
            dx, dy = p[0] - pivot[0], p[1] - pivot[1]
            return np.array([pivot[0] + c * dx - s * dy, pivot[1] + s * dx + c * dy])

        def spin(v):
            return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

        for name in ('gap_point', 'entry', 'pass_exit', 'shift_point',
                     'final_point', 'stage_target'):
            if getattr(self, name, None) is not None:
                setattr(self, name, turn(getattr(self, name)))
        for name in ('gap_normal', 'gap_left'):
            if getattr(self, name, None) is not None:
                setattr(self, name, spin(getattr(self, name)))
        if self.gap_heading is not None:
            self.gap_heading = wrap_pi(self.gap_heading + delta)
        if self.move_target_x is not None:
            self.move_target_x, self.move_target_y = turn(
                (self.move_target_x, self.move_target_y))
        # Guarded separately: a target can exist with no ramp start behind it,
        # and turning None raises inside a subscription callback, which kills
        # the node in flight. See window_traverse._on_heading_reset.
        if self.move_start_x is not None and self.move_start_y is not None:
            self.move_start_x, self.move_start_y = turn(
                (self.move_start_x, self.move_start_y))

    # ------------------------------------------------------------ health

    def flow_is_healthy(self):
        """Flow fusion alone during and after the pass (see course_fsm)."""
        if self.current_stage not in self.FLOW_ONLY_STAGES:
            return super().flow_is_healthy()
        lp = self.local_position
        f = self.estimator_flags
        if f is None:
            return super().flow_is_healthy()
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and lp.dist_bottom > self.FLOW_MIN_AGL
                and f.cs_opt_flow and not f.cs_inertial_dead_reckoning)

    def flight_time(self):
        return 0.0 if self.flight_start is None else time.monotonic() - self.flight_start

    def _check_flight_clock(self):
        """Land at flight_seconds -- never from inside the gap."""
        if self.flight_start is None or self.current_stage == self.PASS:
            return False
        if self.current_stage not in (self.TAKEOFF, self.HOLD) + self.TUBE_STAGES:
            return False
        if self.flight_time() < self.FLIGHT_SECONDS:
            return False
        self._begin_landing(f"{self.FLIGHT_SECONDS:.0f} s airborne")
        return True

    # ------------------------------------------------------- state machine

    def timer_callback(self):
        self._update_solution()
        self.publish_gap()

        if self.current_stage not in self.TUBE_STAGES:
            super().timer_callback()
            return

        if self._check_flight_clock():
            return

        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._begin_landing("operator abort")
            return

        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()

        {
            self.SEARCH: self._handle_search,
            self.LOCK: self._handle_lock,
            self.ALIGN: self._handle_align,
            self.PASS: self._handle_pass,
            self.SHIFT: self._handle_shift,
            self.EXIT: self._handle_exit,
            self.CLEAR: self._handle_clear,
        }[self.current_stage]()

    def _handle_takeoff(self):
        if self.flight_start is None:
            self.flight_start = time.monotonic()
        super()._handle_takeoff()

    def _handle_hold(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining > 0.0:
            self.get_logger().info(f"Holding, {remaining:.1f} s...",
                                   throttle_duration_sec=1.0)
            self.log_flight_state()
            return
        if self.problems:
            self.outcome = 'REFUSED: ' + '; '.join(self.problems)
            self._begin_landing("tube geometry not flyable")
            return
        if not self.hold_xy:
            self.get_logger().warning("Waiting for a healthy lateral estimate.",
                                      throttle_duration_sec=2.0)
            return
        if self.flying_blind:
            sol = self.assumed_solution()
            if sol is not None:
                self._begin_align(sol)
            return
        self._enter_tube_stage(self.SEARCH)
        self.get_logger().warning("SEARCH: holding, looking for the uprights.")

    def _handle_search(self):
        if self.solution is not None and self.hold_xy:
            self._enter_tube_stage(self.LOCK)
            self.get_logger().warning(f"LOCK: {self.gap_summary()}.")
            return
        if self._in_stage_for() > self.SEARCH_TIMEOUT:
            self._abandon(f"no gap found in {self.SEARCH_TIMEOUT:.0f} s. {self.gap_summary()}")
            return
        self.get_logger().info(f"SEARCH: {self.gap_summary()}", throttle_duration_sec=1.0)

    def _handle_lock(self):
        if self.solution is None:
            if (self.solution_lost_since is not None and
                    time.monotonic() - self.solution_lost_since > self.SOLUTION_LOST_TIMEOUT):
                self._abandon(f"lost the gap during LOCK. {self.gap_summary()}")
            return
        if self._in_stage_for() > self.LOCK_TIMEOUT:
            self._abandon("the gap estimate never held steady")
            return
        if (self.solution_ok_since is None
                or time.monotonic() - self.solution_ok_since < self.LOCK_SECONDS):
            self.get_logger().info(f"LOCK: settling. {self.gap_summary()}",
                                   throttle_duration_sec=1.0)
            return
        self._begin_align(self.solution)

    def _begin_align(self, sol):
        self._freeze_path(sol)
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_tube_stage(self.ALIGN)
        self._set_target(self.entry, self.cross_altitude)
        self._aim_yaw_at(self.gap_heading)
        self.get_logger().warning(
            f"ALIGN: {'ASSUMED' if sol['matched'] == 0 else 'measured'} gap at "
            f"({self.gap_point[0]:+.2f}, {self.gap_point[1]:+.2f}), heading "
            f"{math.degrees(self.gap_heading):+.0f} deg. Flying to the entry "
            f"{self.STANDOFF_DISTANCE:.2f} m short of it at {self.cross_altitude:.2f} m.")

    def _handle_align(self):
        self._aim_yaw_at(self.gap_heading)

        # Refine while the uprights are still well in view. The heading is
        # kept; only the gap point may move, and not by much.
        if (not self.flying_blind and self.solution is not None
                and -self._distance_past_plane() > self.REFINE_MIN_DISTANCE):
            jump = float(np.linalg.norm(self.solution['point'] - self.gap_point))
            if jump <= self.REFINE_MAX_JUMP:
                sol = dict(self.solution)
                sol['normal'], sol['left'], sol['heading'] = (
                    self.gap_normal, self.gap_left, self.gap_heading)
                self._freeze_path(sol)
                self._set_target(self.entry, self.cross_altitude)
            else:
                self.get_logger().warning(
                    f"ALIGN: ignoring a {jump:.2f} m jump in the gap estimate.",
                    throttle_duration_sec=2.0)

        if not self.hold_xy:
            self.moving = False
            self.settle_since = None
            if self._in_stage_for() > self.ALIGN_TIMEOUT:
                self._abandon("lateral estimate never recovered before the pass")
            return
        self.moving = True

        along, cross = self._path_frame(self.entry)
        alt = self.relative_altitude()
        ready = (along is not None
                 and abs(along) <= self.ALIGN_ALONG_TOLERANCE
                 and abs(cross) <= self.ALIGN_CROSS_TOLERANCE
                 and alt is not None
                 and abs(alt - self.cross_altitude) <= self.ALT_TOLERANCE
                 and self._heading_error(self.gap_heading) <= self.ALIGN_YAW_TOLERANCE)
        if ready:
            now = time.monotonic()
            if self.settle_since is None:
                self.settle_since = now
            elif now - self.settle_since >= self.SETTLE_SECONDS:
                self._begin_pass()
            return
        self.settle_since = None

        if self._in_stage_for() > self.ALIGN_TIMEOUT:
            self._abandon(f"could not settle on the entry in {self.ALIGN_TIMEOUT:.0f} s")
            return
        self.get_logger().info(
            f"ALIGN: {0.0 if along is None else along:+.2f} along / "
            f"{0.0 if cross is None else cross:+.2f} across (tol "
            f"{self.ALIGN_CROSS_TOLERANCE:.2f}), alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.cross_altitude:.2f}, yaw err "
            f"{math.degrees(self._heading_error(self.gap_heading)):.0f} deg. "
            f"{self.gap_summary()}", throttle_duration_sec=1.0)

    def _begin_pass(self):
        if self.hole_hint is None and not self.flying_blind:
            self.get_logger().error(
                "COMMITTING WITHOUT HAVING SEEN THE OPENING. Nothing has come "
                f"off {self.hole_topic} that survived the gating, so the "
                "crossing height is the TEMPLATE's, and the template cannot "
                "tell which way the diagonal slopes. If it is the wrong way "
                "round this is the run that hits it. Check the overlay: the "
                "cell should be outlined and crossed.")
        self.MOVE_SPEED = self.PASS_SPEED
        self._enter_tube_stage(self.PASS)
        self._set_target(self.pass_exit, self.cross_altitude)
        self.get_logger().warning(
            f"PASS: committed. Through the gap to "
            f"({self.pass_exit[0]:+.2f}, {self.pass_exit[1]:+.2f}) at "
            f"{self.PASS_SPEED:.2f} m/s. The camera is no longer steering.")

    def _handle_pass(self):
        self.yaw_remaining = wrap_pi(self.gap_heading - self.yaw_setpoint)
        now = time.monotonic()

        if not self.hold_xy:
            past = self._distance_past_plane()
            if past < -0.35 and self.push_since is None:
                self._abandon("flow lost before the gap; landing in front of it")
                return
            if self.push_since is None:
                remaining = max(0.0, self.PASS_EXIT_DISTANCE - past)
                self.push_seconds = remaining / max(self.PASS_SPEED, 1e-3)
                self.push_since = now
                self.get_logger().error(
                    f"PASS: flow lost in the gap. Pushing on open-loop "
                    f"{remaining:.2f} m ({self.push_seconds:.1f} s) before landing.")
            elif now - self.push_since >= self.push_seconds:
                self._abandon("flow lost in the gap; pushed through open-loop")
            return
        if self.push_since is not None:
            self.get_logger().warning("PASS: flow is back; resuming.")
            self.push_since = None

        if self._arrived(self.pass_exit):
            if self.MERGE_SHIFT:
                # One diagonal leg from just past the plane to clear of the
                # back upright, instead of a sideways step and then a straight
                # run. The aircraft is square to the gap while it is BETWEEN
                # the uprights -- there is under 10 cm a side there and no
                # room to be going sideways -- and starts easing left the
                # moment it is out the other side.
                self.MOVE_SPEED = self.SHIFT_SPEED
                self._enter_tube_stage(self.EXIT)
                self._set_target(self.final_point, self.cross_altitude)
                self.get_logger().warning(
                    f"EXIT: through. One diagonal leg {self.shift:+.2f} m "
                    f"({'left' if self.shift >= 0 else 'right'}) and "
                    f"{self.exit_distance():.2f} m on, round the back upright.")
                return
            self.MOVE_SPEED = self.SHIFT_SPEED
            self._enter_tube_stage(self.SHIFT)
            self._set_target(self.shift_point, self.cross_altitude)
            self.get_logger().warning(
                f"SHIFT: through. Stepping {self.shift:+.2f} m "
                f"({'left' if self.shift >= 0 else 'right'}) of the gap line "
                "to clear the back upright.")
            return
        if self._in_stage_for() > self.PASS_TIMEOUT:
            self._abandon("the pass timed out")
            return
        self.get_logger().info(
            f"PASS: {self._distance_past_plane():+.2f} m past the plane.",
            throttle_duration_sec=0.5)

    def _handle_shift(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._abandon("flow lost during the shift; landing straight down")
            return
        if self._arrived(self.shift_point):
            self.MOVE_SPEED = self.APPROACH_SPEED
            self._enter_tube_stage(self.EXIT)
            self._set_target(self.final_point, self.cross_altitude)
            self.get_logger().warning(
                f"EXIT: {self.exit_distance():.2f} m on, clear past the back upright.")
            return
        if self._in_stage_for() > self.MOVE_STAGE_TIMEOUT:
            self._abandon("the shift timed out")

    def _handle_exit(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._abandon("flow lost during the exit; landing straight down")
            return
        past = self._distance_past_plane()
        _, cross = self._path_frame(self.final_point)
        if ((past >= self.back_along + self.BACK_TUBE_CLEAR - self.ARRIVE_TOLERANCE
             and cross is not None and abs(cross) <= self.CROSS_CLEAR)
                or self._arrived(self.final_point)):
            self.outcome = f"TUBES CROSSED ({self.gap_side} gap at {self.cross_altitude:.2f} m)"
            self._enter_tube_stage(self.CLEAR)
            return
        if self._in_stage_for() > self.MOVE_STAGE_TIMEOUT:
            # Being past the back upright is what the exit is FOR. Give up on
            # the last few centimetres rather than land alongside it, which is
            # where a plain timeout used to leave the aircraft.
            past = self._distance_past_plane()
            if past > self.back_along + self.CLEARANCE:
                self.get_logger().warning(
                    f"EXIT: did not settle, but {past:.2f} m past the plane is "
                    f"clear of the back upright at {self.back_along:.2f} m. "
                    "Calling it done.")
                self.outcome = (f"TUBES CROSSED ({self.gap_side} gap at "
                                f"{self.cross_altitude:.2f} m, exit not settled)")
                self._enter_tube_stage(self.CLEAR)
                return
            self._abandon("the exit timed out short of the back upright")

    def _handle_clear(self):
        if self._in_stage_for() >= self.CLEAR_SECONDS:
            self._begin_landing("tubes crossed")

    def _abandon(self, reason):
        self.outcome = f"ABANDONED: {reason}"
        self.moving = False
        self.yaw_remaining = 0.0
        self.MOVE_SPEED = self.APPROACH_SPEED
        self.get_logger().error(f"Tube crossing abandoned: {reason}.")
        self._begin_landing(f"tube crossing abandoned -- {reason}")

    # --------------------------------------------------------------- output

    def publish_position_setpoint(self):
        """The inherited setpoint, except for the open-loop push in the gap."""
        if not (self.current_stage == self.PASS and self.push_since is not None
                and not self.hold_xy and self.home_z is not None):
            super().publish_position_setpoint()
            return
        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._step_setpoint_ramp()
        self._step_yaw_ramp()
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [self.PASS_SPEED * float(self.gap_normal[0]),
                        self.PASS_SPEED * float(self.gap_normal[1]), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        if self.current_stage not in self.TUBE_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.current_stage in (self.SEARCH, self.LOCK):
            detail = 'gap' if self.solution is not None else 'look'
        elif self.current_stage in (self.ALIGN, self.PASS):
            detail = f"p{self._distance_past_plane():+.1f}"
        else:
            detail = self.current_stage.lower()[:4]
        msg = String()
        msg.data = "|".join([
            self.current_stage, 'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail])
        self.status_pub.publish(msg)

    def log_flight_state(self):
        super().log_flight_state()
        self.get_logger().info(f"tubes: {self.gap_summary()}", throttle_duration_sec=2.0)

    def destroy_node(self):
        self.get_logger().warning(
            f"Tube crossing outcome: {self.outcome}. "
            f"{self.estimator.accepted_total} samples accepted from "
            f"{self.geometry_seen} frames; rejections: "
            f"{self.estimator.rejection_summary(limit=5)}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TubeCross()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
