"""Metre-based oriented rectangles and rotation-aware continuous contact.

The SAT gap is a lower bound on Euclidean shape distance. Advancing by at
most gap / (sum of maximum corner speeds) therefore cannot skip contact.
Circles are used ONLY to reject distant pairs, never to declare a capture.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Body:
    length_m: float
    width_m: float

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.length_m, self.width_m)):
            raise ValueError('Body dimensions must be positive and finite')

    @property
    def radius(self):
        return math.hypot(self.length_m, self.width_m) / 2


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    angle: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x,self.y,self.angle)):
            raise ValueError('Pose must be finite')


def rectangle(body, pose):
    c, s = math.cos(pose.angle), math.sin(pose.angle)
    l, w = body.length_m / 2, body.width_m / 2
    return [(pose.x + u*c-v*s, pose.y + u*s+v*c)
            for u, v in ((l,w),(-l,w),(-l,-w),(l,-w))]


def separation(body_a, pose_a, body_b, pose_b):
    """Positive: separated; zero: touch; negative: SAT penetration depth."""
    a, b = rectangle(body_a, pose_a), rectangle(body_b, pose_b)
    gap = -math.inf
    for theta in (pose_a.angle, pose_a.angle+math.pi/2,
                  pose_b.angle, pose_b.angle+math.pi/2):
        c, s = math.cos(theta), math.sin(theta)
        ap = [x*c+y*s for x,y in a]; bp = [x*c+y*s for x,y in b]
        gap = max(gap, min(bp)-max(ap), min(ap)-max(bp))
    return gap


def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    den = dx*dx+dy*dy
    u = max(0., min(1., ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return a[0]+u*dx, a[1]+u*dy


def contact_point(body_a, pose_a, body_b, pose_b):
    """Midpoint of nearest boundary pair; meaningful at first contact."""
    a, b = rectangle(body_a, pose_a), rectangle(body_b, pose_b)
    pairs = []
    for first, second in ((a,b),(b,a)):
        for p in first:
            for q, r in zip(second, second[1:]+second[:1]):
                nearest = point_segment(p,q,r)
                pairs.append((math.dist(p,nearest),p,nearest))
    _, p, q = min(pairs)
    return ((p[0]+q[0])/2,(p[1]+q[1])/2)


def first_contact(body_a, pose_a_at, speed_a_bound, body_b, pose_b_at,
                  speed_b_bound, maximum_dt, settings):
    """Return earliest contact delay within tolerance, or None.

    Each pose callback has linear centre motion on this interval, and the
    supplied speed bound includes |angular speed| * circumradius. The
    integrator splits intervals at robot phase changes. Nonconvergence is
    an error, never an assumed noncollision. This is a kinematic referee,
    not a rigid-body impulse or tyre-slip dynamics solver.
    """
    if not math.isfinite(maximum_dt) or maximum_dt < 0:
        raise ValueError('Invalid time interval')
    tolerance = settings['contact_tolerance_m']
    factor = settings['advancement_safety_factor']
    if not (math.isfinite(tolerance) and 0 < tolerance < .001 and 0 < factor < 1):
        raise ValueError('Invalid collision settings')
    # Linear centre-path broad phase. This is safe for rotating rectangles
    # because each entire rectangle fits its circumcircle at every angle.
    a0,b0,a1,b1 = pose_a_at(0),pose_b_at(0),pose_a_at(maximum_dt),pose_b_at(maximum_dt)
    p = (a0.x-b0.x,a0.y-b0.y); q = (a1.x-b1.x,a1.y-b1.y)
    closest = point_segment((0.,0.),p,q)
    if math.hypot(*closest) > body_a.radius+body_b.radius+tolerance:
        return None
    if not all(math.isfinite(v) and v >= 0 for v in (speed_a_bound,speed_b_bound)):
        raise ValueError('Invalid corner speed bound')
    bound = speed_a_bound + speed_b_bound
    if maximum_dt > 0:
        va=((a1.x-a0.x)/maximum_dt,(a1.y-a0.y)/maximum_dt)
        vb=((b1.x-b0.x)/maximum_dt,(b1.y-b0.y)/maximum_dt)
        angular_a=max(0.,speed_a_bound-math.hypot(*va))
        angular_b=max(0.,speed_b_bound-math.hypot(*vb))
        # Cancel common translation. Parallel cars separated by a tiny
        # fixed gap should not need thousands of vanishingly small steps.
        bound=math.hypot(va[0]-vb[0],va[1]-vb[1])+angular_a+angular_b
    t = 0.
    for _ in range(settings['max_iterations']):
        gap = separation(body_a,pose_a_at(t),body_b,pose_b_at(t))
        if gap <= tolerance:
            return t
        if t >= maximum_dt or bound == 0:
            return None
        step = factor * gap / bound
        if step > maximum_dt-t:
            # No possible first contact before the endpoint, but checking
            # the endpoint also handles a numerical near-contact there.
            t = maximum_dt
        else:
            updated = t+step
            if updated == t:
                raise ArithmeticError('Continuous contact made no numerical progress')
            t = updated
    raise ArithmeticError('Continuous contact did not converge; outcome not certified')
