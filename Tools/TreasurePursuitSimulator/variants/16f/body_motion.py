"""Continuous body pose around the unchanged graph travel-time model."""
import math
import simulate as base
from body_geometry import Pose


ANGLES = {'N':-math.pi/2,'E':0.,'S':math.pi/2,'W':math.pi}


def wrap(a):
    return (a+math.pi) % (2*math.pi)-math.pi


class BodyRobot(base.Robot):
    def configure(self, body, turn_model):
        self.body = body
        self.turn_model = turn_model
        fraction = turn_model['brake_fraction']
        if not 0 <= fraction < 1 or turn_model['profile'] != 'smoothstep':
            raise ValueError('Unsupported turn profile')
        if not 0 < turn_model['pivot_angle_deg'] <= 90:
            raise ValueError('Invalid pivot waypoint')
        if turn_model['center_path'] != 'fixed_at_track_node':
            raise ValueError('Uncalibrated centre paths are not silently substituted')
        self.angle = ANGLES[self.body_facing or self.facing]
        self.angle_from = self.angle; self.angle_delta = 0.
        self.turn_elapsed = 0.; self.turn_total = 0.
        return self

    def angle_at(self, delay=0.):
        if self.phase not in ('turn','orient') or self.turn_total <= 0:
            return self.angle
        elapsed = min(self.turn_total,self.turn_elapsed+delay)
        brake = self.turn_model['brake_fraction'] * self.turn_total
        u = max(0., min(1., (elapsed-brake)/(self.turn_total-brake)))
        return self.angle_from + self.angle_delta * u*u*(3-2*u)

    def stage(self):
        if self.phase in ('turn','orient') and abs(self.angle_delta) > base.EPS:
            if self.turn_elapsed <= self.turn_total*self.turn_model['brake_fraction']:
                return 'brake'
            angle = abs(self.angle_at()-self.angle_from)*180/math.pi
            return 'pivot' if angle < self.turn_model['pivot_angle_deg'] else 'align'
        if self.phase == 'reverse':
            return 'switch'
        if self.phase == 'move':
            return 'reverse_move' if self.reversing else 'forward'
        return 'hold'

    def pose_at(self, track, delay=0.):
        x,y = self.xy(track)
        vx,vy = self.velocity(track)
        return Pose((x+vx*delay)*track.spacing,(y+vy*delay)*track.spacing,
                    self.angle_at(delay))

    def corner_speed_bound(self, track):
        vx,vy = self.velocity(track)
        bound = math.hypot(vx,vy)*track.spacing
        if self.phase in ('turn','orient') and self.turn_total > 0:
            duration = self.turn_total * (1-self.turn_model['brake_fraction'])
            # max derivative of smoothstep(u) = 1.5; no angular overshoot.
            bound += abs(self.angle_delta)*1.5/duration*self.body.radius
        return bound

    def command(self, track, routing, neighbor, reason):
        current = self.angle_at()
        super().command(track,routing,neighbor,reason)
        self.angle = current
        self.angle_from = current
        self.angle_delta = wrap(ANGLES[self.body_facing or self.facing]-current)
        self.turn_elapsed = 0.
        self.turn_total = self.remaining if self.phase in ('turn','orient') else 0.
        if self.turn_total == 0 and abs(self.angle_delta) > base.EPS:
            raise ValueError('A body rotation cannot be instantaneous')

    def advance(self, track, dt):
        if dt < 0 or dt > self.remaining+1e-8:
            raise ValueError('Advance must stay within a robot phase')
        super().advance(track,dt)
        if self.phase in ('turn','orient'):
            self.turn_elapsed = min(self.turn_total,self.turn_elapsed+dt)
            self.angle = self.angle_at()

    def finish(self, track):
        current = self.angle_at()
        super().finish(track)
        self.angle = current
