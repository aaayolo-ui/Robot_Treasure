"""Geometry and motion tests independent of visual styling and the C planner."""
import copy, json, math, random, sys, unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
import simulate as base
from body_geometry import Body, Pose, rectangle, separation, first_contact
from body_motion import BodyRobot, ANGLES
MODEL=json.loads((HERE/'body_model.json').read_text(encoding='utf-8'))
BODY=Body(.190,.198)


def hit(a,b,va,vb,duration,body_a=BODY,body_b=BODY):
    return first_contact(body_a,a,va,body_b,b,vb,duration,MODEL['collision'])


class GeometryTests(unittest.TestCase):
    def test_dimensions_reject_nan_negative_zero(self):
        for v in (0,-.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):Body(v,.2)

    def test_known_axis_touch(self):
        a=Pose(0,0,0)
        self.assertAlmostEqual(separation(BODY,a,BODY,Pose(.19,0,0)),0)
        self.assertGreater(separation(BODY,a,BODY,Pose(.191,0,0)),0)
        self.assertLess(separation(BODY,a,BODY,Pose(.189,0,0)),0)

    def test_width_touch_and_corner(self):
        self.assertAlmostEqual(separation(BODY,Pose(0,0,0),BODY,Pose(0,.198,0)),0)
        self.assertAlmostEqual(separation(BODY,Pose(0,0,0),BODY,Pose(.19,.198,0)),0)

    def test_diagonal_collision_not_old_point_threshold(self):
        # Centres are >0.18 m apart but physical square envelopes touch.
        self.assertIsNotNone(hit(lambda t:Pose(0,0,0),lambda t:Pose(.19,.198,0),0,0,0))

    def test_circumcircle_overlap_is_not_capture(self):
        # Thin vehicles with overlapping bounding circles still have a gap.
        thin=Body(.3,.03)
        self.assertIsNone(hit(lambda t:Pose(0,0,0),lambda t:Pose(0,.06,0),0,0,1,thin,thin))

    def test_stationary_overlap(self):
        self.assertEqual(hit(lambda t:Pose(0,0,0),lambda t:Pose(.05,0,.7),0,0,0),0)

    def test_fast_head_on_swept(self):
        a=lambda t:Pose(-1+20*t,0,0)
        b=lambda t:Pose(1-20*t,0,math.pi)
        t=hit(a,b,20,20,.1)
        self.assertAlmostEqual(t,(2-.190)/40,places=7)
        self.assertGreater(separation(BODY,a(.1),BODY,b(.1)),0)

    def test_crossing_between_frames(self):
        a=lambda t:Pose(-.5+t,0,0)
        b=lambda t:Pose(0,-.5+t,math.pi/2)
        t=hit(a,b,1,1,1)
        self.assertAlmostEqual(t,.5-(.190+.198)/2,places=6)

    def test_rotating_corner_endpoints_both_clear(self):
        a=lambda t:Pose(0,0,math.pi/2*t)
        b=lambda t:Pose(.228,0,0)
        self.assertGreater(separation(BODY,a(0),BODY,b(0)),0)
        self.assertGreater(separation(BODY,a(1),BODY,b(1)),0)
        t=hit(a,b,math.pi/2*BODY.radius,0,1)
        self.assertTrue(0<t<1)
        self.assertLessEqual(separation(BODY,a(t),BODY,b(t)),1.0001e-7)

    def test_same_speed_no_false_capture(self):
        self.assertIsNone(hit(lambda t:Pose(t,0,0),lambda t:Pose(t+.2,0,0),1,1,.02))
        self.assertIsNone(hit(lambda t:Pose(t,0,0),lambda t:Pose(t+.19000011,0,0),1,1,.02))

    def test_nonfinite_pose_not_a_capture(self):
        for v in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):Pose(v,0,0)

    def test_order_symmetric(self):
        rng=random.Random(12)
        for _ in range(400):
            a=Pose(rng.random(),rng.random(),rng.uniform(-math.pi,math.pi))
            b=Pose(rng.random(),rng.random(),rng.uniform(-math.pi,math.pi))
            self.assertAlmostEqual(separation(BODY,a,BODY,b),separation(BODY,b,BODY,a),places=13)

    def test_dense_reference_no_missed_contacts(self):
        # Independent sampling detects actual polygon boundary intersections,
        # rather than reusing SAT as the oracle. Non-grazing seeded cases.
        rng=random.Random(48)
        def intersects(poly_a,poly_b):
            def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
            edges=lambda p:list(zip(p,p[1:]+p[:1]))
            for a,b in edges(poly_a):
                for c,d in edges(poly_b):
                    if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:return True
            return False
        checked=0
        for _ in range(55):
            x,y=rng.uniform(-.3,.3),rng.uniform(-.3,.3)
            vx,vy=rng.uniform(-.5,.5),rng.uniform(-.5,.5)
            w=rng.uniform(-math.pi,math.pi)
            a=lambda t:Pose(0,0,w*t)
            b=lambda t:Pose(x+vx*t,y+vy*t,0)
            samples=[k/700 for k in range(701)]
            first=next((t for t in samples if intersects(rectangle(BODY,a(t)),rectangle(BODY,b(t)))),None)
            result=hit(a,b,abs(w)*BODY.radius,math.hypot(vx,vy),1)
            if first is not None:
                checked+=1;self.assertIsNotNone(result)
                self.assertLessEqual(result,first+1e-9)
        self.assertGreater(checked,10)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.track=base.TrackMap()
        cfg=json.loads((HERE.parents[1]/'config.json').read_text(encoding='utf-8'))
        self.routing=base.Routing(self.track,cfg)
        self.node=next(n for n in self.track.graph if
            any(base.heading(self.track.xy[n],self.track.xy[v])=='E' for v in self.track.graph[n]))
        self.east=next(v for v in self.track.graph[self.node] if base.heading(self.track.xy[self.node],self.track.xy[v])=='E')

    def robot(self,facing):
        return BodyRobot('P',self.node,facing,.674).configure(BODY,MODEL['turn'])

    def test_turn_no_jump_then_continuous(self):
        robot=self.robot('N');start=robot.pose_at(self.track)
        robot.command(self.track,self.routing,self.east,'chase')
        self.assertEqual(robot.pose_at(self.track),start)
        duration=robot.remaining;prev=start.angle
        for _ in range(80):
            expected=robot.pose_at(self.track,duration/80)
            robot.advance(self.track,duration/80)
            actual=robot.pose_at(self.track)
            self.assertAlmostEqual(actual.angle,expected.angle,places=12)
            self.assertGreaterEqual(actual.angle,prev)
            self.assertLess(actual.angle-prev,.08)
            prev=actual.angle
        before=robot.pose_at(self.track);robot.finish(self.track)
        self.assertEqual(before,robot.pose_at(self.track))
        self.assertAlmostEqual(before.angle,0)

    def test_reverse_keeps_chassis_heading(self):
        robot=self.robot('W');robot.command(self.track,self.routing,self.east,'chase')
        self.assertTrue(robot.reversing)
        self.assertEqual(robot.stage(),'switch')
        robot.advance(self.track,robot.remaining);robot.finish(self.track)
        before=robot.pose_at(self.track);robot.advance(self.track,.01)
        after=robot.pose_at(self.track)
        self.assertEqual(before.angle,after.angle)
        self.assertGreater(after.x,before.x)
        self.assertAlmostEqual(after.angle,math.pi)

    def test_corner_speed_bound(self):
        robot=self.robot('N');robot.command(self.track,self.routing,self.east,'chase')
        limit=robot.corner_speed_bound(self.track)
        duration=robot.remaining;dt=duration/1000
        for _ in range(1000):
            a=rectangle(BODY,robot.pose_at(self.track))
            b=rectangle(BODY,robot.pose_at(self.track,dt))
            self.assertLessEqual(max(math.dist(p,q)/dt for p,q in zip(a,b)),limit+1e-6)
            robot.advance(self.track,dt)

    def test_no_zero_time_rotation(self):
        self.routing.config['turn90_s']=0
        with self.assertRaises(ValueError):self.robot('N').command(self.track,self.routing,self.east,'chase')

    def test_invalid_turn_model(self):
        model=copy.deepcopy(MODEL['turn']);model['brake_fraction']=1
        with self.assertRaises(ValueError):self.robot('N').configure(BODY,model)


if __name__=='__main__':unittest.main(verbosity=2)
