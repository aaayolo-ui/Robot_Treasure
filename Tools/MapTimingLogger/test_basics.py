"""Check basic-motion summaries against known synthetic event times."""
import copy
import json
import unittest
from analyze_routes import analyze
from build_routes import MAP
from build_full_route import build_plan
from test_full_route import synthetic,raw
from summarize_basics import summarize,describe


class BasicMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=build_plan(json.loads(MAP.read_text(encoding='utf-8')))
        cls.report=analyze(raw(synthetic(cls.plan)),cls.plan)

    def test_event_times_are_separated(self):
        result=summarize(self.report)['measurements']
        self.assertAlmostEqual(result['rotation_phase_s']['median'],.2)
        self.assertAlmostEqual(result['brake_to_rotate_s']['median'],.1)
        self.assertAlmostEqual(result['brake_to_aligned_s']['median'],.45)
        self.assertAlmostEqual(result['node_to_aligned_s']['median'],.65)
        self.assertAlmostEqual(result['forward_to_reverse_ready_s']['median'],.1)
        self.assertAlmostEqual(result['forward_to_reverse_stable_s']['median'],.14)
        self.assertAlmostEqual(result['reverse_to_forward_ready_s']['median'],.1)

    def test_rejected_capture_not_reported_as_measurement(self):
        report=copy.deepcopy(self.report)
        for s in report['sessions']:s['valid']=False
        for x in report['turns']+report['reversals']+report['intervals']:x['valid']=False
        report['simulator_candidate']['turn_extra_samples']=[]
        self.assertTrue(all(x is None for x in summarize(report)['measurements'].values()))

    def test_small_sample_statistics(self):
        self.assertIsNone(describe([]))
        self.assertEqual(describe([2])['p90'],2)
        self.assertAlmostEqual(describe([1,2,3,4,5])['p10'],1.4)
        self.assertEqual(describe([1,2,3,4,5])['median'],3)


if __name__=='__main__':unittest.main()
