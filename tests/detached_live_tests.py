"""Real JSON transport after actual load-driven connection failure; no pose commands."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from live_client import CoreClient


class DetachedTelemetryTests(unittest.TestCase):
    def test_load_failure_owner_telemetry_and_damage(self):
        with CoreClient() as core:
            original_mass = core.state['mass']
            for path in ['left_root_front', 'left_root_rear', 'left_root_skin']:
                core.command('damage spar '+path+' .01')
            state = core.step(1)  # Actual cell loads exhaust these nearly removed paths.
            self.assertEqual(state['debris'], 1)
            self.assertEqual(len(state['detached']), 1)
            piece = state['detached'][0]
            self.assertAlmostEqual(state['mass']+piece['mass'], original_mass)
            self.assertEqual([c['name'] for c in piece['chains']], ['left_aileron'])
            self.assertFalse(piece['chains'][0]['connected'])
            self.assertFalse(state['chains'][0]['connected'])
            self.assertTrue(state['chains'][1]['connected'])
            self.assertEqual(len(piece['cells']), 64)
            owned = {c['id'] for c in piece['cells']}
            self.assertTrue(all(not c['attached'] for c in state['cells'] if c['id'] in owned))
            state = core.step(12)
            piece = state['detached'][0]
            self.assertGreater(sum(v*v for v in piece['aero_force']), 1)
            for name in ['position','velocity','omega','quaternion','aero_force','aero_moment']:
                self.assertTrue(all(math.isfinite(v) for v in piece[name]))
            missing = piece['cells'][0]['id']
            core.command(f'damage cell {missing} 0 0')
            state = core.step(1)
            cell = next(c for c in state['detached'][0]['cells'] if c['id']==missing)
            self.assertEqual(cell['area_fraction'], 0)
            self.assertEqual(cell['force'], [0,0,0])


if __name__ == '__main__':
    unittest.main()
