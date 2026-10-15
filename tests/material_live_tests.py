"""Material events through the actual core process; no alternate mechanics."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from live_client import CoreClient


class MaterialLiveTests(unittest.TestCase):
    def test_inventory_transfer_repeat_and_reset(self):
        with CoreClient('validation-material.airframe') as core:
            before = core.state
            self.assertEqual(len(before['materials']), 2)
            s = core.command('damage material left_outer_forward .2')
            self.assertEqual(s['tick'], before['tick'])
            self.assertAlmostEqual(s['mass'], before['mass']-8)
            self.assertEqual(s['detached'][0]['material'], 'left_outer_forward')
            self.assertEqual(s['detached'][0]['mass'], 8)
            self.assertEqual(s['detached'][0]['cells'], [])
            self.assertTrue(all(not edge['failed'] for edge in s['edges']))
            again = core.command('damage material left_outer_forward .2')
            self.assertEqual(again['debris'], 1)
            with self.assertRaises(ValueError):
                core.command('damage material left_outer_forward .3')
            s = core.step(1)
            self.assertEqual(s['debris'], 1)
            self.assertAlmostEqual(s['mass']+sum(p['mass'] for p in s['detached']), before['mass'])
            reset = core.command('reset')
            self.assertEqual(reset['debris'], 0)
            self.assertEqual(reset['mass'], before['mass'])
            self.assertTrue(all(m['remaining_fraction']==1 for m in reset['materials']))

    def test_removal_follows_material_after_structural_failure(self):
        with CoreClient('validation-material.airframe') as core:
            for path in ['left_root_front', 'left_root_rear', 'left_root_skin']:
                core.command('damage spar '+path+' .01')
            separated=core.step(1)
            self.assertEqual(separated['debris'], 1)
            before=separated['detached'][0]['mass']
            s=core.command('damage material left_outer_aft 0')
            self.assertEqual(s['mass'], separated['mass'])
            self.assertEqual(s['detached'][0]['mass'], before-10)
            self.assertEqual(s['detached'][1]['material'], 'left_outer_aft')
            self.assertEqual(s['detached'][1]['mass'], 10)
            self.assertEqual(core.step(1)['debris'], 2)

    def test_small_positive_parcel_and_old_fixture(self):
        with CoreClient('validation-material.airframe') as core:
            s = core.command('damage material left_outer_forward .999999999999')
            self.assertGreater(s['detached'][0]['mass'], 0)
            self.assertEqual(core.step(2)['tick'], 2)
        with CoreClient() as core:
            self.assertEqual(core.state['materials'], [])
            with self.assertRaises(ValueError):
                core.command('damage material left_outer_forward .2')
            self.assertEqual(core.step(0)['debris'], 0)


if __name__ == '__main__':
    unittest.main()
