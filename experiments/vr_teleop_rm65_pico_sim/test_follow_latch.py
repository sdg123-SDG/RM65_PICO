import unittest
from follow_latch import FollowLatch, ToggleFollow

class FollowLatchTests(unittest.TestCase):
    def test_toggle_and_loss(self):
        f = ToggleFollow()
        self.assertFalse(f.update(1))
        self.assertFalse(f.update(0))
        self.assertTrue(f.update(1))
        self.assertTrue(f.update(1))
        self.assertTrue(f.update(0))
        self.assertFalse(f.update(1))
        f.update(0); self.assertTrue(f.update(1))
        self.assertFalse(f.update(1, valid=False))
        self.assertFalse(f.update(1))
    def test_edges_stop_priority_and_tracking_loss(self):
        f = FollowLatch()
        self.assertFalse(f.update(1, 0))
        self.assertFalse(f.update(0, 0))
        self.assertTrue(f.update(1, 0))
        self.assertTrue(f.update(0, 0))
        self.assertFalse(f.update(0, 1))
        self.assertFalse(f.update(0, 0))
        self.assertFalse(f.update(1, 1))
        self.assertFalse(f.update(0, 0))
        self.assertTrue(f.update(1, 0))
        self.assertFalse(f.update(1, 0, valid=False))
        self.assertFalse(f.update(1, 0))
        self.assertFalse(f.update(0, 0))
        self.assertTrue(f.update(1, 0))

if __name__ == '__main__':
    unittest.main()
