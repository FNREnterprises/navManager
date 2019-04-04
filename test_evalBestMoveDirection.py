from unittest import TestCase
from  robotControl import evalBestMoveDirection
import config

class TestEvalBestMoveDirection(TestCase):
    def test_evalBestMoveDirection(self):
        m, r, e = evalBestMoveDirection(10,[config.Direction.FORWARD])
        self.assertEqual(m, config.Direction.FORWARD)
        self.assertEqual(r, 10)
        self.assertEqual(e, 0)

        m, r, e = evalBestMoveDirection(10,[config.Direction.BACKWARD])
        self.assertEqual(m, config.Direction.BACKWARD)
        self.assertEqual(r, -170)
        self.assertEqual(e, 180)

        m, r, e = evalBestMoveDirection(10,[config.Direction.FORWARD, config.Direction.BACKWARD])
        self.assertEqual(m, config.Direction.FORWARD)
        self.assertEqual(r, 10)
        self.assertEqual(e, 0)

        m, r, e = evalBestMoveDirection(170,[config.Direction.FORWARD, config.Direction.BACKWARD])
        self.assertEqual(m, config.Direction.BACKWARD)
        self.assertEqual(r, -10)

        m, r, e = evalBestMoveDirection(170,[config.Direction.FORWARD, config.Direction.LEFT])
        self.assertEqual(m, config.Direction.LEFT)
        self.assertEqual(r, 80)
        self.assertEqual(e, 90)

        m, r, e = evalBestMoveDirection(10,[config.Direction.RIGHT])
        self.assertEqual(m, config.Direction.RIGHT)
        self.assertEqual(r, 100)
        self.assertEqual(e, -90)
