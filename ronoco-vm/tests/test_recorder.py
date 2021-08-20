import unittest

import rospy
from ronoco_vm import recorder


class TestRecorder(unittest.TestCase):
    def test_record(self):
        r = recorder.Recorder()
        result = r.start_recording()
        self.assertEqual(result, True)
        rospy.sleep(2)
        result = r.stop_and_save("Test trajectory")
        self.assertEqual(result, True)

    def test_replay(self):
        my_motion = recorder.Player().load("Test trajectory")
        self.assertNotEqual(my_motion, None)


if __name__ == '__main__':
    unittest.main()
