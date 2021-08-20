import unittest

from ronoco_vm import config
from tests import test_common, test_cartesianpoint, test_control, test_free, test_recorder, test_teleop


def launcher():
    """
        Gather all the tests from this module in a test suite according to ronoco_mode
    """

    test_suite = unittest.TestSuite()
    if config.ronoco_mode == "rolling":
        test_suite.addTest(unittest.makeSuite(test_teleop.TestTeleop))
    if config.ronoco_mode == "manipulator":
        test_suite.addTest(unittest.makeSuite(test_free.TestFree))
        test_suite.addTest(unittest.makeSuite(test_control.TestControl))
        test_suite.addTest(unittest.makeSuite(test_recorder.TestRecorder))
    test_suite.addTest(unittest.makeSuite(test_cartesianpoint.TestCartesian))
    test_suite.addTest(unittest.makeSuite(test_common.TestCommon))
    return test_suite


if __name__ == '__main__':
    test_suite = launcher()
    if test_suite:
        unittest.TextTestRunner().run(test_suite)
