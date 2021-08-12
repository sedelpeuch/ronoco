import time
import unittest

import ronoco_vm
from ronoco_vm import config
from tests import test_common, test_cartesianpoint, test_control, test_free


def check():
    if config.ronoco_mode == "manipulator":
        checks = {"ronoco_mode": "manipulator", "ros_state": ronoco_vm.common.Common.ros_state(),
                  "moveit_state": ronoco_vm.common.Common.moveit_state(),
                  "rviz_state": ronoco_vm.common.Common.rviz_state(),
                  "commander_state": ronoco_vm.common.Common.commander_state()}
        if checks['ros_state'] and checks['moveit_state'] and checks['commander_state']:
            return True, "manipulator"
        return False, ""
    elif config.ronoco_mode == "rolling":
        checks = {"ronoco_mode": "rolling", "ros_state": ronoco_vm.common.ros_state(),
                  "rolling_topic": ronoco_vm.common.rolling_topic(),
                  "rviz_state": ronoco_vm.common.rviz_state()}
    if checks['ros_state'] and checks['rolling_topic']:
        return True, "rolling"
    return False, ""


def launcher():
    """
        Gather all the tests from this module in a test suite according to ronoco_mode
    """
    begin = time.time()
    result, mode = check()
    while not result and time.time() - begin < 60:
        print("\033[91mCan't launch test, retring in 5 sec ... \033[0m")
        print(
            "\033[93mRonoco seems launch alone, to launch test you must launch a robot like roslaunch ur3_moveit_config demo.launch config:=true for manipulator arm or roslaunch turtlebot3_navigation turtlebot3_navigation.launch for rolling robot \033[0m")
        time.sleep(5)
        result, mode = check()
    if not result:
        print("\033[91m Test suite aborted \033[0m")
        return False

    test_suite = unittest.TestSuite()
    # test_suite.addTest(unittest.makeSuite(test_control.TestControl))

    #
    if config.ronoco_mode == "manipulator":
        test_suite.addTest(unittest.makeSuite(test_free.TestFree))
    test_suite.addTest(unittest.makeSuite(test_cartesianpoint.TestCartesian))
    test_suite.addTest(unittest.makeSuite(test_control.TestControl))
    test_suite.addTest(unittest.makeSuite(test_common.TestCommon))
    return test_suite


if __name__ == '__main__':
    test_suite = launcher()
    if test_suite:
        unittest.TextTestRunner().run(test_suite)
