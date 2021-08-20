import unittest

import requests

import rospy
from ronoco_vm import cartesian_point


class TestCartesian(unittest.TestCase):
    URL = "http://localhost:5000/point/"
    point = {
        'position': {
            'x': 0.1,
            'y': 0.1,
            'z': 0.1,
        },
        'orientation': {
            'x': 0.1,
            'y': 0.1,
            'z': 0.1,
            'w': 0.1

        }
    }
    cartesian_point = cartesian_point.CartesianPoint()

    def test_add_db(self):
        self.cartesian_point.clear_db()
        result = self.cartesian_point.add_bd(self.point)
        self.assertEqual(result, True)
        dictionnary = rospy.get_param("cartesianPoints")
        self.assertEqual(dictionnary['0'], self.point)
        self.cartesian_point.clear_db()

    def test_delete_db(self):
        rospy.set_param("cartesianPoints", {'0': self.point})
        result = self.cartesian_point.delete_bd('0')
        self.assertEqual(result, True)
        result = self.cartesian_point.delete_bd('1')
        self.assertEqual(result, False)

    def test_find_db(self):
        rospy.set_param("cartesianPoints", {'0': self.point})
        result, point = self.cartesian_point.find_db('0')
        self.assertEqual(result, True)
        self.assertEqual(point, self.point)
        result, point = self.cartesian_point.find_db('1')
        self.assertEqual(result, False)
        self.assertEqual(point, {})
        self.cartesian_point.clear_db()

    def test_add_point_real(self):
        """
        Test route "/point/add/actual
        """
        result = requests.post(self.URL + "add/actual")
        self.assertEqual(result.status_code, 200)
        self.assertNotEqual(rospy.get_param("cartesianPoints"), {})
        self.cartesian_point.clear_db()

    def test_add_point_simulation(self):
        """
        Test route "/point/add/simulation
        """
        result = requests.post(self.URL + "add/simulation")
        self.assertEqual(result.status_code, 200)
        self.assertNotEqual(rospy.get_param("cartesianPoints"), {})
        self.cartesian_point.clear_db()

    def test_delete_id(self):
        requests.post(self.URL + "delete")
        rv = requests.post(self.URL + "add/actual")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "delete/0")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "delete/5424")
        self.assertEqual(404, rv.status_code)

    def test_delete_all(self):
        rv = requests.post(self.URL + "delete")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "delete")
        self.assertEqual(200, rv.status_code)

    def test_get_id(self):
        requests.post(self.URL + "delete")
        rv = requests.get(self.URL + "get/0")
        self.assertEqual(404, rv.status_code)
        rv = requests.post(self.URL + "add/actual")
        self.assertEqual(200, rv.status_code)
        rv = requests.get(self.URL + "get/0")
        self.assertEqual(200, rv.status_code)
        self.assertIsNotNone(rv.json)

    def test_get_all(self):
        requests.post(self.URL + "delete")
        rv = requests.get(self.URL + "get")
        self.assertEqual(404, rv.status_code)
        rv = requests.post(self.URL + "add/actual")
        self.assertEqual(200, rv.status_code)
        rv = requests.get(self.URL + "get")
        self.assertEqual(200, rv.status_code)
        self.assertIsNotNone(rv.json)


if __name__ == "__main__":
    unittest.main()
