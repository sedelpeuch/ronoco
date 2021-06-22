import unittest

import requests


class TestMoveEndpoint(unittest.TestCase):
    URL = "http://localhost:5000/"

    def test_move_errors(self):
        requests.post(self.URL + "point/delete")
        rv = requests.post(self.URL + "move/", json={"id": [0, 1], "mode": "plan"})
        self.assertEqual(404, rv.status_code)
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "move/", json={"id": [0, 1], "mode": "ôig"})
        self.assertEqual(400, rv.status_code)

    def test_plan(self):
        requests.post(self.URL + "point/delete")
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "move/", json={"id": [0, 1], "mode": "plan"})
        self.assertEqual(200, rv.status_code)

    def test_execute(self):
        requests.post(self.URL + "point/delete")
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "point/add/rviz")
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL + "move/", json={"id": [0, 1], "mode": "execute"})
        self.assertEqual(200, rv.status_code)


if __name__ == '__main__':
    unittest.main()
