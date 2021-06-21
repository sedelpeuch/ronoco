import unittest

import requests


class TestCartesianPoint(unittest.TestCase):
    URL = "http://localhost:5000/point/"

    def test_add_rviz(self):
        requests.post(self.URL + "delete")
        rv = requests.post(self.URL + "add/rviz")
        self.assertEqual(200, rv.status_code)
        self.assertEqual("Add cartesian point with id:0", rv.text)
        rv = requests.post(self.URL + "add/rviz", data={"lmz": "arg"})
        self.assertEqual(200, rv.status_code)

    def test_add_free(self):
        requests.post(self.URL + "delete")
        rv = requests.post(self.URL + "add/free")
        self.assertEqual(200, rv.status_code)
        self.assertEqual("Add cartesian point with id:0", rv.text)
        rv = requests.post(self.URL + "add/free", data={"lmz": "arg"})
        self.assertEqual(200, rv.status_code)

    def test_delete_id(self):
        requests.post(self.URL + "delete")
        rv = requests.post(self.URL + "add/free")
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
        rv = requests.post(self.URL + "add/free")
        self.assertEqual(200, rv.status_code)
        rv = requests.get(self.URL + "get/0")
        self.assertEqual(200, rv.status_code)
        self.assertIsNotNone(rv.json)

    def test_get_all(self):
        requests.post(self.URL + "delete")
        rv = requests.get(self.URL + "get")
        self.assertEqual(404, rv.status_code)
        rv = requests.post(self.URL + "add/free")
        self.assertEqual(200, rv.status_code)
        rv = requests.get(self.URL + "get")
        self.assertEqual(200, rv.status_code)
        self.assertIsNotNone(rv.json)


if __name__ == "__main__":
    unittest.main()
