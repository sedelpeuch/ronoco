import unittest

import requests


class TestTeleop(unittest.TestCase):
    URL = "http://localhost:5000/teleop/"

    def test_forward(self):
        rv = requests.post(self.URL + "forward")
        self.assertEqual(rv.status_code, 200)
        rv = requests.post(self.URL + "force-stop")

    def test_backward(self):
        rv = requests.post(self.URL + "backward")
        self.assertEqual(rv.status_code, 200)
        rv = requests.post(self.URL + "force-stop")

    def test_left(self):
        rv = requests.post(self.URL + "left")
        self.assertEqual(rv.status_code, 200)
        rv = requests.post(self.URL + "force-stop")

    def test_right(self):
        rv = requests.post(self.URL + "right")
        self.assertEqual(rv.status_code, 200)
        rv = requests.post(self.URL + "force-stop")

    def test_force_stop(self):
        rv = requests.post(self.URL + "force-stop")
        self.assertEqual(rv.status_code, 200)
