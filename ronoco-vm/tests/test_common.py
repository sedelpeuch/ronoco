import unittest

import requests
from werkzeug.exceptions import NotFound

from flaskr import common


class TestCommmonView(unittest.TestCase):
    URL = "http://localhost:5000/"

    def test_common_index(self):
        rv = requests.get(self.URL)
        self.assertEqual(200, rv.status_code)

    def test_common_robot_state(self):
        res = common.Common().robot_state()
        self.assertEqual(res, (True or NotFound))


if __name__ == "__main__":
    unittest.main()
