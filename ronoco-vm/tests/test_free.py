import unittest

import requests


class TestFreeView(unittest.TestCase):
    URL = "http://localhost:5000/free/"

    def test_free_post(self):
        rv = requests.post(self.URL, json={"compliant": "True"})
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL, json={"compliant": "False"})
        self.assertEqual(200, rv.status_code)
        rv = requests.post(self.URL, json={"afgrzîdeadirjg": "droihzqdjgban"})
        self.assertEqual(400, rv.status_code)

    def test_free_get(self):
        rv = requests.post(self.URL, json={"compliant": "True"})
        self.assertEqual('200 OK', rv.status_code)
        rv = requests.get(self.URL)
        self.assertEqual(self.URL, rv)
        self.assertEqual({'compliant': 'True'}, rv.json)


if __name__ == "__main__":
    unittest.main()
