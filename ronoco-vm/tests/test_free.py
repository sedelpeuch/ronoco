import unittest

import requests

from ronoco_vm import config


class TestFree(unittest.TestCase):
    URL = "http://localhost:5000/free/"

    def test_free_post(self):
        if config.mode == "manual":
            rv = requests.post(self.URL, json={"compliant": "True"})
            self.assertEqual(rv.status_code, 200)
            self.assertEqual(rv.json(), {'Info': "compliant mode is manual"})
        elif config.mode is None:
            rv = requests.post(self.URL, json={"compliant": "True"})
            self.assertEqual(rv.status_code, 200)
            self.assertEqual(rv.json(), {"Warning": "compliant mode is None"})
        else:
            rv = requests.post(self.URL, json={"compliant": "True"})
            self.assertEqual(200, rv.status_code)
            rv = requests.post(self.URL, json={"compliant": "False"})
            self.assertEqual(200, rv.status_code)
            rv = requests.post(self.URL, json={"afgrzîdeadirjg": "droihzqdjgban"})
            self.assertEqual(400, rv.status_code)

    def test_free_get(self):
        rv = requests.get(self.URL)
        self.assertEqual(200, rv.status_code)
        self.assertIsNotNone(rv.json)


if __name__ == "__main__":
    unittest.main()
