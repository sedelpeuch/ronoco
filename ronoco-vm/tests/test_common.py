import unittest

import requests

from ronoco_vm import config


class TestCommon(unittest.TestCase):
    URL = "http://localhost:5000/"

    def test_index(self):
        """
        Test route "/"
        """
        result = requests.get(self.URL)
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json(), {"Success": "Server is running"})

    def test_connect_activate(self):
        """
        Test route "/connect" with an active commander
        """
        result = requests.get(self.URL + "connect")
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json(), {"Success": "Connected with commander " + config.move_group})

    # def test_shutdown(self):
    #     """
    #     Test route "/shutdown
    #     """
    #     result = requests.get(self.URL + "shutdown")
    #     self.assertEqual(result.status_code, 200)
    #     self.assertEqual(result.json(), {"Info": 'Server shutting down...'})


if __name__ == "__main__":
    unittest.main()
