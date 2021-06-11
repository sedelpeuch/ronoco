import unittest
import flask_unittest
import flaskr
from flaskr import common_views
from werkzeug.exceptions import NotFound


class TestCommmonView(flask_unittest.ClientTestCase):
    app = flaskr.create_app()

    def test_common_index(self, client):
        """
        TODO
        """
        self.assertEqual(True, True)

    def test_common_robot_state(self, client):
        res = common_views.robot_state()
        self.assertEqual(res, {'robot_state': True} or NotFound)


if __name__ == "__main__":
    unittest.main()
