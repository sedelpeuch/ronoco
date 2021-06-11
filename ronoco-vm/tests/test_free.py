import tracemalloc
import unittest

import flask_unittest
from flask import request

import flaskr


class TestFreeView(flask_unittest.ClientTestCase):
    tracemalloc.start()
    app = flaskr.create_app()

    def test_free_post(self, client):
        rv = client.post('/free/', json={"compliant": "True"})
        self.assertEqual('200 OK', rv.status)
        rv = client.post('/free/', json={"compliant": "False"})
        self.assertEqual('200 OK', rv.status)
        rv = client.post('/free/', json={"afgrzîdeadirjg": "droihzqdjgban"})
        self.assertEqual('400 BAD REQUEST', rv.status)

    def test_free_get(self, client):
        rv = client.post('/free/', json={"compliant": "True"})
        self.assertEqual('200 OK', rv.status)
        rv = client.get('/free/')
        self.assertEqual('200 OK', rv.status)
        self.assertEqual({'compliant': 'True'}, rv.json)


if __name__ == "__main__":
    unittest.main()
