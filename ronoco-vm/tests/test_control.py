import unittest

import requests

from flaskr import control
from flaskr import run


class TestControlEndpoint(unittest.TestCase):
    def setUp(self) -> None:
        self.Control = control.Control(run.socketio)
        self.bt = [
            {
                "id": "8a2e35b6.d07718",
                "type": "tab",
                "label": "Flow 1",
                "disabled": False,
                "info": ""
            },
            {
                "id": "4352ac4d.68f0f4",
                "type": "root",
                "z": "8a2e35b6.d07718",
                "name": "loop",
                "data": "once",
                "number": "4",
                "x": 130,
                "y": 320,
                "wires": [
                    [
                        "cb4b41.3a9b74c"
                    ]
                ]
            },
            {
                "id": "cb4b41.3a9b74c",
                "type": "sequence",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 280,
                "y": 320,
                "wires": [
                    [
                        "7d56eb6e.d80394",
                        "aa8a4707.c90c88"
                    ]
                ]
            },
            {
                "id": "7d56eb6e.d80394",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo2",
                "data": "2",
                "x": 450,
                "y": 280,
                "wires": []
            },
            {
                "id": "aa8a4707.c90c88",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo1",
                "data": "1",
                "x": 450,
                "y": 360,
                "wires": []
            },
            {
                "id": "cad81847.4ece48",
                "type": "root",
                "z": "8a2e35b6.d07718",
                "name": "setup",
                "data": "once",
                "number": "",
                "x": 130,
                "y": 180,
                "wires": [
                    [
                        "df038d8d.fce58"
                    ]
                ]
            },
            {
                "id": "df038d8d.fce58",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo0Init",
                "data": "0",
                "x": 320,
                "y": 180,
                "wires": []
            }
        ]

    def test_find_roots(self):
        result = self.Control.find_roots(self.bt)
        self.assertEqual(result[0]['id'], "4352ac4d.68f0f4")

    def test_find_by_id(self):
        state, result = self.Control.find_by_id("4352ac4d.68f0f4", self.bt)
        self.assertEqual(state, True)
        self.assertEqual(result['type'], 'root')
        state, result = self.Control.find_by_id("3a0ab027.3d1e", self.bt)
        self.assertEqual(state, False)
        self.assertEqual(result, None)

    def test_build_tree(self):
        for _ in range(4):
            rv = requests.post("http://localhost:5000/free/", json={"compliant": "True"})
            rv = requests.post("http://localhost:5000/point/add/free")

        self.Control.build_nodes(self.bt)
        for node in self.bt:
            self.Control.build_decorator(node, self.bt)

        root = self.Control.find_roots(self.bt)
        self.assertEqual(root[0]['id'], "4352ac4d.68f0f4")
        state, child = self.Control.find_by_id(root[0]['wires'][0][0], self.bt)
        self.assertEqual(True, state)
        # Classic run
        state, result = self.Control.build_tree(child, self.bt)
        self.assertEqual(True, state)

    def test_play(self):
        response = requests.post("http://localhost:5000/control/", json={"behavior-tree": self.bt})
        self.assertEqual(200, response.status_code)

        response = requests.post("http://localhost:5000/control/", json={"behavior-tree": None})
        self.assertEqual(400, response.status_code)

        response = requests.post("http://localhost:5000/control/", json={"behavior-tree": {}})
        self.assertEqual(400, response.status_code)

        response = requests.post("http://localhost:5000/control/",
                                 json={"behavior-tree": {"i'm a teapot": "i'm a super teapot"}})
        self.assertEqual(400, response.status_code)


if __name__ == '__main__':
    unittest.main()
