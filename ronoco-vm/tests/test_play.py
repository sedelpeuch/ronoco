import unittest

import requests

from flaskr import play


class TestPlayEndpoint(unittest.TestCase):
    def setUp(self) -> None:
        self.Play = play.Play()
        self.bt = [
            {
                "id": "8a2e35b6.d07718",
                "type": "tab",
                "label": "Flow 1",
                "disabled": False,
                "info": ""
            },
            {
                "id": "8239202.57af0e",
                "type": "sequence",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 200,
                "y": 660,
                "wires": [
                    [
                        "938c2f67.35349",
                        "290e0d25.6a5762",
                        "521c5901.5fab88",
                        "3019ee2e.0e1922",
                        "ac7d561f.c8a488"
                    ]
                ]
            },
            {
                "id": "28cbcd7c.301762",
                "type": "root",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 50,
                "y": 660,
                "wires": [
                    [
                        "8239202.57af0e"
                    ]
                ]
            },
            {
                "id": "938c2f67.35349",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo1",
                "data": "1",
                "x": 390,
                "y": 580,
                "wires": []
            },
            {
                "id": "290e0d25.6a5762",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo0",
                "data": "0",
                "x": 390,
                "y": 520,
                "wires": []
            },
            {
                "id": "521c5901.5fab88",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo4",
                "data": "4",
                "x": 390,
                "y": 700,
                "wires": []
            },
            {
                "id": "3019ee2e.0e1922",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo2",
                "data": "2",
                "x": 390,
                "y": 640,
                "wires": []
            },
            {
                "id": "ac7d561f.c8a488",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goTo2",
                "data": "2",
                "x": 390,
                "y": 760,
                "wires": []
            }
        ]

    def test_find_roots(self):
        result = self.Play.find_roots(self.bt)
        self.assertEqual(result[0]['id'], "28cbcd7c.301762")

    def test_find_by_id(self):
        state, result = self.Play.find_by_id("28cbcd7c.301762", self.bt)
        self.assertEqual(state, True)
        self.assertEqual(result['type'], 'root')
        state, result = self.Play.find_by_id("3a0ab027.3d1e", self.bt)
        self.assertEqual(state, False)
        self.assertEqual(result, None)

    def test_build_root(self):
        root = self.Play.find_roots(self.bt)
        self.assertEqual(root[0]['id'], "28cbcd7c.301762")
        state, child = self.Play.find_by_id(root[0]['wires'][0][0], self.bt)

        state, result = self.Play.build_root(child)
        self.assertEqual(True, state)
        child['type'] = "i'm a teapot"
        state, result = self.Play.build_root(child)
        self.assertEqual(False, state)

    def test_build_tree(self):
        for _ in range(4):
            rv = requests.post("http://localhost:5000/free/", json={"compliant": "True"})
            rv = requests.post("http://localhost:5000/point/add/free")

        root = self.Play.find_roots(self.bt)
        self.assertEqual(root[0]['id'], "28cbcd7c.301762")
        state, child = self.Play.find_by_id(root[0]['wires'][0][0], self.bt)

        state, node_bt = self.Play.build_root(child)
        self.assertEqual(True, state)
        # Classic run
        state, result = self.Play.build_tree(child, node_bt, self.bt)
        self.assertEqual(True, state)

        # Unknown type
        state, node_json = self.Play.find_by_id(child['wires'][0][0], self.bt)
        node_json['type'] = "i'm a teapot"
        state, result = self.Play.build_tree(child, node_bt, self.bt)
        self.assertEqual(False, state)

        # No child node
        state, node_json = self.Play.find_by_id(child['wires'][0][0], self.bt)
        node_json['wires'] = ""
        state, result = self.Play.build_tree(child, node_bt, self.bt)
        self.assertEqual(False, state)

        # Unknown id
        state, node_json = self.Play.find_by_id(child['wires'][0][0], self.bt)
        node_json['id'] = "i'm a super teapot"
        state, result = self.Play.build_tree(child, node_bt, self.bt)
        self.assertEqual(False, state)

    def test_play(self):
        response = requests.post("http://localhost:5000/play/", json={"behavior-tree": self.bt})
        self.assertEqual(200, response.status_code)

        response = requests.post("http://localhost:5000/play/", json={"behavior-tree": None})
        self.assertEqual(400, response.status_code)

        response = requests.post("http://localhost:5000/play/", json={"behavior-tree": {}})
        self.assertEqual(400, response.status_code)

        response = requests.post("http://localhost:5000/play/",
                                 json={"behavior-tree": {"i'm a teapot": "i'm a super teapot"}})
        self.assertEqual(400, response.status_code)


if __name__ == '__main__':
    unittest.main()
