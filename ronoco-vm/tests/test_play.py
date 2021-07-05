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
                "id": "fbf14ae2.358c98",
                "type": "tab",
                "label": "Flow 2",
                "disabled": False,
                "info": ""
            },
            {
                "id": "eb0eded6.b0c3c",
                "type": "selector",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 440,
                "y": 180,
                "wires": [
                    [
                        "7a0f950d.7e624c",
                        "2150f7.297daf0a"
                    ]
                ]
            },
            {
                "id": "c2eb0b03.6678c8",
                "type": "sequence",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 440,
                "y": 380,
                "wires": [
                    [
                        "bbebf4ea.d0f9f8",
                        "931d9b57.4cf298"
                    ]
                ]
            },
            {
                "id": "7a0f950d.7e624c",
                "type": "place",
                "z": "8a2e35b6.d07718",
                "name": "isAtA?",
                "number": "1",
                "x": 630,
                "y": 140,
                "wires": []
            },
            {
                "id": "2150f7.297daf0a",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goToA",
                "number": "1",
                "x": 630,
                "y": 240,
                "wires": []
            },
            {
                "id": "bbebf4ea.d0f9f8",
                "type": "plan",
                "z": "8a2e35b6.d07718",
                "name": "planToC",
                "number": "3",
                "x": 640,
                "y": 440,
                "wires": []
            },
            {
                "id": "931d9b57.4cf298",
                "type": "selector",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 640,
                "y": 340,
                "wires": [
                    [
                        "f6ea254.4d37bd8",
                        "5f29aa06.240384"
                    ]
                ]
            },
            {
                "id": "7717b6.cbcf284c",
                "type": "sequence",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 220,
                "y": 280,
                "wires": [
                    [
                        "eb0eded6.b0c3c",
                        "c2eb0b03.6678c8"
                    ]
                ]
            },
            {
                "id": "3a0ab027.3d1ec",
                "type": "root",
                "z": "8a2e35b6.d07718",
                "name": "",
                "x": 70,
                "y": 280,
                "wires": [
                    [
                        "7717b6.cbcf284c"
                    ]
                ]
            },
            {
                "id": "f6ea254.4d37bd8",
                "type": "place",
                "z": "8a2e35b6.d07718",
                "name": "isAtB?",
                "number": "2",
                "x": 830,
                "y": 300,
                "wires": []
            },
            {
                "id": "5f29aa06.240384",
                "type": "execute",
                "z": "8a2e35b6.d07718",
                "name": "goToB",
                "number": "2",
                "x": 830,
                "y": 380,
                "wires": []
            },
            {
                "id": "6ddc7838.9f2528",
                "type": "root",
                "z": "fbf14ae2.358c98",
                "name": "",
                "x": 340,
                "y": 440,
                "wires": [
                    [
                        "1b4ff8b1.2222b7"
                    ]
                ]
            },
            {
                "id": "1b4ff8b1.2222b7",
                "type": "execute",
                "z": "fbf14ae2.358c98",
                "name": "",
                "number": "",
                "x": 550,
                "y": 440,
                "wires": []
            }
        ]

    def test_find_roots(self):
        result = self.Play.find_roots(self.bt)
        self.assertEqual(result[0]['id'], "3a0ab027.3d1ec")
        self.assertEqual(result[1]['id'], "6ddc7838.9f2528")

    def test_find_by_id(self):
        state, result = self.Play.find_by_id("3a0ab027.3d1ec", self.bt)
        self.assertEqual(state, True)
        self.assertEqual(result['type'], 'root')
        state, result = self.Play.find_by_id("3a0ab027.3d1e", self.bt)
        self.assertEqual(state, False)
        self.assertEqual(result, None)

    def test_build_root(self):
        root = self.Play.find_roots(self.bt)
        self.assertEqual(root[0]['id'], "3a0ab027.3d1ec")
        state, child = self.Play.find_by_id(root[0]['wires'][0][0], self.bt)

        state, result = self.Play.build_root(child)
        self.assertEqual(True, state)
        child['type'] = "i'm a teapot"
        state, result = self.Play.build_root(child)
        self.assertEqual(False, state)

    def test_build_tree(self):
        root = self.Play.find_roots(self.bt)
        self.assertEqual(root[0]['id'], "3a0ab027.3d1ec")
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
