# Add blocks to node-red

The addition of new blocks in Ronoco is done in two parts : on the one hand the creation of the block in Node-RED allowing to define its colour, its parameters, etc. On the other hand the creation of the block in the vm allowing to define the behaviour of the block during its execution.

## Ronoco - Node-RED

Go to the `$HOME/catkin_ws/ronoco/ronoco-nodered` folder. We will create a `simple` block which will be part of the common blocks. This block will have a Name (optional) and a number (not optional) as parameters.

First, it will be necessary to create the `common/simple.html` file as follows:
```html
<script type="text/javascript">
    RED.nodes.registerType('simple', {
        category: 'ronoco/common',
        color: '#59c059',
        defaults: {
            name: {value: ""},
            data: {value: "", required: true}
        },
        credential:{
            name: {},
            mode: {},
            data:{}
        },
        inputs: 1,
        outputs: 1,
        icon: "icons/simple.svg",
        label: function () {
            return this.name || "simple";
        },
    });
</script>

<script type="text/html" data-template-name="simple">
    <div class="form-row">
        <label for="node-input-name"><i class="fa fa-tag"></i> Name</label>
        <input type="text" id="node-input-name" placeholder="Name">
    </div>
    <div class="form-row">
        <label for="node-input-data"><i class="fa fa-tag"></i> Data</label>
        <input type="number" id="node-input-data" placeholder="Data">
    </div>
</script>

<script type="text/html" data-help-name="simple">
    Here we have a simple documentation
</script>
```

Then you need to create the `common/simple.js` file as follows
```javascript
"use strict";

module.exports = function (RED) {
    function Simple(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function (msg) {
            node.send(msg);
        });
    }

    RED.nodes.registerType("simple", Simple);
}
```

Finally, you just have to fill in the new block in the `package.json` of the project
```json
"node-red": {
    "nodes": {
        ...
        "simple": "common/simple.js"
        ...
    }
}
```

Simply reinstall the palette in Node-RED to see the new block
```bash
cd $HOME/.node-red
npm install $HOME/catkin_ws/src/ronoco/ronoco-nodered/ 
```

For more information on the construction of the blocks, please refer to the [Node-RED documentation](https://nodered.org/docs/creating-nodes/)

## Ronoco - VM

Once the block is defined in Node-RED it is still necessary to give it behaviour in the vm. The definition of behaviour in the VM happens in the `$HOME/catkin_ws/src/ronoco/ronoco-vm/ronoco_vm/behaviour` folder. First we will define the node behaviour. As Ronoco-vm uses the `py_trees` library to execute behaviour trees we must respect the interface, refer to the [py_trees documentation](https://py-trees.readthedocs.io/en/devel/behaviours.html) for more information. Then just create a `simple.py` file in the `behaviour` folder.

```python
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import logger
import py_trees

class Simple(py_trees.behaviour.Behaviour):
    def __init__(self, name, data):
        super(Simple, self).__init__(name)
        self.time = float(data)

    def setup(self, timeout):
        self.logger.debug("  %s [Simple::setup()]" % self.name)
        return True

    def initialise(self):
        self.logger.debug("  %s [Simple::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Simple::update()]" % self.name)
        # Do something
        return py_trees.Status.SUCCESS #or return py_trees.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Simple::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
```

⚠ This implementation allows you to define execution blocks. Control blocks and decorators are natively supported by py_trees. To define others, please refer to the [py_trees documentation](https://py-trees.readthedocs.io/en/devel/decorators.html)

Once the *behaviour* of the block is defined it is still necessary to register the block in the vm so that it can be recognised and interpreted. To do this open the `behaviour.py` file in the `behaviour` folder. Start by defining the method for building the block. This method must **require** the parameters **name, data, child**. The method should return a tuple `Boolean, Object, String` where `Boolean` is `True` if the block is buildable, `Object` contains the built block if it is buildable and `String` contains an error message if the object is not buildable. In sum for the `simple` block we have :

```python
def simple(name, data, child):
    # Set default name if it is empty
    if name is None or name == "":
        name = "Record"
    # If data is None we can't build the bloc
    if data is None:
        return False, None, "No data"
    # Else return a Simple object
    return True, behaviour.simple.Simple(name, data), None
```

You still need to associate the name of the block with this function. At the end of the `behaviour.py` file add `'simple':simple` to the `types` dictionary. Since the `simple` block is an execution block and requires data to function it is necessary to add its name to the `leaf` and `data_node` dictionaries.

⚠ In case the new block has multiple parameters it is **necessary** to signal this to the vm by modifying the `multiple_data_nodes(node_json)` method of the `control.py` file in the `ronoco_vm` folder