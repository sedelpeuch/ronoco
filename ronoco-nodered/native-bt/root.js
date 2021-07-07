"use strict";

module.exports = function (RED) {
    function Root(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.warn("Behavior tree successfully recorded")
        node.on('input', function (msg) {
            node.send(msg);
        });
    }

    RED.nodes.registerType("root", Root);
}