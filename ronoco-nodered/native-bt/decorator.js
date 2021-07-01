"use strict";

module.exports = function (RED) {
    function Decorator(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function (msg) {
            node.send(msg);
        });
    }

    RED.nodes.registerType("decorator", Decorator);
}