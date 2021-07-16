"use strict";

module.exports = function (RED) {
    function Cartesian(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function (msg) {
            console.log(msg)
            node.send(msg);
        });
    }

    RED.nodes.registerType("cartesian", Cartesian);
}