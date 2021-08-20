"use strict";

module.exports = function (RED) {
    function Parallel(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function (msg) {
            node.send(msg);
        });
    }

    RED.nodes.registerType("parallel", Parallel);
}