"use strict";

module.exports = function (RED) {
    function Execute(config) {
        RED.nodes.createNode(this, config);
        const node = this;
        node.on('input', function (msg) {
            node.send(msg);
        });
    }

    RED.nodes.registerType("execute", Execute);
}