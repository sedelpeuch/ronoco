const path = require('path');

module.exports = {
    mode: "production",
    entry: {
        polyfill: "babel-polyfill",
        common: "./src/common.js",
        free: "./src/free.js",
        play: "./src/play.js",
        point: "./src/point.js",
        index: "./src/index.js"
    },
    output: {
        filename: "[name].bundle.js",
        path: path.resolve(__dirname, "dist")
    },
    module: {

    }
};