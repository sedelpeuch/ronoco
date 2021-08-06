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
        rules: [
            {
                test: /\.js$/,
                exclude: /node_modules/,
                use: {
                    loader: "babel-loader",
                    options: {
                        presets: ["@babel/preset-env"]
                    }
                }
            },
                  {
        test: /\.css$/i,
        use: ['style-loader', 'css-loader'],
      },
        {
        test: /\.(png|svg|jpg|jpeg|gif)$/i,
        type: 'asset/resource',
      },
    {
        test: /\.html$/i,
        loader: "html-loader",
      },
        ]
    },
    
};