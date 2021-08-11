"use strict";

import {post} from "./common.js"

async function forceStop() {
    let result = await post("http://127.0.0.1:5000/teleop/force-stop", {})
    console.logger(result)
}

document.getElementById('force-stop').addEventListener('click', forceStop)

async function left() {
    let result = await post("http://127.0.0.1:5000/teleop/left", {})
    console.logger(result)
}

document.getElementById('left').addEventListener('click', left)

async function right() {
    let result = await post("http://127.0.0.1:5000/teleop/right", {})
    console.logger(result)
}

document.getElementById('right').addEventListener('click', right)

async function forward() {
    let result = await post("http://127.0.0.1:5000/teleop/forward", {})
    console.logger(result)
}

document.getElementById('up').addEventListener('click', forward)

async function backward() {
    let result = await post("http://127.0.0.1:5000/teleop/backward", {})
    console.logger(result)
}

document.getElementById('down').addEventListener('click', backward)