"use strict";

import {get, post} from "./common.js"

async function recordPositionRviz() {
    let result = await post("http://127.0.0.1:5000/point/add/rviz", {})
    console.log(result)
}

async function recordPositionFree() {
    let result = await post("http://127.0.0.1:5000/point/add/free", {})
    console.log(result)
}

async function getPosition() {
    let result = await get("http://127.0.0.1:5000/point/get")
    console.log(result)
}
document.getElementById('GetPosition').addEventListener('click',getPosition)

async function deletePosition() {
    let result = await post("http://127.0.0.1:5000/point/delete", {})
    console.log(result)
}
document.getElementById('DeletePosition').addEventListener('click',deletePosition)