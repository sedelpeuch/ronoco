"use strict";

import {get, post} from "./common.js"

async function recordPositionRviz() {
    let result = await post("http://127.0.0.1:5000/point/add/rviz", {})
    console.log(result)
}
document.getElementById('rviz').addEventListener('click',recordPositionRviz)

async function recordPositionFree() {
    let result = await post("http://127.0.0.1:5000/point/add/free", {})
    console.log(result)
}
document.getElementById('free').addEventListener('click',recordPositionFree)

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

async function getPositionId(){
    let id = document.getElementById("id").value;
    let result = await get("http://127.0.0.1:5000/point/get/"+id)
    console.log(result)
}
document.getElementById('GetPositionId').addEventListener('click',getPositionId)

async function deletePositionId(){
    let id = document.getElementById("id").value;
    let result = await post("http://127.0.0.1:5000/point/delete/"+id, {})
    console.log(result)
}
document.getElementById('DeletePositionId').addEventListener('click',deletePositionId)