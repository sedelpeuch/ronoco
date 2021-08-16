"use strict";

import {get, post} from "./common.js"

/**
 * Record current robot position in rviz with POST request on /point/add/rviz
 * @returns {Promise<void>}
 */
async function recordPositionRviz() {
    let result = await post(url+":5000/point/add/simulation", {})
    console.logger(result)
}
document.getElementById('rviz').addEventListener('click',recordPositionRviz)

/**
 * Record current robot position in reality with POST request on /point/add/free
 * @returns {Promise<void>}
 */
async function recordPositionFree() {
    let result = await post(url+":5000/point/add/actual", {})
    console.logger(result)
}

document.getElementById('free').addEventListener('click',recordPositionFree)

/**
 * Get all recorded position with a GET request on /point/get
 * @returns {Promise<void>}
 */
async function getPosition() {
    let result = await get(url+":5000/point/get")
    console.logger(result)
}
document.getElementById('GetPosition').addEventListener('click',getPosition)

/**
 * Clear all recorded position with a POST request on /point/delete
 * @returns {Promise<void>}
 */
async function deletePosition() {
    let result = await post(url+":5000/point/delete", {})
    console.logger(result)
}
document.getElementById('DeletePosition').addEventListener('click',deletePosition)

/**
 * Get a recorded position by this id with a GET request on /point/get/<id>
 * @returns {Promise<void>}
 */
async function getPositionId(){
    let id = document.getElementById("id_get").value;
    let result = await get(url+":5000/point/get/"+id)
    console.logger(result)
}
document.getElementById('GetPositionId').addEventListener('click',getPositionId)

/**
 * Delete a recorded position by this id with a POST request on /point/delete/<id>
 * @returns {Promise<void>}
 */
async function deletePositionId(){
    let id = document.getElementById("id_del").value;
    let result = await post(url+":5000/point/delete/"+id, {})
    console.logger(result)
}
document.getElementById('DeletePositionId').addEventListener('click',deletePositionId)