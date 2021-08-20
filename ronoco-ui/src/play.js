"use strict";

import {get, post} from "./common.js"

async function launchExecution() {
    let behaviorTree = await get(url+":1880/flows")
    let result = await post(url+":5000/control/", {"behavior-tree":behaviorTree})
    console.logger(result)
}
document.getElementById('launch').addEventListener('click',launchExecution)

async function connect_commander() {
    let result = await get(url+":5000/connect")
    console.logger(result)
}
document.getElementById('Connect').addEventListener('click',connect_commander)