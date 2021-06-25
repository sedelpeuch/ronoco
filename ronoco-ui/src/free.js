"use strict";

import {get, post} from "./common.js"

/**
 * Get the current robot state (compliant or not)
 * @returns {Promise<void>}
 */
async function callingRobotState() {
    let result = await get("http://127.0.0.1:5000/free/")
    console.logger(result)
}
document.getElementById('robotState').addEventListener('click',callingRobotState)

/**
 * Change current robot state with a post request on /free/
 * @returns {Promise<void>}
 */
async function callingFree(){
    let state = await get("http://127.0.0.1:5000/free/")
    let result
    if (state.compliant === "False"){
        result = await post("http://127.0.0.1:5000/free/", {compliant: "True"})
    }
    else{
        result = await post("http://127.0.0.1:5000/free/", {compliant: "False"})
    }
    console.logger(result)
}
document.getElementById('FreeButton').addEventListener('click',callingFree)