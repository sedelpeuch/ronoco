"use strict";

import {get, post} from "./common.js"

async function callingRobotState() {
    let result = await get("http://127.0.0.1:5000/free/")
    console.log(result)
}
document.getElementById('robotState').addEventListener('click',callingRobotState)

async function callingFree(){
    let state = await get("http://127.0.0.1:5000/free/")
    let result
    if (state.compliant === "False"){
        result = await post("http://127.0.0.1:5000/free/", {compliant: "True"})
    }
    else{
        result = await post("http://127.0.0.1:5000/free/", {compliant: "False"})
    }
    console.log(result)
}
document.getElementById('FreeButton').addEventListener('click',callingFree)