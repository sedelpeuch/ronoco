"use strict";

import {get, post} from "./common.js"

async function launchExecution() {
    let behaviorTree = await get("http://localhost:1880/flows")
    console.logger("\n Success".fontcolor("green") + " Behavior tree successfully recorded\n")
    let result = await post("http://127.0.0.1:5000/control/", {"behavior-tree":behaviorTree})
    console.logger(result)
}
document.getElementById('launch').addEventListener('click',launchExecution)