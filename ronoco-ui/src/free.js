"use strict";

async function callingRobotState() {
    let result = await get("http://127.0.0.1:5000/free/")
    console.log(result)
    let display = document.getElementById("FreeButton")
    display.innerHTML = "Robot state : " + result.compliant
}
async function callingFree(){
    let state = await get("http://127.0.0.1:5000/free/")
    let result
    if (state.compliant === "False"){
        let display = document.getElementById("FreeButton")
        display.innerHTML = "Unfree Robot"
        result = await post("http://127.0.0.1:5000/free/", {compliant: "True"})
    }
    else{
        let display = document.getElementById("FreeButton")
        display.innerHTML = "Free robot"
        result = await post("http://127.0.0.1:5000/free/", {compliant: "False"})
    }
    console.log(result)
    let display = document.getElementById("robotStateResult")
    display.innerHTML = "Robot state : " + result.compliant
}