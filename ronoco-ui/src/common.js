"use strict";

export {get, post};

/**
 * This function defines the logger used to display the logs in ronoco-u. The logger can display any string but is
 * optimized for a return of JSON.stringify()
 */
(function (logger) {
    console.logger = function () {
        let output = "", arg, i;
        console.log("Document height", heightDocument)
        let heightCode = document.getElementById("logger").offsetHeight
        if (heightDocument < heightCode) {
            logger.innerHTML = ""
        }
        for (i = 0; i < arguments.length; i++) {
            arg = arguments[i];
            output += "<span class=\"log-" + (typeof arg) + "\">";
            output += date.getHours() + ":" + date.getMinutes() + ":" + date.getSeconds()

            if (
                typeof arg === "object" &&
                typeof JSON === "object" &&
                typeof JSON.stringify === "function"
            ) {
                let temp = JSON.stringify(arg, null, 1);
                temp = temp.replace(/[{}]/g, '')
                temp = temp.replace(/["]/g, '')
                temp = temp.replace(/[:]/g, '')
                temp = temp.replace("Error", "Error".fontcolor("red"))
                temp = temp.replace("Success", "Success".fontcolor("green"))
                temp = temp.replace("Info", "Info".fontcolor("blue"))
                temp = temp.replace("Warning", "Warning".fontcolor("orange"))
                temp = temp.replace("Debug", "Debug".fontcolor("blue"))
                output += temp
            } else {
                output += arg;
            }
            output += "---"
            output += "</span>&nbsp;";
        }
        logger.innerHTML += output + "<br>";
        console.log.apply(undefined, arguments);
    };
})(document.getElementById("logger"));

/**
 * Common function allow you to get json on an url
 * @param url - url to get
 * @returns {Promise<Response>} if response is ok, a http error else
 */
async function get(url) {
    let headers = new Headers();

    headers.append('Content-Type', 'application/json');
    headers.append('Accept', 'application/json');
    const response = await fetch(url, {
        mode: 'cors',
        method: 'GET',
        headers: headers,
    })
        .catch(function () {
            console.logger({"Error": "Ronoco-vm is not running"})
            document.getElementById("state").src = "/static/circle_red.svg"
        })
    return await response.json()
}

/**
 * Common function allow you to post json on an url
 * @param url - url to post
 * @param data - data to post
 * @returns {Promise<json>} if response is ok, a http error else
 */
async function post(url, data) {
    let headers = new Headers();
    headers.append('Content-Type', 'application/json');
    headers.append('Accept', 'application/json');
    const response = await fetch(url, {
        mode: 'cors',
        method: 'POST',
        headers: headers,
        body: JSON.stringify(data)
    })
        .catch(function () {
            console.logger({"Error": "Ronoco-vm is not running"})
            document.getElementById("state").src = "/static/circle_red.svg"
        })
    return await response.json()
}

async function connect_io() {
    let socket2
    try {
        socket2 = io.connect('http://localhost:5000/states');
    } catch (error) {
        console.logger({"Error": "Ronoco-vm is not running, launch it then refresh Ronoco-ui"})
        document.getElementById("state").src = "/static/circle_black.svg"
    }
    socket2.on('connect', function (msg) {
        console.log("i'm connected to states chanel")
    });
    socket2.on('states', function (msg) {
        console.log(msg)
        if (msg['commander_state']== false){
            document.getElementById("state").src = "/static/circle_red.svg"
        }
        else if (msg['robot_state'] === false ||
            msg['moveit_state'] === false) {
            document.getElementById("state").src = "/static/circle_orange.svg"
        }
        else if(msg['rviz_state'] === false){
            document.getElementById("state").src = "/static/circle_yellow.svg"
        }
        else {
            document.getElementById("state").src = "/static/circle_green.svg"
        }
    })

    const socket = io.connect('http://localhost:5000/control_log');
    socket.on('connect', function (msg) {
        console.log("i'm connected to control_log chanel")
    });
    socket.on('control_log', function (msg) {
        console.logger(msg)
    })
}

function clearLog() {
    let logger = document.getElementById("logger")
    logger.innerHTML = "";
}
document.getElementById('clearLog').addEventListener('click',clearLog)

connect_io()
