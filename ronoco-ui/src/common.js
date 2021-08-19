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
                temp = temp.replace("Debug", "Info".fontcolor("blue"))
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
            document.getElementById("robotState").innerHTML = "<span title='ronoco-vm is not running'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
            document.getElementById("state").src = "/static/circle_black.svg"
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
            document.getElementById("robotState").innerHTML = "<span title='ronoco-vm is not running'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
            document.getElementById("state").src = "/static/circle_black.svg"
        })
    return await response.json()
}

async function connect_io() {
    let socket2
    try {
        socket2 = io.connect(url + ':5000/states');
    } catch (error) {
        console.logger({"Error": "Ronoco-vm is not running, launch it then refresh Ronoco-ui"})
        document.getElementById("robotState").innerHTML = "<span title='ronoco-vm is not running'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
        document.getElementById("state").src = "/static/circle_black.svg"
    }
    socket2.on('connect', function (msg) {
        console.log("i'm connected to states chanel")
    });
    socket2.on('states', function (msg) {
        console.log(msg)
        if (msg['ronoco_mode'] === "manipulator") {
            if (msg['commander_state'] == false) {
                document.getElementById("robotState").innerHTML = "<span title='Check that MoveIt is running and that the order passed as a parameter is the right one, then click on the connect button'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_red.svg"
            } else if (msg['ros_state'] === false ||
                msg['moveit_state'] === false) {
                document.getElementById("robotState").innerHTML = "<span title='it means that ROS is not working as expected. The source of the problem is either the absence of roscore (the rosout/get_loggers service is not available) or the absence of moveit (the move_group/get_loggers service is not available). In this state ronoco is not usable'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_orange.svg"
            } else if (msg['rviz_state'] === false) {
                document.getElementById("robotState").innerHTML = "<span title='t means that Rviz is not communicating the position of the interactive marker. If rviz seems to be working correctly on your machine, simply move the interactive marker to fix the problem. In this state ronoco is usable but the \"simulated\" button will produce an error'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_yellow.svg"
            } else {
                document.getElementById("robotState").innerHTML = "<span title='Everything is running properly'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_green.svg"
            }
        } else if (msg['ronoco_mode'] === "rolling") {
            if (msg['ros_state'] === false ||
                msg['navigation_state'] === false) {
                document.getElementById("robotState").innerHTML = "<span title='The navigation topics do not communicate properly. If you are in the mapping phase everything is normal. If not, check the navigation launch and the namespace passed as a parameter.'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_orange.svg"
            } else if (msg['rviz_state'] === false) {
                document.getElementById("robotState").innerHTML = "<span title='Rviz does not communicate properly. Publish a first point to establish communication'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_yellow.svg"
            } else {
                document.getElementById("robotState").innerHTML = "<span title='Everything is running properly'><img id=\"state\" src=\"/static/circle.svg\" alt=\"StateLogo\" height=\"40\"></span>"
                document.getElementById("state").src = "/static/circle_green.svg"
            }
            document.getElementById("ronoco-rolling").style.display = "grid"
            document.getElementById("connect-free").style.display = "none"
        }
    })

    const socket = io.connect(url + ':5000/control_log');
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

document.getElementById('clearLog').addEventListener('click', clearLog)

connect_io()
