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
    if (response.ok) {
        return await response.json()
    } else {
        return await response.json()

    }
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
    if (response.ok) {
        return await response.json()
    } else {
        return await response.json()
    }
}