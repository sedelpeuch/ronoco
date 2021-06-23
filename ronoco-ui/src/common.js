"use strict";

export {get, post};

(function (logger) {
    console.old = console.log;
    console.log = function () {
        var output = "", arg, i;

        for (i = 0; i < arguments.length; i++) {
            arg = arguments[i];
            output += "<span class=\"log-" + (typeof arg) + "\">";

            if (
                typeof arg === "object" &&
                typeof JSON === "object" &&
                typeof JSON.stringify === "function"
            ) {
                output += JSON.stringify(arg);
            } else {
                output += arg;
            }

            output += "</span>&nbsp;";
        }

        logger.innerHTML += output + "<br>";
        console.old.apply(undefined, arguments);
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
        console.log(await response.json())
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
        console.error(response.error)
        throw new response.error()
    }
}