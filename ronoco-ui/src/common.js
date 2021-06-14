"use strict";

/**
 * Common function allow you to get json on an url
 * @param url - url to get
 * @returns {Promise<json>} if response is ok, a http error else
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
        console.error(response.error)
        throw new response.error()
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