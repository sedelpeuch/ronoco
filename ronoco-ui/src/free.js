function free() {
    const url = "http://127.0.0.1:5000/free/"
    const data = {compliant: 'True'}
    let headers = new Headers();

    headers.append('Content-Type', 'application/json');
    headers.append('Accept', 'application/json');
    fetch(url,{
        mode : 'cors',
        method:'GET',
        headers:headers,
        }
    )
        .then(response => response.json())
        .then(function(data){
            console.log(data.compliant)
            return data.compliant
        })
        .catch(error => {
            console.log('request failed', error);
        }); // Syntax error: unexpected end of input
}