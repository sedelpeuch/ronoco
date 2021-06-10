export default async function retrieveContent() {
    const url = "https://baconipsum.com/api/?type=all-meat&paras=2&start-with-lorem=1";

    const response = await fetch(url);
    return response.json();
}
