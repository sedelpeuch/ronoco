import retrieveContent from './query.js';

async function showContent() {
    try {
        const content = await retrieveContent();

        let elt = document.createElement('div');
        elt.innerHTML = content.join('<br />');

        document.getElementsByTagName('body')[0].appendChild(elt);
    } catch (e) {
        console.log('Error', e);
    }
}

showContent();