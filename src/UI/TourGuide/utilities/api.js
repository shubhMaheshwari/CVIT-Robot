var api ={
    getStory(){
        var url = 'http://127.0.0.1:5005/data.json';
        return fetch(url).then((res)=> res.json());
    }
    
};
module.exports = api;