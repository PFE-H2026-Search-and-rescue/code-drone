function get_url_base(){
    var _url = window.location.href;
    var _url_partial = _url.split("/");
    var _url_port_split = _url_partial[2].split(":");
    if (_url_port_split.length > 1) {
        _url_partial[2] = _url_port_split[0];
    }
    return _url_partial[2];
}

function get_url_port(){
    var _url = window.location.href;
    var _url_port = 80;
    var _url_partial = _url.split("/");
    var _url_port_split = _url_partial[2].split(":");
    if (_url_port_split.length > 1) {
        _url_port = Number(_url_port_split[1]);
        _url_partial[2] = _url_port_split[0];
    }
    return _url_port;
}