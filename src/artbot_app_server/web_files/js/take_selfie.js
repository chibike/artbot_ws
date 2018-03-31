

var image_stream_view = document.getElementById("image_stream")
var fps = 15;
var refresh_rate = (1.0/fps)*1000

function update_image_stream_view()
{
    image_stream_view.src = "/get_image_stream?" + (new Date).getTime();
}


setInterval(update_image_stream_view, refresh_rate);