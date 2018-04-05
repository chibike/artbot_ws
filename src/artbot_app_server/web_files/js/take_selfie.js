var external_input = new ExternalInputSource();

var image_stream_view = document.getElementById("image_stream")
var fps = 10;
var refresh_rate = (1.0/fps)*1000

function update_image_stream_view()
{
    image_stream_view.src = "/get_image_stream?" + (new Date).getTime();
}

function goto_homescreen()
{
	window.location.replace("/home");
}

var image_stream_update = setInterval(update_image_stream_view, refresh_rate);

function stop_image_stream_view()
{
	clearInterval(image_stream_update);
	setTimeout(goto_homescreen, 20000);
}

var select_button_clicked = function ()
{
	fetch("/capture_image");
	setTimeout(stop_image_stream_view, 20000);
}

external_input.attach_btn_down_callback(3, select_button_clicked);