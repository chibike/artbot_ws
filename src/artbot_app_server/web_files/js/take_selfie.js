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

var left_button_clicked = function()
{
	fetch("/stop_image_stream");
	clearInterval(image_stream_update);
	goto_homescreen();
}

var right_button_clicked = function()
{
}

external_input.attach_btn_down_callback(1, left_button_clicked);
external_input.attach_btn_down_callback(2, right_button_clicked);
external_input.attach_btn_down_callback(3, select_button_clicked);

var on_key_pressed = function(event)
{
	const key = event.key;

	if (key === "ArrowLeft")
	{
		left_button_clicked();
	}

	if (key === "ArrowRight")
	{
		right_button_clicked();
	}

	if (key === "Enter")
	{
		select_button_clicked();
	}
}
document.addEventListener("keydown", on_key_pressed);