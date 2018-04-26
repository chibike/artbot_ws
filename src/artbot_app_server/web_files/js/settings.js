
var external_input = new ExternalInputSource();

var left_button_clicked = function()
{
}

var right_button_clicked = function()
{
}

var select_button_clicked = function ()
{
}

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

external_input.attach_btn_down_callback(1, left_button_clicked);
external_input.attach_btn_down_callback(2, right_button_clicked);
external_input.attach_btn_down_callback(3, select_button_clicked);
// external_input.attach_pot_value_change_callback(pot_value_changed);


document.addEventListener("keydown", on_key_pressed);
