
var external_input = new ExternalInputSource();
var cycler = new Cycler(0, 3);

var card_1 = document.getElementById("card_1");
var card_2 = document.getElementById("card_2");
var card_3 = document.getElementById("card_3");
var card_4 = document.getElementById("card_4");

//external_input.detach_all();

card_1.onclick = function() {window.location.replace("/take_selfie");}
card_2.onclick = function() {window.location.replace("#");}
card_3.onclick = function() {window.location.replace("#");}
card_4.onclick = function() {window.location.replace("#");}

var items = [
	card_1,
	card_2,
	card_3,
	card_4
];

var update_item = function()
{
	// console.log("Updating item ...", item_index);
	var current_item  = items[cycler.get_current()];
	var previous_item = items[cycler.get_previous()];

	previous_item.classList.toggle("selection-card-active", false);
	current_item.classList.toggle("selection-card-active", true);
}

var set_selection = function(index)
{
	for (var i = items.length - 1; i >= 0; i--)
	{
		items[i].classList.toggle("selection-card-active", false);
	}

	items[index].classList.toggle("selection-card-active", true);
	cycler.set_current(index);
}

var left_button_clicked = function()
{
	cycler.previous();
	update_item();
}

var right_button_clicked = function()
{
	cycler.next();
	update_item();
}

var pot_changed_threshold = 5;
var previous_pot_value = 50;

var pot_value_changed = function (value)
{
	if (Math.abs(value - previous_pot_value) < pot_changed_threshold)
	{
		return;
	}

	previous_pot_value = value;

	var selection = parseInt(value / 100 * items.length);
	set_selection(selection);
}

var select_button_clicked = function ()
{
	var current_item = items[cycler.get_current()];
	current_item.onclick();
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
external_input.attach_pot_value_change_callback(pot_value_changed);


document.addEventListener("keydown", on_key_pressed);

