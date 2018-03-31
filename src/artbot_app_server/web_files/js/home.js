
var external_input = new ExternalInputSource();
var cycler = new Cycler(1, 4);

var update_item = function(item_index)
{
	console.log("Updating item ...", item_index);
}

external_input.attach_btn_down_callback(1, function()
{
	cycler.previous();
	update_item(cycler.get_current());
});

external_input.attach_btn_down_callback(2, function()
{
	cycler.next();
	update_item(cycler.get_current());
});

external_input.detach_pot();