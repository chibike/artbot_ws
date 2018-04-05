class Button
{
	constructor()
	{
		this.current_state = false;
		this.previous_state = false;

		this.up_callback = function()
		{
			// console.log("BTN UP");
		}

		this.down_callback = function()
		{
			// console.log("BTN DOWN");
		}
	}

	update(state)
	{
		this.previous_state = this.current_state;
		this.current_state = state;

		if (this.previous_state != this.current_state)
		{
			if (this.current_state == true)
			{
				this.down_callback();
			}
			else
			{
				this.up_callback();
			}
		}
	}
}

class Potentiometer
{
	constructor()
	{
		this.current_value = 0;
		this.previous_value = 0;

		this.value_changed_callback = function(new_value)
		{
			// console.log("POT VAL =", new_value);
		}
	}

	update(value)
	{
		this.previous_value = this.current_value;
		this.current_value = value;

		if (this.previous_value != this.current_value)
		{
			this.value_changed_callback(this.current_value);
		}
	}
}

class ExternalInputSource
{
	constructor()
	{
		// this.url = window.location.href;
		// this.url = "http://10.14.128.57:5000/input";

		this.btn_1 = new Button();
		this.btn_2 = new Button();
		this.btn_3 = new Button();
		this.pot   = new Potentiometer();

		var outer_this = this;
		outer_this.url = "/input?t=" + (new Date).getTime();

		var update_input = function(json_data)
		{
			outer_this.btn_1.update( json_data[0] == "1" ? true:false );
			outer_this.btn_2.update( json_data[1] == "1" ? true:false );
			outer_this.btn_3.update( json_data[2] == "1" ? true:false );
			outer_this.pot.update(json_data[3]);
		}


		var get_input = function()
		{
			fetch(outer_this.url).then(res => res.json()).then((out) => {update_input(out);})
			.catch(err => { /*console.log("Could not read input")*/ });
		}

		console.log('url', outer_this.url);

		setInterval(get_input, 100);
	}

	attach_btn_down_callback(index, callback)
	{
		if (index == 1)
		{
			this.btn_1.down_callback = callback;
		}
		else if (index == 2)
		{
			this.btn_2.down_callback = callback;
		}
		else if (index == 3)
		{
			this.btn_3.down_callback = callback;
		}
	}

	attach_btn_up_callback(index, callback)
	{
		if (index == 1)
		{
			this.btn_1.up_callback = callback;
		}
		else if (index == 2)
		{
			this.btn_2.up_callback = callback;
		}
		else if (index == 3)
		{
			this.btn_2.up_callback = callback;
		}
	}

	attach_pot_value_change_callback(callback)
	{
		this.pot.value_changed_callback = callback;
	}

	detach_all()
	{
		detach_pot();
		detach_btns();
	}

	detach_btns()
	{
		var empty_function = function()
		{

		}

		this.attach_btn_down_callback(1, empty_function);
		this.attach_btn_down_callback(2, empty_function);
		this.attach_btn_down_callback(3, empty_function);
		this.attach_btn_up_callback(1, empty_function);
		this.attach_btn_up_callback(2, empty_function);
		this.attach_btn_up_callback(3, empty_function);
	}

	detach_pot()
	{
		var empty_function = function(new_value)
		{
		}

		this.attach_pot_value_change_callback(empty_function);
	}
}