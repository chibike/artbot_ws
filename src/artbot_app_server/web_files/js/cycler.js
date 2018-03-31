class Cycler
{
	constructor(start=0, stop=4)
	{
		this.start = start;
		this.stop = stop;
		this.range = this.stop - this.start
		this.current_index = start;
	}

	set_current(index)
	{
		this.current_index = this.constrainf(index, this.start, this.stop);
	}

	get_current()
	{
		return this.current_index;
	}

	next()
	{
		this.current_index = ((this.current_index + 1 - this.start) % (this.range + 1)) + this.start;
	}

	previous()
	{
		this.current_index = this.current_index - 1;
		this.current_index = (this.current_index - this.start) < 0 ? this.stop : this.current_index;
	}

	constrainf(x, x_min, x_max)
	{
		return min(x_max, max(x, x_min));
	}

	mapf(x, in_min, in_max, out_min, out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}