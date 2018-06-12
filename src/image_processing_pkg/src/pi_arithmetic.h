#ifndef BLINK_PI_ARITHMETIC_H
#define BLINK_PI_ARITHMETIC_H

#include <vector>

namespace blink
{

template <class T> const T& constrainf(const T& x, const T& min_x, const T& max_x)
{
	return std::min(max_x, std::max(min_x, x));
}

template <typename T, typename A>
T maximum_element(std::vector<T,A> const& list)
{
	T max_element = list.at(0);
	for (int i=0; i<list.size(); i++)
	{
		T element = list.at(i);

		if (element > max_element)
		{
			max_element = element;
		}
	}
	return max_element;
}

template <typename T, typename A>
T minimum_element(std::vector<T,A> const& list)
{
	T min_element = list.at(0);
	for (int i=0; i<list.size(); i++)
	{
		T element = list.at(i);

		if (element < min_element)
		{
			min_element = element;
		}
	}
	return min_element;
}

}
#endif