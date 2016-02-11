#ifndef BLINK_PI_ARITHMETIC_H
#define BLINK_PI_ARITHMETIC_H

namespace blink
{

template <class T> const T& constrainf(const T& x, const T& min_x, const T& max_x)
{
	return std::min(max_x, std::max(min_x, x));
}

}
#endif