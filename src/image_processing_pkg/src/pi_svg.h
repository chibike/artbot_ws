#pragma once

namespace blink
{

#include "pi_objects.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

class XmlParser
{
private:
	std::string _file_as_text;

public:
	XmlParser();

	bool load_file(std::string &filename);
	bool load_string(std::string &text);
	bool get_data(std::string tag, std::vector<std::string> &out);
	bool get_data(std::string &text, std::string &tag, std::vector<std::string> &out);
	void strip_tags(std::string &text);
};


XmlParser::XmlParser()
{
}

bool XmlParser::load_file(std::string &filename)
{
	_file_as_text.clear();
	char c;

	ifstream in( filename );
	if( !in )
		return false;

	while ( in.get(c) )
		_file_as_text += c;
	in.close();

	return true;
}

bool XmlParser::load_string(std::string text)
{
	_file_as_text.clear();
	return true;
}

bool XmlParser::get_data(std::string tag, std::vector<std::string> &out)
{
	return this.get_data(_file_as_text, tag, out);
}

bool XmlParser::get_data(std::string &text, std::string &tag, std::vector<std::string> &out)
{
	unsigned int start = 0, end = 0;

	while (1)
	{
		start = text.find("<" + tag + ">", start);
		if (start == string::npos)
			return false;
		start += tag.length() + 2; // skip pass the tags

		end = text.find("</" + tag, start);
		if (end == string::npos)
			return false;

		out.push_back(text.substr(start, end-start));
	}

	return true;
}

void XmlParser::strip_tags(std::string &text)
{
	unsigned int start = 0, end = 0;

	while ( start < text.length() )
	{
		start = text.find("<", start);
		if ( start == string::npos )
			break;

		end = text.find(">", start);
		if ( end == string::npos )
			break;

		text.erase(start, end-start + 1);
	}
}


}