#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

namespace blink
{

class XmlParser
{
private:
	std::string _file_as_text;

public:
	XmlParser();
	XmlParser(const std::string &filename);

	bool load_file(const std::string &filename);
	bool load_string(std::string &text);
	bool get_data(const std::string &tag, std::vector<std::string> &out);
	bool get_params(const std::string &tag, std::vector<std::string> &out);
	bool get_data(std::string &text, const std::string &tag, std::vector<std::string> &out);
	bool get_params(std::string &text, const std::string &tag, std::vector<std::string> &out);
	void strip_tags(std::string &text);
};


XmlParser::XmlParser()
{
}

XmlParser::XmlParser(const std::string &filename)
{
	if ( !this->load_file(filename) )
		throw "could not read file";
}

bool XmlParser::load_file(const std::string &filename)
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

bool XmlParser::load_string(std::string &text)
{
	_file_as_text.clear();
	return true;
}

bool XmlParser::get_data(const std::string &tag, std::vector<std::string> &out)
{
	return this->get_data(_file_as_text, tag, out);
}

bool XmlParser::get_params(const std::string &tag, std::vector<std::string> &out)
{
	return this->get_params(_file_as_text, tag, out);
}

bool XmlParser::get_params(std::string &text, const std::string &tag, std::vector<std::string> &out)
{
	std::size_t start = 0, end = 0;
	bool found = false;

	while(1)
	{
		start = text.find("<" + tag, start);
		if (start == string::npos)
			break;

		start += tag.length() + 1; // skip pass the tags

		end = text.find("/>", start);
		if (end == string::npos)
			break;

		out.emplace_back(text.substr(start, end-start));
		found = true;
		start = end;
	}

	return found;

}

bool XmlParser::get_data(std::string &text, const std::string &tag, std::vector<std::string> &out)
{
	std::size_t start = 0, end = 0;
	bool found = false;

	while(1)
	{
		start = text.find("<" + tag , start);
		if (start == string::npos)
			break;
		start = text.find(">", start);
		start++;

		end = text.find("</" + tag, start);
		if (end == string::npos)
			break;

		out.emplace_back(text.substr(start, end-start));
		found = true;
		start = end;

	}

	return found;
}

void XmlParser::strip_tags(std::string &text)
{
	std::size_t start = 0, end = 0;

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