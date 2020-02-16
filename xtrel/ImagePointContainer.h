#pragma once
#include<map>
#include<set>
#include "ImagePoint.h"

class ImagePointContainer
{
public:
	ImagePointContainer();
	~ImagePointContainer();
	bool read_from_file(std::string filename);
	bool filter_out(std::set<std::string> valid_images);
	void print_in_console();

	std::vector<ImagePoint> Data;

};

