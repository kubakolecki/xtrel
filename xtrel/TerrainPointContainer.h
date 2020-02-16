#pragma once
#include <map>
#include"TerrainPoint.h"

class TerrainPointContainer
{
public:
	TerrainPointContainer();
	~TerrainPointContainer();
	bool read_from_file(std::string filename);
	void print_in_console();

	std::map<std::string, TerrainPoint> Data;
};

