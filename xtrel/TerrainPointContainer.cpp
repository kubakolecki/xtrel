#include "pch.h"
#include "TerrainPointContainer.h"


TerrainPointContainer::TerrainPointContainer()
{

}


TerrainPointContainer::~TerrainPointContainer()
{

}


bool TerrainPointContainer::read_from_file(std::string filename)
{
	
	Data.clear();
	std::ifstream str;
	str.open(filename);
	std::string name;
	double x, y, z;
	int t;
	double mx, my, mz;
	if (str)
	{
		while (str)
		{
			if (str)
			{
				str >> name >> x >> y >> z >> t >> mx >> my >> mz;
				Data.emplace(name, std::move(TerrainPoint::TerrainPoint(name, x, y, z, mx, my, mz, t)) );
			}
		}
	}
	else
	{
		std::cout << "cannot open " << filename << std::endl;
		str.close();
		return false;
	}

	str.close();
	return true;
}

void TerrainPointContainer::print_in_console()
{
	std::cout << "\nTerrainPointContainer Data" << std::endl;
	for (auto &p : Data)
	{
		std::cout << p.second.Name << " " << p.second.Type << " " << p.second.Coords[0] << " " << p.second.Coords[1] << " " << p.second.Coords[2]
			<<" "<< p.second.ErrorsApriori[0] <<" "<<p.second.ErrorsAposteriori[1] <<" "<<p.second.ErrorsAposteriori[2] << std::endl;
	}
}
