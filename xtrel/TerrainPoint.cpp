#include "pch.h"
#include "TerrainPoint.h"

TerrainPoint::TerrainPoint(void)
{
}

TerrainPoint::TerrainPoint(std::string N, double WspX, double WspY, double WspZ) : Name(N)
{
	Type = 3;
	Coords[0] = WspX;
	Coords[1] = WspY;
	Coords[2] = WspZ;
	Use = true;
}

TerrainPoint::TerrainPoint(std::string N, double WspX, double WspY, double WspZ, double ErrX, double ErrY, double ErrZ) 
	: Name(N)
{
	Type = 3;
	Coords[0] = WspX;
	Coords[1] = WspY;
	Coords[2] = WspZ;
	ErrorsApriori[0] = ErrX;
	ErrorsApriori[1] = ErrY;
	ErrorsApriori[2] = ErrZ;
	ErrorsAposteriori[0] = ErrX;
	ErrorsAposteriori[1] = ErrY;
	ErrorsAposteriori[2] = ErrZ;
	Use = true;
}

TerrainPoint::TerrainPoint(std::string N, double WspX, double WspY, double WspZ, short T) : Name(N), Type(T)
{
	Coords[0] = WspX;
	Coords[1] = WspY;
	Coords[2] = WspZ;
	Use = true;
}


TerrainPoint::TerrainPoint(std::string N, double WspX, double WspY, double WspZ, double ErrX, double ErrY, double ErrZ, short T)
	: Name(N), Type(T)
{
	Coords[0] = WspX;
	Coords[1] = WspY;
	Coords[2] = WspZ;
	ErrorsApriori[0] = ErrX;
	ErrorsApriori[1] = ErrY;
	ErrorsApriori[2] = ErrZ;
	ErrorsAposteriori[0] = ErrX;
	ErrorsAposteriori[1] = ErrY;
	ErrorsAposteriori[2] = ErrZ;
	Use = true;
}

void TerrainPoint::print(void)
{
	std::cout << "\nId:\t\t" << Name << "\tType: " << Type << std::endl;
	std::cout << "Coordinates:\t" << Coords[0] << "\t" << Coords[1] << "\t" << Coords[2] << std::endl;
	std::cout << "Errors Aprior:\t" << ErrorsApriori[0] << "\t" << ErrorsApriori[1] << "\t" << ErrorsApriori[2] << std::endl;
	std::cout << "Errors Aposteriori:\t" << ErrorsAposteriori[0] << "\t" << ErrorsAposteriori[1] << "\t" << ErrorsAposteriori[2] << std::endl;
}
