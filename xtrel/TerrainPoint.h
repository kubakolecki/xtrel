#pragma once

class TerrainPoint
{
public:
	TerrainPoint(void);
	TerrainPoint(std::string N, double WspX, double WspY, double WspZ);
	TerrainPoint(std::string N, double WspX, double WspY, double WspZ, short T);
	TerrainPoint(std::string N, double WspX, double WspY, double WspZ, double ErrX, double ErrY, double ErrZ);
	TerrainPoint(std::string N, double WspX, double WspY, double WspZ, double ErrX, double ErrY, double ErrZ, short T);

	std::string Name = "1"; //name
	double Coords[3] = { 0.0, 0.0, 0.0 };
	double ErrorsApriori[3] = { 0.0, 0.0, 0.0 };
	double ErrorsAposteriori[3] = { 0.0, 0.0, 0.0 };
	double V[3] = { 0.0, 0.0, 0.0 }; //corrections;
	
	int Rays{ 0 };

	short Type = 0; //Typ 0 - tie, 3 - XYZ controll point, 4 - Check punkt, 9 - Geodetic controll point
	bool Use = true;

	void print(void);
};
