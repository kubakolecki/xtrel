#include "pch.h"
#include "EO.h"
#include "photogrammetry.h"


EO::EO()
{
	set_angles(Angles);
}

EO::EO(double * coords, double * angles, double * coords_accuracy, double * angles_accuracy)
{
	set_coords(coords);
	set_angles(angles);
	set_coords_accuracy(coords_accuracy);
	set_angles_accuracy(angles_accuracy);
}


EO::~EO()
{

}

void EO::set_coords(double * Coords_)
{
	for (int i : {0, 1, 2}) Coords[i] = Coords_[i];
}

void EO::set_coords_accuracy(double * CoordsAccuracy_)
{
	for (int i : {0, 1, 2}) CoordsErrorsApriori[i] = CoordsAccuracy_[i];
}

void EO::set_angles(double * Angles_)
{
	//set angles as well as rotation matrix and quaternion
	for (int i : {0, 1, 2}) Angles[i] = Angles_[i];
	fT_angles2rot(Angles, R, RotParametrization);
	fT_rot2quaternion(R, Quaternion);
}

void EO::set_angles_accuracy(double * AnglesAccuracy_)
{
	for (int i : {0, 1, 2}) AnglesErrorsApriori[i] = AnglesAccuracy_[i];
}