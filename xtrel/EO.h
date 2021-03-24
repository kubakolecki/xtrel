#pragma once
class EO
{
public:
	EO();
	EO(double *coords, double* angles, double* coords_accuracy, double* angles_accuracy);
	~EO();
	double Coords[3] = { 0.0,0.0,0.0 };
	double CoordsErrorsApriori[3] = { 0.01, 0.01,0.01 };
	double CoordsErrorsAposteriori[3] = { 0.01, 0.01,0.01 };

	double Angles[3] = { 0.0, 1.5707963267948966, 0.0 }; //all math is to be done in radians!
	double AnglesErrorsApriori[3] = { 0.00005, 0.00005,0.00005 };
	double AnglesErrorsAposteriori[3] = { 0.00005, 0.00005,0.00005 };


	double R[9] = {	1.00000,   0.00000,   0.00000,
					0.00000,   0.00000, - 1.00000,
					0.00000,   1.00000,   0.00000 };
	double Quaternion[4];
	std::string RotParametrization = "al-ni-ka";

	void set_coords(double* Coords_);
	void set_coords_accuracy(double* CoordsAccuracy_);
	void set_angles(double* Angles_);
	void set_angles_accuracy(double *AnglesAccuracy_);

private:

};

