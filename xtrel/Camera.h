#pragma once
using namespace std;
class Camera
{
public:
	Camera(void);
	Camera(const char * Filename);
	Camera(double* io, double *k, double *p, double *a);
	~Camera(void);
	std::string Name{ "" };
	std::string Description{ "" };
	int W{ 6000 };
	int H{ 4000 };

	double InternalOrientation[3] = { 5000.0, 0.0, 0.0 }; //ck, x0, y0 [px]
	double RadialDistortion[3] = { 0.0, 0.0, 0.0 }; //k1, k2, k3
	double TangentialDistortion[2] = { 0.0, 0.0 }; //p1, p2

	double InternalOrientationStdDev[3] = { 0.0, 0.0, 0.0 }; //ck, x0, y0
	double RadialDistortionStdDev[3] = { 0.0, 0.0, 0.0 }; //k1, k2, k3
	double TangentialDistortionStdDev[2] = { 0.0, 0.0}; //p1, p2

	double y_scaling{ 0.0 };
	double skewness{ 0.0 };
	double DataScalling{ 1.0 }; //scalling coefficient
	short Valid;
	
	double PixelSize{ 0.00376 };
	std::string CameraSerialNumber{""};
	float LensNominalFocalLength{ 40 };
	std::string LensName{ "" };
	std::string LensSerialNumber{""};

	double getRadialCorrectionX(double x, double y);
	double getRadialCorrectionY(double x, double y);
	double getTangentialCorrectionX(double x, double y);
	double getTangentialCorrectionY(double x, double y);
	double getTotalDistortionCorrectionX(double x, double y);
	double getTotalDistortionCorrectionY(double x, double y);

	void scale();
	void scale_back();
	void wypisz(void);
	void writeToFile(string& filename) const;
};
