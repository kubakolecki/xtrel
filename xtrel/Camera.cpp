#include "pch.h"
#include "Camera.h"
using namespace std;

Camera::Camera(void)
{
}

Camera::Camera (const char * Filename)
{
	std::ifstream Plik;
	Plik.open(Filename);
	if (Plik)
	{
		Valid = 1;
		Name = Filename;
		std::string Linia;
		std::string Opis;
		double M; //blad
		
		//Linia 1
		getline(Plik, Linia, '\n');
		std::istringstream LiniaStream(Linia);
		LiniaStream >> Opis;
		
		//Linia 2
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		Description = Linia; //the whole line

		//Linia 3
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Size

		//Linia 4
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> W >> H;

		//Linia 5
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Interior

		//Linia 6
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> InternalOrientation[0] >> M;

		//Linia 7
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> InternalOrientation[1] >> M;

		//Linia 8
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> InternalOrientation[2] >> M;

		//Linia 9
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Radial distortion

		//Linia 10
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> RadialDistortion[0] >> M;

		//Linia 11
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> RadialDistortion[1] >> M;

		//Linia 12
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> RadialDistortion[2] >> M;

		//Linia 13
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Tangential distortion

		//Linia 14
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> TangentialDistortion[0] >> M;

		//Linia 15
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> TangentialDistortion[1] >> M;

		//Linia 16
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Y scaling

		//Linia 17
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> y_scaling >> M;

		//Linia 18
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //skewness

		//Linia 19
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> skewness >> M;

		DataScalling = 1.0 / InternalOrientation[0];

		//reading additional data
		
		string camera_serial_number;
		string lens_name;
		string lens_serial_number;

		//Linia 20
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Additional_data:

		//Linia 21
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Pixel_size[mm]:

		//Linia 22
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> PixelSize;

		//Linia 23
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Camera_serial_number:

		//Linia 24
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		camera_serial_number = Linia;

		//Linia 25
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Lens_name:

		//Linia 26
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		lens_name = Linia;

		//Linia 27
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Lens_nominal_focal_length[mm]:

		//Linia 28
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> LensNominalFocalLength;

		//Linia 30
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		LiniaStream >> Opis; //Lens_serial_number:

		//Linia 31
		getline(Plik, Linia, '\n');
		LiniaStream.clear();
		LiniaStream.str(Linia);
		lens_serial_number = Linia;


		if (lens_name.length() > 2)
		{
			if (lens_name.at(0) == '\'' && lens_name.back() == '\'')
			{
				LensName = lens_name.substr(1, lens_name.length() - 2);
			}
		}

		if (lens_serial_number.length() > 2)
		{
			if (lens_serial_number.at(0) == '\'' && lens_serial_number.back() == '\'')
			{
				LensSerialNumber = lens_serial_number.substr(1, lens_serial_number.length() - 2);
			}
		}

		if (camera_serial_number.length() > 2)
		{
			if (camera_serial_number.at(0) == '\'' && camera_serial_number.back() == '\'')
			{
				CameraSerialNumber = camera_serial_number.substr(1, camera_serial_number.length() - 2);
			}
		}
		


	}
	else
	{
		std::cout << "cannot open " << std::string(Filename) << std::endl;
		Valid = 0;
	}
	Plik.close();
}

Camera::Camera(double* io, double *k, double *p, double *a)
{
	//To jest konstruktor, ktory tworzy plik kamery na podstawie pliku obrazu
	//ktory moze byc wczytany z pliku ida
	Name = "";
	Description = "";
	W = 0;
	H = 0;
	InternalOrientation[0] = io[0];
	InternalOrientation[1] = io[1];
	InternalOrientation[2] = io[2];
	RadialDistortion[0] = k[0];
	RadialDistortion[1] = k[1];
	RadialDistortion[2] = k[2];
	TangentialDistortion[0] = p[0];
	TangentialDistortion[1] = p[1];
	y_scaling = a[0];
	skewness = a[1];
	DataScalling = 1.0 / InternalOrientation[0];
	Valid = 1;
}

Camera::~Camera(void)
{
}

double Camera::getRadialCorrectionX(double x, double y) const noexcept
{
	double rr = x * x + y * y;
	double dx = x * (RadialDistortion[0] * rr + RadialDistortion[1] * rr * rr + RadialDistortion[2] * rr * rr * rr);
	dx = -dx;
	return dx;
}

double Camera::getRadialCorrectionY(double x, double y) const noexcept
{
	double rr = x * x + y * y;
	double dy = y * (RadialDistortion[0] * rr + RadialDistortion[1] * rr * rr + RadialDistortion[2] * rr * rr * rr);
	dy = -dy;
	return dy;
}

double Camera::getTangentialCorrectionX(double x, double y) const noexcept
{
	double rr = x * x + y * y;
	double dx = TangentialDistortion[0] * (rr + 2.0*x*x) + 2.0 * TangentialDistortion[1] * x * y;
	dx = -dx;
	return dx;
}

double Camera::getTangentialCorrectionY(double x, double y) const noexcept
{
	double rr = x * x + y * y;
	double dy = 2.0 * TangentialDistortion[0] * x * y + TangentialDistortion[1] * (rr + 2.0*y*y);
	dy = -dy;
	return dy;
}

double Camera::getTotalDistortionCorrectionX(double x, double y) const noexcept
{
	double dx = getRadialCorrectionX(x, y) + getTangentialCorrectionX(x, y);
	return dx;
}

double Camera::getTotalDistortionCorrectionY(double x, double y) const noexcept
{
	double dy = getRadialCorrectionX(x, y) + getTangentialCorrectionX(x, y);
	return dy;
}

void Camera::scale()
{
	for (int i : { 0, 1, 2 })
	{
		InternalOrientation[i] *= DataScalling;
		InternalOrientationStdDev[i] *= DataScalling;
	}
	double s = DataScalling * DataScalling;
	RadialDistortion[0] /= s;
	RadialDistortion[1] /= (s*s);
	RadialDistortion[2] /= (s*s*s);
	RadialDistortionStdDev[0] /= s;
	RadialDistortionStdDev[1] /= (s*s);
	RadialDistortionStdDev[2] /= (s*s*s);

	TangentialDistortion[0] /= DataScalling;
	TangentialDistortion[1] /= DataScalling;
	TangentialDistortionStdDev[0] /= DataScalling;
	TangentialDistortionStdDev[1] /= DataScalling;
}

void Camera::scale_back()
{
	for (int i : { 0, 1, 2 })
	{
		InternalOrientation[i] /= DataScalling;
		InternalOrientationStdDev[i] /= DataScalling;
	}
	double s = DataScalling * DataScalling;
	RadialDistortion[0] *= s;
	RadialDistortion[1] *= (s*s);
	RadialDistortion[2] *= (s*s*s);
	RadialDistortionStdDev[0] *= s;
	RadialDistortionStdDev[1] *= (s*s);
	RadialDistortionStdDev[2] *= (s*s*s);

	TangentialDistortion[0] *= DataScalling;
	TangentialDistortion[1] *= DataScalling;
	TangentialDistortionStdDev[0] *= DataScalling;
	TangentialDistortionStdDev[1] *= DataScalling;
}

void Camera::wypisz(void)
{
	std::cout << "\nCamera data: " <<std::endl;
	std::cout << "Name: " << Name <<std::endl;
	std::cout << "Description: " << Description <<std::endl;
	std::cout << "W: "<<W<<std::endl;
	std::cout << "H: "<<H<<std::endl;
	std::cout << "ck: " << InternalOrientation[0] <<std::endl;
	std::cout << "x0: " << InternalOrientation[1] <<std::endl;
	std::cout << "y0: " << InternalOrientation[2] <<std::endl;
	std::cout << "k1: " << RadialDistortion[0] <<std::endl;
	std::cout << "k2: " << RadialDistortion[1] <<std::endl;
	std::cout << "k3: " << RadialDistortion[2] <<std::endl;
	std::cout << "p1: " << TangentialDistortion[0] <<std::endl;
	std::cout << "p2: " << TangentialDistortion[1] <<std::endl;
	std::cout << "y_scaling: "<< y_scaling <<std::endl;
	std::cout << "skewness: "<<skewness <<std::endl;
	std::cout << "Pixel_size: " << PixelSize << std::endl;
	std::cout << "Camera serial number: " << CameraSerialNumber << std::endl;
	std::cout << "Lens nominal focal lenght: " << LensNominalFocalLength << std::endl;
	std::cout << "Lens name: " << LensName << std::endl;
	std::cout << "Lens serial number: " << LensSerialNumber << std::endl;
}

void Camera::writeToFile(string & filename) const
{
	ofstream S(filename);
	S << "Name:\n";
	S << Name << "\n";
	S << "Size:\n";
	S << W << " " << H << "\n";
	S << "Interior:\n";
	S << std::fixed << setprecision(3) << InternalOrientation[0] << " 0.100\n";
	S << std::fixed << setprecision(3) << InternalOrientation[1] << " 0.100\n";
	S << std::fixed << setprecision(3) << InternalOrientation[2] << " 0.100\n";
	S << "Radial distortion:\n";
	S << std::scientific << setprecision(6) << RadialDistortion[0] << " 1.0e-010\n";
	S << std::scientific << setprecision(6) << RadialDistortion[1] << " 1.0e-017\n";
	S << std::scientific << setprecision(6) << RadialDistortion[2] << " 1.0e-028\n";
	S << "Tangential distortion:\n";
	S << std::scientific << setprecision(6) << TangentialDistortion[0] << " 1.0e-010\n";
	S << std::scientific << setprecision(6) << TangentialDistortion[1] << " 1.0e-010\n";
	S << "Y scaling:\n";
	S << std::fixed << setprecision(3) << y_scaling << " 0.0001\n";
	S << "Skewness of axes:\n";
	S << std::fixed << setprecision(3) << skewness << " 0.0001\n";
	S.close();
}
