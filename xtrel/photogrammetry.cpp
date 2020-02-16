#include "pch.h"
#include "photogrammetry.h"

using namespace std;

constexpr const double PI = 3.14159265358979323846;
void fT_rot2angles(const double* R, double* Angles, string System)
{
	string sys1("al-ni-ka");
	string sys2("om-fi-ka");
	string sys3("roll-pitch-yaw");
	

	if (System == sys1) //al-ni-ka
	{
		Angles[1] = acos(R[8]); //kat ni
		if (R[2] / sin(Angles[1]) >= 0) //cwiartka I i II
		{
			Angles[0] = acos(-R[5] / sin(Angles[1])); //kat alfa
		}
		else //cwiartka III i IV
		{
			Angles[0] = 2 * PI - acos(-R[5] / sin(Angles[1])); //kat alfa
		}

		if (R[6] / sin(Angles[1]) >= 0) //cwiartka I i II
		{
			Angles[2] = acos(R[7] / sin(Angles[1])); //kat kappa
		}
		else //cwiartkw III i IV
		{
			Angles[2] = 2 * PI - acos(R[7] / sin(Angles[1])); //kat kappa
		}

		if (Angles[2] > PI)
		{
			Angles[2] = Angles[2] - 2 * PI; //aby kat kappa byl z przedzialu [-180;180]
		}

	}

	if (System == sys2) //om-fi-ka
	{
		double omega, fi, kappa;
		//Trzeba przyjac jakis standard doboru kata fi
		//Zakladamy ze fi: <-pi/2:pi/2>
		//Dlatego mozemy policzyc fi w nastepujacy sposob:
		fi = asin(R[2]);

		//omega
		double b = -R[5] / cos(fi);
		double c = R[8] / cos(fi);

		if (b < 0)
		{
			if (c < 0) // III cwiartka
			{
				omega = PI - asin(b);
			}
			else // IV cwiartka
			{
				omega = asin(b) + 2 * PI;
			}
		}
		else //b>=0
		{
			if (c < 0) //II cwiartka
			{
				omega = PI - asin(b);
			}
			else //I cwiartka
			{
				omega = asin(b);
			}
		}


		//kappa
		b = -R[1] / cos(fi);
		c = R[0] / cos(fi);

		if (b < 0)
		{
			if (c < 0) // III cwiartka
			{
				kappa = PI - asin(b);
			}
			else // IV cwiartka
			{
				kappa = asin(b) + 2 * PI;
			}
		}
		else //b>=0
		{
			if (c < 0) //II cwiartka
			{
				kappa = PI - asin(b);
			}
			else //I cwiartka
			{
				kappa = asin(b);
			}
		}
		Angles[0] = omega;
		Angles[1] = fi;
		Angles[2] = kappa;
	}


	if (System == sys3) //roll-pitch-yaw
	{
		double roll, pitch, yaw;
		//Trzeba przyjac jakis standard doboru kata fi
		//Zakladamy ze pitch: <-pi/2:pi/2>
		//Dlatego mozemy policzyc fi w nastepujacy sposob:
		pitch = asin(-R[6]);

		//roll
		double b = R[7] / cos(pitch);
		double c = R[8] / cos(pitch);


		if (b < 0)
		{
			if (c < 0) // III cwiartka
			{
				roll = PI - asin(b);
			}
			else // IV cwiartka
			{
				roll = asin(b) + 2 * PI;
			}
		}
		else //b>=0
		{
			if (c < 0) //II cwiartka
			{
				roll = PI - asin(b);
			}
			else //I cwiartka
			{
				roll = asin(b);
			}
		}


		//yaw
		b = R[3] / cos(pitch);
		c = R[0] / cos(pitch);

		if (b < 0)
		{
			if (c < 0) // III cwiartka
			{
				yaw = PI - asin(b);
			}
			else // IV cwiartka
			{
				yaw = asin(b) + 2 * PI;
			}
		}
		else //b>=0
		{
			if (c < 0) //II cwiartka
			{
				yaw = PI - asin(b);
			}
			else //I cwiartka
			{
				yaw = asin(b);
			}
		}
		Angles[0] = roll;
		Angles[1] = pitch;
		Angles[2] = yaw;
	}

}

void fT_rot2quaternion(const double* R, double* Q)
{
	//UWAGA! Trzeba rozwazyc przypadek kiedy pod pierwiastkiem jest < 0 => dlatego sa 3 ify
	if ((R[4] > -R[8]) && (R[0] > -R[4]) && (R[0] > -R[8]))
	{
		Q[0] = 0.5*pow(1.0 + R[0] + R[4] + R[8], 0.5);
		Q[1] = (R[7] - R[5]) / (4 * Q[0]);
		Q[2] = (R[2] - R[6]) / (4 * Q[0]);
		Q[3] = (R[3] - R[1]) / (4 * Q[0]);
	}
	else
	{
		if ((R[4] < -R[8]) && (R[0] > R[4]) && (R[0] > -R[8]))
		{
			Q[1] = 0.5*pow(1.0 + R[0] - R[4] - R[8], 0.5);
			Q[0] = (R[7] - R[5]) / (4 * Q[1]);
			Q[2] = (R[3] + R[1]) / (4 * Q[1]);
			Q[3] = (R[2] + R[6]) / (4 * Q[1]);
		}
		else
		{
			if ((R[4] > R[8]) && (R[0] < R[4]) && (R[0] < -R[8]))
			{
				Q[2] = 0.5*pow(1.0 - R[0] + R[4] - R[8], 0.5);
				Q[0] = (R[2] - R[6]) / (4 * Q[2]);
				Q[1] = (R[3] + R[1]) / (4 * Q[2]);
				Q[3] = (R[7] + R[5]) / (4 * Q[2]);
			}
			else
			{
				Q[3] = 0.5*pow(1.0 - R[0] - R[4] + R[8], 0.5);
				Q[0] = (R[3] - R[1]) / (4 * Q[3]);
				Q[1] = (R[2] + R[6]) / (4 * Q[3]);
				Q[2] = (R[7] + R[5]) / (4 * Q[3]);
			}
		}
	}


}

void fT_quaternion2rot(const double * Q, double *R)
{
	double RR[9];

	RR[0] = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
	//RR[0] = pow(Q[0],2) + pow(Q[1],2) - pow(Q[2],2)- pow(Q[3],2) ;
	RR[1] = 2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	RR[2] = 2 * (Q[1] * Q[3] + Q[0] * Q[2]);

	RR[3] = 2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	RR[4] = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
	//RR[4] = pow(Q[0],2) - pow(Q[1],2) + pow(Q[2],2)- pow(Q[3],2) ;
	RR[5] = 2 * (Q[2] * Q[3] - Q[0] * Q[1]);

	RR[6] = 2 * (Q[1] * Q[3] - Q[0] * Q[2]);
	RR[7] = 2 * (Q[2] * Q[3] + Q[0] * Q[1]);
	RR[8] = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
	//RR[8] = pow(Q[0],2) - pow(Q[1],2) - pow(Q[2],2)+ pow(Q[3],2) ;

}

void fT_angles2rot(const double *Angles, double *R, string System)
{
	string sys1("al-ni-ka");
	string sys2("om-fi-ka");
	string sys3("roll-pitch-yaw");

	if (System == sys1)
	{
		double alfa, ni, kappa; // Nazwy dla czytelnosci zapisu		
		alfa = Angles[0];
		ni = Angles[1];
		kappa = Angles[2];

		//Deklaracja skladowych macierzy obrotu
		double Ra[9];
		double Rn[9];
		double Rk[9];

		//Inicjalizacja Ra
		Ra[0] = cos(alfa);		Ra[1] = -sin(alfa);		Ra[2] = 0;
		Ra[3] = sin(alfa);		Ra[4] = cos(alfa);		Ra[5] = 0;
		Ra[6] = 0;				Ra[7] = 0;				Ra[8] = 1;

		//Inicjalizacja Rn
		Rn[0] = 1;				Rn[1] = 0;				Rn[2] = 0;
		Rn[3] = 0;				Rn[4] = cos(ni);		Rn[5] = -sin(ni);
		Rn[6] = 0;				Rn[7] = sin(ni);		Rn[8] = cos(ni);

		//Inicjalizacja Rk
		Rk[0] = cos(kappa);		Rk[1] = -sin(kappa);	Rk[2] = 0;
		Rk[3] = sin(kappa);		Rk[4] = cos(kappa);		Rk[5] = 0;
		Rk[6] = 0;				Rk[7] = 0;				Rk[8] = 1;

		double Rnk[9];
		fT_matMult3(Rn, Rk, Rnk);
		fT_matMult3(Ra, Rnk, R);
		//cout<<"Kontrola: "<<R[0]<<" "<<R[1]<<" "<<R[2]<<" "<<R[3]<<endl;
	}

	if (System == sys2)
	{
		double omega, fi, kappa; // Nazwy dla czytelnosci zapisu		
		omega = Angles[0];
		fi = Angles[1];
		kappa = Angles[2];

		//Deklaracja skladowych macierzy obrotu
		double Ro[9];
		double Rf[9];
		double Rk[9];

		//Inicjalizacja Ro
		Ro[0] = 1;				Ro[1] = 0;				Ro[2] = 0;
		Ro[3] = 0;				Ro[4] = cos(omega);		Ro[5] = -sin(omega);
		Ro[6] = 0;				Ro[7] = sin(omega);		Ro[8] = cos(omega);

		//Inicjalizacja Rf
		Rf[0] = cos(fi);		Rf[1] = 0;				Rf[2] = sin(fi);
		Rf[3] = 0;				Rf[4] = 1;				Rf[5] = 0;
		Rf[6] = -sin(fi);		Rf[7] = 0;				Rf[8] = cos(fi);

		//Inicjalizacja Rk
		Rk[0] = cos(kappa);		Rk[1] = -sin(kappa);	Rk[2] = 0;
		Rk[3] = sin(kappa);		Rk[4] = cos(kappa);		Rk[5] = 0;
		Rk[6] = 0;				Rk[7] = 0;				Rk[8] = 1;

		double Rfk[9];
		fT_matMult3(Rf, Rk, Rfk);
		fT_matMult3(Ro, Rfk, R);
	}

	if (System == sys3)
	{
		double roll, pitch, yaw; // Nazwy dla czytelnosci zapisu		
		roll = Angles[0];
		pitch = Angles[1];
		yaw = Angles[2];

		//Deklaracja skladowych macierzy obrotu
		double Rr[9];
		double Rp[9];
		double Ry[9];

		//Inicjalizacja Rr
		Rr[0] = 1;				Rr[1] = 0;				Rr[2] = 0;
		Rr[3] = 0;				Rr[4] = cos(roll);		Rr[5] = -sin(roll);
		Rr[6] = 0;				Rr[7] = sin(roll);		Rr[8] = cos(roll);

		//Inicjalizacja Rp
		Rp[0] = cos(pitch);		Rp[1] = 0;				Rp[2] = sin(pitch);
		Rp[3] = 0;				Rp[4] = 1;				Rp[5] = 0;
		Rp[6] = -sin(pitch);	Rp[7] = 0;				Rp[8] = cos(pitch);

		//Inicjalizacja Ry
		Ry[0] = cos(yaw);		Ry[1] = -sin(yaw);		Ry[2] = 0;
		Ry[3] = sin(yaw);		Ry[4] = cos(yaw);		Ry[5] = 0;
		Ry[6] = 0;				Ry[7] = 0;				Ry[8] = 1;

		double Rpr[9];
		fT_matMult3(Rp, Rr, Rpr);
		fT_matMult3(Ry, Rpr, R);
	}
}

double fT_xDistCorrection(const double* k, const double *p , double x, double y)
{
	double rr = x*x + y*y;
	return  x * (k[0] * rr + k[1] * pow(rr, 2) + k[2] * pow(rr, 3)) + p[0] * (rr + 2 * pow(x, 2)) + 2 * p[1] * x * y;
}

double fT_yDistCorrection(const double* k, const double *p, double x, double y)
{
	double rr = x*x + y*y;
	return  y * (k[0] * rr + k[1] * pow(rr, 2) + k[2] * pow(rr, 3)) + 2 * p[0] * x * y + p[1] * (rr + 2 * pow(y, 2));
}

void fT_projection(const double * ObjectPoint, const double * Coords, const double * R, const double * Internal, const double * RadialDistortion, const double * TangentialDistortion, double * P)
{
	double Lx, Ly, M;
	double x, y;
	double Fidx, Fidy;
	//std::cout << "in projection: " << Internal[0] << " " << Internal[1] << " " << Internal[2] << std::endl;
	//std::cout << "in projection: " << ObjectPoint[0] << " " << ObjectPoint[1] << " " << ObjectPoint[2] << std::endl;

	Lx = R[0] * (ObjectPoint[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] - Coords[2]);
	Ly = R[1] * (ObjectPoint[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] - Coords[2]);
	M  = R[2] * (ObjectPoint[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] - Coords[2]);
	
	//std::cout << "in projection: " << Lx << " " << Ly << " " << M << std::endl;
	
	x = -Internal[0] * Lx / M;
	y = -Internal[0] * Ly / M;
	Fidx = x;
	Fidy = y;
	//std::cout << "in projection fid: " << Fidx << " " << Fidy << std::endl;
	for (int j = 0; j < 5; j++) //iterative distortion correction addition
	{
		Fidx = x + fT_xDistCorrection(RadialDistortion, TangentialDistortion, Fidx, Fidy);
		Fidy = y + fT_yDistCorrection(RadialDistortion, TangentialDistortion, Fidx, Fidy);
	}
	//std::cout << "in projection fid: " << Fidx << " " << Fidy << std::endl;
	//std::cout << "in projection: " << Internal[1] << " " << Internal[2] << std::endl;
	P[0] = Internal[1] + Fidx;
	P[1] = Internal[2] + Fidy;
	//std::cout << "in projection: " << P[0] << " " << P[1] << std::endl;

}

void fT_projection_thermal(const double * ObjectPoint, const double * Coords, const double * R, const double * Internal, const double * RadialDistortion, const double * TangentialDistortion, const double * ThermalCoeffs, double * P)
{
	double Lx, Ly, M;
	double x, y;
	double Fidx, Fidy;
	//std::cout << "in projection: " << Internal[0] << " " << Internal[1] << " " << Internal[2] << std::endl;
	//std::cout << "in projection: " << ObjectPoint[0] << " " << ObjectPoint[1] << " " << ObjectPoint[2] << std::endl;

	Lx = R[0] * (ThermalCoeffs[0]*ObjectPoint[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ThermalCoeffs[1]*ObjectPoint[2] - Coords[2]);
	Ly = R[1] * (ThermalCoeffs[0]*ObjectPoint[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ThermalCoeffs[1]*ObjectPoint[2] - Coords[2]);
	M = R[2] * (ThermalCoeffs[0]*ObjectPoint[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ThermalCoeffs[1]*ObjectPoint[2] - Coords[2]);

	//std::cout << "in projection: " << Lx << " " << Ly << " " << M << std::endl;

	x = -Internal[0] * Lx / M;
	y = -Internal[0] * Ly / M;
	Fidx = x;
	Fidy = y;
	//std::cout << "in projection fid: " << Fidx << " " << Fidy << std::endl;
	for (int j = 0; j < 5; j++) //iterative distortion correction addition
	{
		Fidx = x + fT_xDistCorrection(RadialDistortion, TangentialDistortion, Fidx, Fidy);
		Fidy = y + fT_yDistCorrection(RadialDistortion, TangentialDistortion, Fidx, Fidy);
	}
	//std::cout << "in projection fid: " << Fidx << " " << Fidy << std::endl;
	//std::cout << "in projection: " << Internal[1] << " " << Internal[2] << std::endl;
	P[0] = Internal[1] + Fidx;
	P[1] = Internal[2] + Fidy;
	//std::cout << "in projection: " << P[0] << " " << P[1] << std::endl;
}

void fT_rotation_differentials(const double * ObjectPoint, const double * Coords, const double * Angles, const double * Internal, const string& Parametrization,
	double * DX, double * DY)
{
	//ObjectPoint : 3 - element vecotr
	//Coords : 3 - element vector , coordinatas of projection cetnter
	//Angles: 3 - element vector
	//Internal: 3 - element vector, internal orientation
	
	//Returns the parital differentials of projected point
	//DX = [dx/dAngle1, dx/dAngle2, dx/dAngle3]
	//DY = [dy/dAngle1, dy/dAngle2, dy/dAngle3]


	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0]*(ObjectPoint[0] - Coords[0]) + R[3]*(ObjectPoint[1] - Coords[1]) + R[6]*(ObjectPoint[2] - Coords[2]);
	double LY = R[1]*(ObjectPoint[0] - Coords[0]) + R[4]*(ObjectPoint[1] - Coords[1]) + R[7]*(ObjectPoint[2] - Coords[2]);
	double M =	R[2]*(ObjectPoint[0] - Coords[0]) + R[5]*(ObjectPoint[1] - Coords[1]) + R[8]*(ObjectPoint[2] - Coords[2]);

	if (Parametrization == "om-fi-ka")
	{
		DX[0] = -(Internal[0] / M)*(((ObjectPoint[1] - Coords[1])*R[8] - (ObjectPoint[2] - Coords[2])*R[5])*(LX / M) - (ObjectPoint[1] - Coords[1])*R[6] + (ObjectPoint[2] - Coords[2])*R[3]); //dx / dOmega
		DY[0] = -(Internal[0] / M)*(((ObjectPoint[1] - Coords[1])*R[8] - (ObjectPoint[2] - Coords[2])*R[5])*(LY / M) - (ObjectPoint[1] - Coords[1])*R[7] + (ObjectPoint[2] - Coords[2])*R[4]); //dy / dOmega

		DX[1] = (Internal[0] / M)*((LX*cos(Angles[2]) - LY * sin(Angles[2]))*(LX / M) + M * cos(Angles[2])); //dx / dPhi
		DY[1] = (Internal[0] / M)*((LX*cos(Angles[2]) - LY * sin(Angles[2]))*(LY / M) - M * sin(Angles[2])); //dy / dPhi

		DX[2] = -Internal[0] * LY / M;//dx / dKappa
		DY[2] = Internal[0] * LX / M; //dy / dKappa
	}

	if (Parametrization == "al-ni-ka") //z-x-z
	{

		//intermiedate calculations;
		double dRAlfa[9] = { -sin(Angles[0])*cos(Angles[2]) - cos(Angles[0])*cos(Angles[1])*sin(Angles[2]), sin(Angles[0])*sin(Angles[2]) - cos(Angles[0])*cos(Angles[1])*cos(Angles[2]), cos(Angles[0])*sin(Angles[1]),
		cos(Angles[0])*cos(Angles[2]) - sin(Angles[0])*cos(Angles[1])*sin(Angles[2]), -cos(Angles[0])*sin(Angles[2]) - sin(Angles[0])*cos(Angles[1])*cos(Angles[2]), sin(Angles[0])*sin(Angles[1]),
		0.0, 0.0, 0.0 };

		double dRNi[9] = { sin(Angles[0])*sin(Angles[1])*sin(Angles[2]), sin(Angles[0])*sin(Angles[1])*cos(Angles[2]), sin(Angles[0])*cos(Angles[1]),
		-cos(Angles[0])*sin(Angles[1])*sin(Angles[2]), -cos(Angles[0])*sin(Angles[1])*cos(Angles[2]), -cos(Angles[0])*cos(Angles[1]),
		cos(Angles[1])*sin(Angles[2]), cos(Angles[1])*cos(Angles[2]), -sin(Angles[1]) };

		double dRKappa[9] = {-cos(Angles[0])*sin(Angles[2]) - sin(Angles[0])*cos(Angles[1])*cos(Angles[2]), -cos(Angles[0])*cos(Angles[2]) + sin(Angles[0])*cos(Angles[1])*sin(Angles[2]), 0.0,
		-sin(Angles[0])*sin(Angles[2]) + cos(Angles[0])*cos(Angles[1])*cos(Angles[2]), -sin(Angles[0])*cos(Angles[2]) - cos(Angles[0])*cos(Angles[1])*sin(Angles[2]), 0.0,
		sin(Angles[1])*cos(Angles[2]), -sin(Angles[1])*sin(Angles[2]), 0.0 };


		DX[0] = (-Internal[0] / (M*M))*((dRAlfa[0]*(ObjectPoint[0] - Coords[0]) + dRAlfa[3]*(ObjectPoint[1] - Coords[1]) + dRAlfa[6]*(ObjectPoint[2] - Coords[2]))*M -
			LX*(dRAlfa[2]*(ObjectPoint[0] - Coords[0]) + dRAlfa[5]*(ObjectPoint[1] - Coords[1]) + dRAlfa[8]*(ObjectPoint[2] - Coords[2]))); //dx / dAlfa

		DY[0] = (-Internal[0] / (M*M))*((dRAlfa[1]*(ObjectPoint[0] - Coords[0]) + dRAlfa[4]*(ObjectPoint[1] - Coords[1]) + dRAlfa[7]*(ObjectPoint[2] - Coords[2]))*M -
				LY*(dRAlfa[2]*(ObjectPoint[0] - Coords[0]) + dRAlfa[5]*(ObjectPoint[1] - Coords[1]) + dRAlfa[8]*(ObjectPoint[2] - Coords[2]))); //dy / dAlfa

		DX[1] = (-Internal[0] / (M*M))*((dRNi[0]*(ObjectPoint[0] - Coords[0]) + dRNi[3]*(ObjectPoint[1] - Coords[1]) + dRNi[6]*(ObjectPoint[2] - Coords[2]))*M -
				LX*(dRNi[2]*(ObjectPoint[0] - Coords[0]) + dRNi[5]*(ObjectPoint[1] - Coords[1]) + dRNi[8]*(ObjectPoint[2] - Coords[2]))); //dx / dNi

		DY[1] = (-Internal[0] / (M*M))*((dRNi[1]*(ObjectPoint[0] - Coords[0]) + dRNi[4]*(ObjectPoint[1] - Coords[1]) + dRNi[7]*(ObjectPoint[2] - Coords[2]))*M -
				LY*(dRNi[2]*(ObjectPoint[0] - Coords[0]) + dRNi[5]*(ObjectPoint[1] - Coords[1]) + dRNi[8]*(ObjectPoint[2] - Coords[2]))); //dy / dNi

		DX[2] = (-Internal[0] / (M*M))*((dRKappa[0]*(ObjectPoint[0] - Coords[0]) + dRKappa[3]*(ObjectPoint[1] - Coords[1]) + dRKappa[6]*(ObjectPoint[2] - Coords[2]))*M -
				LX*(dRKappa[2]*(ObjectPoint[0] - Coords[0]) + dRKappa[5]*(ObjectPoint[1] - Coords[1]) + dRKappa[8]*(ObjectPoint[2] - Coords[2]))); //dx / dKappa

		DY[2] = (-Internal[0] / (M*M))*((dRKappa[1]*(ObjectPoint[0] - Coords[0]) + dRKappa[4]*(ObjectPoint[1] - Coords[1]) + dRKappa[7]*(ObjectPoint[2] - Coords[2]))*M -
				LY*(dRKappa[2]*(ObjectPoint[0] - Coords[0]) + dRKappa[5]*(ObjectPoint[1] - Coords[1]) + dRKappa[8]*(ObjectPoint[2] - Coords[2]))); //dy / dKappa
		
	}

}

void fT_rotation_differentials_thermal(
	const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const double * ThermalCoeffs,
	const string & Parametrization,
	double * DX, double * DY)
{
	//ObjectPoint : 3 - element vecotr
	//Coords : 3 - element vector , coordinatas of projection cetnter
	//Angles: 3 - element vector
	//Internal: 3 - element vector, internal orientation
	//ThermalCoeffs: 2 - element vector, thermal coeffs for X and Z

	//Returns the parital differentials of projected point
	//DX = [dx/dAngle1, dx/dAngle2, dx/dAngle3]
	//DY = [dy/dAngle1, dy/dAngle2, dy/dAngle3]


	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double M = R[2] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);

	if (Parametrization == "om-fi-ka")
	{
		DX[0] = -(Internal[0] / M)*(((ObjectPoint[1] - Coords[1])*R[8] - (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2])*R[5])*(LX / M) - (ObjectPoint[1] - Coords[1])*R[6] + (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2])*R[3]); //dx / dOmega
		DY[0] = -(Internal[0] / M)*(((ObjectPoint[1] - Coords[1])*R[8] - (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2])*R[5])*(LY / M) - (ObjectPoint[1] - Coords[1])*R[7] + (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2])*R[4]); //dy / dOmega

		DX[1] = (Internal[0] / M)*((LX*cos(Angles[2]) - LY * sin(Angles[2]))*(LX / M) + M * cos(Angles[2])); //dx / dPhi
		DY[1] = (Internal[0] / M)*((LX*cos(Angles[2]) - LY * sin(Angles[2]))*(LY / M) - M * sin(Angles[2])); //dy / dPhi

		DX[2] = -Internal[0] * LY / M;//dx / dKappa
		DY[2] = Internal[0] * LX / M; //dy / dKappa
	}

	if (Parametrization == "al-ni-ka") //z-x-z
	{

		//intermiedate calculations;
		double dRAlfa[9] = { -sin(Angles[0])*cos(Angles[2]) - cos(Angles[0])*cos(Angles[1])*sin(Angles[2]), sin(Angles[0])*sin(Angles[2]) - cos(Angles[0])*cos(Angles[1])*cos(Angles[2]), cos(Angles[0])*sin(Angles[1]),
		cos(Angles[0])*cos(Angles[2]) - sin(Angles[0])*cos(Angles[1])*sin(Angles[2]), -cos(Angles[0])*sin(Angles[2]) - sin(Angles[0])*cos(Angles[1])*cos(Angles[2]), sin(Angles[0])*sin(Angles[1]),
		0.0, 0.0, 0.0 };

		double dRNi[9] = { sin(Angles[0])*sin(Angles[1])*sin(Angles[2]), sin(Angles[0])*sin(Angles[1])*cos(Angles[2]), sin(Angles[0])*cos(Angles[1]),
		-cos(Angles[0])*sin(Angles[1])*sin(Angles[2]), -cos(Angles[0])*sin(Angles[1])*cos(Angles[2]), -cos(Angles[0])*cos(Angles[1]),
		cos(Angles[1])*sin(Angles[2]), cos(Angles[1])*cos(Angles[2]), -sin(Angles[1]) };

		double dRKappa[9] = { -cos(Angles[0])*sin(Angles[2]) - sin(Angles[0])*cos(Angles[1])*cos(Angles[2]), -cos(Angles[0])*cos(Angles[2]) + sin(Angles[0])*cos(Angles[1])*sin(Angles[2]), 0.0,
		-sin(Angles[0])*sin(Angles[2]) + cos(Angles[0])*cos(Angles[1])*cos(Angles[2]), -sin(Angles[0])*cos(Angles[2]) - cos(Angles[0])*cos(Angles[1])*sin(Angles[2]), 0.0,
		sin(Angles[1])*cos(Angles[2]), -sin(Angles[1])*sin(Angles[2]), 0.0 };


		DX[0] = (-Internal[0] / (M*M))*((dRAlfa[0] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRAlfa[3] * (ObjectPoint[1] - Coords[1]) + dRAlfa[6] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LX * (dRAlfa[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRAlfa[5] * (ObjectPoint[1] - Coords[1]) + dRAlfa[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dx / dAlfa

		DY[0] = (-Internal[0] / (M*M))*((dRAlfa[1] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRAlfa[4] * (ObjectPoint[1] - Coords[1]) + dRAlfa[7] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LY * (dRAlfa[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRAlfa[5] * (ObjectPoint[1] - Coords[1]) + dRAlfa[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dy / dAlfa

		DX[1] = (-Internal[0] / (M*M))*((dRNi[0] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRNi[3] * (ObjectPoint[1] - Coords[1]) + dRNi[6] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LX * (dRNi[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRNi[5] * (ObjectPoint[1] - Coords[1]) + dRNi[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dx / dNi

		DY[1] = (-Internal[0] / (M*M))*((dRNi[1] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRNi[4] * (ObjectPoint[1] - Coords[1]) + dRNi[7] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LY * (dRNi[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRNi[5] * (ObjectPoint[1] - Coords[1]) + dRNi[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dy / dNi

		DX[2] = (-Internal[0] / (M*M))*((dRKappa[0] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRKappa[3] * (ObjectPoint[1] - Coords[1]) + dRKappa[6] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LX * (dRKappa[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRKappa[5] * (ObjectPoint[1] - Coords[1]) + dRKappa[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dx / dKappa

		DY[2] = (-Internal[0] / (M*M))*((dRKappa[1] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRKappa[4] * (ObjectPoint[1] - Coords[1]) + dRKappa[7] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))*M -
			LY * (dRKappa[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + dRKappa[5] * (ObjectPoint[1] - Coords[1]) + dRKappa[8] * (ObjectPoint[2]*ThermalCoeffs[1] - Coords[2]))); //dy / dKappa

	}
}

void fT_projection_center_differentials(const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const string & Parametrization,
	double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] - Coords[2]);
	double M =  R[2] * (ObjectPoint[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] - Coords[2]);

	DX[0] = (-Internal[0] / (M*M))*(R[2]*LX - R[0]*M); //dx / dX0
	DX[1] = (-Internal[0] / (M*M))*(R[5]*LX - R[3]*M); //dx / dY0
	DX[2] = (-Internal[0] / (M*M))*(R[8]*LX - R[6]*M); //dx / dZ0

	DY[0] = (-Internal[0] / (M*M))*(R[2]*LY - R[1]*M); //dy / dX0
	DY[1] = (-Internal[0] / (M*M))*(R[5]*LY - R[4]*M); //dy / dY0
	DY[2] = (-Internal[0] / (M*M))*(R[8]*LY - R[7]*M); //dy / dZ0

}

void fT_projection_center_differentials_thermal(
	const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const double * ThermalCoeffs,
	const string & Parametrization,
	double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double M = R[2] * (ObjectPoint[0]*ThermalCoeffs[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);

	DX[0] = (-Internal[0] / (M*M))*(R[2] *LX - R[0] * M); //dx / dX0
	DX[1] = (-Internal[0] / (M*M))*(R[5] *LX - R[3] * M); //dx / dY0
	DX[2] = (-Internal[0] / (M*M))*(R[8] *LX - R[6] * M); //dx / dZ0

	DY[0] = (-Internal[0] / (M*M))*(R[2] * LY - R[1] * M); //dy / dX0
	DY[1] = (-Internal[0] / (M*M))*(R[5] * LY - R[4] * M); //dy / dY0
	DY[2] = (-Internal[0] / (M*M))*(R[8] * LY - R[7] * M); //dy / dZ0
}

void fT_object_point_differentials(const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const string & Parametrization,
	double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] - Coords[2]);
	double M =	R[2] * (ObjectPoint[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] - Coords[2]);

	DX[0] = (Internal[0] / (M*M))*(R[2] * LX - R[0] * M); //dx / dX
	DX[1] = (Internal[0] / (M*M))*(R[5] * LX - R[3] * M); //dx / dY
	DX[2] = (Internal[0] / (M*M))*(R[8] * LX - R[6] * M); //dx / dZ

	DY[0] = (Internal[0] / (M*M))*(R[2] * LY - R[1] * M); //dy / dX
	DY[1] = (Internal[0] / (M*M))*(R[5] * LY - R[4] * M); //dy / dY
	DY[2] = (Internal[0] / (M*M))*(R[8] * LY - R[7] * M); //dy / dZ

}

void fT_object_point_differentials_thermal(
	const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const double * ThermalCoeffs,
	const string & Parametrization,
	double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double M = R[2] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);

	DX[0] = (Internal[0] / (M*M))*(R[2] * ThermalCoeffs[0] * LX - R[0] * ThermalCoeffs[0] * M); //dx / dX
	DX[1] = (Internal[0] / (M*M))*(R[5] * LX - R[3] * M); //dx / dY
	DX[2] = (Internal[0] / (M*M))*(R[8] * ThermalCoeffs[1] * LX - R[6] * ThermalCoeffs[1] * M); //dx / dZ

	DY[0] = (Internal[0] / (M*M))*(R[2] * ThermalCoeffs[0] * LY - R[1] * ThermalCoeffs[0] * M); //dy / dX
	DY[1] = (Internal[0] / (M*M))*(R[5] * LY - R[4] * M); //dy / dY
	DY[2] = (Internal[0] / (M*M))*(R[8] * ThermalCoeffs[1] * LY - R[7] * ThermalCoeffs[1] * M); //dy / dZ
}

void fT_thermal_differentials(
	const double * ObjectPoint,
	const double * Coords,
	const double * Angles,
	const double * Internal,
	const double * ThermalCoeffs,
	const string & Parametrization,
	double * DX, double * DY)
{
	//ObjectPoint : 3 - element vecotr
	//Coords : 3 - element vector , coordinatas of projection cetnter
	//Angles: 3 - element vector
	//Internal: 3 - element vector, internal orientation
	//ThermalCoeffs: 2 - element vector, thermal coeffs for X and Z
	
	//Returns the parital differentials of projected point
	//DX = [dx/dTx, dx/dTz]
	//DY = [dy/dTx, dy/dTz]

	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double M = R[2] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);

	DX[0] = (Internal[0] / (M*M)) *(LX*R[2] * ObjectPoint[0] - R[0] * ObjectPoint[0] * M);
	DX[1] = (Internal[0] / (M*M)) *(LX*R[8] * ObjectPoint[2] - R[6] * ObjectPoint[2] * M);
	DY[0] = (Internal[0] / (M*M)) *(LY*R[2] * ObjectPoint[0] - R[1] * ObjectPoint[0] * M);
	DY[1] = (Internal[0] / (M*M)) *(LY*R[8] * ObjectPoint[2] - R[7] * ObjectPoint[2] * M);
}

void fT_distortion_differentials_r(const double * Internal, double x, double y, double *DXr, double *DYr)
{
	//function return the differentials with respect to distortion coefficients
	//DX and DY should be 5 element arrays
	//Internal is a pointer to 3 element array
	//x, and y point to measured image coordinates
	
	//conversion to fiducial coordinates:
	double xt = x - Internal[1];
	double yt = y - Internal[2];

	double r2 = xt * xt + yt * yt;
	double r4 = r2 * r2;
	double r6 = r4 * r2;

	DXr[0] = xt * r2;
	DXr[1] = xt * r4;
	DXr[2] = xt * r6;

	DYr[0] = yt * r2;
	DYr[1] = yt * r4;
	DYr[2] = yt * r6;

}

void fT_distortion_differentials_t(const double * Internal, double x, double y, double *DXt, double *DYt)
{
	//function return the differentials with respect to distortion coefficients
	//DX and DY should be 5 element arrays
	//Internal is a pointer to 3 element array
	//x, and y point to measured image coordinates

	//conversion to fiducial coordinates:
	double xt = x - Internal[1];
	double yt = y - Internal[2];

	double r2 = xt * xt + yt * yt;

	DXt[0] = r2 + 2 * xt * xt;
	DXt[1] = 2 * xt * yt;

	DYt[0] = 2 * xt * yt;
	DYt[1] = r2 + 2 * yt * yt;

}

void fT_internal_differentials(const double * ObjectPoint, const double * Coords, const double * Angles, const string & Parametrization, double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] - Coords[2]);
	double M = R[2] * (ObjectPoint[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] - Coords[2]);

	DX[0] = - LX / M; //dx/dck
	DX[1] = 1.0; //dx/dx0
	DX[2] = 0.0; //dx/dy0

	DY[0] = -LY/ M; //dy/dck
	DY[1] = 0.0; //dy/dx0
	DY[2] = 1.0; //dy/dy0


}

void fT_internal_differentials_thermal(const double * ObjectPoint, const double * Coords, const double * Angles, const double * ThermalCoeffs, const string & Parametrization, double * DX, double * DY)
{
	double R[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
	fT_angles2rot(Angles, R, Parametrization);

	double LX = R[0] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[3] * (ObjectPoint[1] - Coords[1]) + R[6] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double LY = R[1] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[4] * (ObjectPoint[1] - Coords[1]) + R[7] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);
	double M = R[2] * (ObjectPoint[0] * ThermalCoeffs[0] - Coords[0]) + R[5] * (ObjectPoint[1] - Coords[1]) + R[8] * (ObjectPoint[2] * ThermalCoeffs[1] - Coords[2]);

	DX[0] = -LX / M; //dx/dck
	DX[1] = 1.0; //dx/dx0
	DX[2] = 0.0; //dx/dy0

	DY[0] = -LY / M; //dy/dck
	DY[1] = 0.0; //dy/dx0
	DY[2] = 1.0; //dy/dy0
}

void fT_vectorTrans3(double *A, double* v, double *vt)
{
	//Transformacja wektora
	//Funkcja do mnozenia macierzy o wymiarach 3x3 przez wektor 3x1
	//vt = A*v
	vt[0] = A[0] * v[0] + A[1] * v[1] + A[2] * v[2];
	vt[1] = A[3] * v[0] + A[4] * v[1] + A[5] * v[2];
	vt[2] = A[6] * v[0] + A[7] * v[1] + A[8] * v[2];
}

void fT_vectorTrans3(double *A, double *b, double* v, double *vt)
{
	//Transformacja wektora
	//Funkcja do mnozenia macierzy o wymiarach 3x3 przez wektor 3x1
	//vt = A*v+b
	vt[0] = A[0] * v[0] + A[1] * v[1] + A[2] * v[2] + b[0];
	vt[1] = A[3] * v[0] + A[4] * v[1] + A[5] * v[2] + b[1];
	vt[2] = A[6] * v[0] + A[7] * v[1] + A[8] * v[2] + b[2];
}

void fT_transpose3(double *A, double *AT)
{
	//Transpozycja macierzy 3x3;
	AT[0] = A[0];
	AT[1] = A[3];
	AT[2] = A[6];

	AT[3] = A[1];
	AT[4] = A[4];
	AT[5] = A[7];

	AT[6] = A[2];
	AT[7] = A[5];
	AT[8] = A[8];
}

void fT_matTranspose(double *A, int *sizeA, double *AT)
{
	//transpoza macierzy A
	//wynik zapisywany jest w AT
	//wymiary macierzy A podane w sizeA jako [wiersze kolumny]
	for (int i = 0; i < sizeA[0]; i++)
	{
		for (int j = 0; j < sizeA[1]; j++)
		{
			AT[sizeA[0] * j + i] = A[sizeA[1] * i + j];
		}
	}
}


void fT_matInv3(double*A, double *B)
{
	//funkcja oblica odwrotnosc macierzy 3 x 3;
	double Mi[9]; //Macierz minorow
	double DET = A[0] * A[4] * A[8] + A[1] * A[5] * A[6] + A[2] * A[3] * A[7] - A[2] * A[4] * A[6] - A[1] * A[3] * A[8] - A[0] * A[5] * A[7];

	Mi[0] = A[4] * A[8] - A[5] * A[7];
	Mi[1] = A[3] * A[8] - A[5] * A[6];
	Mi[2] = A[3] * A[7] - A[4] * A[6];

	Mi[3] = A[1] * A[8] - A[2] * A[7];
	Mi[4] = A[0] * A[8] - A[2] * A[6];
	Mi[5] = A[0] * A[7] - A[1] * A[6];

	Mi[6] = A[1] * A[5] - A[2] * A[4];
	Mi[7] = A[0] * A[5] - A[2] * A[3];
	Mi[8] = A[0] * A[4] - A[1] * A[3];

	B[0] = Mi[0] / DET;
	B[1] = -Mi[1] / DET;
	B[2] = Mi[2] / DET;

	B[3] = -Mi[3] / DET;
	B[4] = Mi[4] / DET;
	B[5] = -Mi[5] / DET;

	B[6] = Mi[6] / DET;
	B[7] = -Mi[7] / DET;
	B[8] = Mi[8] / DET;
}



void fT_matMult3(double *A, double *B, double *C)
{
	//Funkcja oblicza iloczyn macierzy C=A*B
	//Macierze musza miec wymar 3 x 3 czyli musza byc
	//typu double[9]
	C[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	C[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	C[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

	C[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	C[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	C[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

	C[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	C[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	C[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

void fT_matMult(double *A, double *B, int* sizeA, int* sizeB, double *C)
{
	//funkcja oblicza iloczyn macierzy A i B
	//sizeA zawiera wymiarzy macierzy A [WIERSZE KOLUMNY]
	//sizeB zawiera wymiarzy macierzy B [WIERSZE KOLUMNY]
	//macierz C musi miec odpowiedni rozmiar: double C[sizeA[0]*sizeB[1]]
	//ponadto sizeA[1] == sizeB[0]
	for (int i = 0; i < sizeA[0]; i++) //dla kazdego wiersza macierzy A
	{
		for (int j = 0; j < sizeB[1]; j++) //dokonuj mnozenia z odpowiednia kolumna macierzy B
		{
			int PositionInC = sizeB[1] * i + j; //obliczenie pozycji w macierzy C
			double ValueInC = 0;
			for (int k = 0; k < sizeA[1]; k++)
			{
				ValueInC += A[i*sizeA[1] + k] * B[k*sizeB[1] + j];
			}
			C[PositionInC] = ValueInC;
		}
	}
}

void fT_dispMatrix3(double *A)
{
	//Funkcja wyswietla macierz 3x3
	for (int i = 0; i < 9; i++)
	{
		cout << A[i];
		if ((i + 1) % 3 == 0)
		{
			cout << endl;
		}
		else
		{
			cout << " ";
		}
	}
}