#pragma once
#include "Camera.h"
#include <string>

void fT_rot2angles(const double* R, double* Angles, std::string System);
void fT_rot2quaternion(const double* R, double* Q);
void fT_quaternion2rot(const double * Q, double *R);
void fT_angles2rot(const double *Angles, double *R, std::string System);
double fT_xDistCorrection(const double* k, const double *p, double x, double y);
double fT_yDistCorrection(const double* k, const double *p, double x, double y);
void fT_projection(const double *ObjectPoint, const double *Coords, const double *R,
	const double* Internal, const double *RadialDistortion, const double *TangentialDistortion, double *P );
void fT_projection_thermal(const double *ObjectPoint, const double *Coords, const double *R,
	const double* Internal, const double *RadialDistortion, const double *TangentialDistortion, const double* ThermalCoeffs, double *P);
void fT_rotation_differentials(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const string& Parametrization, double *DX, double *DY);
void fT_rotation_differentials_thermal(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const double* ThermalCoeffs, const string& Parametrization, double *DX, double *DY);
void fT_projection_center_differentials(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const string& Parametrization, double *DX, double *DY);
void fT_projection_center_differentials_thermal(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const double* ThermalCoeffs, const string& Parametrization,  double *DX, double *DY);
void fT_object_point_differentials(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const string& Parametrization, double *DX, double *DY);
void fT_object_point_differentials_thermal(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const double* ThermalCoeffs, const string& Parametrization, double *DX, double *DY);
void fT_thermal_differentials(const double *ObjectPoint, const double *Coords, const double* Angles, const double * Internal, const double* ThermalCoeffs, const string& Parametrization, double *DX, double *DY);
void fT_distortion_differentials_r(const double * Internal, double x, double y, double *DXr, double *DYr);
void fT_distortion_differentials_t(const double * Internal, double x, double y, double *DXt, double *DYt);
void fT_internal_differentials(const double * ObjectPoint, const double * Coords, const double * Angles, const string & Parametrization, double * DX, double * DY);
void fT_internal_differentials_thermal(const double * ObjectPoint, const double * Coords, const double * Angles, const double* ThermalCoeffs, const string & Parametrization, double * DX, double * DY);
void fT_vectorTrans3(const double *A, const double* v, double *vt);
void fT_vectorTrans3(const double *A, const double* b, const double* v, double *vt);
void fT_transpose3(double *A, double *AT);
void fT_matTranspose(double *A, int *sizeA, double *AT);
void fT_matInv3(double*A, double *B);
void fT_matMult3(double *A, double *B, double *C);
void fT_matMult(double *A, double *B, int* sizeA, int* sizeB, double *C);
void fT_dispMatrix3(const double *A);