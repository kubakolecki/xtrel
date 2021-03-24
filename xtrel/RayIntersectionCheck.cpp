#include "pch.h"
#include "RayIntersectionCheck.h"
#include "photogrammetry.h"
#include <iostream> //only for debug

RayIntersectionCheck::RayIntersectionCheck(
	const TerrainPoint & objectPoint,
	const std::vector<ImagePoint>& imagePoints,
	const ImageDataContainer & imageData)
{
	std::cout << "Ray checking for object point: " << objectPoint.Name << " type = " << objectPoint.Type << std::endl;
	
	double sumOfDistances{ 0.0 };
	
	for (const auto &ip : imagePoints)
	{	
		// 1. DETERMINING THE PARAMETRIC EQUATION OF STRAIGHT LINE IN 3D SPACE
		
		// 1.a Calulating image plane coordinates in the camera frame
		std::string cameraName = imageData.DataImages.at(ip.ImageName).CameraName;
		double x = ip.X - imageData.DataCameras.at(cameraName).InternalOrientation[1];
		double y = ip.Y - imageData.DataCameras.at(cameraName).InternalOrientation[2];

		// 1.b Calculating undistorted coordiantes
		double distCorrectionX = imageData.DataCameras.at(cameraName).getTotalDistortionCorrectionX(x, y);
		double distCorrectionY = imageData.DataCameras.at(cameraName).getTotalDistortionCorrectionY(x, y);
		x += distCorrectionX;
		y += distCorrectionY;

		std::cout << ip.X << " " << ip.Y << " " << imageData.DataCameras.at(cameraName).InternalOrientation[0] << " " << imageData.DataCameras.at(cameraName).InternalOrientation[1] << " " << imageData.DataCameras.at(cameraName).InternalOrientation[2] << " ";
		std::cout << distCorrectionX << " " << distCorrectionY << std::endl;

		// 1.c Calculating vector parallel to the ray
		double vCameraFrame[3] = { x, y, -imageData.DataCameras.at(cameraName).InternalOrientation[0] };
		double vObjectFrame[3] = { 0.0, 0.0, 0.0 };
		double R[9];
		fT_angles2rot(imageData.DataImages.at(ip.ImageName).EOApproximated.Angles, R, imageData.DataImages.at(ip.ImageName).EOApproximated.RotParametrization);
		fT_vectorTrans3(R, vCameraFrame, vObjectFrame);

		// 1.d Calculating closest point
		double vObjectFrameLenSq = vObjectFrame[0] * vObjectFrame[0] + vObjectFrame[1] * vObjectFrame[1] + vObjectFrame[2] * vObjectFrame[2];
		double a = 
			vObjectFrame[0] * objectPoint.Coords[0] +
			vObjectFrame[1] * objectPoint.Coords[1] +
			vObjectFrame[2] * objectPoint.Coords[2];
		double b =
			vObjectFrame[0] * imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[0] +
			vObjectFrame[1] * imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[1] +
			vObjectFrame[2] * imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[2];

		double t = (a - b) / vObjectFrameLenSq;

		double pointOnRay[3] = {
			imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[0] + t * vObjectFrame[0],
			imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[1] + t * vObjectFrame[1],
			imageData.DataImages.at(ip.ImageName).EOApproximated.Coords[2] + t * vObjectFrame[2]
		};


		// 2. CALCULATING THE SHORTEST POINT-TO-RAY VECTOR AND PUSHIG TO RESULTS

		//2.a Calculating errors
		array<double, 3> e = {
			objectPoint.Coords[0] - pointOnRay[0],
			objectPoint.Coords[1] - pointOnRay[1],
			objectPoint.Coords[2] - pointOnRay[2]
		};

		//2.b Calculating distnce
		double distance = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);

		//2.c Pushing
		Errors.push_back(std::make_pair(ip.ImageName, e));
		Distances.push_back(std::make_pair(ip.ImageName, distance));

		sumOfDistances += distance;
	}

	MeanDistance = sumOfDistances / imagePoints.size();

	//sorting starting from the longest distance
	std::sort(Distances.begin(), Distances.end(),
		[&](std::pair<std::string, double>&a, std::pair<std::string, double>&b)
	{
		return a.second > b.second;
	});

	std::cout << "mean distance: " << MeanDistance << " max distance: "<< Distances.at(0).second << std::endl;
}
