#include "pch.h"
#include "DataInserter.h"

#include "ResidualProjectionObjPt.h"
#include "ResidualProjectionObjPtFixed.h"
#include "ResidualObjectPoint.h"
#include "ResidualProjectionCenter.h"
#include "ResidualAngleHzLeft.h"
#include "ResidualAngleHzRight.h"
#include "ResidualAngleV.h"

namespace ba
{

	DataInserter::DataInserter(ceres::Problem & problem, ceres::LossFunction * lossfncPtr, BundleAdjustmentData & baData, ParameterAddressBook& addressBook):
		problem(problem), lossfncPtr(lossfncPtr), baData(baData), addressBook(addressBook)
	{

	}

	void DataInserter::insertObservationsOfImagePoints()
	{
		for (auto &p : baData.ImagePoints.Data) //dealing with image point observwations
		{
			try {
				const auto parametrization{ baData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.RotParametrization };
				const auto cameraName{ baData.ImageOrientationData.DataImages.at(p.ImageName).CameraName };
				double* opc_ptr = baData.ObjectPoints.Data.at(p.Name).Coords; //pointer to object point coordinates
				short type{ baData.ObjectPoints.Data.at(p.Name).Type };
				double* ipc_ptr = baData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.Coords; //pointer to image projection center
				double* iro_ptr = baData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.Angles; //pointer to angles
				double* iio_ptr =
					baData.ImageOrientationData.DataCameras.at(cameraName).InternalOrientation; //pointer to internal orientation parameters i.e. [ck, x0, y0]

				double* ird_ptr =
					baData.ImageOrientationData.DataCameras.at(cameraName).RadialDistortion; //pointer to radial distortion parameters i.e. [k1, k2]

				double * irdk3_ptr = ird_ptr + 2; // pointer to the k3 coefficient

				double* itd_ptr =
					baData.ImageOrientationData.DataCameras.at(cameraName).TangentialDistortion; //pointer to tangential distortion parameters i.e. [p1, p2]

				//adding resudial blocks dependig on the ba math model

				if (baData.Settings.MathModel == BAMathModel::RIGID)
				{
					if (type == 3)
					{
						problem.AddResidualBlock(new ResidualProjectionObjPtFixed(p.X, p.Y, opc_ptr, 0.5*(p.MX + p.MY), parametrization),
							lossfncPtr,
							ipc_ptr,	//projection center coordinates
							iro_ptr,	//rotation
							iio_ptr,	//internal orientation
							ird_ptr,	//radial distortion k1 and k2
							irdk3_ptr,	//radial distortion k3
							itd_ptr);	//tangential distortion

						addressBook.insert({ NamesOfParameterGroups::ORIENTATION_COORDINATES, p.ImageName }, { ipc_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::ORIENTATION_ANGLES, p.ImageName }, { iro_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraName }, { iio_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraName }, { ird_ptr, 2 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, cameraName }, { irdk3_ptr, 1 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraName }, { itd_ptr, 2 });
					}

					if (type == 4 || type == 0)
					{
						problem.AddResidualBlock(new ResidualProjectionObjPt(p.X, p.Y, 0.5*(p.MX + p.MY), parametrization),
							lossfncPtr,
							ipc_ptr,	//projection center coordinates
							iro_ptr,	//rotation
							opc_ptr,	//object point coordinates
							iio_ptr,	//internal orientation
							ird_ptr,	//radial distortion k1 and k2
							irdk3_ptr,	//radial distortion k3
							itd_ptr);	//tangential distortion

						addressBook.insert({ NamesOfParameterGroups::ORIENTATION_COORDINATES, p.ImageName }, { ipc_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::ORIENTATION_ANGLES, p.ImageName }, { iro_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, p.Name }, { opc_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraName }, { iio_ptr, 3 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraName }, { ird_ptr, 2 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, cameraName }, { irdk3_ptr, 1 });
						addressBook.insert({ NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraName }, { itd_ptr, 2 });
					}
				}

				if (baData.Settings.MathModel == BAMathModel::SOFT || baData.Settings.MathModel == BAMathModel::TIGHT)
				{

					problem.AddResidualBlock(new ResidualProjectionObjPt(p.X, p.Y, 0.5*(p.MX + p.MY), parametrization),
						lossfncPtr,
						ipc_ptr,	//projection center coordinates
						iro_ptr,	//rotation
						opc_ptr,	//object point coordinates
						iio_ptr,	//internal orientation
						ird_ptr,	//radial distortion k1 and k2
						irdk3_ptr,	//radial distortion k3
						itd_ptr);	//tangential distortion

					addressBook.insert({ NamesOfParameterGroups::ORIENTATION_COORDINATES, p.ImageName }, { ipc_ptr, 3 });
					addressBook.insert({ NamesOfParameterGroups::ORIENTATION_ANGLES, p.ImageName }, { iro_ptr, 3 });
					addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, p.Name }, { opc_ptr, 3 });
					addressBook.insert({ NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraName }, { iio_ptr, 3 });
					addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraName }, { ird_ptr, 2 });
					addressBook.insert({ NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, cameraName }, { irdk3_ptr, 1 });
					addressBook.insert({ NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraName }, { itd_ptr, 2 });
				}
			}
			catch (const std::out_of_range& oor)
			{
				std::cout << "\nERROR WHILE COLLECTING DATA FOR IMAGE POINTS!!!:\n";
				std::cerr << "Out of Range error: " << oor.what() << std::endl;
				std::cout << "problem with '" << p.Name << "' point in the image: '" << p.ImageName << "'" << std::endl;
				std::cout << "possible reasons:\n";
				std::cout << "no object point with the '" << p.Name << "' id" << std::endl;
				std::cout << "no image with the '" << p.ImageName << "' id" << std::endl;
				std::cout << "no camera defined for the image '" << p.ImageName << "'" << std::endl;
				return;
			}
		}

	}

	void DataInserter::insertObservationsOfObjectPoints()
	{
		if (baData.Settings.MathModel != BAMathModel::SOFT)
			return;

		for (auto& p : baData.ObjectPointsMeasurements.Data)
		{
			double* opm_ptr = p.second.Coords;
			double* opc_ptr = baData.ObjectPoints.Data.at(p.second.Name).Coords;
			//std::cout << p.second.ErrorsApriori[0] << " " << p.second.ErrorsApriori[1] << " " << p.second.ErrorsApriori[2] << std::endl;
			problem.AddResidualBlock(new ResidualObjectPoint(opm_ptr,
				p.second.ErrorsApriori[0], p.second.ErrorsApriori[1], p.second.ErrorsApriori[2]),
				lossfncPtr,
				opc_ptr);	//object point coordinates

			addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, p.second.Name }, { opc_ptr, 3 });
		}
		
	}

	void DataInserter::insertObservationsOfExternalOrientation()
	{
		// coordinates
		for (auto & imageData : baData.ImageOrientationData.DataImages)
		{
			if (!imageData.second.observedPosition) continue;
			problem.AddResidualBlock(new ResidualProjectionCenter(imageData.second.EOObserved.Coords,
				imageData.second.EOObserved.CoordsErrorsApriori[0],
				imageData.second.EOObserved.CoordsErrorsApriori[1],
				imageData.second.EOObserved.CoordsErrorsApriori[2]),
				lossfncPtr,
				imageData.second.EOApproximated.Coords);

			addressBook.insert({ NamesOfParameterGroups::ORIENTATION_COORDINATES, imageData.first }, { imageData.second.EOApproximated.Coords, 3 });
		}

		// angles
		for (auto & imageData : baData.ImageOrientationData.DataImages)
		{
			if (!imageData.second.observedOrientation) continue;
			problem.AddResidualBlock(new ResidualProjectionCenter(imageData.second.EOObserved.Angles,
				imageData.second.EOObserved.AnglesErrorsApriori[0],
				imageData.second.EOObserved.AnglesErrorsApriori[1],
				imageData.second.EOObserved.AnglesErrorsApriori[2]),
				lossfncPtr,
				imageData.second.EOApproximated.Angles);

			addressBook.insert({ NamesOfParameterGroups::ORIENTATION_ANGLES, imageData.first }, { imageData.second.EOApproximated.Angles, 3 });
		}

	}


	void DataInserter::insertGeodeticObservations()
	{
		if (baData.Settings.MathModel != BAMathModel::TIGHT)
			return;

		//adding left angle measurements
		for (auto& p : baData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 0)
			{
				ceres::CostFunction * cost_function =
					new ceres::NumericDiffCostFunction<ResidualAngleHzLeft, ceres::CENTRAL, 1, 3>(
						new ResidualAngleHzLeft(baData.ObjectPoints.Data.at(p.Ids.at(1)),
							baData.ObjectPoints.Data.at(p.Ids.at(2)),
							p.Hz,
							1.0 / baData.Settings.GeodeticHzMesAcc
						));

				problem.AddResidualBlock(cost_function, lossfncPtr, baData.ObjectPoints.Data.at(p.Ids.at(0)).Coords);

				addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, baData.ObjectPoints.Data.at(p.Ids.at(0)).Name }, { baData.ObjectPoints.Data.at(p.Ids.at(0)).Coords, 3 });
			}
		}

		//adding right angle measurements
		for (auto& p : baData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 2)
			{
				ceres::CostFunction * cost_function =
					new ceres::NumericDiffCostFunction<ResidualAngleHzRight, ceres::CENTRAL, 1, 3>(
						new ResidualAngleHzRight(baData.ObjectPoints.Data.at(p.Ids.at(1)),
							baData.ObjectPoints.Data.at(p.Ids.at(0)),
							p.Hz,
							1.0 / baData.Settings.GeodeticHzMesAcc
						));

				problem.AddResidualBlock(cost_function, lossfncPtr, baData.ObjectPoints.Data.at(p.Ids.at(2)).Coords);

				addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, baData.ObjectPoints.Data.at(p.Ids.at(2)).Name }, { baData.ObjectPoints.Data.at(p.Ids.at(2)).Coords, 3 });
			}
		}

		//adding angle-from-zenith measurements
		for (auto& p : baData.GeodeticAngularMeasurements.Data)
		{
			ceres::CostFunction * cost_function =
				new ceres::NumericDiffCostFunction<ResidualAngleV, ceres::CENTRAL, 1, 3>(
					new ResidualAngleV(baData.ObjectPoints.Data.at(p.Ids.at(1)),
						p.V,
						1.0 / baData.Settings.GeodeticVMesAcc));
			problem.AddResidualBlock(cost_function, lossfncPtr, baData.ObjectPoints.Data.at(p.Ids.at(p.AdjustedPointId)).Coords);

			addressBook.insert({ NamesOfParameterGroups::OBJECT_POINT_COORDINATES, baData.ObjectPoints.Data.at(p.Ids.at(p.AdjustedPointId)).Name }, { baData.ObjectPoints.Data.at(p.Ids.at(p.AdjustedPointId)).Coords, 3 });
		}
	
	}

}