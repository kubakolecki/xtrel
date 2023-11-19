#include "pch.h"
#include "CameraCalibrationCorrelationAnalyzer.h"

namespace ba
{
	const CalibrationCorrelationAnalyisisResults & CameraCalibrationCorrelationAnalyzer::getResults() const
	{
		return analysisResults;
	}



	CameraCalibrationCorrelationAnalyzer::CameraCalibrationCorrelationAnalyzer(const CovarianceComputer & covarianceComputer, const ImageDataContainer & imageData, const ParameterAddressBook& addressBook):
		covarianceComputer(covarianceComputer), imageData(imageData), addressBook(addressBook)
	{
		
	}

	void CameraCalibrationCorrelationAnalyzer::runAnalysis()
	{
		computeCorralationsBetweenCameraParameters();
		computeCorrelationsBetweenIOAndEO();
		analysisResults.correlationMatricesForCameraParameters = std::move(computeCorrelationMatricesForCameraParameters());
	}


	Eigen::Matrix<double, 8, 8> CameraCalibrationCorrelationAnalyzer::computeCorrelationMatrixForCameraParameters(const std::string & cameraName) const
	{
		Eigen::Matrix<double, 8, 8> correlationMatrix;
		const auto& corrMatrix{ correlationsOfCameraParameters.at(cameraName) };
		const auto idInterior{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraName) };
		const auto idDistK1K2{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraName) };
		const auto idDistK3{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, cameraName) };
		const auto idDistP1P2{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraName) };
		
		correlationMatrix.setZero();
		correlationMatrix.block<3, 3>(0, 0) = corrMatrix.at({ idInterior , idInterior }).matrix;
		correlationMatrix.block<3, 2>(0, 3) = corrMatrix.at({ idInterior, idDistK1K2 }).matrix;
		correlationMatrix.block<3, 1>(0, 5) = corrMatrix.at({ idInterior, idDistK3 }).matrix;
		correlationMatrix.block<3, 2>(0, 6) = corrMatrix.at({ idInterior, idDistP1P2 }).matrix;
		correlationMatrix.block<2, 3>(3, 0) = corrMatrix.at({ idInterior, idDistK1K2 }).matrix.transpose();
		correlationMatrix.block<2, 2>(3, 3) = corrMatrix.at({ idDistK1K2, idDistK1K2 }).matrix;
		correlationMatrix.block<2, 1>(3, 5) = corrMatrix.at({ idDistK1K2, idDistK3 }).matrix;
		correlationMatrix.block<2, 2>(3, 6) = corrMatrix.at({ idDistK1K2, idDistP1P2 }).matrix;
		correlationMatrix.block<1, 2>(5, 0) = corrMatrix.at({ idInterior, idDistK3 }).matrix.transpose();
		correlationMatrix.block<1, 2>(5, 3) = corrMatrix.at({ idDistK1K2, idDistK3 }).matrix.transpose();
		correlationMatrix.block<1, 1>(5, 5) = corrMatrix.at({ idDistK3, idDistK3 }).matrix;
		correlationMatrix.block<1, 2>(5, 6) = corrMatrix.at({ idDistK3, idDistP1P2 }).matrix;
		correlationMatrix.block<2, 3>(6, 0) = corrMatrix.at({ idInterior, idDistP1P2 }).matrix.transpose();
		correlationMatrix.block<2, 2>(6, 3) = corrMatrix.at({ idDistK1K2, idDistP1P2 }).matrix.transpose();
		correlationMatrix.block<2, 1>(6, 5) = corrMatrix.at({ idDistK3, idDistP1P2 }).matrix.transpose();
		correlationMatrix.block<2, 2>(6, 6) = corrMatrix.at({ idDistP1P2, idDistP1P2 }).matrix;

		return correlationMatrix;
	}

	std::map<std::string,Eigen::Matrix<double, 8, 8>> CameraCalibrationCorrelationAnalyzer::computeCorrelationMatricesForCameraParameters() const
	{
		std::map<std::string, Eigen::Matrix<double, 8, 8>> correlationMatrices;

		for (const auto& cameraData : imageData.DataCameras)
		{
			const auto correlationMatrix{ computeCorrelationMatrixForCameraParameters(cameraData.second.Name) };
			correlationMatrices.emplace(cameraData.second.Name, correlationMatrix);
		}

		return correlationMatrices;
	}

	void CameraCalibrationCorrelationAnalyzer::computeCorralationsBetweenCameraParameters()
	{
		correlationsOfCameraParameters.clear();
		
		for (const auto& cameraData : imageData.DataCameras)
		{
			const auto dataOfAllParamtersOfThisCamera{ addressBook.getAllParameterDataForGivenCamera(cameraData.second.Name) };
			const auto numOfParams{ dataOfAllParamtersOfThisCamera.size() };

			std::vector<IdentifierOfParameterGroupPair> parameterPairsToQuery;

			for (auto i{ 0 }; i < numOfParams; i++)
			{
				for (auto j{ i }; j < numOfParams; j++)
				{
					parameterPairsToQuery.emplace_back(dataOfAllParamtersOfThisCamera[i].first, dataOfAllParamtersOfThisCamera[j].first);
				}
			}

			const auto& covarianceMatrices{ covarianceComputer.getCovarianceMatrixForParameters(parameterPairsToQuery) };

			std::map<IdentifierOfParameterGroupPair, CorreltationMatrix> correlationMatrices;

			const auto standardDevs{ getStandardDeviationsForCamera(cameraData.second) };

			//converting covariances to correlations:
			for (const auto& cov : covarianceMatrices)
			{
				const auto corrlationMatrix{ covariance2correlation(
					cov.second,
					standardDevs.at(cov.second.idOfParameterPair.idOfFirstGroup.groupName),
					standardDevs.at(cov.second.idOfParameterPair.idOfSecondGroup.groupName)) };

				correlationMatrices.emplace(cov.second.idOfParameterPair, corrlationMatrix);
			}

			correlationsOfCameraParameters.emplace(cameraData.second.Name, correlationMatrices);
		}
	}

	void CameraCalibrationCorrelationAnalyzer::computeCorrelationsBetweenIOAndEO()
	{
		correlationsBetweenIOAndEO.clear();
		for (const auto& cameraData : imageData.DataCameras)
		{
			std::vector<IdentifierOfParameterGroupPair> parameterPairsToQuery;
			parameterPairsToQuery.reserve(imageData.DataCameras.size() * 2);
			const auto ioIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraData.second.Name) };
			for (const auto& imageData : imageData.DataImages)
			{
				const auto anglesIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_ANGLES, imageData.second.Name) };
				const auto coordsIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_COORDINATES, imageData.second.Name) };
				parameterPairsToQuery.emplace_back(ioIdentifier, anglesIdentifier);
				parameterPairsToQuery.emplace_back(ioIdentifier, coordsIdentifier);
			}

			const auto covarianceMatrices{ covarianceComputer.getCovarianceMatrixForParameters(parameterPairsToQuery) };

			//std::map<IdentifierOfParameterGroupPair, CorreltationMatrix> correlationMatrices;

			const auto standardDevsCamera{ getStandardDeviationsForCamera(cameraData.second) };
			const auto stdDevsOrientation{ getStandardDeviationsForOrientation() };
			const auto stdDevsPosition{ getStandardDeviationsForPosition() };

			//TODO: remove when not needed
			//std::cout << "standard devs for orientation:" << std::endl;
			//for (const auto& stddev : stdDevsOrientation)
			//{
			//	std::cout << stddev.first << " " << stddev.second[0] << " " << stddev.second[1] <<" " << stddev.second[2] << std::endl;
			//}

			std::vector<std::pair<ImageId, CameraCorrelationData>> corrlationsOfck;
			std::vector<std::pair<ImageId, CameraCorrelationData>> corrlationsOfx0;
			std::vector<std::pair<ImageId, CameraCorrelationData>> corrlationsOfy0;

			std::vector<std::tuple<std::string, CovarianceMatrix, CovarianceMatrix>> corrleationMatricesForIOvsEO;
			corrleationMatricesForIOvsEO.reserve(imageData.DataImages.size());

			for (const auto& imageData : imageData.DataImages)
			{
				const auto anglesIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_ANGLES, imageData.second.Name) };
				const auto coordsIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_COORDINATES, imageData.second.Name) };
				
				const auto ioAndAnglesIdentifier{ IdentifierOfParameterGroupPair(ioIdentifier, anglesIdentifier) };
				const auto ioAndCoordsIdentifier{ IdentifierOfParameterGroupPair(ioIdentifier, coordsIdentifier) };
				
				const auto covarianceMatrixAngles{ covarianceMatrices.at(ioAndAnglesIdentifier) };
				const auto covarianceMatrixCoords{ covarianceMatrices.at(ioAndCoordsIdentifier) };

				corrleationMatricesForIOvsEO.emplace_back(imageData.second.Name, std::move(covarianceMatrixCoords), std::move(covarianceMatrixAngles));
			}



			for (const auto& covPair : corrleationMatricesForIOvsEO)
			{
				const auto& covarianceMatrixCoords{ std::get<1>(covPair) };
				const auto& covarianceMatrixAngles{ std::get<2>(covPair) };

				const auto correlationMatrixPosition{ covariance2correlation(
					covarianceMatrixCoords,
					standardDevsCamera.at(covarianceMatrixCoords.idOfParameterPair.idOfFirstGroup.groupName),
					stdDevsPosition.at(covarianceMatrixCoords.idOfParameterPair.idOfSecondGroup.parameterLabel)) };
				
				const auto correlationMatrixOrientation{ covariance2correlation(
					covarianceMatrixAngles,
					standardDevsCamera.at(covarianceMatrixAngles.idOfParameterPair.idOfFirstGroup.groupName),
					stdDevsOrientation.at(covarianceMatrixAngles.idOfParameterPair.idOfSecondGroup.parameterLabel)) };


				CameraCorrelationData corrDatack{
					correlationMatrixPosition(0,0), correlationMatrixPosition(0,1), correlationMatrixPosition(0,2),
					correlationMatrixOrientation(0,0), correlationMatrixOrientation(0,1), correlationMatrixOrientation(0,2) };

				CameraCorrelationData corrDatax0{
					correlationMatrixPosition(1,0), correlationMatrixPosition(1,1), correlationMatrixPosition(1,2),
					correlationMatrixOrientation(1,0), correlationMatrixOrientation(1,1), correlationMatrixOrientation(1,2) };
				
				CameraCorrelationData corrDatay0{
					correlationMatrixPosition(2,0), correlationMatrixPosition(2,1), correlationMatrixPosition(2,2),
					correlationMatrixOrientation(2,0), correlationMatrixOrientation(2,1), correlationMatrixOrientation(2,2) };

				corrlationsOfck.emplace_back(std::get<0>(covPair), std::move(corrDatack));
				corrlationsOfx0.emplace_back(std::get<0>(covPair), std::move(corrDatax0));
				corrlationsOfy0.emplace_back(std::get<0>(covPair), std::move(corrDatay0));

			}

			analysisResults.correlationsForPrincipalDistance.emplace(cameraData.second.Name, std::move(corrlationsOfck));
			analysisResults.correlationsForPrincipalPointX.emplace(cameraData.second.Name, std::move(corrlationsOfx0));
			analysisResults.correlationsForPrincipalPointY.emplace(cameraData.second.Name, std::move(corrlationsOfy0));

		}


	}

	CorreltationMatrix CameraCalibrationCorrelationAnalyzer::covariance2correlation(const CovarianceMatrix & covariance, const std::vector<double>& stdDevsRows, const std::vector<double>& stdDevsCols)
	{
		if (covariance.rows() != stdDevsRows.size())
		{
			throw std::exception("Cannot convert covariance to correlation: size of input arguments does not match!\n");
		}
		
		if (covariance.cols() != stdDevsCols.size())
		{
			throw std::exception("Cannot convert covariance to correlation: size of input arguments does not match!\n");
		}
		
		auto correlationMatrix(covariance);

		for (auto row{ 0 }; row < correlationMatrix.rows(); row++)
		{
			for (auto col{ 0 }; col < correlationMatrix.cols(); col++)
			{
				correlationMatrix(row, col) = correlationMatrix(row, col) / (stdDevsRows[row] * stdDevsCols[col]);
			}

		}
		return correlationMatrix;
	}
	std::map<NamesOfParameterGroups, std::vector<double>> CameraCalibrationCorrelationAnalyzer::getStandardDeviationsForCamera(const Camera & camera) const
	{
		std::map<NamesOfParameterGroups, std::vector<double>> standardDeviations;

		std::vector<double> stdDeviationsInteriorOrientation{camera.InternalOrientationStdDev[0], camera.InternalOrientationStdDev[1], camera.InternalOrientationStdDev[2]};
		std::vector<double> stdDeviationsK1K2{ camera.RadialDistortionStdDev[0], camera.RadialDistortionStdDev[1]};
		std::vector<double> stdDeviationsK3{ camera.RadialDistortionStdDev[2] };
		std::vector<double> stdDeviationsP1P2{ camera.TangentialDistortionStdDev[0], camera.TangentialDistortionStdDev[1] };
		
		standardDeviations.emplace(NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, stdDeviationsInteriorOrientation);
		standardDeviations.emplace(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, stdDeviationsK1K2);
		standardDeviations.emplace(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, stdDeviationsK3);
		standardDeviations.emplace(NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, stdDeviationsP1P2);

		return standardDeviations;
	}
	std::map<std::string, std::vector<double>> CameraCalibrationCorrelationAnalyzer::getStandardDeviationsForOrientation() const
	{
		std::map<std::string, std::vector<double>> standardDeviations;
		
		for (const auto& img : imageData.DataImages)
		{
			std::vector<double>stdDevs{ img.second.EOAdjusted.AnglesErrorsAposteriori[0], img.second.EOAdjusted.AnglesErrorsAposteriori[1], img.second.EOAdjusted.AnglesErrorsAposteriori[2] };
			standardDeviations.emplace(img.second.Name, stdDevs);
		}
		return standardDeviations;
	}
	std::map<std::string, std::vector<double>> CameraCalibrationCorrelationAnalyzer::getStandardDeviationsForPosition() const
	{
		std::map<std::string, std::vector<double>> standardDeviations;
		
		for (const auto& img : imageData.DataImages)
		{
			std::vector<double>stdDevs{ img.second.EOAdjusted.CoordsErrorsAposteriori[0], img.second.EOAdjusted.CoordsErrorsAposteriori[1], img.second.EOAdjusted.CoordsErrorsAposteriori[2] };
			standardDeviations.emplace(img.second.Name, stdDevs);
		}
		return standardDeviations;
	}
}
