#include "pch.h"
#include "StandardDeviationInserter.h"

namespace ba
{

	void StandardDeviationInserter::insertStandardDeviationsForBAData(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData)
	{
		insertStandardDeviationsForExternalOrientation(covarianceData, baData);
		insertStandardDeviationsForObjectPoints(covarianceData, baData);
		insertStandardDeviationsForCameraParameters(covarianceData, baData);
	}

	std::vector<double> StandardDeviationInserter::getStandardDeviationsFromCovarianceMatrix(const CovarianceMatrix & cov)
	{
		if (cov.matrix.rows() != cov.matrix.cols())
		{
			std::cerr << "You asked for standard deviations from covariance matrix for pramters: " << cov.idOfParameterPair.idOfFirstGroup.getLabel() << " and " << cov.idOfParameterPair.idOfSecondGroup.getLabel() << "\n";
			std::cerr << "However it happened that this matrix is not square. It has " << cov.matrix.rows() << " rows and " << cov.matrix.cols() << std::endl;
			throw std::exception("You are trying to get standard deviations from covariance matrix that is not square!\n");
		}

		if (cov.idOfParameterPair.idOfFirstGroup != cov.idOfParameterPair.idOfSecondGroup)
		{
			std::cerr << "You asked for standard deviations from covariance matrix for pramters: " << cov.idOfParameterPair.idOfFirstGroup.getLabel() << " and " << cov.idOfParameterPair.idOfSecondGroup.getLabel() << "\n";
			std::cerr << "As You see though, these parameters groups are not the same so the diagonal elements of the covariance matrix do not represent variances." << std::endl;
			throw std::exception("Illegal operation on covariance matrix - standard deviations cannot be extracted!\n");
		}
		
		std::vector<double> standardDeviations(cov.getNumberOfElements(), 0.0);
		for (int idOfDiagonalElement{ 0 }; idOfDiagonalElement < cov.matrix.rows(); ++idOfDiagonalElement)
		{
			standardDeviations.at(idOfDiagonalElement) = std::sqrt(cov.matrix(idOfDiagonalElement,idOfDiagonalElement));
		}
		return standardDeviations;
	}

	void StandardDeviationInserter::insertStandardDeviationsForExternalOrientation(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData)
	{
		for (auto& imageDataRecord : baData.ImageOrientationData.DataImages)
		{
			auto& imageData{ imageDataRecord.second };
			const auto imageCoordinatesIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_COORDINATES, imageData.Name) };
			const auto imageAnglesIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::ORIENTATION_ANGLES, imageData.Name) };
			const auto& covarianceMatrixCoordinates{ covarianceData.at(NamesOfParameterGroups::ORIENTATION_COORDINATES).at(imageCoordinatesIdentifier) };
			const auto& covarianceMatrixAngles{ covarianceData.at(NamesOfParameterGroups::ORIENTATION_ANGLES).at(imageAnglesIdentifier) };
			const auto standardDeviationsCoordinates{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrixCoordinates) };
			const auto standardDeviationsAngles{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrixAngles) };
			for (auto i : { 0, 1, 2 })
			{
				imageData.EOAdjusted.CoordsErrorsAposteriori[i] = standardDeviationsCoordinates[i];
				imageData.EOAdjusted.AnglesErrorsAposteriori[i] = standardDeviationsAngles[i];
				imageData.EOApproximated.CoordsErrorsAposteriori[i] = standardDeviationsCoordinates[i];
				imageData.EOApproximated.AnglesErrorsAposteriori[i] = standardDeviationsAngles[i];
			}

		}
	}

	void StandardDeviationInserter::insertStandardDeviationsForObjectPoints(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData)
	{
		if (baData.Settings.MathModel == BAMathModel::RIGID)
			return;

		for (auto& pointData : baData.ObjectPoints.Data)
		{
			const auto objectPointIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::OBJECT_POINT_COORDINATES, pointData.second.Name) };
			const auto& covarianceMatrixObjectPoint{ covarianceData.at(NamesOfParameterGroups::OBJECT_POINT_COORDINATES).at(objectPointIdentifier) };
			const auto standardDeviationsObjectPoint{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrixObjectPoint) };

			for (auto i : { 0, 1, 2 })
			{
				pointData.second.ErrorsAposteriori[i] = standardDeviationsObjectPoint[i];
			}
		}
	}

	void StandardDeviationInserter::insertStandardDeviationsForCameraParameters(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData)
	{
		
		for (auto& cameraData : baData.ImageOrientationData.DataCameras)
		{
			const auto& cameraName{ cameraData.first };
			if (!(baData.Settings.CamFixMasks.at(cameraName) & ba_fix_masks::mask_fix_io))
			{
				const auto cameraParametersIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, cameraName) };
				const auto& covarianceMatrix{covarianceData.at(NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION).at(cameraParametersIdentifier) };
				const auto standardDeviations{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrix) };
				for (auto i : { 0, 1, 2 })
				{
					cameraData.second.InternalOrientationStdDev[i] = standardDeviations[i];
				}
			}
			if (!(baData.Settings.CamFixMasks.at(cameraName) & ba_fix_masks::mask_fix_k))
			{
				const auto cameraParametersIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraName) };
				const auto& covarianceMatrix{ covarianceData.at(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12).at(cameraParametersIdentifier) };
				const auto standardDeviations{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrix) };
				cameraData.second.RadialDistortionStdDev[0] = standardDeviations[0];
				cameraData.second.RadialDistortionStdDev[1] = standardDeviations[1];
			}
			if (!(baData.Settings.CamFixMasks.at(cameraName) & ba_fix_masks::mask_fix_k3))
			{
				const auto cameraParametersIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, cameraName) };
				const auto& covarianceMatrix{ covarianceData.at(NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3).at(cameraParametersIdentifier) };
				const auto standardDeviation{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrix) };
				cameraData.second.RadialDistortionStdDev[2] = standardDeviation[0];
			}
			if (!(baData.Settings.CamFixMasks.at(cameraName) & ba_fix_masks::mask_fix_p))
			{
				const auto cameraParametersIdentifier{ IdentifierOfParameterGroup(NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraName) };
				const auto& covarianceMatrix{ covarianceData.at(NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION).at(cameraParametersIdentifier) };
				const auto standardDeviation{ getStandardDeviationsFromCovarianceMatrix(covarianceMatrix) };
				cameraData.second.TangentialDistortionStdDev[0] = standardDeviation[0];
				cameraData.second.TangentialDistortionStdDev[1] = standardDeviation[1];

			}
		}

	}

}
