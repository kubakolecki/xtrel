#pragma once

#include "CovarianceComputer.h"
#include "ImageDataContainer.h"
#include "CovarianceMatrix.h"
#include "ParameterAddressBook.h"


namespace ba
{
	struct CameraCorrelationData
	{
		CameraCorrelationData() {};
		CameraCorrelationData(double corrX, double corrY, double corrZ, double corrAng1, double corrAng2, double corrAng3):
			correlationsWithCoordinates{corrX, corrY, corrZ}, correlationsWithAngles { corrAng1, corrAng2, corrAng3 }
		{}
		
		std::array<double, 3> correlationsWithCoordinates{ 0.0, 0.0, 0.0 };
		std::array<double, 3> correlationsWithAngles{ 0.0, 0.0, 0.0 };
	};

	using CorreltationMatrix = CovarianceMatrix;
	using CameraId = std::string;
	using ImageId = std::string;
	using CorrelationsWithExternalOrientation = std::vector<std::pair<ImageId, CameraCorrelationData>>;

	class CalibrationCorrelationAnalyisisResults
	{
	public:
		std::map<CameraId, Eigen::Matrix<double, 8, 8>> correlationMatricesForCameraParameters;
		std::unordered_map<CameraId, CorrelationsWithExternalOrientation> correlationsForPrincipalPointX; 
		std::unordered_map<CameraId, CorrelationsWithExternalOrientation> correlationsForPrincipalPointY;
		std::unordered_map<CameraId, CorrelationsWithExternalOrientation> correlationsForPrincipalDistance;
	};


	class CameraCalibrationCorrelationAnalyzer
	{
	public:
		CameraCalibrationCorrelationAnalyzer(const CovarianceComputer& covarianceComputer, const ImageDataContainer& imageData, const ParameterAddressBook& addressBook);
		void runAnalysis();
		const CalibrationCorrelationAnalyisisResults& getResults() const;

		//gets covariance matrix for specified camera
		Eigen::Matrix<double, 8, 8> computeCorrelationMatrixForCameraParameters(const std::string& cameraName) const;

		//gets covariance matrix for all cameras
		std::map<std::string, Eigen::Matrix<double, 8, 8> > computeCorrelationMatricesForCameraParameters() const;

	private:
		const CovarianceComputer& covarianceComputer;
		const ImageDataContainer& imageData;
		const ParameterAddressBook& addressBook;

		CalibrationCorrelationAnalyisisResults analysisResults;
		std::map<std::string, std::map<IdentifierOfParameterGroupPair, CorreltationMatrix> > correlationsOfCameraParameters;
		std::map<std::string, std::map<IdentifierOfParameterGroupPair, CorreltationMatrix> > correlationsBetweenIOAndEO;

		void computeCorralationsBetweenCameraParameters();
		void computeCorrelationsBetweenIOAndEO();

		CorreltationMatrix covariance2correlation(const CovarianceMatrix& covariance, const std::vector<double>& stdDevsRows, const std::vector<double>& stdDevsCols);
		std::map<NamesOfParameterGroups, std::vector<double>> getStandardDeviationsForCamera(const Camera& camera) const;
		std::map<std::string, std::vector<double>> getStandardDeviationsForOrientation() const;
		std::map<std::string, std::vector<double>> getStandardDeviationsForPosition() const;

	};


}

