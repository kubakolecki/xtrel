#pragma once
#include "BundleAdjustmentData.h"
#include "ReverseDistortionEstimator.h"
#include "CameraCalibrationCorrelationAnalyzer.h"

#include <ceres/ceres.h>
#include <ceres/problem.h>


namespace ba
{

	class BundleAdjustment
	{
	public:
		BundleAdjustment(BundleAdjustmentData &BAData);
		~BundleAdjustment();
		std::string FullReport;
		double Sigma02{ 0.0 }; //variance
		std::map<string, ReverseDistortionEstimator> ReverseDistortions;
		CalibrationCorrelationAnalyisisResults cameraParametersCorrelation;
	
	private:
		ceres::Covariance::Options prepareCovarianceOptions() const;
		
	};


} 

