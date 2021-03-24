#pragma once
#include "BundleAdjustmentData.h"
#include "ReverseDistortionEstimator.h"

namespace ba
{
	enum class BAAddParam { 
		CK = 0,
		X0 = 1,
		Y0 = 2,
		K1 = 3,
		K2 = 4,
		K3 = 5,
		P1 = 6,
		P2 = 7};
	//TX and TY are the termal parameters added just for experiment (used only if ba::BundleAdjustment::_ThermalFlag == true)
	class BundleAdjustment
	{
	public:
		BundleAdjustment(BundleAdjustmentData &BAData);
		~BundleAdjustment();
		std::string FullReport;
		double Sigma02{ 0.0 }; //variance
		std::map<string, ReverseDistortionEstimator> ReverseDistortions;
		std::map<std::string,std::map<std::pair<BAAddParam, BAAddParam>, double> > CorelationsCamparams;
	}; //class BundleAdjustment


} //namespace ba

