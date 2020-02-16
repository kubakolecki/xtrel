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
		P2 = 7, 
		TX = 8, 
		TZ = 9 };
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
	private:
		bool _ThermalFlag{ false }; //add theral coeeficients to Tx and Tz to X and Z object point coordinates - only test
		double _Thermal[2] = { 1.0, 1.0 }; //thermal coeffs
		double _ThermalStdDev[2] = { 0.0,0.0 }; // standard deviation of themral coeffs
	}; //class BundleAdjustment


} //namespace ba

