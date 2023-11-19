#pragma once

#include <map>
#include "ParameterAddressBook.h"
#include "CovarianceMatrix.h"
#include "BundleAdjustmentData.h"

namespace ba
{
	class StandardDeviationInserter
	{
	public:
		using covarianceForParameters = std::map< IdentifierOfParameterGroup, CovarianceMatrix>;
		static void insertStandardDeviationsForBAData(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData& baData);
	
	private:
		static std::vector<double> getStandardDeviationsFromCovarianceMatrix(const CovarianceMatrix& cov);
		static void insertStandardDeviationsForExternalOrientation(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData);
		static void insertStandardDeviationsForObjectPoints(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData);
		static void insertStandardDeviationsForCameraParameters(const std::map<NamesOfParameterGroups, covarianceForParameters>& covarianceData, BundleAdjustmentData & baData);
	};

}

