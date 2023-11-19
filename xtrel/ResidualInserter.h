#pragma once

#include "BundleAdjustmentData.h"
#include "ResidualInsertionData.h"

namespace ba
{

	class ResidualInserter
	{
	public:
		ResidualInserter(BundleAdjustmentData& baData, ResidualInsertionData& residualData);
		void insertResiduals(const std::vector<double>& residuals);

	private:
		BundleAdjustmentData& baData;
		ResidualInsertionData& residualData;

	};

}