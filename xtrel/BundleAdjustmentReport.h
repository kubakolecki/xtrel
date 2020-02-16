#pragma once
#include "BundleAdjustmentData.h"
#include "BundleAdjustment.h"

namespace ba
{
	class BundleAdjustmentReport
	{
	public:
		BundleAdjustmentReport(BundleAdjustmentData& badata, BundleAdjustment& ba, const std::string& filename);
		~BundleAdjustmentReport();

	}; //class BundleAdjustmentReport

} //namespace ba