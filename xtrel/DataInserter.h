#pragma once

#include <ceres/ceres.h>
#include <ceres/problem.h>

#include "BundleAdjustmentData.h"
#include "ParameterAddressBook.h"

namespace ba
{

	class DataInserter
	{
	public:
		DataInserter(ceres::Problem& problem, ceres::LossFunction* lossfncPtr, BundleAdjustmentData& baData, ParameterAddressBook& addressBook);
		void insertObservationsOfImagePoints();
		void insertObservationsOfObjectPoints();
		void insertObservationsOfExternalOrientation();
		void insertGeodeticObservations();

	private:
		ceres::Problem& problem;
		ceres::LossFunction* lossfncPtr{ nullptr };
		BundleAdjustmentData& baData;
		ParameterAddressBook& addressBook;
	};

}


