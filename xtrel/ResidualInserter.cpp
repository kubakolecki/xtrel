#include "pch.h"
#include "ResidualInserter.h"

namespace ba
{

	ResidualInserter::ResidualInserter(BundleAdjustmentData & baData, ResidualInsertionData& residualData): baData(baData), residualData(residualData)
	{

	}

	void ResidualInserter::insertResiduals(const std::vector<double>& residuals)
	{
		size_t residualCounter{ 0 };
		for (auto& imagePoint : baData.ImagePoints.Data)
		{
			imagePoint.VX = residuals.at(residualCounter++)*imagePoint.MX;
			imagePoint.VY = residuals.at(residualCounter++)*imagePoint.MY;
			residualData.insertData(ObservationId(categoryOfObservation::IMAGE_POINT_COORDINATE, componentOfObservation::X, { imagePoint.ImageName, imagePoint.Name } ), residualCounter - 2, imagePoint.VX);
			residualData.insertData(ObservationId(categoryOfObservation::IMAGE_POINT_COORDINATE, componentOfObservation::Y, { imagePoint.ImageName, imagePoint.Name } ), residualCounter - 1, imagePoint.VY);
		}

		if (baData.Settings.MathModel == BAMathModel::SOFT)
		{
			for (auto& objectPoint : baData.ObjectPointsMeasurements.Data)
			{
				objectPoint.second.V[0] = residuals.at(residualCounter++)*objectPoint.second.ErrorsApriori[0];
				objectPoint.second.V[1] = residuals.at(residualCounter++)*objectPoint.second.ErrorsApriori[1];
				objectPoint.second.V[2] = residuals.at(residualCounter++)*objectPoint.second.ErrorsApriori[2];
				residualData.insertData(ObservationId(categoryOfObservation::OBJECT_POINT_COORDINATE, componentOfObservation::X, { objectPoint.second.Name }), residualCounter - 3, objectPoint.second.V[0]);
				residualData.insertData(ObservationId(categoryOfObservation::OBJECT_POINT_COORDINATE, componentOfObservation::Y, { objectPoint.second.Name }), residualCounter - 2, objectPoint.second.V[1]);
				residualData.insertData(ObservationId(categoryOfObservation::OBJECT_POINT_COORDINATE, componentOfObservation::Z, { objectPoint.second.Name }), residualCounter - 1, objectPoint.second.V[2]);
			}
		}

		if (baData.Settings.MathModel == BAMathModel::TIGHT)
		{
			//extractin residuals for left angle measurements
			for (auto &angle : baData.GeodeticAngularMeasurements.Data)
			{
				if (angle.AdjustedPointId == 0)
				{
					angle.Residuals[0] = residuals.at(residualCounter++)*baData.Settings.GeodeticHzMesAcc; //vHz;
					residualData.insertData(ObservationId(categoryOfObservation::GEODETIC_ANGLE_MEASUREMENT, componentOfObservation::HORIZONTAL, { angle.Ids[0], angle.Ids[1], angle.Ids[2] }), residualCounter - 1, angle.Residuals[0]);
				}
			}

			//extractin residuals for right angle measurements
			for (auto &angle : baData.GeodeticAngularMeasurements.Data)
			{
				if (angle.AdjustedPointId == 2)
				{
					angle.Residuals[0] = residuals.at(residualCounter++)*baData.Settings.GeodeticHzMesAcc; //vHz;
					residualData.insertData(ObservationId(categoryOfObservation::GEODETIC_ANGLE_MEASUREMENT, componentOfObservation::HORIZONTAL, { angle.Ids[0], angle.Ids[1], angle.Ids[2] }), residualCounter - 1, angle.Residuals[0]);

				}
			}

			//extractin residuals for zenith angle measurements
			for (auto &angle : baData.GeodeticAngularMeasurements.Data)
			{
				angle.Residuals[1] = residuals.at(residualCounter++)*baData.Settings.GeodeticVMesAcc; //V;
				residualData.insertData(ObservationId(categoryOfObservation::GEODETIC_ANGLE_MEASUREMENT, componentOfObservation::VERTICAL, { angle.Ids[0], angle.Ids[1], angle.Ids[2] }), residualCounter - 1, angle.Residuals[0]);

			}
		}

		//extracting residuals for projection centers
		for (auto & imageData : baData.ImageOrientationData.DataImages)
		{
			if (!imageData.second.observedPosition) continue;
			imageData.second.CoordsResiduals[0] = residuals.at(residualCounter++)*imageData.second.EOObserved.CoordsErrorsApriori[0];
			imageData.second.CoordsResiduals[1] = residuals.at(residualCounter++)*imageData.second.EOObserved.CoordsErrorsApriori[1];
			imageData.second.CoordsResiduals[2] = residuals.at(residualCounter++)*imageData.second.EOObserved.CoordsErrorsApriori[2];
			residualData.insertData(ObservationId(categoryOfObservation::PROJECTION_CENTER_COORDINATE, componentOfObservation::X, { imageData.second.Name }), residualCounter - 3, imageData.second.CoordsResiduals[0]);
			residualData.insertData(ObservationId(categoryOfObservation::PROJECTION_CENTER_COORDINATE, componentOfObservation::Y, { imageData.second.Name }), residualCounter - 2, imageData.second.CoordsResiduals[1]);
			residualData.insertData(ObservationId(categoryOfObservation::PROJECTION_CENTER_COORDINATE, componentOfObservation::Z, { imageData.second.Name }), residualCounter - 1, imageData.second.CoordsResiduals[2]);
		}

		//extracting residuals for angles
		for (auto & imageData : baData.ImageOrientationData.DataImages)
		{
			if (!imageData.second.observedOrientation) continue;
			imageData.second.AnglesResiduals[0] = residuals.at(residualCounter++)*imageData.second.EOObserved.AnglesErrorsApriori[0];
			imageData.second.AnglesResiduals[1] = residuals.at(residualCounter++)*imageData.second.EOObserved.AnglesErrorsApriori[1];
			imageData.second.AnglesResiduals[2] = residuals.at(residualCounter++)*imageData.second.EOObserved.AnglesErrorsApriori[2];
			residualData.insertData(ObservationId(categoryOfObservation::IMAGE_ORIENTATION_ANGLE, componentOfObservation::OMEGA, { imageData.second.Name }), residualCounter - 3, imageData.second.CoordsResiduals[0]);
			residualData.insertData(ObservationId(categoryOfObservation::IMAGE_ORIENTATION_ANGLE, componentOfObservation::PHI, { imageData.second.Name }), residualCounter - 2, imageData.second.CoordsResiduals[1]);
			residualData.insertData(ObservationId(categoryOfObservation::IMAGE_ORIENTATION_ANGLE, componentOfObservation::KAPPA, { imageData.second.Name }), residualCounter - 1, imageData.second.CoordsResiduals[2]);
		}
	}


}