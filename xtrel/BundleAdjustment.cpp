#include "pch.h"
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include "BundleAdjustment.h"
#include "ResidualProjectionObjPt.h"
#include "ResidualProjectionObjPtFixed.h"
#include "ResidualObjectPoint.h"
#include "ResidualProjectionCenter.h"
#include "ResidualAngleHzLeft.h"
#include "ResidualAngleHzRight.h"
#include "ResidualAngleV.h"


using namespace ba;

BundleAdjustment::BundleAdjustment(BundleAdjustmentData &BAData)
{
	//Controlling camera settings in the settings file
	for (auto &c_settings : BAData.Settings.CamFixMasks)
	{
		try 
		{
			string camera_name = BAData.ImageOrientationData.DataCameras.at(c_settings.first).Name;
		}
		catch (const std::out_of_range& oor)
		{
			std::cout << "\nERROR WHILE EVALUATING CAMERA FIX MASKS!!!:\n";
			std::cerr << "Out of Range error: " << oor.what() << std::endl;
			std::cout << "'" << c_settings.first << "' camera was not deffined" << std::endl;
			return;
		}
	}


	//Scaling BAData for better determinability of parameters:
	//division by principal distance:
	BAData.scale();
	
	std::cout << "Number of images: " << BAData.NumOfImages << std::endl;
	std::cout << "Number of cameras: " << BAData.NumOfCameras << std::endl;
	std::cout << "Number of controll points: " << BAData.NumOfControllPoints << std::endl;
	std::cout << "Number of tie points: " << BAData.NumOfTiePoints << std::endl;
	std::cout << "Number of check points: " << BAData.NumOfCheckPoints << std::endl;
	std::cout << "Number of image points: " << BAData.NumOfImagePoints << std::endl;

	//Defining the ceres problem:

	std::cout << "constructing optimization problem..." << std::endl;
	ceres::Problem Problem;
	ceres::Solver::Options Options;
	Options.max_num_iterations = 50;
	Options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	Options.num_threads = 4;
	//Options.minimizer_type = ceres::LINE_SEARCH;
	Options.minimizer_progress_to_stdout = true;

	ceres::LossFunction* lossfnc_ptr = nullptr;
	if (BAData.Settings.LossFunction == BALossFunction::CAUCHY)
	{
		lossfnc_ptr = new ceres::CauchyLoss(BAData.Settings.LossFunctionParameter);
	}
	if (BAData.Settings.LossFunction == BALossFunction::HUBER)
	{
		lossfnc_ptr = new ceres::HuberLoss(BAData.Settings.LossFunctionParameter);
	}

	std::cout << "adding residual blocks..." << std::endl;

	for (auto &p : BAData.ImagePoints.Data) //dealing with image point observwations
	{
		try {
			std::string parametrization = BAData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.RotParametrization;
			double* opc_ptr = BAData.ObjectPoints.Data.at(p.Name).Coords; //pointer to object point coordinates
			short type = BAData.ObjectPoints.Data.at(p.Name).Type;
			double* ipc_ptr = BAData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.Coords; //pointer to image projection center
			double* iro_ptr = BAData.ImageOrientationData.DataImages.at(p.ImageName).EOApproximated.Angles; //pointer to angles
			double* iio_ptr =
				BAData.ImageOrientationData.DataCameras.at(
					BAData.ImageOrientationData.DataImages.at(p.ImageName).CameraName
				).InternalOrientation; //pointer to internal orientation parameters i.e. [ck, x0, y0]

			double* ird_ptr =
				BAData.ImageOrientationData.DataCameras.at(
					BAData.ImageOrientationData.DataImages.at(p.ImageName).CameraName
				).RadialDistortion; //pointer to internal radial distortion parameters i.e. [k1, k2, ,]

			double * irdk3_ptr = ird_ptr + 2; // pointer to the k3 coefficient

			double* itd_ptr =
				BAData.ImageOrientationData.DataCameras.at(
					BAData.ImageOrientationData.DataImages.at(p.ImageName).CameraName
				).TangentialDistortion; //pointer to internal tangential distortion parameters i.e. [p1, p2]

			//adding resudial blocks dependig on the ba math model

			if (BAData.Settings.MathModel == BAMathModel::RIGID)
			{
				if (type == 3)
				{
					Problem.AddResidualBlock(new ResidualProjectionObjPtFixed(p.X, p.Y, opc_ptr, 0.5*(p.MX + p.MY), parametrization),
						lossfnc_ptr,
						ipc_ptr,	//projection center coordinates
						iro_ptr,	//rotation
						iio_ptr,	//internal orientation
						ird_ptr,	//radial distortion k1 and k2
						irdk3_ptr,	//radial distortion k3
						itd_ptr);	//tangential distortion
				}

				if (type == 4 || type == 0)
				{
					Problem.AddResidualBlock(new ResidualProjectionObjPt(p.X, p.Y, 0.5*(p.MX + p.MY), parametrization),
						lossfnc_ptr,
						ipc_ptr,	//projection center coordinates
						iro_ptr,	//rotation
						opc_ptr,	//object point coordinates
						iio_ptr,	//internal orientation
						ird_ptr,	//radial distortion k1 and k2
						irdk3_ptr,	//radial distortion k3
						itd_ptr);	//tangential distortion
				}
			}

			if (BAData.Settings.MathModel == BAMathModel::SOFT || BAData.Settings.MathModel == BAMathModel::TIGHT)
			{
									
				Problem.AddResidualBlock(new ResidualProjectionObjPt(p.X, p.Y, 0.5*(p.MX + p.MY), parametrization),
					lossfnc_ptr,
					ipc_ptr,	//projection center coordinates
					iro_ptr,	//rotation
					opc_ptr,	//object point coordinates
					iio_ptr,	//internal orientation
					ird_ptr,	//radial distortion k1 and k2
					irdk3_ptr,	//radial distortion k3
					itd_ptr);	//tangential distortion
			}
		}
		catch (const std::out_of_range& oor)
		{
			std::cout << "\nERROR WHILE COLLECTING DATA FOR IMAGE POINTS!!!:\n";
			std::cerr << "Out of Range error: " << oor.what() << std::endl;
			std::cout << "problem with '" << p.Name << "' point in the image: '" << p.ImageName <<"'" << std::endl;
			std::cout << "possible reasons:\n";
			std::cout << "no object point with the '" << p.Name << "' id" << std::endl;
			std::cout << "no image with the '" << p.ImageName << "' id" << std::endl;
			std::cout << "no camera defined for the image '" << p.ImageName <<"'" << std::endl;
			return;
		}
	}

	//object points:

	if (BAData.Settings.MathModel == BAMathModel::SOFT)
	{
		for (auto& p : BAData.ObjectPointsMeasurements.Data)
		{			
			double* opm_ptr = p.second.Coords;
			double* opc_ptr = BAData.ObjectPoints.Data.at(p.second.Name).Coords;
			//std::cout << p.second.ErrorsApriori[0] << " " << p.second.ErrorsApriori[1] << " " << p.second.ErrorsApriori[2] << std::endl;
			Problem.AddResidualBlock(new ResidualObjectPoint(opm_ptr,
				p.second.ErrorsApriori[0], p.second.ErrorsApriori[1], p.second.ErrorsApriori[2]),
				lossfnc_ptr,
				opc_ptr);	//object point coordinates
		}
	}

	//geodetic observations:

	if (BAData.Settings.MathModel == BAMathModel::TIGHT)
	{
		//adding left angle measurements
		for (auto& p : BAData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 0)
			{
				ceres::CostFunction * cost_function = 
					new ceres::NumericDiffCostFunction<ResidualAngleHzLeft, ceres::CENTRAL, 1, 3>(
						new ResidualAngleHzLeft(BAData.ObjectPoints.Data.at(p.Ids.at(1)),
												BAData.ObjectPoints.Data.at(p.Ids.at(2)),
												p.Hz,
												1.0/BAData.Settings.GeodeticHzMesAcc
					));

				Problem.AddResidualBlock(cost_function, lossfnc_ptr, BAData.ObjectPoints.Data.at(p.Ids.at(0)).Coords);
			}
		}

		//adding right angle measurements
		for (auto& p : BAData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 2)
			{
				ceres::CostFunction * cost_function =
					new ceres::NumericDiffCostFunction<ResidualAngleHzRight, ceres::CENTRAL, 1, 3>(
						new ResidualAngleHzRight(BAData.ObjectPoints.Data.at(p.Ids.at(1)),
							BAData.ObjectPoints.Data.at(p.Ids.at(0)),
							p.Hz,
							1.0 / BAData.Settings.GeodeticHzMesAcc
						));

				Problem.AddResidualBlock(cost_function, lossfnc_ptr, BAData.ObjectPoints.Data.at(p.Ids.at(2)).Coords);
			}
		}

		//adding angle-from-zenith measurements
		for (auto& p : BAData.GeodeticAngularMeasurements.Data)
		{
			ceres::CostFunction * cost_function =
				new ceres::NumericDiffCostFunction<ResidualAngleV, ceres::CENTRAL, 1, 3>(
					new ResidualAngleV(BAData.ObjectPoints.Data.at(p.Ids.at(1)),
						p.V,
						1.0 / BAData.Settings.GeodeticVMesAcc));
			Problem.AddResidualBlock(cost_function, lossfnc_ptr, BAData.ObjectPoints.Data.at(p.Ids.at(p.AdjustedPointId)).Coords);
		}
	}

	// coordinates
	for (auto & imageData : BAData.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedPosition) continue;
		Problem.AddResidualBlock(new ResidualProjectionCenter(imageData.second.EOObserved.Coords,
			imageData.second.EOObserved.CoordsErrorsApriori[0],
			imageData.second.EOObserved.CoordsErrorsApriori[1],
			imageData.second.EOObserved.CoordsErrorsApriori[2]),
			lossfnc_ptr,
			imageData.second.EOApproximated.Coords);
	}

	// angles
	for (auto & imageData : BAData.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedOrientation) continue;
		Problem.AddResidualBlock(new ResidualProjectionCenter(imageData.second.EOObserved.Angles,
			imageData.second.EOObserved.AnglesErrorsApriori[0],
			imageData.second.EOObserved.AnglesErrorsApriori[1],
			imageData.second.EOObserved.AnglesErrorsApriori[2]),
			lossfnc_ptr,
			imageData.second.EOApproximated.Angles);
	}

	//Fixing paramter blocks related to camera:
	for (auto &c_settings : BAData.Settings.CamFixMasks)
	{
		if (c_settings.second & ba_fix_masks::mask_fix_io)	// fix ck, x0, y0
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).InternalOrientation);
	
		if (c_settings.second & ba_fix_masks::mask_fix_k) //fix k1 and k2 radial distortion coefficients
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).RadialDistortion);

		if (c_settings.second & ba_fix_masks::mask_fix_k3) //fix k1 and k2 radial distortion coefficients
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).RadialDistortion+2);

		if (c_settings.second & ba_fix_masks::mask_fix_p) //fix k1 and k2 radial distortion coefficients
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).TangentialDistortion);
	}

	//Solving ceres problem:
	std::cout << "solving the problem..." << std::endl;
	ceres::Solver::Summary Summary;
	try {
		ceres::Solve(Options, &Problem, &Summary);
	}
	catch (...)
	{
		std::cout << "exception while solving" << std::endl;
	}
	std::cout << "\nsolver report:\n";
	FullReport = Summary.FullReport();
	std::cout << FullReport  << endl;
	

	//Evaluating the problem:
	ceres::Problem::EvaluateOptions eval_options;
	eval_options.apply_loss_function = false;
	double total_cost = 0.0;
	vector<double> residuals;
	ceres::CRSMatrix jacobian;
	Problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, &jacobian);



	cout << "covariance evaluation..." <<std::endl;
	//EXTRACTING STANDARD DEVIATIONS:
	//1. Defining ceres covariance object with specified options 
	ceres::Covariance::Options covariance_options;
	covariance_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
	covariance_options.null_space_rank = 10e-14;
	covariance_options.num_threads = 1;
	ceres::Covariance covariance(covariance_options);
	vector<pair<const double*, const double*> > covariance_blocks;

	cout << "preparing covariance extraction..." << std::endl;
	//2. Preparing coviarince blocks for external orientation
	for (auto &i : BAData.ImageOrientationData.DataImages)
	{
		covariance_blocks.emplace_back(i.second.EOApproximated.Coords, i.second.EOApproximated.Coords);
		covariance_blocks.emplace_back(i.second.EOApproximated.Angles, i.second.EOApproximated.Angles);
	}


	//3. Preparing coviariance blocks for object points
	if (BAData.Settings.MathModel == BAMathModel::SOFT || BAData.Settings.MathModel == BAMathModel::TIGHT)
	for (auto &p : BAData.ObjectPoints.Data)
	{
		if (p.second.Type == 0 || p.second.Type == 3 || p.second.Type == 4)
		{
			covariance_blocks.emplace_back(p.second.Coords, p.second.Coords);
		}
	}


	//4. Preparing covariance blocks for camera parameters
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		covariance_blocks.emplace_back(c.second.InternalOrientation, c.second.InternalOrientation);
		covariance_blocks.emplace_back(c.second.RadialDistortion, c.second.RadialDistortion);
		covariance_blocks.emplace_back(c.second.RadialDistortion + 2, c.second.RadialDistortion + 2);
		covariance_blocks.emplace_back(c.second.TangentialDistortion, c.second.TangentialDistortion);
	}



	//5. Preparing off-diagonal blocks for camera and thermal parameters
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		covariance_blocks.emplace_back(c.second.InternalOrientation, c.second.RadialDistortion);
		covariance_blocks.emplace_back(c.second.InternalOrientation, c.second.RadialDistortion + 2);
		covariance_blocks.emplace_back(c.second.InternalOrientation, c.second.TangentialDistortion);

		covariance_blocks.emplace_back(c.second.RadialDistortion, c.second.RadialDistortion + 2);
		covariance_blocks.emplace_back(c.second.RadialDistortion, c.second.TangentialDistortion);

		covariance_blocks.emplace_back(c.second.RadialDistortion + 2, c.second.TangentialDistortion);
	}



	std::cout << "covariance estimation..." << std::endl;
	//6. Covariance estimation
	CHECK(covariance.Compute(covariance_blocks, &Problem));

	//7. Calculating sigma0
	Sigma02 = 2 * total_cost / (Summary.num_residuals - Summary.num_parameters);

	if (Sigma02 > 100)
	{
		std::cout << "Warning!!! Extremely high sigma0. Sigma0 = " << std::sqrt(Sigma02) << std::endl;
	}

	std::cout << "extracting covariance values..." << std::endl;
	//8. Extraction of covariance values
	for (auto &i : BAData.ImageOrientationData.DataImages)
	{
		double cov_coords[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double cov_angles[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		covariance.GetCovarianceBlock(i.second.EOApproximated.Coords, i.second.EOApproximated.Coords, cov_coords);
		covariance.GetCovarianceBlock(i.second.EOApproximated.Angles, i.second.EOApproximated.Angles, cov_angles);
		i.second.EOApproximated.CoordsErrorsAposteriori[0] = std::sqrt(cov_coords[0]*Sigma02);
		i.second.EOApproximated.CoordsErrorsAposteriori[1] = std::sqrt(cov_coords[4]*Sigma02);
		i.second.EOApproximated.CoordsErrorsAposteriori[2] = std::sqrt(cov_coords[8]*Sigma02);
		i.second.EOApproximated.AnglesErrorsAposteriori[0] = std::sqrt(cov_angles[0]*Sigma02);
		i.second.EOApproximated.AnglesErrorsAposteriori[1] = std::sqrt(cov_angles[4]*Sigma02);
		i.second.EOApproximated.AnglesErrorsAposteriori[2] = std::sqrt(cov_angles[8]*Sigma02);
	}

	//standard deviations of camera parameters:
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		double cov_internal[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		double cov_radial_k12[4] = { 0.0, 0.0, 0.0, 0.0};
		double cov_radial_k3{ 0.0 };
		double cov_tangential[4] = { 0.0, 0.0, 0.0, 0.0 };
		
		if (covariance.GetCovarianceBlock(c.second.InternalOrientation, c.second.InternalOrientation, cov_internal)) {
			c.second.InternalOrientationStdDev[0] = std::sqrt(cov_internal[0] * Sigma02);
			c.second.InternalOrientationStdDev[1] = std::sqrt(cov_internal[4] * Sigma02);
			c.second.InternalOrientationStdDev[2] = std::sqrt(cov_internal[8] * Sigma02);
		}

		if (covariance.GetCovarianceBlock(c.second.RadialDistortion, c.second.RadialDistortion, cov_radial_k12)) {
			c.second.RadialDistortionStdDev[0] = std::sqrt(cov_radial_k12[0] * Sigma02);
			c.second.RadialDistortionStdDev[1] = std::sqrt(cov_radial_k12[3] * Sigma02);
		}

		if (covariance.GetCovarianceBlock(c.second.RadialDistortion + 2, c.second.RadialDistortion + 2, &cov_radial_k3)) {
			c.second.RadialDistortionStdDev[2] = std::sqrt(cov_radial_k3 * Sigma02);
		}

		if (covariance.GetCovarianceBlock(c.second.TangentialDistortion, c.second.TangentialDistortion, cov_tangential)) {
			c.second.TangentialDistortionStdDev[0] = std::sqrt(cov_tangential[0] * Sigma02);
			c.second.TangentialDistortionStdDev[1] = std::sqrt(cov_tangential[3] * Sigma02);
		}
	}

	std::cout << "calculating Pearsons corelations..." << std::endl;
	//corelation coeffs - diagonal and off-diagonal blocks
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		double cb9[9];
		double cb6[6];
		double cb4[4];
		double cb3[3];
		double cb2[2];
		std::pair<BAAddParam, BAAddParam> p;
		double cor{ 0.0 };
		std::map< std::pair<BAAddParam, BAAddParam>, double> corelcontainer;
		if (covariance.GetCovarianceBlock(c.second.InternalOrientation, c.second.InternalOrientation, cb9))
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::X0); cor = Sigma02 * cb9[1] / (c.second.InternalOrientationStdDev[0] * c.second.InternalOrientationStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::CK, BAAddParam::Y0); cor = Sigma02 * cb9[2] / (c.second.InternalOrientationStdDev[0] * c.second.InternalOrientationStdDev[2]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::Y0); cor = Sigma02 * cb9[5] / (c.second.InternalOrientationStdDev[1] * c.second.InternalOrientationStdDev[2]);
			corelcontainer.emplace(p, cor);

			//std::cout << "ck-ck corelation: " << Sigma02 * cb9[0] / (c.second.InternalOrientationStdDev[0] * c.second.InternalOrientationStdDev[0]) << std::endl;
			//std::cout << "x0-x0 corelation: " << Sigma02 * cb9[4] / (c.second.InternalOrientationStdDev[1] * c.second.InternalOrientationStdDev[1]) << std::endl;
			//std::cout << "y0-y0 corelation: " << Sigma02 * cb9[8] / (c.second.InternalOrientationStdDev[2] * c.second.InternalOrientationStdDev[2]) << std::endl;
		}
		else
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::K1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::CK, BAAddParam::Y0); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::Y0); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.InternalOrientation, c.second.RadialDistortion, cb6))
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::K1); cor = Sigma02 * cb6[0] / (c.second.InternalOrientationStdDev[0] * c.second.RadialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::CK, BAAddParam::K2); cor = Sigma02 * cb6[1] / (c.second.InternalOrientationStdDev[0] * c.second.RadialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K1); cor = Sigma02 * cb6[2] / (c.second.InternalOrientationStdDev[1] * c.second.RadialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K2); cor = Sigma02 * cb6[3] / (c.second.InternalOrientationStdDev[1] * c.second.RadialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K1); cor = Sigma02 * cb6[4] / (c.second.InternalOrientationStdDev[2] * c.second.RadialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K2); cor = Sigma02 * cb6[5] / (c.second.InternalOrientationStdDev[2] * c.second.RadialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::K1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::CK, BAAddParam::K2); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K2); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K2); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.InternalOrientation, c.second.RadialDistortion + 2, cb3))
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::K3); cor = Sigma02 * cb3[0] / (c.second.InternalOrientationStdDev[0] * c.second.RadialDistortionStdDev[2]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K3); cor = Sigma02 * cb3[1] / (c.second.InternalOrientationStdDev[1] * c.second.RadialDistortionStdDev[2]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K3); cor = Sigma02 * cb3[2] / (c.second.InternalOrientationStdDev[2] * c.second.RadialDistortionStdDev[2]);
			corelcontainer.emplace(p, cor);

		}
		else
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::K3); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::K3); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::K3); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.InternalOrientation, c.second.TangentialDistortion, cb6))
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::P1); cor = Sigma02 * cb6[0] / (c.second.InternalOrientationStdDev[0] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::CK, BAAddParam::P2); cor = Sigma02 * cb6[1] / (c.second.InternalOrientationStdDev[0] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::P1); cor = Sigma02 * cb6[2] / (c.second.InternalOrientationStdDev[1] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::X0, BAAddParam::P2); cor = Sigma02 * cb6[3] / (c.second.InternalOrientationStdDev[1] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::P1); cor = Sigma02 * cb6[4] / (c.second.InternalOrientationStdDev[2] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::P2); cor = Sigma02 * cb6[5] / (c.second.InternalOrientationStdDev[2] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::CK, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::CK, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::X0, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::Y0, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.RadialDistortion, c.second.RadialDistortion, cb4))
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::K2); cor = Sigma02 * cb4[1] / (c.second.RadialDistortionStdDev[0] * c.second.RadialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::K2); corelcontainer.emplace(p, 0.0);
		}


		if (covariance.GetCovarianceBlock(c.second.RadialDistortion, c.second.RadialDistortion + 2, cb2))
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::K3); cor = Sigma02 * cb2[0] / (c.second.RadialDistortionStdDev[0] * c.second.RadialDistortionStdDev[2]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::K2, BAAddParam::K3); cor = Sigma02 * cb2[1] / (c.second.RadialDistortionStdDev[1] * c.second.RadialDistortionStdDev[2]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::K3); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::K2, BAAddParam::K3); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.RadialDistortion, c.second.TangentialDistortion, cb4))
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::P1); cor = Sigma02 * cb4[0] / (c.second.RadialDistortionStdDev[0] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::K1, BAAddParam::P2); cor = Sigma02 * cb4[1] / (c.second.RadialDistortionStdDev[0] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::K2, BAAddParam::P1); cor = Sigma02 * cb4[2] / (c.second.RadialDistortionStdDev[1] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::K2, BAAddParam::P2); cor = Sigma02 * cb4[3] / (c.second.RadialDistortionStdDev[1] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);

		}
		else
		{
			p = std::make_pair(BAAddParam::K1, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::K1, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::K2, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::K2, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.RadialDistortion + 2, c.second.TangentialDistortion, cb2))
		{
			p = std::make_pair(BAAddParam::K3, BAAddParam::P1); cor = Sigma02 * cb2[0] / (c.second.RadialDistortionStdDev[2] * c.second.TangentialDistortionStdDev[0]);
			corelcontainer.emplace(p, cor);
			p = std::make_pair(BAAddParam::K3, BAAddParam::P2); cor = Sigma02 * cb2[1] / (c.second.RadialDistortionStdDev[2] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::K3, BAAddParam::P1); corelcontainer.emplace(p, 0.0);
			p = std::make_pair(BAAddParam::K3, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
		}

		if (covariance.GetCovarianceBlock(c.second.TangentialDistortion, c.second.TangentialDistortion, cb4))
		{
			p = std::make_pair(BAAddParam::P1, BAAddParam::P2); cor = Sigma02 * cb4[1] / (c.second.TangentialDistortionStdDev[0] * c.second.TangentialDistortionStdDev[1]);
			corelcontainer.emplace(p, cor);
		}
		else
		{
			p = std::make_pair(BAAddParam::P1, BAAddParam::P2); corelcontainer.emplace(p, 0.0);
		}


		CorelationsCamparams.emplace(c.first, corelcontainer);
	}

	if (BAData.Settings.MathModel == BAMathModel::SOFT || BAData.Settings.MathModel == BAMathModel::TIGHT)
	{
		for (auto &p : BAData.ObjectPoints.Data)
		{
			if (p.second.Type == 0 || p.second.Type == 3) //Covarince is to be extracted only for tie points and controll points, not for geodetic reference points
			{
				double cov[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
				if (covariance.GetCovarianceBlock(p.second.Coords, p.second.Coords, cov))
				{
					p.second.ErrorsAposteriori[0] = std::sqrt(cov[0] * Sigma02);
					p.second.ErrorsAposteriori[1] = std::sqrt(cov[4] * Sigma02);
					p.second.ErrorsAposteriori[2] = std::sqrt(cov[8] * Sigma02);
				}
			}
		}
	}
	

	//EXTRACTING RESIDUALS
	std::cout << "writing residuals..." << std::endl;
	std::cout << "number of residuals: " << residuals.size() <<std::endl;

	
	size_t i{ 0 };
	for (auto& p : BAData.ImagePoints.Data)
	{
		p.VX = residuals.at(i);//*p.MX;
		i++;
		p.VY = residuals.at(i);//*p.MY;
		i++;
	}

	if (BAData.Settings.MathModel == BAMathModel::SOFT)
	{
		for (auto&p : BAData.ObjectPointsMeasurements.Data)
		{
			p.second.V[0] = residuals.at(i);
			i++;
			p.second.V[1] = residuals.at(i);
			i++;
			p.second.V[2] = residuals.at(i);
			i++;
		}
	}

	if (BAData.Settings.MathModel == BAMathModel::TIGHT)
	{
		//extractin residuals for left angle measurements
		for (auto&p : BAData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 0)
			{
				p.Residuals[0] = residuals.at(i)*BAData.Settings.GeodeticHzMesAcc; //vHz;
				i++;
			}
		}

		//extractin residuals for right angle measurements
		for (auto&p : BAData.GeodeticAngularMeasurements.Data)
		{
			if (p.AdjustedPointId == 2)
			{
				p.Residuals[0] = residuals.at(i)*BAData.Settings.GeodeticHzMesAcc; //vHz;
				i++;
			}
		}

		//extractin residuals for zenith angle measurements
		for (auto&p : BAData.GeodeticAngularMeasurements.Data)
		{
			p.Residuals[1] = residuals.at(i)*BAData.Settings.GeodeticVMesAcc; //V;
			i++;
		}
	}
	
	//extracting residuals for projection centers
	for (auto & imageData : BAData.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedPosition) continue;
		imageData.second.CoordsResiduals[0] = residuals.at(i++)*imageData.second.EOObserved.CoordsErrorsApriori[0];
		imageData.second.CoordsResiduals[1] = residuals.at(i++)*imageData.second.EOObserved.CoordsErrorsApriori[1];
		imageData.second.CoordsResiduals[2] = residuals.at(i++)*imageData.second.EOObserved.CoordsErrorsApriori[2];
	}

	//extracting residuals for angles
	for (auto & imageData : BAData.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedOrientation) continue;
		imageData.second.AnglesResiduals[0] = residuals.at(i++)*imageData.second.EOObserved.AnglesErrorsApriori[0];
		imageData.second.AnglesResiduals[1] = residuals.at(i++)*imageData.second.EOObserved.AnglesErrorsApriori[1];
		imageData.second.AnglesResiduals[2] = residuals.at(i++)*imageData.second.EOObserved.AnglesErrorsApriori[2];
	}

	//we have to scale residuals because we have weighted the equations! :
	for (auto& p : BAData.ImagePoints.Data)
	{
		p.VX *= p.MX;
		p.VY *= p.MY;
	}


	BAData.scale_back();


	if (BAData.Settings.MathModel == BAMathModel::SOFT)
	{
		for (auto&p : BAData.ObjectPointsMeasurements.Data)
		{
			for (int i : {0, 1, 2}) p.second.V[i] *= p.second.ErrorsApriori[i];
		}
	}

	
	//estimation of the reverse distortion
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		ReverseDistortions.emplace(c.first, c.second);
	}

	std::cout << "Your bundle adjustment completed! See report." << std::endl;
}


BundleAdjustment::~BundleAdjustment()
{
}
