#include "pch.h"
#include "BundleAdjustment.h"
#include "DataInserter.h"
#include "CovarianceComputer.h"
#include "StandardDeviationInserter.h"
#include "CameraCalibrationCorrelationAnalyzer.h"
#include "ResidualInserter.h"
#include "MyTimer.h"
#include "utils.h"




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
	//Options.linear_solver_type = ceres::LinearSolverType::CGNR;
	//Options.linear_solver_type = ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY;
	//Options.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
	//Options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
	//Options.linear_solver_type = ceres::LinearSolverType::ITERATIVE_SCHUR;
	//Options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	
	Options.preconditioner_type = ceres::PreconditionerType::CLUSTER_JACOBI;

	//Options.visibility_clustering_type = ceres::VisibilityClusteringType::SINGLE_LINKAGE;

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

	ParameterAddressBook addressBook;
	DataInserter dataInserter(Problem, lossfnc_ptr, BAData, addressBook);

	std::cout << "adding residual blocks..." << std::endl;

	dataInserter.insertObservationsOfImagePoints();
	dataInserter.insertObservationsOfObjectPoints();
	dataInserter.insertObservationsOfExternalOrientation();
	dataInserter.insertGeodeticObservations();
	
	//TODO: debug - remove when not needed
	std::cout << "camere fix masks in ba solve:" << std::endl;

	//Fixing paramter blocks related to camera:
	for (auto &c_settings : BAData.Settings.CamFixMasks)
	{
		//TODO: debug - remove when not needed
		std::cout << c_settings.first << " " << (int)c_settings.second << std::endl;
		
		if (c_settings.second & ba_fix_masks::mask_fix_io)	// fix ck, x0, y0
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).InternalOrientation);
	
		if (c_settings.second & ba_fix_masks::mask_fix_k) //fix k1 and k2 radial distortion coefficients
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).RadialDistortion);

		if (c_settings.second & ba_fix_masks::mask_fix_k3) //fix k3 radial distortion coefficients
			Problem.SetParameterBlockConstant(BAData.ImageOrientationData.DataCameras.at(c_settings.first).RadialDistortion+2);

		if (c_settings.second & ba_fix_masks::mask_fix_p) //fix p1 and p2 tangential distortion
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
	auto totalCost{ 0.0 };
	vector<double> residuals;
	ceres::CRSMatrix jacobian;
	Problem.Evaluate(eval_options, &totalCost, &residuals, nullptr, &jacobian);
	
	if (BAData.Settings.ComputeRedundancy == 1)
	{
		std::cout << "computing redundancy" << std::endl;
		const auto jacobianSparseMatrix{utils::ceresMatrix2SparseMatrix(jacobian)};
		utils::storeSparseMatrixToFile(std::filesystem::path("jacobian.txt"), jacobianSparseMatrix);
		
		//const auto jacobianEigen{ utils::ceresMatrix2EigenSparseMatrix(jacobian) };
		//std::ofstream strJacobian;
		//strJacobian.open("jacobian.txt");
		//strJacobian << jacobianEigen;
		//strJacobian.close();

	}

	Sigma02 = 2 * totalCost / (Summary.num_residuals - Summary.num_parameters);

	if (Sigma02 > 100)
	{
		std::cout << "Warning!!! Extremely high sigma0. Sigma0 = " << std::sqrt(Sigma02) << std::endl;
	}

	std::cout << "computing covariance..." << std::endl;
	CovarianceComputer covarianceComputer(prepareCovarianceOptions(), Problem, addressBook, Sigma02);
	MyTimer timer;
	timer.start();
	covarianceComputer.computeCovarianceForAllDiagonalBlocks();
	const auto durationOfCovarianceComputationMs = timer.getMs();
	timer.clear();
	std::cout << "covariance computation took: " << durationOfCovarianceComputationMs << " [ms]" << std::endl;
	StandardDeviationInserter::insertStandardDeviationsForBAData(covarianceComputer.getCovariancesForDiagonalBlocks(), BAData);


	if (BAData.Settings.ComputeCovarianceBetweenParameterGroups == 1)
	{
		try
		{
			std::cout << "\ncomputing covariance between requested parameter groups..." << std::endl;
			timer.start();
			std::ifstream fileWithParameterPairs{ std::filesystem::path{BAData.Settings.InputFiles.FilenameParmeterGroupPairs} };

			if (!fileWithParameterPairs.is_open())
			{
				std::cerr << "File with parameter pairs does not exist." << std::endl;
				throw exception("File with parameter pairs does not exist.\n");
			}

			std::vector<IdentifierOfParameterGroupPair> idsOfPairsOfParameters;
			std::set<IdentifierOfParameterGroupPair> uniqueIdsOfPairsOfParameters;
			std::vector<IdentifierOfParameterGroupPair> requestedIds;

			while (fileWithParameterPairs)
			{
				std::string gruoupName1{ "" };
				std::string id1{ "" };
				std::string gruoupName2{ "" };
				std::string id2{ "" };
				fileWithParameterPairs >> gruoupName1 >> id1 >> gruoupName2 >> id2;
				if (!fileWithParameterPairs)
					break;

				if (ParameterGroupDictionary::inverseDictionary.count(gruoupName1) != 1)
				{
					std::cerr << "ERROR! Parameter group name '" << gruoupName1 << "' is invalid!" << std::endl;
					throw exception("Wrong parameter group name!\n");
				}

				if (ParameterGroupDictionary::inverseDictionary.count(gruoupName2) != 1)
				{
					std::cerr << "ERROR! Parameter group name '" << gruoupName2 << "' is invalid!" << std::endl;
					throw exception("Wrong parameter group name!\n");
				}

				const auto identifier1{ IdentifierOfParameterGroup{ParameterGroupDictionary::inverseDictionary.at(gruoupName1), id1} };
				const auto identifier2{ IdentifierOfParameterGroup{ParameterGroupDictionary::inverseDictionary.at(gruoupName2), id2} };
				uniqueIdsOfPairsOfParameters.emplace(identifier1, identifier1);
				uniqueIdsOfPairsOfParameters.emplace(identifier1, identifier2);
				uniqueIdsOfPairsOfParameters.emplace(identifier2, identifier2);
				requestedIds.emplace_back(identifier1, identifier2);
			}

			if (requestedIds.size() == static_cast<size_t>(0))
			{
				throw logic_error("You intended to compute some covariance martrices, but cannot get your request. Something may be wrong with the input file\n");
			}

			fileWithParameterPairs.close();
			for (const auto& idPair : uniqueIdsOfPairsOfParameters)
			{
				idsOfPairsOfParameters.push_back(idPair);
			}

			const auto covarianceMatrixData{ covarianceComputer.getCovarianceMatrixForParameters(idsOfPairsOfParameters) };
		

			std::ofstream covarianceFile("covariances.txt");
			if (!covarianceFile.is_open())
			{
				std::cerr << "ERROR! Cannot open covariance file for writing." << std::endl;
				throw exception("Cannot open covariance file for writing.");
			}

			covarianceFile << "id1;id2;#rows;#columns;values\n";

			for (const auto& identifiers : requestedIds)
			{

				if (identifiers.idOfFirstGroup != identifiers.idOfSecondGroup)
				{
					const auto idUpperDiagonalPart{ IdentifierOfParameterGroupPair(identifiers.idOfFirstGroup, identifiers.idOfFirstGroup) };
					const auto idLowerDiagonalPart{ IdentifierOfParameterGroupPair(identifiers.idOfSecondGroup, identifiers.idOfSecondGroup) };
					const auto idUpperOffdiagonalPart{ IdentifierOfParameterGroupPair(identifiers.idOfFirstGroup, identifiers.idOfSecondGroup) };
					const auto ceresMatrixUpperDiagional{ covarianceMatrixData.at(idUpperDiagonalPart).matrix };
					const auto ceresMatrixLowerDiagional{ covarianceMatrixData.at(idLowerDiagonalPart).matrix };
					const auto ceresMatrixUpperOffdiagonal{ covarianceMatrixData.at(idUpperOffdiagonalPart).matrix };
					const Eigen::MatrixXd eigenMatrixUpperDiagional{ utils::ceresMatrix2EigenDenseMatrisx(ceresMatrixUpperDiagional) };
					const Eigen::MatrixXd eigenMatrixLowerDiagional{ utils::ceresMatrix2EigenDenseMatrisx(ceresMatrixLowerDiagional) };
					const Eigen::MatrixXd eigenMatrixUpperOffdiagonal{ utils::ceresMatrix2EigenDenseMatrisx(ceresMatrixUpperOffdiagonal) };
					const Eigen::MatrixXd covarianceMatrix{ utils::combineCovarianceMatrices(eigenMatrixUpperDiagional, eigenMatrixLowerDiagional, eigenMatrixUpperOffdiagonal) };
					auto rows{ covarianceMatrix.rows() };
					auto cols{ covarianceMatrix.cols() };
					covarianceFile << identifiers.idOfFirstGroup.getLabel() << ";";
					covarianceFile << identifiers.idOfSecondGroup.getLabel() << ";";
					covarianceFile << rows << ";" << cols << ";";
					for (auto row{ 0 }; row < rows; row++)
					{
						for (auto col{ 0 }; col < cols; col++)
						{
							covarianceFile << std::scientific << std::setprecision(8) << covarianceMatrix(row,col);
							if (row != rows-1 || col != cols-1)
							{
								covarianceFile << ";";
							}
						}
					}
					covarianceFile << "\n";
				}
				else
				{
					const auto martrixIdentifier{ IdentifierOfParameterGroupPair(identifiers.idOfFirstGroup, identifiers.idOfFirstGroup) };		
					const auto ceresCovarianceMatrix{ covarianceMatrixData.at(martrixIdentifier).matrix };
					const auto covarianceMatrix{ utils::ceresMatrix2EigenDenseMatrisx(ceresCovarianceMatrix)};
					auto rows{ covarianceMatrix.rows() };
					auto cols{ covarianceMatrix.cols() };
					covarianceFile << identifiers.idOfFirstGroup.getLabel() << ";";
					covarianceFile << identifiers.idOfFirstGroup.getLabel() << ";";
					covarianceFile << rows << ";" << cols << ";";
					for (auto row{ 0 }; row < rows; row++)
					{
						for (auto col{ 0 }; col < cols; col++)
						{
							covarianceFile << std::scientific << std::setprecision(8) << covarianceMatrix(row, col);
							if (row != rows - 1 || col != cols - 1)
							{
								covarianceFile << ";";
							}
						}
					}
					covarianceFile << "\n";

				}
			}
			covarianceFile.close();
		}
		catch (const std::exception& e)
		{
			std::cerr << "Exception trhow:\n";
			std::cerr << e.what() << std::endl;
		}


		const auto durationOfCovarianceComputationForGroupPairs = timer.getMs();
		std::cout << "covariance computation took: " << durationOfCovarianceComputationForGroupPairs << " [ms]" << std::endl;


	}


	if (BAData.Settings.ComputeCorrelations == 1)
	{
		CameraCalibrationCorrelationAnalyzer correlationAnalyzer(covarianceComputer, BAData.ImageOrientationData, addressBook);
		correlationAnalyzer.runAnalysis();
		cameraParametersCorrelation = correlationAnalyzer.getResults();
	}

	ResidualInsertionData residualInsertionData;
	ResidualInserter residualInserter(BAData, residualInsertionData);
	
	residualInserter.insertResiduals(residuals);

	//std::ofstream strResiduals;
	//strResiduals.open("residuals.txt");
	//
	//for (const auto& obsCat : categoryOfObservationDictionary)
	//{
	//	const auto& resData{ residualInsertionData.getData(obsCat.first) };
	//	for (const auto& data : resData)
	//	{
	//		strResiduals << categoryOfObservationDictionary.at(data.first.category) << " " << componentOfObservationDictionary.at(data.first.component) << " "<<data.second.rowInJacobian << "\n";
	//	}
	//
	//}
	//
	//strResiduals.close();


	BAData.scale_back();

	//estimation of the reverse distortion
	for (auto &c : BAData.ImageOrientationData.DataCameras)
	{
		ReverseDistortions.emplace(c.first, c.second);
	}

	std::cout << "Your bundle adjustment completed! See the report." << std::endl;
}


BundleAdjustment::~BundleAdjustment()
{
}

ceres::Covariance::Options ba::BundleAdjustment::prepareCovarianceOptions() const
{
	ceres::Covariance::Options covariance_options;
	covariance_options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;
	covariance_options.algorithm_type = ceres::CovarianceAlgorithmType::SPARSE_QR;
	covariance_options.null_space_rank = 10e-14;
	covariance_options.num_threads = 1;
	return covariance_options;
}
