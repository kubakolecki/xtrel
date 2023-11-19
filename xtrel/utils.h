#pragma once
#include <Eigen/Sparse>
#include <ceres/ceres.h>
#include <filesystem>

namespace ba
{
	namespace utils
	{
		using RowColIds = std::pair<size_t, size_t>;
		using SparseMatrixEntry = std::pair<RowColIds, double>;
		using SparseMatrixEntries = std::vector<SparseMatrixEntry>;
		
		struct SparseMatrixStorage
		{
			size_t numberOfRows{ 0 };
			size_t numberOfCols{ 0 };
			SparseMatrixEntries entries;
		};

		Eigen::SparseMatrix<double> ceresMatrix2EigenSparseMatrix(const ceres::CRSMatrix& ceresMatrix);
		Eigen::MatrixXd ceresMatrix2EigenDenseMatrisx(const ceres::Matrix & ceresMatrix);
		SparseMatrixStorage ceresMatrix2SparseMatrix(const ceres::CRSMatrix& ceresMatrix);
		void storeSparseMatrixToFile(const std::filesystem::path path,const SparseMatrixStorage& matrix);
		Eigen::MatrixXd combineCovarianceMatrices(const Eigen::MatrixXd& firstMatrix, const Eigen::MatrixXd& secondMatrix, const Eigen::MatrixXd& upperOffdiagonalMatrix);

	}
}