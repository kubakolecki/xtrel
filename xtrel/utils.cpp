#include "pch.h"
#include "utils.h"

#include <limits>

Eigen::SparseMatrix<double> ba::utils::ceresMatrix2EigenSparseMatrix(const ceres::CRSMatrix& ceresMatrix)
{
	Eigen::SparseMatrix<double> eigenSparseMatrix;

	eigenSparseMatrix.resize(ceresMatrix.num_rows, ceresMatrix.num_cols);
	eigenSparseMatrix.reserve(ceresMatrix.values.size());

	for (auto row{ 0 }; row < ceresMatrix.num_rows; ++row)
	{
		for (auto colId{ ceresMatrix.rows[row] }; colId < ceresMatrix.rows[row + 1]; ++colId)
		{
			eigenSparseMatrix.insert(row, ceresMatrix.cols[colId]) = ceresMatrix.values[colId];
		}
	}

	eigenSparseMatrix.data().squeeze();
	return eigenSparseMatrix;
}

Eigen::MatrixXd ba::utils::ceresMatrix2EigenDenseMatrisx(const ceres::Matrix & ceresMatrix)
{
	Eigen::MatrixXd eigenDenseMatrix;
	eigenDenseMatrix.resize(ceresMatrix.rows(), ceresMatrix.cols() );



	for (auto row{ 0 }; row < ceresMatrix.rows(); ++row)
	{
		for (auto col{ 0 }; col < ceresMatrix.cols(); ++col)
		{
			eigenDenseMatrix(row, col) = ceresMatrix(row,col);
		}
	}
	return eigenDenseMatrix;
}

ba::utils::SparseMatrixStorage ba::utils::ceresMatrix2SparseMatrix(const ceres::CRSMatrix & ceresMatrix)
{
	SparseMatrixStorage sparseMatrix;
	sparseMatrix.numberOfRows = ceresMatrix.num_rows;
	sparseMatrix.numberOfCols = ceresMatrix.num_cols;

	sparseMatrix.entries.reserve(16384);

	constexpr auto epsilon{ std::numeric_limits<double>::epsilon() };

	for (auto row{ 0 }; row < ceresMatrix.num_rows; ++row)
	{
		for (auto colId{ ceresMatrix.rows[row] }; colId < ceresMatrix.rows[row + 1]; ++colId)
		{
			if (std::abs(ceresMatrix.values[colId]) <= epsilon)
				continue;		
			sparseMatrix.entries.emplace_back(std::make_pair(row, ceresMatrix.cols[colId]),  ceresMatrix.values[colId]);
		}
	}

	sparseMatrix.entries.shrink_to_fit();
	return sparseMatrix;
}

void ba::utils::storeSparseMatrixToFile(const std::filesystem::path path, const SparseMatrixStorage& matrix)
{
	std::ofstream stream(path);

	if (!stream.is_open())
	{
		std::cerr << "Cannot open " << path.string() << " to store the matrix!" << std::endl;
		throw std::exception("Cannot open file for writing.\n");
	}

	stream << "number_of_rows: " << matrix.numberOfRows << "\n";
	stream << "number_of_cols: " << matrix.numberOfCols << "\n";

	for (const auto& sparseMatrixEntry : matrix.entries)
	{
		const auto&[rowColId, value] {sparseMatrixEntry};
		const auto&[rowId, colId] {rowColId};
		stream << rowId << " " << colId << " " << value << "\n";
	}
	stream.close();

}

Eigen::MatrixXd ba::utils::combineCovarianceMatrices(const Eigen::MatrixXd & firstMatrix, const Eigen::MatrixXd & secondMatrix, const Eigen::MatrixXd & upperOffdiagonalMatrix)
{
	if (firstMatrix.rows() != upperOffdiagonalMatrix.rows())
	{
		throw std::logic_error("Matrices cannot be combined. Dimensions are inconsistent!");
	}

	if (secondMatrix.cols() != upperOffdiagonalMatrix.cols())
	{
		throw std::logic_error("Matrices cannot be combined. Dimensions are inconsistent!");
	}

	if (firstMatrix.rows() != firstMatrix.cols())
	{
		throw std::logic_error("Matrices cannot be combined. Diagonal components are not square!");
	}

	if (secondMatrix.rows() != secondMatrix.cols())
	{
		throw std::logic_error("Matrices cannot be combined. Diagonal components are not square!");
	}
	
	Eigen::MatrixXd lowerOffDiagonalMatrix = upperOffdiagonalMatrix.transpose();

	const int rowsFirstMatrix = firstMatrix.rows();
	const int colsFirstMatrix = firstMatrix.cols();
	const int rowsSecondMatrix = secondMatrix.rows();
	const int colsSecondMatrix = secondMatrix.cols();

	Eigen::MatrixXd combinedMatrix(rowsFirstMatrix + rowsSecondMatrix, colsFirstMatrix + colsSecondMatrix);

	combinedMatrix.block(0,0,rowsFirstMatrix, colsFirstMatrix) = firstMatrix;
	combinedMatrix.block(rowsFirstMatrix, colsFirstMatrix, rowsSecondMatrix, colsSecondMatrix)  = secondMatrix;
	combinedMatrix.block(0, colsFirstMatrix, rowsFirstMatrix, colsSecondMatrix) = upperOffdiagonalMatrix;
	combinedMatrix.block(rowsFirstMatrix, 0, rowsSecondMatrix, colsFirstMatrix) = lowerOffDiagonalMatrix;

	return combinedMatrix;
}
