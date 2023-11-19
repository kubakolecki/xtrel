#pragma once

#include "ParameterAddressBook.h"

#include <Eigen/Dense>

namespace ba
{

	class CovarianceMatrix
	{
	public:
		CovarianceMatrix(
			int numOfParametersRepresentingRows,
			int numOfParametersRepresentingColumns,
			IdentifierOfParameterGroup idOfParametersRepresentingRows,
			IdentifierOfParameterGroup idOfParametersRepresentingColumns):
			idOfParameterPair{idOfParametersRepresentingRows,idOfParametersRepresentingColumns}
		{	
			if (numOfParametersRepresentingRows != numOfParametersRepresentingColumns)
			{
				if (idOfParametersRepresentingRows == idOfParametersRepresentingColumns)
				{
					throw std::exception("Covariance matrix that reprsents diagonal block must be square!");
				}
			}
			matrix.resize(numOfParametersRepresentingRows, numOfParametersRepresentingColumns);
		}

		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix;

		IdentifierOfParameterGroupPair idOfParameterPair;
		
		bool isDiagonal() const
		{
			return idOfParameterPair.idOfFirstGroup == idOfParameterPair.idOfSecondGroup;
		}

		int rows() const
		{
			return matrix.rows();
		}

		int cols() const
		{
			return matrix.cols();
		}

		int getNumberOfElements() const
		{
			return rows()*cols();
		}

		double* getAddress()
		{
			return matrix.data();
		}

		double& operator()(int row, int col)
		{
			return matrix(row, col);
		}

		const double& operator()(int row, int col) const
		{
			return matrix(row, col);
		}

	};
}
