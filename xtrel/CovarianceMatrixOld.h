#pragma once

namespace ba
{
	class CovarianceMatrix
	{
	public:
		CovarianceMatrix(int numOfParams) : numberOfParameters(numOfParams), numberOfMatrixElements(numOfParams*numOfParams)
		{
			matrix = new double[numberOfMatrixElements];
			for (int i{ 0 }; i < numberOfMatrixElements; i++) matrix[i] = 0.0;
		}

		CovarianceMatrix(const CovarianceMatrix& rhs) : numberOfMatrixElements(rhs.numberOfMatrixElements), numberOfParameters(rhs.numberOfParameters)
		{
			matrix = new double[numberOfMatrixElements];
			for (int i{ 0 }; i < numberOfMatrixElements; i++) matrix[i] = rhs.matrix[i];
		}

		CovarianceMatrix& operator=(const CovarianceMatrix& rhs)
		{
			numberOfMatrixElements = rhs.numberOfMatrixElements;
			numberOfParameters = rhs.numberOfParameters;
			matrix = new double[numberOfMatrixElements];
			for (int i{ 0 }; i < numberOfMatrixElements; i++) matrix[i] = rhs.matrix[i];
		}

		CovarianceMatrix(const CovarianceMatrix&& rhs) = delete;
		CovarianceMatrix& operator=(const CovarianceMatrix&& rhs) = delete;

		~CovarianceMatrix()
		{
			delete[] matrix;
		}

		int numberOfMatrixElements{ 0 };
		int numberOfParameters{ 0 };
		double* matrix{ nullptr };
	};
}