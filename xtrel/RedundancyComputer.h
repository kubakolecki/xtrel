#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace ba
{

	class RedundancyComputer
	{
	public:
		RedundancyComputer(const Eigen::SparseMatrix<double>& jacobian);

	private:
		const Eigen::SparseMatrix<double>& jacobian;

	};

}