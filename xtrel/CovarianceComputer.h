#pragma once
#include "ParameterAddressBook.h"
#include "CovarianceMatrix.h"
#include <ceres/ceres.h>
#include <ceres/problem.h>

#include <fstream>

namespace ba
{

	class CovarianceComputer
	{
		using CovarianceForParametersDiagonal = std::map< IdentifierOfParameterGroup, CovarianceMatrix>;
	public:
		CovarianceComputer(const ceres::Covariance::Options& covarianceOptions, ceres::Problem & problem, const ParameterAddressBook& addressBook, double varianceOfUnitWeight);

		void computeCovarianceForAllDiagonalBlocks();
		std::map<IdentifierOfParameterGroupPair, CovarianceMatrix> getCovarianceMatrixForParameters(const std::vector<IdentifierOfParameterGroupPair>& parametersOfYourRequest) const;
		const std::map<NamesOfParameterGroups, CovarianceForParametersDiagonal>& getCovariancesForDiagonalBlocks() const;
	private:
		ceres::Problem & problem;
		const ceres::Covariance::Options & ceresCovarianceOptions;
		const ParameterAddressBook& addressBook;
		ceres::Covariance ceresCovarinaceComputer;
		std::map<NamesOfParameterGroups, CovarianceForParametersDiagonal> covarianceMatricesForDiagonalBlocks;
		std::vector<std::pair<const double*, const double*> > covarianceBlocks;
		void computeCovariance();
		bool isCovariancComputed{ false };
		double varianceOfUnitWeight{ 1.0 };
		void insertComputedCovarianceMatrix(const std::pair<const ba::IdentifierOfParameterGroup, ba::DataOfParameterGroup>& dataOfParameterBlock);
		void multiplyCovarianceMatrixByVarianceOfUnitWeight(CovarianceMatrix& cov) const;

		std::ofstream debugStream; //TODO (remove when not needed)

	};

}