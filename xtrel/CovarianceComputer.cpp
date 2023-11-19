#include "pch.h"
#include "CovarianceComputer.h"

namespace ba
{

	CovarianceComputer::CovarianceComputer(const ceres::Covariance::Options & covarianceOptions, ceres::Problem & problem, const ParameterAddressBook & addressBook, double varianceOfUnitWeight)
		: problem(problem), addressBook(addressBook), ceresCovarinaceComputer(covarianceOptions), ceresCovarianceOptions(covarianceOptions), varianceOfUnitWeight(varianceOfUnitWeight)
	{

	}

	void CovarianceComputer::computeCovarianceForAllDiagonalBlocks()
	{	
		covarianceMatricesForDiagonalBlocks.clear();
		covarianceBlocks.clear();

		for (const auto& idOfParameterType : addressBook.getIdentifiers())
		{
			covarianceMatricesForDiagonalBlocks.insert({ idOfParameterType.groupName, CovarianceForParametersDiagonal{} });
		}

		for (const auto& paramAddresses : addressBook.getAddresses())
		{
			covarianceBlocks.emplace_back(paramAddresses.second.address, paramAddresses.second.address);
		}



		computeCovariance();

		for (const auto& paramAddresses : addressBook.getAddresses())
		{
			insertComputedCovarianceMatrix(paramAddresses);
		}
	}

	std::map<IdentifierOfParameterGroupPair, CovarianceMatrix> CovarianceComputer::getCovarianceMatrixForParameters(const std::vector<IdentifierOfParameterGroupPair>& parametersOfYourRequest) const
	{
		std::map<IdentifierOfParameterGroupPair, CovarianceMatrix> requestedCovarianceMatrices;
		ceres::Covariance ceresCovarianceComputerLocal(ceresCovarianceOptions);

		std::vector<std::pair<const double*, const double*> > blocksLocal;

		for (const auto& idOfParameterPair : parametersOfYourRequest)
		{
			const auto& dataOfParamtersRepresentingRows{ addressBook.getParameterGroupData(idOfParameterPair.idOfFirstGroup) };
			const auto& dataOfParamtersRepresentingColumns{ addressBook.getParameterGroupData(idOfParameterPair.idOfSecondGroup) };

			if (dataOfParamtersRepresentingRows.has_value() && dataOfParamtersRepresentingColumns.has_value())
			{
				blocksLocal.push_back({ dataOfParamtersRepresentingRows.value().address, dataOfParamtersRepresentingColumns.value().address });
			}
			else if (!dataOfParamtersRepresentingRows.has_value() && dataOfParamtersRepresentingColumns.has_value())
			{
				std::cerr << "You asked for computing covariance of: " << idOfParameterPair.idOfFirstGroup.getLabel() << " vs " << idOfParameterPair.idOfSecondGroup.getLabel() << "\n";
				std::cerr << "However parameter group: " << idOfParameterPair.idOfFirstGroup.getLabel() << " is not registerd in the address book." << std::endl;
				throw std::exception("Cannot request covariance matrix!\n");
			}
			else if (dataOfParamtersRepresentingRows.has_value() && !dataOfParamtersRepresentingColumns.has_value())
			{
				std::cerr << "You asked for computing covariance of: " << idOfParameterPair.idOfFirstGroup.getLabel() << " vs " << idOfParameterPair.idOfSecondGroup.getLabel() << "\n";
				std::cerr << "However parameter group: " << idOfParameterPair.idOfSecondGroup.getLabel() << " is not registerd in the address book." << std::endl;
				throw std::exception("Cannot request covariance matrix!\n");
			}
			else
			{
				std::cerr << "You asked for computing covariance of: " << idOfParameterPair.idOfFirstGroup.getLabel() << " vs " << idOfParameterPair.idOfSecondGroup.getLabel() << "\n";
				std::cerr << "However parameter both parameter groups are not registerd in the address book." << std::endl;
				throw std::exception("Cannot request covariance matrix!\n");
			}

		}

		CHECK(ceresCovarianceComputerLocal.Compute(blocksLocal, &problem));

		for (const auto& idOfParameterPair : parametersOfYourRequest)
		{
			const auto& dataOfParamtersRepresentingRows{ addressBook.getParameterGroupData(idOfParameterPair.idOfFirstGroup) };
			const auto& dataOfParamtersRepresentingColumns{ addressBook.getParameterGroupData(idOfParameterPair.idOfSecondGroup) };
			
			CovarianceMatrix cov(
				dataOfParamtersRepresentingRows.value().size,
				dataOfParamtersRepresentingColumns.value().size,
				idOfParameterPair.idOfFirstGroup,
				idOfParameterPair.idOfSecondGroup);
			
			ceresCovarianceComputerLocal.GetCovarianceBlock(dataOfParamtersRepresentingRows.value().address, dataOfParamtersRepresentingColumns.value().address, cov.matrix.data());
			
			multiplyCovarianceMatrixByVarianceOfUnitWeight(cov);
			
			requestedCovarianceMatrices.insert({idOfParameterPair, cov });
		}

		
		return requestedCovarianceMatrices;
	}

	const std::map<NamesOfParameterGroups, CovarianceComputer::CovarianceForParametersDiagonal>& CovarianceComputer::getCovariancesForDiagonalBlocks() const
	{
		return covarianceMatricesForDiagonalBlocks;
	}

	void CovarianceComputer::computeCovariance()
	{
		CHECK(ceresCovarinaceComputer.Compute(covarianceBlocks, &problem));
		isCovariancComputed = true;
	}

	void CovarianceComputer::insertComputedCovarianceMatrix(const std::pair<const ba::IdentifierOfParameterGroup, ba::DataOfParameterGroup>& dataOfParameterBlock)
	{
		if (!isCovariancComputed)
			throw std::exception("Trying to insert covariance matrices before computing them");

		CovarianceMatrix cov(dataOfParameterBlock.second.size, dataOfParameterBlock.second.size, dataOfParameterBlock.first, dataOfParameterBlock.first);
		
		ceresCovarinaceComputer.GetCovarianceBlock(dataOfParameterBlock.second.address, dataOfParameterBlock.second.address, cov.getAddress() );
		multiplyCovarianceMatrixByVarianceOfUnitWeight(cov);
		
		covarianceMatricesForDiagonalBlocks.at(dataOfParameterBlock.first.groupName).emplace(dataOfParameterBlock.first, cov);

	}

	void CovarianceComputer::multiplyCovarianceMatrixByVarianceOfUnitWeight(CovarianceMatrix & cov) const
	{
		cov.matrix = varianceOfUnitWeight * cov.matrix;
	}
}
