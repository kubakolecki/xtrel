#include "pch.h"
#include "ParameterAddressBook.h"

namespace ba
{

	void ParameterAddressBook::insert(const IdentifierOfParameterGroup id, const DataOfParameterGroup data)
	{
		addresses.emplace(id, data);
		identifiers.insert(id);
	}

	std::optional<DataOfParameterGroup> ParameterAddressBook::getParameterGroupData(IdentifierOfParameterGroup id) const
	{
		if (addresses.count(id) == 0)
		{
			return std::nullopt;
		}
		else
		{
			return addresses.at(id);
		}

	}

	std::vector<std::pair<IdentifierOfParameterGroup, const DataOfParameterGroup*>> ParameterAddressBook::getAllParameterDataForGivenCamera(const std::string & cameraId) const
	{
		std::vector<std::pair<IdentifierOfParameterGroup, const DataOfParameterGroup*>> requestedCameraParameterData;

		std::array<IdentifierOfParameterGroup, 4> identifiresToRequest =
		{
			{{NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION,  cameraId },
			 {NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, cameraId },
			 {NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3,  cameraId },
			 {NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, cameraId }}
		};

		for (const auto& idToAskFor : identifiresToRequest)
		{
			const auto dataOfParameters{ getParameterGroupData(idToAskFor) };
			if (dataOfParameters.has_value())
			{
				requestedCameraParameterData.push_back({ idToAskFor, &dataOfParameters.value() });
			}
		}
		
		return requestedCameraParameterData;
	}

	void ParameterAddressBook::clear()
	{
		addresses.clear();
	}

	const std::map<IdentifierOfParameterGroup, DataOfParameterGroup>& ParameterAddressBook::getAddresses() const
	{
		return addresses;
	}

	const std::set<IdentifierOfParameterGroup>& ParameterAddressBook::getIdentifiers() const
	{
		return identifiers;
	}

	void ParameterAddressBook::writeToFile(const std::string & filename)
	{
		std::ofstream str;
		str.open(filename);

		for (const auto& address : addresses)
		{
			str << std::setw(45) << address.first.getLabel() << " "<<std::setw(7) << address.second.size << " "<<std::setw(12) << address.second.address << "\n";
		}


		str.close();
	}

}