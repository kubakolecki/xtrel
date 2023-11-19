#pragma once
#include<map>
#include<tuple>
#include<optional>

namespace ba
{
	enum class NamesOfParameterGroups
	{
		UNDEFINED,
		ORIENTATION_COORDINATES,
		ORIENTATION_ANGLES,
		ORIENTATION_QUATERNION,
		CAMERA_INTERNAL_ORIENTATION,
		CAMERA_RADIAL_DISTORTION_K12,
		CAMERA_RADIAL_DISTORTION_K3,
		CAMERA_TANGENTIAL_DISTORTION,
		OBJECT_POINT_COORDINATES
	};

	class ParameterGroupDictionary
	{
	public:
		const static inline std::map<const NamesOfParameterGroups,const std::string> dictionary
		{
			std::pair{NamesOfParameterGroups::ORIENTATION_COORDINATES, "ORIENTATION_COORDINATES"},
			std::pair{NamesOfParameterGroups::ORIENTATION_ANGLES, "ORIENTATION_ANGLES"},
			std::pair{NamesOfParameterGroups::ORIENTATION_QUATERNION, "ORIENTATION_QUATERNION"},
			std::pair{NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION, "CAMERA_INTERNAL_ORIENTATION"},
			std::pair{NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12, "CAMERA_RADIAL_DISTORTION_K12"},
			std::pair{NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3, "CAMERA_RADIAL_DISTORTION_K3"},
			std::pair{NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION, "CAMERA_TANGENTIAL_DISTORTION"},
			std::pair{NamesOfParameterGroups::OBJECT_POINT_COORDINATES, "OBJECT_POINT_COORDINATES"}
		};

		const static inline std::map<const std::string, const NamesOfParameterGroups> inverseDictionary
		{
			std::pair{"ORIENTATION_COORDINATES", NamesOfParameterGroups::ORIENTATION_COORDINATES},
			std::pair{"ORIENTATION_ANGLES",	NamesOfParameterGroups::ORIENTATION_ANGLES },
			std::pair{"ORIENTATION_QUATERNION", NamesOfParameterGroups::ORIENTATION_QUATERNION},
			std::pair{"CAMERA_INTERNAL_ORIENTATION", NamesOfParameterGroups::CAMERA_INTERNAL_ORIENTATION	},
			std::pair{"CAMERA_RADIAL_DISTORTION_K12", NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K12 },
			std::pair{"CAMERA_RADIAL_DISTORTION_K3", NamesOfParameterGroups::CAMERA_RADIAL_DISTORTION_K3	},
			std::pair{"CAMERA_TANGENTIAL_DISTORTION", NamesOfParameterGroups::CAMERA_TANGENTIAL_DISTORTION },
			std::pair{"OBJECT_POINT_COORDINATES", NamesOfParameterGroups::OBJECT_POINT_COORDINATES}
		};


	};

	class IdentifierOfParameterGroup
	{
	public:
		IdentifierOfParameterGroup() : groupName(NamesOfParameterGroups::UNDEFINED), parameterLabel("") {}
		IdentifierOfParameterGroup(NamesOfParameterGroups idOfFirstGroup, const std::string& label) : groupName(idOfFirstGroup), parameterLabel(label) {}

		
		NamesOfParameterGroups groupName;
		std::string parameterLabel;
		
		std::string getLabel() const
		{
			auto gropuNameStr{ ParameterGroupDictionary::dictionary.at(groupName) };
			const auto label{ gropuNameStr + " OF " + parameterLabel };
			return label;
		}

		friend bool operator<(const IdentifierOfParameterGroup& lhs, const IdentifierOfParameterGroup& rhs)
		{
			return std::tie(lhs.groupName, lhs.parameterLabel) 
				 < std::tie(rhs.groupName, rhs.parameterLabel);
		}

		friend bool operator>(const IdentifierOfParameterGroup& lhs, const IdentifierOfParameterGroup& rhs)
		{
			return std::tie(lhs.groupName, lhs.parameterLabel)
				 > std::tie(rhs.groupName, rhs.parameterLabel);
		}
		
		friend bool operator==(const IdentifierOfParameterGroup& lhs, const IdentifierOfParameterGroup& rhs)
		{
			return std::tie(lhs.groupName, lhs.parameterLabel)
				== std::tie(rhs.groupName, rhs.parameterLabel);
		}

		friend bool operator!=(const IdentifierOfParameterGroup& lhs, const IdentifierOfParameterGroup& rhs)
		{
			return std::tie(lhs.groupName, lhs.parameterLabel)
				!= std::tie(rhs.groupName, rhs.parameterLabel);
		}
	};

	class IdentifierOfParameterGroupPair // for example for covariance or colrrelation matrices
	{
	public:
		
		IdentifierOfParameterGroupPair() : idOfFirstGroup{ IdentifierOfParameterGroup() }, idOfSecondGroup{ IdentifierOfParameterGroup() }
		{}

		IdentifierOfParameterGroupPair(IdentifierOfParameterGroup id1, IdentifierOfParameterGroup id2) : idOfFirstGroup{ id1 }, idOfSecondGroup{id2}
		{}

		IdentifierOfParameterGroup idOfFirstGroup;
		IdentifierOfParameterGroup idOfSecondGroup;

		friend bool operator<(const IdentifierOfParameterGroupPair& lhs, const IdentifierOfParameterGroupPair& rhs)
		{
			return std::tie(lhs.idOfFirstGroup, lhs.idOfSecondGroup)
				< std::tie(rhs.idOfFirstGroup, rhs.idOfSecondGroup);
		}

		friend bool operator>(const IdentifierOfParameterGroupPair& lhs, const IdentifierOfParameterGroupPair& rhs)
		{
			return std::tie(lhs.idOfFirstGroup, lhs.idOfSecondGroup)
				> std::tie(rhs.idOfFirstGroup, rhs.idOfSecondGroup);
		}

		friend bool operator==(const IdentifierOfParameterGroupPair& lhs, const IdentifierOfParameterGroupPair& rhs)
		{
			return std::tie(lhs.idOfFirstGroup, lhs.idOfSecondGroup)
				== std::tie(rhs.idOfFirstGroup, rhs.idOfSecondGroup);
		}

		friend bool operator!=(const IdentifierOfParameterGroupPair& lhs, const IdentifierOfParameterGroupPair& rhs)
		{
			return std::tie(lhs.idOfFirstGroup, lhs.idOfSecondGroup)
				!= std::tie(rhs.idOfFirstGroup, rhs.idOfSecondGroup);
		}

	};

	struct DataOfParameterGroup
	{
		DataOfParameterGroup(const double* address, int size) : address(address), size(size) {}
		const double* address;
		int size;

	};


	class ParameterAddressBook
	{
	public:
		void insert(const IdentifierOfParameterGroup id, const DataOfParameterGroup data);
		std::optional<DataOfParameterGroup> getParameterGroupData(IdentifierOfParameterGroup id) const;
		std::vector<std::pair<IdentifierOfParameterGroup, const DataOfParameterGroup*>> getAllParameterDataForGivenCamera(const std::string& cameraId) const;

		void clear();
		const std::map<IdentifierOfParameterGroup, DataOfParameterGroup>& getAddresses() const;
		const std::set<IdentifierOfParameterGroup>& getIdentifiers() const;
		void writeToFile(const std::string& filename);
	private:
		std::map<IdentifierOfParameterGroup, DataOfParameterGroup> addresses;
		std::set<IdentifierOfParameterGroup> identifiers;
	};


}
