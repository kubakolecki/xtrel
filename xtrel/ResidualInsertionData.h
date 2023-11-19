#pragma once

#include<vector>
#include<map>
#include<cstdarg>

namespace ba
{
	
	enum class categoryOfObservation
	{
		IMAGE_POINT_COORDINATE,
		OBJECT_POINT_COORDINATE,
		PROJECTION_CENTER_COORDINATE,
		IMAGE_ORIENTATION_ANGLE,
		GEODETIC_ANGLE_MEASUREMENT,
		GEODETIC_DISTANCE_MEASUREMENT,
	};

	enum class componentOfObservation
	{
		X,
		Y,
		Z,
		OMEGA,
		PHI,
		KAPPA,
		ALPHA,
		NU,
		HEADING,
		PITCH,
		ROLL,
		HORIZONTAL,
		VERTICAL,
		SLANT
	};

	inline std::map<categoryOfObservation, std::string> categoryOfObservationDictionary
	{
		{
		{categoryOfObservation::IMAGE_POINT_COORDINATE, "IMAGE_POINT_COORDINATE"},
		{categoryOfObservation::OBJECT_POINT_COORDINATE, "OBJECT_POINT_COORDINATE"},
		{categoryOfObservation::PROJECTION_CENTER_COORDINATE, "PROJECTION_CENTER_COORDINATE"},
		{categoryOfObservation::IMAGE_ORIENTATION_ANGLE, "IMAGE_ORIENTATION_ANGLE"},
		{categoryOfObservation::GEODETIC_ANGLE_MEASUREMENT, "GEODETIC_ANGLE_MEASUREMENT"},
		{categoryOfObservation::GEODETIC_DISTANCE_MEASUREMENT, "GEODETIC_DISTANCE_MEASUREMENT"},
		}
	};

	inline std::map<componentOfObservation, std::string> componentOfObservationDictionary
	{
		{
			{componentOfObservation::X, "X"},
			{componentOfObservation::Y, "Y"},
			{componentOfObservation::Z, "Z"},
			{componentOfObservation::OMEGA, "OMEGA"},
			{componentOfObservation::PHI, "PHI"},
			{componentOfObservation::KAPPA, "KAPPA"},
			{componentOfObservation::ALPHA, "ALPHA"},
			{componentOfObservation::NU, "NU"},
			{componentOfObservation::HEADING, "HEADING"},
			{componentOfObservation::PITCH, "PITCH"},
			{componentOfObservation::ROLL, "ROLL"},
			{componentOfObservation::HORIZONTAL, "HORIZONTAL"},
			{componentOfObservation::VERTICAL, "VERTICAL"},
			{componentOfObservation::SLANT, "SLANT"}
		}
	};

	template<typename LABELTYPE>
	class IdentifierOfObservation
	{
	public:
		IdentifierOfObservation(categoryOfObservation category, componentOfObservation component, std::initializer_list<LABELTYPE> labelsToInsert) :category{ category }, component{ component }, labels{ labelsToInsert }
		{
			//labels = std::vector<std::string>{ labelsToInsert ... };
		}
		
		categoryOfObservation category;
		componentOfObservation component;
		std::vector<std::string> labels;
	};

	using ObservationId = IdentifierOfObservation<std::string>;

	struct ResidualData
	{
		ResidualData(int rowId, double val) : rowInJacobian(rowId), value(val)
		{}

		int rowInJacobian{ 0 };
		double value{ 0.0 };
	};

	class ResidualInsertionData
	{
		using ResidualInformation = std::pair<IdentifierOfObservation<std::string>, ResidualData>;
	public:
		ResidualInsertionData()
		{
			data.emplace(categoryOfObservation::IMAGE_POINT_COORDINATE, std::vector<ResidualInformation>());
			data.emplace(categoryOfObservation::OBJECT_POINT_COORDINATE, std::vector<ResidualInformation>());
			data.emplace(categoryOfObservation::PROJECTION_CENTER_COORDINATE, std::vector<ResidualInformation>());
			data.emplace(categoryOfObservation::IMAGE_ORIENTATION_ANGLE, std::vector<ResidualInformation>());
			data.emplace(categoryOfObservation::GEODETIC_ANGLE_MEASUREMENT, std::vector<ResidualInformation>());
			data.emplace(categoryOfObservation::GEODETIC_DISTANCE_MEASUREMENT, std::vector<ResidualInformation>());
		}

		const std::map < categoryOfObservation, std::vector<ResidualInformation>>& getData() const
		{
			return data;
		}

		const std::vector<ResidualInformation>& getData(categoryOfObservation category) const
		{
			return data.at(category);
		}

		void insertData(IdentifierOfObservation<std::string> id, int rowInJacobian, double value)
		{
			data.at(id.category).emplace_back(std::pair<IdentifierOfObservation<std::string>, ResidualData>( id, ResidualData( rowInJacobian, value ) ) );
		}


	private:
		std::map < categoryOfObservation, std::vector<ResidualInformation>> data;
	};
}