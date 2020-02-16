#include "pch.h"
#include "AngularSurvey3DContainer.h"


AngularSurvey3DContainer::AngularSurvey3DContainer()
{
}


AngularSurvey3DContainer::~AngularSurvey3DContainer()
{
}

bool AngularSurvey3DContainer::read_from_file(std::string filename)
{
	Data.clear();
	std::ifstream str;
	str.open(filename);
	std::string l, r, c; //left, right, center
	double hz, v;
	constexpr double rho = 0.01570796326794896619;
	if (str)
	{
		while (str)
		{
			if (str)
			{
				str >> l >> r >> c >> hz >> v;
				Data.emplace_back(l, r, c, rho*hz, rho*v);
			}
		}
	}
	else
	{
		std::cout << "cannot open " << filename << std::endl;
		str.close();
		return false;
	}
	str.close();
	return true;

}

void AngularSurvey3DContainer::print_in_console()
{
	std::cout << "\nGeodetic angular measurements:" << std::endl;
	for (auto &d : Data)
	{
		std::cout << d.Ids[0] << " " << d.Ids[1] << " " << d.Ids[2] << " adj.index: "<<d.AdjustedPointId <<" "  << std::fixed << std::setprecision(6) << d.Hz << " " << d.V << std::endl;
	}
}
