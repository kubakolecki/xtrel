#include "pch.h"
#include "ImagePointContainer.h"


ImagePointContainer::ImagePointContainer()
{
}


ImagePointContainer::~ImagePointContainer()
{
}

bool ImagePointContainer::read_from_file(std::string filename)
{
	Data.clear(); //You have to clear the data first!
	
	std::ifstream str;
	str.open(filename);
	if (!str)
	{
		std::cout << "cannot open " << filename << std::endl;
		str.close();
		return false;
	}
	else
	{	
		double x, y;
		int status;
		std::string point_name, image_name;
		std::string linia; //the whole line in file

		while (std::getline(str, linia, '\n'))
		{
			//if there is no blak space inside the string than the line represents the image name
			int where_blank; //Gdzie jest spacja
			where_blank = linia.find_first_of(" ");
			std::istringstream str(linia);
			if (where_blank == -1)
			{
				str >> image_name;
			}
			else
			{
				str >> point_name >> x >> y >> status;
				Data.emplace_back(point_name, image_name, x, y, status);
			}
		}
		str.close();
		return true;
	}

}

bool ImagePointContainer::filter_out(std::set<std::string> valid_images)
{
	std::vector<ImagePoint> Temp;
	Temp.reserve(Data.size());
	for (auto&p : Data)
	{
		if (valid_images.count(p.ImageName) > 0) Temp.push_back(p);
	}

	Temp.shrink_to_fit();
	Data = Temp;

	return false;
}

void ImagePointContainer::print_in_console()
{
	std::cout << "ImagePointContainer Data:\n";
	for (auto &p : Data)
	{
		std::cout <<"point: " << p.Name << " " << p.ImageName << " " <<
			p.X << " " << p.Y << " " << p.Status << "\n";
	}
	std::cout.flush();

}
