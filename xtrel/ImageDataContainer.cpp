#include "pch.h"
#include "ImageDataContainer.h"

ImageDataContainer::ImageDataContainer()
{
}


ImageDataContainer::~ImageDataContainer()
{
}

bool ImageDataContainer::read_from_file(std::string filename )
{
	DataCameras.clear();
	DataImages.clear();
	std::ifstream str;
	str.open(filename);
	if (str)
	{
		string name{ "" };
		double coords[3]{ 0.0, 0.0, 0.0 };
		double angles[3]{ 0.0, 0.0, 0.0 };
		double cerrors[3]{ 0.001, 0.001, 0.001 };
		double aerrors[3]{ 0.001, 0.001, 0.001 };
		string parametrization{ "" };
		string camera_file_name{ "" };
		int use{ 1 };


		while (str)
		{
			if (str)
			{
				str >> name >> coords[0] >> coords[1] >> coords[2]
					>> angles[0] >> angles[1] >> angles[2]
					>> cerrors[0] >> cerrors[1] >> cerrors[2]
					>> aerrors[0] >> aerrors[1] >> aerrors[2]
					>> parametrization
					>> camera_file_name
					>> use;

				if (use == 0) continue; // this image is flagged out!

				for (int i : {0, 1, 2}) //all math will be done in radians!
				{
					angles[i] *= RHO;
					aerrors[i] *= RHO;
				}
 
				Camera cam(camera_file_name.c_str());
				EO eo(coords, angles, cerrors, aerrors);
				ImageData imagedata(eo, name, camera_file_name, cam.Name);
				imagedata.EOApproximated.RotParametrization = parametrization;

				DataCameras.emplace(cam.Name, cam);
				DataImages.emplace(name, imagedata);
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

void ImageDataContainer::print_image_data()
{
	std::cout << "\nImageDataContainer: DataImages:\n";
	for (auto& i : DataImages)
	{
		std::cout << i.second.Name << " "
			<< i.second.EOApproximated.Coords[0] << " " << i.second.EOApproximated.Coords[1] << " " << i.second.EOApproximated.Coords[2] << " "
			<< i.second.EOApproximated.Angles[0] << " " << i.second.EOApproximated.Angles[1] << " " << i.second.EOApproximated.Angles[2] << " "
			<< i.second.CameraName <<" "
			<< DataCameras.at(i.second.CameraName).InternalOrientation[0] << std::endl;
	}
}

void ImageDataContainer::print_camera_data()
{
	std::cout << "\nImageDataContainer: DataCameras:\n";
	for (auto& c: DataCameras)
	{
		c.second.wypisz();
	}
}

ImageData ImageDataContainer::get_image_data(std::string name)
{
	return DataImages.at(name);
}

Camera ImageDataContainer::get_camera_data(std::string name)
{
	return DataCameras.at(name);
}
