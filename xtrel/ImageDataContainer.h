#pragma once
#include <map>
#include "ImageData.h"
#include "Camera.h"

namespace ba
{
	using DataOfImages = std::map<std::string, ImageData>;
	using DataOfCameras = std::map<std::string, Camera>;


	class ImageDataContainer
	{
	public:
		ImageDataContainer();
		~ImageDataContainer();
		bool read_from_file(std::string filename);
		void print_image_data();
		void print_camera_data();
		ImageData get_image_data(std::string name);
		Camera get_camera_data(std::string name);

		DataOfImages DataImages;
		DataOfCameras DataCameras;

	private:
		const double RHO{ 3.14159265358979323846264 / 180.0 };

	};
}

