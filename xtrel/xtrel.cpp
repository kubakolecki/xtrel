// whiteduck.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include "BundleAdjustmentData.h"
#include "BundleAdjustment.h"
#include "BundleAdjustmentReport.h"
#include "CalibrationCertificate.h"


int main(int argc, char **argv)
//int main()
{
	ba::BundleAdjustmentData BAInputData;

	BAInputData.FilenameSettings = argv[1];

	if (!BAInputData.read_data())
	{
		std::cout << "error while reading data" <<std::endl;
		std::cout << "exiting the application" << std::endl;
		return -1;
	}

	BAInputData.Settings.print_in_console();

	ba::BundleAdjustment BA(BAInputData);
	ba::BundleAdjustmentReport BAReport(BAInputData, BA, BAInputData.Settings.InputFiles.FilenameReport);


	if (BAInputData.Settings.GenerateCalibrationCertificate == 1)
	{
		try
		{
			CalibrationCertificate Certificate(BAInputData.Settings.InputFiles.FilenameCalibrationCertificateData);
			for (auto &c : BAInputData.ImageOrientationData.DataCameras)
			{
				auto p1 = c.first.find("CAM\\");
				auto p2 = c.first.rfind(".cam");
				std::string cam_filename = c.first.substr(p1 + 4, p2 - p1 - 4);
				auto pos = BAInputData.Settings.InputFiles.FilenameReport.rfind("\\");
				std::string tex_filename = BAInputData.Settings.InputFiles.FilenameReport.substr(0, pos+1) + cam_filename +  + "_certificate.tex";
				Certificate.generate(c.second, BAInputData.Settings, BA, tex_filename);
			}

		}
		catch (std::exception e)
		{
			std::cout << "Error while generating camera calibration certificate:" << std::endl;
			std::cout << e.what() << std::endl;

		}
	}
	//std::cout << "press enter to exit the application" << std::endl;
	//std::cin.get();

	return 0;
}

