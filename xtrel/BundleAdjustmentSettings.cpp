#include "pch.h"
#include "BundleAdjustmentSettings.h"

using namespace ba;

BundleAdjustmentSettings::BundleAdjustmentSettings()
{}

BundleAdjustmentSettings::BundleAdjustmentSettings(std::string filename)
{
	read_from_file(filename);
}

BundleAdjustmentSettings::~BundleAdjustmentSettings() {}

bool ba::BundleAdjustmentSettings::read_from_file(std::string filename)
{
	CamFixMasks.clear();
	std::fstream str_check;
	std::fstream str;

	//checking tags:
	std::vector<std::string> tags;
	str_check.open(filename);
	if (str_check)
	{
		while (str_check)
		{
			if (str_check)
			{
				std::string tag = "";
				std::string line = "";
				std::getline(str_check, line);
				std::stringstream sl(line);
				sl >> tag;
				tags.push_back(tag);
			}
		}
		str_check.close();
	}
	else
	{
		std::cout << "cannot open " << filename << std::endl;
		str_check.close();
		return false;
	}

	for (auto &t : RequireTags)
	{
		if (std::count(tags.begin(), tags.end(), t) == 0)
		{
			std::cout << "No tag: '" << t << "' found in the settings file!" << std::endl;
			std::cout << "Have You forgot about ':'?" << std::endl;
			std::cout << "Have You left one blank space after ':'?" << std::endl;
			std::cout << "Or is there any other typo in the tag you've provided?" << std::endl;
			std::cout << "Correct the error and rerun the applictaion." << std::endl;
			return false;
		}
	}
	

	str.open(filename);

	if (str)
	{
		while (str)
		{
			if (str)
			{
				std::string tag = "";
				std::string line = "";
				std::getline(str, line);
				std::stringstream sl(line);
				sl >> tag;
				if (tag == "NumOfCameras:") sl >> NumOfCameras;
				if (tag == "ImageMesAcc:") sl >> ImageMesAcc;
				if (tag == "HzAngleMesAcc:")
				{
					sl >> GeodeticHzMesAcc;
					GeodeticHzMesAcc *= 1.57079632679e-06; //conversion from cc to radians
				}
				if (tag == "VAngleMesAcc:")
				{
					sl >> GeodeticVMesAcc;
					GeodeticVMesAcc *= 1.57079632679e-06; //conversion from cc to radians
				}
				if (tag == "DistMesAcc:")
				{
					sl >> GeodeticDistMesAcc;
				}
				if (tag == "MathModel:")
				{
					std::string bamathmodel;
					sl >> bamathmodel;
					if (bamathmodel == "RIGID" ||
						bamathmodel == "SOFT" ||
						bamathmodel == "TIGHT" ) {
						if (bamathmodel == "RIGID") MathModel = BAMathModel::RIGID;
						if (bamathmodel == "SOFT") MathModel = BAMathModel::SOFT;
						if (bamathmodel == "TIGHT") MathModel = BAMathModel::TIGHT;
					}
					else
					{
						MathModel = BAMathModel::SOFT;
						std::cout << "WARNING! Incorrect MathModel in the config file!\n";
						std::cout << "Using SOFT model!" << std::endl;
					}
				}
				if (tag == "LossFunction:")
				{
					std::string balossfunction;
					sl >> balossfunction;
					if (balossfunction == "NONE" ||
						balossfunction == "CAUCHY" ||
						balossfunction == "HUBER")  {
						if (balossfunction == "NONE") LossFunction = BALossFunction::NONE;
						if (balossfunction == "CAUCHY") LossFunction = BALossFunction::CAUCHY;
						if (balossfunction == "HUBER") LossFunction = BALossFunction::HUBER;
					}
					else
					{
						LossFunction = BALossFunction::NONE;
						std::cout << "WARNING! Incorrect LossFunction in the config file!\n";
						std::cout << "Solution will not use loss function!" << std::endl;
					}
				}

				if (tag == "LossFunctionParameter:")
				{
					sl >> LossFunctionParameter;
				}

				if (tag == "CamFixMasks:")
				{				
					for (unsigned int i = 0; i < NumOfCameras; i++)
					{
						std::getline(str, line);
						std::stringstream sc(line);
						std::string cameraname = "";
						std::string fixmask; 
						sc >> cameraname  >> fixmask;
						unsigned char m = 0;
						for (char c : fixmask)
						{
							m <<= 1;
							m += (c - '0') & 1;
						}
						cameraname = cameraname.substr(0, cameraname.length() - 1);
						CamFixMasks.emplace(cameraname, m);
						std::cout << "cameraname: " << cameraname <<" fixmask: "<< fixmask << std::endl;
					}
				}

				if (tag == "ComputeCorrelations:")
				{
					sl >> ComputeCorrelations;
				}

				if (tag == "ComputeRedundancy:")
				{
					sl >> ComputeRedundancy;
				}

				if (tag == "ComputeCovarianceBetweenParameterGroups:")
				{
					sl >> ComputeCovarianceBetweenParameterGroups;
				}

				if (tag == "PathOfFileWithParmeterGroupPairs:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					
					InputFiles.FilenameParmeterGroupPairs = get_filename(line);
				}


				if (tag == "GenerateCalibrationCertificate:")
				{
					sl >> GenerateCalibrationCertificate;
				}

				if (tag == "FilenameImagePoints:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" <<std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameImagePoints = get_filename(line);
				}

				if (tag == "FilenameObjectPoints:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameObjectPoints = get_filename(line);
				}

				if (tag == "FilenameGeodeticClotrollPoints:" && MathModel == BAMathModel::TIGHT)
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameGeodeticClotrollPoints = get_filename(line);
				}

				if (tag == "FilenameExternalOrientation:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameExternalOrientation = get_filename(line);
				}

				if (tag == "FilenameGeodeticMeasurements:" && MathModel == BAMathModel::TIGHT)
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameGeodeticMeasurements = get_filename(line);
				}

				if (tag == "FilenameReport:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameReport = get_filename(line);
				}

				if (tag == "FilenameCalibrationCertificateData:")
				{
					if (!check_filename(line))
					{
						std::cout << "Error in " << tag << " string!" << std::endl;
						std::cout << "Have you forgotten the quotation marks?" << std::endl;
						std::cout << "Correct the error and rerun the application." << std::endl;
						return false;
					}
					InputFiles.FilenameCalibrationCertificateData = get_filename(line);
				}

			}
		}
		str.close();
		return true;
	}
	else
	{
		std::cout << "cannot open " << filename << std::endl;
		str.close();
		return false;
	}

	
}

void ba::BundleAdjustmentSettings::write_to_file(std::string filename)
{
	std::ofstream str;
	str.open(filename);
	str << "NumOfCameras: " << NumOfCameras << std::endl;
	str << "MathModel: ";
	if (MathModel == BAMathModel::RIGID) str << "RIGID";
	if (MathModel == BAMathModel::SOFT) str << "SOFT";
	if (MathModel == BAMathModel::TIGHT) str << "TIGHT";
	str << " #RIGID, SOFT, TIGHT\n";
	str << "ImageMesAcc: " << std::fixed << std::setprecision(2) << ImageMesAcc << "  #[px]\n";
	str << "HzAngleMesAcc: " << std::fixed << std::setprecision(0) << GeodeticHzMesAcc/ 1.57079632679e-06 << "  #[cc]\n";
	str << "VAngleMesAcc: " << std::fixed << std::setprecision(0) << GeodeticVMesAcc / 1.57079632679e-06 << "  #[cc]\n";
	str << "DistMesAcc: " << std::fixed << std::setprecision(5) << GeodeticDistMesAcc << " #[m]\n";
	str << "LossFunction: ";
	if (LossFunction == BALossFunction::NONE) str << "NONE";
	if (LossFunction == BALossFunction::CAUCHY) str << "CAUCHY";
	if (LossFunction == BALossFunction::HUBER) str << "HUBER";
	str << " #NONE, CAUCHY, HUBER\n";
	str << "LossFunctionParameter: " << std::fixed << std::setprecision(2) << LossFunctionParameter << std::endl;
	str << "CamFixMasks:\n";
	for (auto& c : CamFixMasks)
	{
		str << " " << c.first <<": ";
		if (c.second & ba_fix_masks::mask_fix_io) str << "1"; else str << "0";
		if (c.second & ba_fix_masks::mask_fix_k) str << "1"; else str << "0";
		if (c.second & ba_fix_masks::mask_fix_k3) str << "1"; else str << "0";
		if (c.second & ba_fix_masks::mask_fix_p) str << "1"; else str << "0";
		str << std::endl;
	}
	str << "GenerateCalibrationCertificate: " << GenerateCalibrationCertificate << " #0-no, 1-yes\n";
	str << "FilenameImagePoints: " << "'"<< InputFiles.FilenameImagePoints <<"'\n";
	str << "FilenameObjectPoints: " << "'" << InputFiles.FilenameObjectPoints << "'\n";
	str << "FilenameGeodeticClotrollPoints: " << "'" << InputFiles.FilenameGeodeticClotrollPoints << "'\n";
	str << "FilenameExternalOrientation: " << "'" << InputFiles.FilenameExternalOrientation << "'\n";
	str << "FilenameGeodeticMeasurements: " << "'" << InputFiles.FilenameGeodeticMeasurements << "'\n";
	str << "FilenameReport: " << "'" << InputFiles.FilenameReport << "'\n";
	str << "FilenameCalibrationCertificateData: " << "'" << InputFiles.FilenameCalibrationCertificateData << "'\n";
	str.close();
}

void ba::BundleAdjustmentSettings::print_in_console()
{
	std::cout << "\nBundle Adjustment Settings" << std::endl;
	std::cout << "NumOfCameras: " << NumOfCameras << std::endl;
	std::cout << "ImageMesAcc: " << ImageMesAcc << std::endl;
	std::cout << "MathModel: " << (int)MathModel << std::endl;
	std::cout << "Camera Calibration Fix Masks" << std::endl;
	for (auto &c : CamFixMasks)
	{
		std::cout << c.first << " " << static_cast<int>(c.second) << std::endl;
	}

}

bool ba::BundleAdjustmentSettings::check_filename(std::string & ln)
{
	auto pos = ln.find(':');
	if (ln.length() < pos + 4)
	{
		return false;
	}
	if (ln.at(pos + 2) != '\'')
	{
		return false;
	}
	if (ln.back() != '\'')
	{
		return false;
	}
	
	return true;
}

std::string ba::BundleAdjustmentSettings::get_filename(std::string & ln)
{
	auto pos1 = ln.find('\'');
	auto pos2 = ln.rfind('\'');
	return ln.substr(pos1+1, pos2 - pos1-1);
}
