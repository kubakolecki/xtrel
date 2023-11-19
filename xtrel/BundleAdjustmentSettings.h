#pragma once
#include <map>

struct BAInputFiles
{
	std::string FilenameImagePoints{ "" };
	std::string FilenameObjectPoints{ "" };
	std::string FilenameGeodeticClotrollPoints{ "" };
	std::string FilenameExternalOrientation{ "" };
	std::string FilenameGeodeticMeasurements{ "" };
	std::string FilenameReport{ "" };
	std::string FilenameCalibrationCertificateData{ "" };
	std::string FilenameParmeterGroupPairs{ "" };
};

namespace ba_fix_masks
{
	constexpr unsigned char mask_fix_io = 0b0000'1000; //bit 0 - fix io
	constexpr unsigned char mask_fix_k = 0b0000'0100; //bit 1 - fix k1 and k2 radial distortion coeff
	constexpr unsigned char mask_fix_k3 = 0b0000'0010; //bit 2 - fix k3 raidial distortion coeff
	constexpr unsigned char mask_fix_p = 0b0000'0001; //bit 3 - fix tangential distortion
}

namespace ba
{
	enum class BAMathModel {RIGID=0, SOFT=1, TIGHT=2 };
	enum class BALossFunction { NONE = 0, CAUCHY=1, HUBER=2 };

	class BundleAdjustmentSettings
	{
	public:
		BundleAdjustmentSettings();
		BundleAdjustmentSettings(std::string filename);
		~BundleAdjustmentSettings();
		bool read_from_file(std::string filename);
		void write_to_file(std::string filename);
		void print_in_console();
		
		BAInputFiles InputFiles;

		unsigned int NumOfCameras = 1;
		double ImageMesAcc = 0.5; //image measurement accuracy [px]
		double GeodeticHzMesAcc = 0.000031415; //horizontal angle measurement accuracy in radians
		double GeodeticVMesAcc =  0.000078537; //horizontal angle measurement accuracy in radians
		double GeodeticDistMesAcc = 0.002; //[m] distance measurement accuracy
		double LossFunctionParameter = 2.0;
		int ComputeCorrelations{ 0 };
		int ComputeRedundancy{ 0 };
		int ComputeCovarianceBetweenParameterGroups{ 0 };
		int GenerateCalibrationCertificate{ 0 };
		
		
		BAMathModel MathModel = BAMathModel::SOFT;
		BALossFunction LossFunction = BALossFunction::NONE;
		std::map<std::string, unsigned char> CamFixMasks;

	private:
		std::vector<std::string> RequireTags{
			"NumOfCameras:",
			"ImageMesAcc:",
			"HzAngleMesAcc:",
			"VAngleMesAcc:",
			"DistMesAcc:",
			"MathModel:",
			"LossFunction:",
			"LossFunctionParameter:",
			"CamFixMasks:",
			"ComputeCorrelations:",
			"ComputeRedundancy:",
			"GenerateCalibrationCertificate:",
			"FilenameImagePoints:",
			"FilenameObjectPoints:",
			"FilenameGeodeticClotrollPoints:",
			"FilenameExternalOrientation:",
			"FilenameGeodeticMeasurements:",
			"FilenameReport:",
			"FilenameCalibrationCertificateData:"
		};

		bool check_filename(std::string& ln);
		std::string get_filename(std::string& ln);

	};//class BundleAdjustmentSettings

} //namespace ba