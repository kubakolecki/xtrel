#pragma once
#include "BundleAdjustmentData.h"
#include "BundleAdjustment.h"

class CalibrationCertificate
{
public:
	CalibrationCertificate(std::string filename_data);
	~CalibrationCertificate();
	void generate(const Camera& camera, const ba::BundleAdjustmentSettings& settings, const ba::BundleAdjustment& ba,  std::string filename_tex);
	std::wstring ContractingAuthority{ L"" };
	std::wstring ContractingAuthorityAddress{ L"" };
	std::wstring Contractor{ L"" };
	std::wstring ContractorAddress{ L"" };
	std::wstring Worker{ L"" };
	std::wstring Date{ L"" };


private:
	bool check_information(std::wstring & ln);
	std::wstring get_information(std::wstring & ln);
	string num2str(int num);
	std::wstring to_wstring(std::string str);


};

