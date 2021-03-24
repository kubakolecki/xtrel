#include "pch.h"
#include <codecvt>
#include <fcntl.h>
#include <io.h>
#include "CalibrationCertificate.h"

using convert_t = std::codecvt_utf8<wchar_t>;
std::wstring_convert<convert_t, wchar_t> strconverter;

CalibrationCertificate::CalibrationCertificate(std::string filename_data)
{
	std::wifstream str;
	std::wstring tag{ L"" };
	std::wstring information{ L"" };
	std::wstring line{ L"" };
	str.open(filename_data);
	if (str)
	{
		int l = 0;
		while (str)
		{
			std::getline(str, line);
			if (str && line.length() >5)
			{
				
				if (!check_information(line))
				{
					throw std::exception( (std::string("Error in line ") + num2str(l) + " of Calibration Certificate Data File").c_str() );
				}
				std::wstringstream sl(line);
				sl >> tag;
				if (tag == L"contracting_authority:") ContractingAuthority = get_information(line);
				if (tag == L"contracting_authority_address:") ContractingAuthorityAddress = get_information(line);
				if (tag == L"contractor:") Contractor = get_information(line);
				if (tag == L"contractor_address:") ContractorAddress = get_information(line);
				if (tag == L"worker:") Worker = get_information(line);
				if (tag == L"date:") Date = get_information(line);
				l++;
			}
		}
	}
	else
	{
		throw std::exception((std::string("Problem with openning ") + filename_data).c_str());
	}
	str.close();
}


CalibrationCertificate::~CalibrationCertificate()
{
}

void CalibrationCertificate::generate(const Camera& camera, const ba::BundleAdjustmentSettings& settings, const ba::BundleAdjustment& ba, std::string filename_tex)
{
	std::wofstream str;
	str.imbue(std::locale(std::locale::empty(), new std::codecvt_utf8<wchar_t, 0x10ffff, std::generate_header>));
	str.open(filename_tex);

	int a = 5;
	double dim = camera.PixelSize;

	str << "\\documentclass[a4paper, oneside, 10pt]{article}"<<"\n";
	str << "\\usepackage{enumitem}"<<"\n";
	str << "\\usepackage[OT4, T1]{fontenc}"<<"\n";
	str << "\\usepackage{polski}"<<"\n";
	str << "\\usepackage[english]{babel}" << "\n";
	str << "\\usepackage[utf8]{inputenc}"<<"\n";
	str << "\\usepackage{lmodern}"<<"\n";
	str << "\\usepackage{graphicx}"<<"\n";
	str << "\\usepackage{epstopdf}"<<"\n";
	str << "\\usepackage{float}"<<"\n";
	str << "\\usepackage{amsmath}"<<"\n";
	str << "\\usepackage{amsthm}"<<"\n";
	str << "\\usepackage{nccmath}" << "\n";
	str << "\\usepackage{amsfonts}"<<"\n";
	str << "\\usepackage{a4wide}"<<"\n";
	str << "\\usepackage{setspace}" << "\n";
	str << "\\usepackage{fancyhdr}" << "\n";
	str << "\\pagestyle{fancy}" << "\n";
	str << "\\fancyhf{}" << "\n";
	str << "\\fancyhead[LE,RO]{" << to_wstring(camera.Description) << "}" << "\n";
	str << "\\fancyhead[RE,LO]{Certificate of Calibration }" << "\n";
	str << "\\fancyfoot[CE,CO]{\\thepage}" << "\n";
	str << "\\renewcommand{\\headrulewidth}{1pt}" << "\n";
	str << "\\renewcommand{\\footrulewidth}{1pt}" << "\n";

	str << "\\begin{document}"<<"\n";
	str << "\\pagestyle{empty}"<<"\n";
	str << "\\begin{center}"<<"\n";
	str << "\\Large" << "\n";
	str << "\\textbf{CERTIFICATE OF CAMERA CALIBRATION}"<<"\\newline\n";
	str << "\\vskip 0.5in" << "\n";
	str << to_wstring(camera.Description) << "\n";
	str << "\\end{center}"<<"\n";
	str << "\\clearpage" << "\n";
	str << "\\pagestyle{fancy}" << "\n";
	str << "\\large" << "\n";
	str << "\\begin{center}" << "\n";
	str << "\\textbf{FORMAL DATA}" << "\n";
	str << "\\end{center}" << "\n";
	str << "\\textbf{Company/Authority}: " << ContractingAuthority << "\\newline\n";
	str << "\\textbf{Company/Authority address}: " << ContractingAuthorityAddress << "\\newline\n";
	str << "\\textbf{Certification Institution}: " << Contractor << "\\newline\n";
	str << "\\textbf{Certification Institution address}: " << ContractorAddress << "\\newline\n";
	str << "\\textbf{Calibration Date}: " <<Date << "\\newline\n";
	str << "\\textbf{Camera Model}: " << to_wstring(camera.Description) << "\\newline\n";
	str << "\\textbf{Camera Serial Number}: " << to_wstring(camera.CameraSerialNumber) << "\\newline\n";
	str << "\\textbf{Lens Model}: "<< to_wstring(camera.LensName) << "\\newline\n";
	str << "\\textbf{Lens Nominal Focal Length [mm]}: " << camera.LensNominalFocalLength << "\\newline\n";
	str << "\\textbf{Lens Serial Number}: " << to_wstring(camera.LensSerialNumber) << "\\newline\n";
	str << "\\clearpage" << "\n";

	str << "\\large" << "\n";
	str << "\\begin{center}" << "\n";
	str << "\\textbf{DEFINITIONS}" << "\n";
	str << "\\end{center}" << "\n";

	str << "\nCamera parameters provide the relationship between points in the 3D camera frame (object points) and coordinates in image plane.\n";
	str << "In the idealized projection, where no distortion is present, the projection equation are as follows:\n";

	str << "\\begin{equation}" << "\n";
	str << "x' = x_0 - c\\frac{X'}{Z'}" << "\n";
	str << "\\label{eq:1}\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "y' = y_0 - c\\frac{Y'}{Z'}" << "\n";
	str << "\\label{eq:2}\n";
	str << "\\end{equation}" << "\n";

	str << "where:\n";
	str << "$x'$ and $y'$ are the measured image plane coordinates (Figure \\ref{fig:01}) of the object point $P(X',Y',Z')$, projected using the camera\n";
	str << "with principal distance $c$ and principal point $(x_0, y_0)$.\n";
	str << "The Z-axis of the camera frame looks towards back of the camera body\n";
	str << "Parameters $c$, $x_0$ and $y_0$ are known as internal orientation parameters.\n";
	str << "$X'$, $Y'$, $Z'$ provide the position of point $P$ in the 3D camera frame.:\n";
	str << "Please notice that the dimension of $x'$ and $y'$ depends purely on the unit of provided internal orientation parameters and is independant on the dimensions\n";
	str << " of $X$, $Y$ and $Z$.\n";
	str << "In the presence of distortion the polynomial correction coefficients $[k_1, k_2, k_3, p_1, p_2]$ are estimated along with the internal orientation parameters. Mentioned coefficients provide the\n";
	str << "correction to the measured image coordinates. This concept should be distinguished from the case where the distortion coefficients provide the\n";
	str << "displacement of the ideal coordinates to their measured locations. This will be addressed in the further part of this chapter.\n";
	str << "The following observation equations are used for the estimation:\n";
	str << "\\begin{equation}" << "\n";
	str << "x' = x_0 - c\\frac{X'}{Z'} + x(k_1r^2 + k_2r^4 + k_3r^6) + p_1(r^2 + 2x^2) + 2p_2xy\n";
	str << "\\label{eq:3}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "y' = y_0 - c\\frac{Y'}{Z'} + y(k_1r^2 + k_2r^4 + k_3r^6) + 2p_1xy + p_2(r^2 + 2y^2)\n";
	str << "\\label{eq:4}\n";
	str << "\\end{equation}" << "\n";

	str << "where:\n";
	str << "\\begin{equation}" << "\n";
	str <<"x = x'-x_0" << "\n";
	str << "\\label{eq:5}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "y = y'-y_0" << "\n";
	str << "\\label{eq:6}\n";
	str << "\\end{equation}" << "\n";
	str << "and:\n";
	str << "\\begin{equation}" << "\n";
	str <<"r^2  = x^2 + y^2"<<"\n";
	str << "\\label{eq:7}\n";
	str << "\\end{equation}" << "\n";

	str << "This results in the following formulation of the correction to the measured image coordinates:\n";
	str << "\\begin{equation}" << "\n";
	str << "dx = - [x(k_1r^2 + k_2r^4 + k_3r^6) + p_1(r^2 + 2x^2) + 2p_2xy]\n";
	str << "\\label{eq:8}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "dy = - [y(k_1r^2 + k_2r^4 + k_3r^6) + 2p_1xy + p_2(r^2 + 2y^2)]\n";
	str << "\\label{eq:9}\n";
	str << "\\end{equation}" << "\n";
	str << "As a result:\n";
	str << "\\begin{equation}" << "\n";
	str << "x' + dx = x_0 - c\\frac{X'}{Z'}\n";
	str << "\\label{eq:10}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "y' + dy = y_0 - c\\frac{Y'}{Z'}\n";
	str << "\\label{eq:11}\n";
	str << "\\end{equation}" << "\n";
	str << "Note that the dimension of corrections expressed by equations \\ref{eq:8} and \\ref{eq:9} must be the same as the dimension of measurements.\n";
	str << "The value and the dimension of each distortion coefficient depends on the the dimension of measurements. In this document distortion coefficients will be provided\n";
	str << "in 3 spaces: metric (units: [mm]), pixel (units: [px]) and normalized (units: [-]).\n";
	str << "The normalized distortion coefficients provide the correction term for the normalized coordinates i.e. coordinates after division by pricipal distance:\n";
	
	str << "\\begin{equation}" << "\n";
	str << "x_n = \\frac{x'-x_0}{c}" << "\n";
	str << "\\label{eq:12}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "y_n = \\frac{y'-y_0}{c}" << "\n";
	str << "\\label{eq:13}\n";
	str << "\\end{equation}" << "\n";

	str << "resulting in\n";

	str << "\\begin{equation}" << "\n";
	str << "dx_n = - [x_n(k_1r_n^2 + k_2r_n^4 + k_3r_n^6) + p_1(r_n^2 + 2x^2) + 2p_2x_ny_n]\n";
	str << "\\label{eq:14}\n";
	str << "\\end{equation}" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "dy_n = - [y_n(k_1r_n^2 + k_2r_n^4 + k_3r_n^6) + 2p_1x_ny_n + p_2(r_n^2 + 2y_n^2)]\n";
	str << "\\label{eq:15}\n";
	str << "\\end{equation}" << "\n";

	str << "where:\n";
	str << "\\begin{equation}" << "\n";
	str << "r_n^2  = x_n^2 + y_n^2" << "\n";
	str << "\\label{eq:16}\n";
	str << "\\end{equation}" << "\n";

	str << "A number of photogrammetric and computer vision software handles the distortion in a different way:\n";
	str << "Once the object point is projected to the image to the P':[xp, yp] location (the 'ideal location'), the distortion parameters are used to\n";
	str << "shift the P' point to the place where it occurs in real the image (to the measured loation)\n";
	str << "Here we will use the 'reveresed distortion' term, as the distortion is considerd to act in a reversed manner compared to the case where it was treated as an phenomene we aimed to correct for\n";
	str << "As this distortion model is more often used by computer vision community, we will keep the OpenCV library definitions (Figure \\ref{fig:01}):\n";
	str << "(+Z axis: looking forward, along the lens axis)" << "\\newline\n";
	str << "An object point P (X, Y, Z) projects to the image according to the equation (notice the difference in sign):\n";

	str << "\\begin{equation}" << "\n";
	str << "x' = x_0 + c*(X'/Z' + dx')\n";
	str << "\\label{eq:17}\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "y' = y_0 + c*(Y'/Z' + dy')\n";
	str << "\\label{eq:18}\n";
	str << "\\end{equation}" << "\n";

	str << "where:" << "\\newline\n";
	str << "\\begin{equation}" << "\n";
	str << "x_p = c*(X'/Z' + d_x')\n";
	str << "\\label{eq:19}\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "y_p = c*(Y'/Z' + d_y')\n";
	str << "\\label{eq:20}\n";
	str << "\\end{equation}" << "\n";

	
	str << "represent the fiducial position of the undistorted point location (dimension: [px]), $x_0$ and $y_0$ are the principal point coordinates (dimension: [px]),\n";
	str << "$d_x'$ and $d_y'$ are the distortion corrections.\n";
	str << "The values of $d_x'$ and $d_y'$ are functions of the estimated distortion coefficients and the normalized point coordintaes $x_n$ and $y_n$:\n";
	str << "\\begin{equation}" << "\n";
	str << "x_n = X'/Z'\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "y_n = Y'/Z'\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "d_x' = x_n(k_1'r^2 + k_2'r^4 + k_3'r^6) + 2p_1'x_n y_n + p_2'(r^2 + 2x_n^2)\n";
	str << "\\end{equation}" << "\n";

	str << "\\begin{equation}" << "\n";
	str << "d_y' = y_n(k_1'r^2 + k_2'r^4 + k_3'r^6) + p_1'(r^2 + 2 y_n^2) + 2 p_2'x_n y_n\n";
	str << "\\end{equation}" << "\n";

	str << "where:\n";
	str << "\\begin{equation}" << "\n";
	str << "r^2 = x_n^2 + y_n^2\n";
	str << "\\end{equation}" << "\n";

	str << "$k_1'$, $k_2'$, $k_3'$ are  radial distortion parameters,\n";
	str << "$p_1'$, $p_2'$ are tangential distortion parameters.\n";
	str << "Equations (\\ref{eq:17}) and (\\ref{eq:18}) can be expressed using matrix notation:\n";
	str << "\\begin{equation}" << "\n";
	str << "\\begin{bmatrix}" << "\n";
	str << "x' \\\\" << "\n";
	str << "y' \\\\" << "\n";
	str << "1" << "\n";
	str << "\\end{bmatrix}" << "\n";
	str << "= K" << "\n";
	str << "\\begin{bmatrix}" << "\n";
	str << "x_n \\\\" << "\n";
	str << "y_n \\\\" << "\n";
	str << "1" << "\n";
	str << "\\end{bmatrix}" << "\n";
	str << "+" << "\n";
	str << "\\begin{bmatrix}" << "\n";
	str << "d_x' \\\\" << "\n";
	str << "d_y' \\\\" << "\n";
	str << "1" << "\n";
	str << "\\end{bmatrix}" << "\n";
	str << "\\end{equation}" << "\n";
	str << "where:" << "\n";
	str << "\\begin{equation}" << "\n";
	str << "K =" << "\n";
	str << "\\begin{bmatrix}" << "\n";
	str << "ck & 0 & x_0 \\\\" << "\n";
	str << "0 & ck & y_0 \\\\" << "\n";
	str << "0 & 0 & 1 \\\\" << "\n";
	str << "\\end{bmatrix}" << "\n";
	str << "\\end{equation}" << "\n";
	str << "is known as camera matrix. Note that $x_0$ and $y_0$ provide the position of principal point in a software-specific image frame." << "\n";
	str << "This certificate provides $x_0$ and $y_0$ for two frames defined in the figure below." << "\n";

	str << "\\begin{figure}[H]" << "\n";
	str << "\\centering"<< "\n";
	str << "\\includegraphics[scale=0.8]{../../../../frame_definitions.pdf}" << "\n";
	str << "\\caption{\\label{fig:01}}Definitions of image frames" << "\n";
	str << "\\end{figure}" << "\n";


	str << "\\clearpage" << "\n";

	str << "\\large" << "\n";
	str << "\\begin{center}" << "\n";
	str << "\\textbf{CAMERA PARAMETERS}" << "\n";
	str << "\\end{center}" << "\n";

	//BEGINING THE TABLE - METRIC SPACE
	//***************************************************************************************************************************
	str << "\\begin{table}[H]" << "\n";
	str << "\\centering" << "\n";
	str << "\\renewcommand{\\arraystretch}{1.5}" << "\n";
	str << "\\begin{tabular}{ | l | l | l | l | l | }" << "\n";
	str << "\\hline" << "\n";
	str << "\\textbf{parameter}                & \\textbf{dim. symbol}			& \\textbf{status} & \\textbf{value} & \\textbf{std dev.} \\\\ \\hline" << "\n";
	
	str << "number of rows           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.H << " &";							//value
	str << 0 <<"\\\\ \\hline" << "\n";					//std.dev.
	
	str << "number of columns           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.W << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "pixel size           &$[mm]$	&"; //parameter
	str << "known  &";									//status
	str << camera.PixelSize << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "principal distance   &$[mm]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str <<std::fixed <<std::setprecision(5) << dim*camera.InternalOrientation[0] << " &";							//value
	str << std::fixed << std::setprecision(5) << dim*camera.InternalOrientationStdDev[0] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point x0   &$[mm]$	&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(5) << dim * camera.InternalOrientation[1] << " &";							//value
	str << std::fixed << std::setprecision(5) << dim * camera.InternalOrientationStdDev[1] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point y0   &$[mm]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(5) << dim * camera.InternalOrientation[2] << " &";							//value
	str << std::fixed << std::setprecision(5) << dim * camera.InternalOrientationStdDev[2] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k1   & $[mm^{-2}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[0]/(dim*dim) << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[0] / (dim*dim) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k2   & $[mm^{-4}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[1] / (std::pow(dim,4)) << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[1] / (std::pow(dim,4)) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k3   & $[mm^{-6}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k3)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[2] / (std::pow(dim, 6)) << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[2] / (std::pow(dim, 6)) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p1   & $[mm^{-1}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortion[0] / dim << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortionStdDev[0] / dim << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p2   & $[mm^{-1}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortion[1] / dim << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortionStdDev[1] / dim << "\\\\ \\hline" << "\n";	//std.dev.

	str << "\\end{tabular}" << "\n";
	str << "\\caption{\\label{tab:table1}Camera calibration parameters in metric space - image-centered frame.}" << "\n";
	str << "\\end{table}" << "\n";

	//TABLE END - METRIC SPACE
	//***************************************************************************************************************************

	// BEGINING THE TABLE - PIXEL SPACE
	//***************************************************************************************************************************
	str << "\\begin{table}[H]" << "\n";
	str << "\\centering" << "\n";
	str << "\\renewcommand{\\arraystretch}{1.5}" << "\n";
	str << "\\begin{tabular}{ | l | l | l | l | l | }" << "\n";
	str << "\\hline" << "\n";
	str << "\\textbf{parameter}                & \\textbf{dim. symbol}			& \\textbf{status} & \\textbf{value} & \\textbf{std dev.} \\\\ \\hline" << "\n";

	str << "number of rows           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.H << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "number of columns           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.W << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "pixel size           &$[mm]$	&"; //parameter
	str << "known  &";									//status
	str << camera.PixelSize << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "principal distance   &$[px]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[0] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[0] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point x0   &$[px]$	&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[1] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[1] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point y0   &$[px]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[2] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[2] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k1   & $[px^{-2}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[0] << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[0]<< "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k2   & $[px^{-4}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[1] << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[1]  << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k3   & $[px^{-6}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k3)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.RadialDistortion[2] << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.RadialDistortionStdDev[2] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p1   & $[px^{-1}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortion[0]  << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortionStdDev[0] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p2   & $[px^{-1}]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortion[1]  << " &";							//value
	str << std::scientific << std::setprecision(5) << camera.TangentialDistortionStdDev[1] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "\\end{tabular}" << "\n";
	str << "\\caption{\\label{tab:table1}Camera calibration parameters in pixel space - image-centered frame.}" << "\n";
	str << "\\end{table}" << "\n";

	//TABLE END - PIXEL SPACE
	//***************************************************************************************************************************

	// BEGINING THE TABLE - NORMALIZED SPACE
	//***************************************************************************************************************************
	double ck = camera.InternalOrientation[0];
	str << "\\begin{table}[H]" << "\n";
	str << "\\centering" << "\n";
	str << "\\renewcommand{\\arraystretch}{1.5}" << "\n";
	str << "\\begin{tabular}{ | l | l | l | l | l | }" << "\n";
	str << "\\hline" << "\n";
	str << "\\textbf{parameter}                & \\textbf{dim. symbol}			& \\textbf{status} & \\textbf{value} & \\textbf{std dev.} \\\\ \\hline" << "\n";

	str << "number of rows           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.H << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "number of columns           &{ [}-{]}		&"; //parameter
	str << "known  &";									//status
	str << camera.W << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "pixel size           &$[mm]$	&"; //parameter
	str << "known  &";									//status
	str << camera.PixelSize << " &";							//value
	str << 0 << "\\\\ \\hline" << "\n";					//std.dev.

	str << "principal distance   &$[px]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[0] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[0] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point x0   &$[px]$	&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[1] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[1]<< "\\\\ \\hline" << "\n";	//std.dev.

	str << "principal point y0   &$[px]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_io)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(3) << camera.InternalOrientation[2] << " &";							//value
	str << std::fixed << std::setprecision(3) << camera.InternalOrientationStdDev[2] << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k1   & $[-]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(7) << camera.RadialDistortion[0] * pow(ck, 2) << " &";							//value
	str << std::fixed << std::setprecision(7) << camera.RadialDistortionStdDev[0] * pow(ck, 2) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k2   & $[-]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(7) << camera.RadialDistortion[1]* pow(ck, 4) << " &";							//value
	str << std::fixed << std::setprecision(7) << camera.RadialDistortionStdDev[1]* pow(ck, 4) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "radial distortion k3   & $[-]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_k3)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(7) << camera.RadialDistortion[2]* pow(ck, 6) << " &";							//value
	str << std::fixed << std::setprecision(7) << camera.RadialDistortionStdDev[2]* pow(ck, 6) << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p1   & $[-]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(7) << camera.TangentialDistortion[0]*ck << " &";							//value
	str << std::fixed << std::setprecision(7) << camera.TangentialDistortionStdDev[0]*ck << "\\\\ \\hline" << "\n";	//std.dev.

	str << "tangential distortion p2   & $[-]$		&"; //parameter
	if (settings.CamFixMasks.at(camera.Name) & ba_fix_masks::mask_fix_p)
	{
		str << "fixed  &";									//status
	}
	else
	{
		str << "estimated  &";
	}
	str << std::fixed << std::setprecision(7) << camera.TangentialDistortion[1]*ck << " &";							//value
	str << std::fixed << std::setprecision(7) << camera.TangentialDistortionStdDev[1]*ck << "\\\\ \\hline" << "\n";	//std.dev.

	str << "\\end{tabular}" << "\n";
	str << "\\caption{\\label{tab:table1}Camera calibration parameters in normalized space - image-centered frame.}" << "\n";
	str << "\\end{table}" << "\n";

	//TABLE END - NORMALIZED SPACE
	//***************************************************************************************************************************


	str << "Reversed distortion coefficients (OpenCV standard, Agisoft Metashape):" << "\\newline\n";
	str << "Remark: always the full set of coefficients is provided." << "\\newline\n";
	str << "$k_1'$ : " << fixed << setprecision(9) << ba.ReverseDistortions.at(camera.Name).DistortionYDown.at(0)<< "\\newline\n";
	str << "$k_2'$ : " << fixed << setprecision(9) << ba.ReverseDistortions.at(camera.Name).DistortionYDown.at(1)<< "\\newline\n";
	str << "$k_3'$ : " << fixed << setprecision(9) << ba.ReverseDistortions.at(camera.Name).DistortionYDown.at(2)<< "\\newline\n";
	str << "$p_1'$ : " << fixed << setprecision(9) << ba.ReverseDistortions.at(camera.Name).DistortionYDown.at(3)<< "\\newline\n";
	str << "$p_2'$ : " << fixed << setprecision(9) << ba.ReverseDistortions.at(camera.Name).DistortionYDown.at(4)<< "\\newline\n";
	str << "\\vskip 0.08in" << "\n";
	str << "Camera matrix (OpenCV, parameters in [px]):\n";


	str << "\\begin{fleqn}" << "\n";
	str << "\\begin{equation*}" << "\n";
	str << "K=" << "\n";
	str << "\\begin{bmatrix}" << "\n";
	str << fixed << setprecision(3);
	str << camera.InternalOrientation[0] << "&" << 0.0 << "&" <<camera.W / 2.0 + camera.InternalOrientation[1] - 0.5 << "\\\\" << "\n";
	str << 0.0 << "&" << camera.InternalOrientation[0] << "&" <<camera.H / 2.0 - camera.InternalOrientation[2] - 0.5 << "\\\\" << "\n";
	str << 0.0 << "&" << 0.0 << "&" << 1.0 << "\\\\" << "\n";
	str << "\\end{bmatrix}" << "\n";
	str << "\\end{equation*}" << "\n";
	str << "\\end{fleqn}" << "\n";

	str << "\\vskip 0.5in" << "\n";
	str << "Calibrated by:" << "\n";
	str << "\\vskip 0.8in" << "\n";

	str << "\\noindent\\begin{tabular}{ll}" << "\n";
	str << "\\makebox[2.5in]{ \\hrulefill } &\\makebox[2.5in]{ \\hrulefill }\\\\" << "\n";
	str << Worker << "& Date\\\\" << "\n";
	str << "\\end{tabular} " << "\n";

	str << "\\end{document}" << "\n";

	str.close();
}

bool CalibrationCertificate::check_information(std::wstring & ln)
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

std::wstring CalibrationCertificate::get_information(std::wstring & ln)
{
	auto pos1 = ln.find('\'');
	auto pos2 = ln.rfind('\'');
	return ln.substr(pos1 + 1, pos2 - pos1 - 1);
}

string CalibrationCertificate::num2str(int num)
{
	stringstream ss;
	string str;
	ss << num;
	ss >> str;
	return str;
}

std::wstring CalibrationCertificate::to_wstring(std::string str)
{
	return strconverter.from_bytes(str);
}
