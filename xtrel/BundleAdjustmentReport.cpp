#include "pch.h"
#include "photogrammetry.h"
#include "BundleAdjustmentReport.h"
#include "RayIntersectionCheck.h"

#include <tuple>

using namespace ba;

BundleAdjustmentReport::BundleAdjustmentReport(BundleAdjustmentData& badata, BundleAdjustment& ba, const std::string& filename)
{
	constexpr double rho = 57.295779513082320876798;
	constexpr double rhog = 63.661977236758;
	
	std::ofstream str;
	str.open(filename);
	str << "Budle Adjustment Report\n\n";

	str << "\n********************************************************************************************************************\n";
	str << "******************************************************* SETTINGS ****************************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "Applied Settings:\n";
	switch (badata.Settings.MathModel)
	{
	case BAMathModel::RIGID:
		str << "Optimization model: 'RIGID' - all controll points were treated as fixed - no correction to controll points was allowed." << std::endl;
		break;
	case BAMathModel::SOFT:
		str << "Optimization model: 'SOFT' - correction to controll points was allowed, apriori errors from input file were applied for each point." << std::endl;
		break;
	case BAMathModel::TIGHT:
		str << "Optimization model: 'TIGHT' - instead of using controll points directly, geodetic measurements of angles and distances were used." << std::endl;
		break;
	default:
		break;
	}

	switch (badata.Settings.LossFunction)
	{
	case BALossFunction::NONE:
		str << "Loss function: NONE\n";
		break;
	case BALossFunction::CAUCHY:
		str << "Loss function: CAUCHY\n";
		break;
	case BALossFunction::HUBER:
		str << "Loss function: HUBER\n";
		break;
	default:
		break;
	}

	str << "Image measurement error: " << badata.Settings.ImageMesAcc <<" [px]" << std::endl;
	str << "Hz angle measurement error: " << badata.Settings.GeodeticHzMesAcc << " [rad]" << std::endl;
	str << "V angle measurement error: " << badata.Settings.GeodeticVMesAcc << " [rad]" << std::endl;

	str << "\nCalibration masks: " << std::endl;
	for (auto&s : badata.Settings.CamFixMasks)
	{
		str << "\nCamera: " << s.first << " Mask: " << (int)s.second << std::endl;
		str << "which means:\n";
		str << "   internal orientation is fixed: " << std::boolalpha << (bool)(s.second & ba_fix_masks::mask_fix_io) << std::endl;
		str << "k1 and k2 dist. params are fixed: " << std::boolalpha << (bool)(s.second & ba_fix_masks::mask_fix_k) << std::endl;
		str << "k3 distortion parameter is fixed: " << std::boolalpha << (bool)(s.second & ba_fix_masks::mask_fix_k3) << std::endl;
		str << "  tangential distortion is fixed: " << std::boolalpha << (bool)(s.second & ba_fix_masks::mask_fix_p) << std::endl;
	}

	str << "\n*********************************************************************************************************************\n";
	str << "*********************************************** INPUT DATA STATISTICS ***********************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "Number of images          : " <<std::setw(9) << badata.NumOfImages << std::endl;
	str << "Number of camreas         : " <<std::setw(9) << badata.NumOfCameras << std::endl;
	str << "Number of controll points : " <<std::setw(9) << badata.NumOfControllPoints << std::endl;
	str << "Number of tie points      : " <<std::setw(9) << badata.NumOfTiePoints << std::endl;
	str << "Number of check points    : " <<std::setw(9) << badata.NumOfCheckPoints << std::endl;
	str << "Number of image points    : " <<std::setw(9) << badata.NumOfImagePoints << std::endl;

	str << "\nCameras assignments and number of points:\n";
	str << setw(12) << "image_name:" << " " << setw(40) << "camera_name:" << " " << setw(16) << "#image_points" << std::endl;
	for (auto &i : badata.ImageOrientationData.DataImages)
	{
		str << setw(12) << i.second.Name << " " <<setw(40) << i.second.CameraName << " "<<setw(16) <<i.second.NumOfPoints << std::endl;
	}


	str << "\n*********************************************************************************************************************\n";
	str << "***************************************************** SOLVER REPORT *************************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "\nSolver report:\n";
	str << ba.FullReport;

	str << "\n*********************************************************************************************************************\n";
	str << "***************************************************** ESTIMATES *****************************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "\nEstimated variance of unit weight: " <<fixed <<setprecision(4) << ba.Sigma02 << "[-]"<< std::endl;
	str << "Square root of  variance of unit weight (sigma0): " << fixed << setprecision(4) << std::sqrt(ba.Sigma02) << "[-]" << std::endl;

	str << "\nCamera parameters (internal orientation and distortion) after estimation:\n";

	str << "\nExplanation:\n";
	str << "\nCamera parameters provide the relationship between points in the 3D camera frame (object points) and the pixel coordinates in the image\n";
	str << "In the idealized projection, where no distortion is present, the projection equation are as follows:\n";
	str << "x' = x0 - c*(X'/Z')\n";
	str << "y' = y0 - c*(Y'/Z')\n";
	str << "where:\n";
	str << "x' and y' are the measured image plane coordinates (the unit of which can be [px]) of the object point P(X',Y',Z') projected using the camera\n";
	str << "with principal distance c and principal point (x0, y0). X', Y', Z' provide the position of point P in the 3D camera frame with the following definition:\n";
	str << "For the horizontal, terrestrial image:\n";
	str << "+X axis: towards left\n";
	str << "+Y axis: upwards\n";
	str << "+Z axis: backwards\n";
	str << "In the presence of distortion the polynomial correction coefficients [k1, k2, k3, p1, p2] are estimated. This coefficients provide the\n";
	str << "correction to the measured image coordinates, that should be distinguished from the case where the distortion coefficients provide the\n";
	str << "displacement of the ideal coordinates to their measured locations.\n";
	str << "The following observation equation is used for the estimation:\n";
	str << "x' = x0 - c*(X'/Z') + x(k1*r^2 + k2*r^4 + k3*r^6) + p1*(r^2 + 2*x^2) + 2*p2*x*y\n";
	str << "y' = y0 - c*(Y'/Z') + y(k1*r^2 + k2*r^4 + k3*r^6) + 2*p1*x*y + p2*(r^2 + 2*y^2)\n";
	str << "where: x = x'-x0, y = y'-y0\n";
	str << "and: r^2  = x^2 + y^2\n";
	str << "This results in the following formulation of the correction to the measured image coordinates:\n";
	str << "dx = - [x(k1*r^2 + k2*r^4 + k3*r^6) + p1*(r^2 + 2*x^2) + 2*p2*x*y]\n";
	str << "dy = - [y(k1*r^2 + k2*r^4 + k3*r^6) + 2*p1*x*y + p2*(r^2 + 2*y^2)]\n";
	str << "As a result:\n";
	str << "x' + dx = x0 - c*(X'/Z')\n";
	str << "y' + dy = y0 - c*(Y'/Z')\n";

	str << "\nA number of photogrammetric and computer vision software handles the distortion in a different way:\n";
	str << "once the object point is projected to the image to the P':[xp, yp] location (the 'ideal location'), the distortion parameters are used to\n";
	str << "shifts the P' point to the place where it occurs in real  the image\n";
	str << "This report provides the second set of distortion coefficients for each calibrated camera, that corresponds to the above\n";
	str << "mentioned case. The applied mathematical model can be found in OpenCV documentation: https://docs.opencv.org/3.4.6/d9/d0c/group__calib3d.html\n";
	str << "However this software does NOT use OpenCV to estimate for OpenCV distortion coefficients. The important feature of the OpenCV standard is the\n";
	str << "orientation of camera frame axes, which follows:\n";
	str << "For the horizontal, terrestrial image:\n";
	str << "+X axis: towards left\n";
	str << "+Y axis: downwards\n";
	str << "+Z axis: looking forward, along the lens axis\n";
	str << "An object point P (X, Y, Z) projects to the image according to the equation (notice the difference in sign):\n";
	str << "x' = x0 + c*(X'/Z' + dx')\n";
	str << "y' = y0 + c*(Y'/Z' + dy')\n";
	str << "where: \n";
	str << "xp = c*(X'/Z' + dx')\n";
	str << "yp = c*(Y'/Z' + dy')\n";
	str << "represent the fiducial position of the undistorted point location (unit: [px]), x0 and y0 are the principal point coordinates (dimension: [px]),\n";
	str << "dx' and dy' are the distortion corrections.\n";
	str << "The values of dx' and dy' function of the estimated distortion coefficients and the normalized point coordintaes xn and yn:\n";
	str << "xn = X'/Z'\n";
	str << "yn = Y'/Z'\n";
	str << "dx' = xn*(k1'*r^2 + k2'*r^4 + k3'*r^6) + 2*p1'*xn*yn + p2'*(r^2 + 2*xn^2)\n";
	str << "dy' = yn*(k1'*r^2 + k2'*r^4 + k3'*r^6) + p1'*(r^2 + 2*yn^2) + 2*p2'*xn*yn\n";
	str << "where r^2 = xn^2 + yn^2\n";
	str << "k1', k2', k3' are  radial distortion parameters,\n";
	str << "p1', p2' are tangential distortion parameters\n";
	str << "we use the \"'\" upper index to distinguish from the distortion model described beforehand\n";
	str << "As mentioned before this software does not use OpenCV to estimate for OpenCV distortion coefficients.\n";
	str << "Here the surface fitting approach is applied: we can define a grid of points and calculate the distortion corrections dx', dy' for\n";
	str << "each grid node. Having N grid nodes we can form 2*N linear equations for dx' and dy' and estimate distortion parameters\n";
	str << "using the least squares method.\n";

	for (auto& c : badata.ImageOrientationData.DataCameras)
	{
		str << "\ncamera name: " << c.second.Name << std::endl;
		str << "parameters for pixel space: " << std::endl;
		str << setw(39) << "parameter name: " << setw(15) << "value:"<<" "<< setw(15) << "std.dev." <<" " <<setw(15)<<"std.dev./val[%]"  << std::endl;
		str << setw(39) << "number of rows: [px]: " << setw(15) << c.second.H << std::endl;
		str << setw(39) << "number of columns: [px]: " << setw(15) << c.second.W << std::endl;
		str << setw(39) << "pixel size (phisical): [mm]: " << setw(15) <<std::fixed <<setprecision(10) << c.second.PixelSize << std::endl;
		str << setw(39) << "principal distance (ck)[px]: "
			<< fixed << setprecision(4) << setw(15) << c.second.InternalOrientation[0] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(4) << setw(15) << c.second.InternalOrientationStdDev[0] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[0] / c.second.InternalOrientation[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (x0)[px]: "
			<< fixed << setprecision(4) << setw(15) << c.second.InternalOrientation[1] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
				str << fixed << setprecision(4) << setw(15) << c.second.InternalOrientationStdDev[1] << " "
					<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[1] / c.second.InternalOrientation[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (y0)[px]: "
			<< fixed << setprecision(4) << setw(15) << c.second.InternalOrientation[2] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
				str << fixed << setprecision(4) << setw(15) << c.second.InternalOrientationStdDev[2] << " "
					<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[2] / c.second.InternalOrientation[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}

		str << setw(39) << "radial distortion (k1)[px^-2]: "
			<< scientific << setprecision(4) << setw(15) << c.second.RadialDistortion[0] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(4) << setw(15) << c.second.RadialDistortionStdDev[0] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[0] / c.second.RadialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k2)[px^-4]: "
			<< scientific << setprecision(4) << setw(15) << c.second.RadialDistortion[1] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(4) << setw(15) << c.second.RadialDistortionStdDev[1] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[1] / c.second.RadialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k3)[px^-6]: "
			<< scientific << setprecision(4) << setw(15) << c.second.RadialDistortion[2] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k3)) {
			str << scientific << setprecision(4) << setw(15) << c.second.RadialDistortionStdDev[2] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[2] / c.second.RadialDistortion[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p1)[px^-1]: "
			<< scientific << setprecision(4) << setw(15) << c.second.TangentialDistortion[0] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(4) << setw(15) << c.second.TangentialDistortionStdDev[0] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[0] / c.second.TangentialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p2)[px^-1]: "
			<< scientific << setprecision(4) << setw(15) << c.second.TangentialDistortion[1] << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(4) << setw(15) << c.second.TangentialDistortionStdDev[1] << " "
				<< fixed << setprecision(4) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[1] / c.second.TangentialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}

		
		str << "\nparameters for metric space: " << std::endl;
		double dim = c.second.PixelSize;
		str << setw(39) << "parameter name: " << setw(15) << "value:" << " " << setw(15) << "std.dev." << " " << setw(15) << "std.dev./val[%]" << std::endl;
		str << setw(39) << "number of rows: [px]: " << setw(15) << c.second.H << std::endl;
		str << setw(39) << "number of columns: [px]: " << setw(15) << c.second.W << std::endl;
		str << setw(39) << "pixel size (phisical): [mm]: " << setw(15) << std::fixed << setprecision(10) << c.second.PixelSize << std::endl;
		str << setw(39) << "principal distance (ck)[mm]: "
			<< fixed << setprecision(5) << setw(15) << c.second.InternalOrientation[0] * dim << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(5) << setw(15) << c.second.InternalOrientationStdDev[0] * dim << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[0] / c.second.InternalOrientation[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (x0)[mm]: "
			<< fixed << setprecision(5) << setw(15) << c.second.InternalOrientation[1] * dim << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(5) << setw(15) << c.second.InternalOrientationStdDev[1] * dim << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[1] / c.second.InternalOrientation[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (y0)[mm]: "
			<< fixed << setprecision(5) << setw(15) << c.second.InternalOrientation[2] * dim << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(5) << setw(15) << c.second.InternalOrientationStdDev[2] * dim << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[2] / c.second.InternalOrientation[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k1)[mm^-2]: "
			<< scientific << setprecision(5) << setw(15) << c.second.RadialDistortion[0] / (dim*dim) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(5) << setw(15) << c.second.RadialDistortionStdDev[0] / (dim*dim) << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[0] / c.second.RadialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k2)[mm^-4]: "
				<< scientific << setprecision(5) << setw(15) << c.second.RadialDistortion[1] / (std::pow(dim, 4.0)) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(5) << setw(15) << c.second.RadialDistortionStdDev[1] / (std::pow(dim, 4.0)) << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[1] / c.second.RadialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k3)[mm^-6]: "
			<< scientific << setprecision(5) << setw(15) << c.second.RadialDistortion[2] / (std::pow(dim, 6.0)) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k3)) {
			str << scientific << setprecision(5) << setw(15) << c.second.RadialDistortionStdDev[2] / (std::pow(dim, 6.0)) << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[2] / c.second.RadialDistortion[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p1)[mm^-1]: "
			<< scientific << setprecision(5) << setw(15) << c.second.TangentialDistortion[0] / dim << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(5) << setw(15) << c.second.TangentialDistortionStdDev[0] / dim << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[0] / c.second.TangentialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p2)[mm^-1]: "
			<< scientific << setprecision(5) << setw(15) << c.second.TangentialDistortion[1] / dim << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(5) << setw(15) << c.second.TangentialDistortionStdDev[1] / dim << " "
				<< fixed << setprecision(5) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[1] / c.second.TangentialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}

		str << "\nparameters for normalized space: " << std::endl;
		double ck = c.second.InternalOrientation[0];
		str << setw(39) << "parameter name: " << setw(15) << "value:" << " " << setw(15) << "std.dev." << " " << setw(15) << "std.dev./val[%]" << std::endl;
		str << setw(39) << "number of rows: [px]: " << setw(15) << c.second.H << std::endl;
		str << setw(39) << "number of columns: [px]: " << setw(15) << c.second.W << std::endl;
		str << setw(39) << "pixel size (phisical): [mm]: " << setw(15) << std::fixed << setprecision(10) << c.second.PixelSize << std::endl;
		str << setw(39) << "principal distance (ck)[-]: "
			<< fixed << setprecision(6) << setw(15) << c.second.InternalOrientation[0] / ck << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(6) << setw(15) << c.second.InternalOrientationStdDev[0] / ck << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[0] / c.second.InternalOrientation[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (x0)[-]: "
			<< fixed << setprecision(6) << setw(15) << c.second.InternalOrientation[1] / ck << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(6) << setw(15) << c.second.InternalOrientationStdDev[1] / ck << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[1] / c.second.InternalOrientation[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "principal point (y0)[-]: "
			<< fixed << setprecision(6) << setw(15) << c.second.InternalOrientation[2] / ck << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_io)) {
			str << fixed << setprecision(6) << setw(15) << c.second.InternalOrientationStdDev[2] / ck << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.InternalOrientationStdDev[2] / c.second.InternalOrientation[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k1)[-]: "
			<< scientific << setprecision(6) << setw(15) << c.second.RadialDistortion[0] * (ck*ck) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(6) << setw(15) << c.second.RadialDistortionStdDev[0] * (ck*ck) << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[0] / c.second.RadialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k2)[-]: "
			<< scientific << setprecision(6) << setw(15) << c.second.RadialDistortion[1] * (std::pow(ck, 4.0)) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k)) {
			str << scientific << setprecision(6) << setw(15) << c.second.RadialDistortionStdDev[1] * (std::pow(ck, 4.0)) << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[1] / c.second.RadialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "radial distortion (k3)[-]: "
			<< scientific << setprecision(6) << setw(15) << c.second.RadialDistortion[2] * (std::pow(ck, 6.0)) << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_k3)) {
			str << scientific << setprecision(6) << setw(15) << c.second.RadialDistortionStdDev[2] * (std::pow(ck, 6.0)) << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.RadialDistortionStdDev[2] / c.second.RadialDistortion[2])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p1)[-]: "
			<< scientific << setprecision(6) << setw(15) << c.second.TangentialDistortion[0] * ck << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(6) << setw(15) << c.second.TangentialDistortionStdDev[0] * ck << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[0] / c.second.TangentialDistortion[0])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}
		str << setw(39) << "tangential distortion (p2)[-]: "
			<< scientific << setprecision(6) << setw(15) << c.second.TangentialDistortion[1] * ck << " ";
		if (!(badata.Settings.CamFixMasks.at(c.first) & ba_fix_masks::mask_fix_p)) {
			str << scientific << setprecision(6) << setw(15) << c.second.TangentialDistortionStdDev[1] * ck << " "
				<< fixed << setprecision(6) << setw(15) << std::fabs(100.0*(c.second.TangentialDistortionStdDev[1] / c.second.TangentialDistortion[1])) << std::endl;
		}
		else
		{
			str << "this parameter was fixed\n";
		}

		std::vector < std::pair< std::pair<int, int>, double> > radial_dist_corrections;
		std::vector < std::pair< std::pair<int, int>, double> > tangential_dist_corrections;

		str << "\nMagnitudes of radial distortion corrections:\n";

		int step_x = c.second.W / 6;
		int step_y = c.second.H / 6;

		str << setw(15) << " ";
		str << setw(15) << "y values[px]:" << " \n";
 		str << setw(15) <<"x values[px]->" <<" ";
		str << setw(10) << " ";
		for (int i = -c.second.W/2; i <= c.second.W/2; i += step_x)
		{
			str << setw(10) << i << " ";
		}
		str << std::endl;


		for (int j = c.second.H / 2; j > -c.second.H/2; j -= step_y)
		{
			str << setw(15) << " ";
			str << setw(10) << j <<" ";
			for (int i = - c.second.W / 2; i <= c.second.W/2; i += step_x)
			{
				double dr = std::sqrt(std::pow(c.second.getRadialCorrectionX(i, j), 2) + std::pow(c.second.getRadialCorrectionY(i, j), 2));
				str << fixed <<setprecision(1) << setw(10) << dr <<" ";

			}
			str << std::endl;
		}

		str << "\nMagnitudes of tangential distortion corrections:\n";

		str << setw(15) << " ";
		str << setw(15) << "y values[px]:" << " \n";
		str << setw(15) << "x values[px]->" << " ";
		str << setw(10) << " ";
		for (int i = -c.second.W / 2; i <= c.second.W / 2; i += step_x)
		{
			str << setw(10) << i << " ";
		}
		str << std::endl;

		for (int j = c.second.H / 2; j > -c.second.H / 2; j -= step_y)
		{
			str << setw(15) << " ";
			str << setw(10) << j << " ";
			for (int i = -c.second.W / 2; i <= c.second.W / 2; i += step_x)
			{
				double dt = std::sqrt(std::pow(c.second.getTangentialCorrectionX(i, j), 2) + std::pow(c.second.getTangentialCorrectionY(i, j), 2));
				str << fixed << setprecision(1) << setw(10) << dt << " ";

			}
			str << std::endl;
		}

		str << "\nDistortion coefficients for the reveresed distortion model (OpenCV):\n";
		str << "k1' : " << fixed << setprecision(9)<<setw(18) <<ba.ReverseDistortions.at(c.first).DistortionYDown.at(0)<<" [-]" << std::endl;
		str << "k2' : " << fixed << setprecision(9)<<setw(18) <<ba.ReverseDistortions.at(c.first).DistortionYDown.at(1)<<" [-]" << std::endl;
		str << "k3' : " << fixed << setprecision(9)<<setw(18) <<ba.ReverseDistortions.at(c.first).DistortionYDown.at(2)<<" [-]" << std::endl;
		str << "p1' : " << fixed << setprecision(9)<<setw(18) <<ba.ReverseDistortions.at(c.first).DistortionYDown.at(3)<<" [-]" << std::endl;
		str << "p2' : " << fixed << setprecision(9)<<setw(18) <<ba.ReverseDistortions.at(c.first).DistortionYDown.at(4)<<" [-]" << std::endl;
		str << "surface fitting quality: " <<scientific <<setprecision(5) << c.second.InternalOrientation[2] * ba.ReverseDistortions.at(c.first).RMSE <<" [px]" << std::endl;

		str << "\nCamera matrix (OpenCV convention):\n";
		str << " " << " " << "| " << fixed<< setprecision(3) << setw(12) << c.second.InternalOrientation[0] <<" "<<setw(12)<< 0.0 <<" "<<setw(12) <<c.second.W/2 + c.second.InternalOrientation[1] - 0.5 <<" |" <<std::endl;
		str << "K" << "=" << "| " << fixed<< setprecision(3)<< setw(12) << 0.0 << " " << setw(12) << c.second.InternalOrientation[0] << " " <<setw(12) << c.second.H / 2 - c.second.InternalOrientation[2] - 0.5 << " |" << std::endl;
		str << " " << " " << "| " << fixed << setprecision(3) << setw(12) <<0.0 << " " << setw(12) << 0.0 << " " << setw(12) << 1.0 << " |" << std::endl;
	
		str << "\nPearson correlation coefficients for camera parameters:" << std::endl;
		str << "[nan(ind)] or [Inf] means: one of parameters was fixed."<<std::endl;
		double c_ck_x0 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::X0));
		double c_ck_y0 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::Y0));
		double c_ck_k1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::K1));
		double c_ck_k2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::K2));
		double c_ck_k3 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::K3));
		double c_ck_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::P1));
		double c_ck_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::CK, ba::BAAddParam::P2));

		double c_x0_y0 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::Y0));
		double c_x0_k1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::K1));
		double c_x0_k2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::K2));
		double c_x0_k3 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::K3));
		double c_x0_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::P1));
		double c_x0_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::X0, ba::BAAddParam::P2));

		double c_y0_k1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::Y0, ba::BAAddParam::K1));
		double c_y0_k2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::Y0, ba::BAAddParam::K2));
		double c_y0_k3 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::Y0, ba::BAAddParam::K3));
		double c_y0_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::Y0, ba::BAAddParam::P1));
		double c_y0_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::Y0, ba::BAAddParam::P2));

		double c_k1_k2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K1, ba::BAAddParam::K2));
		double c_k1_k3 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K1, ba::BAAddParam::K3));
		double c_k1_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K1, ba::BAAddParam::P1));
		double c_k1_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K1, ba::BAAddParam::P2));

		double c_k2_k3 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K2, ba::BAAddParam::K3));
		double c_k2_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K2, ba::BAAddParam::P1));
		double c_k2_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K2, ba::BAAddParam::P2));

		double c_k3_p1 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K3, ba::BAAddParam::P1));
		double c_k3_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::K3, ba::BAAddParam::P2));

		double c_p1_p2 = ba.CorelationsCamparams.at(c.first).at(std::make_pair(ba::BAAddParam::P1, ba::BAAddParam::P2));

		str << std::fixed << std::setprecision(3);
		str << std::setw(15) << "."<<" "<<std::setw(10) << "CK" << " " << std::setw(10) << "X0" << " " << std::setw(10) << "Y0" << " " << std::setw(10) << "K1" << " " << std::setw(10) << "K2" <<" "<<std::setw(10) <<"K3"<< "\n";
		str << std::setw(15) << "CK" << " " << std::setw(10) << 1.0     << " " << std::setw(10) << c_ck_x0 << " " << std::setw(10) << c_ck_y0 << " " << std::setw(10) << c_ck_k1 << " " << std::setw(10) << c_ck_k2<<" " << std::setw(10) <<c_ck_k3 <<"\n";
		str << std::setw(15) << "X0" << " " << std::setw(10) << c_ck_x0 << " " << std::setw(10) << 1.00000 << " " << std::setw(10) << c_x0_y0 << " " << std::setw(10) << c_x0_k1 << " " << std::setw(10) << c_x0_k2<<" " << std::setw(10) <<c_x0_k3 <<"\n";
		str << std::setw(15) << "Y0" << " " << std::setw(10) << c_ck_y0 << " " << std::setw(10) << c_x0_y0 << " " << std::setw(10) << 1.00000 << " " << std::setw(10) << c_y0_k1 << " " << std::setw(10) << c_y0_k2<<" " << std::setw(10) <<c_y0_k3 <<"\n";
		str << std::setw(15) << "K1" << " " << std::setw(10) << c_ck_k1 << " " << std::setw(10) << c_x0_k1 << " " << std::setw(10) << c_y0_k1 << " " << std::setw(10) << 1.00000 << " " << std::setw(10) << c_k1_k2<<" " << std::setw(10) <<c_k1_k3 <<"\n";
		str << std::setw(15) << "K2" << " " << std::setw(10) << c_ck_k2 << " " << std::setw(10) << c_x0_k2 << " " << std::setw(10) << c_y0_k2 << " " << std::setw(10) << c_k1_k2 << " " << std::setw(10) << 1.00000<<" " << std::setw(10) <<c_k2_k3 <<"\n";
		str << std::setw(15) << "K3" << " " << std::setw(10) << c_ck_k3 << " " << std::setw(10) << c_x0_k3 << " " << std::setw(10) << c_y0_k3 << " " << std::setw(10) << c_k1_k3 << " " << std::setw(10) << c_k2_k3<<" " << std::setw(10) <<1.00000 <<"\n";
		str << std::endl;

		str << std::setw(15) << "." << " " << std::setw(10) << "P1" << " " << std::setw(10) << "P2" << "\n";
		str << std::setw(15) << "CK" << " " << std::setw(10) << c_ck_p1	<< " " <<std::setw(10)  << c_ck_p1 <<"\n";
		str << std::setw(15) << "X0" << " " << std::setw(10) << c_x0_p1 << " " << std::setw(10) << c_x0_p2 <<"\n";
		str << std::setw(15) << "Y0" << " " << std::setw(10) << c_y0_p1 << " " << std::setw(10) << c_y0_p2 <<"\n";
		str << std::setw(15) << "K1" << " " << std::setw(10) << c_k1_p1 << " " << std::setw(10) << c_k1_p2 <<"\n";
		str << std::setw(15) << "K2" << " " << std::setw(10) << c_k2_p1 << " " << std::setw(10) << c_k2_p2 <<"\n";
		str << std::setw(15) << "K3" << " " << std::setw(10) << c_k3_p1 << " " << std::setw(10) << c_k3_p2 <<"\n";
		str << std::setw(15) << "P1" << " " << std::setw(10) << 1.00000 << " " << std::setw(10) << c_p1_p2 <<"\n";
		str << std::setw(15) << "P2" << " " << std::setw(10) << c_p1_p2 << " " << std::setw(10) << 1.00000 <<std::endl;

		str << "\nProjection to image plane: 10 X 10 grid of points - to compare several variants of calibration.";
		str << "\nAll external orientation parameters are set to zero.";
		{
			double f_in_px = c.second.LensNominalFocalLength / c.second.PixelSize;
			double xmin = 0.9 * (-c.second.W / 2.0);
			double ymin = 0.9 * (-c.second.H / 2.0);
			double xstep = 0.9 * (c.second.W / 10.0);
			double ystep = 0.9 * (c.second.H / 10.0);
			double coords[3] = { 0.0,0.0,0.0 };
			double r[9] = { 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0 };
			//x coordinate:
			str << "\nx[px]:\n";
			for (unsigned int i = 0; i <= 10; i++) //changing y
			{
				for (unsigned int j = 0; j <= 10; j++) //changing x
				{
					double op[3] = { xmin + j * xstep, ymin + i * ystep, -f_in_px };
					double p[2];
					fT_projection(op, coords, r, c.second.InternalOrientation, c.second.RadialDistortion, c.second.TangentialDistortion, p);
					str << fixed << setprecision(2) << p[0];
					if (j < 10) str << " ";
					else str << "\n";

				}	
			}
			//y coordinate:
			str << "\ny[px]:\n";
			for (unsigned int i = 0; i <= 10; i++) //changing y
			{
				for (unsigned int j = 0; j <= 10; j++) //changing x
				{
					double op[3] = { xmin + j * xstep, ymin + i * ystep, -f_in_px };
					double p[2];
					fT_projection(op, coords, r, c.second.InternalOrientation, c.second.RadialDistortion, c.second.TangentialDistortion, p);
					str << fixed << setprecision(2) << p[1];
					if (j < 10) str << " ";
					else str << "\n";
				}
			}
		}
	}


	str << "\nEstimated external orientation parameters of images:\n";

	str << "Explanation:\n";
	str << "The projection of object point to the image plane was defined by the following equation (see the camera calibration section):\n";
	str << "x' + dx' = x0 - c*(X'/Z')\n";
	str << "y' + dy' = y0 - c*(Y'/Z')\n";
	str << "Here object point 3D coordinates: X', Y', Z' are provided in the camera 3D reference frame\n";
	str << "External orientation parameters give the relation between camera reference frame and external reference frame\n";
	str << "The relationship between object point coordinates in the camera frame and object point coordinates in the external frame\n";
	str << "are given by the following equations:\n";
	str << "X = r11*(X-X0) + r21*(Y-Y0) + r31*(Z-Z0)\n";
	str << "Y = r12*(X-X0) + r23*(Y-Y0) + r32*(Z-Z0)\n";
	str << "Z = r13*(X-X0) + r23*(Y-Y0) + r33*(Z-Z0)\n";
	str << "where X0, Y0, Z0 are the coordinates of the image projection center\n";
	str << "rij are the entries of the 3 X 3 rotation matrix R\n";
	str << "The rotation matrix acts as follows: having the 3 element vertical vector v, coordinates of which are expressed in the\n";
	str << "camera frame, the transformation V = R*v produces 3 element vector V that coordinates are expressed in the external reference frame\n";
	str << "Rotation matrix can be seen as a composition of three 3-element vertical vectors i, j, k: R = [i j k], where each of i, j, k vector\n";
	str << "is the unit vector parallel to the respective x, y, z axis of the camera frame that coordinates are provided in the external frame (direction cosine interpretation)\n";
	str << "With the aim of estimation, the rotation matrix is parametrized in various ways.\n";
	str << "The following parametrizations are used by this software:\n";
	str << "1. Euler angles omega - phi - kappa:\n";
	str << "    |   1       0             0     |   |  cos(phi)   0   sin(phi) |   | cos(kappa)   -sin(kappa)    0 |\n";
	str << "R = |   0   cos(omega)  -sin(omega) | * |      0      1       0    | * | sin(kappa)    cos(kappa)    0 |\n";
	str << "    |   0   sin(omega)   cos(omega) |   | -sin(phi)   0   cos(phi) |   |     0             0         1 |\n";
	str << "2. Euler angles alpha - nu - kappa:\n";
	str << "    | cos(alpha)   -sin(alpha)    0 |   |   1       0        0     |   | cos(kappa)   -sin(kappa)    0 |\n";
	str << "R = | sin(alpha)    cos(alpha)    0 | * |   0   cos(nu)   -sin(nu) | * | sin(kappa)    cos(kappa)    0 |\n";
	str << "    |     0             0         1 |   |   0   sin(nu)    cos(nu) |   |     0             0         1 |\n";
	str << "3. Unit quaternion:\n";
	str << "    | a^2+b^2-c^2-d^2   2*(b*c-a*d)      2*(b*d+a*c)   |\n";
	str << "R = | 2*(b*c+a*d)    a^2-b^2+c^2-d^2     2*(c*d-a*b)   |\n";
	str << "    | 2*(b*d+a*c)       2*(c*d-a*b)    a^2-b^2-c^2+d^2 |\n";
	str << "where q = a + i*b + j*c + k*d\n";
	str << "where i*b, j*c, k*d are the imaginary components" << std::endl;
	str << "\nCoordinates:\n";
	
	int lp = 1;
	str << std::setw(5) << "lp" << " "
		<< std::setw(9) << "id" << " "
		<< std::fixed << std::setprecision(5) << std::setw(10) << "X0" << " "
		<< std::fixed << std::setprecision(5) << std::setw(10) << "Y0" << " "
		<< std::fixed << std::setprecision(5) << std::setw(10) << "Z0" << " "
		<< std::fixed << std::setprecision(5) << std::setw(13) << "std.dev.X0" << " "
		<< std::fixed << std::setprecision(5) << std::setw(13) << "std.dev.Y0" << " "
		<< std::fixed << std::setprecision(5) << std::setw(13) << "std.dev.Z0" << std::endl;
	for (auto& i : badata.ImageOrientationData.DataImages)
	{
		str << std::setw(5) << lp <<" "
			<< std::setw(9) << i.second.Name << " "
			<< std::fixed << std::setprecision(5) << std::setw(10) << i.second.EOApproximated.Coords[0] << " "
			<< std::fixed << std::setprecision(5) << std::setw(10) << i.second.EOApproximated.Coords[1] << " "
			<< std::fixed << std::setprecision(5) << std::setw(10) << i.second.EOApproximated.Coords[2] << " "
			<< std::fixed << std::setprecision(5) << std::setw(13) << i.second.EOApproximated.CoordsErrorsAposteriori[0] << " "
			<< std::fixed << std::setprecision(5) << std::setw(13) << i.second.EOApproximated.CoordsErrorsAposteriori[1] << " "
			<< std::fixed << std::setprecision(5) << std::setw(13) << i.second.EOApproximated.CoordsErrorsAposteriori[2] <<std::endl;
		lp++;
	}

	str << "\nRotation:\n";
	str << std::setw(5) << "lp" << " "
		<< std::setw(9) << "id" << " "
		<< std::setw(18) << "parametrization"
		<< std::fixed << std::setprecision(5) << std::setw(12) << "a1[deg]" << " "
		<< std::fixed << std::setprecision(5) << std::setw(12) << "a2[deg]" << " "
		<< std::fixed << std::setprecision(5) << std::setw(12) << "a3[deg]" << " "
		<< std::fixed << std::setprecision(5) << std::setw(17) << "std.dev.a1[deg]" << " "
		<< std::fixed << std::setprecision(5) << std::setw(17) << "std.dev.a2[deg]" << " "
		<< std::fixed << std::setprecision(5) << std::setw(17) << "std.dev.a3[deg]" << " "
		<< std::fixed << std::setprecision(1) << std::setw(17) << "std.dev.a1[\"]" << " "
		<< std::fixed << std::setprecision(1) << std::setw(17) << "std.dev.a2[\"]" << " "
		<< std::fixed << std::setprecision(1) << std::setw(17) << "std.dev.a3[\"]" << " "
		<< std::endl;
	lp = 1;
	for (auto& i : badata.ImageOrientationData.DataImages)
	{
		str << std::setw(5)  << lp << " "
			<< std::setw(9)  << i.second.Name << " "
			<< std::setw(18) << i.second.EOApproximated.RotParametrization
			<< std::fixed << std::setprecision(5) << std::setw(12) << rho * i.second.EOApproximated.Angles[0] << " "
			<< std::fixed << std::setprecision(5) << std::setw(12) << rho * i.second.EOApproximated.Angles[1] << " "
			<< std::fixed << std::setprecision(5) << std::setw(12) << rho * i.second.EOApproximated.Angles[2] << " "
			<< std::fixed << std::setprecision(5) << std::setw(17) << rho * i.second.EOApproximated.AnglesErrorsAposteriori[0] << " "
			<< std::fixed << std::setprecision(5) << std::setw(17) << rho * i.second.EOApproximated.AnglesErrorsAposteriori[1] << " "
			<< std::fixed << std::setprecision(5) << std::setw(17) << rho * i.second.EOApproximated.AnglesErrorsAposteriori[2] << " "
			<< std::fixed << std::setprecision(1) << std::setw(17) << 3600 * rho * i.second.EOApproximated.AnglesErrorsAposteriori[0] << " "
			<< std::fixed << std::setprecision(1) << std::setw(17) << 3600 * rho * i.second.EOApproximated.AnglesErrorsAposteriori[1] << " "
			<< std::fixed << std::setprecision(1) << std::setw(17) << 3600 * rho * i.second.EOApproximated.AnglesErrorsAposteriori[2] << " "
			<< std::endl;
		lp++;
	}

	str << "\nExternal orientation, list:\n";
	for (auto& i : badata.ImageOrientationData.DataImages)
	{
		str << i.second.Name << " ";
		str << std::fixed << std::setprecision(5)  << i.second.EOApproximated.Coords[0] << " ";
		str << std::fixed << std::setprecision(5)  << i.second.EOApproximated.Coords[1] << " ";
		str << std::fixed << std::setprecision(5)  << i.second.EOApproximated.Coords[2] << " ";
		str << std::fixed << std::setprecision(6)  << rho * i.second.EOApproximated.Angles[0] << " ";
		str << std::fixed << std::setprecision(6)  << rho * i.second.EOApproximated.Angles[1] << " ";
		str << std::fixed << std::setprecision(6)  << rho * i.second.EOApproximated.Angles[2] << "\n";
	}

	if (badata.Settings.MathModel == BAMathModel::SOFT || badata.Settings.MathModel == BAMathModel::TIGHT)
	{
		str << "\nEstimated coordinates of object points:\n";
		str << "Types: 0 -tie point, 3 - controll point, 4 - check point, 9 - geodetic controll\n";
		str << setw(8) << "Name" << " " <<setw(8) <<"Type" <<" " << setw(7) << "#Rays" << " "
			<< fixed << setprecision(5) << setw(15) << "X" << " " << setw(15) << "Y" << " " << setw(15) << "Z" << " "
			<< setw(15) << "Std.Dev.X" << " " << setw(15) << "Std.Dev.Y" << " " << setw(15) << "Std.Dev.Z" << std::endl;
		for (auto &p : badata.ObjectPoints.Data)
		{
			str << setw(8) << p.second.Name <<" " <<setw(8) <<p.second.Type << " " << setw(7) << p.second.Rays << " "
				<< fixed << setprecision(5) << setw(15) << p.second.Coords[0] << " " << setw(15) << p.second.Coords[1] << " " << setw(15) << p.second.Coords[2] << " "
				<< setw(15) << p.second.ErrorsAposteriori[0] << " " << setw(15) << p.second.ErrorsAposteriori[1] << " " << setw(15) << p.second.ErrorsAposteriori[2] << std::endl;
		}
	}

	std::vector<ImagePoint> sorted_image_points;
	std::vector<TerrainPoint> sorted_object_points;
	sorted_image_points.reserve(badata.ImagePoints.Data.size());
	sorted_object_points.reserve(badata.ObjectPointsMeasurements.Data.size());
	//local copy form map to vector
	for (const auto &p : badata.ImagePoints.Data)
	{
		sorted_image_points.push_back(p);
	}
	for (const auto &p : badata.ObjectPointsMeasurements.Data)
	{
		sorted_object_points.push_back(p.second);
	}

	std::sort(sorted_image_points.begin(), sorted_image_points.end(),
		[](ImagePoint& p1, ImagePoint& p2)
	{
		return p1.VX*p1.VX + p1.VY*p1.VY > p2.VX*p2.VX + p2.VY*p2.VY;
	}) ;

	std::sort(sorted_object_points.begin(), sorted_object_points.end(),
		[](TerrainPoint& p1, TerrainPoint& p2)
	{
		return	p1.V[0] * p1.V[0] + p1.V[1] * p1.V[1] + p1.V[2] * p1.V[2] >
				p2.V[0] * p2.V[0] + p2.V[1] * p2.V[1] + p2.V[2] * p2.V[2];
	});


	//TODO: fix computation of ray intersection
	/*
	str << "\n*********************************************************************************************************************\n";
	str << "************************************** OBJECT POINTS - INTERSECTION OF RAYS *****************************************\n";
	str << "*********************************************************************************************************************\n";


	for (const auto &op : badata.ObjectPoints.Data)
	{
		//selecting image point measurements
		std::vector<ImagePoint> imagePoints;
		for (const auto &ip : badata.ImagePoints.Data) if (op.second.Name == ip.Name) imagePoints.push_back(ip);
		if (imagePoints.size() == 0ull) continue;
		RayIntersectionCheck rayCheck(op.second, imagePoints, badata.ImageOrientationData);
	}
	*/


	str << "\n*********************************************************************************************************************\n";
	str << "****************************************************** CHECK POINTS *************************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "\nDifferences in check point coordinates (given - estimated), sorted:\n";
	str << setw(12) <<"Id" <<" "
		<< setw(12) << "DX" << " "
		<< setw(12) << "DY" << " "
		<< setw(12) << "DZ" << " "
		<< setw(12) << "DXYZ" << "\n";
	std::vector<std::pair<std::string,std::array<double, 4> > > chkp_res_data;
	chkp_res_data.reserve(badata.CheckPoints.Data.size());

	std::array<double, 4> chkp_rmse{ 0.0, 0.0, 0.0, 0.0 };

	for (const auto &p : badata.CheckPoints.Data)
	{
		double dx = p.second.Coords[0] - badata.ObjectPoints.Data.at(p.first).Coords[0];
		double dy = p.second.Coords[1] - badata.ObjectPoints.Data.at(p.first).Coords[1];
		double dz = p.second.Coords[2] - badata.ObjectPoints.Data.at(p.first).Coords[2];
		double dxyz = std::sqrt(dx * dx + dy * dy + dz * dz);
		chkp_res_data.push_back(std::make_pair(p.first,std::array<double,4>({ dx,dy,dz,dxyz }) ) );
		chkp_rmse[0] += dx * dx;
		chkp_rmse[1] += dy * dy;
		chkp_rmse[2] += dz * dz;
		chkp_rmse[3] += dxyz * dxyz;
	}
	chkp_res_data.shrink_to_fit();

	for (auto &r : chkp_rmse) r = std::sqrt(r / badata.CheckPoints.Data.size());

	std::sort(
		chkp_res_data.begin(),
		chkp_res_data.end(),
		[&](std::pair<std::string, std::array<double, 4> >& p1, std::pair<std::string, std::array<double, 4> >&p2)
		{return p1.second.at(3) > p2.second.at(3); });
	
	for (const auto &p : chkp_res_data)
	{
		str << setw(12) << p.first << " "
			<< fixed << setprecision(6) << setw(12) << p.second.at(0) << " "
			<< fixed << setprecision(6) << setw(12) << p.second.at(1) << " "
			<< fixed << setprecision(6) << setw(12) << p.second.at(2) << " "
			<< fixed << setprecision(6) << setw(12) << p.second.at(3) << "\n";
	}
	
	str <<fixed <<setprecision(6) << "\nRMSEX: " << chkp_rmse.at(0) << " RMSEY: " << chkp_rmse.at(1) << " RMSEZ: " << chkp_rmse.at(2) << " RMSEXYZ: " << chkp_rmse.at(3) << std::endl;

	



	str << "\n*********************************************************************************************************************\n";
	str << "******************************************************** RMSE *******************************************************\n";
	str << "*********************************************************************************************************************\n";

	//calcualting per-image RMSE errors
	std::map<std::string, double> xresiduals_per_image;
	std::map<std::string, double> yresiduals_per_image;
	std::map<std::string, double> xyresiduals_per_image;

	std::map<std::string, double> xresiduals_per_camera;
	std::map<std::string, double> yresiduals_per_camera;
	std::map<std::string, double> xyresiduals_per_camera;

	std::map<std::string, int> points_per_camera;


	for (auto&i : badata.ImageOrientationData.DataImages)
	{
		xresiduals_per_image.emplace(i.second.Name, 0.0);
		yresiduals_per_image.emplace(i.second.Name, 0.0);
		xyresiduals_per_image.emplace(i.second.Name, 0.0);
	}

	for (auto&i : badata.ImageOrientationData.DataCameras)
	{
		xresiduals_per_camera.emplace(i.first, 0.0);
		yresiduals_per_camera.emplace(i.first, 0.0);
		xyresiduals_per_camera.emplace(i.first, 0.0);
		points_per_camera.emplace(i.first, 0);
	}

	for (auto&p : badata.ImagePoints.Data)
	{
		xresiduals_per_image.at(p.ImageName) += (p.VX * p.VX);
		yresiduals_per_image.at(p.ImageName) += (p.VY * p.VY);
		xyresiduals_per_image.at(p.ImageName) += (p.VX * p.VX + p.VY * p.VY);

		xresiduals_per_camera.at(badata.ImageOrientationData.DataImages.at(p.ImageName).CameraName) += (p.VX * p.VX);
		yresiduals_per_camera.at(badata.ImageOrientationData.DataImages.at(p.ImageName).CameraName) += (p.VY * p.VY);
		xyresiduals_per_camera.at(badata.ImageOrientationData.DataImages.at(p.ImageName).CameraName) += (p.VX * p.VX + p.VY * p.VY);
		points_per_camera.at(badata.ImageOrientationData.DataImages.at(p.ImageName).CameraName)++;
	}

	for (auto&r : xresiduals_per_image) r.second = std::sqrt(r.second / (badata.ImageOrientationData.DataImages.at(r.first).NumOfPoints - 1));
	for (auto&r : yresiduals_per_image) r.second = std::sqrt(r.second / (badata.ImageOrientationData.DataImages.at(r.first).NumOfPoints - 1));
	for (auto&r : xyresiduals_per_image) r.second = std::sqrt(r.second / (2*badata.ImageOrientationData.DataImages.at(r.first).NumOfPoints - 1));

	for (auto&r : xresiduals_per_camera) r.second = std::sqrt(r.second / (points_per_camera.at(r.first) - 1.0));
	for (auto&r : yresiduals_per_camera) r.second = std::sqrt(r.second / (points_per_camera.at(r.first) - 1.0));
	for (auto&r : xyresiduals_per_camera) r.second = std::sqrt(r.second / (2*points_per_camera.at(r.first) - 1.0));
	//printing per-image RMSE

	str << "\nPer-image RMSE:\n";
	str << setw(12) << "image_id" << setw(16) << "#image_points" << " "
		<< fixed << setprecision(3) << setw(15) << "rmse_x[px]" << " " << setw(15) << "rmse_y[px]" << " " << setw(15) << "rmse_xy[px]" << " "<< setw(36) <<"camera" << std::endl;
	for (auto&r : xresiduals_per_image)
	{
		str <<setw(12) << r.first <<setw(16) << badata.ImageOrientationData.DataImages.at(r.first).NumOfPoints <<  " "
			<<fixed <<setprecision(3) << setw(15) << r.second << " " <<setw(15) << yresiduals_per_image.at(r.first) <<" "<<setw(15) << xyresiduals_per_image.at(r.first) <<" "
			<<setw(36) << badata.ImageOrientationData.DataImages.at(r.first).CameraName << std::endl;
	}

	str << "\nPer-camera RMSE:\n";
	str << setw(35) << "camera" << setw(16) << "#image_points" << " "
		<< fixed << setprecision(3) << setw(15) << "rmse_x[px]" << " " << setw(15) << "rmse_y[px]" << " " << setw(15) << "rmse_xy[px]" << std::endl;
	for (auto&r : xresiduals_per_camera)
	{
		str << setw(35) << r.first << setw(16) << points_per_camera.at(r.first) << " "
			<< fixed << setprecision(3) << setw(15) << r.second << " " << setw(15) << yresiduals_per_camera.at(r.first) << " " << setw(15) << xyresiduals_per_camera.at(r.first) << std::endl;
	}

	//residuals for controll points
	if (badata.Settings.MathModel == BAMathModel::SOFT)
	{
		
		std::array<double, 3> rmsexyz = { 0.0, 0.0, 0.0 };
		for (auto& p : badata.ObjectPointsMeasurements.Data)
		{
			for (int i : {0, 1, 2}) rmsexyz.at(i) += p.second.V[i] * p.second.V[i];
		}
		
		for (auto &r : rmsexyz) r = std::sqrt(r / (badata.ObjectPointsMeasurements.Data.size() - 1));
		str << "\nRMSE of control points: (X, Y and Z):\n";
		str <<fixed<<setprecision(6) << rmsexyz.at(0) << " " << rmsexyz.at(1) << " " << rmsexyz.at(2) << std::endl;

	}

	if (badata.Settings.MathModel == BAMathModel::TIGHT)
	{
		std::array<double, 2> rmsehzv = { 0.0, 0.0 };
		for (auto&p : badata.GeodeticAngularMeasurements.Data)
		{
			rmsehzv[0] += p.Residuals[0] * p.Residuals[0];
			rmsehzv[1] += p.Residuals[1] * p.Residuals[1];
		}
		rmsehzv[0] = std::sqrt(rmsehzv[0] / (badata.GeodeticAngularMeasurements.Data.size() - 1));
		rmsehzv[1] = std::sqrt(rmsehzv[1] / (badata.GeodeticAngularMeasurements.Data.size() - 1));
		str << "\nRMSE of geodetic measurements of angles:\n";
		str << std::fixed << std::setprecision(1) << std::setw(12) << "RMSE Hz[\"]" << " " << std::setw(12) << "RMSE Hz[cc]" << " " <<
			std::setw(12) << "RMSE V[\"]" << " " << std::setw(12) << "RMSE V[cc]" << std::endl;
		str << std::fixed <<std::setprecision(1) << std::setw(12) << 3600 * rho*rmsehzv[0] <<" " << std::setw(12) << 10000 * rhog*rmsehzv[0] << " " <<
			std::setw(12) << 3600 * rho*rmsehzv[1]<< " " << std::setw(12) << 10000 * rhog*rmsehzv[1] << std::endl;
	}

	//generating and sorting projection center resisuals, caluclating projeciton center RMSE
	std::vector<std::pair<std::string, std::array<double, 5>>> projectionCenterResiduals; //[image_id [VX VY VZ VXY VXYZ]]
	std::array<double, 5> projectionCenterRMSE{ 0.0, 0.0, 0.0, 0.0, 0.0 };
	int numberOfObservedProjectionCenterCoordinates{ 0 };
	for (const auto &imageData : badata.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedPosition) continue;
		numberOfObservedProjectionCenterCoordinates++;
		std::array<double, 5> residuals{ imageData.second.CoordsResiduals[0], imageData.second.CoordsResiduals[1], imageData.second.CoordsResiduals[2],
		std::sqrt(imageData.second.CoordsResiduals[0] * imageData.second.CoordsResiduals[0] + imageData.second.CoordsResiduals[1] * imageData.second.CoordsResiduals[1]),
		std::sqrt(imageData.second.CoordsResiduals[0] * imageData.second.CoordsResiduals[0] + imageData.second.CoordsResiduals[1] * imageData.second.CoordsResiduals[1] + imageData.second.CoordsResiduals[2] * imageData.second.CoordsResiduals[2]) };
		for (int i : {0, 1, 2, 3, 4}) projectionCenterRMSE[i] += residuals[i] * residuals[i];
		projectionCenterResiduals.emplace_back(std::pair<std::string, std::array<double, 5>>(imageData.first, residuals));
	}
	std::sort(projectionCenterResiduals.begin(), projectionCenterResiduals.end(), [](const std::pair<std::string, std::array<double, 5>>& r1, const std::pair<std::string, std::array<double, 5>>& r2)
	{
		return r1.second[4] > r2.second[4];
	});
	std::transform(projectionCenterRMSE.begin(), projectionCenterRMSE.end(), projectionCenterRMSE.begin(), [&badata](double& val) {
		val /= (badata.ImageOrientationData.DataImages.size() - 1);
		return sqrt(val);
	});

	//sorting orientation residuals by angular component of axis-angle representaiton
	std::vector<std::tuple<std::string, std::string, const ImageData*, double> > angleResiduals;
	std::array<double, 4> orientationRMSE{ 0.0, 0.0, 0.0, 0.0 };
	int numberOfObservedAngularOrientation{ 0 };
	for (const auto &imageData : badata.ImageOrientationData.DataImages)
	{
		if (!imageData.second.observedOrientation) continue;
		numberOfObservedAngularOrientation++;
		double R[9];
		fT_angles2rot(imageData.second.AnglesResiduals, R, imageData.second.EOObserved.RotParametrization);
		//std::acos(0.5*(R[0] + R[4] + R[8] - 1.0)) provides the angle of axis and angle representation
		double absoluteAngle{ std::abs(std::acos(0.5*(R[0] + R[4] + R[8] - 1.0))) };
		angleResiduals.emplace_back(std::make_tuple(imageData.first, imageData.second.EOObserved.RotParametrization, &imageData.second, absoluteAngle));
		for (int i : {0, 1, 2}) orientationRMSE[i] += imageData.second.AnglesResiduals[i] * imageData.second.AnglesResiduals[i];
		orientationRMSE[3] += absoluteAngle * absoluteAngle;
	}
	std::sort(angleResiduals.begin(), angleResiduals.end(), [](const std::tuple<std::string, std::string, const ImageData*, double>& t1, std::tuple<std::string, std::string, const ImageData*, double>& t2)
	{
		return std::get<3>(t1) > std::get<3>(t2);
	});
	std::transform(orientationRMSE.begin(), orientationRMSE.end(), orientationRMSE.begin(), [&badata](double& val) {
		val /= (badata.ImageOrientationData.DataImages.size() - 1);
		return sqrt(val);
	});

	str << "\nRMSE of observed external orientaiton:\n";
	str << "Number of images with observed projection center coordinates: " << numberOfObservedProjectionCenterCoordinates << "\n";
	str << "Number of images with observed angles: " << numberOfObservedAngularOrientation << "\n";
	str << "\nRMSE of projection center coordinates:\n";
	str << std::fixed << std::setprecision(5);
	str << "RMSE X  : " <<setw(13)<< projectionCenterRMSE[0] << "\n";
	str << "RMSE Y  : " <<setw(13)<< projectionCenterRMSE[1] << "\n";
	str << "RMSE Z  : " <<setw(13)<< projectionCenterRMSE[2] << "\n";
	str << "RMSE XY : " <<setw(13)<< projectionCenterRMSE[3] << "\n";
	str << "RMSE XYZ: " <<setw(13)<< projectionCenterRMSE[4] << "\n";

	str << "\nRMSE of angles:\n";
	str << std::fixed;
	str << "RMSE Angle1        : " << setprecision(5) << setw(13) << orientationRMSE[0] << " [deg]   = " << setprecision(1) << setw(10) << 3600 * rho * orientationRMSE[0] << " [\"]\n";
	str << "RMSE Angle2        : " << setprecision(5) << setw(13) << orientationRMSE[1] << " [deg]   = " << setprecision(1) << setw(10) << 3600 * rho * orientationRMSE[1] << " [\"]\n";
	str << "RMSE Angle3        : " << setprecision(5) << setw(13) << orientationRMSE[2] << " [deg]   = " << setprecision(1) << setw(10) << 3600 * rho * orientationRMSE[2] << " [\"]\n";
	str << "RMSE Absolute Angle: " << setprecision(5) << setw(13) << orientationRMSE[3] << " [deg]   = " << setprecision(1) << setw(10) << 3600 * rho * orientationRMSE[3] << " [\"]\n";

	str << "\n*********************************************************************************************************************\n";
	str << "***************************************************** RESIDUALS *****************************************************\n";
	str << "*********************************************************************************************************************\n";

	str << "\nImage points residuals: " << std::endl;
	str << std::setw(10) << "point_id" << " " << std::setw(10) << "image_id" << " " << std::setw(9) << "vx" << " " << std::setw(9) << "vy" << " " << std::setw(9) << "vxy\n";
	for (auto &p : sorted_image_points)
	{
		str <<std::setw(10) <<p.Name <<" " <<std::setw(10) << p.ImageName <<
			" " <<std::fixed << std::setprecision(3) <<std::setw(9) << p.VX <<" " << std::setw(9) << p.VY <<" " << std::setw(9) << std::sqrt(p.VX*p.VX + p.VY*p.VY) << std::endl;
	}


	//printing residuals of extarnal orientation parameters
	str << "\nResiduals of coordinates of projection centers, sorted by VXZY: " << std::endl;
	str << std::setw(10) << "image_id" << " " << std::setw(9) << "VX" << " " << std::setw(9) << "VY" << " " << std::setw(9) << "VZ" << " " << std::setw(9) << "VXY" << " " << std::setw(9) << "VXYZ\n";
	for (const auto &resData : projectionCenterResiduals)
	{
		str << std::setw(10) << resData.first << " ";
		str << std::fixed << std::setprecision(5) << std::setw(9) << resData.second[0] << " " << std::setw(9) << resData.second[1] << " " << std::setw(9) << resData.second[2] << " ";
		str << std::fixed << std::setprecision(5) << std::setw(9) << resData.second[3] << " " << std::setw(9) << resData.second[4] << "\n";
	}


	str << "\nResiduals of orientation of images.";
	str << "\nSorted by absoulute angle residual derived from rotation marix R: arccos(0.5*(trace(R) - 1.0)):" << std::endl;
	str	<< std::setw(10) << "image_id" << " "
		<< std::setw(18) << "parametrization" <<" "
		<< std::setw(12) << "Va1[deg]" << " "
		<< std::setw(12) << "Va2[deg]" << " "
		<< std::setw(12) << "Va3[deg]" << " "
		<< std::setw(20) << "Abs.Angle[deg]" << " "
		<< std::setw(17) << "Va1[\"]" << " "
		<< std::setw(17) << "Va2[\"]" << " "
		<< std::setw(17) << "Va3[\"]" << " "
		<< std::setw(20) << "Abs.Angle[\"]" << " "
		<< std::endl;
	for (const auto &res : angleResiduals)
	{
		str << std::setw(10) << std::get<0>(res) << " " << std::setw(18) << std::get<1>(res) <<" ";
		str << std::fixed << std::setprecision(5);
		str << std::setw(12) << std::get<2>(res)->AnglesResiduals[0] << " " << std::setw(12) << std::get<2>(res)->AnglesResiduals[1] << " " << std::setw(12) << std::get<2>(res)->AnglesResiduals[2] << " "<< std::setw(20) << std::get<3>(res) <<" ";
		str << std::fixed << std::setprecision(1);
		str << std::setw(17) << 3600 * rho * std::get<2>(res)->AnglesResiduals[0] << " " << std::setw(17) << 3600 * rho *std::get<2>(res)->AnglesResiduals[1] << " " << std::setw(17) << 3600 * rho * std::get<2>(res)->AnglesResiduals[2] << " "<< std::setw(20) << 3600 * rho * std::get<3>(res) << " \n";
		
	}

	if (badata.Settings.MathModel == BAMathModel::SOFT)
	{
		str << "\nObject point residuals: " << std::endl;
		str << std::setw(10) << "point_id" << " " << std::setw(10) << "VX" << " " << std::setw(10) << "VY" << " " << std::setw(10) << "VZ" << " " << std::setw(10) << "VXYZ\n";
		for (auto &p : sorted_object_points)
		{
			str << std::setw(10) << p.Name << " "
				<< std::fixed << std::setprecision(5) << std::setw(10) << p.V[0] << " " << std::setw(10) << p.V[1] << " " << std::setw(10) << p.V[2] << " "
				<< std::setw(10) << std::sqrt(p.V[0] * p.V[0] + p.V[1] * p.V[1] + p.V[2] * p.V[2]) << std::endl;
		}
	}

	if (badata.Settings.MathModel == BAMathModel::TIGHT)
	{
		std::sort(badata.GeodeticAngularMeasurements.Data.begin(), badata.GeodeticAngularMeasurements.Data.end(),
			[](AngularSurvey3D& p1, AngularSurvey3D& p2)
		{
			return p1.Residuals[0]*p1.Residuals[0] > p2.Residuals[0] * p2.Residuals[0];
		});
		
		str << "\nResiduals of geodetic measurements of Hz angles: " << std::endl;
		str << std::setw(12) << "left_point" << " " << std::setw(12) << "standpoint" << " " << std::setw(12) << "right_point" << " " <<
			std::fixed << std::setprecision(4) << std::setw(12) << "v_Hz [deg]" << " " << std::setprecision(1) << std::setw(8) << "v_Hz [\"]" << " " <<
			std::setprecision(4) << std::setw(12) << "v_Hz [g]" << " " << std::setprecision(1) << std::setw(8) << "v_Hz [cc]" << std::endl;
		for (auto &p : badata.GeodeticAngularMeasurements.Data)
		{
			str << std::setw(12) << p.Ids[0] << " "<<std::setw(12) << p.Ids[1] << " " <<std::setw(12) << p.Ids[2] << " " <<
				std::fixed<<std::setprecision(4) <<std::setw(12) << rho * p.Residuals[0] << " " <<std::setprecision(1) << std::setw(8) <<3600 * rho*p.Residuals[0] << " " <<
				std::setprecision(4) <<std::setw(12) <<rhog*p.Residuals[0] << " " <<std::setprecision(1) <<std::setw(8) <<10000 * rhog*p.Residuals[0] << std::endl;
		}

		std::sort(badata.GeodeticAngularMeasurements.Data.begin(), badata.GeodeticAngularMeasurements.Data.end(),
			[](AngularSurvey3D& p1, AngularSurvey3D& p2)
		{
			return p1.Residuals[1] * p1.Residuals[1] > p2.Residuals[1] * p2.Residuals[1];
		});

		str << "\nResiduals of geodetic measurements of V angles: " << std::endl;
		str << std::setw(12) << "left_point" << " " << std::setw(12) << "standpoint" << " " << std::setw(12) << "right_point" << " " <<
			std::fixed << std::setprecision(4) << std::setw(12) << "v_V [deg]" << " " << std::setprecision(1) << std::setw(8) << "v_V [\"]" << " " <<
			std::setprecision(4) << std::setw(12) << "v_V [g]" << " " << std::setprecision(1) << std::setw(8) << "v_V [cc]" << std::endl;
		for (auto &p : badata.GeodeticAngularMeasurements.Data)
		{
			str << std::setw(12) << p.Ids[0] << " " << std::setw(12) << p.Ids[1] << " " << std::setw(12) << p.Ids[2] << " " <<
				std::fixed << std::setprecision(4) << std::setw(12) << rho * p.Residuals[1] << " " << std::setprecision(1) << std::setw(8) << 3600 * rho*p.Residuals[1] << " " <<
				std::setprecision(4) << std::setw(12) << rhog * p.Residuals[1] << " " << std::setprecision(1) << std::setw(8) << 10000 * rhog*p.Residuals[1] << std::endl;
		}

	}
	str << std::endl;
	str.close();
}


BundleAdjustmentReport::~BundleAdjustmentReport()
{
}
