#include "pch.h"
#include <Eigen/Dense>
#include "ReverseDistortionEstimator.h"
#include "photogrammetry.h"


ReverseDistortionEstimator::ReverseDistortionEstimator(const Camera& cam)
{
	//solves for the inverse distortion (like opencv)
	//see: https://docs.opencv.org/3.4.6/d9/d0c/group__calib3d.html
	
	int x_spacing = cam.W / 10;
	int y_spacing = cam.H / 10;
	vector<std::array<double, 2> > grid_points;
	vector<std::array<double, 2> > grid_points_undistorted;
	vector<std::array<double, 2> > dist_corrections;
	grid_points.reserve(200);
	
	//calculating the estimation grid:
	for (int x = 0; x <= cam.W; x += x_spacing)
	{
		for (int y = 0; y <= cam.H; y += y_spacing)
		{
			grid_points.push_back({
				static_cast<double>(x),
				static_cast<double>(y) });
		}
	}
	grid_points.shrink_to_fit();

	//transformation from column/row coordinates to image-centered coordinates
	std::transform(grid_points.begin(), grid_points.end(), grid_points.begin(),
		[&](std::array<double,2>& p ){
		return std::array<double, 2>{
			p[0] = p[0] - cam.W / 2.0 + 0.5,
			p[1] = cam.H / 2.0 - p[1] - 0.5}; });


	//transformation from image-centered coordinates to fiducial coordiantes
	std::transform(grid_points.begin(), grid_points.end(), grid_points.begin(),
		[&](std::array<double, 2>& p) {
		return std::array<double, 2>{
			p[0] = p[0] - cam.InternalOrientation[1],
			p[1] = p[1] - cam.InternalOrientation[2]}; });

	
	grid_points_undistorted.resize(grid_points.size());
	dist_corrections.resize(grid_points.size());

	
	std::transform(grid_points.begin(), grid_points.end(), dist_corrections.begin(),
		[&](std::array<double, 2>& p) {
		return std::array<double, 2>{
			fT_xDistCorrection(cam.RadialDistortion, cam.TangentialDistortion, p[0], p[1]),
			fT_yDistCorrection(cam.RadialDistortion, cam.TangentialDistortion, p[0], p[1])}; });
	

	for (size_t i = 0, S = grid_points.size(); i < S; i++)
	{
		grid_points_undistorted.at(i)[0] = grid_points.at(i)[0] - dist_corrections.at(i)[0];
		grid_points_undistorted.at(i)[1] = grid_points.at(i)[1] - dist_corrections.at(i)[1];
	}

	//solving the reversed distortion using Eieng linear algerbra tools:
	//each coordinate generates one equation:
	Eigen::MatrixXd A(2 * grid_points_undistorted.size(), 5);
	Eigen::VectorXd L(2 * grid_points_undistorted.size());
	Eigen::VectorXd X(5);
	Eigen::VectorXd V(2 * grid_points_undistorted.size());

	for (size_t i = 0, S = grid_points_undistorted.size(); i < S; i++)
	{
		//we are estimating in the normalized space so w have to use normalized values
		double x = grid_points_undistorted.at(i)[0] /cam.InternalOrientation[0];
		double y = -grid_points_undistorted.at(i)[1] /cam.InternalOrientation[0];
		double dx = dist_corrections.at(i)[0] / cam.InternalOrientation[0];
		double dy = -dist_corrections.at(i)[1] / cam.InternalOrientation[0];
		double rr = x * x + y * y;

		A(2 * i, 0) = x * rr;
		A(2 * i, 1) = x * rr*rr;
		A(2 * i, 2) = x * rr*rr*rr;
		A(2 * i, 3) = 2 * x * y;
		A(2 * i, 4) = rr + 2 * x*x;
		L(2 * i) = dx;

		A(2 * i + 1, 0) = y * rr;
		A(2 * i + 1, 1) = y * rr*rr;
		A(2 * i + 1, 2) = y * rr*rr*rr;
		A(2 * i + 1, 3) = rr + 2 * y * y;
		A(2 * i + 1, 4) = 2 * x * y;
		L(2 * i + 1) = dy;
	}

	X = A.colPivHouseholderQr().solve(L);
	V = L - A * X;
	RMSE = std::sqrt((1.0 /(L.rows()-1)) * V.transpose()*V);

	for (int i = 0; i < 5; i++) DistortionYDown.at(i) = X(i);

}


ReverseDistortionEstimator::~ReverseDistortionEstimator()
{
}
