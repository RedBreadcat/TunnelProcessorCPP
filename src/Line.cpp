#include "Line.h"

Line::Line(double xIntercept, double xSlope, double yIntercept, double ySlope, double zIntercept, double zSlope)
{
	xLine = Eigen::Vector2d(xIntercept, xSlope);
	yLine = Eigen::Vector2d(yIntercept, ySlope);
	zLine = Eigen::Vector2d(zIntercept, zSlope);
}

//https://math.stackexchange.com/questions/1815397/distance-between-point-and-parametric-line
double Line::CalculateDistance(Eigen::Vector3d point3D)
{
	//line minus point
	double Xm = xLine[0] - point3D[0];
	double Ym = yLine[0] - point3D[1];
	double Zm = zLine[0] - point3D[2];

	//RHS:
	double tCoefficient = xLine[1] * xLine[1] + yLine[1] * yLine[1] + zLine[1] * zLine[1];
	double rhs = -xLine[1] * Xm - yLine[1] * Ym - zLine[1] * Zm;

	auto A = Eigen::Matrix<double, 1, 1>(tCoefficient);
	auto b = Eigen::Matrix<double, 1, 1>(rhs);

	auto t = A.colPivHouseholderQr().solve(b);	//colPivHouseholderQr is the solving algorithm. https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html

	double qx = xLine[0] + xLine[1] * t[0];
	double qy = yLine[0] + yLine[1] * t[0];
	double qz = zLine[0] + zLine[1] * t[0];

	return sqrt(pow(point3D[0] - qx, 2) + pow(point3D[1] - qy, 2) + pow(point3D[2] - qz, 2));
}
