#pragma once
#include "Eigen/Dense"

class Line
{
public:
	Line(double xIntercept, double xSlope, double yIntercept, double ySlope, double zIntercept, double zSlope);
	double CalculateDistance(Eigen::Vector3d point3D);

private:
	Eigen::Vector2d xLine, yLine, zLine;	//0: intercept value. 1: t gradient
};

