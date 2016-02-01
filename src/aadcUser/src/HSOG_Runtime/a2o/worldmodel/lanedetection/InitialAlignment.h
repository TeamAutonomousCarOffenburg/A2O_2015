#pragma once
#include <eigen3/Eigen/Dense>


class InitialAlignment
{

public:
   	Eigen::Vector2d startposition;
	Eigen::Vector2d endposition;
	bool parallelParking = false;
	bool valid = false;
};
