#pragma once
#include <string>
#include "Eigen/Eigen"
#include "tf/transform_listener.h"
using namespace std;

class transformUltity
{
public:
    static int transformFrame(const string & baseFrame, const string & referFrame,
                              Eigen::Vector3d &t, Eigen::Matrix3d &r);
private:
    static void rosTf2Eigen(const tf::StampedTransform &transform, Eigen::Vector3d &t, Eigen::Matrix3d &r);
};
