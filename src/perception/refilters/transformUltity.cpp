#include "transformUltity.h"


int transformUltity::transformFrame(const string &baseFrame, const string &referFrame,
                                    Eigen::Vector3d &t, Eigen::Matrix3d &r)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
        listener.waitForTransform(baseFrame, referFrame, ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform(baseFrame, referFrame, ros::Time(0), transform);
    }catch (tf::TransformException &ex) {
        /**
         * @brief 获取变换关系失败，等待1秒后继续获取坐标变换
         */
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }

    rosTf2Eigen(transform, t, r);
}

void transformUltity::rosTf2Eigen(const tf::StampedTransform &transform, Eigen::Vector3d &t, Eigen::Matrix3d &r)
{
    t(0) = transform.getOrigin().x();
    t(1) = transform.getOrigin().y();
    t(2) = transform.getOrigin().z();

    tf::Quaternion tmp;
    tmp = transform.getRotation();

//    ROS("TF::Quaternion.x = %f, y = %f, z = %f, w = %f", tmp.x(), tmp.y(), tmp.z(), tmp.w());
    Eigen::Quaterniond q( tmp.w(), tmp.x(), tmp.y(), tmp.z());

//    IDebug("Eigen::Quaternion.x = %f, y = %f, z = %f, w = %f", q.x(), q.y(), q.z(), q.w());

    r = q.toRotationMatrix();
}
