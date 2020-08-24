#pragma once

//#define DLL_EXPORT __attribute__((visibility("default")))
//#define DLL_EXPORT _declspec(dllexport)

#include <iostream>
using namespace std;
#include "PclFilterEntry.h"
#include "pcl/point_cloud.h"
#include "boost/shared_ptr.hpp"
#include "yaml-cpp/yaml.h"

template<typename TPoint>
class PclFilterEntry{
public:
//    PclFilterEntry();
    virtual string getName()=0;
    virtual int filterPCL(boost::shared_ptr< pcl::PointCloud<TPoint> >  & input){
        return 0;
    }

    virtual void updateParm(YAML::Node & node){}
    virtual void printInfo(){}
};
