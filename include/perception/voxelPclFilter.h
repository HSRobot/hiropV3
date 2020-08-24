#pragma once

#include <iostream>
using namespace std;
#include "PclFilterEntry.h"
#include "pcl/filters/voxel_grid.h"

template<typename TPoint>
class voxelPclFilter :public PclFilterEntry<TPoint>{

public:
    voxelPclFilter(std::string name):name(name){
        leafX  = leafY = leafZ = 0.01f;
    }

    virtual std::string getName(){
        return string("voxel");
    }

    virtual int filterPCL(boost::shared_ptr< pcl::PointCloud<TPoint> >  & input){
            typename pcl::PointCloud<TPoint>::Ptr cloudFiltered(new pcl::PointCloud<TPoint>());
            pcl::VoxelGrid< TPoint > sor;
            sor.setInputCloud (input);
            sor.setLeafSize (leafX, leafY, leafZ);
            sor.filter (*cloudFiltered);
            input = boost::make_shared< pcl::PointCloud<TPoint> >(*cloudFiltered);
    }

    virtual void updateParm(YAML::Node & node){
        try{
            leafX = std::atof(node["voxel"]["leafX"].as<std::string>().c_str());
            leafY = std::atof(node["voxel"]["leafY"].as<std::string>().c_str());
            leafZ = std::atof(node["voxel"]["leafZ"].as<std::string>().c_str());
        }catch (YAML::Exception &e)
        {
            std::cout <<"ERROR "<<getName() << e.msg<<std::endl;
            return ;
        }
        std::cout <<"INFO "<<getName() <<" update param OK" << std::endl;
		std::cout << node["voxel"]["leafZ"].as<std::string>().c_str() << std::endl;
		std::cout << "转换后的leafZ: " << leafZ << std::endl; 

    }

    virtual void printInfo(){
        std::cout << "voxelPclFilter: " << std::endl;
        std::cout << "\tleafX: "<< leafX << std::endl;
        std::cout << "\tleafY: "<< leafY << std::endl;
        std::cout << "\tleafZ: "<< leafZ << std::endl;

    }

private:
    std::string name;
    double leafX, leafY,leafZ;
};
