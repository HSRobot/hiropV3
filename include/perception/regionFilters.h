#pragma once

#include <iostream>
using namespace std;
#include "PclFilterEntry.h"
#include "pcl/filters/voxel_grid.h"

template<typename TPoint>
class regionPclFilter :public PclFilterEntry<TPoint>{
public:
    regionPclFilter(std::string name):name(name){
        maxZ  = maxX = maxY = 2000;
        minZ  = minX = minY = 2000;

    }

    virtual std::string getName(){
        return string("region");
    }

    virtual int filterPCL(boost::shared_ptr< pcl::PointCloud<TPoint> >  & input){
            typename pcl::PointCloud<TPoint>::Ptr outPointCloud(new pcl::PointCloud<TPoint>);

            int size = input->points.size();
			for(int i = 0; i < size; i++){
                if(input->points[i].z > minZ && input->points[i].z < maxZ \
                        && input->points[i].x > minX && input->points[i].x < maxX \
                        && input->points[i].y > minY && input->points[i].y < maxY){
					outPointCloud->points.push_back(input->points[i]);
				}
				    //continue;
                    
			}

			outPointCloud->width = 1;
			outPointCloud->height = outPointCloud->points.size();
            input = boost::make_shared< pcl::PointCloud<TPoint> >(*outPointCloud);
    }

    virtual void updateParm(YAML::Node & node){

        try{
            maxZ = std::atof(node["region"]["maxZ"].as<std::string>().c_str());
            maxY = std::atof(node["region"]["maxY"].as<std::string>().c_str());
            maxX = std::atof(node["region"]["maxX"].as<std::string>().c_str());
            minZ = std::atof(node["region"]["minZ"].as<std::string>().c_str());
            minY = std::atof(node["region"]["minY"].as<std::string>().c_str());
            minX = std::atof(node["region"]["minX"].as<std::string>().c_str());
        }catch (YAML::Exception &e)
        {

            std::cout <<"ERROR "<<getName() << e.msg<<std::endl;
            return ;
        }

        std::cout <<"INFO "<<getName() <<" update param OK"<<std::endl;


    }


    virtual void printInfo(){
        std::cout << "regionPclFilter: " << std::endl;
        std::cout << "\tmaxX: "<< maxX << std::endl;
        std::cout << "\tmaxY: "<< maxY << std::endl;
        std::cout << "\tmaxZ: "<< maxZ << std::endl;
        std::cout << "\tminX: "<< minX << std::endl;
        std::cout << "\tminY: "<< minY << std::endl;
        std::cout << "\tminZ: "<< minZ << std::endl;
    }

private:
    std::string name;
    double maxZ, maxX,maxY;
    double minZ,minX,minY;
};
