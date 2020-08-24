#pragma once

//#define DLL_EXPORT __attribute__((visibility("default")))
//#define DLL_EXPORT _declspec(dllexport)

#include <iostream>
using namespace std;
#include "PclFilterEntry.h"
#include "pcl/point_cloud.h"
#include "boost/shared_ptr.hpp"
#include "list"
#include "vector"
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "pcl/io/pcd_io.h"

using namespace boost;
template<typename TPoint>
class PclFilterManager{
public:
    void addFilterEntry(boost::shared_ptr<PclFilterEntry<TPoint > >& entry ){
        pclList.push_back(entry);
    }

    void setInputPCLPoint(pcl::PointCloud<TPoint> & point){
        std::cout<<"input: " << point.size() << std::endl;

        this->point = boost::make_shared<pcl::PointCloud<TPoint> >(point);
    }

    void updatePxaram(YAML::Node & fileNode)
    {
        for(auto it = pclList.begin(); it != pclList.end(); it++)
        {
            boost::shared_ptr<PclFilterEntry<TPoint> > ptr = (*it);
            ptr->updateParm(fileNode);
        }
    }

    int  filterProcess(){
        if(point ==nullptr || point.get()->size() == 0 )
            return -1;

        for(auto it = pclList.begin(); it != pclList.end(); it++)
        {
            std::cout <<(*it)->getName()<<std::endl;
            int ret = (*it)->filterPCL(point);
            if(ret != 0)
                return -2;
        }
        return 0;
    }

    bool savePointCloud(const string &objName, const string & path= "./")
    {
        assert(point->size() != 0);
//        pcl::PointCloud<TPoint>* temp = point.get();
        int ret = pcl::io::savePCDFileASCII(path+objName+".pcd", *point.get());
        if(ret != 0)
            return false;
        std::cout << "save pcd ok "<<std::endl;
        return true;
    }

    bool loadPointCloud(const string & path)
    {
//        pcl::PointCloud<TPoint>* temp = new pcl::PointCloud<TPoint>();
        point->clear();
        point = nullptr;
        int ret = pcl::io::loadPCDFile(path, *point.get());
        std::cout<<" loadPointCloud input: " << point->size() << std::endl;

        if(ret != 0)
            return false;
//        point = boost::make_shared<pcl::PointCloud<TPoint>>(*temp);
        return true;
    }

    void getOutCloud(boost::shared_ptr<pcl::PointCloud<TPoint> > & point)
    {
        point = this->point;
    }

    void printInfo(){
        for(auto it = pclList.begin(); it != pclList.end(); it++)
        {
            (*it)->printInfo();
        }
    }
private:
   list< boost::shared_ptr<PclFilterEntry<TPoint> > >   pclList;
   boost::shared_ptr<pcl::PointCloud<TPoint>>  point;

};

