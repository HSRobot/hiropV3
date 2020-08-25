#pragma once

#include "hdata.h"
#include "datauri.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <ctime>
#include <geometry_msgs/PoseStamped.h>

namespace hirop{

namespace data_manager{

class IDataWriter{

public:

    virtual std::string loadRawData(DataUri &uri) = 0;

    virtual int saveRawData(std::string raw, DataUri &uri) = 0;

    /**
     * @brief setConfing 设置文件头信息
     * @param RobotName 机器人的名字
     * @param DOF 自由度
    */
    virtual int setConfing(std::string RobotName, int DOF) = 0;


    /**
    * @brief saveDataMulti 记录一个或多个点位, 每次调用保存一个
    * @param joint 关节弧度
    * @param uri 数据的URI
    * @return 0 成功 -1 失败
    */
    virtual int addJointDataMulti(std::vector<double> joint) = 0;

    /**
     * @brief 加载关节的文件的数据
     * @param uri 数据的URI
     * @return 返回这个文件内所有的关节数据
    */
    virtual std::vector<std::vector<double> > loadJointDataMulti(DataUri& uri) = 0;

    /**
     * @brief 记录一个或多个位置数据, 每次调用保存一个
     * @param 位置数据
     * @return 返回这个文件内所有的位置数据
    */
    virtual int addPoseDataMulti(geometry_msgs::PoseStamped& pose) = 0;
    virtual std::vector<geometry_msgs::PoseStamped> loadPoseDataMulti(DataUri& uri) = 0;

    /**
     * @brief saveDataMultiEnd 保存多个点位数据的结束标志函数
     * @return 0 成功 -1 失败
    */
    virtual int saveDataMultiEnd(DataUri& uri) = 0;


    /**
     * @brief loadData  加载数据
     * @param uri       数据的URI
     * @return          成功则返回被加载的数据 失败则返回NULL
     */
    virtual HData *loadData( DataUri &uri) = 0;

    /**
     * @brief saveData  保存数据
     * @param data      需要被保存的数据
     * @param uri       数据的URI
     * @return          0 成功 -1 失败
     */
    virtual int saveData(HData *data, DataUri &uri) = 0;

    /**
     * @brief deletData 删除指定数据
     * @param uri       数据的uri
     * @return          0 成功 -1 失败
     */
    virtual int deleteData(DataUri &uri) = 0;

    /**
     * @brief getAllDatas   获取指定uri下的所有数据
     * @param uri[in]           uri
     * @param datas[out]         数据
     * @return  0 成功 -1 失败
     */
    virtual int getAllDatas(DataUri &uri, std::vector<HData*> &datas) = 0;

    /**
     * @brief listUri       获取指定uri下的所有uri
     * @param uri[in]       指定的uri
     * @param uris[out]     所有的uri
     * @return              0 成功 -1 失败
     */
    virtual int listUri(DataUri &uri, std::vector<DataUri> &uris) = 0;

};

}
}
