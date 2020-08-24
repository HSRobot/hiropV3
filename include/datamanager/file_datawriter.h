#pragma once

#include "idatawirter.h"


namespace hirop{

namespace data_manager{

class FileDataWriter : public IDataWriter{

public:

    /**
     * @brief FileDataWriter
     * @param basePath          数据将会在该文件夹下按规律存储
     */
    FileDataWriter(std::string basePath) : _basePath(basePath){}

    /**
     * @brief FileDataWriter    构造函数
     */
    FileDataWriter();

    /**
     * @brief FileDataWriter    析构函数
     */
    ~FileDataWriter();

    /**
     * @brief loadData  加载数据
     * @param uri       数据的URI
     * @return          成功则返回被加载的数据 失败则返回NULL
     */
    HData *loadData(DataUri &uri);

    /**
     * @brief loadRawData   加载对象的序列化数据
     * @param uri           对象的URI
     * @return              对象的序列化数据
     */
    std::string loadRawData(DataUri &uri);

    /**
     * @brief saveData  保存数据
     * @param data      需要被保存的数据
     * @param uri       数据的URI
     * @return          0 成功 -1 失败
     */
    int saveData(HData *data, DataUri &uri);

    /**
     * @brief saveRawData   保存对象的序列化数据
     * @param raw           需要保存的序列化数据
     * @param uri           数据的URI
     * @return              0 成功 -1 失败
     */
    int saveRawData(std::string raw, DataUri &uri);

    /**
     * @brief deleteData    删除指定数据
     * @param uri           数据的uri
     * @return              0 成功 -1 失败
     */
    int deleteData(DataUri &uri);

    /**
     * @brief getAllDatas   获取指定uri路径下的所有data
     * @param uri[in]
     * @param datas[out]
     * @return      0 成功 -1 失败
     */
    int getAllDatas(DataUri &uri, std::vector<HData *> &datas);


    int listUri(DataUri &uri, std::vector<DataUri> &uris);

    int setConfing(std::string RobotName, int DOF);
    /**
    * @brief addDataMulti 保存多个点位, 每次调用保存一个
    * @param joint 关节弧度

    * @return 0 成功 -1 失败
    */
    int addJointDataMulti(std::vector<double> joint);


    /**
     * @brief loadDataMulti 加载多个点位的结束标志函数
     * @param uri 数据的URI
     * @return 0 成功 -1 失败
    */
    std::vector<std::vector<double> > loadJointDataMulti(DataUri& uri);


    int addPoseDataMulti(geometry_msgs::PoseStamped& pose);
    std::vector<geometry_msgs::PoseStamped> loadPoseDataMulti(DataUri& uri);

    /**
     * @brief saveDataMultiEnd 保存多个点位的结束标志函数
     * @param uri 数据的URI
     * @return 0 成功 -1 失败
    */
    int saveDataMultiEnd(DataUri& uri);



private:
    void recordInit();
    void delay_ms(int ms);
    int addData(YAML::Node& node, std::vector<double> joint, int num);
    int addData(YAML::Node& node, geometry_msgs::PoseStamped& pose, int num);
    int getData(geometry_msgs::PoseStamped& pose, YAML::Node& config, int num);
    /**
     * @brief _basePath     搜索的根路径
     */
    std::string _basePath;

    int cnt = 0;
    std::string robotName = "co605";
    int DOFNum = 6;
    int time;
    YAML::Node yamlNode;
};

}

}
