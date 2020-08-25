#include <datamanager/file_datawriter.h>
#include <boost/filesystem.hpp>
#include <fstream>

using namespace hirop::data_manager;

FileDataWriter::~FileDataWriter(){

}

FileDataWriter::FileDataWriter(){
    std::string homePath = getenv("HOME");
    _basePath = homePath + "/" +  ".hirop/data/";
}

HData* FileDataWriter::loadData(DataUri &uri) {

    std::string raw = loadRawData(uri);

    if(raw == "")
        return NULL;

    HData * data = HData::fromBuffer(raw);

    return data;
}

std::string FileDataWriter::loadRawData(DataUri &uri){

    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();
    std::string fileName = _basePath + "/" + uriStr + dataName;

    std::ifstream ifile(fileName.c_str());

    if(ifile.fail()){
        return "";
    }

    std::stringstream dataStr;

    dataStr << ifile.rdbuf();

    return dataStr.str();
}

int FileDataWriter::saveData(HData *data, DataUri &uri){
    std::string raw = data->toBuffer();
    return saveRawData(raw, uri);
}

int FileDataWriter::saveRawData(std::string raw, DataUri &uri){

    std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();

    std::string path = _basePath + "/" + uriStr;

    /**
     *  如果路径不存在，则创建相关目录
     */
    if( !boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);

    std::ofstream file((path + dataName).c_str());

    file << raw;

    file.close();

    return 0;
}

int FileDataWriter::deleteData(DataUri &uri){

    std::string uriStr = uri.toStr();
    std::string name = uri.getName();
    std::string path = _basePath + "/" + uriStr + name;

    if( !boost::filesystem::exists(path))
        return -1;

    return boost::filesystem::remove(path);
}

int FileDataWriter::getAllDatas(DataUri &uri, std::vector<HData *> &datas){

    std::vector<DataUri> uris;

    listUri(uri, uris);

    for(int i = 0; i < uris.size(); i++){
        HData *data = loadData(uris[i]);
        if(data != NULL)
            datas.push_back(data);
    }

    return 0;
}

int FileDataWriter::listUri(DataUri &uri, std::vector<DataUri> &uris){

    std::string uriStr = uri.toStr();
    std::string pathStr = _basePath + "/" + uriStr;

    boost::filesystem::directory_iterator end;

    if(!boost::filesystem::exists(pathStr))
        return -1;
    /**
     *  变量目录下的所有文件
     */
    for(boost::filesystem::directory_iterator pos(pathStr); pos != end; ++pos){

        if(boost::filesystem::is_directory(*pos)){
            continue;
        }

        /**
         * 通过构建uri的方式来获取data 该方法并不完善
         */
        DataUri tmpUri(pos->path().filename().string());
        tmpUri.setUriFromStr(uriStr);

        uris.push_back(tmpUri);
    }

    return 0;
}

int FileDataWriter::addJointDataMulti(std::vector<double> joint)
{
    if(cnt == 0)
    {
        recordInit();
    }
    addData(yamlNode, joint, cnt);
    cnt++;
    return 0;
}

int FileDataWriter::addData(YAML::Node& node, std::vector<double> joint,  int num)
{
    for(int i=0; i < DOFNum; ++i)
    {
        node["num" + std::to_string(num)]["joint" + std::to_string(i)] = joint[i];
        delay_ms(1);
    }
    return 0;
}

int FileDataWriter::saveDataMultiEnd(DataUri& uri)
{
    cnt = 0;
    std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();

    std::string path = _basePath + "/" + uriStr;

    /**
     *  如果路径不存在，则创建相关目录
     */
    if( !boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);

    std::ofstream file((path + dataName).c_str());
    if(!file)
    {
        std::cout << "open file FAILED" << std::endl;
        return -1;
    }
    file << yamlNode;
    yamlNode.reset();
    file.close();
    return 0;
}



int FileDataWriter::setConfing(std::string RobotName, int DOF)
{
    robotName = RobotName;
    DOFNum = DOF;
    return 0;
}

std::vector<std::vector<double> > FileDataWriter::loadJointDataMulti(DataUri& uri)
{
    std::vector<std::vector<double> > joints;
    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();
    std::string fileName = _basePath + "/" + uriStr + dataName;

    YAML::Node node;
    node = YAML::LoadFile(fileName);
    std::size_t Count = node.size() - 3;
    joints.resize(Count);
    for(int i=0; i < Count; ++i)
    {
        for(int j=0; j < DOFNum; ++j)
        {
            joints[i].push_back(node["num" + std::to_string(i)]["joint" + std::to_string(j)].as<double>());
        }
    }
    return joints;
}

int FileDataWriter::addPoseDataMulti(geometry_msgs::PoseStamped& pose)
{
    if(cnt == 0)
    {
        recordInit();
    }
    addData(yamlNode, pose, cnt);
    cnt++;
    return 0;
}

std::vector<geometry_msgs::PoseStamped> FileDataWriter::loadPoseDataMulti(DataUri& uri)
{
    std::vector<geometry_msgs::PoseStamped>  pose;
    const std::string uriStr = uri.toStr();
    std::string dataName = uri.getName();
    std::string fileName = _basePath + "/" + uriStr + dataName;

    YAML::Node node;
    node = YAML::LoadFile(fileName);
    std::size_t Count = node.size() - 3;
    pose.resize(Count);
    for(int i=0; i < Count; ++i)
    {
        getData(pose[i], node, i);
    }
    return pose;
}

void FileDataWriter::delay_ms(int ms)
{
    clock_t now = clock();
    while(clock() - now < ms);
}

void FileDataWriter::recordInit()
{
    time_t now_time;
    std::time(&now_time);
    std::string time = std::ctime(&now_time);
    yamlNode["robot"] = robotName;
    delay_ms(1);
    yamlNode["DOF"] = DOFNum;
    delay_ms(1);
    yamlNode["time"] = time;
    delay_ms(3);
}

int FileDataWriter::addData(YAML::Node& config, geometry_msgs::PoseStamped& pose, int num)
{
    config["num" + std::to_string(num)]["header"]["frame_id"] = pose.header.frame_id;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["position"]["x"] = pose.pose.position.x;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["position"]["y"] = pose.pose.position.y;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["position"]["z"] = pose.pose.position.z;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["orientation"]["x"] = pose.pose.orientation.x;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["orientation"]["y"] = pose.pose.orientation.y;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["orientation"]["z"] = pose.pose.orientation.z;
    delay_ms(3);
    config["num" + std::to_string(num)]["pose"]["orientation"]["w"] = pose.pose.orientation.w;
    delay_ms(3);
    return 0;
}

int FileDataWriter::getData(geometry_msgs::PoseStamped& pose, YAML::Node& config, int num)
{
    pose.header.frame_id = config["num" + std::to_string(num)]["header"]["frame_id"].as<std::string>();
    pose.pose.position.x = config["num" + std::to_string(num)]["pose"]["position"]["x"].as<double>();
    pose.pose.position.y = config["num" + std::to_string(num)]["pose"]["position"]["y"].as<double>();
    pose.pose.position.z = config["num" + std::to_string(num)]["pose"]["position"]["z"].as<double>();
    pose.pose.orientation.x = config["num" + std::to_string(num)]["pose"]["orientation"]["x"].as<double>();
    pose.pose.orientation.y = config["num" + std::to_string(num)]["pose"]["orientation"]["y"].as<double>();
    pose.pose.orientation.z = config["num" + std::to_string(num)]["pose"]["orientation"]["z"].as<double>();
    pose.pose.orientation.w = config["num" + std::to_string(num)]["pose"]["orientation"]["w"].as<double>();
}






