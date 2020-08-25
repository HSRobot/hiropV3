#include "classic_actuator.h"

Actuator::Actuator()
{
    _init();
    ros::NodeHandle n;
    node = n;
    pickTraSub = node.subscribe<moveit_msgs::PickupActionResult>("/pickup/result", 10, &Actuator::pickTraCB, this);
    placeTraSub = node.subscribe<moveit_msgs::PlaceActionResult>("/place/result", 10, &Actuator::placeTraCB, this);
}

Actuator::~Actuator()
{
    delete _moveGroup;
    _moveGroup = NULL;
}

int Actuator::parseConfig(YAML::Node& yamlNode)
{
    YAML::Node node;
    if(!yamlNode["parameters"]){
        std::cerr<<"无参数设置"<<std::endl;
        return -1;
    }
    node = yamlNode["parameters"];
    // if(node["geometry"]){
    //     m_parm.geometry.type = node["geometry"]["type"].as<std::string>();
    //     m_parm.geometry.length = node["geometry"]["L"].as<double>();
    //     m_parm.geometry.width  = node["geometry"]["W"].as<double>();
    //     m_parm.geometry.height = node["geometry"]["H"].as<double>();
    //     m_parm.geometry.mesh = node["geometry"]["mesh"].as<std::string>();
    //     m_parm.geometry.table_link = node["geometry"]["talbe_link"].as<std::string>();
    //     m_parm.geometry.table_px = node["geometry"]["table_pose_x"].as<double>();
    //     m_parm.geometry.table_py = node["geometry"]["table_pose_y"].as<double>();
    //     m_parm.geometry.table_pz = node["geometry"]["table_pose_z"].as<double>();
    // }
    // if(node["moveitConfig"]){
    //     m_parm.moveitConfig.planner_id = node["moveitConfig"]["plannerId"].as<std::string>();
    //     m_parm.moveitConfig.velocity = node["moveitConfig"]["velocity"].as<double>();
    //     m_parm.moveitConfig.accelerated = node["moveitConfig"]["accelerated"].as<double>();
    // }
    if(node["pickConfig"]){
        m_parm.pickConfig.direct = node["pickConfig"]["direct_pick"].as<bool>();
        m_parm.pickConfig.dist_scale = node["pickConfig"]["dist_scale"].as<double>();
        m_parm.pickConfig.m_Roll = node["pickConfig"]["Roll"].as<std::vector<double> >();
        m_parm.pickConfig.m_Pitch = node["pickConfig"]["Pitch"].as<std::vector<double> >();
        m_parm.pickConfig.m_Yaw = node["pickConfig"]["Yaw"].as<std::vector<double> >();

        m_parm.pickConfig.pre_dist_min = node["pickConfig"]["pre_dist_min"].as<float>();
        m_parm.pickConfig.pre_dist_max = node["pickConfig"]["pre_dist_max"].as<float>();
        m_parm.pickConfig.pre_vect_x =   node["pickConfig"]["pre_vect_x"].as<double>();
        m_parm.pickConfig.pre_vect_y =   node["pickConfig"]["pre_vect_y"].as<double>();
        m_parm.pickConfig.pre_vect_z =   node["pickConfig"]["pre_vect_z"].as<double>();
        m_parm.pickConfig.back_dist_min =  node["pickConfig"]["back_dist_min"].as<double>();
        m_parm.pickConfig.back_dist_max =  node["pickConfig"]["back_dist_max"].as<double>();
        m_parm.pickConfig.back_vect_x =  node["pickConfig"]["back_vect_x"].as<double>();
        m_parm.pickConfig.back_vect_y =  node["pickConfig"]["back_vect_y"].as<double>();
        m_parm.pickConfig.back_vect_z =  node["pickConfig"]["back_vect_z"].as<double>();
    }
    if(node["placeConfig"]){
        m_parm.placeConfig.direct = node["placeConfig"]["direct_place"].as<bool>();
        m_parm.placeConfig.dist_scale = node["placeConfig"]["dist_scale"].as<double>();
        m_parm.placeConfig.m_Roll = node["placeConfig"]["Roll"].as<std::vector<double> >();
        m_parm.placeConfig.m_Pitch = node["placeConfig"]["Pitch"].as<std::vector<double> >();
        m_parm.placeConfig.m_Yaw = node["placeConfig"]["Yaw"].as<std::vector<double> >();

        m_parm.placeConfig.pre_dist_min = node["placeConfig"]["pre_dist_min"].as<float>();
        m_parm.placeConfig.pre_dist_max = node["placeConfig"]["pre_dist_max"].as<float>();
        m_parm.placeConfig.pre_vect_x =   node["placeConfig"]["pre_vect_x"].as<double>();
        m_parm.placeConfig.pre_vect_y =   node["placeConfig"]["pre_vect_y"].as<double>();
        m_parm.placeConfig.pre_vect_z =   node["placeConfig"]["pre_vect_z"].as<double>();
        m_parm.placeConfig.back_dist_min =  node["placeConfig"]["back_dist_min"].as<double>();
        m_parm.placeConfig.back_dist_max =  node["placeConfig"]["back_dist_max"].as<double>();
        m_parm.placeConfig.back_vect_x =  node["placeConfig"]["back_vect_x"].as<double>();
        m_parm.placeConfig.back_vect_y =  node["placeConfig"]["back_vect_y"].as<double>();
        m_parm.placeConfig.back_vect_z =  node["placeConfig"]["back_vect_z"].as<double>();
    }
    // if(node["gripperConfig"]){
    //     m_parm.gripperConfig.allowed_coliision = node["gripperConfig"]["allowed_coliision"].as<std::string>();
    //     m_parm.gripperConfig.joint_name = node["gripperConfig"]["joint_name"].as<std::string>();
    //     m_parm.gripperConfig.open_position = node["gripperConfig"]["open_position"].as<double>();
    //     m_parm.gripperConfig.close_position = node["gripperConfig"]["close_position"].as<double>();
    // }

#ifdef jointConstraints
    if(node["jointConstraints"]){
        m_parm.jointConstraints.joint_name = node["jointConstraints"]["joint_name"].as<std::vector<std::string> >();
        m_parm.jointConstraints.position = node["jointConstraints"]["position"].as<std::vector<double> >();
        m_parm.jointConstraints.above = node["jointConstraints"]["above"].as<std::vector<double> >();
        m_parm.jointConstraints.below = node["jointConstraints"]["below"].as<std::vector<double> >();
        m_parm.jointConstraints.weight = node["jointConstraints"]["weight"].as<std::vector<double> >();
    }
#endif
    pickScale = m_parm.pickConfig.dist_scale;
    placeScale = m_parm.placeConfig.dist_scale;

#ifdef _COUT_
    std::cout
        // <<"m_parm.geometry.type:"<<m_parm.geometry.type<<std::endl<<
        // "m_parm.geometry.length:"<<m_parm.geometry.length<<std::endl<<
        // "m_parm.geometry.width:"<<m_parm.geometry.width<<std::endl<<
        // "m_parm.geometry.height:"<<m_parm.geometry.height<<std::endl<<
        // "m_parm.geometry.mesh:"<<m_parm.geometry.mesh<<std::endl<<
        // "m_parm.geometry.table_link:"<<m_parm.geometry.table_link<<std::endl<<
        // "m_parm.geometry.table_px:"<<m_parm.geometry.table_px<<std::endl<<
        // "m_parm.geometry.table_py:"<<m_parm.geometry.table_py<<std::endl<<
        // "m_parm.geometry.table_pz:"<<m_parm.geometry.table_pz<<std::endl<<

        << 
        // "m_parm.moveitConfig.planner_id:"<<m_parm.moveitConfig.planner_id<<std::endl<<
        // "m_parm.moveitConfig.velocity:"<<m_parm.moveitConfig.velocity<<std::endl<<
        // "m_parm.moveitConfig.accelerated:"<<m_parm.moveitConfig.accelerated<<std::endl<<
        "m_parm.pickConfig.direct:"<<m_parm.pickConfig.direct<<std::endl<<
        "m_parm.pickConfig.pre_dist_min:"<<m_parm.pickConfig.pre_dist_min<<std::endl<<
        "m_parm.pickConfig.pre_dist_max:"<<m_parm.pickConfig.pre_dist_max<<std::endl<<
        "m_parm.pickConfig.pre_vect_x:"<<m_parm.pickConfig.pre_vect_x<<std::endl<<
        "m_parm.pickConfig.pre_vect_y:"<<m_parm.pickConfig.pre_vect_y<<std::endl<<
        "m_parm.pickConfig.pre_vect_z:"<<m_parm.pickConfig.pre_vect_z<<std::endl<<
        "m_parm.pickConfig.back_dist_min:"<<m_parm.pickConfig.back_dist_min<<std::endl<<
        "m_parm.pickConfig.back_dist_max:"<<m_parm.pickConfig.back_dist_max<<std::endl<<
        "m_parm.pickConfig.back_vect_x:"<<m_parm.pickConfig.back_vect_x<<std::endl<<
        "m_parm.pickConfig.back_vect_y:"<<m_parm.pickConfig.back_vect_y<<std::endl<<
        "m_parm.pickConfig.back_vect_z:"<<m_parm.pickConfig.back_vect_z<<std::endl<<


        "m_parm.placeConfig.direct:"<<m_parm.placeConfig.direct<<std::endl<<
        "m_parm.placeConfig.pre_dist_min:"<<m_parm.placeConfig.pre_dist_min<<std::endl<<
        "m_parm.placeConfig.pre_dist_max:"<<m_parm.placeConfig.pre_dist_max<<std::endl<<
        "m_parm.placeConfig.pre_vect_x:"<<m_parm.placeConfig.pre_vect_x<<std::endl<<
        "m_parm.placeConfig.pre_vect_y:"<<m_parm.placeConfig.pre_vect_y<<std::endl<<
        "m_parm.placeConfig.pre_vect_z:"<<m_parm.placeConfig.pre_vect_z<<std::endl<<
        "m_parm.placeConfig.back_vect_x:"<<m_parm.placeConfig.back_vect_x<<std::endl<<
        "m_parm.placeConfig.back_vect_y:"<<m_parm.placeConfig.back_vect_y<<std::endl<<
        "m_parm.placeConfig.back_vect_z:"<<m_parm.placeConfig.back_vect_z<<std::endl<<

        // "m_parm.gripperConfig.joint_name:"<<m_parm.gripperConfig.joint_name<<std::endl<<
        // "m_parm.gripperConfig.open_position:"<<m_parm.gripperConfig.open_position<<std::endl<<
        // "m_parm.gripperConfig.close_position:"<<m_parm.gripperConfig.close_position<<std::endl<<
        // "m_parm.gripperConfig.allowed_coliision:"<<m_parm.gripperConfig.allowed_coliision<<
        std::endl;

#ifdef jointConstraints
    for(int i = 0 ; i < m_parm.jointConstraints.joint_name.size(); i++){
        std::cout<<"m_parm.jointConstraints.joint_name["<<i<<"]:"<<m_parm.jointConstraints.joint_name[i]<<std::endl;
        std::cout<<"m_parm.jointConstraints.position["<<i<<"]:"<<m_parm.jointConstraints.position[i]<<std::endl;
        std::cout<<"m_parm.jointConstraints.above["<<i<<"]:"<<m_parm.jointConstraints.above[i]<<std::endl;
        std::cout<<"m_parm.jointConstraints.below["<<i<<"]:"<<m_parm.jointConstraints.below[i]<<std::endl;
        std::cout<<"m_parm.jointConstraints.weight["<<i<<"]:"<<m_parm.jointConstraints.weight[i]<<std::endl;
    }
#endif

//        "m_parm.pickConfig.m_Roll:"<<m_parm.pickConfig.m_Roll<<
//        "m_parm.pickConfig.m_Pitch:"<<m_parm.pickConfig.m_Pitch<<
#endif
    std::cout << "--------------------" << std::endl;
    return 0;
}

void Actuator::_init()
{
    //  loadMoveit();
     _pickStopFlag = false;
     return;
}

int Actuator::loadMoveit()
{
    MOVE_GROUP = m_parm.moveitConfig.name;
    _moveGroup = new MoveGroupInterface(MOVE_GROUP);

    GRIPPER_GROUP = m_parm.gripperConfig.name;
    openGripper = m_parm.gripperConfig.openGripper;
    closeGripper = m_parm.gripperConfig.closeGripper;
    // _gripperGroup = new MoveGroupInterface(GRIPPER_GROUP);
    
    // std::vector<std::string> joint_name = _gripperGroup->getJointNames();
    // for(int i=0; i < joint_name.size(); i++)
    // {
    //     std::cout << "joint_name: " << joint_name[i] << std::endl;
    //     std::cout << "open: " << openGripper[i] << std::endl;
    //     std::cout << "close: " << closeGripper[i] << std::endl;
    // }
    _moveGroup->setNumPlanningAttempts(3);
    _moveGroup->setPlanningTime(3);
    return 0;
}

int Actuator::setMoveitConfig()
{
    // setPlannerId(m_parm.moveitConfig.planner_id);
    // setPickplaceSpeed(m_parm.moveitConfig.velocity, m_parm.moveitConfig.accelerated);
}

int Actuator::setPlannerId(std::string plannerId)
{
    if(_moveGroup == NULL)
        return -1;
    _moveGroup->setPlannerId(plannerId);
    return 0;
}

int Actuator::setPickplaceSpeed(double velocity,double accelerated)
{
    if(_moveGroup == NULL)
        return -1;
    _moveGroup->setMaxAccelerationScalingFactor(accelerated);
    _moveGroup->setMaxVelocityScalingFactor(velocity);
    return 0;
}

int Actuator::setPickPose(PoseStamped pickPos)
{
    m_pickPose.header.frame_id = pickPos.frame_id;
    m_pickPose.pose.position.x = pickPos.pose.position.x;
    m_pickPose.pose.position.y = pickPos.pose.position.y;
    m_pickPose.pose.position.z = pickPos.pose.position.z;

    m_pickPose.pose.orientation.w = pickPos.pose.orientation.w;
    m_pickPose.pose.orientation.x = pickPos.pose.orientation.x;
    m_pickPose.pose.orientation.y = pickPos.pose.orientation.y;
    m_pickPose.pose.orientation.z = pickPos.pose.orientation.z;

#ifdef _COUT_
    std::cout<<"_pickposes.px ="<<m_pickPose.pose.position.x<<std::endl
             <<",_pickposes.py ="<<m_pickPose.pose.position.y<<std::endl
             <<",_pickposes.pz ="<<m_pickPose.pose.position.z<<std::endl;
#endif

    return 0;
}

int Actuator::setPlacePose(PoseStamped placePos)
{
    m_placePose.header.frame_id = placePos.frame_id;
    m_placePose.pose.position.x = placePos.pose.position.x;
    m_placePose.pose.position.y = placePos.pose.position.y;
    m_placePose.pose.position.z = placePos.pose.position.z;

    m_placePose.pose.orientation.w = placePos.pose.orientation.w;
    m_placePose.pose.orientation.x = placePos.pose.orientation.x;
    m_placePose.pose.orientation.y = placePos.pose.orientation.y;
    m_placePose.pose.orientation.z = placePos.pose.orientation.z;

#ifdef _COUT_
    std::cout<<"_placeposes.px ="<<m_placePose.pose.position.x<<std::endl
             <<"_placeposes.py ="<<m_placePose.pose.position.y<<std::endl
             <<"_placeposes.pz ="<<m_placePose.pose.position.z<<std::endl;
#endif

    return 0;
}

int Actuator::moveToPos(PoseStamped pose)
{
    if(_pickStopFlag)
        return -1;
    int count = 0;
    MoveItErrorCode result;
    ros::Rate loop_rate(10);

    setMoveitConfig();

    geometry_msgs::PoseStamped TPose;
    TPose.header.frame_id = pose.frame_id;
    TPose.pose.position.x = pose.pose.position.x;
    TPose.pose.position.y = pose.pose.position.y;
    TPose.pose.position.z = pose.pose.position.z;

    TPose.pose.orientation.w = pose.pose.orientation.w;
    TPose.pose.orientation.x = pose.pose.orientation.x;
    TPose.pose.orientation.y = pose.pose.orientation.y;
    TPose.pose.orientation.z = pose.pose.orientation.z;

    _moveGroup->setPoseTarget(TPose);

    ROS_INFO("moveToPos ready!!!!");

    while(count < 3 && result != MoveItErrorCode::SUCCESS){
        if(_pickStopFlag)
            return -1;
        count ++;
        ROS_ERROR("count = %d", count);
        result = _moveGroup->move();
        ROS_ERROR("moveToPos result = %d", result);
        loop_rate.sleep();
    }
    if(!result)
        return -1;
    return 0;
}

int Actuator::moveToFoName(std::string posName)
{
    if(_pickStopFlag)
        return -1;
    MoveItErrorCode result;

    setMoveitConfig();

    _moveGroup->setNamedTarget(posName);
    result = _moveGroup->move();
    ROS_ERROR("move from name result = %d", result);

    return 0;
}

int Actuator::quaternionToEuler(Quaternion quat,euler &pick_euler)
{
    tf::Quaternion quatt;
    quatt.setW(quat.w);
    quatt.setX(quat.x);
    quatt.setY(quat.y);
    quatt.setZ(quat.z);

    tf::Matrix3x3(quatt).getRPY(pick_euler.roll,pick_euler.pitch,pick_euler.yaw);

    return 0;
}

int Actuator::eulerToQuaternion(euler euler_angle, Quaternion &quat)
{
    tf::Quaternion tq;
    tq = tf::createQuaternionFromRPY(euler_angle.roll,euler_angle.pitch,euler_angle.yaw);
    quat.w = tq.w();
    quat.x = tq.x();
    quat.y = tq.y();
    quat.z = tq.z();
    return 0;
}

int Actuator::getDiraction(geometry_msgs::PoseStamped pose, std::vector<double> Roll,
                                  std::vector<double> Pitch, std::vector<double> Yaw, pick_vect& vect,
                                  std::vector<Quaternion>& quats, double scale)
{
    euler pick_euler;
    Quaternion quat;

    quat.w = pose.pose.orientation.w;
    quat.x = pose.pose.orientation.x;
    quat.y = pose.pose.orientation.y;
    quat.z = pose.pose.orientation.z;

#ifdef _QUAT_VECT_

    if(quat.w != 1.0){
        double xta = acos(quat.w);
        double sw = sin(xta);

        std::cout<<"qv xta =" << xta <<std::endl;
        std::cout<<"sw =" << sw <<std::endl;

        vect.vect_x = quat.x / sw;
        vect.vect_y = quat.y / sw;
        vect.vect_z = quat.z / sw;
    }
    else{
        vect.vect_x = 1;
        vect.vect_y = 0;
        vect.vect_z = 0;
    }

    std::cout<<"qv quat.w =" <<pose.pose.orientation.w <<std::endl;
    std::cout<<"qv quat.x =" <<pose.pose.orientation.x <<std::endl;
    std::cout<<"qv quat.y =" <<pose.pose.orientation.y <<std::endl;
    std::cout<<"qv quat.z =" <<pose.pose.orientation.z <<std::endl;
    std::cout<<"qv vect.vect_x =" <<vect.vect_x <<std::endl;
    std::cout<<"qv vect.vect_y =" <<vect.vect_y <<std::endl;
    std::cout<<"qv vect.vect_z =" <<vect.vect_z <<std::endl;
    std::cout<<"cos45 =" <<cos(45) <<std::endl;
    std::cout<<"cosr45 =" <<cos(0.7854) <<std::endl;

#endif

#define _EULER_VECT_
#ifdef _EULER_VECT_
    quaternionToEuler(quat, pick_euler);

    if((fabs(pick_euler.pitch - 1.5708) <= esp) && pick_euler.yaw == 0){
        vect.vect_x = 0 * scale;
        vect.vect_y = 0 * scale;
        vect.vect_z = -(sin(pick_euler.pitch) * scale);
    }
    else{
#ifdef _VECT_CH_
        vect.vect_x = cos(pick_euler.pitch) * scale;
        vect.vect_y = sin(pick_euler.yaw) * scale;
        vect.vect_z = -(sin(pick_euler.pitch) * scale);
#endif
        vect.vect_z = -(sin(pick_euler.pitch) * scale);
        vect.vect_x = ((cos(pick_euler.pitch) * cos(pick_euler.yaw))) * scale;
        vect.vect_y = ((cos(pick_euler.pitch) * sin(pick_euler.yaw))) * scale;
    }

    makeMoreQuat(pick_euler, Roll, Pitch, Yaw, quats);
#endif

#ifdef _COUT_
    std::cout<<"pick_euler.roll =" << pick_euler.roll <<std::endl;
    std::cout<<"pick_euler.pitch =" <<pick_euler.pitch <<std::endl;
    std::cout<<"pick_euler.yaw =" <<pick_euler.yaw <<std::endl;
    std::cout<<"quat.w =" <<pose.pose.orientation.w <<std::endl;
    std::cout<<"quat.x =" <<pose.pose.orientation.x <<std::endl;
    std::cout<<"quat.y =" <<pose.pose.orientation.y <<std::endl;
    std::cout<<"quat.z =" <<pose.pose.orientation.z <<std::endl;
    std::cout<<"vect.vect_x =" <<vect.vect_x <<std::endl;
    std::cout<<"vect.vect_y =" <<vect.vect_y <<std::endl;
    std::cout<<"vect.vect_z =" <<vect.vect_z <<std::endl;
#endif

    return 0;
}

int Actuator::makeMoreQuat(euler euler_angle, std::vector<double> Roll, std::vector<double> Pitch,
                                      std::vector<double> Yaw, std::vector<Quaternion>& quats)
{
    euler remake_euler;
    Quaternion remake_quat;

    remake_euler = euler_angle;

    for(int i=0; i < Roll.size(); i++){
        for(int j=0; j < Pitch.size(); j++){
            for(int k = 0 ; k < Yaw.size(); k++){

                remake_euler.roll += Roll[i];
                remake_euler.pitch += Roll[j];
                remake_euler.yaw += Roll[k];

                eulerToQuaternion(remake_euler, remake_quat);

                quats.push_back(remake_quat);
            }
        }
    }
}

trajectory_msgs::JointTrajectory Actuator::makeGripperPosture(bool is_open)
{
    // trajectory_msgs::JointTrajectory tj;
    // tj.joint_names = _gripperGroup->getJointNames();
    // tj.points.resize(1);
    // if(is_open)
    // {
    //     tj.points[0].positions = openGripper;
    // }
    // else
    // {
    //     tj.points[0].positions = closeGripper;
    // }
    // return tj;
}

int Actuator::makeGrasp()
{
    pick_vect vect, pre_vect,back_vect;
    std::vector<Quaternion> quats;
    moveit_msgs::Grasp grasp_pose;
    _graspPoses.clear();

    getDiraction(m_pickPose, m_parm.pickConfig.m_Roll, m_parm.pickConfig.m_Pitch,
                 m_parm.pickConfig.m_Yaw, vect, quats, pickScale);

    if(m_parm.pickConfig.pre_vect_x == 0 && m_parm.pickConfig.pre_vect_y == 0 && m_parm.pickConfig.pre_vect_z == 0){
        pre_vect = vect;
    }
    else{
        pre_vect.vect_x = m_parm.pickConfig.pre_vect_x;
        pre_vect.vect_y = m_parm.pickConfig.pre_vect_y;
        pre_vect.vect_z = m_parm.pickConfig.pre_vect_z;
    }
    if(m_parm.pickConfig.back_vect_x == 0 && m_parm.pickConfig.back_vect_y == 0 && m_parm.pickConfig.back_vect_z == 0){
        back_vect.vect_x = -(vect.vect_x);
        back_vect.vect_y = -(vect.vect_y);
        back_vect.vect_z = -(vect.vect_z);
    }
    else{
        back_vect.vect_x = m_parm.pickConfig.back_vect_x;
        back_vect.vect_y = m_parm.pickConfig.back_vect_y;
        back_vect.vect_z = m_parm.pickConfig.back_vect_z;
    }

    grasp_pose.grasp_pose.header.frame_id = m_pickPose.header.frame_id;

    //抓取前位置
    grasp_pose.pre_grasp_approach.direction.header.frame_id = m_pickPose.header.frame_id;
    grasp_pose.pre_grasp_approach.direction.vector.x = pre_vect.vect_x;
    grasp_pose.pre_grasp_approach.direction.vector.y = pre_vect.vect_y;
    grasp_pose.pre_grasp_approach.direction.vector.z = pre_vect.vect_z;
    grasp_pose.pre_grasp_approach.min_distance = m_parm.pickConfig.pre_dist_min;
    grasp_pose.pre_grasp_approach.desired_distance = m_parm.pickConfig.pre_dist_max;
    //抓取后的位置
    grasp_pose.post_grasp_retreat.direction.header.frame_id = m_pickPose.header.frame_id;
    grasp_pose.post_grasp_retreat.direction.vector.x = back_vect.vect_x;
    grasp_pose.post_grasp_retreat.direction.vector.y = back_vect.vect_y;
    grasp_pose.post_grasp_retreat.direction.vector.z = back_vect.vect_z;
    grasp_pose.post_grasp_retreat.min_distance = m_parm.pickConfig.back_dist_min;
    grasp_pose.post_grasp_retreat.desired_distance = m_parm.pickConfig.back_dist_max;

    //抓取位置
    grasp_pose.grasp_pose.pose.position.x = m_pickPose.pose.position.x;
    grasp_pose.grasp_pose.pose.position.y = m_pickPose.pose.position.y;
    grasp_pose.grasp_pose.pose.position.z = m_pickPose.pose.position.z;

    grasp_pose.allowed_touch_objects.push_back(OBJECT_ID);
    grasp_pose.max_contact_force = 0;
    // grasp_pose.pre_grasp_posture = makeGripperPosture(true);
    // grasp_pose.grasp_posture = makeGripperPosture(false);

    std::stringstream ss;
    //if(quats.size() > 0){
    if(false){
        for(int i = 0; i<quats.size();i++){
            ss.clear();
            grasp_pose.grasp_pose.pose.orientation.w = quats[i].w;
            grasp_pose.grasp_pose.pose.orientation.x = quats[i].x;
            grasp_pose.grasp_pose.pose.orientation.y = quats[i].y;
            grasp_pose.grasp_pose.pose.orientation.z = quats[i].z;

            ss<<i;
            grasp_pose.id = ss.str();
            _graspPoses.push_back(grasp_pose);
        }
    }
    else{
        grasp_pose.grasp_pose.pose.orientation.w = m_pickPose.pose.orientation.w;
        grasp_pose.grasp_pose.pose.orientation.x = m_pickPose.pose.orientation.x;
        grasp_pose.grasp_pose.pose.orientation.y = m_pickPose.pose.orientation.y;
        grasp_pose.grasp_pose.pose.orientation.z = m_pickPose.pose.orientation.z;

        grasp_pose.id = "0";
        _graspPoses.push_back(grasp_pose);

#ifdef _COUT_
    std::cout<<"m_pickPose.px,"
    <<"m_pickPose.py,"
    <<"m_pickPose.pz,"
    <<"m_pickPose.qx,"
    <<"m_pickPose.qy,"
    <<"m_pickPose.qz,"
    <<"m_pickPose.qw ="

    <<m_pickPose.pose.position.x<<" "
    <<m_pickPose.pose.position.y<<" "
    <<m_pickPose.pose.position.z<<" "
    <<m_pickPose.pose.orientation.x<<" "
    <<m_pickPose.pose.orientation.y<<" "
    <<m_pickPose.pose.orientation.z<<" "
    <<m_pickPose.pose.orientation.w <<std::endl;
#endif

    }

    return 0;
}

int Actuator::pick()
{
    int count = 0;
    MoveItErrorCode result;
    ros::Rate loope(10);
    _pickStopFlag = false;
    PoseStamped pickPose;

    setMoveitConfig();

    IDebug("pick ready!!!!!");

    // if(!m_parm.gripperConfig.allowed_coliision.empty() ||
    //     m_parm.gripperConfig.allowed_coliision != "NULL"){
    //     allowed.setEntry(m_parm.gripperConfig.allowed_coliision, OBJECT_ID, true);
    // }

#ifdef _Constraints_

    for(int i=0; i<m_parm.jointConstraints.joint_name.size(); i++){
        jointConstraints(m_parm.jointConstraints.joint_name[i],
                         m_parm.jointConstraints.position[i],
                         m_parm.jointConstraints.above[i],
                         m_parm.jointConstraints.below[i],
                         m_parm.jointConstraints.weight[i]);
        std::cout<<"count_for:"<<i<<std::endl;
    }
#endif

    if(m_parm.pickConfig.direct){
        pickPose.frame_id = m_pickPose.header.frame_id;
        pickPose.pose.position.x = m_pickPose.pose.position.x;
        pickPose.pose.position.y = m_pickPose.pose.position.y;
        pickPose.pose.position.z = m_pickPose.pose.position.z;
        pickPose.pose.orientation.w = m_pickPose.pose.orientation.w;
        pickPose.pose.orientation.x = m_pickPose.pose.orientation.x;
        pickPose.pose.orientation.y = m_pickPose.pose.orientation.y;
        pickPose.pose.orientation.z = m_pickPose.pose.orientation.z;

        return moveToPos(pickPose);
    }

    // std::vector<std::string> joint_name = _gripperGroup->getJointNames();
    // for(int i=0; i<joint_name.size(); i++)
    // {
    //     allowed.setEntry(joint_name[i], OBJECT_ID, true);
    // }

    makeGrasp();

    IDebug("_graspPoses.size()=%d\n", _graspPoses.size());
    
    while(count < (_graspPoses.size() + 3) && result != MoveItErrorCode::SUCCESS){
         if(_pickStopFlag)
             return -1;
         ROS_ERROR("count = %d\n", count);
         std::cout << "-----------#################-----------" << std::endl;
         std::cout << _graspPoses[0] << std::endl;
         std::cout << "-----------#################-----------" << std::endl;
         result = _moveGroup->pick(OBJECT_ID, _graspPoses, true);
         loope.sleep();
         count ++;
         ROS_ERROR("result = %d\n", result);
    }

    int cnt = 0;
    while (ros::ok() && cnt < 10 && result == MoveItErrorCode::SUCCESS)
    {
        if(receivePickTra)
        {
            receivePickTra = false;
            adjusterPtr->adjustmentTrajecotry(Picktraject);
            std::vector<moveit_msgs::RobotTrajectory> tra;
            adjusterPtr->getTrajectory(tra);
            result = pickExecute(tra);
        }
        cnt ++;
        ros::Duration(0.25).sleep();
    }
    

#ifdef jointConstraints
    _vecCons.joint_constraints.clear();
#endif

    // if(!result){
    //     if(!m_parm.gripperConfig.allowed_coliision.empty() ||
    //         m_parm.gripperConfig.allowed_coliision != "NULL"){
    //         allowed.removeEntry(m_parm.gripperConfig.allowed_coliision, OBJECT_ID);
    //     }
    //     return -1;
    // }
    if(!result)
        return -1;


    return 0;
}


int Actuator::makePlace()
{
    pick_vect vect, pre_vect, back_vect;
    std::vector<Quaternion> quats;
    moveit_msgs::PlaceLocation placeLocPos;
    _placeLocPoses.clear();

    getDiraction(m_placePose, m_parm.placeConfig.m_Roll, m_parm.placeConfig.m_Pitch,
                 m_parm.placeConfig.m_Yaw, vect, quats, placeScale);


    if(m_parm.placeConfig.pre_vect_x == 0 && m_parm.placeConfig.pre_vect_y == 0 && m_parm.placeConfig.pre_vect_z == 0){
        pre_vect = vect;
    }
    else{
        pre_vect.vect_x = m_parm.placeConfig.pre_vect_x;
        pre_vect.vect_y = m_parm.placeConfig.pre_vect_y;
        pre_vect.vect_z = m_parm.placeConfig.pre_vect_z;
    }
    if(m_parm.placeConfig.back_vect_x == 0 && m_parm.placeConfig.back_vect_y == 0 && m_parm.placeConfig.back_vect_z == 0){
        back_vect.vect_x = -(vect.vect_x);
        back_vect.vect_y = -(vect.vect_y);
        back_vect.vect_z = -(vect.vect_z);
    }
    else{
        back_vect.vect_x = m_parm.placeConfig.back_vect_x;
        back_vect.vect_y = m_parm.placeConfig.back_vect_y;
        back_vect.vect_z = m_parm.placeConfig.back_vect_z;
    }

    placeLocPos.place_pose.header.frame_id = m_placePose.header.frame_id;

    //放置进给位置
    placeLocPos.pre_place_approach.direction.header.frame_id = m_placePose.header.frame_id;
    placeLocPos.pre_place_approach.direction.vector.x = pre_vect.vect_x;
    placeLocPos.pre_place_approach.direction.vector.y = pre_vect.vect_y;
    placeLocPos.pre_place_approach.direction.vector.z = pre_vect.vect_z;
    placeLocPos.pre_place_approach.min_distance = m_parm.placeConfig.pre_dist_min;
    placeLocPos.pre_place_approach.desired_distance = m_parm.placeConfig.pre_dist_max;

    //放置回退位置
    placeLocPos.post_place_retreat.direction.header.frame_id = m_placePose.header.frame_id;
    placeLocPos.post_place_retreat.direction.vector.x = back_vect.vect_x;
    placeLocPos.post_place_retreat.direction.vector.y = back_vect.vect_y;
    placeLocPos.post_place_retreat.direction.vector.z = back_vect.vect_z;
    placeLocPos.post_place_retreat.min_distance = m_parm.placeConfig.back_dist_min;
    placeLocPos.post_place_retreat.desired_distance = m_parm.placeConfig.back_dist_max;

    //放置位置
    placeLocPos.place_pose.pose.position.x = m_placePose.pose.position.x;
    placeLocPos.place_pose.pose.position.y = m_placePose.pose.position.y;
    placeLocPos.place_pose.pose.position.z = m_placePose.pose.position.z;
    placeLocPos.allowed_touch_objects.push_back(OBJECT_ID);
    // placeLocPos.max_contact_force = 0;
    // placeLocPos.post_place_posture = makeGripperPosture(true);

    std::stringstream ss;
    if(quats.size() > 0){
        for(int i = 0; i<quats.size();i++){
            ss.clear();
            placeLocPos.place_pose.pose.orientation.w = m_placePose.pose.orientation.w;
            placeLocPos.place_pose.pose.orientation.x = m_placePose.pose.orientation.x;
            placeLocPos.place_pose.pose.orientation.y = m_placePose.pose.orientation.y;
            placeLocPos.place_pose.pose.orientation.z = m_placePose.pose.orientation.z;

            ss<<i;
            placeLocPos.id = ss.str();
            _placeLocPoses.push_back(placeLocPos);
        }
    }
    else{
        placeLocPos.place_pose.pose.orientation.w = m_placePose.pose.orientation.w;
        placeLocPos.place_pose.pose.orientation.x = m_placePose.pose.orientation.x;
        placeLocPos.place_pose.pose.orientation.y = m_placePose.pose.orientation.y;
        placeLocPos.place_pose.pose.orientation.z = m_placePose.pose.orientation.z;
        placeLocPos.id = "0";
        _placeLocPoses.push_back(placeLocPos);
    }

    return 0;
}

int Actuator::place()
{  
    PoseStamped pose;
    int count = 0;
    int ret = -1;
    MoveItErrorCode result;
    ros::Rate loope(10);
    _pickStopFlag = false;

    IDebug("place start!!!!!");

    setMoveitConfig();

    // if(!m_parm.gripperConfig.allowed_coliision.empty() ||
    //     m_parm.gripperConfig.allowed_coliision != "NULL"){
    //     allowed.setEntry(m_parm.gripperConfig.allowed_coliision, OBJECT_ID, true);
    // }
    
    // std::vector<std::string> joint_name = _gripperGroup->getJointNames();
    // for(int i=0; i<joint_name.size(); i++)
    // {
    //     allowed.setEntry(joint_name[i], OBJECT_ID, true);
    // }

#ifdef jointConstraints
    for(int i=0; i<m_parm.jointConstraints.joint_name.size(); i++){
        jointConstraints(m_parm.jointConstraints.joint_name[i],
                         m_parm.jointConstraints.position[i],
                         m_parm.jointConstraints.above[i],
                         m_parm.jointConstraints.below[i],
                         m_parm.jointConstraints.weight[i]);
        std::cout<<"count_for:"<<i<<std::endl;
    }
#endif

    if(m_parm.placeConfig.direct){

        result = MoveItErrorCode::SUCCESS;

        pose.frame_id = m_placePose.header.frame_id;
        pose.pose.position.x = m_placePose.pose.position.x;
        pose.pose.position.y = m_placePose.pose.position.y;
        pose.pose.position.z = m_placePose.pose.position.z;

        pose.pose.orientation.w = m_placePose.pose.orientation.w;
        pose.pose.orientation.x = m_placePose.pose.orientation.x;
        pose.pose.orientation.y = m_placePose.pose.orientation.y;
        pose.pose.orientation.z = m_placePose.pose.orientation.z;

        return moveToPos(pose);
    }

    ret = 0;
    makePlace();

    IDebug("place ready!!!!!");

    while(count < (_placeLocPoses.size() + 3) && result != MoveItErrorCode::SUCCESS){
        if(_pickStopFlag)
            return -1;
        ROS_ERROR("count = %d\n", count);
         std::cout << "-----------#################-----------" << std::endl;
         std::cout << _placeLocPoses[0] << std::endl;
         std::cout << "-----------#################-----------" << std::endl;
        result = _moveGroup->place(OBJECT_ID, _placeLocPoses, true);
        loope.sleep();
        count ++;
        ROS_ERROR("result = %d\n", result);
    }

    int cnt = 0;
    while (ros::ok() && cnt < 10 && result ==  MoveItErrorCode::SUCCESS)
    {
        if(receivePlaceTra)
        {
            receivePlaceTra = false;
            adjusterPtr->adjustmentTrajecotry(Placetraject);
            std::vector<moveit_msgs::RobotTrajectory> tra;
            adjusterPtr->getTrajectory(tra);
            result = placeExecute(tra);
        }
        cnt ++;
        ros::Duration(0.25).sleep();
    }
    _moveGroup->detachObject(OBJECT_ID);

    // if(!m_parm.gripperConfig.allowed_coliision.empty() ||
    //     m_parm.gripperConfig.allowed_coliision != "NULL"){
    //     allowed.removeEntry(m_parm.gripperConfig.allowed_coliision, OBJECT_ID);
    // }

#ifdef jointConstraints
    _vecCons.joint_constraints.clear();
#endif

    if(!result || ret != 0){
        return -1;
    }
    return 0;
}

void Actuator::jointConstraints(std::string jointName, double position, double above, double below,double weight)
{
#ifdef jointConstraints
    moveit_msgs::JointConstraint jocm;
    jocm.joint_name = jointName;
    jocm.position = position;
    jocm.tolerance_above = above;
    jocm.tolerance_below = below;
    jocm.weight = weight;
    _vecCons.joint_constraints.push_back(jocm);
#endif
    return;
}

int Actuator::stopPickplace()
{
    _pickStopFlag = true;
    _moveGroup->stop();
    return 0;
}


void Actuator::setMoveGroup()
{
    MOVE_GROUP = name;
    loadMoveit();
    adjusterPtr = new adjuster();
    adjusterPtr->setMoveGroupName(MOVE_GROUP);
    gripperPub = node.advertise<std_msgs::Bool>(MOVE_GROUP + "/gripper_state", 10);
}


/****/
int Actuator::updateParam(std::string path)
{
    YAML::Node config;
    config = YAML::LoadFile(path);
    parseConfig(config);
    return 0;
}

void Actuator::setVelocityAccelerated(double v, double a)
{
    adjusterPtr->setVelocityAcclerated(v, a);
}

void Actuator::pickTraCB(const moveit_msgs::PickupActionResultConstPtr& msg)
{
    if(msg->result.trajectory_stages.empty())
        return;
    std::vector<moveit_msgs::RobotTrajectory>().swap(Picktraject);
    std::vector<moveit_msgs::RobotTrajectory>().swap(gripperPicktraject);
    std::vector<std::string> jointNames = _moveGroup->getJointNames();
    for(int i=0; i<msg->result.trajectory_stages.size(); i++)
    {
        if(jointNames[0] != msg->result.trajectory_stages[i].joint_trajectory.joint_names[0])
        {
            gripperPicktraject.push_back(msg->result.trajectory_stages[i]);
            continue;
        }
        Picktraject.push_back(msg->result.trajectory_stages[i]);
    }
    receivePickTra = true;
}

void Actuator::placeTraCB(const moveit_msgs::PlaceActionResultConstPtr& msg)
{
    if(msg->result.trajectory_stages.empty())
        return;
    std::vector<moveit_msgs::RobotTrajectory>().swap(Placetraject);
    std::vector<moveit_msgs::RobotTrajectory>().swap(gripperPlacetraject);
    std::vector<std::string> jointNames = _moveGroup->getJointNames();
    for(int i=0; i<msg->result.trajectory_stages.size(); i++)
    {
        if(jointNames[0] != msg->result.trajectory_stages[i].joint_trajectory.joint_names[0])
        {
            gripperPlacetraject.push_back(msg->result.trajectory_stages[i]);
            continue;
        }
        Placetraject.push_back(msg->result.trajectory_stages[i]);
    }
    receivePlaceTra = true;
}

bool Actuator::execute(moveit_msgs::RobotTrajectory& tra)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = tra;
    if(_moveGroup->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        return true;
    }
    return false;
}

bool Actuator::pickExecute(std::vector<moveit_msgs::RobotTrajectory>& tra)
{
    if(!_pickStopFlag)
    if(execute(tra[0]))
    {
        gripperController(true);
        if(!_pickStopFlag)
        if(execute(tra[1]))
        {
            gripperController(false);
            _moveGroup->attachObject(OBJECT_ID);
            if(!_pickStopFlag)
            if(execute(tra[2]))
            {
                return true;
            }
        }
    }
    return false;
}

bool Actuator::placeExecute(std::vector<moveit_msgs::RobotTrajectory>& tra)
{
    if(!_pickStopFlag)
    if(execute(tra[0]))
    {
        if(!_pickStopFlag)
        if(execute(tra[1]))
        {
            gripperController(true);
            _moveGroup->detachObject(OBJECT_ID);
            if(!_pickStopFlag)
            if(execute(tra[2]))
            {
                return true;
            }
        }
    }
    return false;
}

int Actuator::getName(std::string &name)
{
    name = this ->_name;
    return 0;
}

ENTITY_TYPE Actuator::getEntityType()
{
    return this->_entityType;
}

bool Actuator::gripperController(bool open)
{
    // ros::Publisher gripperPub = node.advertise<std_msgs::Bool>(MOVE_GROUP + "/gripper_state", 10);
    std_msgs::Bool msg;
    msg.data = open;
    gripperPub.publish(msg);
    return true;
}


int Actuator::groupConfig(YAML::Node& yamlNode)
{
    if(!yamlNode["parameters"]){
        std::cerr<<"无参数设置"<<std::endl;
        return -1;
    }
    YAML::Node node = yamlNode["parameters"];
    if(node["moveitConfig"]){
        m_parm.moveitConfig.name = node["moveitConfig"]["name"].as<std::string>();
    }
    if(node["gripperConfig"]){
        m_parm.gripperConfig.name = node["gripperConfig"]["name"].as<std::string>();
        m_parm.gripperConfig.closeGripper = node["gripperConfig"]["closeGripper"].as<std::vector<double> >();
        m_parm.gripperConfig.openGripper = node["gripperConfig"]["openGripper"].as<std::vector<double> >();
    }
    
    return 0;
}

/****/

 H_EXPORT_PLUGIN(Actuator, "Actuator", "1.0")