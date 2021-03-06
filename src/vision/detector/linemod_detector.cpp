#include "linemod_detector.h"

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG
#include <utils/idebug.h>




LinemodDetector::LinemodDetector():CBaseDetector("LinemodDetector", false){
    icp_dist_min_ = 0.06f;
    px_match_min_ = 0.25;
    renderer_iterator_s = std::map <std::string, RendererIterator*>();
    initParam();
}

LinemodDetector::~LinemodDetector(){

}

void LinemodDetector::initParam(){
    param_n_points_ = 150;
    param_angle_step_ = 10;
    param_radius_min_ = 0.6;
    param_radius_max_ = 1.1;
    param_radius_step_ = 0.4;
    param_width_ = 960;
    param_height_ = 540;
    param_focal_length_x_ = 525.0;
    param_focal_length_y_ = 525.0;
    param_near_ = 0.1;
    param_far_ = 1000;
    th_obj_dist_ = 0.04f;
    verbose_ = true;


    static const int T_LVLS[] = {4, 15};
    std::vector< cv::Ptr<cv::linemod::Modality> > modalities;
    modalities.push_back(new cv::linemod::ColorGradient());
    modalities.push_back(new cv::linemod::DepthNormal());
    detector_ = new cv::linemod::Detector(modalities, std::vector<int>(T_LVLS, T_LVLS +2));
}

int LinemodDetector::loadData(const std::string path, const std::string objectName){

    IDebug("%s", "LinemodDetector loading data");

    if(Rs_.count(objectName)){
        IDebug("The %s object data alreadly loaded", objectName.c_str());
        return 0;
    }

    meshPath = path + "/mesh.stl";
    std::string RsFileName= path + "/RS.txt";
    std::string TsFileName= path + "/TS.txt";
    std::string KsFileName= path + "/KS.txt";
    std::string DistancesFileName= path + "/distances.txt";

    Renderer3d *renderer_ = new Renderer3d(meshPath);
    renderer_->set_parameters(param_width_, param_height_,\
                              param_focal_length_x_, param_focal_length_y_, param_near_, param_far_);

    RendererIterator *renderer_iterator_;
    renderer_iterator_ = new RendererIterator(renderer_, param_n_points_);
    renderer_iterator_->angle_step_ = param_angle_step_;
    renderer_iterator_->radius_min_ = float(param_radius_min_);
    renderer_iterator_->radius_max_ = float(param_radius_max_);
    renderer_iterator_->radius_step_ = float(param_radius_step_);
    renderer_iterator_s.insert(std::pair<std::string, RendererIterator*>(objectName, renderer_iterator_));

    // 加载平移选择矩阵表
    std::vector<cv::Mat> Rs_tmp, Ts_tmp, Ks_tmp;
	std::cout<< " begin "<<std::endl;
    loadVectorMat(RsFileName, Rs_tmp, 3, 3);
    loadVectorMat(TsFileName, Ts_tmp, 1, 3);
    loadVectorMat(KsFileName, Ks_tmp, 3, 3);

    Rs_.insert(std::pair<std::string, std::vector<cv::Mat>>(objectName, Rs_tmp));
    Ts_.insert(std::pair<std::string, std::vector<cv::Mat>>(objectName, Ts_tmp));
    Ks_.insert(std::pair<std::string, std::vector<cv::Mat>>(objectName, Ks_tmp));

    // 加载距离表
    std::vector<float> distances_tmp;
    std::ifstream distancesIf(DistancesFileName);
    std::string disTmp;
    size_t disIndex = 0;
    while(distancesIf >> disTmp){
        distances_tmp.push_back(std::stof(disTmp));
        disIndex ++;
    }

    distances_.insert(std::pair<std::string, std::vector<float> >(objectName, distances_tmp));

    /**
    *   加载Detector
    */
    cv::linemod::Detector *detector = new cv::linemod::Detector();
    loadDetctor(path + "/detector.xml", detector);

    std::string object_id_in_db = detector->classIds()[0];
    for (size_t template_id = 0; template_id < detector->numTemplates();
         ++template_id) {
        const std::vector<cv::linemod::Template> &templates_original = detector->getTemplates(object_id_in_db, template_id);
        detector_->addSyntheticTemplate(templates_original, objectName);
    }

    if(detector_->classIds().empty())
        return -1;

    return 0;
}


void LinemodDetector::loadVectorMat(std::string &fileName, std::vector<cv::Mat> &out, int rows, int cols){

    std::ifstream in(fileName);
    std::string tmpLine;
    std::vector<std::string> rss;

    while(std::getline(in, tmpLine, '|')){
        // 删除所有换行
        tmpLine = boost::algorithm::erase_all_copy(tmpLine, "\n");
        tmpLine.erase(0, 1);

        boost::split(rss, tmpLine, boost::is_any_of("  "));

        cv::Mat_<double> tmpMat_(rows,cols);

        for(int i = 0; i < rows; i ++)
            for(int j = 0; j < cols; j ++)
                tmpMat_(i,j) = std::stod(rss[i * cols + j].erase(rss[i * cols + j].length()-1));

        cv::Mat tmpMat = tmpMat_;

        out.push_back(tmpMat);
    }
    in.close();
}

void LinemodDetector::loadDetctor(const std::string file_name, cv::linemod::Detector *value){

    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    value->read(fs.root());

    cv::FileNode fn = fs["classes"];
    for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
        value->readClass(*i);
}

int LinemodDetector::detection(){

    std::cout<< " Linemod detectioning "<<std::endl;

    objs_.clear();
    poses.clear();

    // 构建检测输入源
    std::vector<cv::Mat> sources;

    // 转换源深度图的格式
    cv::Mat depth = depth_;
    if (depth_.depth() == CV_32F)
        depth_.convertTo(depth, CV_16UC1, 1000.0);

    cv::Mat color;
    if (color_.rows > 960)
        cv::pyrDown(color_.rowRange(0, 960), color);
    else
        color_.copyTo(color);

    // 将深度图作为检测源
    sources.push_back(color);
    sources.push_back(depth);
	cv::imwrite("show.jpg", color);
    /**
     * @brief matches 保存匹配成功的匹配对象
     */
    std::vector<cv::linemod::Match> matches;

    detector_->match(sources, 80.0f, matches);

    //将深度图转换为点云
    cv::Mat_<cv::Vec3f> depth_real_ref_raw;
    cv::Mat_<float> K, K_depth_;
    cv::Mat_<double> tmpMat_(3,3);
    tmpMat_ << 527.6911 , 0, 478.74231, 0, 528.01282, 266.14407, 0, 0, 1;
    K_depth_ = tmpMat_;
    K_depth_.convertTo(K, CV_32F);
#if CV_MAJOR_VERSION == 3
    cv::rgbd::depthTo3d(depth, K, depth_real_ref_raw);
#else
    cv::depthTo3d(depth, K, depth_real_ref_raw);
#endif


    /**
     * @brief BOOST_FOREACH 遍历刚刚使用linemod模板匹配到所有匹配对象
     */
    BOOST_FOREACH(const cv::linemod::Match & match, matches){

        const std::vector<cv::linemod::Template>& templates =
                detector_->getTemplates(match.class_id, match.template_id);

        /**
         * 填充位姿，训练过程中，一个物体代表一个claas_id，因此在识别过程中detctor_只能识别一个物体
         * 而在识别过程中的，每渲染一张图片，代表的是一个template_id，因此，我们通过识别匹配中得到的template_id来对应训练过程中
         * 对应的渲染图片，由于渲染过程中，每个图片[template_id]的位姿都是已知的，所以我们就可以获取到当前匹配的位姿态
         */
        cv::Matx33d R_match = Rs_.at(match.class_id)[match.template_id].clone();
        cv::Vec3d T_match = Ts_.at(match.class_id)[match.template_id].clone();
        float D_match = distances_.at(match.class_id)[match.template_id];
        cv::Mat K_match = Ks_.at(match.class_id)[match.template_id];

        /**
         * 将通过模板匹配得到的粗略位姿 放到渲染器进行重新渲染，得到深度图，然后将得到的深度图转换为点云
         */
        cv::Mat mask;
        cv::Rect rect;
        cv::Matx33d R_temp(R_match.inv());
        cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
        cv::Mat depth_ref_;
        renderer_iterator_s.at(match.class_id)->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);

        /**
         * @brief depth_real_model_raw  保存通过渲染器得到的深度图转换的点云
         */
        cv::Mat_<cv::Vec3f> depth_real_model_raw;
        cv::Mat renderK;
        K_match.convertTo(renderK, CV_32F);
#if CV_MAJOR_VERSION == 3
        cv::rgbd::depthTo3d(depth_ref_, renderK, depth_real_model_raw);
#else
        cv::depthTo3d(depth_ref_, renderK, depth_real_model_raw);
#endif

        /**
         * 生成点云和模型的包围矩形，<包围整个深度图>？
         */
        cv::Rect_<int> rect_model(0, 0, depth_real_model_raw.cols, depth_real_model_raw.rows);
        // 根据匹配得到的数据 修改包围矩形的X，Y
        cv::Rect_<int> rect_ref(rect_model);
        rect_ref.x += match.x;
        rect_ref.y += match.y;

        /**
         *  求图像深度图与rect_ref两个矩形的相交
         */
        rect_ref = rect_ref & cv::Rect(0, 0, depth_real_ref_raw.cols, depth_real_ref_raw.rows);

        // 当相交的区域宽小于5 或者长小于5时，跳过当前匹配
        if ((rect_ref.width < 5) || (rect_ref.height < 5))
            continue;

        // 得到两个矩形的最小
        if (rect_ref.width > rect_model.width)
            rect_ref.width = rect_model.width;
        if (rect_ref.height > rect_model.height)
            rect_ref.height = rect_model.height;
        if (rect_model.width > rect_ref.width)
            rect_model.width = rect_ref.width;
        if (rect_model.height > rect_ref.height)
            rect_model.height = rect_ref.height;


        //  裁剪从传感器获得的深度转点云
        cv::Mat_<cv::Vec3f> depth_real_ref = depth_real_ref_raw(rect_ref);
        //  裁剪渲染获得的深度转点云
        cv::Mat_<cv::Vec3f> depth_real_model = depth_real_model_raw(rect_model);

        // 获取裁剪过后的点云的中心点
        cv::Vec3f T_crop = depth_real_ref(depth_real_ref.rows / 2.0f, depth_real_ref.cols / 2.0f);
        // 加上物体的中心点到表面的距离
        T_crop(2) += D_match;

        // 检测得到的中心点坐标是否是正常的
        if (!cv::checkRange(T_crop))
            continue;

        // 为什么将相机点云的中心作为ICP运算中model的平移矩阵？
        cv::Vec3f T_real_icp(T_crop);

        if (!cv::checkRange(R_match))
            continue;

        // 将匹配得到的模板用来获取训练时的旋转矩阵来当作icp算法的旋转矩阵
        cv::Matx33f R_real_icp(R_match);

        //  将之前生成的点云[mat对象]转为vector的点云对象
        std::vector<cv::Vec3f> pts_real_model_temp;
        std::vector<cv::Vec3f> pts_real_ref_temp;

        /**
         * @brief px_ratio_missing  传感器深度图转的点云比渲染得到的点云缺少的点云比
         */
        float px_ratio_missing = matToVec(depth_real_ref, depth_real_model,\
                                          pts_real_ref_temp, pts_real_model_temp);
        if (px_ratio_missing > (1.0f-px_match_min_))
            continue;


        // 执行第一次近似ICP
        float px_ratio_match_inliers = 0.0f;
        float icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp,\
                                         R_real_icp, T_real_icp, px_ratio_match_inliers, 1);
        if (icp_dist > icp_dist_min_)
            continue;

        //执行最佳icp配准
        icp_dist = icpCloudToCloud(pts_real_ref_temp, pts_real_model_temp,\
                                   R_real_icp, T_real_icp, px_ratio_match_inliers, 2);

        // 保存相关的结果
        objs_.push_back(ObjData(pts_real_ref_temp, pts_real_model_temp, \
                                match.class_id, match.similarity, icp_dist, px_ratio_match_inliers, R_real_icp, T_crop));
    }

    //local non-maxima supression to find the best match at each position
    int count_pass = 0;
    std::vector<ObjData>::iterator it_o = objs_.begin();
    for (; it_o != objs_.end(); ++it_o)
        if (!it_o->check_done)
        {
            //initialize the object to publishT_real_icp
            ObjData *o_match = &(*it_o);
            int size_th = static_cast<int>((float)o_match->pts_model.size()*0.85);
            //find the best object match among near objects
            std::vector <ObjData>::iterator it_o2 = it_o;
            ++it_o2;
            for (; it_o2 != objs_.end(); ++it_o2)
                if (!it_o2->check_done)
                    if (cv::norm(o_match->t, it_o2->t) < th_obj_dist_)
                    {
                        it_o2->check_done = true;
                        if ((it_o2->pts_model.size() > size_th) && (it_o2->icp_dist < o_match->icp_dist))
                            o_match = &(*it_o2);
                    }

            //perform the final precise icp
            float icp_px_match = 0.0f;
            float icp_dist = icpCloudToCloud(o_match->pts_ref, o_match->pts_model, o_match->r, o_match->t, icp_px_match, 0);

            if (verbose_)
                std::cout << o_match->match_class <<  " " << o_match->match_sim << " icp " << icp_dist << ", ";

            //icp_dist in the same units as the sensor data
            //this distance is used to compute the ratio of inliers (points laying within this distance between the point clouds)
            icp_dist = 0.007f;
            float px_inliers_ratio = getL2distClouds(o_match->pts_model, o_match->pts_ref, icp_dist);
            if (verbose_)
                std::cout << " ratio " << o_match->icp_px_match << " or " << px_inliers_ratio << std::endl;

            pose p;
            p.objectName = o_match->match_class;
            RT2Pose(o_match->r, o_match->t, p);
            this->poses.push_back(p);

            ++count_pass;
        }
    if (verbose_ && (matches.size()>0)){
        std::cout << "matches  " << objs_.size() << " / " << count_pass << " / " << matches.size() << std::endl;
        return 0;
    }

    std::cout<< " Linemod detectioning out"<<std::endl;

    IDebug("%s", "nothing detection");
    return -1;

}

void LinemodDetector::RT2Pose(const cv::Matx33f &R, const cv::Vec3f &T, pose &pose){

    Eigen::Matrix3d t_R;
    cv::cv2eigen((cv::Matx33d)R, t_R);
    Eigen::Quaterniond q(t_R);
    Eigen::Vector4d q_tmp = q.coeffs();

    pose.quaternion.x = q_tmp[0];
    pose.quaternion.y = q_tmp[1];
    pose.quaternion.z = q_tmp[2];
    pose.quaternion.w = q_tmp[3];

    pose.position.x = T(0);
    pose.position.y = T(1);
    pose.position.z = T(2);

}

int LinemodDetector::getResult(std::vector<pose> &poses){
    poses = this->poses;

    if(poses.empty())
        return -1;

    IDebug("x = %lf, y = %lf, z = %lf, qx = %lf, qy = %lf, qz = %lf, qw = %lf", \
           this->poses[0].position.x, this->poses[0].position.y, this->poses[0].position.z,\
            this->poses[0].quaternion.x, this->poses[0].quaternion.y, this->poses[0].quaternion.z, this->poses[0].quaternion.w);

    return 0;
}

void LinemodDetector::setDepthImg(const cv::Mat &inputImg){
    // 保存输入的图像
    depth_ = inputImg;
}

void LinemodDetector::setColorImg(const cv::Mat &inputImg){
    // 保存输入的图像
    color_ = inputImg;
}


float LinemodDetector::icpCloudToCloud(const std::vector<cv::Vec3f> &pts_ref,\
                                       std::vector<cv::Vec3f> &pts_model, \
                                       cv::Matx33f &R, cv::Vec3f &T, float &px_inliers_ratio, int mode){

    //optimal rotation matrix
    cv::Matx33f R_optimal;
    //optimal transformation
    cv::Vec3f T_optimal;

    //the number of desired iterations defined depending on the mode
    int icp_it_th = 35; //maximal number of iterations
    if (mode == 1)
        icp_it_th = 4; //minimal number of iterations
    else if (mode == 2)
        icp_it_th = 4;


    // 两个点云之间所需的最小距离
    const float dist_th = 0.012f;

    // 两个点云距离之间的平均值
    float dist_mean = 0.0f;

    // 计算两个点云的欧式距离
    px_inliers_ratio = getL2distClouds(pts_model, pts_ref, dist_mean, mode);
    //The difference between two previously obtained mean distances between the reference and the model point clouds
    float dist_diff = std::numeric_limits<float>::max();

    //the number of performed iterations
    int iter = 0;
    while (( ((dist_mean > dist_th) && (dist_diff > 0.0001f)) || (mode == 1) ) && (iter < icp_it_th))
    {
        ++iter;

        //subsample points from the match and ref clouds
        if (pts_model.empty() || pts_ref.empty())
            continue;

        // 为每个点云计算中心点
        //compute centroids of each point subset
        cv::Vec3f m_centroid, r_centroid;
        getMean(pts_model, m_centroid);
        getMean(pts_ref, r_centroid);

        //compute the covariance matrix
        cv::Matx33f covariance (0,0,0, 0,0,0, 0,0,0);
        std::vector<cv::Vec3f>::iterator it_s = pts_model.begin();
        std::vector<cv::Vec3f>::const_iterator it_ref = pts_ref.begin();
        for (; it_s < pts_model.end(); ++it_s, ++it_ref)
            covariance += (*it_s) * (*it_ref).t();

        // 通过SVD求解最优解
        cv::Mat w, u, vt;
        cv::SVD::compute(covariance, w, u, vt);
        //compute the optimal rotation
        R_optimal = cv::Mat(vt.t() * u.t());

        //compute the optimal translation
        T_optimal = r_centroid - R_optimal * m_centroid;
        if (!cv::checkRange(R_optimal) || !cv::checkRange(T_optimal))
            continue;

        //transform the point cloud
        transformPoints(pts_model, pts_model, R_optimal, T_optimal);

        //compute the distance between the transformed and ref point clouds
        dist_diff = dist_mean;
        px_inliers_ratio = getL2distClouds(pts_model, pts_ref, dist_mean, mode);
        dist_diff -= dist_mean;

        //update the translation matrix: turn to opposite direction at first and then do translation
        T = R_optimal * T;
        //do translation
        cv::add(T, T_optimal, T);
        //update the rotation matrix
        R = R_optimal * R;
        //std::cout << " it " << iter << "/" << icp_it_th << " : " << std::fixed << dist_mean << " " << d_diff << " " << px_inliers_ratio << " " << pts_model.size() << std::endl;
    }

    return dist_mean;

}


/** Computes the L2 distance between two vectors of 3D points of the same size */
float LinemodDetector::getL2distClouds(const std::vector<cv::Vec3f> &model, const std::vector<cv::Vec3f> &ref, float &dist_mean, const float mode)
{
    int nbr_inliers = 0;
    int counter = 0;
    float ratio_inliers = 0.0f;

    float dist_expected = dist_mean * 3.0f;
    dist_mean = 0.0f;

    //use the whole region
    std::vector<cv::Vec3f>::const_iterator it_match = model.begin();
    std::vector<cv::Vec3f>::const_iterator it_ref = ref.begin();
    for(; it_match != model.end(); ++it_match, ++it_ref)
    {
        if (!cv::checkRange(*it_ref))
            continue;

        if (cv::checkRange(*it_match))
        {
            float dist = cv::norm(*it_match - *it_ref);
            if ((dist < dist_expected) || (mode == 0))
                dist_mean += dist;
            if (dist < dist_expected)
                ++nbr_inliers;
        }
        ++counter;
    }

    if (counter > 0)
    {
        dist_mean /= float(nbr_inliers);
        ratio_inliers = float(nbr_inliers) / float(counter);
    }
    else
        dist_mean = std::numeric_limits<float>::max();

    return ratio_inliers;
}


void LinemodDetector::transformPoints(const std::vector<cv::Vec3f> &src, \
                                      std::vector<cv::Vec3f>& dst, const cv::Matx33f &R, const cv::Vec3f &T)
{
    std::vector<cv::Vec3f>::const_iterator it_src = src.begin();
    std::vector<cv::Vec3f>::iterator it_dst = dst.begin();
    for (; it_src != src.end(); ++it_src, ++it_dst) {
        if (!cv::checkRange(*it_src))
            continue;
        (*it_dst) = R * (*it_src) + T;
    }
}


void LinemodDetector::getMean(const std::vector<cv::Vec3f> &pts, cv::Vec3f& centroid)
{
    centroid = cv::Vec3f(0.0f, 0.0f, 0.0f);
    size_t n_points = 0;
    for (std::vector<cv::Vec3f>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
        if (!cv::checkRange(*it))
            continue;
        centroid += (*it);
        ++n_points;
    }

    if (n_points > 0)
    {
        centroid(0) /= float(n_points);
        centroid(1) /= float(n_points);
        centroid(2) /= float(n_points);
    }
}

float LinemodDetector::matToVec(const cv::Mat_<cv::Vec3f> &src_ref, const cv::Mat_<cv::Vec3f> &src_mod, \
                                std::vector<cv::Vec3f>& pts_ref, std::vector<cv::Vec3f>& pts_mod)
{
    pts_ref.clear();
    pts_mod.clear();
    int px_missing = 0;

    cv::MatConstIterator_<cv::Vec3f> it_ref = src_ref.begin();
    cv::MatConstIterator_<cv::Vec3f> it_mod = src_mod.begin();
    for (; it_ref != src_ref.end(); ++it_ref, ++it_mod)
    {
        if (!cv::checkRange(*it_ref))
            continue;

        pts_ref.push_back(*it_ref);
        if (cv::checkRange(*it_mod))
        {
            pts_mod.push_back(*it_mod);
        }
        else
        {
            pts_mod.push_back(cv::Vec3f(0.0f, 0.0f, 0.0f));
            ++px_missing;
        }
    }

    float ratio = 0.0f;
    if (pts_ref.size() > 0)
        ratio = static_cast<float>(px_missing) /static_cast<float>(pts_ref.size());
    return ratio;
}


H_EXPORT_PLUGIN(LinemodDetector, "LinemodDetector", "1.0")
