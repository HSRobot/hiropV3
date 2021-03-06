#pragma GCC diagnostic ignored "-Wconversion-null"

#include "vision/py_base_detector.h"
#include "utils/idebug.h"
#include "utils/py_lock_helper.h"

#include <stdlib.h>
//#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>

using namespace hirop_vision;

PyBaseDetector::PyBaseDetector(PyObject *pClass, std::string detectorName){

    this->pClass = pClass;
    this->name = detectorName + "Detector";

    if(!pClass){
        IErrorPrint("%s", "Python instance load failed: the class was null");
        return;
    }

    pClassInstance = PyObject_CallObject(pClass, NULL);
    if(!pClassInstance){
        IErrorPrint("%s", "Python Detector can't intance");
        PyErr_Print();
    }

    entityType = PYTHON;
    initNump();
}

PyBaseDetector::~PyBaseDetector(){

    if(pClassInstance)
        Py_DECREF(pClassInstance);

}
#ifdef WITH_PYTHON3
int PyBaseDetector::initNump(){
    import_array();
}
#else
void PyBaseDetector::initNump(){
    import_array();
}
#endif

int PyBaseDetector::loadData(const std::string path, const std::string objectName){

    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector load data failed: the class not instance");
        PyErr_Print();
        return -1;
    }

    PyObject* ret =  PyObject_CallMethod(pClassInstance, "loadData", "ss",  path.c_str(), objectName.c_str());

    if(!ret){
        PyErr_Print();
        return -1;
    }

    return 0;
}

int PyBaseDetector::detection(){

    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector start detetion error: the class not instance");
        return -1;
    }

    PyObject *ret = PyObject_CallMethod(pClassInstance, "detection","");
    if(!ret){
        PyErr_Print();
        return -1;
    }

    int res = -1;
    PyArg_Parse(ret, "i", &res);

    if(res)
        return -1;

    return 0;
}

void PyBaseDetector::setColorImg(const cv::Mat &inputImg){

    if(inputImg.empty() == true)
        return;

    PyLockHelper lock;
    int rows =  inputImg.rows;
    int cols = inputImg.cols;
    int channels = inputImg.channels();

    IDebug("rows = %d, cols = %d, channels = %d", rows, cols, channels);

    //@todo 此处会造成内存泄露
    char * colorTmp= new char[rows * cols * channels];
    memcpy(colorTmp, inputImg.ptr<uchar>(0), rows * cols * channels);

    IDebug("memcpy mat finish");

    npy_intp Dims[3] = {rows, cols, channels};
    PyObject *matObj = PyArray_SimpleNewFromData(3, Dims, NPY_UBYTE, colorTmp);

    IDebug("PyArray_SimpleNewFromData finish");

    PyObject *ret = PyObject_CallMethod(pClassInstance, "setColorImg", "O", matObj);
    if(!ret)
        PyErr_Print();

    delete colorTmp;

}

void PyBaseDetector::setDepthImg(const cv::Mat &inputImg){

    PyLockHelper lock;
    int rows =  inputImg.rows;
    int cols = inputImg.cols;
    int channels = inputImg.channels();

    if(inputImg.empty() == true)
        return;

    IDebug("rows = %d, cols = %d, channels = %d", rows, cols, channels);

    //@todo 此处会造成内存泄露
    char * depthTmp= new char[rows * cols * channels];
    memcpy(depthTmp, inputImg.ptr<uchar>(0), rows * cols * channels);

    npy_intp Dims[3] = {rows, cols, channels};
    PyObject *matObj = PyArray_SimpleNewFromData(3, Dims, NPY_UBYTE, depthTmp);

    PyObject *ret = PyObject_CallMethod(pClassInstance, "setDepthImg", "O", matObj);

    if(!ret)
        PyErr_Print();

    delete depthTmp;
}

void PyBaseDetector::setPointCloud(const pcl::PCLPointCloud2 &pointcloud2_ptr)
{

}

int PyBaseDetector::getResult(std::vector<pose> &poses){
    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector getResult: the class not instance");
        return -1;
    }

    PyObject* ret =  PyObject_CallMethod(pClassInstance, "getResult", "");

    if(!ret)
        PyErr_Print();


    PyArrayObject *ret_array;
    PyArray_OutputConverter(ret, &ret_array);

    /**
     * 获取数据维度，每组数据有7维(x, y, z, qx, qy, qz, qw)
     */
    npy_intp *shape = PyArray_SHAPE(ret_array);
    double* pDataPtr = (double*)PyArray_DATA(ret_array);

    int rows = shape[0];

    /**
     * 如果包含数据则进行处理
     */
    if( rows > 0 ){
        poses.reserve(rows);
        for(int i = 0; i < shape[0]; ++i){
            int row = i * shape[1];
            pose p;
            p.position.x = static_cast<double>(pDataPtr[row +0]);
            p.position.y = static_cast<double>(pDataPtr[row + 1]);
            p.position.z = static_cast<double>(pDataPtr[row + 2]);
            p.quaternion.x = static_cast<double>(pDataPtr[row + 3]);
            p.quaternion.y = static_cast<double>(pDataPtr[row + 4]);
            p.quaternion.z = static_cast<double>(pDataPtr[row + 5]);
            p.quaternion.w = static_cast<double>(pDataPtr[row + 6]);
            poses.push_back(p);
        }
    }else{
        pose p;
        p.position.x = 0;
        p.position.y = 0;
        p.position.z = 0;
        p.quaternion.x = 0;
        p.quaternion.y = 0;
        p.quaternion.z = 0;
        p.quaternion.w = 1;
        poses.push_back(p);
    }

    Py_DECREF(pDataPtr);
    Py_DECREF(ret_array);

    //    Py_DECREF(ret);
    return 0;
}

int PyBaseDetector::getName(std::string &name){
    name = this->name;
    return 0;
}

ENTITY_TYPE PyBaseDetector::getEntityType(){
    return entityType;
}

int PyBaseDetector::isMultiDetector(){

    return 0;
}

int PyBaseDetector::parseConfig(const YAML::Node &node){
    return 0;
}

int PyBaseDetector::getPreImg(cv::Mat &preImg){

    PyLockHelper lock;

    if(!pClassInstance){
        IErrorPrint("%s", "Python detector getPreImg: the class not instance");
        return -1;
    }

    PyObject* ret =  PyObject_CallMethod(pClassInstance, "getPreImg", "");

    if(!ret){
        PyErr_Print();
        return -1;
    }
    PyArrayObject *ret_array;
    PyArray_OutputConverter(ret, &ret_array);

    /**
     * 获取数据维度，每组数据有7维(x, y, z, qx, qy, qz, qw)
     */
    npy_intp *shape = PyArray_SHAPE(ret_array);
    char* pDataPtr = (char*)PyArray_DATA(ret_array);

    int rows = shape[0];
    int cols = shape[1];
    int channels = shape[2];

    IDebug("rows = %d, cols = %d, channels = %d", rows, cols, channels);

    switch(channels){

    case 1:
        preImg = cv::Mat(rows, cols, CV_8UC1);
        break;
    case 2:
        preImg = cv::Mat(rows, cols, CV_8UC2);
        break;
    case 3:
        preImg = cv::Mat(rows, cols, CV_8UC3);
        break;
    }

    memcpy(preImg.data, pDataPtr, rows*cols*channels);
    return 0;
}

bool PyBaseDetector::havePreImg(){
    return true;
}
