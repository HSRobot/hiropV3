#pragma once

//#define DLL_EXPORT __attribute__((visibility("default")))
//#define DLL_EXPORT _declspec(dllexport)

#include <iostream>
using namespace std;
#include "PclFilterEntry.h"
#include "PclFilterManage.h"
using namespace boost;
template<typename TPoint>
class PclFilterManagerSingle{

private:
    PclFilterManagerSingle(){
        manager = new PclFilterManager<TPoint>();
    }
    static PclFilterManagerSingle<TPoint> * ptr;
    PclFilterManager<TPoint> *manager;

public:
    static PclFilterManagerSingle<TPoint> * getInstance(){
        return ptr;
    }

    PclFilterManager<TPoint>* getManager(){
        return manager;
    }

    ~PclFilterManagerSingle<TPoint>()
    {
        delete ptr;
        delete manager;
    }

};

template<typename TPoint>
PclFilterManagerSingle<TPoint> * PclFilterManagerSingle<TPoint>::ptr = new PclFilterManagerSingle<TPoint>();

