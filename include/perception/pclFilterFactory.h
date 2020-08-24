#pragma once
#include "PclFilterEntry.h"
#include "boost/shared_ptr.hpp"
#include "voxelPclFilter.h"
#include "regionFilters.h"
#include "objectPclFilter.h"

namespace PCLFILTER{

enum FilterSort{
    voxel,
    radius,
    object,
    region,
}FILTERSORT;

}

using namespace boost;
template<typename TPoint>
class pclFilterFactory
{
public:
    static boost::shared_ptr<PclFilterEntry<TPoint > > create(PCLFILTER::FilterSort sort);
private:
    static boost::shared_ptr<PclFilterEntry<TPoint > > ptr;
};

template<typename TPoint>
boost::shared_ptr<PclFilterEntry<TPoint > > pclFilterFactory<TPoint >::ptr = nullptr;

template<typename TPoint>
boost::shared_ptr<PclFilterEntry<TPoint > > pclFilterFactory<TPoint >::create(PCLFILTER::FilterSort sort)
{
    switch (sort) {
        case PCLFILTER::voxel:
            ptr = boost::make_shared<voxelPclFilter<TPoint > >("voxel");
            break;
        case PCLFILTER::region:
            ptr = boost::make_shared<regionPclFilter<TPoint > >("region");
            break;
        case PCLFILTER::object:
            ptr = boost::make_shared<objectPclFilter<TPoint > >("object");
            break;
        default:
            ptr = nullptr;
            break;
    }
    return ptr;
}
