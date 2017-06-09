#ifndef LIDARMAP_H
#define LIDARMAP_H

#include "KeyFrame.h"
#include <set>

#include <mutex>

#include "Converter.h"

#include "Odom/PMtypes.h"

namespace ORB_SLAM2
{

class KeyFrame;

class KFComapre
{
public:
    bool operator()(const KeyFrame* kfleft,const KeyFrame* kfright) const;
};

class LidarMap
{
protected:
  
    PM::DataPointsFilters inputFilters;
  
    shared_ptr<DP> _cloud;
    
    shared_ptr<PM::Transformation> transformation;
  
public:
    LidarMap();

    void AddKeyFrame(KeyFrame* pKF);
    void EraseKeyFrame(KeyFrame* pKF);

    std::vector<KeyFrame*> GetAllKeyFrames();
    shared_ptr<DP> GetAllLidarMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    
protected:
    
    std::set<KeyFrame*,KFComapre> mspKeyFrames;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;
    
};

} //namespace ORB_SLAM

#endif // LIDARMAP_H
