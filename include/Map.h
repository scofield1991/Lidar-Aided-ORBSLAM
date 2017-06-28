/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include <set>

#include <mutex>
#include <iomanip>

#include "Odom/NavState.h"
#include "Odom/PMtypes.h"

namespace ORB_SLAM2
{

  
class MapPoint;
class KeyFrame;

class KFIdComapre
{
public:
    bool operator()(const KeyFrame* kfleft,const KeyFrame* kfright) const;
};

struct KFPair
{
    cv::Mat Twc;
    NavState Twl;
    DBoW2::BowVector vBow;
};

struct PointItem
{
    double maxDis;
    double minDis;
    cv::Mat normal;
};

// struct for model keyframe
struct ModelKeyFrame
{
    //Id
    int nId;
    //number of model points
    unsigned int num;
    //camera pose
    cv::Mat Twc;
    //navigation pose
    NavState Twl;
    //bow vector
    DBoW2::BowVector vBow;
    //feature vector
    DBoW2::FeatureVector vFeat;
    //model map points seen in this keyframe
    std::vector<MapPoint*> mvpModelPoints;
    //KF descriptor
    cv::Mat mDes;
    //KF keypoint angle
    std::vector<float> vAngle;
    //database
    int score;
    
    bool operator<( const ModelKeyFrame& other ) const
    {
        return (this->nId < other.nId);
    }
};

class Map
{ 
  
public:
    void LoadMap(const string &str);
    
    void setMaxIndex(const int &index);
    int getMaxIndex();
    
    void setGlobalMapUpdated(const bool &isupdated);
    bool hasGlobalMap();
    
    //Model based method
    void AddModelPoint(MapPoint* pMP);
    std::vector<MapPoint*> GetAllModelPoints();
    long unsigned int ModelPointsInMap();
    
    void AddModelKeyFrame(ModelKeyFrame pMdKF);
    std::vector<ModelKeyFrame> GetAllModelKFs();
    long unsigned int ModelKFsInMap();
    
    std::vector<MapPoint*> vpSearchedModelPoints;
    
protected:
    std::set<MapPoint*> mspModelPoints;
    std::vector<ModelKeyFrame> mvpModelKeyFrames;
    
private:
    
    bool _updateMap;
  
    int maxindex;
    
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    
    shared_ptr<DP> _cloud;
    shared_ptr<DP> _local_cloud;
    
    std::vector<Eigen::Vector3d> mvPointsPos;
    std::vector<std::vector<cv::Mat>> vFeatureDescriptros;
    
    //id, mnId, KF
    std::map<int, std::pair<int, KFPair>> mKFItems;
    
    // mnId, point list
    std::map<int, std::vector<int>> mKFPtIds;
    
    std::vector<shared_ptr<DP>> vCloud;
    
    std::map<int, PointItem> mPtItems;
    
    std::vector<int> vIndexProjFrameMapPt;
    
    std::vector<double> vRadiuls;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*,KFIdComapre> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;


    std::mutex mMutexMap;
    
};

} //namespace ORB_SLAM

#endif // MAP_H
