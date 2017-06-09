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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

bool KFIdComapre::operator ()(const KeyFrame* kfleft,const KeyFrame* kfright) const
{
    return kfleft->mnId < kfright->mnId;
}
  
Map::Map():mnMaxKFid(0),maxindex(0)
{
}

void Map::setMaxIndex(const int& index)
{
    maxindex = index;
}

int Map::getMaxIndex()
{
    return maxindex;
}

void Map::LoadMap(const string& str)
{
    stringstream ss;
    ss << str.c_str() << "/lidarmap.vtk";
    DP tempdp = DP::load(ss.str().c_str());
    shared_ptr<DP> cloud_(new DP(tempdp));
    _cloud = cloud_;
    
    ss.str("");
    ss << str.c_str() << "/visualmap.txt";
    ifstream points_in(ss.str().c_str());
    int NumPoints, NumKFs;
    points_in >> NumKFs;
    points_in >> NumPoints;
    mvPointsPos.clear();
    for(int i = 0; i < NumPoints; i++)
    {
	Eigen::Vector3d vPos;
	points_in >> vPos[0];
	points_in >> vPos[1];
	points_in >> vPos[2];
	mvPointsPos.push_back(vPos);
    }
    points_in.close();
    
    mvKFItems.clear();
    ifstream kfs_in;
    for(int i = 0; i < NumKFs; i++)
    {
	ss.str("");
	ss << str.c_str() << "/kf_" << std::setfill ('0') << std::setw (5) << i << ".txt";

	kfs_in.open(ss.str().c_str());
	
	KFPair kfPair;
	
	Eigen::Vector3d twc;
	Eigen::Quaterniond qwc;
	cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
	kfs_in >> twc[0]; kfs_in >> twc[1]; kfs_in >> twc[2];
	kfs_in >> qwc.w(); kfs_in >> qwc.x(); kfs_in >> qwc.y(); kfs_in >> qwc.z();
	Eigen::Matrix3d Rwc = qwc.toRotationMatrix();
	Twc = Converter::toCvSE3(Rwc, twc).t();

	Eigen::Vector3d twl;
	kfs_in >> twl[0]; kfs_in >> twl[1]; kfs_in >> twl[2];
	Eigen::Quaterniond qwl;
	kfs_in >> qwl.w(); kfs_in >> qwl.x(); kfs_in >> qwl.y(); kfs_in >> qwl.z();
	NavState Twl;
	Twl.Set_Pos(twl);
	Twl.Set_Rot(qwl.toRotationMatrix());

	kfPair.Twc = Twc;
	kfPair.Twl = Twl;
	
	int Num_Bows;
	kfs_in >> Num_Bows;  
	DBoW2::BowVector vBow;
	for(int k = 0; k < Num_Bows; k++)
	{
	    DBoW2::WordId id;
	    DBoW2::WordValue value;
	    kfs_in >> id;
	    kfs_in >> value;
	    vBow.addWeight(id, value);
	}
	
	kfPair.vBow = vBow;
	
	mvKFItems.push_back(kfPair);
	
	kfs_in.close();
    }
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    _cloud = pKF->pCloud;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
