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
#include <MapPoint.h>

#include<mutex>

namespace ORB_SLAM2
{

bool KFIdComapre::operator ()(const KeyFrame* kfleft,const KeyFrame* kfright) const
{
    return kfleft->mnId < kfright->mnId;
}
  
Map::Map():mnMaxKFid(0),maxindex(0),_updateMap(false)
{
}

void Map::AddModelPoint(MapPoint* pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspModelPoints.insert(pMP);
}

std::vector< MapPoint* > Map::GetAllModelPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspModelPoints.begin(),mspModelPoints.end());
}

long unsigned int Map::ModelPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
//     return mspModelPoints.size();
    unsigned int number = 0;
    for(ModelKeyFrame it:mvpModelKeyFrames)
    {
	number += it.num;
    }
    return number;
}

void Map::AddModelKeyFrame(ModelKeyFrame pMdKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpModelKeyFrames.push_back(pMdKF);
}

std::vector<ModelKeyFrame> Map::GetAllModelKFs()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpModelKeyFrames;
}

long unsigned int Map::ModelKFsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpModelKeyFrames.size();
}

void Map::setGlobalMapUpdated(const bool& isupdated)
{
    _updateMap = isupdated;
}

bool Map::hasGlobalMap()
{
    return _updateMap;
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
    //load the whole lidar map
    stringstream ss;
    ss << str.c_str() << "/lidarmap.vtk";
    DP tempdp = DP::load(ss.str().c_str());
    shared_ptr<DP> cloud_(new DP(tempdp));
    _cloud = cloud_;
    
    //load total number of KFs
    ss.str("");
//     ss << str.c_str() << "/visual_map/visualmap.txt";
    ss << str.c_str() << "/numKF.txt";
    ifstream points_in(ss.str().c_str());
    int /*NumPoints, */NumKFs;
    points_in >> NumKFs;
    cout << "There should be " << NumKFs << " KFs" << endl;
//     points_in >> NumPoints;
    
    //load lidar map
    vCloud.clear();
    for(int i = 0; i < NumKFs; i++)
    {
	ss.str("");
	ss << str.c_str() << "/lidar_map/lm_" << std::setfill ('0') << std::setw (5) << i << ".vtk";
	DP tDP = DP::load(ss.str().c_str());
	tDP.removeFeature("z");
	DP::View viewOnnormals = tDP.getDescriptorViewByName("normals");
	Eigen::MatrixXf newNormals = viewOnnormals.topRows(2);
	tDP.removeDescriptor("normals");
	tDP.addDescriptor("normals", newNormals);
	shared_ptr<DP> _cloud_(new DP(tDP));
	vCloud.push_back(_cloud_);
    }
    
    //load KF
    mKFItems.clear();
    ifstream kfs_in;
    for(int i = 0; i < NumKFs; i++)
    {
	cout << "load " << i << "th KF" << endl;
      
	ss.str("");
	ss << str.c_str() << "/kf/kf_" << std::setfill ('0') << std::setw (5) << i << ".txt";

	kfs_in.open(ss.str().c_str());
	
	ModelKeyFrame mMdKF;
	
	// read ID
	int mnId;
	kfs_in >> mnId;
	
// 	KFPair kfPair;
	
	// read camera pose and navigation pose
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

// 	kfPair.Twc = Twc;
// 	kfPair.Twl = Twl;
	
	// read BoW vector
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
	
	// read feature vector
	int Num_Feats;
	kfs_in >> Num_Feats;
	DBoW2::FeatureVector vFeat;
	for(int k = 0; k < Num_Feats; k++)
	{
	    DBoW2::NodeId id;
	    unsigned int value;
	    kfs_in >> id;
	    int numVec;
	    kfs_in >> numVec;
	    for(int num = 0; num < numVec; num++)
	    {
		kfs_in >> value;
		vFeat.addFeature(id, value);
	    }
	}
	
	// read model points
	int num_ModelPoints, real_num = 0;
	kfs_in >> num_ModelPoints;
	vector<MapPoint*> vMdMPs;
	for(int k = 0; k < num_ModelPoints; k++)
	{ 
	    //read pose
	    Eigen::Vector3d vPos;
	    kfs_in >> vPos[0];
	 
	    //judge if this is a good point
	    if(vPos[0] == -1)
	    {
		vMdMPs.push_back(static_cast<MapPoint*>(NULL));
		continue;
	    }
	    real_num++;
	    
	    kfs_in >> vPos[1];
	    kfs_in >> vPos[2];
	    cv::Mat x3D = Converter::toCvMat(vPos);
	    
	    //read observations
	    int numObs;
	    kfs_in >> numObs;
	    vector<int> vObs;
	    for(int k = 0; k < numObs; k++)
	    {
		int mnId;
		kfs_in >> mnId;
		vObs.push_back(mnId);
	    }
	    
	    //read min(max) dis and norm vector
	    double mindis, maxdis;
	    kfs_in >> mindis >> maxdis;
	    double normX, normY, normZ;
	    cv::Mat norm = cv::Mat(3,1,CV_32F);
	    kfs_in >> normX; kfs_in >> normY; kfs_in >> normZ;
	    norm.at<float>(0,0) = normX;
	    norm.at<float>(1,0) = normY;
	    norm.at<float>(2,0) = normZ;
	    
	    //read descriptor
	    int numRows, numCols;
	    kfs_in >> numRows >> numCols;
	    cv::Mat mDes = cv::Mat(1, 32, CV_8U);
	    for(int r = 0; r < numRows; r++)
	    {
		int temp;
		for(int c = 0; c < numCols; c++)
		{kfs_in >> temp; mDes.at<unsigned char>(r, c) = temp;}
	    }
	    
	    //create model point
	    MapPoint* pMP = new MapPoint(x3D, mindis, maxdis, norm, mDes, vObs, this);
	    vMdMPs.push_back(pMP);
	}
	
	//read KF descriptor
	int numRowsKF, numColsKF;
	kfs_in >> numRowsKF >> numColsKF;
	cv::Mat mDesKF = cv::Mat(numRowsKF, numColsKF, CV_8U);
	for(int r = 0; r < numRowsKF; r++)
	{
	    int temp;
	    for(int c = 0; c < numColsKF; c++)
	    {kfs_in >> temp; mDesKF.at<unsigned char>(r, c) = temp;}
	}
	
	//read KF keypoint angles
	int numKpt;
	kfs_in >> numKpt;
	vector<float> tempAngles;
	tempAngles.resize(numKpt);
	for(int times = 0; times < numKpt; times++)
	{
	    kfs_in >> tempAngles[i];
	}
	
	ModelKeyFrame pMdKF;
	pMdKF.nId = mnId;
	pMdKF.num = real_num;
	pMdKF.Twc = Twc;
	pMdKF.Twl = Twl;
	pMdKF.vBow = vBow;
	pMdKF.vFeat = vFeat;
	pMdKF.mvpModelPoints = vMdMPs;
	pMdKF.mDes = mDesKF;
	pMdKF.vAngle = tempAngles;
	//add model keyframe into map
	this->AddModelKeyFrame(pMdKF);
	
	cout << "this KeyFrame has " << real_num << " model points." << endl;
	
// 	kfPair.vBow = vBow;
	
// 	std::pair<int, KFPair> tmpPair;
// 	tmpPair.first = mnId;
// 	tmpPair.second = kfPair;
// 	mKFItems[i] = tmpPair;
	
	kfs_in.close();
    }
    cout << "Have " << mvpModelKeyFrames.size() << " model keyframes" << endl;
    //     //load visual map
//     mvPointsPos.clear();
//     vFeatureDescriptros.clear();
//     mKFPtIds.clear();
//     mPtItems.clear();
//     ifstream desIn;
//     for(int i = 0; i < NumPoints; i++)
//     {
// 	Eigen::Vector3d vPos;
// 	points_in >> vPos[0];
// 	points_in >> vPos[1];
// 	points_in >> vPos[2];
// 	int numObs;
// 	points_in >> numObs;
// 	for(int k = 0; k < numObs; k++)
// 	{
// 	    int mnId;
// 	    points_in >> mnId;
// 	    mKFPtIds[mnId].push_back(i);
// 	}
// 	double mindis, maxdis;
// 	points_in >> mindis >> maxdis;
// 	double normX, normY, normZ;
// 	cv::Mat norm = cv::Mat(3,1,CV_32F);
// 	points_in >> normX; points_in >> normY; points_in >> normZ;
// 	norm.at<float>(0,0) = normX;
// 	norm.at<float>(1,0) = normY;
// 	norm.at<float>(2,0) = normZ;
// 	PointItem tempPti;
// 	tempPti.minDis = mindis;
// 	tempPti.maxDis = maxdis;
// 	tempPti.normal = norm;
// 	mPtItems[i] = tempPti;
// 	mvPointsPos.push_back(vPos);
// 	ss.str("");
// 	ss << str.c_str() << "/visual_map/des_" << std::setfill ('0') << std::setw (5) << i << ".txt";
// 	desIn.open(ss.str().c_str());
// 	int numDes, numRows, numCols;
// 	desIn >> numDes >> numRows >> numCols;
// 	vector<cv::Mat> vDes;
// 	for(int numd = 0; numd < numDes; numd++)
// 	{
// 	    cv::Mat mDes = cv::Mat(1, 32, CV_8U);
// 	    for(int r = 0; r < numRows; r++)
// 	    {
// 		int temp;
// 		for(int c = 0; c < numCols; c++)
// 		{desIn >> temp; mDes.at<unsigned char>(r, c) = temp;}
// 	    }
// 	    vDes.push_back(mDes);
// 	}
// 	vFeatureDescriptros.push_back(vDes);
// 	
// 	//Add Model Point
// 	cv::Mat x3D = Converter::toCvMat(vPos);
// 	MapPoint* pMP = new MapPoint(i, x3D, mindis, maxdis, norm, vDes, this);
// 
// 	this->AddModelPoint(pMP);
//     }
//     cout << "There are " << this->ModelPointsInMap() << " " << MapPoint::nNextId << " model points in map." << endl;
//     points_in.close();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    _local_cloud = pKF->pCloud;
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
