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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

MapDrawer::MapDrawer(Map* pMap, LidarMap* pLidarMap, const string& strSettingPath):mpMap(pMap), mpLidarMap(pLidarMap), mpCloud(0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    
    extM = Eigen::Matrix4d::Identity();
    extM << -0.0060, -0.0380, 0.9993, -0.202,
	    -1.0000, 0.0001, -0.0060, 0.03,
	    0.0001, -0.9993, -0.0380, 0.999,
	    0,       0,      0,      1;
}

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap), mpCloud(0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    
    extM = Eigen::Matrix4d::Identity();
//     extM << -0.0050, -1.0000, 0.0009, 0.0230,
// 	    -0.0370,  0.0011, 0.9993, 1.0000,
// 	    -0.9993,  0.0050,-0.0370, 0.1970,
// 	     0,       0,      0,      1;
    extM << -0.0060, -0.0380, 0.9993, -0.202,
	    -1.0000, 0.0001, -0.0060, 0.03,
	    0.0001, -0.9993, -0.0380, 0.999,
	    0,       0,      0,      1;

}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.902, 0.902, 0.980);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.255, 0.412, 0.882);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawLidarPoints()
{
    shared_ptr<DP> cloud_ = mpLidarMap->GetAllLidarMapPoints();
    
    if(!cloud_)
	return;
  
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.5,0.5,1.0);
    
    for(size_t i=0, iend=cloud_->features.cols(); i<iend;i++)
    {
	Eigen::Vector4d Pl(cloud_->features(0,i),
			   cloud_->features(1,i),
			   0,
			   1);
	glVertex3f(Pl[0], Pl[1], Pl[2]);
    }
    glEnd();
}

void MapDrawer::DrawAllLidarPoints()
{
  if(!mpCloud)
  {
    shared_ptr<PM::Transformation> transformation(PM::get().REG(Transformation).create("RigidTransformation"));
    
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    for(int i = 0; i < vpKFs.size(); i++)
    {
	KeyFrame* pKF = vpKFs[i];
	if(!mpCloud)
	{
	    shared_ptr<DP> refDP = pKF->pCloud;
	    DP newDP = *refDP;
	    Eigen::MatrixXf features_ = refDP->features;
	    Eigen::MatrixXf newFeatures(4, features_.cols());
	    newFeatures.setZero();
	    newFeatures.topRows<2>() = features_.topRows<2>();
	    newFeatures.bottomRows<1>().setOnes();
	    newDP.features = Converter::toMatrix4f(pKF->GetPoseInverse() * ConfigParam::GetMatT_co()) * newFeatures;
	    shared_ptr<DP> cloud_(new DP(newDP));
	    mpCloud = cloud_;
	    continue;
	}
	int tempMapPts = mpCloud->features.cols();
	const int readPtsCount(pKF->pCloud->features.cols());
	const int mapPtsCount(tempMapPts);
	
	PM::Matrix radius_reading = pKF->pCloud->features.topRows(3).colwise().norm();//1*cols的矩阵？，每个元素是每个点的距离

	PM::Matrix angles_reading(2, readPtsCount); // 0=inclination, 1=azimuth

	//构造newPointloud的angles_reading
	for(int k=0; k<readPtsCount; k++)
	{
		const float ratio = pKF->pCloud->features(2,k)/radius_reading(0,k);//z值和距离的比
		angles_reading(0,k) = acos(ratio);
		angles_reading(1,k) = atan2(pKF->pCloud->features(1,k), pKF->pCloud->features(0,k));//newPointCloud是输入的scanner坐标系下的点云
	}
	std::shared_ptr<NNS> featureNNS;
	featureNNS.reset( NNS::create(angles_reading));

// 	PM::TransformationParameters Ttemp = Converter::toMatrix4f(pKF->GetNavState());
// 	DP mapLocalFrame = transformation->compute(*mpCloud, Ttemp);
	
	shared_ptr<DP> refDP = mpCloud;
	DP mapLocalFrame = *refDP;
	Eigen::MatrixXf features_ = refDP->features;
	Eigen::MatrixXf newFeatures(4, features_.cols());
	newFeatures.setZero();
	newFeatures.topRows<2>() = features_.topRows<2>();
	newFeatures.bottomRows<1>().setOnes();
	mapLocalFrame.features = Converter::toMatrix4f(pKF->GetPose() * ConfigParam::GetMatToc()) * newFeatures;
	
	PM::Matrix globalId(1, mapPtsCount); //地图点数量  关联mapLocalFrameCut里面的点和地图点的index

	int mapCutPtsCount = 0;//有多少地图点在当前帧能看到的范围内
	DP mapLocalFrameCut(mapLocalFrame.createSimilarEmpty());//mapLocalFrame, 将map地图点换到sensor坐标系
    	float sensorMaxRange = 100.0;
	for (int k = 0; k < mapPtsCount; k++)	
	{
		if (mapLocalFrame.features.col(k).head(3).norm() < sensorMaxRange)
		{
			mapLocalFrameCut.setColFrom(mapCutPtsCount, mapLocalFrame, k);
			globalId(0,mapCutPtsCount) = k;//cut 里面的第i个点对应local里面的第j个点
			mapCutPtsCount++;
		}
	}
	mapLocalFrameCut.conservativeResize(mapCutPtsCount);//将后面的零元素resize()

	DP tmp_map = mapLocalFrameCut; 
	shared_ptr<DP> refDP2 = pKF->pCloud;
	DP tempDP = *refDP2;
	Eigen::MatrixXf features_2 = refDP2->features;
	Eigen::MatrixXf newFeatures2(4, features_2.cols());
	newFeatures2.setZero();
	newFeatures2.topRows<2>() = features_2.topRows<2>();
	newFeatures2.bottomRows<1>().setOnes();
	tempDP.features = newFeatures2;
	tmp_map.concatenate(tempDP);
	
// 	cout << " build and populate NNS" << endl;
	featureNNS.reset( NNS::create(tmp_map.features, tmp_map.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));	
	PM::Matches matches_overlap(
		Matches::Dists(1, readPtsCount),
		Matches::Ids(1, readPtsCount)
	);
	
	featureNNS->knn(tempDP.features, matches_overlap.ids, matches_overlap.dists, 1, 0);
	
	DP overlap(tempDP.createSimilarEmpty());
	DP no_overlap(tempDP.createSimilarEmpty());
	
	int ptsOut = 0;
	int ptsIn = 0;
	float minAddDist = 2.0;
	for (int k = 0; k < readPtsCount; ++k)
	{
		// TEST 将激光周围1m范围的点不加入地图中
		if ((matches_overlap.dists(k) > 0.01) && (tempDP.features.col(k).head(3).norm() > minAddDist))
		{
			no_overlap.setColFrom(ptsOut, tempDP, k);
			ptsOut++;
		}
		else
		{
			overlap.setColFrom(ptsIn, tempDP, k);
			ptsIn++;
		}
	}

	no_overlap.conservativeResize(ptsOut);
	overlap.conservativeResize(ptsIn);
	
	DP newDP1 = no_overlap;
	Eigen::MatrixXf features_1 = newDP1.features;
	Eigen::MatrixXf newFeatures1(4, features_1.cols());
	newFeatures1.setZero();
	newFeatures1.topRows<2>() = features_1.topRows<2>();
	newFeatures1.bottomRows<1>().setOnes();
	newDP1.features = Converter::toMatrix4f(pKF->GetPoseInverse() * ConfigParam::GetMatT_co()) * newFeatures1;
	mpCloud->concatenate(newDP1);
      
// 	KeyFrame* pKF = vpKFs[i];
// 	shared_ptr<DP> refDP = pKF->pCloud;
// 	DP newDP = *refDP;
// 	Eigen::MatrixXf features_ = refDP->features;
// 	Eigen::MatrixXf newFeatures(4, features_.cols());
// 	newFeatures.setZero();
// 	newFeatures.topRows<2>() = features_.topRows<2>();
// 	newFeatures.bottomRows<1>().setOnes();
// 	newDP.features = Converter::toMatrix4f(pKF->GetPoseInverse() * ConfigParam::GetMatT_co()) * newFeatures;
// 	if(!mpCloud)
// 	{
// 	    shared_ptr<DP> cloud_(new DP(newDP));
// 	    mpCloud = cloud_;
// 	    continue;
// 	}
// 	mpCloud->concatenate(newDP);
    }
  }
  
  glPointSize(mPointSize);
  glBegin(GL_POINTS);
  glColor3f(0.580, 0.000, 0.827);
  
  for(size_t i=0, iend=mpCloud->features.cols(); i<iend;i++)
  {
      Eigen::Vector4d Pl(mpCloud->features(0,i),
			  mpCloud->features(1,i),
			  0,
			  1);
      glVertex3f(Pl[0], Pl[1], Pl[2]);
  }
  glEnd();
}

void MapDrawer::SaveMap()
{
    //------------------ Save whole LM --------------------
    //-----------------------------------------------------
    {
	if(!mpCloud)
	{
	    cerr << "lidar map hasnt been initialized" << endl;
	    return;
	}
	// save lidar map points
	mpCloud->save("/home/doom/indoor_map/lidarmap.vtk");
    }
  
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //------------------ Save VM --------------------
    //-----------------------------------------------
//     {
// 	if(vpMPs.empty())
// 	{
// 	    cerr << "visual map is empty" << endl;
// 	    return;
// 	}
// 	
// 	// save visual map points
// 	ofstream f;
// 	f.open("/home/doom/indoor_map/visual_map/visualmap.txt");
// 	f << fixed;
// 	int totalNum = 0;
// 	vector<size_t> index;
// 	for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
// 	{
// 	    if(vpMPs[i]->isBad())
// 		continue;
// 	    
// 	    if(vpMPs[i]->Observations() < 10)
// 		continue;
// 	    
// 	    double minDis = vpMPs[i]->GetMinDistanceInvariance();
// 	    double maxDis = vpMPs[i]->GetMaxDistanceInvariance();
// 	    double diffDis = abs(maxDis - minDis);
// 	    if(diffDis > 10)
// 		continue;
// 	    
// 	    totalNum++;
// 	    index.push_back(i);
// 	}
// 	f << vpKFs.size() << endl;
// 	f << totalNum << endl;
// 	// visual map points format: x y z, num_Obs, [obs_id...], mindist, maxidst, scaleFactor, ScaleLevels, [normVec]
// 	for(size_t i=0, iend=index.size(); i<iend;i++)
// 	{
// 	    cv::Mat pos = vpMPs[index[i]]->GetWorldPos();
// 	    map<KeyFrame*, size_t> mObs = vpMPs[index[i]]->GetObservations();
// 	    f << setprecision(9) << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << " " << mObs.size();
// 	    for(map<KeyFrame*, size_t>::iterator it = mObs.begin(), ite = mObs.end(); it!=ite; it++)
// 	    {
// 		f << " " << it->first->mnId;
// 	    }
// 	    f << " " << vpMPs[index[i]]->GetMinDistanceInvariance() << " " << vpMPs[index[i]]->GetMaxDistanceInvariance();
// 	    cv::Mat normv = vpMPs[index[i]]->GetNormal();
// 	    f << " " << normv.at<float>(0,0) << " " << normv.at<float>(1,0) << " " << normv.at<float>(2,0); 
// 	    f << endl;
// 	}
// 	f.close();
// 	
// 	//------------------ Save Descriptor --------------------
// 	//-------------------------------------------------------
// 	stringstream ss;
// 	for(size_t i=0, iend=index.size(); i<iend;i++)
// 	{
// 	    cout << "saving " << i << "th Descriptor vector" << endl;
// // 	    cv::Mat mDes = vpMPs[index[i]]->GetDescriptor();
// 	    vector<cv::Mat> vDes = vpMPs[index[i]]->mvDescriptors;
// 	    ss.str("");
// 	    ss << "/home/doom/indoor_map/visual_map/des_";
// 	    ss << std::setfill ('0') << std::setw (5) << i << ".txt";
// 	    f.open(ss.str().c_str());
// 	    f << fixed;
// 	    
// 	    f << vDes.size() << endl;
// 	    for(int j = 0; j < vDes.size(); j++)
// 	    {
// 		cv::Mat mDes = vDes[j];
// 	    
// 		if(j == 0)
// 		  f << mDes.rows << " " << mDes.cols << endl;
// 		for(int r = 0; r < mDes.rows; r++)
// 		{
// 		    for(int c = 0; c < mDes.cols; c++)
// 		    {
//     // 		    cout << mDes.at<uchar>(r, c) << endl;
// 			f << (int)mDes.at<unsigned char>(r, c) << " ";
// 		    }
// 		    f << endl;
// 		}
// 	    }
// 	    
// 	    f.close();
// 	}
//     }
    
    {
	ofstream f;
	stringstream ss;
	
	//------------------ Save Num KF --------------------
	//---------------------------------------------------
	ss.str("");
	ss << "/home/doom/indoor_map/numKF.txt";
	f.open(ss.str().c_str());
	f << fixed;
	f << vpKFs.size() << endl;
	f.close();
	
	for(size_t i = 0, iend = vpKFs.size(); i < iend; i++)
	{
	    KeyFrame* pKF = vpKFs[i];
	  
	    //------------------ Save LM --------------------
	    //-----------------------------------------------
	    ss.str("");
	    ss << "/home/doom/indoor_map/lidar_map/lm_";
	    ss << std::setfill ('0') << std::setw (5) << i << ".vtk";
	    pKF->pCloud->save(ss.str().c_str());
	  
	    //------------------ Save KF --------------------
	    //-----------------------------------------------
	    cout << "saving " << i << "th KF" << endl;
	  
	    ss.str("");
	    ss << "/home/doom/indoor_map/kf/kf_";
	    ss << std::setfill ('0') << std::setw (5) << i << ".txt";
	    f.open(ss.str().c_str());
	    f << fixed;
	    
	    f << pKF->mnId << endl;
	    
	    //save pose and navi state
	    cv::Mat Twc = pKF->GetPoseInverse();
	    vector<float> qwc = Converter::toQuaternion(Twc.rowRange(0,3).colRange(0,3));
	    // x y z qw qx qy qz
	    f << setprecision(9) << Twc.at<float>(0,3) << " " << Twc.at<float>(1,3) << " " << Twc.at<float>(2,3) << " "
	      << qwc[3] << " " << qwc[0] << " " << qwc[1] << " " << qwc[2];
	    f << endl;
	    NavState Twl = pKF->GetNavState();
	    Eigen::Vector3d twl = Twl.Get_P();
	    Eigen::Quaterniond qwl = Twl.Get_R().unit_quaternion();
	    f << setprecision(9) << twl[0] << " " << twl[1] << " " << twl[2] << " "
	      << qwl.w() << " " << qwl.x() << " " << qwl.y() << " " << qwl.z();
	    f << endl;
	    
	    //save bow
	    DBoW2::BowVector _mBowVec = pKF->mBowVec;
	    int N = _mBowVec.size();
	    //output num of bow vec
	    f << N << endl;
	    DBoW2::BowVector::const_iterator vit;
	    for(vit = _mBowVec.begin(); vit != _mBowVec.end(); ++vit)
	    {
	      f << vit->first << " " << vit->second << endl;
	    }
	    
	    //save feat vec
	    DBoW2::FeatureVector _mFeatVec = pKF->mFeatVec;
	    int M = _mFeatVec.size();
	    //output num of feat vector
	    f << M << endl;
	    DBoW2::FeatureVector::const_iterator fit;
	    for(fit = _mFeatVec.begin(); fit != _mFeatVec.end(); ++fit)
	    {
		f << fit->first << " " << fit->second.size() << " ";
		for(int i = 0; i < fit->second.size(); i++)
		{
		    f << fit->second[i] << " ";
		}
		f << endl;
	    }
	    
	    //save map points
	    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
	    f << vpMapPointsKF.size() << endl;
	    for(int i = 0; i < vpMapPointsKF.size(); i++)
	    {
		MapPoint* pMP = vpMapPointsKF[i];
		if(!pMP || pMP->isBad())
		{
		    f << -1 << endl;
		}
		else
		{
		    cv::Mat pos = pMP->GetWorldPos();
		    map<KeyFrame*, size_t> mObs = pMP->GetObservations();
		    f << setprecision(9) << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2) << " " << mObs.size();
		    for(map<KeyFrame*, size_t>::iterator it = mObs.begin(), ite = mObs.end(); it!=ite; it++)
		    {
			f << " " << it->first->mnId;
		    }
		    f << " " << pMP->GetMinDistanceInvariance() << " " << pMP->GetMaxDistanceInvariance();
		    cv::Mat normv = pMP->GetNormal();
		    f << " " << normv.at<float>(0,0) << " " << normv.at<float>(1,0) << " " << normv.at<float>(2,0);
		    
		    //save Descriptor
		    cv::Mat mDes = pMP->GetDescriptor();
		    f << " " << mDes.rows << " " << mDes.cols << " ";
		    for(int r = 0; r < mDes.rows; r++)
		    {
			for(int c = 0; c < mDes.cols; c++)
			{
			    f << (int)mDes.at<unsigned char>(r, c) << " ";
			}
		    }
		    
		    f << endl;
		}
	    }
	    
	    //save KF Descriptor
	    cv::Mat mDesKF = pKF->mDescriptors;
	    f << mDesKF.rows << " " << mDesKF.cols << endl;
	    for(int r = 0; r < mDesKF.rows; r++)
	    {
		for(int c = 0; c < mDesKF.cols; c++)
		{
		    f << (int)mDesKF.at<unsigned char>(r, c) << " ";
		}
	    }
	    f << endl;
	    
	    //save KF key points
	    const vector<cv::KeyPoint> vKpt = pKF->mvKeysUn;
	    int numKpt = vKpt.size();
	    f << numKpt << endl;
	    for(int i = 0; i < numKpt; i++)
	    {
		cv::KeyPoint kpt = vKpt[i];
		f << kpt.angle << " ";
	    }
	    f << endl;
	    
	    f.close();
	}
    }
}

void MapDrawer::DrawMapL()
{
    shared_ptr<DP> tempDP = mpMap->_cloud;
    
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.000, 1.000, 1.000);
    
    for(size_t i=0, iend=tempDP->features.cols(); i<iend;i++)
    {
	Eigen::Vector4d Pl(tempDP->features(0,i),
			   tempDP->features(1,i),
			   0,
			   1);
	glVertex3f(Pl[0], Pl[1], Pl[2]);
    }
    
    glEnd();
    
}

void MapDrawer::DrawMapLocalL()
{
    if(!mpMap->_local_cloud)
      return;
    shared_ptr<DP> tempDP = mpMap->_local_cloud;
    
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.000, 0.412, 0.706);
    
    for(size_t i=0, iend=tempDP->features.cols(); i<iend;i++)
    {
	Eigen::Vector4d Pl(tempDP->features(0,i),
			   tempDP->features(1,i),
			   0,
			   1);
	glVertex3f(Pl[0], Pl[1], Pl[2]);
    }
    
    glEnd();
    
}

void MapDrawer::DrawMapV()
{
//     vector<Eigen::Vector3d> vpPoints = mpMap->mvPointsPos;
    std::vector<ModelKeyFrame> vMdKFs = mpMap->GetAllModelKFs();
        
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.580, 0.000, 0.827);
    
//     for(size_t i=0, iend=vpPoints.size(); i<iend;i++)
    for(vector<ModelKeyFrame>::iterator it = vMdKFs.begin(), ite = vMdKFs.end(); it!=ite; it++)
    {
	ModelKeyFrame mMdKF = *it;
	vector<MapPoint*> vMdMP = mMdKF.mvpModelPoints;
	for(MapPoint* it:vMdMP)
	{
	    if(!it)
	      continue;
	    cv::Mat mPos = it->GetWorldPos();
	    glVertex3f(mPos.at<float>(0), mPos.at<float>(1), mPos.at<float>(2));
	}
    }
    
    glEnd();
}

void MapDrawer::DrawMapK()
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

//     map<int, pair<int, KFPair>> mKFItems = mpMap->mKFItems;
    std::vector<ModelKeyFrame> vMdKFs = mpMap->GetAllModelKFs();
    int maxindex = mpMap->getMaxIndex();
//     for(map<int, pair<int, KFPair>>::iterator it = mKFItems.begin(), ite = mKFItems.end(); it!=ite; it++)
    for(vector<ModelKeyFrame>::iterator it = vMdKFs.begin(), ite = vMdKFs.end(); it!=ite; it++)
    {
// 	KFPair kfItem = it->second.second;
// 	cv::Mat kfPose = kfItem.Twc;
	ModelKeyFrame mMdKF = *it;

	glPushMatrix();

// 	glMultMatrixf(kfPose.ptr<GLfloat>(0));
	glMultMatrixf(mMdKF.Twc.ptr<GLfloat>(0));

	glLineWidth(mKeyFrameLineWidth);
	if(it->first != maxindex)
	  glColor3f(0.941, 0.502, 0.502);
	else
	  glColor3f(0, 1, 1);
	glBegin(GL_LINES);
	glVertex3f(0,0,0);
	glVertex3f(w,h,z);
	glVertex3f(0,0,0);
	glVertex3f(w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,-h,z);
	glVertex3f(0,0,0);
	glVertex3f(-w,h,z);

	glVertex3f(w,h,z);
	glVertex3f(w,-h,z);

	glVertex3f(-w,h,z);
	glVertex3f(-w,-h,z);

	glVertex3f(-w,h,z);
	glVertex3f(w,h,z);

	glVertex3f(-w,-h,z);
	glVertex3f(w,-h,z);
	glEnd();

	glPopMatrix();
    }
    
    glEnd();
}

void MapDrawer::DrawRay(pangolin::OpenGlMatrix &Twc)
{
    glLineWidth(mGraphLineWidth);
    glColor4f(1.000, 0.843, 0.000,0.6f);
    glBegin(GL_LINES);
    
    float x = Twc.m[12], y = Twc.m[13], z = Twc.m[14];
    vector<Eigen::Vector3d> vPtPos = mpMap->mvPointsPos;
    vector<int> vIndex = mpMap->vIndexProjFrameMapPt;
    for(size_t i=0; i < vIndex.size(); i++)
    {
	if(vIndex[i] == -1)
	  continue;
	Eigen::Vector3d pt = vPtPos[vIndex[i]];
	glVertex3f(x, y, z);
	glVertex3f(pt[0], pt[1], pt[2]);
    }
//     cout << endl;
    
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawModel)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    
    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
	  
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
	    
	    if(i == vpKFs.size() - 1)
	      pKF->DrawLidarPoints();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.196, 0.804, 0.196);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.878, 1.000, 1.000,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
    
    if(mpMap->hasGlobalMap() && bDrawModel)
    {
	glLineWidth(mGraphLineWidth);
        glColor4f(1.000, 0.843, 0.000,0.6f);
        glBegin(GL_LINES);
      
	for(size_t i=0; i<vpKFs.size(); i++)
        {
	    KeyFrame* pKF = vpKFs[i];
	    cv::Mat Ow = pKF->GetCameraCenter();
	    
	    vector<MapPoint*> vModelPt = pKF->mvpModelPoints;
	    for(MapPoint* it:vModelPt)
	    {
		if(!it)
		  continue;
		cv::Mat mPos = it->GetWorldPos();
		glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(mPos.at<float>(0),mPos.at<float>(1),mPos.at<float>(2));
	    }
	}
	glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(1.000, 0.271, 0.000);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::SetCurrentLidarPose(const cv::Mat& Tlw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mLidarPose = Tlw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

void MapDrawer::DrawLidarMotion(pangolin::OpenGlMatrix& Twl)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twl.m);
#else
        glMultMatrixd(Twl.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(1.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void MapDrawer::GetCurrentOpenGLLidarMatrix(pangolin::OpenGlMatrix& M)
{
    if(!mLidarPose.empty())
    {
        cv::Mat Rwl(3,3,CV_32F);
        cv::Mat twl(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwl = mLidarPose.rowRange(0,3).colRange(0,3);
            twl = mLidarPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwl.at<float>(0,0);
        M.m[1] = Rwl.at<float>(1,0);
        M.m[2] = Rwl.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwl.at<float>(0,1);
        M.m[5] = Rwl.at<float>(1,1);
        M.m[6] = Rwl.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwl.at<float>(0,2);
        M.m[9] = Rwl.at<float>(1,2);
        M.m[10] = Rwl.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twl.at<float>(0);
        M.m[13] = twl.at<float>(1);
        M.m[14] = twl.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
