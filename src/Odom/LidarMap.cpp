#include "Odom/LidarMap.h"

#include<mutex>

namespace ORB_SLAM2
{

bool KFComapre::operator ()(const KeyFrame* kfleft,const KeyFrame* kfright) const
{
    return kfleft->mnId < kfright->mnId;
}
  
LidarMap::LidarMap():mnMaxKFid(0),transformation(PM::get().REG(Transformation).create("RigidTransformation")), _cloud(0)
{
  std::ifstream filterIfs("/home/doom/my_ws/src/laser_icp/config/input_filter.yaml");
  if (filterIfs.good())
    {
	inputFilters = PM::DataPointsFilters(filterIfs);
    }
    else
    {
	std::cout << "Cannot load filter config from YAML file " 
	<< "/home/doom/my_ws/src/laser_icp/config/input_filter.yaml" << std::endl;
    }
}

void LidarMap::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    shared_ptr<DP> refDP = pKF->pCloud;
    DP newDP = *refDP;
    Eigen::MatrixXf features_ = refDP->features;
    Eigen::MatrixXf newFeatures(4, features_.cols());
    newFeatures.setZero();
    newFeatures.topRows<2>() = features_.topRows<2>();
    newFeatures.bottomRows<1>().setOnes();
    newDP.features = Converter::toMatrix4f(pKF->GetNavState()) * newFeatures;
    inputFilters.apply(newDP);
    if(!_cloud)
    {
	shared_ptr<DP> cloud_(new DP(newDP));
	_cloud = cloud_;
	return;
    }
    _cloud->concatenate(newDP);
}

void LidarMap::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

vector<KeyFrame*> LidarMap::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

shared_ptr< DP > LidarMap::GetAllLidarMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return _cloud;
}

long unsigned int LidarMap::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return _cloud->features.cols();
}

long unsigned int LidarMap::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

long unsigned int LidarMap::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void LidarMap::clear()
{
    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspKeyFrames.clear();
    mnMaxKFid = 0;
    
    _cloud.reset();
}

} //namespace ORB_SLAM
