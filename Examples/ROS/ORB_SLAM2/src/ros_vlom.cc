#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
	#include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "/home/doom/lidar_aid/ORB_SLAM2/include/System.h"

#include "/home/doom/lidar_aid/ORB_SLAM2/include/Odom/PMtypes.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"

#include<opencv2/core/core.hpp>

using namespace std;
using namespace PointMatcherSupport;

//typedef message_filters::sync_policies::ApproximateTime<
//	     sensor_msgs::Image, 
//	     sensor_msgs::Image
//	    ,sensor_msgs::LaserScan> sync_pol;

class Matcher
{
	    
public:
    Matcher(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){};
    
    void GrabStereoLidar(const sensor_msgs::ImageConstPtr& msgLeft, 
		  const sensor_msgs::ImageConstPtr& msgRight, 
		  const sensor_msgs::LaserScanConstPtr& msgLaser);
    
    ORB_SLAM2::System* mpSLAM;
  
    DP laserScanToPointMatcherCloud(const sensor_msgs::LaserScan& scanMsgIn);

};

DP Matcher::laserScanToPointMatcherCloud(const sensor_msgs::LaserScan& scanMsgIn)
{
    typedef typename DP::Label Label;
    typedef typename DP::Labels Labels;
    typedef typename DP::View View;

    Labels featLabels;
    featLabels.push_back(Label("x", 1));
    featLabels.push_back(Label("y", 1));
    featLabels.push_back(Label("pad", 1));

    // Build descriptors
    Labels descLabels;
    if (!scanMsgIn.intensities.empty())
    {
        descLabels.push_back(Label("intensity", 1));
        assert(scanMsgIn.intensities.size() == scanMsgIn.ranges.size());
    }

    // filter points based on range
    std::vector<size_t> ids(scanMsgIn.ranges.size());
    std::vector<double> ranges(scanMsgIn.ranges.size());
    std::vector<double> intensities(scanMsgIn.intensities.size());

    size_t goodCount(0);
    for (size_t i = 0; i < scanMsgIn.ranges.size(); ++i)
    {
        const float range(scanMsgIn.ranges[i]);
        if (range >= scanMsgIn.range_min && range <= scanMsgIn.range_max)
        {
            ranges[goodCount] = range;
            ids[goodCount] = i;
            if(!scanMsgIn.intensities.empty())
            {
                intensities[goodCount] = scanMsgIn.intensities[i];
            }
            ++goodCount;
        }
    }
    if (goodCount == 0)
        return PM::DataPoints();

    ids.resize(goodCount);
    ranges.resize(goodCount);
    if(!scanMsgIn.intensities.empty())
        intensities.resize(goodCount);

    DP cloud(featLabels, descLabels, goodCount);
    cloud.getFeatureViewByName("pad").setConstant(1);

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        float angle = scanMsgIn.angle_min + ids[i]*scanMsgIn.angle_increment;
        float range(ranges[i]);
        float x = cos(angle) * range;
        float y = sin(angle) * range;

        // write back
        cloud.features(0,i) = x;
        cloud.features(1,i) = y;
    }

    // fill descriptors
    if (!scanMsgIn.intensities.empty())
    {
        auto is(cloud.getDescriptorViewByName("intensity"));
        for (size_t i = 0; i < intensities.size(); ++i)
        {
            is(0,i) = intensities[i];
        }
    }

    //cerr << "point cloud:\n" << cloud.features.leftCols(10) << endl;
    return cloud;
}

void Matcher::GrabStereoLidar(const sensor_msgs::ImageConstPtr& msgLeft, 
			      const sensor_msgs::ImageConstPtr& msgRight, 
			      const sensor_msgs::LaserScanConstPtr& msgLaser)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }
    
    shared_ptr<DP> cloud(new DP(laserScanToPointMatcherCloud(*msgLaser)));
    
    mpSLAM->TrackStereoLidar(cv_ptrLeft->image,
			     cv_ptrRight->image,
			     cloud,
			     cv_ptrLeft->header.stamp.toSec());
    
}

// Main function supporting the Matcher class
int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "Vlom");
    ros::start();
    
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Vlom path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }  
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
        
    Matcher matcher(&SLAM);
    
    ros::NodeHandle nh;
    
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/wide_stereo/left/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/wide_stereo/right/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/base_scan", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,       sensor_msgs::Image, sensor_msgs::LaserScan> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub, laser_sub);
    sync.registerCallback(boost::bind(&Matcher::GrabStereoLidar,&matcher,_1,_2,_3));

    ros::spin();
    
    // Stop all threads
    SLAM.Shutdown();
    
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();
    
    return 0;
}
