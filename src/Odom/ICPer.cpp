#include "Odom/ICPer.h"

namespace ORB_SLAM2
{
ICPer::ICPer():
minPointsCount(50),
transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
    std::ifstream icpIfs("/home/doom/my_ws/src/laser_icp/config/icp.yaml");
    std::ifstream filterIfs("/home/doom/my_ws/src/laser_icp/config/input_filter.yaml");
    if (icpIfs.good())
    {
	icp.loadFromYaml(icpIfs);
    }
    else
    {
	std::cout << "Cannot load ICP config from YAML file " 
	<< "/home/doom/my_ws/src/laser_icp/config/icp.yaml" << std::endl;
	icp.setDefault();
    }
    
    if (filterIfs.good())
    {
	inputFilters = PM::DataPointsFilters(filterIfs);
    }
    else
    {
	std::cout << "Cannot load filter config from YAML file " 
	<< "/home/doom/my_ws/src/laser_icp/config/input_filter.yaml" << std::endl;
    }
    
    extrinsicMat = Eigen::Matrix4d::Identity();
//     extrinsicMat << -0.0050, -1.0000, 0.0009, 0.0230,
// 		    -0.0370,  0.0011, 0.9993, 1.0000,
// 		    -0.9993,  0.0050,-0.0370, 0.1970,
// 		          0,       0,      0,      1;
    extrinsicMat << -0.0060, -0.0380, 0.9993, -0.202,
		    -1.0000, 0.0001, -0.0060, 0.03,
		     0.0001, -0.9993, -0.0380, 0.999,
		          0,       0,      0,      1;
}
  
ICPer::ICPer(const string& icpConfig, const string& filterConfig):
minPointsCount(50),
transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
    std::ifstream icpIfs(icpConfig.c_str());
    std::ifstream filterIfs(filterConfig.c_str());
    if (icpIfs.good())
    {
	icp.loadFromYaml(icpIfs);
    }
    else
    {
	std::cout << "Cannot load ICP config from YAML file " 
	<< icpConfig << std::endl;
	icp.setDefault();
    }
    
    if (filterIfs.good())
    {
	inputFilters = PM::DataPointsFilters(filterIfs);
    }
    else
    {
	std::cout << "Cannot load filter config from YAML file " 
	<< filterConfig << std::endl;
    }
    
    extrinsicMat = Eigen::Matrix4d::Identity();
//     extrinsicMat << -0.0050, -1.0000, 0.0009, 0.0230,
// 		    -0.0370,  0.0011, 0.9993, 1.0000,
// 		    -0.9993,  0.0050,-0.0370, 0.1970,
// 		          0,       0,      0,      1;
    extrinsicMat << -0.0060, -0.0380, 0.9993, -0.202,
		    -1.0000, 0.0001, -0.0060, 0.03,
		     0.0001, -0.9993, -0.0380, 0.999,
		          0,       0,      0,      1;
}

ICPer::ICPer(const ICPer& other):
_delta_P(other._delta_P),
_delta_R(other._delta_R),
_cov_P_Phi(other._cov_P_Phi),
minPointsCount(other.minPointsCount),
transformation(other.transformation),
icp(other.icp),
inputFilters(other.inputFilters),
extrinsicMat(other.extrinsicMat)
{
}

void ICPer::reset()
{
    // delta measurements, position/velocity/rotation(matrix)
    _delta_P.setZero();    // P_k+1 = P_k + V_k*dt + R_k*a_k*dt*dt/2
    _delta_R.setIdentity();    // R_k+1 = R_k*exp(w_k*dt).     note: Rwc, Rwc'=Rwc*[w_body]x

    // noise covariance propagation of delta measurements
    _cov_P_Phi.setZero();

}

void ICPer::process(shared_ptr< DP > refPointCloud, 
		    shared_ptr< DP > newPointCloud)
{ 
    // Dimension of the point cloud, important since we handle 2D and 3D
    const int dimp1(newPointCloud->features.rows());

    // Convert point cloud
    const int initialPtsCount = newPointCloud->features.cols();
    if (initialPtsCount < minPointsCount)
    {
        std::cout << "No enough points in newPointCloud: only " 
		  << initialPtsCount << " pts." << std::endl;
        return;
    }

    // input filter on the new cloud
    inputFilters.apply(*refPointCloud);
    inputFilters.apply(*newPointCloud);
    // Ensure a minimum amount of point after filtering
    const int ptsCount = newPointCloud->features.cols();
    if(ptsCount < minPointsCount)
    {
        std::cout << "No enough points in newPointCloud after input filter: only " 
		  << ptsCount << " pts." << std::endl;
        return;
    }

    icp.setMap(*refPointCloud);
    
    // Check dimension
    if (newPointCloud->features.rows() != icp.getInternalMap().features.rows())
    {
        std::cout << "Dimensionality missmatch: incoming cloud is " << newPointCloud->features.rows()-1
		  << " while map is " << icp.getInternalMap().features.rows()-1 << std::endl;
        return;
    }

    PM::TransformationParameters priorTSensorRelative, posteriorTSensorRelative;
    priorTSensorRelative = posteriorTSensorRelative  = PM::TransformationParameters::Identity(dimp1, dimp1);
    
    try
    {
//         std::cout << "calculating icp" << std::endl;
        posteriorTSensorRelative = icp(*newPointCloud, priorTSensorRelative);

//         std::cout << "Ticp:\n" << posteriorTSensorRelative
// 		  << std::endl;

	// read the last match information
        int matchNum = icp.errorMinimizer->lastErrorElements.matches.ids.cols();
        Eigen::MatrixXi indices(1, matchNum);
        indices = icp.errorMinimizer->lastErrorElements.matches.ids;
	
        Eigen::MatrixXf weights(1, matchNum);
        weights = icp.errorMinimizer->lastErrorElements.weights;
        // ref and new pc in the ICP
        DP refInICP = icp.errorMinimizer->lastErrorElements.reference;
        DP newInICP = icp.errorMinimizer->lastErrorElements.reading;
        PM::TransformationParameters TRefMean = icp.T_refIn_refMean;
        refInICP = transformation->compute(refInICP, TRefMean);
        newInICP = transformation->compute(newInICP, TRefMean);
	
        DP::View viewOnnormals= refInICP.getDescriptorViewByName("normals");
	
	float Xref, Yref, Xnew, Ynew;
        float Xt = posteriorTSensorRelative(0, 2), Yt = posteriorTSensorRelative(1, 2);
        float sinT = posteriorTSensorRelative(0, 1), cosT = posteriorTSensorRelative(0, 0);
	
	double x = Xt, y = Yt, a = atan(double(sinT/cosT));
	double z = 0, b = 0, c = 0;
	
	Eigen::Vector3d trans(Xt, Yt, 0);
	_delta_P = trans;
	Eigen::Matrix3d rota = Eigen::Matrix3d::Identity();
	rota(0,0) = cosT; rota(0,1) = sinT;
	rota(1,0) = -sinT; rota(1,1) = cosT;
	_delta_R = rota;
	
        //TODO calculate COV
	Eigen::MatrixXd d2J_dX2(3,3);
	d2J_dX2 = Eigen::MatrixXd::Zero(3,3);
	for(int i = 0 ; i < matchNum ; i++)
        {
            const Eigen::Vector3f normal_ref = viewOnnormals.col(i);
	    
	    double pix = refInICP.features(0,i);
	    double piy = refInICP.features(1,i);
	    
	    double qix = newInICP.features(0,i);
	    double qiy = newInICP.features(1,i);

	    double nix = normal_ref[0];
	    double niy = normal_ref[1];
	    
	    if (niy!=niy)// for nan removal in the input point cloud data:)
	      continue;
	    
	    double 	d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dcdx,
			d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
			d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
			d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
			d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
			d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;

        d2J_dx2 =

                2*pow(nix,2);


        d2J_dy2 =

                2*pow(niy,2);


        d2J_dydx =

                2*nix*niy;


        d2J_dxdy =

                2*nix*niy;

        d2J_da2 =

                (2*nix*(pix*cos(a) - piy*sin(a)) + 2*niy*(piy*cos(a) + pix*sin(a)))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a))) + (nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)))*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));

        d2J_dxda =

                -2*nix*(nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)));


        d2J_dadx =

                -nix*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));


        d2J_dyda =

                -2*niy*(nix*(piy*cos(a) + pix*sin(a)) - niy*(pix*cos(a) - piy*sin(a)));


        d2J_dady =

                -niy*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));
		
	  Eigen::MatrixXd d2J_dX2_temp(3,3);
	  d2J_dX2_temp << d2J_dx2,  d2J_dydx, d2J_dadx,
			  d2J_dxdy, d2J_dy2, d2J_dady,
			  d2J_dxda, d2J_dyda, d2J_da2;
			  
	  d2J_dX2 = d2J_dX2 + d2J_dX2_temp;
	}
	
	if (matchNum > 200) matchNum = 200;
	
	Eigen::MatrixXd d2J_dZdX(3,4*matchNum);
	
	for(int i = 0 ; i < matchNum ; i++)
        {
            const Eigen::Vector3f normal_ref = viewOnnormals.col(i);
	    
	    double pix = refInICP.features(0,i);
	    double piy = refInICP.features(1,i);
	    
	    double qix = newInICP.features(0,i);
	    double qiy = newInICP.features(1,i);

	    double nix = normal_ref[0];
	    double niy = normal_ref[1];
	    
	    if (niy!=niy)// for nan removal in the input point cloud data:)
	      continue;

	Eigen::MatrixXd d2J_dZdX_temp(3,4);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                d2J_dpix_dz,    d2J_dpiy_dz,    d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                d2J_dpix_db,    d2J_dpiy_db,    d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                d2J_dpix_dc,    d2J_dpiy_dc,    d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;


        d2J_dpix_dx =

                2*nix*(nix*cos(a) + niy*sin(a));


        d2J_dpix_dy =

                2*niy*(nix*cos(a) + niy*sin(a));

        d2J_dpix_da =

                - (2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)))*(nix*cos(a) + niy*sin(a)) - (2*niy*cos(a) - 2*nix*sin(a))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a)));

        d2J_dpiy_dx =

                2*nix*(niy*cos(a) - nix*sin(a));


        d2J_dpiy_dy =

                2*niy*(niy*cos(a) - nix*sin(a));

        d2J_dpiy_da =

                (2*nix*cos(a) + 2*niy*sin(a))*(nix*(qix - x - pix*cos(a) + piy*sin(a)) - niy*(y - qiy + piy*cos(a) + pix*sin(a))) - (2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)))*(niy*cos(a) - nix*sin(a));

        d2J_dqix_dx =

                -2*pow(nix,2);


        d2J_dqix_dy =

                -2*nix*niy;

        d2J_dqix_da =

                nix*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));

        d2J_dqiy_dx =

                -2*nix*niy;


        d2J_dqiy_dy =

                -2*pow(niy,2);

        d2J_dqiy_da =

                niy*(2*nix*(piy*cos(a) + pix*sin(a)) - 2*niy*(pix*cos(a) - piy*sin(a)));

	d2J_dZdX_temp << d2J_dpix_dx, d2J_dpiy_dx, d2J_dqix_dx, d2J_dqiy_dx,
			 d2J_dpix_dy, d2J_dpiy_dy, d2J_dqix_dy, d2J_dqiy_dy,
			 d2J_dpix_da, d2J_dpiy_da, d2J_dqix_da, d2J_dqiy_da;

        d2J_dZdX.block<3,4>(0,4*i) = d2J_dZdX_temp;
	}
	Eigen::MatrixXd cov_z(4*matchNum,4*matchNum);
	cov_z = 0.1 * Eigen::MatrixXd::Identity(4*matchNum,4*matchNum);
	
	Eigen::MatrixXd P = d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();
	
	_cov_P_Phi = Eigen::Matrix<double, 6, 6>::Identity()*9999999;
	_cov_P_Phi.block<2,2>(0,0) = P.block<2,2>(0,0);
	_cov_P_Phi(5,0) = P(2,0); _cov_P_Phi(5,1) = P(2,1);
	_cov_P_Phi(0,5) = P(0,2); _cov_P_Phi(1,5) = P(1,2);
	_cov_P_Phi(5,5) = P(2,2); 
	_cov_P_Phi(3,3) = 0.001; _cov_P_Phi(4,4) = 0.001;
    }
    catch (PM::ConvergenceError error)
    {
        std::cout << "ICP failed to converge: " << error.what() << std::endl;
        return;
    }
}
}