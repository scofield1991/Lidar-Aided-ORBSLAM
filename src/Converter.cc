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


#include "Converter.h"

namespace ORB_SLAM2
{

void Converter::updateNS(NavState& ns, const ICPer& icper)
{
    Matrix3d dR = icper.getDeltaR();
    Vector3d dP = icper.getDeltaP();
    
    Vector3d Pwbpre = ns.Get_P();
    Matrix3d Rwbpre = ns.Get_RotMatrix();
    
    Matrix3d Rwb = Rwbpre * dR;
    Vector3d Pwb = Pwbpre + Rwbpre*dP;

    // Here assume that the pre-integration is re-computed after bias updated, so the bias term is ignored
    ns.Set_Pos(Pwb);
    ns.Set_Rot(Rwb);

}

cv::Mat Converter::toCvMat(const NavState& ns)
{
    Eigen::Matrix3d R = ns.Get_RotMatrix();
    Eigen::Vector3d t = ns.Get_P();
    cv::Mat res(4, 4, CV_32F);
    res(cv::Rect(0,0,3,3)) = Converter::toCvMat(R);
    res(cv::Rect(0,3,3,1)) = Converter::toCvMat(t);
    return res;
}

cv::Mat Converter::toCvMatInverse(const cv::Mat &Tcw)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc*tcw;

    cv::Mat Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    twc.copyTo(Twc.rowRange(0,3).col(3));

    return Twc.clone();
}

NavState Converter::toNavState(const Eigen::Matrix4d& Teig)
{
    Eigen::Vector3d p(Teig(0,3), Teig(1,3), Teig(2,3));
    Eigen::Matrix3d R = Teig.topLeftCorner(3,3);
    NavState result;
    result.Set_Pos(p);
    result.Set_Rot(R);
    return result;
}

NavState Converter::toSE3NS(const cv::Mat& SE3)
{
    cv::Mat R_ = SE3(cv::Rect(0,0,3,3));
    cv::Mat t_ = SE3(cv::Rect(0,3,3,1));
    Eigen::Matrix3d Reig = Converter::toMatrix3d(R_);
    Eigen::Vector3d teig = Converter::toVector3d(t_);
    NavState res;
    res.Set_Pos(teig);
    res.Set_Rot(Reig);
    return res;
}

PointMatcher<float>::TransformationParameters Converter::toPMT(const NavState& ns)
{
    PM::TransformationParameters res = PM::TransformationParameters::Identity(4, 4);
    Eigen::Matrix3d R = ns.Get_RotMatrix();
    Eigen::Vector3d t = ns.Get_P();
    res(0,0) = R(0,0); res(0,1) = R(0,1); res(1,0) = R(1,0); res(1,1) = R(1,1);
    res(0,2) = t[0]; res(1,2) = t[1];
    return res;
}
  
Eigen::Matrix4f Converter::toMatrix4f(const NavState& ns)
{
    Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
    Eigen::Matrix3d R = ns.Get_RotMatrix();
    Eigen::Vector3d t = ns.Get_P();
    res.topLeftCorner(3,3) = R.cast<float>();
    res.topRightCorner(3,1) = t.cast<float>();
    return res;
}

Eigen::Matrix4f Converter::toMatrix4f(const cv::Mat& cvMat)
{
    Eigen::Matrix4f res = Eigen::Matrix4f::Identity();
    cv::Mat R = cvMat(cv::Rect(0,0,3,3));
    cv::Mat t = cvMat.col(3).rowRange(0,2);
    res.topLeftCorner(3,3) = Converter::toMatrix3d(R).cast<float>();
    res.topRightCorner(3,1) = Converter::toVector3d(t).cast<float>();
    return res;
}
  
std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

} //namespace ORB_SLAM
