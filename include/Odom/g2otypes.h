#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include "Odom/NavState.h"
#include "Odom/SophusType.h"
#include "Odom/ICPer.h"
// #include "Odom/OdomPreintegrator.h"
//#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"

namespace g2o
{

using namespace ORB_SLAM2;

/**
 * @brief The VertexNavStatePVR class
 */
class VertexNavStatePR : public BaseVertex<6, NavState>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexNavStatePR() : BaseVertex<6, NavState>(){}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    virtual void setToOriginImpl() {
      _estimate = NavState();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);
        _estimate.IncSmall(update);
    }

};

class EdgeNavStatePR : public BaseBinaryEdge<6, NavState, VertexNavStatePR, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePR() : BaseBinaryEdge<6, NavState, VertexNavStatePR, VertexNavStatePR>() {}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError();

//     virtual void linearizeOplus();

};

class EdgeNavStatePRStereoPointXYZ : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePRStereoPointXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexNavStatePR>() {}

    bool read(std::istream& is) {return true;}

    bool write(std::ostream& os) const {return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector3d obs(_measurement);

        _error = obs - cam_project(Pc, bf);
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexSBAPointXYZ* vPoint = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexNavStatePR* vNavState = static_cast<const VertexNavStatePR*>(_vertices[1]);

        const NavState& ns = vNavState->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbo.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbo;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }

    Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const {
      const float invz = 1.0f/trans_xyz[2];
      Vector3d res;
      res[0] = trans_xyz[0]*invz*fx + cx;
      res[1] = trans_xyz[1]*invz*fy + cy;
      res[2] = res[0] - bf*invz;
      return res;
    }

    //
//     virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_, const double& bf_,
                   const Matrix3d& Rbo_, const Vector3d& Pbo_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
	bf = bf_;
        Rbo = Rbo_;
        Pbo = Pbo_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy, bf;
    // Camera-Odom extrinsics
    Matrix3d Rbo;
    Vector3d Pbo;
};

class EdgeNavStatePRStereoPointXYZOnlyPose : public BaseUnaryEdge<3, Vector3d, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePRStereoPointXYZOnlyPose(){}

    bool read(std::istream& is){return true;}

    bool write(std::ostream& os) const{return true;}

    void computeError() {
        Vector3d Pc = computePc();
        Vector3d obs(_measurement);

        _error = obs - cam_project(Pc, bf);
    }

    bool isDepthPositive() {
        Vector3d Pc = computePc();
        return Pc(2)>0.0;
    }

    Vector3d computePc() {
        const VertexNavStatePR* vNSPVR = static_cast<const VertexNavStatePR*>(_vertices[0]);

        const NavState& ns = vNSPVR->estimate();
        Matrix3d Rwb = ns.Get_RotMatrix();
        Vector3d Pwb = ns.Get_P();
        //const Vector3d& Pw = vPoint->estimate();

        Matrix3d Rcb = Rbo.transpose();
        Vector3d Pc = Rcb * Rwb.transpose() * (Pw - Pwb) - Rcb * Pbo;

        return Pc;
        //Vector3d Pwc = Rwb*Pbc + Pwb;
        //Matrix3d Rcw = (Rwb*Rbc).transpose();
        //Vector3d Pcw = -Rcw*Pwc;
        //Vector3d Pc = Rcw*Pw + Pcw;
    }
    
    Vector3d cam_project(const Vector3d & trans_xyz, const float &bf) const {
      const float invz = 1.0f/trans_xyz[2];
      Vector3d res;
      res[0] = trans_xyz[0]*invz*fx + cx;
      res[1] = trans_xyz[1]*invz*fy + cy;
      res[2] = res[0] - bf*invz;
      return res;
    }

    //
//     virtual void linearizeOplus();

    void SetParams(const double& fx_, const double& fy_, const double& cx_, const double& cy_, const double& bf_,
                   const Matrix3d& Rbo_, const Vector3d& Pbo_, const Vector3d& Pw_) {
        fx = fx_;
        fy = fy_;
        cx = cx_;
        cy = cy_;
	bf = bf_;
        Rbo = Rbo_;
        Pbo = Pbo_;
        Pw = Pw_;
    }

protected:
    // Camera intrinsics
    double fx, fy, cx, cy, bf;
    // Camera-Odom extrinsics
    Matrix3d Rbo;
    Vector3d Pbo;
    // Point position in world frame
    Vector3d Pw;
};

/**
 * @brief The EdgeNavStatePrior class
 */

class EdgeNavStatePrior : public BaseUnaryEdge<6, NavState, VertexNavStatePR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeNavStatePrior(){}

    bool read(std::istream &is){return true;}

    bool write(std::ostream &os) const{return true;}

    void computeError();

//     virtual void linearizeOplus();

};

}

#endif // G2OTYPES_H
