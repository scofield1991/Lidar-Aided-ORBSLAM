#include "Odom/NavState.h"

namespace ORB_SLAM2
{

NavState::NavState()
{
    //_qR.setIdentity();     // rotation
    _P.setZero();         // position
}

// if there's some other constructor, normalizeRotation() is needed
NavState::NavState(const NavState &_ns):
    _P(_ns._P), _R(_ns._R)
{
    //normalizeRotation();
}

void NavState::IncSmall(Vector6d update)
{

    Vector3d upd_P = update.segment<3>(0);
    Vector3d upd_Phi = update.segment<3>(3);

    // rotation matrix before update
    //Matrix3d R = Get_qR().toRotationMatrix();
    Matrix3d R = Get_R().matrix();

    // position
    _P += R * upd_P;
    // rotation
    SO3Type dR = SO3Type::exp(upd_Phi);
    _R = Get_R()*dR;

}

void NavState::IncSmall(NavState& dT)
{
    Vector3d upd_P = dT.Get_P();
    SO3Type  upd_R = dT.Get_R();
    
    Matrix3d R = Get_R().matrix();
    
    // position
    _P += R * upd_P;
    // rotation
    _R = Get_R()*upd_R;
}

}
