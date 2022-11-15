#ifndef OW_ONEDOFMPC_H
#define OW_ONEDOFMPC_H
#include "QuadProg++.hh"
#include "rbdl/rbdl.h"
#include "BasicMatrix.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
class ow_onedofMPC
{
private:
    int N;
    double dt;
    double flyT;
    double maxu;
    MatrixNd A,B,C,D,ZD;
    MatrixNd An[30];
    MatrixNd Afly;
    MatrixNd R,Q;
    MatrixNd H,F,A_eq,B_eq,A_ineq,B_ineq;
    MatrixNd V,v, ref_now,xnow;
public:
    ow_onedofMPC(int _N, double _dt, double _maxu);

    double getddx(vec3 x0, std::vector<vec3> ref, int stanceN, double _flyT,
                  double _R, double _Qx, double _Qv, double _maxu);
};

#endif // OW_ONEDOFMPC_H
