#ifndef JS_RBDL
#define JS_RBDL

#include <iostream>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class JS_DYNAMICS
{

public:
    Model* model;

    Body body1;
    Joint joint1;


public:
    JS_DYNAMICS()
    {

        model = new Model();
        model->gravity = Vector3d(0., 0., -9.81);

//        body1 = Body(0.132, Vector3d(0., 0., 0.13), Matrix3d(110.555*1e-5, 0., 0., 0.,110.755*1e-5, 0., 0., 0., 0.4*1e-5));
        body1 = Body(0.0, Vector3d(0., 0., 0.13), Matrix3d(0, 0., 0., 0., 0, 0., 0., 0., 0));

        joint1 = Joint(JointTypeRevoluteX);

        unsigned int body1_id;
        body1_id = model->AddBody(0, Xtrans(Vector3d(0.,0.,0.)), joint1, body1);

//        VectorNd Q = VectorNd::Zero (model->dof_count);
//        VectorNd QDot = VectorNd::Zero (model->dof_count);
//        VectorNd Tau = VectorNd::Zero (model->dof_count);
//        VectorNd QDDot = VectorNd::Zero (model->dof_count);
    }

    MatrixNd calc_est_torque(void);



    MatrixNd calc_M(VectorNd _Q)
    {
        MatrixNd _H(model->dof_count,model->dof_count);

        CompositeRigidBodyAlgorithm(*model,_Q,_H,1);

        return _H;
    }
};


#endif // JS_RBDL

