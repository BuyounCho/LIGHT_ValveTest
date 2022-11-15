#include "BasicFiles/BasicJoint.h"
#include "ManualCAN.h"
#include "LIGHT_var_and_func.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern pUSER_SHM               userData;
extern JointControlClass       *jCon;
extern int                     PODO_NO;
extern INFO_LIGHT LIGHT_Info;

double LPF_1st(double x_old, double x_new, double f_cut);
double LPF_2nd(double x_oold, double x_old, double x_new, double f_cut);
VectorNd Vector_LPF(VectorNd v, VectorNd v_new, double f_cut);
VectorNd Vector_LPF_2nd(VectorNd voo, VectorNd vo, VectorNd v_new, double f_cut);

void PrintHere(int n = 0);

// ////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////
// //////////////////////   TASK FUNCTION   ///////////////////////////
// ////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////


//==============================//
// Etc Functions
//==============================//

Matrix3d ExtractRotZ(Matrix3d _R)
{
    return RotZ(atan2(_R(1,0), _R(0,0)));
}

Quaternion Quaterion_multiplication(Quaternion A, Quaternion B)
{
    double s0,s1;
    Vector3d v0,v1;

    s0 = A(3); s1 = B(3);
    v0(0) = A(0); v0(1) = A(1); v0(2) = A(2);
    v1(0) = B(0); v1(1) = B(1); v1(2) = B(2);

    Vector3d v_r = s0*v1+s1*v0+v0.cross(v1);
    double s_r = s0*s1-v0.dot(v1);

    return Quaternion(v_r(0), v_r(1), v_r(2), s_r);
}


//// To find orientation of ordered triplet (p, q, r).
//// The function returns following values
//// 0 --> p, q and r are colinear
//// 1 --> Clockwise
//// 2 --> Counterclockwise
//int orientation(Vector3d p, Vector3d q, Vector3d r)
//{
//    double val = (q(1) - p(1)) * (r(0) - q(0)) -
//            (q(0) - p(0)) * (r(1) - q(1));
//    double val2 = (q(0) - p(0)) * (r(0) - q(0)) +
//            (q(1) - p(1)) * (r(1) - q(1));

//    if (val == 0 && val2 > 0) return 0;
//    else if (val == 0 && val2 < 0) return 2;
//    else if (val < 0) return 2;
//    else return 1;
//}

//// Prints convex hull of a set of n points.
//void convexHull(Vector3d points[], int n, MatrixNd& _A, VectorNd& _B)
//{
//    // There must be at least 3 points
//    if (n < 3) return;

//    // Initialize Result
//    vector<Vector3d> hull;

//    // Find the leftmost point
//    int l = 0;
//    for (int i = 1; i < n; i++)
//        if (points[i](0) < points[l](0))
//            l = i;


//    // Start from leftmost point, keep moving counterclockwise
//    // until reach the start point again.  This loop runs O(h)
//    // times where h is number of points in result or output.
//    int p = l, q;
//    do
//    {
//        // Add current point to result
//        hull.push_back(points[p]);

//        // Search for a point 'q' such that orientation(p, x,
//        // q) is counterclockwise for all points 'x'. The idea
//        // is to keep track of last visited most counterclock-
//        // wise point in q. If any point 'i' is more counterclock-
//        // wise than q, then update q.
//        q = (p+1)%n;

//        for (int i = 0; i < n; i++)
//        {
//           // If i is more counterclockwise than current q, then
//           // update q
//           if (orientation(points[p], points[i], points[q]) == 2) {
//               q = i;
//           }
//        }

//        // Now q is the most counterclockwise with respect to p
//        // Set p as q for next iteration, so that q is added to
//        // result 'hull'
//        p = q;

//    } while (p != l);  // While we don't come to first point

//    // Output : Matrix A and Vector B of Ax<B
//    _A.resize(hull.size(),3);
//    _B.resize(hull.size(),1);

//    for(int i=0;i<hull.size();i++) {
//        Vector3d V1 = hull[i];
//        Vector3d V2 = hull[(i+1)%hull.size()];
//        double val = V1(0)*V2(1) - V2(0)*V1(1);

//        if(fabs(val) < 1e-6) {
//            _A.row(i) << 1.0, -V1(0)/V1(1), 0.0;
//            _B(i) = 0.0;
//        } else {
//            _A.row(i) << (V2(1)-V1(1))/val, -(V2(0)-V1(0))/val, 0.0;
//            _B(i) = 1.0;
//        }

//        Vector3d V3 = hull[(i+2)%hull.size()];
//        VectorNd mat = _A.row(i)*V3;

//        if(mat(0) > _B(i))
//        {
//            _A.row(i) = -_A.row(i);
//            _B(i) = -_B(i);
//        }

//    }

//    // Print Result
//    cout << "[Outside Points]" << endl;
//    for (int i = 0; i < hull.size(); i++) {
//        cout << "(" << hull[i](0) << ", " << hull[i](1) << ")" << endl;
//    }

//    // Print Result
//    cout << "A : " << endl << _A << endl << "B : " << endl << _B << endl;
//}


typedef struct Point
{
    int x, y;
}Points;
Point p0;

void swap(Point *v1, Point *v2)
{
    Point temp = *v1;
    *v1 = *v2;
    *v2 = temp;
}
int orientation(Point p, Point q, Point r)
{
    int val = (int)(q.y - p.y) * (r.x - q.x) - ( int)(q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;
    return (val > 0)? 1: 2;
}
int distSq(Point p1, Point p2)
{
    return (int)(p1.x - p2.x)*(p1.x - p2.x) + ( int)(p1.y - p2.y)*(p1.y     - p2.y);
}
int compare(const void *vp1, const void *vp2)
{
    Point *p1 = (Point *)vp1;
    Point *p2 = (Point *)vp2;


    int o = orientation(p0, *p1, *p2);
    if (o == 0)
        return (distSq(p0, *p2) >= distSq(p0, *p1))? -1 : 1;

    return (o == 2)? -1: 1;
}
Point* Convex_Hull(Point *v, int *count)
{
    int n = *count, ymin = v[0].y, min = 0, i,m;
    Point *stack;
    for(i = 1; i < n; i++)
    {
        if((v[i].y < ymin) || ((v[i].y == ymin) && (v[i].x < v[min].x)))
        {
            ymin = v[i].y;
            min = i;
        }
    }
    swap(&v[0], &v[min]);
    p0 = v[0];
    if(n > 1)
        qsort(&v[1], n - 1, sizeof(Point), compare);
    m = 1;
    for(i = 1; i < n; i++)
    {
        while((i < n - 1) && orientation(v[0], v[i], v[i + 1]) == 0)
            i++;
        v[m++] = v[i];
    }
    *count = n = m;
    if(n < 3)
        return v;
    stack = (Point *)malloc(n * sizeof(Point));
    stack[0] = v[0];
    stack[1] = v[1];
    stack[2] = v[2];
    m = 2;
    for(i = 3; i < n; i++)
    {
        while(orientation(stack[m-1], stack[m], v[i]) != 2)
            m--;
        stack[++m] = v[i];
    }
    *count = n = ++m;
    free(v);
    v = NULL;
    return stack;
}

void Get_SupportConvexHull(int StanceLeg, Vector3d StancePosition, double StanceZAngle,
                           bool StepOn, Vector3d StepPosition, double StepZAngle,
                           MatrixNd& _A, VectorNd& _B)
{
    Vector3d FootPoints[8];

    double ZMPx_RF_min = -LIGHT_Info.footlimit_heel;
    double ZMPx_RF_max = LIGHT_Info.footlimit_toe;
    double ZMPy_RF_min = -LIGHT_Info.footlimit_outside;
    double ZMPy_RF_max = LIGHT_Info.footlimit_inside;
    double ZMPx_LF_min = -LIGHT_Info.footlimit_heel;
    double ZMPx_LF_max = LIGHT_Info.footlimit_toe;
    double ZMPy_LF_min = -LIGHT_Info.footlimit_inside;
    double ZMPy_LF_max = LIGHT_Info.footlimit_outside;

    // Stance Foot's Points
    if(StanceLeg == RIGHTLEG) {
        FootPoints[0] << ZMPx_RF_max,ZMPy_RF_max,0.0;
        FootPoints[1] << ZMPx_RF_max,ZMPy_RF_min,0.0;
        FootPoints[2] << ZMPx_RF_min,ZMPy_RF_min,0.0;
        FootPoints[3] << ZMPx_RF_min,ZMPy_RF_max,0.0;
    } else if(StanceLeg == LEFTLEG) {
        FootPoints[0] << ZMPx_LF_max,ZMPy_LF_max,0.0;
        FootPoints[1] << ZMPx_LF_max,ZMPy_LF_min,0.0;
        FootPoints[2] << ZMPx_LF_min,ZMPy_LF_min,0.0;
        FootPoints[3] << ZMPx_LF_min,ZMPy_LF_max,0.0;
    } else {
        return;
    }
    FootPoints[0] = StancePosition + RotZ(StanceZAngle)*FootPoints[0];
    FootPoints[1] = StancePosition + RotZ(StanceZAngle)*FootPoints[1];
    FootPoints[2] = StancePosition + RotZ(StanceZAngle)*FootPoints[2];
    FootPoints[3] = StancePosition + RotZ(StanceZAngle)*FootPoints[3];

    int n;
    Point* p;
    if(StepOn) {
        if(StanceLeg == RIGHTLEG) { // Step Leg : LEFTLEG
            FootPoints[4] << ZMPx_LF_max,ZMPy_LF_max,0.0;
            FootPoints[5] << ZMPx_LF_max,ZMPy_LF_min,0.0;
            FootPoints[6] << ZMPx_LF_min,ZMPy_LF_min,0.0;;
            FootPoints[7] << ZMPx_LF_min,ZMPy_LF_max,0.0;;
        } else if(StanceLeg == LEFTLEG) { // Step Leg : RIGHTLEG
            FootPoints[4] << ZMPx_RF_max,ZMPy_RF_max,0.0;
            FootPoints[5] << ZMPx_RF_max,ZMPy_RF_min,0.0;
            FootPoints[6] << ZMPx_RF_min,ZMPy_RF_min,0.0;
            FootPoints[7] << ZMPx_RF_min,ZMPy_RF_max,0.0;
        }
        FootPoints[4] = StancePosition + RotZ(StanceZAngle)*StepPosition + RotZ(StanceZAngle+StepZAngle)*FootPoints[4];
        FootPoints[5] = StancePosition + RotZ(StanceZAngle)*StepPosition + RotZ(StanceZAngle+StepZAngle)*FootPoints[5];
        FootPoints[6] = StancePosition + RotZ(StanceZAngle)*StepPosition + RotZ(StanceZAngle+StepZAngle)*FootPoints[6];
        FootPoints[7] = StancePosition + RotZ(StanceZAngle)*StepPosition + RotZ(StanceZAngle+StepZAngle)*FootPoints[7];

//        FILE_LOG(logINFO) << StancePosition.transpose();
//        FILE_LOG(logINFO) << StepPosition.transpose();
//        cout << "[All Points]" << endl;
//        cout << FootPoints[0].segment(0,2).transpose() << endl;
//        cout << FootPoints[1].segment(0,2).transpose() << endl;
//        cout << FootPoints[2].segment(0,2).transpose() << endl;
//        cout << FootPoints[3].segment(0,2).transpose() << endl;
//        cout << FootPoints[4].segment(0,2).transpose() << endl;
//        cout << FootPoints[5].segment(0,2).transpose() << endl;
//        cout << FootPoints[6].segment(0,2).transpose() << endl;
//        cout << FootPoints[7].segment(0,2).transpose() << endl;

        n = 8;
        p = (Point*)malloc(n*sizeof(Point));
        for(int i=0;i<n;i++) {
            p[i].x = (int)(FootPoints[i](0)*1000.0);
            p[i].y = (int)(FootPoints[i](1)*1000.0);
        }
        p = Convex_Hull(p, &n);
//        cout << "--------------------" << endl;
//        cout << "[Hull Points]" << endl;
//        for(int i=0;i<n;i++) printf("%d %d\n",p[i].x,p[i].y);
    } else {
//        cout << "[All Points]" << endl;
//        cout << FootPoints[0].segment(0,2).transpose() << endl;
//        cout << FootPoints[1].segment(0,2).transpose() << endl;
//        cout << FootPoints[2].segment(0,2).transpose() << endl;
//        cout << FootPoints[3].segment(0,2).transpose() << endl;

        n = 4;
        p = (Point*)malloc(4*sizeof(Point));
        for(int i=0;i<4;i++) {
            p[i].x = (int)(FootPoints[i](0)*1000.0);
            p[i].y = (int)(FootPoints[i](1)*1000.0);
        }
        p = Convex_Hull(p, &n);
//        cout << "--------------------" << endl;
//        cout << "[Hull Points]" << endl;
//        for(int i=0;i<n;i++) printf("%d %d\n",p[i].x,p[i].y);
    }

    MatrixNd hull(n,2);
    for(int i=0;i<n;i++) {
        hull(i,0) = (double)p[i].x/1000.0;
        hull(i,1) = (double)p[i].y/1000.0;
    }

    // Output : Matrix A and Vector B of Ax<B
    _A.resize(n,2);
    _B.resize(n,1);

    for(int i=0;i<n;i++) {
        VectorNd V1 = hull.row(i).transpose();
        VectorNd V2 = hull.row((i+1)%n).transpose();
        double val = V1(0)*V2(1) - V2(0)*V1(1);

        if(fabs(val) < 1e-6) {
            _A.row(i) << 1.0, -V1(0)/V1(1);
            _B(i) = 0.0;
        } else {
            _A.row(i) << (V2(1)-V1(1))/val, -(V2(0)-V1(0))/val;
            _B(i) = 1.0;
        }

        VectorNd V3 = hull.row((i+2)%n).transpose();
        VectorNd mat = _A.row(i)*V3;

        if(mat(0) > _B(i))
        {
            _A.row(i) = -_A.row(i);
            _B(i) = -_B(i);
        }
    }

//    // Print Result
//    cout << "A : " << endl << _A << endl << "B : " << endl << _B << endl;
}

bool Get_SupportConvexHull_Simplified4Lateral(int StanceLeg, Vector3d StancePosition, bool StepOn,
                                              MatrixNd& _A, VectorNd& _B)
{
    double ZMPy_RF_max = LIGHT_Info.footlimit_inside;
    double ZMPy_RF_min = 0.0;
    double ZMPy_LF_min = -LIGHT_Info.footlimit_inside;
    double ZMPy_LF_max = 0.0;

    if(StepOn) {  // During DSP, no constraints
        return false;
    } else { // During SSP, there is a foot-inside constraint
        _A.resize(2,1);
        _B.resize(2,1);
        if(StanceLeg == RIGHTLEG) {
            _A(0,0) = 1.0;
            _B(0,0) = StancePosition(1) + ZMPy_RF_max;
            _A(1,0) = -1.0;
            _B(1,0) = -(StancePosition(1) + ZMPy_RF_min);
        } else if(StanceLeg == LEFTLEG) {
            _A(0,0) = -1.0;
            _B(0,0) = -(StancePosition(1) + ZMPy_LF_min);
            _A(1,0) = 1.0;
            _B(1,0) = StancePosition(1) + ZMPy_LF_max;
        } else {
            return false;
        }
        return true;

//        _A.resize(1,1);
//        _B.resize(1,1);
//        if(StanceLeg == RIGHTLEG) {
////            _A(0,0) = 1.0;
////            _B(0,0) = StancePosition(1) + ZMPy_RF_max;
//            _A(0,0) = -1.0;
//            _B(0,0) = -(StancePosition(1) + ZMPy_RF_min);
//        } else if(StanceLeg == LEFTLEG) {
////            _A(0,0) = -1.0;
////            _B(0,0) = -(StancePosition(1) + ZMPy_LF_min);
//            _A(0,0) = 1.0;
//            _B(0,0) = StancePosition(1) + ZMPy_LF_max;
//        } else {
//            return false;
//        }
//        return true;
    }
}


MatrixNd pseudoInverse(MatrixNd & origin)
{
    // perform svd decomposition
    Eigen::JacobiSVD<MatrixNd> svd_holder(origin,
                                                 Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    // Build SVD decomposition results
    MatrixNd U = svd_holder.matrixU();
    MatrixNd V = svd_holder.matrixV();
    MatrixNd D = svd_holder.singularValues();

    // Build the S matrix
    MatrixNd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i) {

        if (D(i, 0) > 0.0) {
            S(i, i) = 1 / D(i, 0);
        } else {
            S(i, i) = 0;
        }
    }

    // pinv_matrix = V * S * U^T
    return V * S * U.transpose();
}

//==============================//
// Movement function
//==============================//

void fifth_trajectory_oneaxis(double t, double p, double v, double a, double pf, double vf, double af, double &pn, double &vn, double &an)
{
    // t : moving time
    // p, v, a : current position, velocity, acceleration
    // pf, vf, af : final position, velocity, acceleration
    // pn, vn, an : next step position, velocity, acceleration

    double k[6] = {0.0, }; // Coefficient for 'fifth order polynomial trajectory'
    double theta = p,theta_dot = v,theta_ddot = a;
    double theta_f = pf,theta_dot_f = vf,theta_ddot_f = af;

    // ref(0) : Position
    // ref(1) : Velocity
    // ref(2) : Acceleration
    if(t < 1e-6)
    {
        pn = theta;
        vn = theta_dot;
        an = theta_ddot;
    }else {
        k[0] = theta;
        k[1] = theta_dot;
        k[2] = theta_ddot/2.;
        k[3] = (20.*(theta_f-theta) - (8.*theta_dot_f + 12.*theta_dot)*t-(3.*theta_ddot - theta_ddot_f)*t*t)/(2.*t*t*t);
        k[4] = ((30.*theta - 30.*theta_f) + (14.*theta_dot_f+16.*theta_dot)*t+(3.*theta_ddot-2.*theta_ddot_f)*t*t)/(2.*t*t*t*t);
        k[5] = (12.*theta_f - 12.*theta - (6.*theta_dot_f + 6.*theta_dot)*t - (theta_ddot-theta_ddot_f)*t*t)/(2.*t*t*t*t*t);

        double dt = SYS_DT_WALKING;
        pn = k[0] + k[1]*dt  + k[2]*dt*dt + k[3]*dt*dt*dt + k[4]*dt*dt*dt*dt + k[5]*dt*dt*dt*dt*dt;
        vn = k[1] + 2.0*k[2]*dt + 3.0*k[3]*dt*dt + 4.0*k[4]*dt*dt*dt + 5.0*k[5]*dt*dt*dt*dt;
        an = 2.0*k[2] + 6.0*k[3]*dt + 12.0*k[4]*dt*dt + 20.0*k[5]*dt*dt*dt;
    }
}

void third_trajectory_oneaxis(double t, double p, double v, double pf, double vf, double &pn, double &vn)
{
    // t : moving time
    // p, v : current position, velocity
    // pf, vf : final position, velocity
    // pn, vn : next step position, velocity

    double k[4] = {0.0, }; // Coefficient for 'third order polynomial trajectory'
    double theta = p,theta_dot = v;
    double theta_f = pf,theta_dot_f = vf;

    // ref(0) : Position
    // ref(1) : Velocity
    if(t < 1e-6)
    {
        pn = theta;
        vn = theta_dot;
    }else {
        k[0] = theta;
        k[1] = theta_dot;
        k[2] = (3.0*(theta_f-theta) - (1.0*theta_dot_f + 2.0*theta_dot)*t)/(t*t);
        k[3] = (-2.0*(theta_f-theta) + (1.0*theta_dot_f + 1.0*theta_dot)*t)/(t*t*t);

        double dt = SYS_DT_WALKING;
        pn = k[0] + k[1]*dt  + k[2]*dt*dt + k[3]*dt*dt*dt;
        vn = k[1] + 2.0*k[2]*dt + 3.0*k[3]*dt*dt;
    }
}

void slerp_trajectory(double s, Quaternion q, Quaternion qf, Quaternion &qn)
{
    // this function is for getting interpolation parameter of slerp.
    // f(0) = q  f(T) = qf
    double p = sqrt (q.squaredNorm() * qf.squaredNorm());
    double angle_0 = acos(q.dot(qf)/p);

    if (angle_0 == 0.0 || std::isnan(angle_0)) {
        qn = qf;
        return;
    }

    double p0 = sin((1.0 - s) * angle_0) / sin(angle_0);
    double p1 = sin(s * angle_0) / sin(angle_0);
    qn = Quaternion((q * p0 + qf * p1));
}

void AngularInfo2Quaternion(Matrix3d R, Vector3d w, Vector3d dw, Quaternion &q, Quaternion &dq, Quaternion &ddq)
{
    R = q.toMatrix();

    Quaternion temp_w = Quaterion_multiplication(dq, q.conjugate());
    w(0) = 2.0*temp_w(0);
    w(1) = 2.0*temp_w(1);
    w(2) = 2.0*temp_w(2);

    Quaternion temp_q0 = Quaterion_multiplication(ddq, q.conjugate());
    Quaternion temp_q1 = Quaterion_multiplication(temp_w, temp_w);
    dw(0) =  2.0*(temp_q0(0)-temp_q1(0));
    dw(1) =  2.0*(temp_q0(1)-temp_q1(1));
    dw(2) =  2.0*(temp_q0(2)-temp_q1(2));

}

void Quaternion2AngularInfo(Quaternion q, Matrix3d &R)
{
    R = q.toMatrix();

//    Quaternion temp_w = Quaterion_multiplication(dq, q.conjugate());
//    w(0) = 2.0*temp_w(0);
//    w(1) = 2.0*temp_w(1);
//    w(2) = 2.0*temp_w(2);

//    Quaternion temp_q0 = Quaterion_multiplication(ddq, q.conjugate());
//    Quaternion temp_q1 = Quaterion_multiplication(temp_w, temp_w);
//    dw(0) =  2.0*(temp_q0(0)-temp_q1(0));
//    dw(1) =  2.0*(temp_q0(1)-temp_q1(1));
//    dw(2) =  2.0*(temp_q0(2)-temp_q1(2));

}

Matrix3d VectorCross2Matrix(Vector3d _V)
{
    Matrix3d _M = Matrix3d::Zero();
    _M(0,1) = -_V(2);
    _M(0,2) = _V(1);
    _M(1,2) = -_V(0);
    _M(1,0) = _V(2);
    _M(2,0) = -_V(1);
    _M(2,1) = _V(0);

    return _M;
}

Matrix3d expMatrix(Matrix3d _M) {
    Matrix3d _Mo = Matrix3d::Identity();
    Matrix3d tempM = _M;

    int _N = 20;
    double k = 2.0;
    for(int i=1;i<=_N;i++) {
        _Mo = _Mo + tempM;
        tempM = tempM*_M*(1.0/k);
        k = k + 1.0;
    }
    return _Mo;
}

//==============================//
// States Filtering
//==============================//
double LPF_1st(double x_old, double x_new, double f_cut)
{
    double alpha = 1.0/(1.0+SYS_FREQ_WALKING/(2.0*PI*f_cut));

    return (1.0-alpha) * x_old + alpha * x_new;
}

double LPF_2nd(double x_oold, double x_old, double x_new, double f_cut)
{
    double w_cut = 2.0*PI*f_cut;
    double alpha1 = -(SYS_FREQ_WALKING*SYS_FREQ_WALKING)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);
    double alpha2 = (2.0*SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);
    double alpha3 = (w_cut*w_cut)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);

    return alpha1*x_oold+alpha2*x_old+alpha3*x_new;
}

VectorNd Vector_LPF(VectorNd v, VectorNd v_new, double f_cut) {
    VectorNd v_updated = VectorNd::Zero(v.rows());

    if(v.rows()!=v_new.rows()) {
        FILE_LOG(logERROR) << v.rows();
        FILE_LOG(logERROR) << v_new.rows();
        FILE_LOG(logERROR) << "Vector size mismatch.";
        return v_updated;
    } else {
        for(int i=0;i<v.rows();i++) {
            double alpha = 1.0/(1.0+SYS_FREQ_WALKING/(2.0*PI*f_cut));
            v_updated(i)=(1.0-alpha)*v(i)+(alpha)*v_new(i);
        }
        return v_updated;
    }
}

// ref : https://dsp.stackexchange.com/questions/8693/how-does-a-low-pass-filter-programmatically-work
// ref2 : https://www.electronics-tutorials.ws/filter/filter_8.html
VectorNd Vector_LPF_2nd(VectorNd voo, VectorNd vo, VectorNd v_new, double f_cut) {
    VectorNd v_updated = VectorNd::Zero(vo.rows());

    if(vo.rows()!=v_new.rows()) {
        FILE_LOG(logERROR) << vo.rows();
        FILE_LOG(logERROR) << v_new.rows();
        FILE_LOG(logERROR) << "Vector size mismatch.";
        return v_updated;
    } else {
        for(int i=0;i<vo.rows();i++) {
            double w_cut = 2.0*PI*f_cut;
            double alpha1 = -(SYS_FREQ_WALKING*SYS_FREQ_WALKING)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);
            double alpha2 = (2.0*SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);
            double alpha3 = (w_cut*w_cut)/(SYS_FREQ_WALKING*SYS_FREQ_WALKING+sqrt(2.0)*w_cut*SYS_FREQ_WALKING+w_cut*w_cut);

            v_updated(i)=alpha1*voo(i)+alpha2*vo(i)+alpha3*v_new(i);
        }
        return v_updated;
    }
}

//==============================//
// Save Functions
//==============================//

void PrintHere(int n)
{
    static int cnt = 0;
    if(n == 0) {
        FILE_LOG(logDEBUG1) << "I'm Here : " << cnt;
        cnt++;
    } else {
        FILE_LOG(logDEBUG) << "I'm Here : " << n;
    }
}

//==============================//



