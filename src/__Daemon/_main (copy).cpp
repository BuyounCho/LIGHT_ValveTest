#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <QSettings>

#include "RBCAN.h"
#include "RBRawLAN.h"
#include "RBDataBase.h"
#include "../../share/Headers/LANData/GazeboLANData.h"

#include "RBProcessManager.h"

#include "HydraulicActuatorDataConverting.h"
#include "HydraulicActuatorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
#include "RBSmartPower.h"
#include "RBOpticFlowSensor.h"
#include "RBFOGSensor.h"
#include "RBELMO.h"

//#include "Eigen/Dense"
//#include "rbdl/rbdl.h"

using namespace std;

QString     settingFile;

// Basic --------
int     IS_WORKING = false;
int     IS_CHILD = false;
int     IS_CAN_OK = false;
int     IS_RS232_OK = false;

int     NO_RESPONSE_CNT = 0;

pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pRBCAN                  canHandler;
pRBLAN                  lanHandler;
RBProcessManager        *pmHandler;
OpticalDisplacement     ODHandler;

// Daemon Options
int     __IS_GAZEBO = false;
int     __IS_FOG = false;
int     __IS_ROS = false;

// Initialize --------
int     RBCore_Initialize();
int     RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_LANInitialize();
int     RBCore_ThreadInitialize();
int     RBCore_PMInitialize();
int     RBCore_Termination();
void    RBCore_RTThreadCon(void *);
void    *RBCore_NRTThreadCon(void *);
bool    DO_CANCHECK = false;
RT_TASK rtTaskCon;
ulong   nrtTaskCon;

// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_PC           RBDataBase::_DB_PC[MAX_PC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_OF           RBDataBase::_DB_OF[MAX_OF];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

int     _VERSION;
int     _NO_OF_AL;
int     _NO_OF_COMM_CH;
int     _NO_OF_MC;
int     _NO_OF_PC;
int     _NO_OF_FT;
int     _NO_OF_IMU;
int     _NO_OF_SP;
int     _NO_OF_OF;



#define     g_const      9.81
#define     COLS_InEKF   3
#define     ROWS_InEKF   3

class Vector3_InEKF {
public:
    double v[COLS_InEKF];
public:
    Vector3_InEKF(double a = 0.0, double b = 0.0, double c = 0.0) {
        v[0] = a;        v[1] = b;        v[2] = c;
    }
    ~Vector3_InEKF() {

    }
    void show() {
        std::cout << v[0] << ", " << v[1] << ", " << v[2] << std::endl << std::endl;
    }

    void operator=(const Vector3_InEKF &in)
    {
        for(int i=0;i<COLS_InEKF;i++) {
            v[i] = in.v[i];
        }
    }

    Vector3_InEKF operator+(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = v[i] + in.v[i];
        }
        return out;
    }
    Vector3_InEKF operator-(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = v[i] - in.v[i];
        }
        return out;
    }

    Vector3_InEKF operator*(const double &in) {
        Vector3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            out.v[i] = in*v[i];
        }
        return out;
    }

    Vector3_InEKF Zero() {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = 0.0;
        }
        return out;
    }

    double norm() {
        return sqrt(this->v[0]*this->v[0]
                +this->v[1]*this->v[1]
                +this->v[2]*this->v[2]);
    }
};
Vector3_InEKF operator*(const double &a,const Vector3_InEKF &in) {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = a*in.v[i];
    }
    return out;
}
Vector3_InEKF Ones_Vec3() {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = 1.0;
    }
    return out;
}
Vector3_InEKF Zero_Vec3() {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = 0.0;
    }
    return out;
}

class Matrix3_InEKF {
public:
    double M[3][3];
public:
    Matrix3_InEKF(double a = 0.0, double b = 0.0, double c = 0.0,
                  double d = 0.0, double e = 0.0, double f = 0.0,
                  double g = 0.0, double h = 0.0, double i = 0.0)
    {
        M[0][0] = a; M[0][1] = b; M[0][2] = c;
        M[1][0] = d; M[1][1] = e; M[1][2] = f;
        M[2][0] = g; M[2][1] = h; M[2][2] = i;
    }

    ~Matrix3_InEKF() {

    }

    void show() {
        std::cout << M[0][0] << ", " << M[0][1] << ", " << M[0][2] << std::endl;
        std::cout << M[1][0] << ", " << M[1][1] << ", " << M[1][2] << std::endl;
        std::cout << M[2][0] << ", " << M[2][1] << ", " << M[2][2] << std::endl << std::endl;
    }

    void operator=(const Matrix3_InEKF &in)
    {
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                M[i][j] = in.M[i][j];
            }
        }
    }

    Matrix3_InEKF Transpose(){
        Matrix3_InEKF copy;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                copy.M[i][j] = M[i][j];
            }
        }
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                M[j][i] = copy.M[i][j];
            }
        }
        return *this;
    }

    Matrix3_InEKF Inverse() {
        Matrix3_InEKF out;
        double det = M[0][0] * (M[1][1] * M[2][2] - M[2][1] * M[1][2]) -
                M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

        if(abs(det)<1e-10) {
            if(det>=0.0) det = 1e-10;
            else det = -1e-10;
        }

        out.M[0][0] =  (M[1][1] * M[2][2] - M[2][1] * M[1][2]) / det;
        out.M[0][1] =  (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
        out.M[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
        out.M[1][0] =  (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
        out.M[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
        out.M[1][2] =  (M[1][0] * M[0][2] - M[0][0] * M[1][2]) / det;
        out.M[2][0] =  (M[1][0] * M[2][1] - M[2][0] * M[1][1]) / det;
        out.M[2][1] =  (M[2][0] * M[0][1] - M[0][0] * M[2][1]) / det;
        out.M[2][2] =  (M[0][0] * M[1][1] - M[1][0] * M[0][1]) / det;
        return out;
    }

    Matrix3_InEKF operator+(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = M[i][j] + in.M[i][j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator-(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = M[i][j] - in.M[i][j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator*(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                for (int k=0; k<ROWS_InEKF; k++) {
                    out.M[i][j] += M[i][k] * in.M[k][j];
                }
            }
        }
        return out;
    }

    Vector3_InEKF operator*(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.v[i] += M[i][j] * in.v[j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator*(const double &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = in*M[i][j];
            }
        }
        return out;
    }

};
Matrix3_InEKF Zero_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = 0.0;
        }
    }
    return out;
}
Matrix3_InEKF Identity_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            if(i==j) out.M[i][j] = 1.0;
            else out.M[i][j] = 0.0;
        }
    }
    return out;
}
Matrix3_InEKF operator*(const double &a, const Matrix3_InEKF &in) {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = a*in.M[i][j];
        }
    }
    return out;
}
Matrix3_InEKF hat_oper(const Vector3_InEKF in) {
    double a = in.v[0];
    double b = in.v[1];
    double c = in.v[2];
    Matrix3_InEKF out(0.0,   -c,    b,
                      c,  0.0,   -a,
                      -b,    a,  0.0);
    return out;
}
Matrix3_InEKF expM(const Matrix3_InEKF M) {
    // exp(M) := I + M + M^2/2! + M^3/3! + M^4/4! + .... + M^N/N!
    Matrix3_InEKF out = Identity_Mat3();
    Matrix3_InEKF tempM = M;

    int N = 20; // approximation
    double k = 2.0;
    for(int i=1;i<=N;i++) {
        out = out + tempM;
        tempM = tempM*M*(1.0/k);
        k = k + 1.0;
    }
    return out;
}

Matrix3_InEKF exp_so3(Vector3_InEKF v) {
    // M = hat(v)
    // exp(M) = I + M + M^2/2! + M^3/3! + M^4/4! + .... + M^N/N!
    //        = I + sin(theta)*hat(v) + (1-cos(theta))*hat(v)*hat(v)
    double theta = v.norm();
    if (fabs(theta) > 1e-6) {
        Matrix3_InEKF K = hat_oper(v*(1.0/v.norm()));
        return (Identity_Mat3() + sin(theta)*K + (1-cos(theta))*K*K);
    } else {
        return Identity_Mat3();
    }
}

class InEKF_Parameter {
private:
    bool Flag_initialized;
    double t;

    Vector3_InEKF stdev_gyro; //standard deviation for the gyroscope
    Vector3_InEKF stdev_acc; //standard deviation for the accelerometer
    Vector3_InEKF rotation_stdev_prior; //standard deviation for the initial position estimate
    Vector3_InEKF g_vec; // Gravity Vector
    //    Vector3_InEKF g_vec(0.0,-9.81,0.0); // Gravity Vector
    //    Vector3_InEKF g_vec(0.0,9.81,0.0); // Gravity Vector

    Vector3_InEKF w_init; // initial angular velocity
    Vector3_InEKF w_prev; // previous angular velocity

    Matrix3_InEKF Qg; // Covariance Matrix for gyroscope
    Matrix3_InEKF Qa; // Covariance Matrix for accelerometer
    Matrix3_InEKF P_prior; // covariance matrix for the initial position

public:
    Matrix3_InEKF R; // Rotation Matrix
    Matrix3_InEKF P; // Covariance Matrix

public:
    InEKF_Parameter() {
        Flag_initialized = false;
        t = 0.0;

        stdev_gyro.v[0] = 0.001;
        stdev_gyro.v[1] = 0.001;
        stdev_gyro.v[2] = 0.001;

        stdev_acc.v[0] = 0.005;
        stdev_acc.v[1] = 0.005;
        stdev_acc.v[2] = 0.005;

        rotation_stdev_prior.v[0] = 0.01;
        rotation_stdev_prior.v[1] = 0.01;
        rotation_stdev_prior.v[2] = 0.01;

        g_vec.v[0] = 0.0;
        g_vec.v[1] = 0.0;
        g_vec.v[2] = g_const;

        R = Identity_Mat3();
        P = Identity_Mat3();

        Qg = Identity_Mat3();
        Qg.M[0][0] = stdev_gyro.v[0]*stdev_gyro.v[0];
        Qg.M[1][1] = stdev_gyro.v[1]*stdev_gyro.v[1];
        Qg.M[2][2] = stdev_gyro.v[2]*stdev_gyro.v[2];

        Qa = Identity_Mat3();
        Qa.M[0][0] = stdev_acc.v[0]*stdev_acc.v[0];
        Qa.M[1][1] = stdev_acc.v[1]*stdev_acc.v[1];
        Qa.M[2][2] = stdev_acc.v[2]*stdev_acc.v[2];

        P_prior = Identity_Mat3();
        P_prior.M[0][0] = rotation_stdev_prior.v[0]*rotation_stdev_prior.v[0];
        P_prior.M[1][1] = rotation_stdev_prior.v[1]*rotation_stdev_prior.v[1];
        P_prior.M[2][2] = rotation_stdev_prior.v[2]*rotation_stdev_prior.v[2];
    }

    void Zero() {
        t = 0.0;
    }

    bool Initialize(Vector3_InEKF _a) {
        double norm = _a.norm();
        if(norm > 1e-6) {
            Vector3_InEKF a_n = _a*(1.0/norm);

            // axis-angle
            // unit vector : u = [ux;uy;uz] = [cos(phi) sin(phi) 0]; (-PI/2.0<phi<PI/2.0)
            // angle : theta;
            // R = [ cos(theta) + ux*ux*(1-cos(theta));    ux*uy*(1-cos(theta))-uz*sin(theta);   ux*uz*(1-cos(theta))+uy*sin(theta);
            //       ux*uy*(1-cos(theta))+uz*sin(theta);   cos(theta) + uy*uy*(1-cos(theta));    uy*uz*(1-cos(theta))-ux*sin(theta);
            //       ux*uz*(1-cos(theta))-uy*sin(theta);   uy*uz*(1-cos(theta))+ux*sin(theta);   cos(theta) + uz*uz*(1-cos(theta))];
            //     [ cos(theta) + ux*ux*(1-cos(theta));    ux*uy*(1-cos(theta));                 uy*sin(theta);
            //       ux*uy*(1-cos(theta));                 cos(theta) + uy*uy*(1-cos(theta));    -ux*sin(theta);
            //       -uy*sin(theta);                       ux*sin(theta);                        cos(theta)];

            double theta = acos(a_n.v[2]);

            if(fabs(theta) < 1e-2) {
                Matrix3_InEKF _R_init = Identity_Mat3();
                R = _R_init;
                P = P_prior;
            } else {
                double ux = a_n.v[1]/sin(theta);
                double uy = -a_n.v[0]/sin(theta);

                Matrix3_InEKF _R_init(cos(theta) + ux*ux*(1-cos(theta)), ux*uy*(1-cos(theta))              , uy*sin(theta),
                                      ux*uy*(1-cos(theta))             , cos(theta) + uy*uy*(1-cos(theta)) ,-ux*sin(theta),
                                      -uy*sin(theta)                   , ux*sin(theta)                     , cos(theta));

                //                _R_init.show();
                R = _R_init;
                P = P_prior;
            }
            FILE_LOG(logSUCCESS) << "IMU Initializing is Done!!";
            return Flag_initialized = true;
        } else {
            return Flag_initialized = false;
        }
    }

    void Update(Vector3_InEKF _Uw, Vector3_InEKF _Ua, bool Correction_OnOff = true) {
        // calculate the state from the input information

        // Initializing
        if(t<(double)RT_TIMER_PERIOD_MS/1000.0/2.0) {
            Initialize(_Ua);
        }

        // prediction step - update all the state using the gyroscope data
        double dT = (double)RT_TIMER_PERIOD_MS/1000.0;
        double K_YawZero = 2.0*3.141592/(30.0); // 30sec convergence for prevent drift
        Vector3_InEKF psi_vec(0.0,0.0,atan2(R.M[1][0], R.M[0][0]));
        Vector3_InEKF w = _Uw - K_YawZero*(psi_vec);
        Matrix3_InEKF R_pred = R*exp_so3(w*dT);

        // Define the Adjoint matrix
        Matrix3_InEKF Adj = R_pred;

        // define Linearized dynamics matrix
        Matrix3_InEKF A = Zero_Mat3();

        // propagate the covariance matrix with the riccati equation
        Matrix3_InEKF F = Identity_Mat3()+A*dT;
        Matrix3_InEKF Cov_w = Qg;
        Matrix3_InEKF Q = F*Adj*Cov_w*Adj.Transpose()*F*dT; // approximation
        Matrix3_InEKF P_pred = F*P*F.Transpose() + Q;

        // correction step
        if(Correction_OnOff) {
            // kalman filter gain
            Matrix3_InEKF H = hat_oper(g_vec);
            Matrix3_InEKF S = H*P_pred*H.Transpose() + Qa;
            Matrix3_InEKF K = P_pred*H.Transpose()*S.Inverse();

            Vector3_InEKF g_error = R_pred*_Ua*(g_vec.norm()/_Ua.norm())-g_vec;
            Matrix3_InEKF R_cor = exp_so3(K*g_error);

            R = R_cor*R_pred;
            P = (Identity_Mat3() - K*H)*P_pred;
            //            R.show();

            //            double norm = _Ua.norm();
            //            double theta = 0.0;
            //            Vector3_InEKF u;
            //            if(norm > 1e-6) {
            //                Vector3_InEKF Ua_n = _Ua*(1.0/norm);
            //                theta = acos(Ua_n.v[2]);
            //                if(fabs(theta) < 1e-2) {
            //                    u.v[0] = 1.0;
            //                    u.v[1] = 0.0;
            //                    u.v[2] = 0.0;
            //                } else {
            //                    u.v[0] = Ua_n.v[1]/sin(theta);
            //                    u.v[1] = -Ua_n.v[0]/sin(theta);
            //                    u.v[2] = 0.0;
            //                }
            //            } else {
            //                theta = 0.0;
            //                u.v[0] = 1.0;
            //                u.v[1] = 0.0;
            //                u.v[2] = 0.0;
            //            }
            //            Matrix3_InEKF R_cor = exp_so3(K*u*theta);
            //            R = R_cor*R_pred;
            //            P = (Identity_Mat3() - K*H)*P_pred;
            //            R_cor.show();

        } else {
            R = R_pred;
            P = P_pred;
        }

        t += dT;
        if(t>100000.0) {
            Zero();
        }
    }
};

InEKF_Parameter  InEKF_IMU;



float local_wx_o,local_ax_o;
float local_wy_o,local_ay_o;
float local_wz_o,local_az_o;
float local_wx_oo,local_ax_oo;
float local_wy_oo,local_ay_oo;
float local_wz_oo,local_az_oo;

float local_wx_f,local_ax_f;
float local_wy_f,local_ay_f;
float local_wz_f,local_az_f;
float local_wx_fo,local_ax_fo;
float local_wy_fo,local_ay_fo;
float local_wz_fo,local_az_fo;
float local_wx_foo,local_ax_foo;
float local_wy_foo,local_ay_foo;
float local_wz_foo,local_az_foo;

void THREAD_ReadNewIMU(void){

    if(_DEV_IMU[0].ConnectionStatus == true) {

        float local_wx,local_ax;
        float local_wy,local_ay;
        float local_wz,local_az;

        /// Local Angular Velocity Receive
        RBCAN_MB mb2;
        mb2.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb2.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_X;
        canHandler->RBCAN_ReadData(&mb2);
        if(mb2.status != RBCAN_NODATA){
            memcpy(&local_wx,mb2.data,4);   // data[0]~data[3]
            memcpy(&local_ax,mb2.data+4,4); // data[4]~data[7]
            mb2.status = RBCAN_NODATA;
        } else {
            local_wx = local_wx_o;
            local_ax = local_ax_o;
        }

        RBCAN_MB mb3;
        mb3.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb3.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_Y;
        canHandler->RBCAN_ReadData(&mb3);
        if(mb3.status != RBCAN_NODATA){
            memcpy(&local_wy,mb3.data,4);   // data[0]~data[3]
            memcpy(&local_ay,mb3.data+4,4); // data[4]~data[7]
            mb3.status = RBCAN_NODATA;
        } else {
            local_wy = local_wy_o;
            local_ay = local_ay_o;
        }

        RBCAN_MB mb4;
        mb4.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb4.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_Z;
        canHandler->RBCAN_ReadData(&mb4);
        if(mb4.status != RBCAN_NODATA){
            memcpy(&local_wz,mb4.data,4);   // data[0]~data[3]
            memcpy(&local_az,mb4.data+4,4); // data[4]~data[7]
            mb4.status = RBCAN_NODATA;
        } else {
            local_wz = local_wz_o;
            local_az = local_az_o;
        }

//        // Low pass filter (or notch filter) for pump vibration
//        if(sharedSEN->PUMP[0].CurrentVelocity < 600.0) {
//            local_wx_f = local_wx;
//            local_wy_f = local_wy;
//            local_wz_f = local_wz;
//            local_ax_f = local_ax;
//            local_ay_f = local_ay;
//            local_az_f = local_az;
//        } else {
//            double wdt_notch = 2.0*3.141592*sharedSEN->PUMP[0].CurrentVelocity/60.0*(double)RT_TIMER_PERIOD_MS/1000.0;
//            double Q_notch = 0.3;
//            double a1_notch = 1.0 + wdt_notch/Q_notch + wdt_notch*wdt_notch;
//            double a2_notch = -2.0 - wdt_notch/Q_notch;
//            double a3_notch = 1.0;
//            double b1_notch = 1.0 + wdt_notch*wdt_notch;
//            double b2_notch = -2.0;
//            double b3_notch = 1.0;

//            local_wx_f = (b1_notch*local_wx + b2_notch*local_wx_o + b3_notch*local_wx_oo - a2_notch*local_wx_fo - a3_notch*local_wx_foo)/a1_notch;
//            local_wy_f = (b1_notch*local_wy + b2_notch*local_wy_o + b3_notch*local_wy_oo - a2_notch*local_wy_fo - a3_notch*local_wy_foo)/a1_notch;
//            local_wz_f = (b1_notch*local_wz + b2_notch*local_wz_o + b3_notch*local_wz_oo - a2_notch*local_wz_fo - a3_notch*local_wz_foo)/a1_notch;
//            local_ax_f = (b1_notch*local_ax + b2_notch*local_ax_o + b3_notch*local_ax_oo - a2_notch*local_ax_fo - a3_notch*local_ax_foo)/a1_notch;
//            local_ay_f = (b1_notch*local_ay + b2_notch*local_ay_o + b3_notch*local_ay_oo - a2_notch*local_ay_fo - a3_notch*local_ay_foo)/a1_notch;
//            local_az_f = (b1_notch*local_az + b2_notch*local_az_o + b3_notch*local_az_oo - a2_notch*local_az_fo - a3_notch*local_az_foo)/a1_notch;
//        }

//        double w_cut = 2.0*PI*1.0;
//        double SYS_FREQ = RT_TIMER_FREQ;
//        double alpha1 = -(SYS_FREQ*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
//        double alpha2 = (2.0*SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
//        double alpha3 = (w_cut*w_cut)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);

//        local_wx_f = alpha1*local_wx_oo + alpha2*local_wx_o + alpha3*local_wx;
//        local_wy_f = alpha1*local_wy_oo + alpha2*local_wy_o + alpha3*local_wy;
//        local_wz_f = alpha1*local_wz_oo + alpha2*local_wz_o + alpha3*local_wz;
//        local_ax_f = alpha1*local_ax_oo + alpha2*local_ax_o + alpha3*local_ax;
//        local_ay_f = alpha1*local_ay_oo + alpha2*local_ay_o + alpha3*local_ay;
//        local_az_f = alpha1*local_az_oo + alpha2*local_az_o + alpha3*local_az;

        double fcut_wp = 15.0;
        double alpha_wp = 1.0/(1.0+2.0*PI*fcut_wp*(double)RT_TIMER_PERIOD_MS/1000.0);

        local_wx_f = alpha_wp*local_wx_f + (1.0-alpha_wp)*local_wx;
        local_wy_f = alpha_wp*local_wy_f + (1.0-alpha_wp)*local_wy;
        local_wz_f = alpha_wp*local_wz_f + (1.0-alpha_wp)*local_wz;
        local_ax_f = alpha_wp*local_ax_f + (1.0-alpha_wp)*local_ax;
        local_ay_f = alpha_wp*local_ay_f + (1.0-alpha_wp)*local_ay;
        local_az_f = alpha_wp*local_az_f + (1.0-alpha_wp)*local_az;

        local_wx_oo = local_wx_o;
        local_wy_oo = local_wy_o;
        local_wz_oo = local_wz_o;
        local_ax_oo = local_ax_o;
        local_ay_oo = local_ay_o;
        local_az_oo = local_az_o;
        local_wx_o = local_wx;
        local_wy_o = local_wy;
        local_wz_o = local_wz;
        local_ax_o = local_ax;
        local_ay_o = local_ay;
        local_az_o = local_az;

        local_wx_foo = local_wx_fo;
        local_wy_foo = local_wy_fo;
        local_wz_foo = local_wz_fo;
        local_ax_foo = local_ax_fo;
        local_ay_foo = local_ay_fo;
        local_az_foo = local_az_fo;
        local_wx_fo = local_wx_f;
        local_wy_fo = local_wy_f;
        local_wz_fo = local_wz_f;
        local_ax_fo = local_ax_f;
        local_ay_fo = local_ay_f;
        local_az_fo = local_az_f;


        // reversed IMU with sponge (210830)
        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wy_B      = (double)local_wx_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wz_B      = (double)-local_wz_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Ay_B      = (double)local_ax_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Az_B      = (double)-local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // for LIGHT2
//        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Wy_B      = (double)-local_wx_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Wz_B      = (double)local_wz_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
//        sharedSEN->IMU[0].Ay_B      = (double)-local_ax_f*9.81; // Unit : (normalized g) > m/s^2
//        sharedSEN->IMU[0].Az_B      = (double)local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // Invarient Extended Kalman Filter
        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
        InEKF_IMU.Update(Uw,Ua);
        Matrix3_InEKF R = InEKF_IMU.R;

        // Projection to Yaw rotation angle zero
        bool Flag_YawZero = true;
        if(Flag_YawZero) {
            if(fabs(sqrt(R.M[1][0]*R.M[1][0]+R.M[0][0]*R.M[0][0]))<1e-6) {
                if(R.M[0][0] < 0.0) {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = 90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, 1.0,
                                            0.0, 1.0, 0.0,
                                            1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                } else {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = -90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, -1.0,
                                            0.0, 1.0, 0.0,
                                            -1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                }
            } else {
                double psi = atan2(R.M[1][0], R.M[0][0]);   // z-axis (Yaw)
    //            double theta = -asin(R.M[2][0]);            // y'-axis (Pitch)
    //            double phi = atan2(R.M[2][1], R.M[2][2]);   // x''-axis (Roll)
                Matrix3_InEKF R_yaw(cos(psi), -sin(psi), 0.0,
                                    sin(psi), cos(psi) , 0.0,
                                    0.0     , 0.0      , 1.0);
                Matrix3_InEKF R_yawzero = R_yaw.Transpose()*R;

                sharedSEN->IMU[0].Q[0] = sqrt(1.0+R_yawzero.M[0][0]+R_yawzero.M[1][1]+R_yawzero.M[2][2])/2.0;
                sharedSEN->IMU[0].Q[1] = (R_yawzero.M[2][1]-R_yawzero.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[2] = (R_yawzero.M[0][2]-R_yawzero.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[3] = (R_yawzero.M[1][0]-R_yawzero.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

                sharedSEN->IMU[0].Roll = atan2(-R_yawzero.M[1][2],R_yawzero.M[1][1]);
                sharedSEN->IMU[0].Pitch = atan2(-R_yawzero.M[2][0],R_yawzero.M[0][0]);
                sharedSEN->IMU[0].Yaw = 0.0;

                Vector3_InEKF W_G = R_yawzero*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF A_G = R_yawzero*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            }
        } else {
            sharedSEN->IMU[0].Q[0] = sqrt(1.0+R.M[0][0]+R.M[1][1]+R.M[2][2])/2.0;
            sharedSEN->IMU[0].Q[1] = (R.M[2][1]-R.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[2] = (R.M[0][2]-R.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[3] = (R.M[1][0]-R.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

            double q0 = sharedSEN->IMU[0].Q[0];
            double q1 = sharedSEN->IMU[0].Q[1];
            double q2 = sharedSEN->IMU[0].Q[2];
            double q3 = sharedSEN->IMU[0].Q[3];
            sharedSEN->IMU[0].Yaw   = atan2(2.0*(q1*q2 + q0*q3), 1.0-2.0*(q2*q2 + q3*q3)); // z-axis
            sharedSEN->IMU[0].Pitch = -asin(2.0*(q1*q3 - q0*q2));                          // y'-axis
            sharedSEN->IMU[0].Roll  = atan2(2.0*(q2*q3 + q0*q1), 1.0-2.0*(q1*q1 + q2*q2)); // x''-axis

            Vector3_InEKF W_G = R*Uw;
            sharedSEN->IMU[0].Wx_G = W_G.v[0];
            sharedSEN->IMU[0].Wy_G = W_G.v[1];
            sharedSEN->IMU[0].Wz_G = W_G.v[2];

            Vector3_InEKF A_G = R*Ua;
            sharedSEN->IMU[0].Ax_G = A_G.v[0];
            sharedSEN->IMU[0].Ay_G = A_G.v[1];
            sharedSEN->IMU[0].Az_G = A_G.v[2];
        }
    }
}
