#ifndef IKF_H
#define IKF_H

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
inline Vector3_InEKF operator*(const double &a,const Vector3_InEKF &in) {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = a*in.v[i];
    }
    return out;
}
inline Vector3_InEKF Ones_Vec3() {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = 1.0;
    }
    return out;
}
inline Vector3_InEKF Zero_Vec3() {
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
inline Matrix3_InEKF Zero_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = 0.0;
        }
    }
    return out;
}
inline Matrix3_InEKF Identity_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            if(i==j) out.M[i][j] = 1.0;
            else out.M[i][j] = 0.0;
        }
    }
    return out;
}
inline Matrix3_InEKF operator*(const double &a, const Matrix3_InEKF &in) {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = a*in.M[i][j];
        }
    }
    return out;
}
inline Matrix3_InEKF hat_oper(const Vector3_InEKF in) {
    double a = in.v[0];
    double b = in.v[1];
    double c = in.v[2];
    Matrix3_InEKF out(0.0,   -c,    b,
                      c,  0.0,   -a,
                      -b,    a,  0.0);
    return out;
}
inline Matrix3_InEKF expM(const Matrix3_InEKF M) {
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

inline Matrix3_InEKF exp_so3(Vector3_InEKF v) {
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

    Matrix3_InEKF R_Offset;

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
        if(t<(double)(RT_TIMER_PERIOD_US/1000)/1000.0/2.0) {
            Initialize(_Ua);
        }

        // prediction step - update all the state using the gyroscope data
        double dT = (double)(RT_TIMER_PERIOD_US/1000)/1000.0;
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
            t = 0.0;
        }
    }
};


#endif // IKF_H
