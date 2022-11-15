#include "QUAD_BasicFunction.h"

Vector3d zv = Vector3d::Zero(); // Zero vector
Matrix3d I3 = Matrix3d::Identity(); // 3x3 Identity matrix

//==============================//
// Saving function
//==============================//
void save_File(char *filecomment)
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "QuadEffTest_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    if(!filecomment[0] == NULL) {
        strcat(filename, "_");
        strcat(filename, filecomment);
    }
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<save_Index ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    save_Index=0;
    save_Flag=0;

    std::cout << "Saved Filename : " << filename << std::endl;
}

// ==================================================================================
// Lowpass-filter Functions
// ==================================================================================
// ref : https://dsp.stackexchange.com/questions/8693/how-does-a-low-pass-filter-programmatically-work
// ref2 : https://www.electronics-tutorials.ws/filter/filter_8.html
VectorNd Vector_LPF_2nd(VectorNd voo, VectorNd vo, VectorNd v_new, double f_cut)
{
    VectorNd v_updated = VectorNd::Zero(vo.rows());

    if(vo.rows()!=v_new.rows()) {
        FILE_LOG(logERROR) << vo.rows();
        FILE_LOG(logERROR) << v_new.rows();
        FILE_LOG(logERROR) << "Vector size mismatch.";
        return v_updated;
    } else {
        for(int i=0;i<vo.rows();i++) {
            double w_cut = 2.0*PI*f_cut;
            double alpha1 = -(SYS_FREQ*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
            double alpha2 = (2.0*SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
            double alpha3 = (w_cut*w_cut)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);

            v_updated(i)=alpha1*voo(i)+alpha2*vo(i)+alpha3*v_new(i);
        }
        return v_updated;
    }
}

void PrintHere(int n=0)
{
    static int cnt = 0;
    if(n == 0) {
        FILE_LOG(logDEBUG1) << "I'm Here : " << cnt;
        cnt++;
    } else {
        FILE_LOG(logDEBUG) << "I'm Here : " << n;
    }
}



// ==================================================================================
// Trajectory Functions
// ==================================================================================

void PolyTrajectory_1st(double tf, double tn,
                    double x, double xf,
                    double &xn, double &dxn, double &ddxn)
{
    // t : Left time
    // tn : next time
    // x : current position
    // xf : final position
    // xn,dxn,ddxn : next step position, velocity, acceleration

    double k[2] = {0.0, }; // Coefficient for '1st order polynomial trajectory'

    if(tf < 1e-6)
    {
        xn = x;
        dxn = 0.0;
        ddxn = 0.0;
    } else if (tf < tn) {
        xn = xf;
        dxn = 0.0;
        ddxn = 0.0;
    } else {
        double t = tf;
        k[0] = x;
        k[1] = (xf-x)/t;

        double dt = tn;
        xn = k[0] + k[1]*dt;
        dxn = k[1];
        ddxn = 0.0;
    }
}

void PolyTrajectory_3rd(double tf, double tn,
                    double x, double dx, double xf, double dxf,
                    double &xn, double &dxn, double &ddxn)
{
    // t : Left time
    // tn : next time
    // x, dx : current position, velocity
    // xf, dxf : final position, velocity
    // xn,dxn,ddxn : next step position, velocity, acceleration

    double k[4] = {0.0, }; // Coefficient for '3rd order polynomial trajectory'

    if(tf < 1e-6)
    {
        xn = x;
        dxn = dx;
        ddxn = 0.0;
    } else if (tf <= tn) {
        xn = xf;
        dxn = dxf;
        ddxn = 0.0;
    } else {
        double t = tf;
        k[0] = x;
        k[1] = dx;
        k[2] = (3.0*(xf-x) - (1.0*dxf + 2.0*dx)*t)/(t*t);
        k[3] = (-2.0*(xf-x) + (1.0*dxf + 1.0*dx)*t)/(t*t*t);

        double dt = tn;
        xn = k[0] + k[1]*dt + k[2]*dt*dt + k[3]*dt*dt*dt;
        dxn = k[1] + 2.0*k[2]*dt + 3.0*k[3]*dt*dt;
        ddxn = 2.0*k[2] + 6.0*k[3]*dt;
    }
}


void PolyTrajectory_5th(double tf, double tn,
                    double x, double dx, double ddx, double xf, double dxf, double ddxf,
                    double &xn, double &dxn, double &ddxn)
{
    // t : Left time
    // tn : next time
    // x, dx, ddx : current position, velocity, acceleration
    // xf, dxf, ddxf : final position, velocity, acceleration
    // xn,dxn,ddxn : next step position, velocity, acceleration

    double k[6] = {0.0, }; // Coefficient for '3rd order polynomial trajectory'

    if(tf < 1e-6)
    {
        xn = x;
        dxn = dx;
        ddxn = ddx;
    } else if (tf <= tn) {
        xn = xf;
        dxn = dxf;
        ddxn = ddxf;
    } else {
        double t = tf;
        k[0] = x;
        k[1] = dx;
        k[2] = ddx/2.;
        k[3] = (20.*(xf-x) - (8.*dxf + 12.*dx)*t-(3.*ddx - ddxf)*t*t)/(2.*t*t*t);
        k[4] = ((30.*x - 30.*xf) + (14.*dxf+16.*dx)*t+(3.*ddx-2.*ddxf)*t*t)/(2.*t*t*t*t);
        k[5] = (12.*xf - 12.*x - (6.*dxf + 6.*dx)*t - (ddx-ddxf)*t*t)/(2.*t*t*t*t*t);

        double dt = tn;
        xn = k[0] + k[1]*dt  + k[2]*dt*dt + k[3]*dt*dt*dt + k[4]*dt*dt*dt*dt + k[5]*dt*dt*dt*dt*dt;
        dxn = k[1] + 2.0*k[2]*dt + 3.0*k[3]*dt*dt + 4.0*k[4]*dt*dt*dt + 5.0*k[5]*dt*dt*dt*dt;
        ddxn = 2.0*k[2] + 6.0*k[3]*dt + 12.0*k[4]*dt*dt + 20.0*k[5]*dt*dt*dt;
    }
}


void PolyTrajectory_Vector_1st(double tf, double tn,
                          Vector3d X, Vector3d Xf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        PolyTrajectory_1st(tf,tn,
                       X(i),Xf(i),
                       _x,_dx,_ddx);
        Xn(i) = _x;
        dXn(i) = _dx;
        ddXn(i) = _ddx;
    }
}

void PolyTrajectory_Vector_3rd(double tf, double tn,
                          Vector3d X, Vector3d dX, Vector3d Xf, Vector3d dXf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        PolyTrajectory_3rd(tf,tn,
                       X(i),dX(i),Xf(i),dXf(i),
                       _x,_dx,_ddx);
        Xn(i) = _x;
        dXn(i) = _dx;
        ddXn(i) = _ddx;
    }
}

void PolyTrajectory_Vector_5th(double tf, double tn,
                          Vector3d X, Vector3d dX, Vector3d ddX, Vector3d Xf, Vector3d dXf, Vector3d ddXf,
                          Vector3d &Xn, Vector3d &dXn, Vector3d &ddXn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        PolyTrajectory_5th(tf,tn,
                       X(i),dX(i),ddX(i),Xf(i),dXf(i),ddXf(i),
                       _x,_dx,_ddx);
        Xn(i) = _x;
        dXn(i) = _dx;
        ddXn(i) = _ddx;
    }
}

// p0 : Initial point
// p1 : Final point
void BezierTrajectory_1st(double tf, double tn,
                          double p0, double p1,
                          double &pn, double &dpn, double &ddpn)
{
    // p0 : p
    // p1 : pf

    if(tf < 1e-6)
    {
        pn = p0;
        dpn = (p1-p0)/tf;
        ddpn = 0.0;
    } else if (tf < tn) {
        pn = p1;
        dpn = (p1-p0)/tf;
        ddpn = 0.0;
    } else {
        pn = (p0*(tf-tn) + p1*tn)/tf;
        dpn = (p1-p0)/tf;
        ddpn = 0.0;
    }
}

// p0 : Initial point
// p1, p2 : Way point
// p3 : Final point
void BezierTrajectory_3rd(double tf, double tn,
                          double p0, double p1, double p2, double p3,
                          double &pn, double &dpn, double &ddpn)
{
    // p0 : p
    // p1 : p + dp*tf/3.0
    // p2 : pf - dpf*tf/3.0
    // p3 : pf

    if(tf < 1e-6)
    {
        pn = p0;
        dpn = 3.0*(p1-p0)/tf;
        ddpn = 6.0*(p2-2.0*p1+p0)/(tf*tf);
    } else if (tf < tn) {
        pn = p3;
        dpn = 3.0*(p3-p2)/tf;;
        ddpn = 6.0*(p3-2.0*p2+p1)/(tf*tf);
    } else {
        pn = (p0*(tf-tn)*(tf-tn)*(tf-tn) + p1*3.0*(tf-tn)*(tf-tn)*tn + p2*3.0*(tf-tn)*tn*tn + p3*tn*tn*tn)/(tf*tf*tf);
        dpn = (p0*(-3.0*(tf-tn)*(tf-tn))
             + p1*(3.0*(tf-tn)*(tf-tn)-6.0*(tf-tn)*tn)
             + p2*(-3.0*tn*tn+6.0*(tf-tn)*tn)
             + p3*(3.0*tn*tn))/(tf*tf*tf);
        ddpn = (p0*(6.0*(tf-tn))
              + p1*(-12.0*(tf-tn)+6.0*tn)
              + p2*(-12.0*tn+6.0*(tf-tn))
              + p3*(6.0*tn))/(tf*tf*tf);
    }
}

// p0 : Initial point
// p1~p4 : Way point
// p5 : Final point
void BezierTrajectory_5th(double tf, double tn,
                          double p0, double p1, double p2, double p3, double p4, double p5,
                          double &pn, double &dpn, double &ddpn)
{
    // p0 : p
    // p1 : p + dp*tf/5.0
    // p2 : p + 2.0*dp*tf/5.0 + ddp*tf*tf/20.0
    // p3 : pf - 2.0*dpf*tf/5.0 + ddpf*tf*tf/20.0
    // p4 : pf - dpf*tf/5.0
    // p5 : pf

    if(tf < 1e-6)
    {
        pn = p0;
        dpn = 5.0*(p1-p0)/tf;
        ddpn = 20.0*(p2-2.0*p1+p0)/(tf*tf);
    } else if (tf < tn) {
        pn = p5;
        dpn = 5.0*(p5-p4)/tf;
        ddpn = 20.0*(p5-2.0*p4+p3)/(tf*tf);
    } else {
        pn = (p0*(tf-tn)*(tf-tn)*(tf-tn)*(tf-tn)*(tf-tn)
            + p1*5.0*(tf-tn)*(tf-tn)*(tf-tn)*(tf-tn)*tn
            + p2*10.0*(tf-tn)*(tf-tn)*(tf-tn)*tn*tn
            + p3*10.0*(tf-tn)*(tf-tn)*tn*tn*tn
            + p4*5.0*(tf-tn)*tn*tn*tn*tn
            + p5*tn*tn*tn*tn*tn)/(tf*tf*tf*tf*tf);
        dpn = (p0*(-5.0*(tf-tn)*(tf-tn)*(tf-tn)*(tf-tn))
             + p1*(-20.0*(tf-tn)*(tf-tn)*(tf-tn)*tn+5.0*(tf-tn)*(tf-tn)*(tf-tn)*(tf-tn))
             + p2*(-30.0*(tf-tn)*(tf-tn)*tn*tn+20.0*(tf-tn)*(tf-tn)*(tf-tn)*tn)
             + p3*(-20.0*(tf-tn)*tn*tn*tn+30.0*(tf-tn)*(tf-tn)*tn*tn)
             + p4*(-5.0*tn*tn*tn*tn+20.0*(tf-tn)*tn*tn*tn)
             + p5*(5.0*tn*tn*tn*tn))/(tf*tf*tf*tf*tf);
        ddpn = (p0*(20.0*(tf-tn)*(tf-tn)*(tf-tn))
              + p1*(-40.0*(tf-tn)*(tf-tn)*(tf-tn)+60.0*(tf-tn)*(tf-tn)*tn)
              + p2*(60.0*(tf-tn)*tn*tn-120.0*(tf-tn)*(tf-tn)*tn+20.0*(tf-tn)*(tf-tn)*(tf-tn))
              + p3*(20.0*tn*tn*tn-120.0*(tf-tn)*tn*tn+60.0*(tf-tn)*(tf-tn)*tn)
              + p4*(-40.0*tn*tn*tn+60.0*(tf-tn)*tn*tn)
              + p5*(20.0*tn*tn*tn))/(tf*tf*tf*tf*tf);
    }
}


void BezierTrajectory_Vector_1st(double tf, double tn,
                                Vector3d P0, Vector3d P1,
                                Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        BezierTrajectory_1st(tf,tn,
                             P0(i),P1(i),
                             _x,_dx,_ddx);
        Pn(i) = _x;
        dPn(i) = _dx;
        ddPn(i) = _ddx;
    }
}

void BezierTrajectory_Vector_3rd(double tf, double tn,
                                 Vector3d P0, Vector3d P1, Vector3d P2, Vector3d P3,
                                 Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        BezierTrajectory_3rd(tf,tn,
                             P0(i),P1(i),P2(i),P3(i),
                             _x,_dx,_ddx);
        Pn(i) = _x;
        dPn(i) = _dx;
        ddPn(i) = _ddx;
    }
}

void BezierTrajectory_Vector_5th(double tf, double tn,
                                 Vector3d P0, Vector3d P1, Vector3d P2, Vector3d P3, Vector3d P4, Vector3d P5,
                                 Vector3d &Pn, Vector3d &dPn, Vector3d &ddPn)
{
    double _x,_dx,_ddx;
    for(int i=0;i<3;i++) {
        BezierTrajectory_5th(tf,tn,
                             P0(i),P1(i),P2(i),P3(i),P4(i),P5(i),
                             _x,_dx,_ddx);
        Pn(i) = _x;
        dPn(i) = _dx;
        ddPn(i) = _ddx;
    }
}

// /////////////////////////////////////////////////////////////////////////////////////////////////
// QP Setup Function
// ///////////////////////////////////////////////////////////////////////////////////////////////

void QuadEffTest_QP::setNums(int _xlength, int numCost, int numEqConstraints, int numIneqConstraints)
{
    NUMCOLS = _xlength;
    NUMCOST = numCost;
    NUMEQ = numEqConstraints;
    NUMINEQ = numIneqConstraints;

    //maybe matrix assignment here
    X.resize(_xlength,1);
    X = MatrixNd::Zero(NUMCOLS,1);
    A_equ.resize(NUMEQ, NUMCOLS);
    A_equ = MatrixNd::Zero(NUMEQ, NUMCOLS);
    B_equ.resize(NUMEQ, 1);
    B_equ = MatrixNd::Zero(NUMEQ, 1);

    A_inequ.resize(NUMINEQ, NUMCOLS);
    A_inequ = MatrixNd::Zero(NUMINEQ, NUMCOLS);
    B_inequ.resize(NUMINEQ, 1);
    B_inequ = MatrixNd::Zero(NUMINEQ, 1);

    P_cost.resize(NUMCOLS,NUMCOLS);
    P_cost = MatrixNd::Zero(NUMCOLS,NUMCOLS);
    Q_cost.resize(NUMCOLS,1);
    Q_cost = MatrixNd::Zero(NUMCOLS,1);
}

void QuadEffTest_QP::make_EQ(MatrixNd A, MatrixNd b)
{
    //Aeq*X = Beq
    A_equ = A;
    B_equ = b;
}

void QuadEffTest_QP::update_Bequ(MatrixNd b)
{
    B_equ = b;
}

void QuadEffTest_QP::make_IEQ(MatrixNd A, MatrixNd b)
{
    // Aineq*X < Bineq
    A_inequ = A;
    B_inequ = b;
}

void QuadEffTest_QP::update_Binequ(MatrixNd b)
{
    B_inequ = b;
}

void QuadEffTest_QP::make_COST(MatrixNd A, MatrixNd b)
{
    //min (0.5* x P x + Q' x) << (0.5*x(AT*A)x - (AT*b)*x + bT*b) << (0.5*|Ax-b|^2)
    A_cost = A; // m-by-n matrix
    B_cost = b;
    int m = A_cost.transpose().cols();
    int n = A_cost.transpose().rows();
    for(int i=0;i<n;i++) {
        P_cost.block(i,0,1,n) = A_cost.transpose().block(i,0,1,m) * A_cost;
    }
    Q_cost = -A_cost.transpose() * B_cost;
}

void QuadEffTest_QP::update_Bcost(MatrixNd b)
{
    B_cost = b;
    Q_cost = -A_cost.transpose() * B_cost;
}

void QuadEffTest_QP::make_COST_direct(MatrixNd _P, MatrixNd _Q)
{
    //min (0.5* x P x + Q'x)
    P_cost = _P;
    Q_cost = _Q;
}

MatrixNd QuadEffTest_QP::solve_QP()
{
    quadprogpp::Vector<double> outX;
    quadprogpp::Matrix<double> G;
    quadprogpp::Vector<double> g0;
    quadprogpp::Matrix<double> CE;
    quadprogpp::Vector<double> ce0;
    quadprogpp::Matrix<double> CI;
    quadprogpp::Vector<double> ci0;
    //min 0.5 * x G x + g0 x
    //CE^T x + ce0 = 0
    //CI^T x + ci0 >= 0

    G.resize(NUMCOLS,NUMCOLS);
    g0.resize(NUMCOLS);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMCOLS;j++)
        {
            G[i][j] = P_cost(i,j);
        }
        g0[i] = Q_cost(i,0);
    }
    CE.resize(NUMCOLS,NUMEQ);
    ce0.resize(NUMEQ);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMEQ;j++)
        {
            CE[i][j] = -A_equ(j,i);
        }
    }
    for(int j=0;j<NUMEQ;j++)
    {
        ce0[j] = B_equ(j,0);
    }
    CI.resize(NUMCOLS,NUMINEQ);
    ci0.resize(NUMINEQ);
    for(int i=0;i<NUMCOLS;i++)
    {
        for(int j=0;j<NUMINEQ;j++)
        {
            CI[i][j] = -A_inequ(j,i);
        }
    }
    for(int j=0;j<NUMINEQ;j++)
    {
        ci0[j] = B_inequ(j,0);
    }
    outX.resize(NUMCOLS);

    solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
    for(int i=0;i<NUMCOLS;i++)
    {
        X(i,0) = outX[i];
    }

    return X;

}

qp_int QuadEffTest_QP::GetNumberOfNonZero_QPswift(MatrixNd mat)
{
    int n_cols = mat.cols();
    int n_rows = mat.rows();

    qp_int nnz = 0; // the number of non-zero components
    for(int i=0; i<n_cols; i++)
    {
        for(int j=0; j<n_rows; j++)
        {
            if(fabs(mat(j,i)) > zero_threshold)
            {
                nnz++;
            }
        }
    }
    return nnz;
}

void QuadEffTest_QP::ConvertMatrixA_Full2CCS_QPswift(MatrixNd A, qp_real* A_x, qp_int* A_i, qp_int* A_p)
{
    // ref : https://ko.wikipedia.org/wiki/%ED%9D%AC%EC%86%8C%ED%96%89%EB%A0%AC

    // The real array A_x holds all the nonzero entries of A in column major format
    // The integer array A_i holds the rows indices of the corresponding elements in A_x
    // The integer array A_p is defined as
    //   – A_p[0] = 0
    //   – A_p[i] = A_p[i − 1] + number of non-zeros in i th column of A

    //    For the following sample matrix A,
    //    A =  [ 4.5 0   3.2 0
    //           3.1 2.9 0   2.9
    //           0   1.7 3.0 0
    //           3.5 0.4 0   1.0 ]

    //    we have the following CCS representation
    //    >> double A_x ={4.5, 3.1, 3.5, 2.9, 1.7, 0.4, 3.2, 3.0, 2.9, 1.0}
    //    >> int A_i ={0, 1, 3, 1, 2, 3, 0, 2, 1, 3}
    //    >> int A_p ={0, 3, 6, 8, 10}

    int n_cols = A.cols();
    int n_rows = A.rows();

    int cnt_A_x = 0;
    int cnt_A_i = 0;
    int cnt_A_p = 0;
    A_p[0] = 0;

    for(int i=0; i<n_cols; i++)
    {
        for(int j=0; j<n_rows; j++)
        {
            if(fabs(A(j,i)) > zero_threshold)
            {
                // Get A_x
                A_x[cnt_A_x] = A(j,i);
                cnt_A_x++;

                // Get A_i
                A_i[cnt_A_i] = j;
                cnt_A_i++;

                // Get A_p
                cnt_A_p++;
            }
        }
        A_p[i+1] = cnt_A_p;
    }
}

void QuadEffTest_QP::ConvertMatrixP_Full2CCS_QPswift(MatrixNd P, qp_real* P_x, qp_int* P_i, qp_int* P_p)
{
    // Convert Symmetric Matrix P >> Upper Triangular Matrix
    // and Make it CCS format

    int n_cols = P.cols();

    int cnt_P_x = 0;
    int cnt_P_i = 0;
    int cnt_P_p = 0;
    P_p[0] = 0;

    for(int i=0; i<n_cols; i++)
    {
        for(int j=0; j<i+1; j++)
        {
            if(fabs(P(j,i)) > zero_threshold)
            {
                // Get P_x
                P_x[cnt_P_x] = P(j,i);
                cnt_P_x++;

                // Get P_i
                P_i[cnt_P_i] = j;
                cnt_P_i++;

                // Get P_p
                cnt_P_p++;
            }
        }
        P_p[i+1] = cnt_P_p;
    }
}


void QuadEffTest_QP::ConvertVector2Array_QPswift(VectorNd vec, qp_int n, qp_real* v)
{
    for(int i=0;i<n;i++) {
        v[i] = vec(i);
    }
}

void QuadEffTest_QP::ConvertArray2Vector_QPswift(qp_real* v, qp_int n, VectorNd &vec)
{
    for(int i=0;i<n;i++) {
        vec(i) = v[i];
    }
}

void QuadEffTest_QP::Example_Problem()
{
    ///////////////////////////////////
    /// Problem

    int n_var = 2;
    int n_cost = 2;
    int n_inequ = 0;
    int n_equ = 1;

    MatrixNd A_cost(2,2);
    MatrixNd B_cost(2,1);
    A_cost << 4.0,1.0,
              1.0,2.0;
    B_cost << 1.0,
              1.0;

    MatrixNd A_inequ(n_inequ,n_var);
    MatrixNd Bl_inequ(n_inequ,1);
    MatrixNd Bu_inequ(n_inequ,1);
//        A_inequ <<  1.0, 0.0,
//                    0.0, 1.0;
//        Bl_inequ << 0.0, 0.0;
//        Bu_inequ << 0.7, 0.7;

    MatrixNd A_equ(n_equ,n_var);
    MatrixNd B_equ(n_equ,1);
    A_equ << 1.0, 1.0;
    B_equ << 1.0;

    VectorNd sol = VectorNd::Zero(n_var);
    /// Answer : 0.25 0.75
    ///////////////////////////////////////
    system_clock::time_point StartTime_QPSwift = system_clock::now();

    qp_int     P_nnz,A_nnz,G_nnz;
    qp_real  *P_x;
    qp_int     *P_i,*P_p;
    qp_real  *q_x;
    qp_real  *A_x;
    qp_int     *A_i,*A_p;
    qp_real  *b_x;
    qp_real  *G_x;
    qp_int     *G_i,*G_p;
    qp_real  *h_x;

    // 1. Define the cost functions.
    MatrixNd P = A_cost;
    VectorNd q = B_cost;
    P_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(P);
    P_x = (qp_real*)malloc(P_nnz*sizeof(qp_real));
    P_i = (qp_int*)malloc(P_nnz*sizeof(qp_int));
    P_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
    this->ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
    q_x = (qp_real*)malloc(n_var*sizeof(qp_real));
    this->ConvertVector2Array_QPswift(q,n_var,q_x);

    // 2. Define the equality constraints.
    MatrixNd A = A_equ;
    VectorNd b = B_equ;
    A_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(A);
    A_x = (qp_real*)malloc(A_nnz*sizeof(qp_real));
    A_i = (qp_int*)malloc(A_nnz*sizeof(qp_int));
    A_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
    this->ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
    b_x = (qp_real*)malloc(n_equ*sizeof(qp_real));
    this->ConvertVector2Array_QPswift(b,n_equ,b_x);

    // 3. Define the inequality constraints.
    MatrixNd G = MatrixNd::Zero(2*n_inequ,n_var);
    MatrixNd h = MatrixNd::Zero(2*n_inequ,1);
    G.block(0,0,n_inequ,n_var) = -A_inequ;
    G.block(n_inequ,0,n_inequ,n_var) = A_inequ;
    h.block(0,0,n_inequ,1) = -Bl_inequ;
    h.block(n_inequ,0,n_inequ,1) = Bu_inequ;
    G_nnz = (qp_int)this->GetNumberOfNonZero_QPswift(G);
    G_x = (qp_real*)malloc(G_nnz*sizeof(qp_real));
    G_i = (qp_int*)malloc(G_nnz*sizeof(qp_int));
    G_p = (qp_int*)malloc((n_var+1)*sizeof(qp_int));
    this->ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
    h_x = (qp_real*)malloc((2*n_inequ)*sizeof(qp_real));
    this->ConvertVector2Array_QPswift(h,(2*n_inequ),h_x);

//        QPswift *myQP;
//        myQP = QP_SETUP(n_var,n_inequ,n_equ, P_p,P_i,P_x, A_p,A_i,A_x, G_p,G_i,G_x,q_x,h_x,b_x, 0.0,nullptr);

//        // 5. Solve.
//        int ExitCode = QP_SOLVE(myQP);

    QPswift *myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                     P_p,P_i,P_x,
                                     A_p,A_i,A_x,
                                     q_x,b_x,
                                     0.0,nullptr);
    QP_SOLVE_NoInequ(myQP);
    this->ConvertArray2Vector_QPswift(myQP->x, n_var, sol);

    QP_CLEANUP(myQP);
    free(P_x);  free(P_i);  free(P_p);
    free(A_x);  free(A_i);  free(A_p);
    free(G_x);  free(G_i);  free(G_p);
    free(q_x);  free(b_x);  free(h_x);

    system_clock::time_point EndTime_QPSwift = system_clock::now();
    FILE_LOG(logINFO) << "qpSWIFT Solution : " << sol.transpose();

    ///////////////////////////////////////
    system_clock::time_point StartTime_QuadProg = system_clock::now();

    // 1. Define the number of Variables.
    this->setNums(n_var,n_cost,n_equ,2*n_inequ);

    // 2. Define the cost functions.
    this->make_COST_direct(A_cost, B_cost);

    // 3. Define the equality constraints.
    this->make_EQ(A_equ, B_equ);

    // 4. Define the inequality constraints.
    MatrixNd Ai = MatrixNd::Zero(2*n_inequ,n_var);
    MatrixNd Bi = MatrixNd::Zero(2*n_inequ,1);
    Ai.block(0,0,n_inequ,n_var) = -A_inequ;
    Ai.block(n_inequ,0,n_inequ,n_var) = A_inequ;
    Bi.block(0,0,n_inequ,1) = -Bl_inequ;
    Bi.block(n_inequ,0,n_inequ,1) = Bu_inequ;
    this->make_IEQ(Ai,Bi);

    // 5. Solve.
    sol = this->solve_QP();
    system_clock::time_point EndTime_QuadProg = system_clock::now();
    FILE_LOG(logINFO) << "QuadProg Solution : " << sol.transpose();

    //////////////////////////////////////////////////////////////////////////
//        microseconds t_calc_OSQP = duration_cast<std::chrono::microseconds>(EndTime_OSQP - StartTime_OSQP);
    microseconds t_calc_QPSwift = duration_cast<std::chrono::microseconds>(EndTime_QPSwift - StartTime_QPSwift);
    microseconds t_calc_QuadProg = duration_cast<std::chrono::microseconds>(EndTime_QuadProg - StartTime_QuadProg);
//        FILE_LOG(logDEBUG) << "Time OSQP : "<< t_calc_OSQP.count() <<" usecond(s).";
    FILE_LOG(logDEBUG) << "Time QPSwift : "<< t_calc_QPSwift.count() <<" usecond(s).";
    FILE_LOG(logDEBUG) << "Time QuadProg : "<< t_calc_QuadProg.count() <<" usecond(s).";

}

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B) {
    A.resize(m,n);
    A = MatrixNd::Zero(m,n);
    B.resize(m,1);
    B = MatrixNd::Zero(m,1);
}

void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new) {
    int m_now = A_base.rows();
    int n_new = A_new.cols();
    int m_new = A_new.rows();

    A_base.conservativeResize(m_now+m_new,n_new);
    B_base.conservativeResize(m_now+m_new,1);
    A_base.block(m_now,0,m_new,n_new) = A_new;
    B_base.block(m_now,0,m_new,1) = B_new;
}
