#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
//extern pUSER_SHM                userData;

PumpControl_QP  QP_PsCtrl(SolverIsQuadProg);
extern double wp_ref_max;
extern double wp_ref_min;
extern double Ps_des[MAX_PREVIEW+1];

void PumpPressure_VirtualRefMotion(double t_now, double dT_Prev, VectorNd &Psdes_Future, VectorNd &Qdes_Future)
{
    // Simulation Scenario : Squat Motion with 2-DoF Legs

    double Do = 30; // Rotary actuator outer diameter [mm]
    double Di = 20; // Rotary actuator inner diameter [mm]
    double Dt = 20; // Rotary actuator thickness [mm]
    double rA_act = (Do+Di)/2.0*(Do-Di)*Dt*2.0; // [mm^3]

    double m_load = 20.0;   // Body Weight [kg]
    double L_leg = 0.40;    // Leg Length [m]
    double h_ini = 0.78;    // Squat Motion Initial Height [m]
    double h_mag = -0.30;   // Squat Motion Height Difference [m]
    double T_squat = 2.50;  // Squat Motion Period [s]
    double t_start = 2.0;

    int N = Psdes_Future.size();
    double t_prev = t_now;
    for(int i=0;i<N;i++) {
        double h,dh,ddh;
        if(t_prev < t_start) {
            h = h_ini;
            dh = 0.0;
            ddh = 0.0;
        } else {
            h = h_ini + (h_mag/2.0)*(1.0-cos(2.0*PI*(t_prev-t_start)/T_squat));
            dh = (h_mag/2.0)*(2.0*PI/T_squat)*sin(2.0*PI*(t_prev-t_start)/T_squat);
            ddh = (h_mag/2.0)*(2.0*PI/T_squat)*(2.0*PI/T_squat)*cos(2.0*PI*(t_prev-t_start)/T_squat);
        }

        double F_push = m_load*(g_const+ddh);
        double th = 2.0*asin(h/(2.0*L_leg));
        double dth = dh/(L_leg*cos(th/2.0f));
        double T = F_push*L_leg*cos(th/2.0f); // [Nm]

        Qdes_Future(i) = 4.0*fabs(rA_act/1000000.0*dth*60.0); // [LPM]
        Psdes_Future(i) = fabs(T/rA_act*10000.0); // [bar]
        if(Psdes_Future(i)<Ps_min) { Psdes_Future(i) = Ps_min; }
        t_prev += dT_Prev;
    }

}

void PumpPressure_VirtualDynamics(double Ps_now, double wp_now, double Q_act, double dT_SIM, double& Ps_next)
{
    if(wp_now > wp_ref_max/60.0*2.0*PI) {
        wp_now = wp_ref_max/60.0*2.0*PI;
    } else if (wp_now < wp_ref_min/60.0*2.0*PI) {
        wp_now = wp_ref_min/60.0*2.0*PI;
    }

    double Ps = Ps_now;
    double wp = wp_now;

    double k = n_gas/(pow(P_pre,(1.0/n_gas))*V_pre);

    double q_p = OutputFlowPerRev / (2.0 * PI);  // Pump Displacement [L/rad]
    double n_act = 4.0; // Number of actuators

    int integration_grid = 30;
    double Q_pump, Q_leak;
    for (int i = 0; i < integration_grid; i++) {
        Q_pump = q_p * wp * 60.0;
        Q_leak = n_act * K_leak * Ps;
        double dPs = k * pow(Ps,(n_gas+1.0)/n_gas)* (Q_pump - Q_act - Q_leak) / 60.0; // [bar/s]

        // Integration
        Ps = Ps + dPs * dT_SIM / (double)integration_grid;
    }

//    double Ps_rand = (((double)rand()/(double)RAND_MAX)-0.5)*1.0; // Pressure Measurement Noise
//    Ps_next = Ps + Ps_rand;
    Ps_next = Ps;
}

void PumpPressureController_LinearMPC(int N, double dT,
                                      double Ps_now, VectorNd Ps_des_window, VectorNd Qact_des_window,
                                      VectorNd &X_MPC, VectorNd &U_MPC)
{
    const int n_X = X_MPC.size();
    const int n_U = U_MPC.size();
    const int n_var = n_X + n_U;
    const int idx_X = 0;
    const int idx_U = n_X;
    QP_PsCtrl.n_X = n_X;
    QP_PsCtrl.n_U = n_U;
    QP_PsCtrl.n_var = n_var;

    MatrixNd A_temp;
    VectorNd B_temp;

    //////// Cost Functions /////////////////////////////////////////////////////////////////////////
    /// 1) Supply Pressure Condition (Ps > Ps,ref)  [Soft Constraint]
    /// 2) Power minimization
    ///    W_loss = (mu*wp+fc)*wp + KL*Ps*Ps
    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Cost Function Weight
    double W_Tracking = 10000.0;
    double W_SpeedLossMin = 100.0;
    double W_LeakLossMin = 100.0;
    double W_SolChangeMin = 5.0;
    double W_SpeedChangeMin = 0.3;
    double W_PressureChangeMin = 15.0*(Ps_now/Ps_min)*(Ps_now/Ps_min)*(Ps_now/Ps_min);

    int n_cost = 0;
    int idx_cost = 0;
    int temp_size_cost = 0;
    MatrixNd A_cost;
    VectorNd B_cost;
    double W_temp = 0.0;

    // 1. Supply Pressure Tracking + Penalty
    W_temp = W_Tracking;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,idx_X+i) = 1.0;
        A_temp(i,idx_U+2*i+1) = -1.0;
        B_temp(i) = Ps_des_window(i+1);
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 2. Speed Loss Minimization [Watt]
    W_temp = W_SpeedLossMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        double mu = 0.0051166;
        double fc = 0.56983;
        A_temp(i,idx_U+2*i) = sqrt(mu);
        B_temp(i) = -fc/(2.0*sqrt(mu));
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 3. Pressure Loss Minimization [Watt]
    W_temp = W_LeakLossMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        double n_act = 13.0;
        double KL = n_act*K_leak;
        A_temp(i,idx_X+i) = sqrt(KL*100.0/60.0);
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 4. Pump Speed Solution Change Minimization
    W_temp = W_SolChangeMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,idx_U+2*i) = 1.0;
        B_temp(i,0) = U_MPC(2*i);
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 5. Pump Speed Change Minimization (Acceleration)
    W_temp = W_SpeedChangeMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N-1;i++) {
        A_temp(i,idx_U+2*i) = -1.0/dT;
        A_temp(i,idx_U+2*(i+1)) = 1.0/dT;
        B_temp(i) = 0.0;
    }
    A_temp(N-1,idx_U) = 1.0/SYS_DT_PUMPING;
    B_temp(N-1) = U_MPC(0)/SYS_DT_PUMPING;
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

//    // 6. Pressure Change Minimization
//    W_temp = W_PressureChangeMin;
//    temp_size_cost = N-1;
//    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
//    for(int i=0;i<N-1;i++) {
//        A_temp(i,idx_X+i) = -1.0/dT;
//        A_temp(i,idx_X+i+1) = 1.0/dT;
//        B_temp(i) = 0.0;
//    }
//    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
//    idx_cost += temp_size_cost;

    // ////////////////////////////////////////////////////////////
    // Cost Function End

    n_cost = idx_cost;

//    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//    std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

    //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////

    /// 1) Model Predictive Control :
    /// X_MPC = [x_k+1; x_k+2; ...; x_k+N]
    /// A_MPC = [A^1; A^2; ...; A^N]
    /// B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
    /// U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
    ///    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;
    ///    >> X_MPC - B_MPC*U_MPC = -A_MPC*_xnow;
    ///    >> [I -B_MPC]*[X_MPC;U_MPC] = -A_MPC*_xnow;
    /////////////////////////////////////////////////////////////////////////////////////////////////

    int n_equ =0;
    int idx_equ = 0;
    int temp_size_equ = 0;
    MatrixNd A_equ;
    VectorNd B_equ;

    double k = n_gas/(pow(P_pre,(1.0/n_gas))*V_pre);

    double q_p = OutputFlowPerRev / (2.0 * PI);  // Pump Displacement [L/rad]
    double n_act = 4.0; // Number of actuators
    VectorNd Ps_op = VectorNd::Zero(N+1);
    VectorNd wp_op = VectorNd::Zero(N);
    Ps_op(0) = Ps_now;
    for(int i=0;i<N;i++) {
        Ps_op(i+1) = X_MPC(i);
        wp_op(i) = U_MPC(2*i);
    }

    MatrixNd Ac = MatrixNd::Zero(1,1);
    MatrixNd Bc = MatrixNd::Zero(1,2);
    MatrixNd Ec = MatrixNd::Zero(1,1);
    Ac(0,0) = ((n_gas+1.0)/n_gas)*k*pow(Ps_op(0),(1.0/n_gas))*(q_p*wp_op(0)*60.0 - Qact_des_window(0) - ((2.0*n_gas+1.0)/n_gas)*n_act*K_leak*Ps_op(0))/60.0;
    Bc(0,0) = (k*pow(Ps_op(0),(n_gas+1.0)/n_gas)*q_p*60.0)/60.0;
    Ec(0,0) = k*pow(Ps_op(0),(n_gas+1.0)/n_gas)*(-(n_gas+1.0)/n_gas*q_p*wp_op(0)*60.0 + (n_gas+1.0)/n_gas*K_leak*Ps_op(0) + Qact_des_window(0)/n_gas)/60.0;
    MatrixNd Ad = MatrixNd::Identity(1,1)+dT*Ac;
    MatrixNd Bd = dT*Bc;
    MatrixNd Ed = dT*Ec;

    MatrixNd A_MPC = MatrixNd::Zero(N,1);
    MatrixNd B_MPC = MatrixNd::Zero(N,2*N);
    MatrixNd E_MPC = MatrixNd::Zero(N,1);
    A_MPC.block(0,0,1,1) = Ad;
    B_MPC.block(0,0,1,2) = Bd;
    E_MPC.block(0,0,1,1) = Ed;

    for(int i=1;i<N;i++) {
        Ac = MatrixNd::Zero(1,1);
        Bc = MatrixNd::Zero(1,2);
        Ec = MatrixNd::Zero(1,1);
        Ac(0,0) = ((n_gas+1.0)/n_gas)*k*pow(Ps_op(i),(1.0/n_gas))*(q_p*wp_op(i)*60.0 - Qact_des_window(i) - ((2.0*n_gas+1.0)/n_gas)*n_act*K_leak*Ps_op(i))/60.0;
        Bc(0,0) = (k*pow(Ps_op(i),(n_gas+1.0)/n_gas)*q_p*60.0)/60.0;
        Ec(0,0) = k*pow(Ps_op(i),(n_gas+1.0)/n_gas)*(-(n_gas+1.0)/n_gas*q_p*wp_op(i)*60.0 + (n_gas+1.0)/n_gas*K_leak*Ps_op(i) + Qact_des_window(i)/n_gas)/60.0;
        Ad = MatrixNd::Identity(1,1)+dT*Ac;
        Bd = dT*Bc;
        Ed = dT*Ec;

        A_MPC.block(i,0,1,1) = Ad*A_MPC.block((i-1),0,1,1);
        for(int j=0;j<i;j++) {
            B_MPC.block(i,2*j,1,2) = Ad*B_MPC.block((i-1),2*j,1,2);
        }
        B_MPC.block(i,2*i,1,2) = Bd;
        E_MPC.block(i,0,1,1) = Ad*E_MPC.block((i-1),0,1,1)+Ed;
    }

    temp_size_equ = N;
    Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);
    A_temp.block(0,0,N,N) = MatrixNd::Identity(N,N);
    A_temp.block(0,N,N,2*N) = -B_MPC;
    B_temp.block(0,0,N,1) = A_MPC*Ps_now + E_MPC;
    Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
    idx_equ += temp_size_equ;

    n_equ = idx_equ;

    //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
    /// 1) Pump Speed Constraints
    /// 2) Pump Acceleration Constraints
    /// 2) Delta Limit
    /////////////////////////////////////////////////////////////////////////////////////////////////

    int n_inequ = 0;
    int idx_inequ = 0;
    int temp_size_inequ = 0;
    MatrixNd A_inequ;
    VectorNd B_inequ;

    // 1) Pump Speed Constraints
    double wp_lb = wp_ref_min/60.0*2.0*PI; // [rad/s]
    double wp_ub = wp_ref_max/60.0*2.0*PI; // [rad/s]
    temp_size_inequ = 2*N;
    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,n_X+2*i) = -1.0;
        B_temp(i) = -wp_lb;
        A_temp(i+N,n_X+2*i) = 1.0;
        B_temp(i+N) = wp_ub;
    }
    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
    idx_inequ += temp_size_inequ;

//    // 2) Pump Acceleration Constraints
//    double dwp_lb = -1000.0; // [rad/s^2]
//    double dwp_ub = 1000.0; // [rad/s^2]
//    temp_size_inequ = 2*N;
//    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//    for(int i=0;i<N-1;i++) {
//        A_temp(i,n_X+2*i) = -1.0;
//        A_temp(i,n_X+2*(i-1)) = 1.0;
//        B_temp(i) = -(dwp_lb*dT);
//    }
//    for(int i=0;i<N-1;i++) {
//        A_temp(i+N-1,n_X+2*i) = 1.0;
//        A_temp(i+N-1,n_X+2*(i-1)) = -1.0;
//        B_temp(i+N-1) = (dwp_ub*dT);
//    }
//    A_temp(2*N-2,n_X) = 1.0;
//    B_temp(2*N-2) = (dwp_ub*SYS_DT_PUMPING+U_MPC(0));
//    A_temp(2*N-1,n_X) = -1.0;
//    B_temp(2*N-1) = -(dwp_lb*SYS_DT_PUMPING+U_MPC(0));
//    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//    idx_inequ += temp_size_inequ;

    // 3) Delta Limit
    double delta_lb = 0.0;
    temp_size_inequ = N;
    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,n_X+2*i+1) = -1.0;
        B_temp(i) = -delta_lb;
    }
    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
    idx_inequ += temp_size_inequ;

    n_inequ = idx_inequ;

    ////////////////////////////////////////////////////////////////////////////////

    QP_PsCtrl.setNums(n_var,n_cost,n_equ,n_inequ);
    QP_PsCtrl.make_COST(A_cost, B_cost);
    QP_PsCtrl.make_EQ(A_equ,B_equ);
    QP_PsCtrl.make_IEQ(A_inequ,B_inequ);

    switch(QP_PsCtrl.WhichSolver()) {
    case SolverIsQuadProg:
    {
        // 1. Solve the problem.
        VectorNd QP_sol = QP_PsCtrl.solve_QP();
        X_MPC = QP_sol.segment(0,N);
        U_MPC = QP_sol.segment(N,2*N);
        break;
    }
    case SolverIsQPSwift:
    {
        qp_int n_var = QP_PsCtrl.NUMCOLS;
        qp_int n_equ = QP_PsCtrl.NUMEQ;
        qp_int n_inequ = QP_PsCtrl.NUMINEQ;

        qp_int   P_nnz,A_nnz,G_nnz;
        qp_real  *P_x = NULL;
        qp_int   *P_i = NULL,*P_p = NULL;
        qp_real  *q_x = NULL;
        qp_real  *A_x = NULL;
        qp_int   *A_i = NULL, *A_p = NULL;
        qp_real  *b_x = NULL;
        qp_real  *G_x = NULL;
        qp_int   *G_i = NULL, *G_p = NULL;
        qp_real  *h_x = NULL;

        // 1-0. Convert cost function to sparse form .
        MatrixNd P = QP_PsCtrl.P_cost;
        VectorNd q = QP_PsCtrl.Q_cost;
        P_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(P);
        P_x = new qp_real[P_nnz];
        P_i = new qp_int[P_nnz];
        P_p = new qp_int[n_var+1];
        QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = new qp_real[n_var];
        QP_PsCtrl.ConvertVector2Array_QPswift(q,n_var,q_x);

        // 1-1. Convert equality constraint to sparse form .
        if(n_equ > 0) {
            MatrixNd A = QP_PsCtrl.A_equ;
            MatrixNd b = QP_PsCtrl.B_equ;
            A_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(A);
            A_x = new qp_real[A_nnz];
            A_i = new qp_int[A_nnz];
            A_p = new qp_int[n_var+1];
            QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
            b_x = new qp_real[n_equ];
            QP_PsCtrl.ConvertVector2Array_QPswift(b,n_equ,b_x);
        }

        // 1-2. Convert inequality constraint to sparse form .
        if(n_inequ > 0) {
            MatrixNd G = QP_PsCtrl.A_inequ;
            MatrixNd h = QP_PsCtrl.B_inequ;
            G_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(G);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_PsCtrl.ConvertVector2Array_QPswift(h,n_inequ,h_x);
        }

        // 2. Set Parameters and solve.
        QPswift      *myQP;
        if(n_inequ != 0) {
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                                    P_p,P_i,P_x,
                                    A_p,A_i,A_x,
                                    G_p,G_i,G_x,
                                    q_x,h_x,b_x,
                                    0.0,nullptr);
            QP_SOLVE(myQP);
//            PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup)*1000.0);
//            PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//            PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
        } else {
            myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                    P_p,P_i,P_x,
                                    A_p,A_i,A_x,
                                    q_x,b_x,
                                    0.0,nullptr);
            QP_SOLVE_NoInequ(myQP);
//            PRINT("Setup Time     : %f ms\n", (myQP->stats->tsetup)*1000.0);
//            PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve)*1000.0);
        }
        VectorNd QP_sol = VectorNd::Zero(n_var);
        QP_PsCtrl.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
        X_MPC = QP_sol.segment(0,N);
        U_MPC = QP_sol.segment(N,2*N);

//        cout << "Ps : \n" << Ps_window << endl;
//        cout << "wp : \n" << wp_window << endl;

        // 3. destruction allocated memory
        QP_CLEANUP(myQP);
        delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
        delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
        delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
        break;
    }
    default:
        FILE_LOG(logERROR) << "[Pump Speed MPC Error] QP Solver is not set!";
    }
}

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B)
{
    A.resize(m,n);
    A = MatrixNd::Zero(m,n);
    B.resize(m,1);
    B = MatrixNd::Zero(m,1);
}

void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new)
{
    int m_now = A_base.rows();
    int n_new = A_new.cols();
    int m_new = A_new.rows();

    A_base.conservativeResize(m_now+m_new,n_new);
    B_base.conservativeResize(m_now+m_new,1);
    A_base.block(m_now,0,m_new,n_new) = A_new;
    B_base.block(m_now,0,m_new,1) = B_new;
}

