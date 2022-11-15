#ifndef JOINT_INVERSE
#define JOINT_INVERSE

#include <cmath>
struct JOINTPARAMS
{
    double L1,L2,Lx,Ly,toff,moff;
    double q_direction, m_direction;
};
class ONELEGFKIK
{
    double L_HP, L_KN;
    double minSignum;
public:
    double R_GR;
    double I_R, I_P, I_K;//Inertia
    double F_R, F_P, F_K;//Friction
    double F_R2, F_P2, F_K2;//Friction
    double T2CR, T2CP;//Torque to Current
    JOINTPARAMS JP[3];//HP,KN//RIGHT_HP
    //double e2P[2] = {-0.001/100,0.001/100};//change in meter, and 0.01 more
    //not used in future. should be applied to DB
    //mnow[LKN] = e2P[LKN]*sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition + moff[LKN];
    //            qnow[LKN] = Oi.m2q_1joint(mnow[LKN],LKN);

    ONELEGFKIK()//param initialize;
    {
        R_GR = 200;
        I_R = I_P = I_K = 0;//inertia
        F_R = F_P = F_K = 0.0;//friction
        T2CR = 1/(20.5*0.001);//Torque constant 20.5mNm/A
        double E2M_PulsePerMeter = 27.0*100000.0/10.0*4000/360.0;
        double E2M_RadPerMeter = E2M_PulsePerMeter/4000*2*M_PI;
        T2CP = T2CR/E2M_RadPerMeter;//Torque constant 20.5mNm/A,
        //Actually force to current.

        //Gear Ratio from motor to ball screw
        //270:100000??
        I_P = I_K = 0.3321/T2CP*2.0;//0.3321 Amp per ddm
        I_R =1e-5*0.5;
        F_K = 0.1016/T2CP*2.0*2.0;
        F_P = 0.2/T2CP*2.0*2.0;
        F_R = 0.6/T2CR*0.0;
        //        
        F_K2 = 0.1016/T2CP*2.0*4.5;
        F_P2 = 0.2/T2CP*2.0*4.5;
        F_R2 = 0.6/T2CR*2.0;
        //


       JP[0].L1 = 0.085;//b1//lhp
       JP[0].L2 = 0.035;//a1
       JP[0].Lx = 0.0393;//c1
       JP[0].Ly = 0.020;//d1
       JP[0].toff =-42.0/180*M_PI-3.0/180*M_PI;//-M_PI/4-asin(0.02/0.25);//have to ask taejin
       JP[0].moff = -0.022067-0.0547188-0.00108603;
       JP[0].q_direction = -1;
       JP[0].m_direction = 1;



       JP[1].L1 = 0.085;//b2//kn
       JP[1].L2 = 0.035;//a2
       JP[1].Lx = 0.0945;//c2
       JP[1].Ly = 0.020;//d2
       JP[1].toff = 0.0+asin(0.02/0.25);//-?+? have to check
       JP[1].moff = 0.0426605;

       JP[1].q_direction = 1;
       JP[1].m_direction = -1;



       L_HP = 0.25;
       L_KN = 0.24;

       minSignum = 0.01;
    }
    double signum(double in)
    {
        if(in>minSignum) return 1.0;
        if(in<-minSignum) return -1.0;
        return in/minSignum;

    }
private:
    double m2q_1joint(double m, int jnum)
    {
        double x,t1,h,t2,d;
        double Lx,Ly,L1,L2, toff;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        toff = JP[jnum].toff;
        d = JP[jnum].q_direction;
        x = m*JP[jnum].m_direction+JP[jnum].moff;
        t1 = atan2(Ly,Lx-x);
        h = sqrt((Lx-x)*(Lx-x) + Ly*Ly);
        double cost2 = (L2*L2+h*h - L1*L1)/(2*h*L2);
        t2 = acos(cost2);
        double rt = (-(t1+t2))*d + toff;
        if(rt<-M_PI){rt+=M_PI*2;}
        if(rt>M_PI){rt-=M_PI*2;}
        return rt;
    }
    double q2m_1joint(double q, int jnum)
    {
        double Lx,Ly,L1,L2, toff,d;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        toff = JP[jnum].toff;
        d = JP[jnum].q_direction;
        double tL2 = -((q-toff)*d);
        //printf("tL2 %f |||  ",tL2*180/M_PI);
        double hL1 = (Ly-L2*sin(tL2));
        double x = (Lx-sqrt(L1*L1-hL1*hL1)-L2*cos(tL2)-JP[jnum].moff)/JP[jnum].m_direction;
        return x;
    }

    double J(double m, int jnum)
    {
        double Lx,Ly,L1,L2;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;

        double qD = JP[jnum].q_direction;
        double mD = JP[jnum].m_direction;
        double mO = JP[jnum].moff;
        double Jout = qD*(1.0/sqrt((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0)*((mD*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*(-Lx+mO+mD*m))/L2-(mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2)-(Ly*mD)/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly));

        return Jout;
    }
    double dJ(double m, double dm, int jnum)
    {
        double Lx,Ly,L1,L2;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;

        double qD = JP[jnum].q_direction;
        double mD = JP[jnum].m_direction;
        double mO = JP[jnum].moff;
        double dJout = qD*(1.0/sqrt((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0)*(((mD*mD)*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*dm)/L2-((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*dm*pow(-Lx+mO+mD*m,2.0)*2.0)/L2-((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*dm*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2+((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,5.0/2.0)*dm*pow(-Lx+mO+mD*m,2.0)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(3.0/2.0))/L2)-1.0/pow((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0,3.0/2.0)*(1.0/(L2*L2)*mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,2.0)*dm*(-Lx+mO+mD*m)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(1.0/2.0)-(1.0/(L2*L2)*mD*dm*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly))*((mD*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*(-Lx+mO+mD*m))/L2-(mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2)*(1.0/2.0)+Ly*(mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,2.0)*dm*(-Lx+mO+mD*m)*2.0);
        return dJout;
    }
public:
    double  m2q_kn(double m)
    {
        return m2q_1joint(m,1);
    }
    double  m2q_hp(double m)
    {
        return m2q_1joint(m,0);
    }
    double  q2m_kn(double q)
    {
        if(q>150.0/180.0*M_PI){q = 150.0/180.0*M_PI;}
        double aa = q2m_1joint(q,1);
        return std::max(aa,0.0);
    }
    double  q2m_hp(double q)
    {
        if(q>100.0/180.0*M_PI){q = 100.0/180.0*M_PI;}
        double aa = q2m_1joint(q,0);
        return std::max(aa,0.0);
    }
    double J_kn(double m)
    {
        return J(m,1);
    }
    double J_hp(double m)
    {
        return J(m,0);
    }
    double dJ_kn(double m,double dm)
    {
        return dJ(m,dm,1);
    }
    double dJ_hp(double m,double dm)
    {
        return dJ(m,dm,0);
    }
    double dq2dm_hp(double q,double dq)
    {
        double m = q2m_hp(q);
        return dq/J_hp(m);
    }

    double dq2dm_kn(double q,double dq)
    {
        double m = q2m_kn(q);
        return dq/J_kn(m);
    }

    double dm2dq_hp(double m, double dm)
    {
        return dm*J_hp(m);
    }
    double dm2dq_kn(double m, double dm)
    {
        return dm*J_kn(m);
    }

    double ddq2ddm_hp(double q, double dq, double ddq)
    {
        double m = q2m_hp(q);
        double dm = dq2dm_hp(q,dq);
        return ddq/J_hp(m) -dq/J_hp(m)/J_hp(m)*dJ_hp(m,dm);
    }
    double ddq2ddm_kn(double q, double dq, double ddq)
    {
        double m = q2m_kn(q);
        double dm = dq2dm_kn(q,dq);
        return ddq/J_kn(m) -dq/J_kn(m)/J_kn(m)*dJ_kn(m,dm);
    }

    //front//////////////////////////////////////////////////
    //lets not revert m for simplicity
    ///////////////////////just revert qs
//    double  m2q_kn_front(double m)
//    {
//        return -m2q_1joint(m,1);
//    }
//    double  m2q_hp_front(double m)
//    {
//        return -m2q_1joint(m,0);
//    }
//    double  q2m_kn_front(double q)
//    {
//        double q2 = -q;
//        if(q2>150.0/180.0*M_PI){q2 = 150.0/180.0*M_PI;}
//        double aa = q2m_1joint(q2,1);
//        return std::max(aa,0.0);
//    }
//    double  q2m_hp_front(double q)
//    {
//        double q2 = -q;
//        if(q2>100.0/180.0*M_PI){q2 = 100.0/180.0*M_PI;}
//        double aa = q2m_1joint(q2,0);
//        return std::max(aa,0.0);
//    }
//    double J_kn_front(double m)
//    {
//        return -J(m,1);
//    }
//    double J_hp_front(double m)
//    {
//        return -J(m,0);
//    }
//    double dJ_kn_front(double m,double dm)
//    {
//        return -dJ(m,dm,1);
//    }
//    double dJ_hp_front(double m,double dm)
//    {
//        return -dJ(m,dm,0);
//    }
//    double dq2dm_hp_front(double q,double dq)
//    {
//        double m = q2m_hp_front(q);
//        return dq/J_hp_front(m);
//    }

//    double dq2dm_kn_front(double q,double dq)
//    {
//        double m = q2m_kn_front(q);
//        return dq/J_kn_front(m);
//    }

//    double dm2dq_hp_front(double m, double dm)
//    {
//        return dm*J_hp_front(m);
//    }
//    double dm2dq_kn_front(double m, double dm)
//    {
//        return dm*J_kn_front(m);
//    }

//    double ddq2ddm_hp_front(double q, double dq, double ddq)
//    {
//        double m = q2m_hp_front(q);
//        double dm = dq2dm_hp_front(q,dq);
//        return ddq/J_hp_front(m) -dq/J_hp_front(m)/J_hp_front(m)*dJ_hp_front(m,dm);
//    }
//    double ddq2ddm_kn_front(double q, double dq, double ddq)
//    {
//        double m = q2m_kn_front(q);
//        double dm = dq2dm_kn_front(q,dq);
//        return ddq/J_kn_front(m) -dq/J_kn_front(m)/J_kn_front(m)*dJ_kn_front(m,dm);
//    }


};
#endif // MAIN
