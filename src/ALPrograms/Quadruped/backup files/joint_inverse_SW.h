#ifndef JOINT_INVERSE
#define JOINT_INVERSE

#include <cmath>
#include "iostream"
using namespace std;
struct JOINTPARAMS
{
//    double L1,L2,Lx,Ly,toff,moff;
//    double q_direction, m_direction;
    double md,mo,L1,L2,L3,L4,q4o,q4d1,q4d2,qd,qo;
};
class ONELEGFKIK
{
    double L_HP, L_KN;
    double minSignum;
public:
    double R_GR;
    double I_R, I_P, I_K;//Inertia
    double F_R, F_P, F_K;//Friction
    double T2CR, T2CP;//Torque to Current
    JOINTPARAMS JP[3];//HR,KN

    ONELEGFKIK()//param initialize;
    {
        //// Right Hip Roll
        JP[0].L1 = 0.110;  //r1
        JP[0].L2 = 0.060;  //r2
        JP[0].L3 = 0.105;  //r3
        JP[0].L4 = 0.06801;//r4
		JP[0].md = -1;
		JP[0].mo = 90.0*M_PI/180;
		JP[0].q4o = M_PI;
		JP[0].q4d1 = -1;
		JP[0].q4d2 = -1;
		JP[0].qd = -1;
		JP[0].qo = (90.0+17.1)*M_PI/180;

        //// Left Hip Roll
        JP[1].L1 = 0.110;  //r1
        JP[1].L2 = 0.060;  //r2
        JP[1].L3 = 0.105;  //r3
        JP[1].L4 = 0.06801;//r4
		JP[0].md = 1;
		JP[0].mo = 90.0*M_PI/180;
		JP[0].q4o = M_PI;
		JP[0].q4d1 = -1;
		JP[0].q4d2 = -1;
		JP[0].qd = 1;
		JP[0].qo = -(90.0+17.1)*M_PI/180;

        //// Knee Pitch
        JP[2].L1 = 0.280;  //r1
        JP[2].L2 = 0.040;  //r2
        JP[2].L3 = 0.277;  //r3
        JP[2].L4 = 0.040;  //r4
		JP[0].md = -1;
		JP[0].mo = 0.0;
		JP[0].q4o = 0.0;
		JP[0].q4d1 = -1;
		JP[0].q4d2 = 1;
		JP[0].qd = -1;
		JP[0].qo = -13.0*M_PI/180;

        /*
        R_GR = 200;
        I_R = I_P = I_K = 0;//inertia

        F_R = F_P = F_K = 0.0;//friction
        T2CR = 1/(20.5*0.001);//Torque constant 20.5mNm/A
        T2CP = T2CR*270/100000.0;//Torque constant 20.5mNm/A,
        //Actually force to current.
        //Gear Ratio from motor to ball screw
        //270:100000??
        I_P = I_K = 0.3321/T2CP;//0.3321 Amp per ddm
        //


       JP[0].L1 = 0.085;//b1//lhp
       JP[0].L2 = 0.035;//a1
       JP[0].Lx = 0.0393;//c1
       JP[0].Ly = 0.020;//d1
       JP[0].toff =-42.0/180*M_PI;//-M_PI/4-asin(0.02/0.25);//have to ask taejin
       JP[0].moff = -0.022067-0.0547188-0.00108603;
       JP[0].q_direction = -1;
       JP[0].m_direction = 1;



       JP[1].L1 = 0.085;//b2//kn
       JP[1].L2 = 0.035;//a2
       JP[1].Lx = 0.0945;//c2
       JP[1].Ly = 0.020;//d2
       JP[1].toff = 0.0+asin(0.02/0.25);//-?+? have to check
       JP[1].moff = 0.0426605;

       //effectively 360kg
       //lets calculate not tune....
       JP[1].q_direction = 1;
       JP[1].m_direction = -1;


//       JP[2].L1 = 0.085;//b1//rhp
//       JP[2].L2 = 0.035;//a1
//       JP[2].Lx = 0.0393;//c1
//       JP[2].Ly = 0.020;//d1
//       JP[2].toff =+42.0/180*M_PI+M_PI/2;//hmmmmm..... confusing a lot
//       JP[2].moff = -0.012969;//-0.022067;//different referense position
//       JP[2].I = 0.00001563*(3*M_PI/0.002)*(3*M_PI/0.002)/0.8*0.276/2.0*0.5;
//       JP[2].q_direction = 1;
//       JP[2].m_direction = -1;
//       JP[2].friction_I = 1.0;


       L_HP = 0.25;
       L_KN = 0.24;

       minSignum = 0.01;
       */
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
        double L1,L2,L3,L4;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;

        if(jnum==0){ // Right Hip roll
            double A = 2*L1*L4 - 2*L2*L4*cos(m);
            double B = -2*L2*L4*sin(m);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 - 2*L1*L2*cos(-m);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Right Hip roll] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double q = 2*atan(t);
            return q;
        }
        else if(jnum==1){ // Left Hip roll
            double A = 2*L1*L4 - 2*L2*L4*cos(m);
            double B = -2*L2*L4*sin(m);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 - 2*L1*L2*cos(-m);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Left Hip roll] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double q = 2*atan(t);
            return q;
        }
        else if(jnum==2){ // Knee pitch
            double A = 2*L1*L4 - 2*L2*L4*cos(m);
            double B = 2*L2*L4*sin(m);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 - 2*L1*L2*cos(-m);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Knee pitch] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double q = 2*atan(t);
            return q;
        }
    }
    double q2m_1joint(double q, int jnum)
    {
        double L1,L2,L3,L4;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;

        if(jnum==0){ // Right Hip roll
            double A = -2*L1*L2 - 2*L2*L4*cos(q);
            double B = -2*L2*L4*sin(q);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 + 2*L1*L4*cos(-q);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Right Hip roll] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double m = 2*atan(t);
            return m;
        }
        else if(jnum==1){ // Left Hip roll
            double A = -2*L1*L2 - 2*L2*L4*cos(q);
            double B = -2*L2*L4*sin(q);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 + 2*L1*L4*cos(-q);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Left Hip roll] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double m = 2*atan(t);
            return m;
        }
        else if(jnum==2){ // Knee pitch
            double A = -2*L1*L2 - 2*L2*L4*cos(q);
            double B = +2*L2*L4*sin(q);
            double C = L1*L1 + L2*L2 + L4*L4 - L3*L3 + 2*L1*L4*cos(+q);
            double delta = A*A+B*B-C*C;
            if(delta<0){cout << "[Knee pitch] Infeasible Solution!" << endl;}
            double t = B-sqrt(delta)/(A-C);
            double m = 2*atan(t);
            return m;
        }
    }

    /*
    double J(double m, int jnum)
    {



        double Lx,Ly,L1,L2, toff,d;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
//        toff = JP[jnum].toff;
//        d = JP[jnum].q_direction;
//        double x = m*JP[jnum].m_direction+JP[jnum].moff;
//        double J =  -(((Lx*2.0-x*2.0)*1.0/sqrt(Ly*Ly+pow(Lx-x,2.0))*(1.0/2.0))/L2-((Lx*2.0-x*2.0)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),3.0/2.0)*(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0))*(1.0/4.0))/L2)*1.0/sqrt((1.0/(L2*L2)*pow(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0),2.0)*(-1.0/4.0))/(Ly*Ly+pow(Lx-x,2.0))+1.0)-Ly/(Ly*Ly+pow(Lx-x,2.0));

//        J = J*d;

        double qD = JP[jnum].q_direction;
        double mD = JP[jnum].m_direction;
        double mO = JP[jnum].moff;
        double Jout = qD*(1.0/sqrt((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0)*((mD*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*(-Lx+mO+mD*m))/L2-(mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2)-(Ly*mD)/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly));



        return Jout;
    }
    double dJ(double m, double dm, int jnum)
    {
        double Lx,Ly,L1,L2, toff,d;
        Lx = JP[jnum].Lx;
        Ly = JP[jnum].Ly;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
//        toff = JP[jnum].toff;
//        d = JP[jnum].q_direction;
//        double x = m*JP[jnum].m_direction+JP[jnum].moff;
//        double dx = dm*JP[jnum].m_direction;
//        double dJ = 1.0/sqrt((1.0/(L2*L2)*pow(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0),2.0)*(-1.0/4.0))/(Ly*Ly+pow(Lx-x,2.0))+1.0)*((dx*1.0/sqrt(Ly*Ly+pow(Lx-x,2.0)))/L2-(dx*1.0/pow(Ly*Ly+pow(Lx-x,2.0),3.0/2.0)*(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0))*(1.0/2.0))/L2-(dx*(Lx-x)*(Lx*2.0-x*2.0)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),3.0/2.0))/L2+(dx*(Lx-x)*(Lx*2.0-x*2.0)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),5.0/2.0)*(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0))*(3.0/4.0))/L2)-(((Lx*2.0-x*2.0)*1.0/sqrt(Ly*Ly+pow(Lx-x,2.0))*(1.0/2.0))/L2-((Lx*2.0-x*2.0)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),3.0/2.0)*(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0))*(1.0/4.0))/L2)*(1.0/(L2*L2)*dx*(Lx-x)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),2.0)*pow(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0),2.0)*(1.0/2.0)-(1.0/(L2*L2)*dx*(Lx-x)*(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0)))/(Ly*Ly+pow(Lx-x,2.0)))*1.0/pow((1.0/(L2*L2)*pow(-L1*L1+L2*L2+Ly*Ly+pow(Lx-x,2.0),2.0)*(-1.0/4.0))/(Ly*Ly+pow(Lx-x,2.0))+1.0,3.0/2.0)*(1.0/2.0)-Ly*dx*(Lx-x)*1.0/pow(Ly*Ly+pow(Lx-x,2.0),2.0)*2.0;
//        dJ = dJ*d;

        double qD = JP[jnum].q_direction;
        double mD = JP[jnum].m_direction;
        double mO = JP[jnum].moff;
        double dJout = qD*(1.0/sqrt((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0)*(((mD*mD)*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*dm)/L2-((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*dm*pow(-Lx+mO+mD*m,2.0)*2.0)/L2-((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*dm*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2+((mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,5.0/2.0)*dm*pow(-Lx+mO+mD*m,2.0)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(3.0/2.0))/L2)-1.0/pow((1.0/(L2*L2)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(-1.0/4.0))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)+1.0,3.0/2.0)*(1.0/(L2*L2)*mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,2.0)*dm*(-Lx+mO+mD*m)*pow(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly,2.0)*(1.0/2.0)-(1.0/(L2*L2)*mD*dm*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly))/(pow(-Lx+mO+mD*m,2.0)+Ly*Ly))*((mD*1.0/sqrt(pow(-Lx+mO+mD*m,2.0)+Ly*Ly)*(-Lx+mO+mD*m))/L2-(mD*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,3.0/2.0)*(-Lx+mO+mD*m)*(pow(-Lx+mO+mD*m,2.0)-L1*L1+L2*L2+Ly*Ly)*(1.0/2.0))/L2)*(1.0/2.0)+Ly*(mD*mD)*1.0/pow(pow(-Lx+mO+mD*m,2.0)+Ly*Ly,2.0)*dm*(-Lx+mO+mD*m)*2.0);
        return dJout;
    }
    */
public:

    double  m2q_kn(double m)
    {
        double t = -m;
//        double t = -m*M_PI/180.0;
        double q4 = m2q_1joint(t,2);
        double q = -q4 - (13.0+180.0)*M_PI/180.0; //-90.0*M_PI/180.0
//        double q = -q4 - (13.0+180.0-90.0)*M_PI/180.0;
        return q;
//        return m2q_1joint(m,2);
    }
    double  m2q_lhr(double m) //should be lhr
    {
        double t = m + 90.0*M_PI/180.0;
//        double t = (m+90.0)*M_PI/180.0;
        double q4 = m2q_1joint(t,1);
        double q = q4 + (90.0-17.1)*M_PI/180.0;
        return q;
//        return m2q_1joint(m,1);
    }
    double  m2q_rhr(double m) //should be rhr
    {
        double t = 90.0*M_PI/180.0 - m;
//        double t = (90.0-m)*M_PI/180.0;
        double q4 = m2q_1joint(t,0);
        double q = -q4 + (90.0+17.1)*M_PI/180.0;
        return q;
//        return m2q_1joint(m,0);
    }

    double  q2m_kn(double q)
    {
        double t = -q - (13.0+180.0)*M_PI/180.0;
//        double t = -(q+13.0+180.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,2);
        double m = -q2; //+82.2552*M_PI/180.0
//        double m = -q2 + 82.2552*M_PI/180.0;
        return m;
    }
    double  q2m_lhr(double q) //should be lhr
    {
        double t = q + (17.1+90.0)*M_PI/180.0;
//        double t = (q+17.1-90.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,1);
        double m = q2 - 90.0*M_PI/180.0;
        return m;
    }
    double  q2m_rhr(double q) //should be rhr
    {
        /*
//        double D2R = M_PI/180.0;
//        double q4_b = q - 17.1*D2R;
//        double q4 = 90*D2R - q4_b;
//        double q2 = q2m_1joint(q4,0);
//        double m = 90*D2R - q2;
*/
        double t = -q + (17.1+90.0)*M_PI/180.0;
//        double t = (-q+17.1+90.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,0);
        double m = -q2 + 90.0*M_PI/180.0;
        return m;
    }

    /*
    double J_kn(double m)
    {
        return J(m,1);
    }
    double J_lhp(double m)
    {
        return J(m,0);
    }
    double J_rhp(double m)
    {
        return J(m,0);
       // return J(m,2);
    }
    double dJ_kn(double m,double dm)
    {
        return dJ(m,dm,1);
    }
    double dJ_lhp(double m,double dm)
    {
        return dJ(m,dm,0);
    }
    double dJ_rhp(double m,double dm)
    {
        return dJ(m,dm,0);
       // return J(m,2);
    }

    double dq2dm_rhp(double q,double dq)
    {
        double m = q2m_rhp(q);
        return dq/J_rhp(m);
    }
    double dq2dm_lhp(double q,double dq)
    {
        double m = q2m_lhp(q);
        return dq/J_lhp(m);
    }
    double dq2dm_kn(double q,double dq)
    {
        double m = q2m_kn(q);
        return dq/J_kn(m);
    }

    double dm2dq_rhp(double m, double dm)
    {
        return dm*J_rhp(m);
    }
    double dm2dq_lhp(double m, double dm)
    {
        return dm*J_lhp(m);
    }
    double dm2dq_kn(double m, double dm)
    {
        return dm*J_kn(m);
    }

    double ddq2ddm_rhp(double q, double dq, double ddq)
    {
        double m = q2m_rhp(q);
        double dm = dq2dm_rhp(q,dq);
        return ddq/J_rhp(m) -dq/J_rhp(m)/J_rhp(m)*dJ_rhp(m,dm);
    }
    double ddq2ddm_lhp(double q, double dq, double ddq)
    {
        double m = q2m_lhp(q);
        double dm = dq2dm_lhp(q,dq);
        return ddq/J_lhp(m) -dq/J_lhp(m)/J_lhp(m)*dJ_lhp(m,dm);
    }
    double ddq2ddm_kn(double q, double dq, double ddq)
    {
        double m = q2m_kn(q);
        double dm = dq2dm_kn(q,dq);
        return ddq/J_kn(m) -dq/J_kn(m)/J_kn(m)*dJ_kn(m,dm);
    }
    */


};
#endif // MAIN
