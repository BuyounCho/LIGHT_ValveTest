#ifndef JOINT_INVERSE
#define JOINT_INVERSE

#include <cmath>
struct JOINTPARAMS
{
    // double L1,L2,Lx,Ly,toff,moff;
    // double q_direction, m_direction;
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
    JOINTPARAMS JP[3];//HP,KN//RIGHT_HP
    //double e2P[2] = {-0.001/100,0.001/100};//change in meter, and 0.01 more
    //not used in future. should be applied to DB
    //mnow[LKN] = e2P[LKN]*sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition + moff[LKN];
    //            qnow[LKN] = Oi.m2q_1joint(mnow[LKN],LKN);

    ONELEGFKIK()//param initialize;
    {
        // R_GR = 200;
        // I_R = I_P = I_K = 0;//inertia

        // F_R = F_P = F_K = 0.0;//friction
        // T2CR = 1/(20.5*0.001);//Torque constant 20.5mNm/A
        // T2CP = T2CR*270/100000.0;//Torque constant 20.5mNm/A,
		
        //Actually force to current.
        //Gear Ratio from motor to ball screw
        //270:100000??
        // I_P = I_K = 0.3321/T2CP;//0.3321 Amp per ddm
        //


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
        double md,mo,L1,L2,L3,L4,q4o,q4d1,q4d2,qd,qo;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;
		md = JP[jnum].md;
		mo = JP[jnum].mo;
		q4o = JP[jnum].q4o;
		q4d1 = JP[jnum].q4d1;
		q4d2 = JP[jnum].q4d2;
		qd = JP[jnum].qd;
		qo = JP[jnum].qo;
		
        t = m*md+mo;
		L6 = sqrt(L1*L1+L2*L2-2*L1*L2*cos(t));
		alpha = asin(L2/L6*sin(t));
		beta = acos((L4*L4+L6*L6-L3*L3)/(2*L4*L6));
		q4 = q4o + alpha*q4d1 + beta*q4d2;
		q = q4*qd + qo;
        return q;
    }
    double q2m_1joint(double q, int jnum)
    {
        double md,mo,L1,L2,L3,L4,q4o,q4d1,q4d2,qd,qo;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;
		// md = JP[jnum].md;
		// mo = JP[jnum].mo;
		// q4o = JP[jnum].q4o;
		// q4d1 = JP[jnum].q4d1;
		// q4d2 = JP[jnum].q4d2;
		// qd = JP[jnum].qd;
		// qo = JP[jnum].qo;
		
		if(jnum==0 || jnum==1){ // Right Hip roll, Left Hip roll
            L5 = sqrt(L1*L1+L2*L2-2*L1*L2*cos(pi-q));
			alpha = asin(L4/L5*sin(pi-q));
			beta = acos((L2*L2+L5*L5-L3*L3)/(2*L2*L5));
			q2 = beta + alpha;
        }
        else if(jnum==2){ // Knee Pitch
            L5 = sqrt(L1*L1+L2*L2-2*L1*L2*cos(t));
			alpha = asin(L4/L5*sin(t));
			beta = acos((L2*L2+L5*L5-L3*L3)/(2*L2*L5));
			q2 = beta - alpha;
        }
        return q2;
    }

    double J(double m, int jnum)
    {
		double md,mo,L1,L2,L3,L4,q4o,q4d1,q4d2,qd,qo;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;
		md = JP[jnum].md;
		mo = JP[jnum].mo;
		q4o = JP[jnum].q4o;
		q4d1 = JP[jnum].q4d1;
		q4d2 = JP[jnum].q4d2;
		qd = JP[jnum].qd;
		qo = JP[jnum].qo;

        double Jout = qd*(q4d1*(L2*md*cos(mo+md*m(t))*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)-L1*(L2*L2)*md*pow(sin(mo+md*m(t)),2.0)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0))*1.0/sqrt(-((L2*L2)*pow(sin(mo+md*m(t)),2.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0)-q4d2*((L1*L2*md*sin(mo+md*m(t))*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0))/L4-(L1*L2*md*sin(mo+md*m(t))*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0)*(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0)*(1.0/2.0))/L4)*1.0/sqrt((1.0/(L4*L4)*pow(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0,2.0)*(-1.0/4.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0));

        return Jout;
    }
    double dJ(double m, double dm, int jnum)
    {
        double md,mo,L1,L2,L3,L4,q4o,q4d1,q4d2,qd,qo;
        L1 = JP[jnum].L1;
        L2 = JP[jnum].L2;
        L3 = JP[jnum].L3;
        L4 = JP[jnum].L4;
		md = JP[jnum].md;
		mo = JP[jnum].mo;
		q4o = JP[jnum].q4o;
		q4d1 = JP[jnum].q4d1;
		q4d2 = JP[jnum].q4d2;
		qd = JP[jnum].qd;
		qo = JP[jnum].qo;

        double dJout = qd*(q4d2*1.0/sqrt((1.0/(L4*L4)*pow(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0,2.0)*(-1.0/4.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0)*(((L1*L1)*(L2*L2)*(md*md)*pow(sin(mo+md*m(t)),2.0)*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0)*2.0)/L4-(L1*L2*(md*md)*cos(mo+md*m(t))*diff(m(t),t)*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0))/L4+(L1*L2*(md*md)*cos(mo+md*m(t))*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0)*(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0)*(1.0/2.0))/L4-((L1*L1)*(L2*L2)*(md*md)*pow(sin(mo+md*m(t)),2.0)*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,5.0/2.0)*(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0)*(3.0/2.0))/L4)-q4d1*1.0/sqrt(-((L2*L2)*pow(sin(mo+md*m(t)),2.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0)*(L2*(md*md)*sin(mo+md*m(t))*diff(m(t),t)*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)-(L1*L1)*(L2*L2*L2)*(md*md)*pow(sin(mo+md*m(t)),3.0)*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,5.0/2.0)*3.0+L1*(L2*L2)*(md*md)*cos(mo+md*m(t))*sin(mo+md*m(t))*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0)*3.0)-q4d1*(L2*md*cos(mo+md*m(t))*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)-L1*(L2*L2)*md*pow(sin(mo+md*m(t)),2.0)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0))*1.0/pow(-((L2*L2)*pow(sin(mo+md*m(t)),2.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0,3.0/2.0)*(L1*(L2*L2*L2)*md*pow(sin(mo+md*m(t)),3.0)*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,2.0)*2.0-((L2*L2)*md*cos(mo+md*m(t))*sin(mo+md*m(t))*diff(m(t),t)*2.0)/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0))*(1.0/2.0)+q4d2*((L1*L2*md*sin(mo+md*m(t))*1.0/sqrt(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0))/L4-(L1*L2*md*sin(mo+md*m(t))*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,3.0/2.0)*(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0)*(1.0/2.0))/L4)*(L1*L2*1.0/(L4*L4)*md*sin(mo+md*m(t))*diff(m(t),t)*1.0/pow(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0,2.0)*pow(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0,2.0)*(1.0/2.0)-(L1*L2*1.0/(L4*L4)*md*sin(mo+md*m(t))*diff(m(t),t)*(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0))*1.0/pow((1.0/(L4*L4)*pow(L1*L1+L2*L2-L3*L3+L4*L4-L1*L2*cos(mo+md*m(t))*2.0,2.0)*(-1.0/4.0))/(L1*L1+L2*L2-L1*L2*cos(mo+md*m(t))*2.0)+1.0,3.0/2.0)*(1.0/2.0));
        
		return dJout;
    }
public:
    double  m2q_kn(double m)
    {
        return m2q_1joint(m,2);
    }
    double  m2q_lhr(double m)
    {
        return m2q_1joint(m,1);
    }
    double  m2q_rhr(double m)
    {
        return m2q_1joint(m,0);
    }
    double  q2m_kn(double q)
    {
		double t = -q - 13.0*M_PI/180.0;
//        double t = -(q+13.0+180.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,2);
        double m = -q2; //+82.2552*M_PI/180.0
//        double m = -q2 + 82.2552*M_PI/180.0;
        return m;
    }
    double  q2m_lhr(double q)
    {
        double t = q + (17.1+90.0)*M_PI/180.0;
//        double t = (q+17.1-90.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,1);
        double m = q2 - 90.0*M_PI/180.0;
        return m;
    }
    double  q2m_rhr(double q)
    {
//        double D2R = M_PI/180.0;
//        double q4_b = q - 17.1*D2R;
//        double q4 = 90*D2R - q4_b;
//        double q2 = q2m_1joint(q4,0);
//        double m = 90*D2R - q2;

        double t = -q + (17.1+90.0)*M_PI/180.0;
//        double t = (-q+17.1+90.0)*M_PI/180.0;
        double q2 = q2m_1joint(t,0);
        double m = -q2 + 90.0*M_PI/180.0;
        return m;
    }

    double J_kn(double m)
    {
        return J(m,1);
    }
    double J_lhr(double m)
    {
        return J(m,0);
    }
    double J_rhr(double m)
    {
        return J(m,0);
       // return J(m,2);
    }
    double dJ_kn(double m,double dm)
    {
        return dJ(m,dm,1);
    }
    double dJ_lhr(double m,double dm)
    {
        return dJ(m,dm,0);
    }
    double dJ_rhr(double m,double dm)
    {
        return dJ(m,dm,0);
       // return J(m,2);
    }

    double dq2dm_rhr(double q,double dq)
    {
        double m = q2m_rhr(q);
        return dq/J_rhr(m);
    }
    double dq2dm_lhr(double q,double dq)
    {
        double m = q2m_lhr(q);
        return dq/J_lhr(m);
    }
    double dq2dm_kn(double q,double dq)
    {
        double m = q2m_kn(q);
        return dq/J_kn(m);
    }

    double dm2dq_rhr(double m, double dm)
    {
        return dm*J_rhr(m);
    }
    double dm2dq_lhr(double m, double dm)
    {
        return dm*J_lhr(m);
    }
    double dm2dq_kn(double m, double dm)
    {
        return dm*J_kn(m);
    }

    double ddq2ddm_rhr(double q, double dq, double ddq)
    {
        double m = q2m_rhr(q);
        double dm = dq2dm_rhr(q,dq);
        return ddq/J_rhr(m) -dq/J_rhr(m)/J_rhr(m)*dJ_rhr(m,dm);
    }
    double ddq2ddm_lhr(double q, double dq, double ddq)
    {
        double m = q2m_lhr(q);
        double dm = dq2dm_lhr(q,dq);
        return ddq/J_lhr(m) -dq/J_lhr(m)/J_lhr(m)*dJ_lhr(m,dm);
    }
    double ddq2ddm_kn(double q, double dq, double ddq)
    {
        double m = q2m_kn(q);
        double dm = dq2dm_kn(q,dq);
        return ddq/J_kn(m) -dq/J_kn(m)/J_kn(m)*dJ_kn(m,dm);
    }


};
#endif // MAIN
