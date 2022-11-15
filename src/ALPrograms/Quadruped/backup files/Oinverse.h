#ifndef OINVERSE_H
#define OINVERSE_H

#include "Otypes.h"

class Oinverse//after LIG, fill these values
{
public:
    double err_max;


    double m_rhr;
    double m_rhp;
    double m_rkn;
    double m_rf;

    double m_lhr;
    double m_lhp;
    double m_lkn;
    double m_lf;

    double m_rsr;
    double m_rsp;
    double m_reb;
    double m_rh;

    double m_lsr;
    double m_lsp;
    double m_leb;
    double m_lh;

    double m_torso;


    double P2HX;
    double P2HY;
    double R2P;
    double ULEG;
    double LLEG;
    double xlim, ylim;



    vec3 offset_p2rh, offset_p2lh, offset_uleg, offset_lleg;
    vec3 offset_rh2rp, offset_lh2lp;
    vec3 c_rhr, c_rhp, c_rkn;
    vec3 c_lhr, c_lhp, c_lkn;
    mat3 I_rhr, I_rhp, I_rkn;
    mat3 I_lhr, I_lhp, I_lkn;

    vec3 offset_p2rs, offset_p2ls;
    vec3 offset_rs2rp, offset_ls2lp;
    vec3 c_rsr, c_rsp, c_reb;
    vec3 c_lsr, c_lsp, c_leb;
    mat3 I_rsr, I_rsp, I_reb;
    mat3 I_lsr, I_lsp, I_leb;


    vec3 c_torso;
    mat3 I_torso;



    mat3 L2R(mat3 in)
    {
        mat3 out;
        mat3 mY = mat3::eye();
        mY[1][1] = -1;
        out = mY*in*mY;
        return out;
    }
    mat3 F2B(mat3 in)
    {
        mat3 out;
        mat3 mX = mat3::eye();
        mX[0][0] = -1;
        out = mX*in*mX;
        return out;
    }
    vec3 F2B(vec3 in)
    {
        vec3 out = in;
        out.x = -in.x;
        return out;
    }
    vec3 L2R(vec3 in)
    {
        vec3 out = in;
        out.y = -in.y;
        return out;
    }
public:
    double M_total;
    int num_itter;

    Oinverse()
    {//set initial something

        err_max = 1e-7;

        //mass

		m_rhr = 1.27386;//temp_for ODE
        m_rhp = 2.00227;
        m_rkn = 0.10891;//LLEG only
		// m_rf = 0.09082;
		m_rf = 0.0; // seungwoo: should be modified to 0.09082, now it's for test
		// m_rkn = 0.19972;//LLEG + FT
        // m_rf = 0.0;
        m_torso = 30.82959 - 4*m_rhr-4*m_rhp-4*m_rkn-4*m_rf;

        m_lsr = m_rsr = m_lhr = m_rhr;
        m_lsp = m_rsp = m_lhp = m_rhp;
        m_leb = m_reb = m_lkn = m_rkn;
        m_lh = m_rh = m_lf = m_rf;

        M_total = (m_rhr+m_rhp+m_rkn+m_rf)*4+m_torso;


        //length
        P2HX = 0.231;
        P2HY = 0.110;
        ULEG = 0.28;
        LLEG = 0.25; //LLEG only: coordinate 6 (-0.01271,0.0,-0.03554)
		// LLEG = 0.28554; //LLEG extended: coordinate 5 (-0.01271,0.0,0.0)
        R2P = 0.047;
        xlim = (ULEG+LLEG)/2.0*1.5;
        ylim = (ULEG+LLEG)/2.0*1.5;


        //offsets
        offset_p2rs = vec3(P2HX,-P2HY,0);
        offset_p2ls = vec3(P2HX,+P2HY,0);
        offset_rs2rp = vec3(0,-R2P,0);
        offset_ls2lp = vec3(0,+R2P,0);

        offset_p2rh = vec3(-P2HX,-P2HY,0);
        offset_p2lh = vec3(-P2HX,+P2HY,0);
        offset_rh2rp = vec3(0,-R2P,0);
        offset_lh2lp = vec3(0,+R2P,0);

        offset_uleg = vec3(0,0,-ULEG);
        offset_lleg = vec3(0,0,-LLEG);

        //centerofmass
        c_torso = vec3(0,0,0);

        // c_lsr = vec3(-0.16233670,0.00099486,-0.00716505);
        // c_lsp = vec3(0.00013875,0.05693960,-0.03681816);
        // c_leb = vec3(0.00139678,-0.00001109,-0.12233874);
		c_lsr = vec3(0.00011,0.00283,0.00140);
        c_lsp = vec3(-0.00026,0.03369,-0.02516);
        c_leb = vec3(0.00144,0.0,-0.04022); //LLEG only
		// c_leb = vec3(-0.0045,-0.00004,-0.15035); //LLEG + FT

        c_lhr = F2B(c_lsr);
        c_lhp = (c_lsp);
        c_lkn = (c_leb);

        c_rsr = L2R(c_lsr);
        c_rsp = L2R(c_lsp);
        c_reb = L2R(c_leb);

        c_rhr = F2B(c_rsr);
        c_rhp = (c_rsp);
        c_rkn = (c_reb);


        //inertia

        // I_torso = mat3(0.48,0,0, 0,2.56395,0, 0,0,2.75595);

        // I_lsr = mat3(0.00391128,0.00063853,0.00155189,
                     // 0.00063858,0.01979592,-0.00002282,
                     // 0.00155189,0-0.00002282,0.01939834);
        // I_lsp = mat3(0.01575245, -0.00001087, -0.00010127,
                     // -0.00001087, 0.01459433, 0.00234147,
                     // -0.00010127, 0.00234147, 0.00328424);
        // I_leb = mat3(0.00272308, 0, 0,
                     // 0, 0.00274442, 0,
                     // 0, 0, 0.00005158);
					 
	    I_torso = mat3(	0.9998,-0.0005,-0.0223,
						0.0001,-0.9999,0.0171,
					   -0.0223,0.0171,0.9996);
        I_lsr = mat3(0.10,-0.16,0.98,
                     -0.99,0.01,0.10,
                     -0.03,-0.99,-0.16);
        I_lsp = mat3(0.06, 0.18, 0.98,
                     -0.99, 0.10, 0.04,
                     -0.10, -0.98, 0.18);
        I_leb = mat3(0.02, 0.0, 1.0,	//LLEG only
                     -0.97, 0.23, 0.02,
                     -0.23, -0.97, 0.0);
		// I_leb = mat3(0.05, 0.0, 1.0,    //LLEG + FT
                     // -1.0, 0.0, 0.05,
                     // 0.0, -0.1, 0.0);

        I_lhr = F2B(I_lsr);
        I_lhp = (I_lsp);
        I_lkn = (I_leb);

        I_rsr = L2R(I_lsr);
        I_rsp = L2R(I_lsp);
        I_reb = L2R(I_leb);

        I_rhr = F2B(I_rsr);
        I_rhp = (I_rsp);
        I_rkn = (I_reb);


        num_itter = -1;

            }


        QuadPos FK(QuadJoints QJ)
        {
            QuadPos QP;
            QP.pPel = QJ.pPel;
            QP.qPel = QJ.qPel;            
            vec3 pPel = QJ.pPel;
            quat qPel = QJ.qPel;
            mat4 T_PEL = mat4(pPel,mat3(qPel));
            mat4 T_RHR = T_PEL*mat4(offset_p2rh,vec3(1,0,0),QJ.RHR);
            mat4 T_RHP = T_RHR*mat4(offset_rh2rp,vec3(0,1,0),QJ.RHP);
            mat4 T_RKN = T_RHP*mat4(offset_uleg,vec3(0,1,0),QJ.RKN);
            mat4 T_RF = T_RKN*mat4(offset_lleg,vec3(1,0,0),0);

            mat4 T_LHR = T_PEL*mat4(offset_p2lh,vec3(1,0,0),QJ.LHR);
            mat4 T_LHP = T_LHR*mat4(offset_lh2lp,vec3(0,1,0),QJ.LHP);
            mat4 T_LKN = T_LHP*mat4(offset_uleg,vec3(0,1,0),QJ.LKN);
            mat4 T_LF = T_LKN*mat4(offset_lleg,vec3(1,0,0),0);

            mat4 T_RSR = T_PEL*mat4(offset_p2rs,vec3(1,0,0),QJ.RSR);
            mat4 T_RSP = T_RSR*mat4(offset_rs2rp,vec3(0,1,0),QJ.RSP);
            mat4 T_REB = T_RSP*mat4(offset_uleg,vec3(0,1,0),QJ.REB);
            mat4 T_RH = T_REB*mat4(offset_lleg,vec3(1,0,0),0);

            mat4 T_LSR = T_PEL*mat4(offset_p2ls,vec3(1,0,0),QJ.LSR);
            mat4 T_LSP = T_LSR*mat4(offset_ls2lp,vec3(0,1,0),QJ.LSP);
            mat4 T_LEB = T_LSP*mat4(offset_uleg,vec3(0,1,0),QJ.LEB);
            mat4 T_LH = T_LEB*mat4(offset_lleg,vec3(1,0,0),0);

            QP.pRF = (T_RF*vec3(0,0,0));
            QP.pLF = (T_LF*vec3(0,0,0));
            QP.pRH = (T_RH*vec3(0,0,0));
            QP.pLH = (T_LH*vec3(0,0,0));

            QP.qRF = quat(mat3(T_RF));
            QP.qLF = quat(mat3(T_LF));
            QP.qRH = quat(mat3(T_RH));
            QP.qLH = quat(mat3(T_LH));

            vec4 pCOM_torso = T_PEL*c_torso;

            vec4 pCOM_rhr = T_RHR*c_rhr;
            vec4 pCOM_rhp = T_RHP*c_rhp;
            vec4 pCOM_rkn = T_RKN*c_rkn;

            vec4 pCOM_lhr = T_LHR*c_lhr;
            vec4 pCOM_lhp = T_LHP*c_lhp;
            vec4 pCOM_lkn = T_LKN*c_lkn;

            vec4 pCOM_rsr = T_RSR*c_rsr;
            vec4 pCOM_rsp = T_RSP*c_rsp;
            vec4 pCOM_reb = T_REB*c_reb;

            vec4 pCOM_lsr = T_LSR*c_lsr;
            vec4 pCOM_lsp = T_LSP*c_lsp;
            vec4 pCOM_leb = T_LEB*c_leb;

            QP.pCOM = (pCOM_rhr*m_rhr +pCOM_rhp*m_rhp+pCOM_rkn*m_rkn + vec4(QP.pRF)*m_rf)
                    +(pCOM_lhr*m_lhr +pCOM_lhp*m_lhp+pCOM_lkn*m_lkn + vec4(QP.pLF)*m_lf)
                    +(pCOM_rsr*m_rsr +pCOM_rsp*m_rsp+pCOM_reb*m_reb + vec4(QP.pRH)*m_rh)
                    +(pCOM_lsr*m_lsr +pCOM_lsp*m_lsp+pCOM_leb*m_leb + vec4(QP.pLH)*m_lh)
                    +(pCOM_torso*m_torso);
            QP.pCOM =QP.pCOM/M_total;

            return QP;
        }
        QuadPos FK_RF(QuadJoints QJ, vec3 pRF)
        {
            QuadPos QP = FK(QJ);
            vec3 tP = pRF-QP.pRF;
            QP.pRF = QP.pRF+tP;
            QP.pLF = QP.pLF+tP;
            QP.pRH = QP.pRH+tP;
            QP.pLH = QP.pLH+tP;
            QP.pPel = QP.pPel+tP;
            QP.pCOM = QP.pCOM+tP;
            return QP;
        }
        QuadPos FK_LF(QuadJoints QJ, vec3 pLF)
        {
            QuadPos QP = FK(QJ);
            vec3 tP = pLF - QP.pLF;
            QP.pRF = QP.pRF+tP;
            QP.pLF = QP.pLF+tP;
            QP.pRH = QP.pRH+tP;
            QP.pLH = QP.pLH+tP;
            QP.pPel = QP.pPel+tP;
            QP.pCOM = QP.pCOM+tP;
            return QP;
        }
        QuadPos FK_RH(QuadJoints QJ, vec3 pRH)
        {
            QuadPos QP = FK(QJ);
            vec3 tP = pRH- QP.pRH;
            QP.pRF = QP.pRF+tP;
            QP.pLF = QP.pLF+tP;
            QP.pRH = QP.pRH+tP;
            QP.pLH = QP.pLH+tP;
            QP.pPel = QP.pPel+tP;
            QP.pCOM = QP.pCOM+tP;
            return QP;
        }
        QuadPos FK_LH(QuadJoints QJ, vec3 pLH)
        {
            QuadPos QP = FK(QJ);
            vec3 tP = pLH - QP.pLH;
            QP.pRF = QP.pRF+tP;
            QP.pLF = QP.pLF+tP;
            QP.pRH = QP.pRH+tP;
            QP.pLH = QP.pLH+tP;
            QP.pPel = QP.pPel+tP;
            QP.pCOM = QP.pCOM+tP;
            return QP;
        }
    vec3 IK_oneleg(vec3 xyz, int lr)
    {
        vec3 RPK;
        if(xyz.z>-0.08){ xyz.z = -0.08;}
        if(xyz.x>xlim){xyz.x = xlim;}
        if(xyz.x<-xlim){xyz.x = -xlim;}
        if(xyz.y>ylim){xyz.y = ylim;}
        if(xyz.y<-ylim){xyz.y = -ylim;}

        double a,b,c,d;

        double ul = ULEG;
        double ll = LLEG;
        double r2p = R2P*lr;
        double y = xyz.y;
        double z = xyz.z;

        double t_l = sqrtp(y*y + z*z - r2p*r2p);
        //printf("t_l %f\n",t_l);
        if(t_l>ULEG+LLEG-0.02)//2cm less stretched
        {
            t_l = ULEG+LLEG-0.02;//2cm less stretched
        }
        if(t_l<0.03)
        {
            t_l = 0.03;
        }
        //printf("t_l %f\n",t_l);
        a = r2p;    b  = -t_l;
        c = -t_l;   d = -r2p;


        double cR = (d*y+-b*z)/(a*d-b*c);
        double sR = (-c*y+a*z)/(a*d-b*c);


        RPK[0] = -atan2(sR,cR);//reversed;;;;;
        double newx = xyz.x;
        double newy = t_l;
        //printf("t_l2 %f\n",sqrtp(newx*newx+newy*newy));
        if(sqrtp(newx*newx+newy*newy) > ULEG+LLEG-0.02)//2cm less stretched
        {
            double nxt, nyt;
            nxt = newx*((ULEG+LLEG-0.02)/sqrtp(newx*newx+newy*newy));//2cm less stretched
            nyt = newy*((ULEG+LLEG-0.02)/sqrtp(newx*newx+newy*newy));//2cm less stretched
            newx = nxt;
            newy = nyt;
        }
        //printf("t_l2 %f\n",sqrtp(newx*newx+newy*newy));
        double cPminusK = (ul*ul+ll*ll-(newx*newx+newy*newy))/(2*ul*ll);
        //printf("cPminusK %f\n",cPminusK);
        RPK[2] = -(M_PI-acos(cPminusK));

        a = -ll*sin(RPK[2]);
        b  = -ul-ll*cos(RPK[2]);
        c = ul+ll*cos(RPK[2]);
        d = -ll*sin(RPK[2]);

        double cP = (d*newx+-b*newy)/(a*d-b*c);
        double sP = (-c*newx+a*newy)/(a*d-b*c);

        RPK[1] = atan2(sP,cP);

        for(int i=0;i<3;i++)
        {
            if(std::isnan(RPK[i]))
            {
                RPK[i] = 0.0;
                printf("nan!!! xyz %f %f %f\n",xyz.x,xyz.y,xyz.z);
            }
        }
        return RPK;
    }
    QuadJoints IK_pel(QuadPos QP)
    {
        QuadJoints QJ;

        mat4 T_PEL = mat4(QP.pPel,mat3(QP.qPel));
        mat4 i_T_PEL = T_PEL.inverse();
        vec3 QRF_local = i_T_PEL*QP.pRF;
        vec3 QLF_local = i_T_PEL*QP.pLF;
        vec3 QRH_local = i_T_PEL*QP.pRH;
        vec3 QLH_local = i_T_PEL*QP.pLH;
        QRF_local = QRF_local-offset_p2rh;
        QLF_local = QLF_local-offset_p2lh;
        QRH_local = QRH_local-offset_p2rs;
        QLH_local = QLH_local-offset_p2ls;
        vec3 t1AJ;
        t1AJ = IK_oneleg(QRF_local,-1);
        QJ.RHR = t1AJ[0];
        QJ.RHP = t1AJ[1];
        QJ.RKN = t1AJ[2];
        t1AJ = IK_oneleg(QLF_local,1);
        QJ.LHR = t1AJ[0];
        QJ.LHP = t1AJ[1];
        QJ.LKN = t1AJ[2];
        t1AJ = IK_oneleg(QRH_local,-1);
        QJ.RSR = t1AJ[0];
        QJ.RSP = t1AJ[1];
        QJ.REB = t1AJ[2];
        t1AJ = IK_oneleg(QLH_local,1);
        QJ.LSR = t1AJ[0];
        QJ.LSP = t1AJ[1];
        QJ.LEB = t1AJ[2];

        if(QJ.RHR> 35.0*D2Rf) QJ.RHR =  35.0;
        if(QJ.RHR<-35.0*D2Rf) QJ.RHR = -35.0;
        if(QJ.RSR> 35.0*D2Rf) QJ.RSR =  35.0;
        if(QJ.RSR<-35.0*D2Rf) QJ.RSR = -35.0;
        if(QJ.LHR> 35.0*D2Rf) QJ.LHR =  35.0;
        if(QJ.LHR<-35.0*D2Rf) QJ.LHR = -35.0;
        if(QJ.LSR> 35.0*D2Rf) QJ.LSR =  35.0;
        if(QJ.LSR<-35.0*D2Rf) QJ.LSR = -35.0;

        QJ.pPel = QP.pPel;
        QJ.qPel = QP.qPel;

        return QJ;
    }
    QuadJoints IK_COM(QuadPos _QP)//not tested yet
    {

        QuadJoints QJ;
        QuadPos QP = _QP;
        QP.pCOM = vec3(0,0,-1000);
        QP.pPel = _QP.pCOM;//initial value
        int cnt = 0;
        int cnt1 = 0;

        vec3 dpPel;
        vec3 ks = vec3(1,1,1);
        while((QP.pCOM-_QP.pCOM).norm()>err_max)
        {
            QJ = IK_pel(QP);//loop loop loop
            QP = FK(QJ);
            vec3 dpCOM = _QP.pCOM - QP.pCOM;
            if(cnt<60){ dpPel = dpCOM; cnt1++;}
            else
            {
                printf("cnt1 %d cnt %d\n",cnt1, cnt);
                break;
            }

            QP.pPel = QP.pPel + dpPel;//should be improved
            cnt++;
            if(cnt>2000){ break;}

        }

        num_itter = cnt;


        return QJ;
    }

};
#endif // OINVERSE_H
