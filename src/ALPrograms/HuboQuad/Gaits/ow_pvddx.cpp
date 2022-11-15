
#include "ow_pvddx.h"
void OW_PVDDX::pv_test()
{

    zc = 0.350;
    //Open gain values from txt file
    int t_zc = (zc+0.0005)*1000;
    FILE* fp1;
    char tt[] = "../share/OW_GAINS_PVDDX/gj000.txt";
    tt[26] = t_zc/100+'0';
    tt[27] = (t_zc/10)%10+'0';
    tt[28] = t_zc%10+'0';

    fp1 = fopen(tt,"r");
    if(fp1==NULL){printf("no Kdi file 0.%d\n",t_zc);return;}
    for(int i=0;i<NL;i++)
    {
        fscanf(fp1,"%lf",&Kdi[i]);
    }
    fclose(fp1);
    tt[18] = 'x';
    char tt2[] = "../share/OW_GAINS_PVDDX/K000.txt";
    tt2[25] = t_zc/100+'0';
    tt2[26] = (t_zc/10)%10+'0';
    tt2[27] = t_zc%10+'0';
    fp1 = fopen(tt2,"r");
    if(fp1==NULL){printf("no K file \n");return;}

     fscanf(fp1,"%lf %lf %lf", &Kx[0],&Kx[1],&Ke);

    fclose(fp1);

//        printf("Kx %f %fKi %f \n",Kx[0],Kx[1],Ke);
//        printf("Gain load done 0.%d\n",t_zc);
    for(int i=0;i<NL2ms;i++)
    {
        double toff = i*dt;
        double n_dt = toff/dt_pv;
        int n1 = (int)(n_dt);
        int n2 = n1+1;
        double r1 = (n_dt-n1)/1.0;
        double r2 = (n2-n_dt)/1.0;

        Kdi2ms[i] = (r2*Kdi[n1]+r1*Kdi[n2])/dt_pv*dt;
    }

    t_zc = (zc+0.0025)*200;
    t_zc = t_zc*5;
    fp1;
    char tt3[] = "../share/OW_GAINS_PVDDX_QUAD/gj000.txt";
    tt3[31] = t_zc/100+'0';
    tt3[32] = (t_zc/10)%10+'0';
    tt3[33] = t_zc%10+'0';

    fp1 = fopen(tt3,"r");
    if(fp1==NULL){printf("no Kdi file 0.%d\n",t_zc);return;}
    double Kdi2[NL2ms];
    for(int i=0;i<NL2ms;i++)
    {
        Kdi2[i] = Kdi2ms[i];
        fscanf(fp1,"%lf",&Kdi2ms[i]);
    }
    fclose(fp1);
    char tt4[] = "../share/OW_GAINS_PVDDX_QUAD/K000.txt";
    tt4[30] = t_zc/100+'0';
    tt4[31] = (t_zc/10)%10+'0';
    tt4[32] = t_zc%10+'0';
    fp1 = fopen(tt4,"r");
    if(fp1==NULL){printf("no K file \n");return;}

     fscanf(fp1,"%lf %lf %lf", &Kx[0],&Kx[1],&Ke);

    fclose(fp1);

//        printf("Kx %f %fKi %f \n",Kx[0],Kx[1],Ke);
//        printf("Gain load done 0.%d\n",t_zc);
    for(int i=0;i<NL2ms;i++)
    {
        //cout<<Kdi2[i]<<" "<<Kdi2ms[i]<<endl;
    }




}
void OW_PVDDX::preview_init(double _zc, vec3 pCOM, vec3 dCOM, std::deque<vec3> ZMPrs )
{
    COMr = pCOM;
    dCOMr = dCOM;
    zc = _zc;
    for(int i=0;i<NL2ms*4;i++)
    {
        if(i<ZMPrs.size())
        {
            zmp_refs[i] = ZMPrs[i];
        }
        else
        {
            zmp_refs[i] = zmp_refs[i-1];
        }
    }


    int t_zc = (zc+0.0025)*200;
    t_zc = t_zc*5;
    FILE* fp1;
    char tt3[] = "../share/OW_GAINS_PVDDX_QUAD/gj000.txt";
    tt3[31] = t_zc/100+'0';
    tt3[32] = (t_zc/10)%10+'0';
    tt3[33] = t_zc%10+'0';

    fp1 = fopen(tt3,"r");
    if(fp1==NULL){printf("no Kdi file 0.%d\n",t_zc);return;}
    for(int i=0;i<NL2ms;i++)
    {
        fscanf(fp1,"%lf",&Kdi2ms[i]);
    }
    fclose(fp1);
    char tt4[] = "../share/OW_GAINS_PVDDX_QUAD/K000.txt";
    tt4[30] = t_zc/100+'0';
    tt4[31] = (t_zc/10)%10+'0';
    tt4[32] = t_zc%10+'0';
    fp1 = fopen(tt4,"r");
    if(fp1==NULL){printf("no K file \n");return;}

     fscanf(fp1,"%lf %lf %lf", &Kx[0],&Kx[1],&Ke);

    fclose(fp1);

    sumKse = vec3(0,0,0);
    pvcnt = 0;

}
void OW_PVDDX::calc_ddCOM_pvddx()
{

    //push detect and recover
    double w = sqrt(g/zc);


    vec3 sumgp = vec3(0,0,0);
    if(pvcnt==0)
    {
        int it;
        for(it = 0;it<20;it++)
        {
            vec3 COMt,dCOMt;
            COMt = COMr; dCOMt = dCOMr;
            sumgp = vec3(0,0,0);
            for(int k=0;k<NL2ms;k++)
            {
                sumgp=sumgp+Kdi2ms[k]*zmp_refs[k];//k? k+1?
            }
            sumKse = -(Kx[0]*COMt + Kx[1]*dCOMt) -sumgp;
            sumKse = vec3();//

            vec3 u = sumKse + (Kx[0]*COMr + Kx[1]*dCOMt) +sumgp;

            ddCOMr.x = u.x;
            ddCOMr.y = u.y;

            COMt = COMt + dCOMt*dt + dt*dt/2*ddCOMr;//maybe better...
            ZMP = COMt -zc/g*ddCOMr;

            int offnum = (0.2+0.5*dt)/dt;
            double k = 0.01;
            bool ok = false;
            if(1e-5<fabs(ZMP.x-zmp_refs[0].x))
            {
                for(int i=0;i<offnum;i++)
                {
                    zmp_refs[i].x+=k*(ZMP.x-zmp_refs[0].x);
                }
            }
            else
            {
                ok = true;
            }
            if(1e-5<fabs(ZMP.y-zmp_refs[0].y))
            {
                for(int i=0;i<offnum;i++)
                {
                    zmp_refs[i].y+=k*(ZMP.y-zmp_refs[0].y);
                }
            }
            else
            {
                if(ok) break;
            }

            //cout<<it<<" "<<ZMP.x-zmp_refs[0].x<<" "<<ZMP.y-zmp_refs[0].y<<endl;
        }
        //cout<<it<<" "<<ZMP.x-zmp_refs[0].x<<" "<<ZMP.y-zmp_refs[0].y<<endl;
       // exit(0);

    }


    {
        sumgp = vec3(0,0,0);
        for(int k=0;k<NL2ms;k++)
        {
            sumgp=sumgp+Kdi2ms[k]*zmp_refs[k];//k? k+1?
        }
        if(pvcnt==0)//feedforward->change in next ssp
        {
            //printf("FFchanged!\n");
            //should modefy pvddx to change ref
            sumKse = -(Kx[0]*COMr + Kx[1]*dCOMr) -sumgp;
            sumKse = vec3();//
            //init u to be zmp_refs

        }
        vec3 u = sumKse + (Kx[0]*COMr + Kx[1]*dCOMr) +sumgp;

        ddCOMr.x = u.x;
        ddCOMr.y = u.y;
    }
    ZMP = COMr -zc/g*ddCOMr;

    dCOMr.z =0;
    ddCOMr.z =0;
    COMr = COMr + dCOMr*dt + dt*dt/2*ddCOMr;
    dCOMr = dCOMr + dt*ddCOMr;

    sumKse = sumKse+Ke*(zmp_refs[0]-ZMP);

    pvcnt++;

    for(int i=1;i<NL2ms*4;i++)
    {
        zmp_refs[i-1] = zmp_refs[i];
    }

}
