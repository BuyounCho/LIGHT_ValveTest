#include "ow_quad.h"

void OW_Quad::start_standing_control(int _cTT)
{
    cTT = _cTT;
    isStandingControl = true;
    stopcnt = 100;
}
void OW_Quad::stop_standing_control()
{
    QJ_CRef.RHR = QJ_CRef.RHP = QJ_CRef.RKN = 0;
    QJ_CRef.LHR = QJ_CRef.LHP = QJ_CRef.LKN = 0;
    QJ_CRef.RSR = QJ_CRef.RSP = QJ_CRef.REB = 0;
    QJ_CRef.LSR = QJ_CRef.LSP = QJ_CRef.LEB = 0;
    QJ_CFF.RHR = QJ_CFF.RHP = QJ_CFF.RKN = 0;
    QJ_CFF.LHR = QJ_CFF.LHP = QJ_CFF.LKN = 0;
    QJ_CFF.RSR = QJ_CFF.RSP = QJ_CFF.REB = 0;
    QJ_CFF.LSR = QJ_CFF.LSP = QJ_CFF.LEB = 0;
    isStandingControl = false;
    isWalking = false;
    save_all(1);
}
void OW_Quad::start_Walking()
{
    isWalking = true;
    isStopping = true;
    isCOMadjusting = true;
    isCOMadjusting_finish = false;
    if(WP.Gait==Trot||WP.Gait==Wave||WP.Gait==Wave2||WP.Gait==Standing)
    {
        stopcnt = 85000;
        //stopcnt = 10;
    }
    else if(WP.Gait==Pronk)
    {
        stopcnt =3;
    }
    else if(WP.Gait==Flytrot)
    {
        stopcnt = 600;
    }
    else
    {
        stopcnt = 3;
    }
}
void OW_Quad::stop_Walking()//walking, slowalking
{
    if(WP.Gait==Standing)
    {
        isFinishing = true;
        return;
    }
    WalkParams tWP = WP;
    tWP.step_Rot = 0;
    tWP.step_L = vec3(0,0,0);
    tWP.FB_L = Oi.P2HX*2;//walkready_ready
    tWP.LR_L = Oi.P2HY*2+Oi.R2P*2;//walkready_ready
    isStopping = true;

    stopcnt = 2;

    changeWalk(tWP);
}
void OW_Quad::changeWalk(WalkParams _WP)
{
    WP_next = _WP;
    walkchanged = true;
    isStopping = true;
}

double OW_Quad::fComp(double dmnow, double dmMaxF, double fricA)
{
    if(dmnow>dmMaxF){return fricA;}
    if(dmnow<-dmMaxF){return -fricA;}
    return fricA*(dmnow)/dmMaxF;
}

