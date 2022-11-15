#ifndef OW_PVDDX_H
#define OW_PVDDX_H
#include "Oinverse.h"
#include "BasicMatrix.h"
#include "OW_RBDL.h"
#include <deque>

class OW_PVDDX
{
    private:
    double dt;
    double e;
    double g;


public:
    const static int NL = 300;//how long?? 1.5sec?
    const static int NL2ms = 750;//how long?? 1.5sec?
    double dt_pv;
    vec3 ZMP;
    vec3 COMr, dCOMr, ddCOMr;
private:
    //ratio of ssp and dsp
    double Kdi[NL];
    double Kdi2ms[NL2ms];
    double Ke,Kx[2];
public:
    vec3 zmp_refs[NL2ms*4];

    double zc;


    vec3 sumKse;
    vec3 sumKsedy;
    int pvcnt;

public:

    OW_PVDDX()
    {
        dt = 0.002;
        dt_pv = 0.005;
        e = M_E;
        g = 9.81;
    }

    void pv_test();

    void preview_init(double _zc, vec3 pCOM, vec3 dCOM, std::deque<vec3> ZMPrs );

    void calc_ddCOM_pvddx();


};
#endif // OW_PREVIEW_H
