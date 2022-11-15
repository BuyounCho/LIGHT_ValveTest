#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

void PrintHere(int n)
{
    static int cnt = 0;
    if(n == 0) {
        FILE_LOG(logDEBUG1) << "I'm Here : " << cnt;
        cnt++;
    } else {
        FILE_LOG(logDEBUG) << "I'm Here : " << n;
    }
}

void linear_trajectory_pressure(double tf, double tn,
                                double Pnow, double Pfin,
                                double &Pnext, double &dPnext)
{
    // t : Left time
    // Pnow : current position
    // Pfin : final position
    // Pnext : next step position, velocity

    double k[2] = {0.0, }; // Coefficient for 'third order polynomial trajectory'
    double theta = Pnow;
    double theta_f = Pfin;

    if(tf < 1e-6)
    {
        Pnext = theta;
    } else if (tf < tn) {
        Pnext = Pfin;
        dPnext = 0.0;
    } else {
        double t = tf;
        k[0] = theta;
        k[1] = (theta_f-theta)/t;

        double dt = tn;
        Pnext = k[0] + k[1]*dt;
        dPnext = k[1];
    }
}
