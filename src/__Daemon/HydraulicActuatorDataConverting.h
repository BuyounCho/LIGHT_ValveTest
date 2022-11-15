#ifndef HydraulicActuatorDataConverting_H
#define HydraulicActuatorDataConverting_H

#include <math.h>

void Actuator2Joint_HipPitch(double S, double dS, double F,
                             double &theta, double &dtheta, double &T);
void Actuator2Joint_Knee(double S, double dS, double F,
                         double &theta, double &dtheta, double &T);
bool Actuator2Joint_Ankle(double theta_old, double phi_old,
                          double x_R, double x_L, double dx_R, double dx_L, double F_R, double F_L,
                          double& theta_new, double& phi_new, double& dtheta, double& dphi, double& T_p, double& T_r);

void Rotary2Joint_QuadKnee(double S, double dS, double F,
                            double &theta, double &dtheta, double &T);


#endif // HydraulicActuatorDataConverting_H
