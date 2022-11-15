#ifndef MAIN
#define MAIN
//=====================================================================//
struct LegJoint
{
    double S1, S2;
};
struct FootPos
{
    double X, Y;
};

struct SystemInfo
{
    double pos;
    double vel;
    double cur;

    double est_torque;
    double N_gear;
    double TorqueConst;
};


LegJoint InverseKinematics(float X, float Y);
FootPos SawProfile(float Deceleration, float Departure_Position, float Start_Position, float Departure_Velocity, float elapsed_time);
FootPos TrapezoidalProfile(float Deceleration_1, float Deceleration_2, float Departure_Position, float Start_Position, float Departure_Velocity, float Landing_Velocity, float Cycle, float Stop_Time,float elapsed_time);
FootPos SpringMassModel(float Spring_Constant, float Initial_Pos, float Initial_Vel, float Delay_Time, float Aerial_Seg_num, float Clock);

#endif // MAIN

