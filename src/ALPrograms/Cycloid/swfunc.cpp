
//#include "BasicFiles/BasicSetting.h"
#include "RBLog.h"
#include "main.h"
#include "math.h"

#include "myvariables.h"
LegJoint InverseKinematics(float X, float Y)
{
    LegJoint Lj;
    /*----------------------------------------------------------------------
     * Leg parameters
     ---------------------------------------------------------------------*/
    float       a1 = 30;
    float       b1 = 70;
    float       c1 = 39.3;
    float       d1 = 24;
    float       alpha = 130*D2R;
    float       a2 = 30;
    float       b2 = 94.5;
    float       c2 = 64.96;
    float       d2 = 25;
    float       l1 = 250;
    float       l2 = 250;
    float       theta;
    double      Q1 = 0;
    double      Q2 = 0;

    if (l1+l2 < sqrt(X*X+Y*Y))
    {
        FILE_LOG(logERROR) << "Unreachable solution !!!";
    }

    Q2 = -acos((X*X+Y*Y-l1*l1-l2*l2)/(2*l1*l2)); // elbow-up
    Q1 = atan2(Y,X) + acos((X*X+Y*Y+l1*l1-l2*l2)/(2*l1*sqrt(X*X+Y*Y)));

    if (Q1*R2D > 34.26)
    {
        Q1 = 34.26/R2D;
        FILE_LOG(logERROR) << "Q1 exceeds the upper limit !!!";
    }else if (Q1*R2D < -90)
    {
        Q1 = -90/R2D;
        FILE_LOG(logERROR) << "Q1 exceeds the lower limit !!!";
    }

    if (Q2*R2D < -160.75)
    {
        Q2 = -160.75/R2D;
        FILE_LOG(logERROR) << "Q2 exceeds the lower limit !!!";
    }else if (Q2*R2D > -12)
    {
        Q2 = -12/R2D;
        FILE_LOG(logERROR) << "Q2 exceeds the upper limit !!!";
    }


    theta = PI - (alpha + Q1);
    Lj.S1 = -a1*cos(theta) + sqrt(b1*b1-(d1-a1*sin(theta))*(d1-a1*sin(theta))) - c1;


    theta = PI + Q2;
    Lj.S2 = -a2*cos(theta) + sqrt(b2*b2-(d2-a2*sin(theta))*(d2-a2*sin(theta))) - c2;

    return Lj;
}


FootPos SawProfile(float Deceleration, float Departure_Position, float Start_Position, float Departure_Velocity, float elapsed_time)
{
    FootPos FP;
    /*----------------------------------------------------------------------
     * Leg parameters
     ---------------------------------------------------------------------*/

    float G = 9814; //mm/s^2

    float Velocity_Profile = 0;
    float Position_Profile = 0;
    float End_Position_Profile = 0;
    float End_Velocity_Profile = 0;

    float Departure_Time = 2*(Departure_Position-Start_Position)/Departure_Velocity;
    float Acceleration = Departure_Velocity*Departure_Velocity/(2*(Departure_Position-Start_Position));
    float Landing_Position = 400;
    float Deceleration_Time_1 = -(Departure_Velocity)/Deceleration;
    float Deceleration_Time_2 = -Departure_Velocity/Deceleration;
    float Acceleration_Time_1 = Departure_Velocity/Acceleration;
    float Cycle = Departure_Time+Deceleration_Time_1+Deceleration_Time_2+Acceleration_Time_1;

    if (elapsed_time <= Departure_Time + Cycle*ITERATION)
    {
        End_Velocity_Profile = Acceleration*(elapsed_time-Cycle*ITERATION);
        End_Position_Profile = Start_Position+0.5*Acceleration*(elapsed_time-Cycle*ITERATION)*(elapsed_time-Cycle*ITERATION);
        Velocity_Profile = End_Velocity_Profile;
        Position_Profile = End_Position_Profile;
    }
    else if (elapsed_time <= Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION)
    {
        End_Velocity_Profile = Departure_Velocity + Deceleration*(elapsed_time-Departure_Time-Cycle*ITERATION);
        End_Position_Profile = Departure_Position+0.5*Deceleration*(elapsed_time-Departure_Time-Cycle*ITERATION)*(elapsed_time-Departure_Time-Cycle*ITERATION)+Departure_Velocity*(elapsed_time-Departure_Time-Cycle*ITERATION);
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-Departure_Time-Cycle*ITERATION);
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-Departure_Time-Cycle*ITERATION)*(elapsed_time-Departure_Time-Cycle*ITERATION)+Departure_Velocity*(elapsed_time-Departure_Time-Cycle*ITERATION);
        if (Position_Profile<  End_Position_Profile)
        {
            Velocity_Profile = End_Velocity_Profile;
            Position_Profile = End_Position_Profile;
        }
    }


    else if (elapsed_time <= Departure_Time+Deceleration_Time_1+Deceleration_Time_2+Acceleration_Time_1+ Cycle*ITERATION)
    {
        End_Velocity_Profile = -Departure_Velocity + Acceleration*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION));
        End_Position_Profile = Departure_Position+0.5*Acceleration*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION))*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION))-Departure_Velocity*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-Departure_Time-Cycle*ITERATION);
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-Departure_Time-Cycle*ITERATION)*(elapsed_time-Departure_Time-Cycle*ITERATION)+Departure_Velocity*(elapsed_time-Departure_Time-Cycle*ITERATION);
        if (Position_Profile<  End_Position_Profile)
        {
            Velocity_Profile = End_Velocity_Profile;
            Position_Profile = End_Position_Profile;
        }
 /*       if (elapsed_time > Cycle*(ITERATION+1)-0.005)
            ITERATION = ITERATION+1;
*/
    }

    else
    {
        End_Velocity_Profile = -Departure_Velocity + Acceleration*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION));
        End_Position_Profile = Departure_Position+0.5*Acceleration*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION))*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION))-Departure_Velocity*(elapsed_time-(Departure_Time+Deceleration_Time_1+Deceleration_Time_2+ Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-Departure_Time-Cycle*ITERATION);
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-Departure_Time-Cycle*ITERATION)*(elapsed_time-Departure_Time-Cycle*ITERATION)+Departure_Velocity*(elapsed_time-Departure_Time-Cycle*ITERATION);
        ITERATION = ITERATION+1;
    }


    FP.X = 0;
    FP.Y = End_Position_Profile;

    return FP;
}

FootPos TrapezoidalProfile(float Deceleration_1, float Deceleration_2, float Departure_Position, float Start_Position, float Departure_Velocity, float Landing_Velocity, float Cycle, float Stop_Time,float elapsed_time)
{
    FootPos FP;
    /*----------------------------------------------------------------------
     * Leg parameters
     ---------------------------------------------------------------------*/

    float G = 9814; //mm/s^2

    float Velocity_Profile = 0;
    float Position_Profile = 0;
    float End_Position_Profile = 0;
    float End_Velocity_Profile = 0;

    float Departure_Time = 0.1;
    float Deceleration_Time_1 = -Departure_Velocity/Deceleration_1;
    float Deceleration_Time_2 = Landing_Velocity/Deceleration_2;
    float Acceleration_Time_1;
    float Acceleration_Time_2;
    float Acceleration_1;
    float Acceleration_2;
    float Constant_Vel_Time_1;
    float Constant_Vel_Time_2;


    if(Departure_Time<=(Departure_Position-Start_Position)/Departure_Velocity)
        Departure_Time = (Departure_Position-Start_Position)/Departure_Velocity*1.2;


    if (0.5*Departure_Velocity*Departure_Time<(Departure_Position-Start_Position))
    {
        Acceleration_Time_1 = 2*(Departure_Time - (Departure_Position-Start_Position)/Departure_Velocity);
        Acceleration_1 = Departure_Velocity/Acceleration_Time_1;
        Constant_Vel_Time_1 = Departure_Time - Acceleration_Time_1;
    }
    else
    {
        Departure_Time=(Departure_Position-Start_Position)/(0.5*Departure_Velocity);
        Acceleration_Time_1 = Departure_Time;
        Acceleration_1 = Departure_Velocity/Acceleration_Time_1;
        Constant_Vel_Time_1 = 0;
    }

    if (-0.5*Landing_Velocity*(Cycle-Departure_Time-Deceleration_Time_1-Deceleration_Time_2-Stop_Time)<(Departure_Position-Start_Position)+0.5*Departure_Velocity*Deceleration_Time_1+0.5*Landing_Velocity*Deceleration_Time_2)
    {
        Acceleration_Time_2 = 2*((Cycle-Departure_Time-Deceleration_Time_1-Deceleration_Time_2-Stop_Time) + ((Departure_Position-Start_Position)-0.5*Departure_Velocity*Departure_Velocity/Deceleration_1+0.5*Landing_Velocity*Landing_Velocity/Deceleration_2)/Landing_Velocity);
        Acceleration_2 = -Landing_Velocity/Acceleration_Time_2;
        Constant_Vel_Time_2 =  Cycle-Departure_Time-Deceleration_Time_1-Deceleration_Time_2-Stop_Time-Acceleration_Time_2;
    }

    else
    {
        Acceleration_Time_2=((Departure_Position-Start_Position)+0.5*Departure_Velocity*Deceleration_Time_1+0.5*Landing_Velocity*Deceleration_Time_2)/(-0.5*Landing_Velocity);
        Cycle = Acceleration_Time_2 + Departure_Time +Deceleration_Time_1+Deceleration_Time_2+Stop_Time;
        Acceleration_2 = -Landing_Velocity/Acceleration_Time_2;
        Constant_Vel_Time_2 = 0;
    }



    if (elapsed_time <= Acceleration_Time_1 + Cycle*ITERATION)
    {
        End_Velocity_Profile = Acceleration_1*(elapsed_time-Cycle*ITERATION);
        End_Position_Profile = Start_Position+0.5*Acceleration_1*(elapsed_time-Cycle*ITERATION)*(elapsed_time-Cycle*ITERATION);
        Velocity_Profile = End_Velocity_Profile;
        Position_Profile = End_Position_Profile;
    }

    else if (elapsed_time <= Departure_Time + Cycle*ITERATION)
    {
        End_Velocity_Profile = Departure_Velocity;
        End_Position_Profile = (Start_Position+0.5*Departure_Velocity*Acceleration_Time_1)+Departure_Velocity*(elapsed_time-(Acceleration_Time_1 + Cycle*ITERATION));
        Velocity_Profile = End_Velocity_Profile;
        Position_Profile = End_Position_Profile;
    }

    else if (elapsed_time <= Departure_Time+Deceleration_Time_1+Cycle*ITERATION)
    {
        End_Velocity_Profile = Departure_Velocity + Deceleration_1*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        End_Position_Profile = Departure_Position+0.5*Deceleration_1*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
    }

    else if (elapsed_time <= Departure_Time+Deceleration_Time_1+ Stop_Time + Cycle*ITERATION)
    {
        End_Velocity_Profile = 0;
        End_Position_Profile = (Start_Position+Departure_Velocity*(Constant_Vel_Time_1+0.5*(Acceleration_Time_1+Deceleration_Time_1)));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
    }

    else if (elapsed_time <= Departure_Time+Deceleration_Time_1+Stop_Time+Deceleration_Time_2 + Cycle*ITERATION)
    {
        End_Velocity_Profile = Deceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Cycle*ITERATION));
        End_Position_Profile = (Start_Position+Departure_Velocity*(Constant_Vel_Time_1+0.5*(Acceleration_Time_1+Deceleration_Time_1)))+0.5*Deceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
    }

    else if (elapsed_time <= Departure_Time+Deceleration_Time_1 + Stop_Time+ Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION)
    {
        End_Velocity_Profile = Landing_Velocity;
        End_Position_Profile = (Start_Position+Departure_Velocity*(Constant_Vel_Time_1+0.5*(Acceleration_Time_1+Deceleration_Time_1))+0.5 * Landing_Velocity * Deceleration_Time_2)+ Landing_Velocity *(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time +Deceleration_Time_2+Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
    }

    else if (elapsed_time <= Cycle*(ITERATION+1))
    {
        End_Velocity_Profile = Landing_Velocity + Acceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time +Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION));
        End_Position_Profile = (Start_Position+Departure_Velocity*(Constant_Vel_Time_1+0.5*(Acceleration_Time_1+Deceleration_Time_1))+ Landing_Velocity *(0.5 * Deceleration_Time_2 +Constant_Vel_Time_2)) + 0.5*Acceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION))*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION))+Landing_Velocity*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time +Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
/*        if (elapsed_time > Cycle*(ITERATION+1)-0.005)
            ITERATION = ITERATION+1;
        FILE_LOG(logSUCCESS) << "Q1 exceeds the upper limit !!!";
*/
    }

    else
    {
        //printf("33333\n, iteration = %f\n",ITERATION);
        End_Velocity_Profile = Landing_Velocity + Acceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time +Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION));
        End_Position_Profile = (Start_Position+Departure_Velocity*(Constant_Vel_Time_1+0.5*(Acceleration_Time_1+Deceleration_Time_1))+ Landing_Velocity *(0.5 * Deceleration_Time_2 +Constant_Vel_Time_2)) + 0.5*Acceleration_2*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION))*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time+Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION))+Landing_Velocity*(elapsed_time-(Departure_Time+Deceleration_Time_1+ Stop_Time +Deceleration_Time_2 + Constant_Vel_Time_2+ Cycle*ITERATION));
        Velocity_Profile = Departure_Velocity - G*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        Position_Profile = Departure_Position-0.5*G*(elapsed_time-(Departure_Time+Cycle*ITERATION))*(elapsed_time-(Departure_Time+Cycle*ITERATION))+Departure_Velocity*(elapsed_time-(Departure_Time+Cycle*ITERATION));
        ITERATION = ITERATION+1;
    }


    if (Position_Profile <=  End_Position_Profile)
    {
        Velocity_Profile = End_Velocity_Profile;
        Position_Profile = End_Position_Profile;
    }

    FP.X = 0;
    FP.Y = End_Position_Profile;

    return FP;
}

FootPos SpringMassModel(float Spring_Constant, float Initial_Pos, float Initial_Vel, float Delay_Time, float Aerial_Seg_num, float Clock)
{
    FootPos FP;
    /*----------------------------------------------------------------------
     * Leg parameters
     ---------------------------------------------------------------------*/

    float g = 9.814; //m/s^2
    float Mass = 5; //kg
    float w = sqrt(Spring_Constant/Mass);
    float v0 = Initial_Vel/1000; //m/s
    float tc = (2/w)*atan2(v0,-g/w);
    float ta = 2*v0/g + Delay_Time;
    float ta1 = ta/Aerial_Seg_num;
    float ta2 = ta*(Aerial_Seg_num-1)/Aerial_Seg_num;
    float a1 = v0/ta1;
    float d1 = v0*ta1 - 0.5*a1*ta1*ta1;
    float y0 = -((v0/w)*sin(w*(tc/2)) - (g/(w*w))*cos(w*(tc/2)) + g/(w*w));

    float T = tc + ta + Delay_Time;

    float Y = 0;
    float Y_robot = 0;
    float Clock_aerial_1 = 0;
    float Clock_aerial_2 = 0;
    float Clock_contact_2 = 0;

    //FLOOR_POS= Initial_Pos-((v0/w)*sin(w*(tc/2)) - (g/(w*w))*cos(w*(tc/2)) + g/(w*w))*1000;

    //printf("FLOOR_POS= %f\n",FLOOR_POS);

    if (Clock <= 0.5*tc + T*ITERATION)
    {
        Y = -((v0/w)*sin(w*((Clock + tc/2) - T*ITERATION)) - (g/(w*w))*cos(w*((Clock + tc/2) - T*ITERATION)) + g/(w*w));
        Y_robot = -((v0/w)*sin(w*((Clock + tc/2) -T*ITERATION)) - (g/(w*w))*cos(w*((Clock + tc/2) - T*ITERATION)) + g/(w*w));
    }

    else if (Clock <= (0.5*tc + ta + Delay_Time) + T*ITERATION)
    {
        Clock_aerial_1 = Clock - (0.5*tc + T*ITERATION);
        Y = v0*Clock_aerial_1 - 0.5*g*Clock_aerial_1*Clock_aerial_1;
        if (Clock <= (0.5*tc + ta1) + T*ITERATION)
        {
            Y_robot = v0*Clock_aerial_1 - 0.5*a1*Clock_aerial_1*Clock_aerial_1;
        }
        else if (Clock <= (0.5*tc + ta2 + Delay_Time) + T*ITERATION)
        {
            Y_robot = d1;
        }
        else if (Clock <= (0.5*tc + ta + Delay_Time) + T*ITERATION)
        {
            Clock_aerial_2 = Clock - (0.5*tc + ta2 + Delay_Time + T*ITERATION);
            Y_robot = d1 - 0.5*a1*Clock_aerial_2*Clock_aerial_2;
        }
    }
    else if (Clock <= T + T*ITERATION)
    {
        Clock_contact_2 = Clock - ((0.5*tc + ta + Delay_Time) + T*ITERATION);
        Y = -((v0/w)*sin(w*(Clock_contact_2)) - (g/(w*w))*cos(w*(Clock_contact_2)) + g/(w*w));
        Y_robot = -((v0/w)*sin(w*(Clock_contact_2)) - (g/(w*w))*cos(w*(Clock_contact_2)) + g/(w*w));
        if (Clock > T+ T*ITERATION - 0.005)
        {
            ITERATION = ITERATION + 1;
        }
    }


  /*  if (ITERATION != MAX_ITERATION)
    {
        if (Clock < tc + T*ITERATION)
        {
            Y_robot = -((v0/w)*sin(w*(Clock-T*ITERATION)) - (g/(w*w))*cos(w*(Clock-T*ITERATION)) + g/(w*w));
            DY_robot = -(v0*cos(w*(Clock-T*ITERATION)) + (g/w)*sin(w*(Clock-T*ITERATION)));
        }
        else if (Clock < T + T*ITERATION)
        {
            Clock_aerial = Clock - (tc + T*ITERATION);
            if (Clock < (tc + (ta-Delay_Time)/Aerial_Seg_num)+T*ITERATION)
            {
                DY_robot = v0 - aerial_decel*Clock_aerial;
                Y_robot = v0 * Clock_aerial - 0.5*aerial_decel*Clock_aerial*Clock_aerial;
            }
            else if (Clock < (tc + Delay_Time + (ta-Delay_Time)*(Aerial_Seg_num-1)/Aerial_Seg_num)+T*ITERATION)
            {
                DY_robot = 0;
                Y_robot = v0 * ((ta-Delay_Time)/Aerial_Seg_num) - 0.5*aerial_decel*((ta-Delay_Time)/Aerial_Seg_num)*((ta-Delay_Time)/Aerial_Seg_num);
            }
            else if (Clock < (tc + ta)+T*ITERATION)
            {
                Clock_aerial_2 = Clock - (tc + Delay_Time + (ta-Delay_Time)*(Aerial_Seg_num-1)/Aerial_Seg_num+ T*ITERATION);
                DY_robot = -aerial_decel*Clock_aerial_2;
                Y_robot = v0 * ((ta-Delay_Time)/Aerial_Seg_num) - 0.5*aerial_decel*((ta-Delay_Time)/Aerial_Seg_num)*((ta-Delay_Time)/Aerial_Seg_num) - 0.5*aerial_decel*Clock_aerial_2*Clock_aerial_2;
                if (Clock > (tc + ta)+T*ITERATION - 0.005)
                {
                    ITERATION = ITERATION + 1;
                    printf("1111: iteration = %f\n",ITERATION);
                }
            }
        }
    }
    else
    {
        if (Clock < T*ITERATION + tc/2)
        {
            Y_robot = -((v0/w)*sin(w*(Clock-T*ITERATION)) - (g/(w*w))*cos(w*(Clock-T*ITERATION)) + g/(w*w));
            DY_robot = -(v0*cos(w*(Clock-T*ITERATION)) + (g/w)*sin(w*(Clock-T*ITERATION)));
            if (Clock > T*ITERATION+tc/2 - 0.005)
            {
                ITERATION = ITERATION + 1;
                printf("1111: iteration = %f\n",ITERATION);
            }
        }
    } */

    Y = (Y + fabs(y0))*1000; // m -> mm
    Y_robot = (Y_robot + fabs(y0))*1000; // m -> mm
    FP.X = 0;
    FP.Y = Y_robot + Initial_Pos;

    return FP;
}
