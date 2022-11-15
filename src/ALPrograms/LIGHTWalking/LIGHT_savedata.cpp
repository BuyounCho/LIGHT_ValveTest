#include "LIGHT_savedata.h"
#include "LIGHT_var_and_func.h"
#include "LIGHT_robotmodel.h"
#include "LIGHT_motion.h"
#include <iomanip>

using namespace std;
using namespace std::chrono;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern LIGHTWholeBody LIGHT;
extern LIGHTWholeMotions LIGHT_WholeMotions;

bool            save_PutDataFlag = false;
bool            save_ActivateFlag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];

bool            save_PutDataFlag_SupPres = false;
bool            save_ActivateFlag_SupPres = false;
unsigned int    save_Index_SupPres = 0;
float           save_Buf_SupPres[SAVEKIND_SupPres][SAVENUM_SupPres];

//bool            save_Flag_SYS_ID;
//unsigned int    save_Index_SYS_ID;
//float           save_Buf_SYS_ID[SAVEKIND_SYS_ID][SAVENUM_SYS_ID];
//void            save_File_SYS_ID(int _DataType);
//void            save_PutData_SYS_ID(unsigned int cur_Index, int _DataType, double _CoM_Mag, double _ratio_Freq, double _max_Freq);
//int             save_DataType_SYS_ID;
//bool            SaveFlagOn_SYS_ID(int _DataType);
//bool            SaveFlagOFFandSaveData_SYS_ID();


void save_File(char *filecomment)
{
    int n_col = save_Index;
    save_Index = 0;

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "DataLog/LIGHT_AL_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    if(!filecomment[0] == NULL) {
        strcat(filename, "_");
        strcat(filename, filecomment);
    }
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<n_col ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    save_Index = 0;
    std::cout << "Saved Filename : " << filename << std::endl;
}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT_WALKING;

    // RIGHT HIP ROLL (Board 0)
    save_Buf[1][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefAngle;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentAngle;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentActVel;
    save_Buf[4][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefTorque;
    save_Buf[5][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentTorque;

    // RIGHT HIP YAW (Board 1)
    save_Buf[6][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentRefAngle;
    save_Buf[7][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentAngle;
    save_Buf[8][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentActVel;
    save_Buf[9][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentRefTorque;
    save_Buf[10][cur_Index] = sharedSEN->ENCODER[RHY][0].CurrentTorque;

    // RIGHT HIP PITCH (Board 2)
    save_Buf[11][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefAngle;
    save_Buf[12][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentAngle;
    save_Buf[13][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentAngVel;
    save_Buf[14][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefActPos;
    save_Buf[15][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentActPos;
    save_Buf[16][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentActVel;
    save_Buf[17][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefActForce;
    save_Buf[18][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentActForce;
    save_Buf[19][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefTorque;
    save_Buf[20][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentTorque;

    // RIGHT KNEE (Board 3)
    save_Buf[21][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefAngle;
    save_Buf[22][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentAngle;
    save_Buf[23][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentAngVel;
    save_Buf[24][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefActPos;
    save_Buf[25][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentActPos;
    save_Buf[26][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentActVel;
    save_Buf[27][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefActForce;
    save_Buf[28][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentActForce;
    save_Buf[29][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentRefTorque;
    save_Buf[30][cur_Index] = sharedSEN->ENCODER[RKN][0].CurrentTorque;

    // RIGHT ANKLE1 (Board 4)
    save_Buf[31][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefAngle;
    save_Buf[32][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentAngle;
    save_Buf[33][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentAngVel;
    save_Buf[34][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefActPos;
    save_Buf[35][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentActPos;
    save_Buf[36][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentActVel;
    save_Buf[37][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefActForce;
    save_Buf[38][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentActForce;
    save_Buf[39][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefTorque;
    save_Buf[40][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentTorque;

    // RIGHT ANKLE2 (Board 5)
    save_Buf[41][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefAngle;
    save_Buf[42][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentAngle;
    save_Buf[43][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentAngVel;
    save_Buf[44][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefActPos;
    save_Buf[45][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentActPos;
    save_Buf[46][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentActVel;
    save_Buf[47][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefActForce;
    save_Buf[48][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentActForce;
    save_Buf[49][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefTorque;
    save_Buf[50][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentTorque;

    // LEFT HIP ROLL (Board 6)
    save_Buf[51][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentRefAngle;
    save_Buf[52][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentAngle;
    save_Buf[53][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentActVel;
    save_Buf[54][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentRefTorque;
    save_Buf[55][cur_Index] = sharedSEN->ENCODER[LHR][0].CurrentTorque;

    // LEFT HIP YAW (Board 7)
    save_Buf[56][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentRefAngle;
    save_Buf[57][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentAngle;
    save_Buf[58][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentActVel;
    save_Buf[59][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentRefTorque;
    save_Buf[60][cur_Index] = sharedSEN->ENCODER[LHY][0].CurrentTorque;

    // LEFT HIP PITCH (Board 8)
    save_Buf[61][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefAngle;
    save_Buf[62][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentAngle;
    save_Buf[63][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentAngVel;
    save_Buf[64][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefActPos;
    save_Buf[65][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentActPos;
    save_Buf[66][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentActVel;
    save_Buf[67][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefActForce;
    save_Buf[68][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentActForce;
    save_Buf[69][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentRefTorque;
    save_Buf[70][cur_Index] = sharedSEN->ENCODER[LHP][0].CurrentTorque;

    // LEFT KNEE (Board 9)
    save_Buf[71][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefAngle;
    save_Buf[72][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentAngle;
    save_Buf[73][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentAngVel;
    save_Buf[74][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefActPos;
    save_Buf[75][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentActPos;
    save_Buf[76][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentActVel;
    save_Buf[77][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefActForce;
    save_Buf[78][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentActForce;
    save_Buf[79][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentRefTorque;
    save_Buf[80][cur_Index] = sharedSEN->ENCODER[LKN][0].CurrentTorque;

    // LEFT ANKLE1 (Board 10)
    save_Buf[81][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefAngle;
    save_Buf[82][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentAngle;
    save_Buf[83][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentAngVel;
    save_Buf[84][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefActPos;
    save_Buf[85][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActPos;
    save_Buf[86][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActVel;
    save_Buf[87][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefActForce;
    save_Buf[88][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActForce;
    save_Buf[89][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefTorque;
    save_Buf[90][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentTorque;

    // LEFT ANKLE2 (Board 11)
    save_Buf[91][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefAngle;
    save_Buf[92][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentAngle;
    save_Buf[93][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentAngVel;
    save_Buf[94][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefActPos;
    save_Buf[95][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActPos;
    save_Buf[96][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActVel;
    save_Buf[97][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefActForce;
    save_Buf[98][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActForce;
    save_Buf[99][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefTorque;
    save_Buf[100][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentTorque;

    // PUMP
    save_Buf[101][cur_Index] = sharedSEN->PUMP[0].CurrentRefPressure;
    save_Buf[102][cur_Index] = sharedSEN->PUMP[0].CurrentPressure;
    save_Buf[103][cur_Index] = sharedSEN->PUMP[0].CurrentRefVelocity;
    save_Buf[104][cur_Index] = sharedSEN->PUMP[0].CurrentVelocity;

    // Calculation Time
    save_Buf[105][cur_Index] = CalcTime_WALKING;

    // RightFoot >> CoM Position (Workspace coordinates)
    save_Buf[111][cur_Index] = LIGHT.Xref_RF2CoM(0);
    save_Buf[112][cur_Index] = LIGHT.Xref_RF2CoM(1);
    save_Buf[113][cur_Index] = LIGHT.Xref_RF2CoM(2);
    save_Buf[114][cur_Index] = LIGHT.Xnow_RF2CoM(0);
    save_Buf[115][cur_Index] = LIGHT.Xnow_RF2CoM(1);
    save_Buf[116][cur_Index] = LIGHT.Xnow_RF2CoM(2);

    // RightFoot >> LeftFoot Position (Workspace coordinates)
    save_Buf[117][cur_Index] = LIGHT.Xref_RF2LF(0);
    save_Buf[118][cur_Index] = LIGHT.Xref_RF2LF(1);
    save_Buf[119][cur_Index] = LIGHT.Xref_RF2LF(2);
    save_Buf[120][cur_Index] = LIGHT.Xnow_RF2LF(0);
    save_Buf[121][cur_Index] = LIGHT.Xnow_RF2LF(1);
    save_Buf[122][cur_Index] = LIGHT.Xnow_RF2LF(2);

    // Pelvis >> RightFoot Position (Workspace coordinates)
    save_Buf[123][cur_Index] = LIGHT.Xref_Pel2RF(0);
    save_Buf[124][cur_Index] = LIGHT.Xref_Pel2RF(1);
    save_Buf[125][cur_Index] = LIGHT.Xref_Pel2RF(2);
    save_Buf[126][cur_Index] = LIGHT.Xnow_Pel2RF(0);
    save_Buf[127][cur_Index] = LIGHT.Xnow_Pel2RF(1);
    save_Buf[128][cur_Index] = LIGHT.Xnow_Pel2RF(2);

    // LeftFoot >> CoM Position (Workspace coordinates)
    save_Buf[129][cur_Index] = LIGHT.Xref_LF2CoM(0);
    save_Buf[130][cur_Index] = LIGHT.Xref_LF2CoM(1);
    save_Buf[131][cur_Index] = LIGHT.Xref_LF2CoM(2);
    save_Buf[132][cur_Index] = LIGHT.Xnow_LF2CoM(0);
    save_Buf[133][cur_Index] = LIGHT.Xnow_LF2CoM(1);
    save_Buf[134][cur_Index] = LIGHT.Xnow_LF2CoM(2);

    // LeftFoot >> RightFoot Position (Workspace coordinates)
    save_Buf[135][cur_Index] = LIGHT.Xref_LF2RF(0);
    save_Buf[136][cur_Index] = LIGHT.Xref_LF2RF(1);
    save_Buf[137][cur_Index] = LIGHT.Xref_LF2RF(2);
    save_Buf[138][cur_Index] = LIGHT.Xnow_LF2RF(0);
    save_Buf[139][cur_Index] = LIGHT.Xnow_LF2RF(1);
    save_Buf[140][cur_Index] = LIGHT.Xnow_LF2RF(2);

    // Pelvis >> LeftFoot Position (Workspace coordinates)
    save_Buf[141][cur_Index] = LIGHT.Xref_Pel2LF(0);
    save_Buf[142][cur_Index] = LIGHT.Xref_Pel2LF(1);
    save_Buf[143][cur_Index] = LIGHT.Xref_Pel2LF(2);
    save_Buf[144][cur_Index] = LIGHT.Xnow_Pel2LF(0);
    save_Buf[145][cur_Index] = LIGHT.Xnow_Pel2LF(1);
    save_Buf[146][cur_Index] = LIGHT.Xnow_Pel2LF(2);

    // RightFoot >> CoM Velocity (Workspace coordinates)
    save_Buf[147][cur_Index] = LIGHT.dXref_RF2CoM(0);
    save_Buf[148][cur_Index] = LIGHT.dXref_RF2CoM(1);
    save_Buf[149][cur_Index] = LIGHT.dXref_RF2CoM(2);
    save_Buf[150][cur_Index] = LIGHT.dXnow_RF2CoM(0);
    save_Buf[151][cur_Index] = LIGHT.dXnow_RF2CoM(1);
    save_Buf[152][cur_Index] = LIGHT.dXnow_RF2CoM(2);

    // LeftFoot >> CoM Velocity (Workspace coordinates)
    save_Buf[153][cur_Index] = LIGHT.dXref_LF2CoM(0);
    save_Buf[154][cur_Index] = LIGHT.dXref_LF2CoM(1);
    save_Buf[155][cur_Index] = LIGHT.dXref_LF2CoM(2);
    save_Buf[156][cur_Index] = LIGHT.dXnow_LF2CoM(0);
    save_Buf[157][cur_Index] = LIGHT.dXnow_LF2CoM(1);
    save_Buf[158][cur_Index] = LIGHT.dXnow_LF2CoM(2);

    // IMU
    save_Buf[159][cur_Index] = sharedSEN->IMU[0].Roll;
    save_Buf[160][cur_Index] = sharedSEN->IMU[0].Pitch;
    save_Buf[161][cur_Index] = sharedSEN->IMU[0].Yaw;
    save_Buf[162][cur_Index] = sharedSEN->IMU[0].Wx_G;
    save_Buf[163][cur_Index] = sharedSEN->IMU[0].Wy_G;
    save_Buf[164][cur_Index] = sharedSEN->IMU[0].Wz_G;
    save_Buf[165][cur_Index] = sharedSEN->IMU[0].Ax_B;
    save_Buf[166][cur_Index] = sharedSEN->IMU[0].Ay_B;
    save_Buf[167][cur_Index] = sharedSEN->IMU[0].Az_B;

    // Minimum Supply Pressure
    save_Buf[168][cur_Index] = sharedSEN->FT[0].Mx;
    save_Buf[169][cur_Index] = sharedSEN->FT[0].My;
    save_Buf[170][cur_Index] = sharedSEN->FT[0].Fz;
    save_Buf[171][cur_Index] = sharedSEN->FT[1].Mx;
    save_Buf[172][cur_Index] = sharedSEN->FT[1].My;
    save_Buf[173][cur_Index] = sharedSEN->FT[1].Fz;
    save_Buf[174][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentTorque;
    save_Buf[175][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentTorque;
    save_Buf[176][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentTorque;
    save_Buf[177][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentTorque;
    save_Buf[178][cur_Index] = sharedSEN->ENCODER[RAR][0].CurrentRefTorque;
    save_Buf[179][cur_Index] = sharedSEN->ENCODER[RAP][0].CurrentRefTorque;
    save_Buf[180][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefTorque;
    save_Buf[181][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefTorque;

//    for(int i=0;i<=sharedREF->N_PrevPump;i++) {
//        save_Buf[183+i][cur_Index] = sharedREF->RequiredPressureReference_Future[i];
//    }

    save_Buf[183][cur_Index] = LIGHT.SavingVector[40](0);
    save_Buf[184][cur_Index] = LIGHT.SavingVector[40](1);
    save_Buf[185][cur_Index] = LIGHT.SavingVector[41](0);
    save_Buf[186][cur_Index] = LIGHT.SavingVector[41](1);
    save_Buf[187][cur_Index] = LIGHT.SavingVector[42](0);
    save_Buf[188][cur_Index] = LIGHT.SavingVector[42](1);
    save_Buf[189][cur_Index] = LIGHT.SavingVector[43](0);
    save_Buf[190][cur_Index] = LIGHT.SavingVector[43](1);
    save_Buf[191][cur_Index] = LIGHT.SavingVector[44](0);
    save_Buf[192][cur_Index] = LIGHT.SavingVector[44](1);
    save_Buf[193][cur_Index] = LIGHT.SavingVector[45](0);
    save_Buf[194][cur_Index] = LIGHT.SavingVector[45](1);
    save_Buf[195][cur_Index] = LIGHT.SavingVector[46](0);
    save_Buf[196][cur_Index] = LIGHT.SavingVector[46](1);
    save_Buf[197][cur_Index] = LIGHT.SavingVector[47](0);
    save_Buf[198][cur_Index] = LIGHT.SavingVector[47](1);
    save_Buf[199][cur_Index] = LIGHT.SavingVector[48](0);
    save_Buf[200][cur_Index] = LIGHT.SavingVector[48](1);
    save_Buf[201][cur_Index] = LIGHT.SavingVector[49](0);
    save_Buf[202][cur_Index] = LIGHT.SavingVector[49](1);

    save_Buf[204][cur_Index] = LIGHT.SavingVector[15](0);
    save_Buf[205][cur_Index] = LIGHT.SavingVector[16](0);
    save_Buf[206][cur_Index] = LIGHT.SavingVector[17](0);
    save_Buf[207][cur_Index] = LIGHT.SavingVector[18](0);
    save_Buf[208][cur_Index] = LIGHT.SavingVector[19](0);
    save_Buf[209][cur_Index] = LIGHT.SavingVector[20](0);
    save_Buf[210][cur_Index] = LIGHT.SavingVector[21](0);
    save_Buf[211][cur_Index] = LIGHT.SavingVector[22](0);
    save_Buf[212][cur_Index] = LIGHT.SavingVector[23](0);
    save_Buf[213][cur_Index] = LIGHT.SavingVector[24](0);
    save_Buf[214][cur_Index] = LIGHT.SavingVector[25](0);
    save_Buf[215][cur_Index] = LIGHT.SavingVector[26](0);

    save_Buf[216][cur_Index] = LIGHT.SavingVector[15](1);
    save_Buf[217][cur_Index] = LIGHT.SavingVector[16](1);
    save_Buf[218][cur_Index] = LIGHT.SavingVector[17](1);
    save_Buf[219][cur_Index] = LIGHT.SavingVector[18](1);
    save_Buf[220][cur_Index] = LIGHT.SavingVector[19](1);
    save_Buf[221][cur_Index] = LIGHT.SavingVector[20](1);
    save_Buf[222][cur_Index] = LIGHT.SavingVector[21](1);
    save_Buf[223][cur_Index] = LIGHT.SavingVector[22](1);
    save_Buf[224][cur_Index] = LIGHT.SavingVector[23](1);
    save_Buf[225][cur_Index] = LIGHT.SavingVector[24](1);
    save_Buf[226][cur_Index] = LIGHT.SavingVector[25](1);
    save_Buf[227][cur_Index] = LIGHT.SavingVector[26](1);

    save_Buf[228][cur_Index] = LIGHT.SavingVector[0](0);
    save_Buf[229][cur_Index] = LIGHT.SavingVector[1](0);
    save_Buf[230][cur_Index] = LIGHT.SavingVector[2](0);
    save_Buf[231][cur_Index] = LIGHT.SavingVector[3](0);
    save_Buf[232][cur_Index] = LIGHT.SavingVector[4](0);
    save_Buf[233][cur_Index] = LIGHT.SavingVector[5](0);
    save_Buf[234][cur_Index] = LIGHT.SavingVector[6](0);
    save_Buf[235][cur_Index] = LIGHT.SavingVector[7](0);
    save_Buf[236][cur_Index] = LIGHT.SavingVector[8](0);
    save_Buf[237][cur_Index] = LIGHT.SavingVector[9](0);
    save_Buf[238][cur_Index] = LIGHT.SavingVector[10](0);
    save_Buf[239][cur_Index] = LIGHT.SavingVector[11](0);

    save_Buf[240][cur_Index] = LIGHT.SavingVector[0](1);
    save_Buf[241][cur_Index] = LIGHT.SavingVector[1](1);
    save_Buf[242][cur_Index] = LIGHT.SavingVector[2](1);
    save_Buf[243][cur_Index] = LIGHT.SavingVector[3](1);
    save_Buf[244][cur_Index] = LIGHT.SavingVector[4](1);
    save_Buf[245][cur_Index] = LIGHT.SavingVector[5](1);
    save_Buf[246][cur_Index] = LIGHT.SavingVector[6](1);
    save_Buf[247][cur_Index] = LIGHT.SavingVector[7](1);
    save_Buf[248][cur_Index] = LIGHT.SavingVector[8](1);
    save_Buf[249][cur_Index] = LIGHT.SavingVector[9](1);
    save_Buf[250][cur_Index] = LIGHT.SavingVector[10](1);
    save_Buf[251][cur_Index] = LIGHT.SavingVector[11](1);

    save_Buf[252][cur_Index] = LIGHT.SavingVector[27](0);
    save_Buf[253][cur_Index] = LIGHT.SavingVector[28](0);
    save_Buf[254][cur_Index] = LIGHT.SavingVector[29](0);
    save_Buf[255][cur_Index] = LIGHT.SavingVector[30](0);
    save_Buf[256][cur_Index] = LIGHT.SavingVector[31](0);
    save_Buf[257][cur_Index] = LIGHT.SavingVector[32](0);
    save_Buf[258][cur_Index] = LIGHT.SavingVector[33](0);
    save_Buf[259][cur_Index] = LIGHT.SavingVector[34](0);
    save_Buf[260][cur_Index] = LIGHT.SavingVector[35](0);
    save_Buf[261][cur_Index] = LIGHT.SavingVector[36](0);
    save_Buf[262][cur_Index] = LIGHT.SavingVector[37](0);
    save_Buf[263][cur_Index] = LIGHT.SavingVector[38](0);

    save_Buf[264][cur_Index] = LIGHT.SavingVector[27](1);
    save_Buf[265][cur_Index] = LIGHT.SavingVector[28](1);
    save_Buf[266][cur_Index] = LIGHT.SavingVector[29](1);
    save_Buf[267][cur_Index] = LIGHT.SavingVector[30](1);
    save_Buf[268][cur_Index] = LIGHT.SavingVector[31](1);
    save_Buf[269][cur_Index] = LIGHT.SavingVector[32](1);
    save_Buf[270][cur_Index] = LIGHT.SavingVector[33](1);
    save_Buf[271][cur_Index] = LIGHT.SavingVector[34](1);
    save_Buf[272][cur_Index] = LIGHT.SavingVector[35](1);
    save_Buf[273][cur_Index] = LIGHT.SavingVector[36](1);
    save_Buf[274][cur_Index] = LIGHT.SavingVector[37](1);
    save_Buf[275][cur_Index] = LIGHT.SavingVector[38](1);
}

void save_File_SupPres()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
//    ss << std::put_time(std::localtime(&in_time_t), "DataLog/LIGHT_FullScenarioTask_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<save_Index_SupPres ; i++)
    {
        for(j=0 ; j<SAVEKIND_SupPres ; j++){
            fprintf(fp, "%f\t", save_Buf_SupPres[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    save_Index_SupPres = 0;
    std::cout << "Saved Filename : " << filename << std::endl;
}

void save_PutData_SupPres(unsigned int cur_Index)
{
    // System Period
    save_Buf_SupPres[0][cur_Index] = SYS_DT_WALKING;

    for(int i=0;i<12;i++) {
        save_Buf_SupPres[1+i][cur_Index] = sharedREF->LoadPressureReference_Future[0][i];
    }

    for(int i=0;i<12;i++) {
        save_Buf_SupPres[13+i][cur_Index] = sharedREF->ActFlowrateReference_Future[0][i];
    }

    for(int i=0;i<12;i++) {
        save_Buf_SupPres[25+i][cur_Index] = 0.0;
    }

//    save_Buf_SupPres[0][cur_Index] = SYS_DT;
//    save_Buf_SupPres[1][cur_Index] = sharedSEN->PUMP[0].CurrentRefPressure;
//    save_Buf_SupPres[2][cur_Index] = sharedSEN->PUMP[0].CurrentPressure;
//    save_Buf_SupPres[3][cur_Index] = sharedSEN->PUMP[0].CurrentRefVelocity;
//    save_Buf_SupPres[4][cur_Index] = sharedSEN->PUMP[0].CurrentVelocity;

}


//bool SaveFlagOn_SYS_ID(int _DataType) {
//    save_Flag_SYS_ID = true;
//    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_DATA_SAVE_SYS_ID;
//    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = true; // Save Flag On!
//    sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = _DataType; // Save Flag On!
//}

//bool SaveFlagOFFandSaveData_SYS_ID() {
//    save_Flag_SYS_ID = false;
//    sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_DATA_SAVE_SYS_ID;
//    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = false; // Save Flag off and Save Data!
//}

//void save_File_SYS_ID(int _DataType)
//{
//    FILE* fp;
//    unsigned int i, j;

//    char file_name[70] = "DataLog/LIGHT_CoM_SYS_ID_Data_";
//    char file_type0[10] = "DSP_RF_X";
//    char file_type1[10] = "DSP_RF_Y";
//    char file_type2[10] = "DSP_LF_X";
//    char file_type3[10] = "DSP_LF_Y";
//    char file_type4[10] = "SSP_RF_X";
//    char file_type5[10] = "SSP_RF_Y";
//    char file_type6[10] = "SSP_LF_X";
//    char file_type7[10] = "SSP_LF_Y";
//    char extension[5] = ".txt";

//    if (_DataType == 0) {
//        strcat(file_name,file_type0);
//    } else if (_DataType == 1) {
//        strcat(file_name,file_type1);
//    } else if (_DataType == 2) {
//        strcat(file_name,file_type2);
//    } else if (_DataType == 3) {
//        strcat(file_name,file_type3);
//    } else if (_DataType == 4) {
//        strcat(file_name,file_type4);
//    } else if (_DataType == 5) {
//        strcat(file_name,file_type5);
//    } else if (_DataType == 6) {
//        strcat(file_name,file_type6);
//    } else if (_DataType == 7) {
//        strcat(file_name,file_type7);
//    }
//    strcat(file_name,extension);

//    fp = fopen(file_name, "w");
//    for(i=0 ; i<save_Index_SYS_ID ; i++)
//    {
//        for(j=0 ; j<SAVEKIND_SYS_ID ; j++){
//            fprintf(fp, "%f\t", save_Buf_SYS_ID[j][i]);
//        }
//        fprintf(fp, "\n");
//    }
//    fclose(fp);

//    save_Index_SYS_ID=0;
//    save_Flag_SYS_ID=0;

//    FILE_LOG(logSUCCESS) << "SYS ID Data Saving is Finished!!";
//}

//void save_PutData_SYS_ID(unsigned int cur_Index, int _DataType, double _CoM_Mag, double _ratio_Freq, double _max_Freq)
//{
//    // System Period
//    save_Buf_SYS_ID[0][0] = SYS_DT;
//    save_Buf_SYS_ID[0][1] = _CoM_Mag;
//    save_Buf_SYS_ID[0][2] = _ratio_Freq;
//    save_Buf_SYS_ID[0][3] = _max_Freq;

//    if(_DataType == 0) // DSP RF X
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_RF2CoM(0);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_RF2CoM(0);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 1) // DSP RF Y
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_RF2CoM(1);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_RF2CoM(1);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 2) // DSP RF X
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_LF2CoM(0);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_LF2CoM(0);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 3) // DSP RF Y
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_LF2CoM(1);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_LF2CoM(1);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 4) // SSP RF X
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_RF2CoM(0);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_RF2CoM(0);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 5) // SSP RF Y
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_RF2CoM(1);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_RF2CoM(1);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 6) // SSP RF X
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_LF2CoM(0);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_LF2CoM(0);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//    else if(_DataType == 7) // SSP RF Y
//    {
//        save_Buf_SYS_ID[1][cur_Index] = LIGHT.Xdes_LF2CoM(1);
//        save_Buf_SYS_ID[2][cur_Index] = LIGHT.Xnow_LF2CoM(1);
//        save_Buf_SYS_ID[3][cur_Index] = _DataType;
//    }
//}

void save_File_TuningParameters()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
//    ss << std::put_time(std::localtime(&in_time_t), "DataLog/LIGHT_AL_Data/TuningParameters/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    fprintf(fp, "SUPPORTCONTROL_DSP : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.WeightCompensation_DSP);
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_pos_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_pos_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_ori_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_ori_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_pos_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_pos_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_ori_DSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_ori_DSP(0));
    fprintf(fp, "\n\n");

    fprintf(fp, "SUPPORTCONTROL_RSSP : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.WeightCompensation_RSSP);
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_pos_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_pos_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_ori_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_ori_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_pos_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_pos_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_ori_RSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_ori_RSSP(0));
    fprintf(fp, "\n\n");

    fprintf(fp, "SUPPORTCONTROL_LSSP : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.WeightCompensation_LSSP);
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_pos_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_pos_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_body_ori_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_body_ori_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_pos_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_pos_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_ori_LSSP(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_ori_LSSP(0));
    fprintf(fp, "\n\n");

    fprintf(fp, "SUPPORTCONTROL_FLOAT : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_pos_Float(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_pos_Float(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Freq_foot_ori_Float(0));
    fprintf(fp, "%f\t", LIGHT_WholeMotions.Zeta_foot_ori_Float(0));
    fprintf(fp, "\n\n");

    fprintf(fp, "JOINGIMPEDANCE_RF (On Contact) : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OnContact(2));
    fprintf(fp, "\n\n");

    fprintf(fp, "JOINGIMPEDANCE_RF (Off Contact) : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.K_RF_ori_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.K_RF_pos_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.D_RF_ori_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.D_RF_pos_OffContact(2));
    fprintf(fp, "\n\n");

    fprintf(fp, "JOINGIMPEDANCE_LF (On Contact) : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OnContact(2));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OnContact(0));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OnContact(1));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OnContact(2));
    fprintf(fp, "\n\n");

    fprintf(fp, "JOINGIMPEDANCE_LF (Off Contact) : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.K_LF_ori_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.K_LF_pos_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.D_LF_ori_OffContact(2));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OffContact(0));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OffContact(1));
    fprintf(fp, "%f\t", LIGHT.D_LF_pos_OffContact(2));
    fprintf(fp, "\n\n");

    fprintf(fp, "CoMLEADCOMPENSATE : ");
    fprintf(fp, "\n");
    fprintf(fp, "%f\t", LIGHT_WholeMotions.CoM_wn_X);
    fprintf(fp, "%f\t", LIGHT_WholeMotions.CoM_wn_Y);
    fprintf(fp, "%f\t", LIGHT_WholeMotions.CoM_zeta_X);
    fprintf(fp, "%f\t", LIGHT_WholeMotions.CoM_zeta_Y);
    fprintf(fp, "\n\n");

    fclose(fp);
}

void Show_TuningParameters()
{
    cout << "[ SUPPORTCONTROL_DSP ]" << endl;
    cout << "WeightCompensation_DSP : " << LIGHT_WholeMotions.WeightCompensation_DSP << endl;
    cout << "Freq_body_pos_DSP : " << LIGHT_WholeMotions.Freq_body_pos_DSP(0) << endl;
    cout << "Zeta_body_pos_DSP : " << LIGHT_WholeMotions.Zeta_body_pos_DSP(0) << endl;
    cout << "Freq_body_ori_DSP : " << LIGHT_WholeMotions.Freq_body_ori_DSP(0) << endl;
    cout << "Zeta_body_ori_DSP : " << LIGHT_WholeMotions.Zeta_body_ori_DSP(0) << endl;
    cout << "Freq_foot_pos_DSP : " << LIGHT_WholeMotions.Freq_foot_pos_DSP(0) << endl;
    cout << "Zeta_foot_pos_DSP : " << LIGHT_WholeMotions.Zeta_foot_pos_DSP(0) << endl;
    cout << "Freq_foot_ori_DSP : " << LIGHT_WholeMotions.Freq_foot_ori_DSP(0) << endl;
    cout << "Zeta_foot_ori_DSP : " << LIGHT_WholeMotions.Zeta_foot_ori_DSP(0) << endl << endl;

    cout << "[ SUPPORTCONTROL_RSSP ]" << endl;
    cout << "WeightCompensation_RSSP : " << LIGHT_WholeMotions.WeightCompensation_RSSP << endl;
    cout << "Freq_body_pos_RSSP : " << LIGHT_WholeMotions.Freq_body_pos_RSSP(0) << endl;
    cout << "Zeta_body_pos_RSSP : " << LIGHT_WholeMotions.Zeta_body_pos_RSSP(0) << endl;
    cout << "Freq_body_ori_RSSP : " << LIGHT_WholeMotions.Freq_body_ori_RSSP(0) << endl;
    cout << "Zeta_body_ori_RSSP : " << LIGHT_WholeMotions.Zeta_body_ori_RSSP(0) << endl;
    cout << "Freq_foot_pos_RSSP : " << LIGHT_WholeMotions.Freq_foot_pos_RSSP(0) << endl;
    cout << "Zeta_foot_pos_RSSP : " << LIGHT_WholeMotions.Zeta_foot_pos_RSSP(0) << endl;
    cout << "Freq_foot_ori_RSSP : " << LIGHT_WholeMotions.Freq_foot_ori_RSSP(0) << endl;
    cout << "Zeta_foot_ori_RSSP : " << LIGHT_WholeMotions.Zeta_foot_ori_RSSP(0) << endl << endl;

    cout << "[ SUPPORTCONTROL_LSSP ]" << endl;
    cout << "WeightCompensation_LSSP : " << LIGHT_WholeMotions.WeightCompensation_LSSP << endl;
    cout << "Freq_body_pos_LSSP : " << LIGHT_WholeMotions.Freq_body_pos_LSSP(0) << endl;
    cout << "Zeta_body_pos_LSSP : " << LIGHT_WholeMotions.Zeta_body_pos_LSSP(0) << endl;
    cout << "Freq_body_ori_LSSP : " << LIGHT_WholeMotions.Freq_body_ori_LSSP(0) << endl;
    cout << "Zeta_body_ori_LSSP : " << LIGHT_WholeMotions.Zeta_body_ori_LSSP(0) << endl;
    cout << "Freq_foot_pos_LSSP : " << LIGHT_WholeMotions.Freq_foot_pos_LSSP(0) << endl;
    cout << "Zeta_foot_pos_LSSP : " << LIGHT_WholeMotions.Zeta_foot_pos_LSSP(0) << endl;
    cout << "Freq_foot_ori_LSSP : " << LIGHT_WholeMotions.Freq_foot_ori_LSSP(0) << endl;
    cout << "Zeta_foot_ori_LSSP : " << LIGHT_WholeMotions.Zeta_foot_ori_LSSP(0) << endl << endl;

    cout << "[ SUPPORTCONTROL_FLOAT ]" << endl;
    cout << "Freq_foot_pos_Float : " << LIGHT_WholeMotions.Freq_foot_pos_Float(0) << endl;
    cout << "Zeta_foot_pos_Float : " << LIGHT_WholeMotions.Zeta_foot_pos_Float(0) << endl;
    cout << "Freq_foot_ori_Float : " << LIGHT_WholeMotions.Freq_foot_ori_Float(0) << endl;
    cout << "Zeta_foot_ori_Float : " << LIGHT_WholeMotions.Zeta_foot_ori_Float(0) << endl << endl;

    cout << "[ JOINGIMPEDANCE_RF (On Contact) ]" << endl;
    cout << "K_RF_ori_OnContact(0) : " << LIGHT.K_RF_ori_OnContact(0) << endl;
    cout << "K_RF_ori_OnContact(1) : " << LIGHT.K_RF_ori_OnContact(1) << endl;
    cout << "K_RF_ori_OnContact(2) : " << LIGHT.K_RF_ori_OnContact(2) << endl;
    cout << "K_RF_pos_OnContact(0) : " << LIGHT.K_RF_pos_OnContact(0) << endl;
    cout << "K_RF_pos_OnContact(1) : " << LIGHT.K_RF_pos_OnContact(1) << endl;
    cout << "K_RF_pos_OnContact(2) : " << LIGHT.K_RF_pos_OnContact(2) << endl;
    cout << "D_RF_ori_OnContact(0) : " << LIGHT.D_RF_ori_OnContact(0) << endl;
    cout << "D_RF_ori_OnContact(1) : " << LIGHT.D_RF_ori_OnContact(1) << endl;
    cout << "D_RF_ori_OnContact(2) : " << LIGHT.D_RF_ori_OnContact(2) << endl;
    cout << "D_RF_pos_OnContact(0) : " << LIGHT.D_RF_pos_OnContact(0) << endl;
    cout << "D_RF_pos_OnContact(1) : " << LIGHT.D_RF_pos_OnContact(1) << endl;
    cout << "D_RF_pos_OnContact(2) : " << LIGHT.D_RF_pos_OnContact(2) << endl << endl;

    cout << "[ JOINGIMPEDANCE_RF (Off Contact) ]" << endl;
    cout << "K_RF_ori_OnContact(0) : " << LIGHT.K_RF_ori_OffContact(0) << endl;
    cout << "K_RF_ori_OnContact(1) : " << LIGHT.K_RF_ori_OffContact(1) << endl;
    cout << "K_RF_ori_OnContact(2) : " << LIGHT.K_RF_ori_OffContact(2) << endl;
    cout << "K_RF_pos_OnContact(0) : " << LIGHT.K_RF_pos_OffContact(0) << endl;
    cout << "K_RF_pos_OnContact(1) : " << LIGHT.K_RF_pos_OffContact(1) << endl;
    cout << "K_RF_pos_OnContact(2) : " << LIGHT.K_RF_pos_OffContact(2) << endl;
    cout << "D_RF_ori_OnContact(0) : " << LIGHT.D_RF_ori_OffContact(0) << endl;
    cout << "D_RF_ori_OnContact(1) : " << LIGHT.D_RF_ori_OffContact(1) << endl;
    cout << "D_RF_ori_OnContact(2) : " << LIGHT.D_RF_ori_OffContact(2) << endl;
    cout << "D_RF_pos_OnContact(0) : " << LIGHT.D_RF_pos_OffContact(0) << endl;
    cout << "D_RF_pos_OnContact(1) : " << LIGHT.D_RF_pos_OffContact(1) << endl;
    cout << "D_RF_pos_OnContact(2) : " << LIGHT.D_RF_pos_OffContact(2) << endl << endl;

    cout << "[ JOINGIMPEDANCE_LF (On Contact) ]" << endl;
    cout << "K_LF_ori_OnContact(0) : " << LIGHT.K_LF_ori_OnContact(0) << endl;
    cout << "K_LF_ori_OnContact(1) : " << LIGHT.K_LF_ori_OnContact(1) << endl;
    cout << "K_LF_ori_OnContact(2) : " << LIGHT.K_LF_ori_OnContact(2) << endl;
    cout << "K_LF_pos_OnContact(0) : " << LIGHT.K_LF_pos_OnContact(0) << endl;
    cout << "K_LF_pos_OnContact(1) : " << LIGHT.K_LF_pos_OnContact(1) << endl;
    cout << "K_LF_pos_OnContact(2) : " << LIGHT.K_LF_pos_OnContact(2) << endl;
    cout << "D_LF_ori_OnContact(0) : " << LIGHT.D_LF_ori_OnContact(0) << endl;
    cout << "D_LF_ori_OnContact(1) : " << LIGHT.D_LF_ori_OnContact(1) << endl;
    cout << "D_LF_ori_OnContact(2) : " << LIGHT.D_LF_ori_OnContact(2) << endl;
    cout << "D_LF_pos_OnContact(0) : " << LIGHT.D_LF_pos_OnContact(0) << endl;
    cout << "D_LF_pos_OnContact(1) : " << LIGHT.D_LF_pos_OnContact(1) << endl;
    cout << "D_LF_pos_OnContact(2) : " << LIGHT.D_LF_pos_OnContact(2) << endl << endl;

    cout << "[ JOINGIMPEDANCE_LF (Off Contact) ]" << endl;
    cout << "K_LF_ori_OnContact(0) : " << LIGHT.K_LF_ori_OffContact(0) << endl;
    cout << "K_LF_ori_OnContact(1) : " << LIGHT.K_LF_ori_OffContact(1) << endl;
    cout << "K_LF_ori_OnContact(2) : " << LIGHT.K_LF_ori_OffContact(2) << endl;
    cout << "K_LF_pos_OnContact(0) : " << LIGHT.K_LF_pos_OffContact(0) << endl;
    cout << "K_LF_pos_OnContact(1) : " << LIGHT.K_LF_pos_OffContact(1) << endl;
    cout << "K_LF_pos_OnContact(2) : " << LIGHT.K_LF_pos_OffContact(2) << endl;
    cout << "D_LF_ori_OnContact(0) : " << LIGHT.D_LF_ori_OffContact(0) << endl;
    cout << "D_LF_ori_OnContact(1) : " << LIGHT.D_LF_ori_OffContact(1) << endl;
    cout << "D_LF_ori_OnContact(2) : " << LIGHT.D_LF_ori_OffContact(2) << endl;
    cout << "D_LF_pos_OnContact(0) : " << LIGHT.D_LF_pos_OffContact(0) << endl;
    cout << "D_LF_pos_OnContact(1) : " << LIGHT.D_LF_pos_OffContact(1) << endl;
    cout << "D_LF_pos_OnContact(2) : " << LIGHT.D_LF_pos_OffContact(2) << endl << endl;

    cout << "[ CoMLEADCOMPENSATE ]" << endl;
    cout << "CoM_wn_X : " << LIGHT_WholeMotions.CoM_wn_X << endl;
    cout << "CoM_wn_Y : " << LIGHT_WholeMotions.CoM_wn_Y << endl;
    cout << "CoM_zeta_X : " << LIGHT_WholeMotions.CoM_zeta_X << endl;
    cout << "CoM_zeta_Y : " << LIGHT_WholeMotions.CoM_zeta_Y << endl << endl;
}
