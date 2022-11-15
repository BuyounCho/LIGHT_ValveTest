#ifndef SH_TASK_BASICFUNCTION
#define SH_TASK_BASICFUNCTION

#include "BasicFiles/BasicJoint.h"

#include <unistd.h>
#include <cstdlib>

#include <iomanip>
#include <iostream>
#include <chrono>
using namespace std;
using namespace std::chrono;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;


void    PrintHere(int n);
void    fifth_trajectory_oneaxis(double tf, double tn,
                                 double p, double v, double a,
                                 double pf, double vf, double af,
                                 double &pn, double &vn, double &an);

// Save Function ------
#define SAVEKIND            130
#define SAVEKIND_FORID      10
#define SAVENUM             100000

extern bool            save_PutDataFlag;
extern bool            save_ActivateFlag;
extern unsigned int    save_Index;
extern float           save_Buf[SAVEKIND][SAVENUM];
extern bool            save_Flag_forID;
extern unsigned int    save_Index_forID;
extern float           save_Buf_forID[SAVEKIND_FORID][SAVENUM];
extern int             ID_BNO;

void            save_File(char* filecomment = "");
void            save_PutData(unsigned int cur_Index);
void            save_File_forID(string Date, int _Ps, int _OPEN, int direction);
void            save_PutData_forID(unsigned int cur_Index, int BNO);

#endif // SH_TASK_BASICFUNCTION
