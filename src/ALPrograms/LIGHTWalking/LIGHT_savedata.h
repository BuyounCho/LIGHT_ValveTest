#ifndef LIGHT_SAVEDATA
#define LIGHT_SAVEDATA

#include <iostream>
#include <chrono>
#include <stdlib.h>
#include <string>

// Save Function and Parameter
#define SAVEKIND    280
#define SAVENUM     60000 // 250Hz >> 240sec

#define SAVEKIND_SupPres    5
#define SAVENUM_SupPres     1000000 // 250Hz >> 240sec

//#define SAVEKIND_SYS_ID    6
//#define SAVENUM_SYS_ID     200000

extern int     PODO_NO;
extern int     CalcTime_WALKING;

extern bool             save_PutDataFlag;
extern bool             save_ActivateFlag;
extern unsigned int     save_Index;
extern float            save_Buf[SAVEKIND][SAVENUM];

void                    save_File(char* filecomment = "");
void                    save_PutData(unsigned int cur_Index);

extern bool             save_PutDataFlag_SupPres;
extern bool             save_ActivateFlag_SupPres;
extern unsigned int     save_Index_SupPres;
extern float            save_Buf_SupPres[SAVEKIND_SupPres][SAVENUM_SupPres];

void                    save_File_SupPres();
void                    save_PutData_SupPres(unsigned int cur_Index);

void            save_File_TuningParameters();
void            Show_TuningParameters();

#endif // LIGHT_SAVEDATA
