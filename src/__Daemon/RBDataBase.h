#ifndef RBDATABASE_H
#define RBDATABASE_H

#include "RBLog.h"
#include "RBDataType.h"

//For Database File
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>

class RBDataBase
{
public:
    RBDataBase();

    void    SetFilename(QString name);
    bool    OpenDB();
    bool    UpdateDB_CAN_Channel_VC(int BOARD_ID, int CAN_CH); // Motion Controller Update
    bool    UpdateDB_CAN_Channel_PC(int BOARD_ID, int CAN_CH); // Pump Controller Update
    bool    UpdateDB_CAN_Channel_IMU(int BOARD_ID, int CAN_CH); // IMU Controller Update
    bool    UpdateDB_CAN_Channel_FT(int BOARD_ID, int CAN_CH); // FT Controller Update

    static DB_GENERAL      _DB_GENERAL;
    static DB_VC           _DB_VC[MAX_VC];
    static DB_PC           _DB_PC[MAX_PC]; // Pump Controller (Buyoun, 20190710)
    static DB_FT           _DB_FT[MAX_FT];
    static DB_IMU          _DB_IMU[MAX_IMU];
//    static DB_SP           _DB_SP[MAX_SP];
//    static DB_OF           _DB_OF[MAX_OF];
    static DB_AL           _DB_AL[MAX_AL];


private:
    QString         filename;
    QSqlDatabase    dbCore;
};

#endif // RBDATABASE_H
