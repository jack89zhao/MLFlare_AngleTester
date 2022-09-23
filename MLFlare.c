//
//  MLFlare.c
//  MLFlare
//
//  Created by Jackie Wang on 2019/7/2.
//  Copyright © 2019 Jackie Wang. All rights reserved.
//
//

//#define NDEBUG

#include "MLFlare.h"
#include "LTSMC.h"
#include "MLConfiger.h"
#include "serial.h"
#include "byte_fifo.h"
#include "DLRS1A.h"
#include "MLExpection.h"
#include "JKCL200A.h"
#include "SCA126T.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <math.h>
#include <sys/timeb.h>
#include <stdio.h>
#include <time.h>
#include <dirent.h>     // DIR
// Motion API Handle
typedef int MLMotionHandle;

typedef struct {
    void *arg1;
    double arg2;
    bool arg3;
    void *arg4;
} TArgs;

const int RegisterDataSize                  = 4096; // 寄存器存储量（字节数）
const int ControllerInitializedFlagByte     = 0;    // 标示控制器中是否已设置校准参数（字节索引，0～4095）
const int AxesNumberIndex                   = 1;    // 存储轴数的寄存器地址（字节索引）
const int AxisParamStartIndex               = 1;    // 轴参数起始索引（浮点数索引， 0～1023）
const int AxisParamLengthInRegister         = 16;   // 浮点数个数， 16*4 byte，每个轴可保存16个参数 ( 16*12 = 192）
typedef enum {
    AP_StartSpeed       = 0,
    AP_RunSpeed         = 1,
    AP_StopSpeed        = 2,
    AP_HomeSpeed        = 3,
    AP_AccTime          = 4,
    AP_DecTime          = 5,
    AP_HomeDirection    = 6,
    AP_HomeLevel        = 7,
    AP_PPRatio          = 8,
    AP_InitPos          = 9,
    AP_TestPos          = 10,
    AP_Reserve1         = 11,
    AP_Reserve2         = 12,
    AP_Reserve3         = 13,
    AP_Equiv            = 14,
    AP_Backlash         = 15,
}AxisParamIndex;
const int BaseCalibrationDataStartIndex     = 500;      // 异常校准参考数据起始存储索引（浮点数索引，Axis1~Axis12 <==> 500～511）
const int DisabledAxisStartIndex            = 4000;     // 轴禁用标示起始索引（字节索引）

// gloable parameter
static const int gAxisNum = 10;    // 当前轴数量
static int gAxisAvailableStates[gAxisNum+1];      // 轴启用状态集（算上0轴）

void InitializeCalibrationData(void);

#define FIFO_DEBUG(...)

#define MLAbsolute  1       // 绝对运动模式
#define MLRelative  0       // 相对运动模式

//#undef  _ML_ASSERT_TEST_      //禁用
#define _ML_ASSERT_TEST_        //启用

#ifdef _ML_ASSERT_TEST_         //启用断言测试
void assert_report( const char * file_name, const char * function_name, unsigned int line_no ) {
    printf( "\n[MLFlare]Error Report file_name: %s, function_name: %s, line %u\n", file_name, function_name, line_no );
}
#define ASSERT_REPORT( condition )       \
do {       \
    if ( condition )       \
        NULL;        \
    else         \
        assert_report( __FILE__, __func__, __LINE__ ); \
} while(0)
#else // 禁用断言测试
#define ASSERT_REPORT( condition )  NULL
#endif /* end of ASSERT */

typedef struct {
    float lv;      // lux
    float x;       // color.x
    float y;       // color.y
    float u;
    float v;
    float X;       // RGB Color X
    float Y;
    float Z;
} MLLuxMeterVal;

typedef enum {
    LuxMeter_Moving,
    LuxMeter_Home,
    LuxMeter_Shop
}LuxMeterState;

typedef enum {
    Door_Opened,
    Door_Closed,
    Door_Moving
}DoorState;

pthread_mutex_t _mutex;

// global Variables
static MLMotionHandle gHandle       = -1;
static AxisParam gAxisPrm[MLMaxAxisCount];
static MLDUT *duts;
static int dutCount;

static bool gStopped;
static bool gIsTesting;
static bool gIsRaster;
static bool gLuxConnected;

static DoorState        doorState;
static LuxMeterState    luxmeterState;

static bool gIsPowerOn;

static bool gIsOpenDoor;
static bool gIsCloseDoor;
static bool gIsStartTest;
static bool gIsEmgStopTest;
static bool gIsStopTest;
static pthread_t checkSensorThread;

string anglePortName; // Illuminometer
string illuminometerPortName; // Illuminometer
string laserPortName;       // laser
MLDUT dutPrm;

string errmsg;

static bool bCalibrated;
static MLLuxMeterVal *calLuxVals;

static bool isNeedStop = false;
static bool isNeedEmgStop = false;
static bool isActiveFinished = false;

static double startSpeedDef[] = {100, 10, 2, 3, 5, 5,  2, 5,  5,  1, 40,  1,  1,  100,  100,  100};
static double runSpeedDef[] =   {100, 20, 3, 5, 5, 15, 8, 15, 15, 8, 120, 10, 10, 1000, 1000, 1000};
static double stopSpeedDef[] =  {100, 10, 2, 3, 5, 5,  2, 5,  5,  1, 40,  1,  1,  100,  100,  100};
static double homeSpeedDef[] =  {100, 20, 3, 5, 5, 15, 8, 15, 15, 5, 120, 10, 10, 1000, 1000, 1000};
static double accTImeDef[] =    {0.2, 1,  1, 1, 1, 1,  1, 1,  1,  1, 3,   1,  1,  0.1,  0.1,  0.1};
static double ppRatioDef[] =    {0,  100, 2500, 2500, 15000, 1000, 1000, 1000, 1000, 40000, 800, 2000, 2000, 100, 100, 100};
static int homeModes[] = {27, 27, 27, 27, 27, 27, 30, 30, 30, 30};
static int homeDirDef = 1;
static int homeLevelDef = 0;
static bool hasInitialzed = false;

static char* axesNames[11] = {
    "",
    "AxisRotation", "AxisXSwing", "AxisYSwing", "AxisLifter",
    "AxisDUTX", "AxisDUTY", "AxisLightSource", "AxisLaser",
    "AxisBaseX", "AxisBaseY"
};
static char* outBitDescs[32] = {
    "ServoBrake", "", "", "", "VacuumSuction", "LightSourcePower", "Axis1Enable", "Axis2Enable",
    "Axis3Enable", "Axis4Enable", "Axis5Enable", "Axis6Enable", "Axis7Enable", "Axis8Enable", "SpotPower", "CylinderOrigin",
    "CylinderTest", "DoorOpen", "DoorClose", "", "DUTPower", "SunLight", "RedButtonLed", "GreenButtonLed",
    "BlueButtonLed", "LaserPower", "LuxmeterPower", "", "", "", "", ""
};

void MoveToInitPos(int *axises, int count);
void ResetAllAxis(bool pthread);

// ========================= private method ============================

char* GetLibraryVersion(void) {
    return "v2.0.1";
}

static char** ListUSBDeviceNames(int *count)
{
    FILE        *fd = NULL;
    char        line[64];
    int         index = 0;
    char        **dev_names;
    
    dev_names = (char **)malloc(16 * sizeof(char*));
    fd = popen("ls /dev", "r");         // get port device name by pipe.
    
    if (NULL != fd) {
        while (fgets(line, 64, fd) != NULL) {
            char *ptr = strstr(line, "cu.usb");
            
            if (ptr != NULL) {
                size_t n = strlen(line) + strlen("/dev/");
                char *item = (char *)malloc((n + 1) * sizeof(char));
                snprintf(item, n, "/dev/%s", line);
                dev_names[index++] = item;
            }
        }
        pclose(fd);
        fd = NULL;
    }
    *count = index--;
    
    return dev_names;
}

char* CreateLogPath(char *path)  {
    char *flareLogPath;
    char cmd[128];
    char *home = getenv("HOME");
    
    flareLogPath = (char*)malloc(strlen(home) + strlen(path) + 1);
    sprintf(flareLogPath, "%s%s", home, path);
    
    int error = access(flareLogPath, F_OK);
    if (error != 0) {
    // create log path, create intermediate directories as required.
        strcpy(cmd, "mkdir -p ");
        strcat(cmd, flareLogPath);
        system(cmd);
    }
    
    return flareLogPath;
}

void Logger(MLLogLevel type, char* message, ...) {
    //create log path
    char* path = CreateLogPath("/Documents/Flare/Logs/");
    
    char logString[1024];
    memset(logString, 0, 1024);
    va_list args;
    va_start(args, message);
    vsnprintf(logString, 1024, message, args);
    va_end(args);
    
    // current time.
    char tips[256];
    char folder[256];
    memset(tips, 0, 256);
    memset(folder, 0, 256);
    
    struct tm *tmPtr = NULL;
    struct timeb stTimeb;
    
    ftime(&stTimeb);
    tmPtr  = localtime(&stTimeb.time);
    sprintf(tips, "[%04d-%02d-%02d %02d:%02d:%02d:%03d] ", (1900+tmPtr->tm_year), (1+tmPtr->tm_mon), tmPtr->tm_mday, tmPtr->tm_hour, tmPtr->tm_min, tmPtr->tm_sec, stTimeb.millitm);
    sprintf(folder, "%s/%04d_%02d_%02d/", path, (1900+tmPtr->tm_year), (1+tmPtr->tm_mon), tmPtr->tm_mday);
    
    char result[1024+32];
    memset(result, 0, 1024+32);
    // write logString to file.
    switch (type) {
        case MLLogInfo: {
            strcat(tips, "[Info]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        case MLLogWarning: {
            strcat(tips, "[Warning]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        case MLLogError: {
            strcat(tips, "[Error]: ");
            strcat(result, tips);
            strcat(result, logString);
            break;
        }
        default:
            break;
    }
    
    FILE *logFile = NULL;
    FILE *errorLogFile = NULL;
    
    char *errFile = malloc(64);
    strcpy(errFile, folder);
    strcat(errFile, "error.log");
    
    // create folder.
    if (access(folder, 0)) {
        char *cmd = (char*)malloc(256);
        strcpy(cmd, "mkdir -p ");
        strcat(cmd, folder);
        system(cmd);
        free(cmd);
        cmd = NULL;
    }
    
    strcat(folder, "process.log");
    
    logFile = fopen(folder, "a+");
    
    if (logFile) {
        fputs(result, logFile);
        fclose(logFile);
    }
    
    // append error message.
    if (type == MLLogError) {
        errorLogFile = fopen(errFile, "a+");
        if (errorLogFile) {
            fputs(result, errorLogFile);
            fclose(errorLogFile);
        }
    }
    
    // destory malloc buffer
    free(errFile);
    free(path);
    errFile = NULL;
    path = NULL;
}

static bool ContainString(const char *array[], int array_size, const char *target_str)
{
    bool success = false;
    
    for (int i = 0; i < array_size; i++) {
        char *ptr = (char*)array[i];
        
        if (!strcmp(ptr, target_str)) {
            success = true;
            break;
        }
    }
    
    return success;
}

static int CheckDeviceName(const char *device_array[], int array_size, const char *device_name)
{
    int error_code = 0;
    bool existed = true;
    
    if (NULL == device_name) {
        error_code = 1;
        return error_code;
    }
    
    existed = ContainString(device_array, array_size, device_name);
    
    if (!existed) {
        error_code = 1;
        Logger(MLLogError, "<%s>: Undetect device's name named '%s' on the computer.\n", __func__, device_name);
    }
    
    return error_code;
}

void LoadDefaultProfile() {
    SetIniFileName(MLConfigFileName);
//    double startSpeed = GetDoubleValue("axis_0", "start_speed");
}

int GetCountOfPtr(MLDUT *buffer) {
    int currentDUTTypeCnt = 0;
    MLDUT *p;
    p = buffer;
    
    while (p->name != NULL) {
        currentDUTTypeCnt++;
        p++;
    }
    
    return currentDUTTypeCnt;
}

bool ReleaseSysResource() {
    if (!hasInitialzed) {
        return false;
    }
    
    bool flag = true;
    char filename[128];
    char *homepath = getenv("HOME");
    sprintf(filename, "%s/Documents/Flare/profile.ini", homepath);
    CreateLogPath("/Documents/Flare/");
    SetIniFileName(filename);
    
//    int count = GetCountOfPtr(duts);
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        AxisParam param = gAxisPrm[axis];
        
        char *title = (char *)malloc(32);
        sprintf(title, "axis_%d", axis);
        if (!PutDoubleValue(title, "start_speed", param.startSpeed)) { Logger(MLLogError, "%s when save start speed at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "run_speed",   param.runSpeed)) { Logger(MLLogError, "%s when save run speed at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "stop_speed",  param.stopSpeed)) { Logger(MLLogError, "%s when save stop speed at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "home_speed",  param.homeSpeed)) { Logger(MLLogError, "%s when save home speed at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "acc_time",    param.accTime)) { Logger(MLLogError, "%s when save acc time at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutIntValue(title,    "home_dir",    param.homeDirect)) { Logger(MLLogError, "%s when save home direction at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutIntValue(title,    "home_level",  param.homeLevel)) { Logger(MLLogError, "%s when save home leveel at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "pp_ratio",    param.ppratio)) { Logger(MLLogError, "%s when save pp_ratio at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "initialize_positon", param.posToInit)) { Logger(MLLogError, "%s when save initialize position at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        if (!PutDoubleValue(title, "test_position", param.posToTest)) { Logger(MLLogError, "%s when save test position at axis %d\n", GetConfigFileErrorMessage(), param.axis); flag = false; }
        
        free(title);
        title = NULL;
    }
    ConfigLaserPortName(laserPortName);
    ConfigIlluminometerPortName(illuminometerPortName);
    ConfigAnglePortName(anglePortName);
    return flag;
}

void SetEncoderUnit() {
    short ret = 0;
//    short res = 0;
    
    WORD MyCardNo = 0;
    WORD Mymode = 3;
    
    for (int i = 1; i <= gAxisNum; i++) {
        ret=smc_set_counter_inmode(MyCardNo,i,Mymode);//设置编码器计数方式
//        res=smc_set_encoder_unit(MyCardNo,i,100); //设置编码值为 100
    }
}

void Encoder() {
    /*********************变量定义****************************/
    short ret = 0;
    short res = 0;
    //错误返回
    WORD MyCardNo = 0;
    WORD Myaxis = 9;
    WORD Mymode = 3;
    double Myencoder_value=0;
    /*********************函数调用执行**************************/ //第一步、设置 0 号轴的编码器计数方式
    ret=smc_set_counter_inmode(MyCardNo,Myaxis,Mymode); //第二步、设置 0 轴的编码值为 100
    res=smc_set_encoder_unit(MyCardNo,Myaxis,100); //第三步、读轴 0 轴的编码值
    res=smc_get_encoder_unit(MyCardNo,Myaxis,&Myencoder_value);
}

double retain3decimals(double value) {
    return (long)((value+0.0005) * 1000) / 1000.0;
}

char* GetProjectName(){
    
    char dirname[128];
    char *filename[1024];
    char *homepath = getenv("HOME");
    sprintf(dirname, "%s/Documents/MATLAB/project", homepath);
    
    int i = 0;
    DIR *dirp;
    struct dirent *dp;
    dirp = opendir(dirname); //打开目录指针
    while ((dp = readdir(dirp)) != NULL) { //通过目录指针读目录
        printf("%s\n", dp->d_name );
        i++;
        filename[i] = dp->d_name;
    }
    (void) closedir(dirp);
    
    return filename;
}

bool NearlyEqual(double value1, double value2)
{
    return fabs(value1 - value2) < 0.5;
}

void CheckAllAxisState() {
    int state[12];
    for(int axis=1; axis<=gAxisNum; axis++) {
        state[axis-1] = smc_check_done(gHandle, axis);
    }
    Logger(MLLogInfo, "axes state: 1:%d, 2:%d, 3:%d, 4:%d, 5:%d, 6:%d, 7:%d, 8:%d, 9:%d, 10:%d\n", state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9]);
}

bool GetAxisEnableState(int axis) {
    short state = smc_read_sevon_pin(gHandle, axis);
    return state==0;
}

void SetAxisEnableState(int axis, bool enable) {
    short ret = smc_write_sevon_pin(gHandle, axis, enable?0:1);
    bool checkflag = GetAxisEnableState(axis);
    if(enable==checkflag) {
        
    }
}

void GetAxisErrCode(int axis, DWORD *errCode) {
    short rc = nmcs_get_node_od(gHandle, 2, 1002+axis, 0x603F, 00, 16, errCode);
//    short rc = nmcs_get_axis_errcode(gHandle, axis, errCode);
    if(rc!=0) {
        Logger(MLLogWarning, "<GetAxisErrCode>: fail to get axis<%d>(nodeId:%d) error code. (ret=%d)\n", axis, 1002+axis, rc);
    }
}

// mode: 0->normal parameter   1->home parameter
bool CheckAxisParameters(MLAxis axis, int mode)
{
    short rtn = 0;
    double checkStartSpeed = 0;
    double checkTargetSpeed = 0;
    double checkStopSpeed = 0;
    double checkAccTime = 0;
    double checkDecTime = 0;
    int retry = 0;

    AxisParam param = gAxisPrm[axis];
    double targetSpeed = mode==0 ? param.runSpeed : param.homeSpeed;

    Logger(MLLogInfo, "<%s>: Check axis {%d} parameters.\n", __func__, axis);

    if(mode==0) {
        rtn = smc_get_profile_unit(gHandle, axis, &checkStartSpeed, &checkTargetSpeed, &checkAccTime, &checkDecTime, &checkStopSpeed);
    } else {
        rtn = smc_get_home_profile_unit(gHandle, axis, &checkStartSpeed, &checkTargetSpeed, &checkAccTime, &checkDecTime);
    }

    if(rtn==0) {
        do {
            if (!NearlyEqual(checkStartSpeed, param.startSpeed*param.ppratio)
            || !NearlyEqual(checkTargetSpeed, targetSpeed*param.ppratio)
            || !NearlyEqual(checkAccTime, param.accTime)
            || !NearlyEqual(checkStopSpeed, param.stopSpeed*param.ppratio)) {
                if(mode==0) {
                    rtn |= smc_set_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, targetSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed*param.ppratio);
                } else {
                    rtn |= smc_set_home_profile_unit(gHandle, axis, param.startSpeed*param.ppratio, targetSpeed*param.ppratio, param.accTime, param.accTime);
                }
                rtn |= smc_set_s_profile(gHandle, axis,0,0.1);
                rtn |= smc_get_profile_unit(gHandle, axis, &checkStartSpeed, &checkTargetSpeed, &checkAccTime, &checkDecTime, &checkStopSpeed);
            } else {
                break;
            }
        } while (++retry < 3 && rtn !=0);
    }

    if (retry >= 3 || rtn != 0) {
        param.isRuning = false;
        isNeedStop = true;
        Logger(MLLogError, "<%s>: unexpected axis {%d} parameters { minSpeed:%.2lf(ex:%.2lf), maxSpeed:%.2lf(ex:%.2lf), stopSpeed:%.2lf(ex:%.2lf) accTime:%.2lf(ex:%.2lf).\n", __func__, axis, checkStartSpeed/param.ppratio, param.startSpeed, checkTargetSpeed/param.ppratio, targetSpeed, checkStopSpeed/param.ppratio, param.stopSpeed, checkAccTime, param.accTime);
        return false;
    }
    
    return true;
}

short GetByteRegisterValue(DWORD valueIndex, char *value) {
    short rc = smc_get_persistent_reg_byte(gHandle, valueIndex, 1, value);
    return rc;
}

short SetByteRegisterValue(DWORD valueIndex, char *value) {
    short rc = smc_set_persistent_reg_byte(gHandle, valueIndex, 1, value);
    if(rc==0) {
        if(valueIndex>=DisabledAxisStartIndex && valueIndex<=DisabledAxisStartIndex+gAxisNum) {
            gAxisAvailableStates[valueIndex-DisabledAxisStartIndex] = value==0 ? 1 : 0;
        }
        
    }
    return rc;
}

short GetCalibrationData(float *data) {
    short rc = 0;
    int axisParamSize = AxisParamLengthInRegister*12;
    rc = smc_get_persistent_reg_float(gHandle, AxisParamStartIndex, axisParamSize, data);
    return rc;
}

short SetCalibrationData(float *data) {
    short rc = 0;
    int axisParamSize = AxisParamLengthInRegister*12;
    rc = smc_set_persistent_reg_float(gHandle, AxisParamStartIndex, axisParamSize, data);
    return rc;
}

short GetFloatRegisterValue(DWORD valueIndex, float *value) {
    short rc = smc_get_persistent_reg_float(gHandle, valueIndex, 1, value);
    return rc;
}

short SetFloatRegisterValue(DWORD valueIndex, float value) {
    short rc = smc_set_persistent_reg_float(gHandle, valueIndex, 1, &value);
    return rc;
}

void SetSingleAxisParamInRegister(MLAxis axis, AxisParamIndex api, float value) {
    short rc = SetFloatRegisterValue(AxisParamStartIndex+(axis-MLAxisRotation)*AxisParamLengthInRegister+api, value);
    if(rc!=0) {
        Logger(MLLogError, "<SetSingleAxisParamInRegister>: fail to set parameter(%d) value(%d) for axis %d\n", api, value, axis);
    } else {
        InitializeCalibrationData();
    }
}

double GetSingleAxisParamInRegister(MLAxis axis, AxisParamIndex api) {
    float value;
    short rc = 0, counter=0;
    do {
        rc = GetFloatRegisterValue(AxisParamStartIndex+(axis-MLAxisRotation)*AxisParamLengthInRegister+api, &value);
    }while(rc!=0 && counter++<3);
    if(rc!=0) {
//        Logger(MLLogError, "<GetSingleAxisParamInRegister>:[ret-%d] fail to get parameter(%d) for axis %d\n", rc, api, axis);
    }
    return value;
}

void SetAxisParamInRegister(MLAxis axis, float *axisParams) {
    short rc = 0, counter=0;
    do {
        rc =smc_set_persistent_reg_float(gHandle, AxisParamStartIndex+(axis-MLAxisRotation)*AxisParamLengthInRegister, AxisParamLengthInRegister, axisParams);
    }while(rc!=0 && counter++<3);
    if(rc!=0) {
        Logger(MLLogError, "<SetAxisParamInRegister>:[ret-%d] fail to set parameters of axis %d\n", rc, axis);
    }
}

void GetAxisParamInRegister(MLAxis axis, float *axisParams) {
//    axisParams = malloc(sizeof(float)*AxisParamLengthInRegister);
    short rc = smc_get_persistent_reg_float(gHandle, AxisParamStartIndex+(axis-MLAxisRotation)*AxisParamLengthInRegister, AxisParamLengthInRegister, axisParams);
    if(rc!=0) {
        Logger(MLLogWarning, "<GetAxisParamInRegister>: fail to get parameters of axis %d. (ret: %d)\n", axis, rc);
    }
}

// laser DUT measure position.
double GetLaserDUTPos(void) {
    return GetSingleAxisParamInRegister(MLAxisLaser, AP_TestPos)/gAxisPrm[MLAxisLaser].ppratio;
}

// laser spot measure postion.
double GetLaserSpotPos(void) {
    return GetSingleAxisParamInRegister(MLAxisLaser, AP_Reserve1)/gAxisPrm[MLAxisLaser].ppratio;;
}

void GetCalibrationOffset(float *offset) {
    float baseData[gAxisNum];
    smc_get_persistent_reg_float(0, BaseCalibrationDataStartIndex, gAxisNum, baseData);
    double currentAxesPos[gAxisNum];
    for(int i=1; i<=gAxisNum; i++) {
        smc_get_encoder_unit(0, i, &currentAxesPos[i-1]);
        offset[i-1] = currentAxesPos[i-1]-baseData[i-1];
    }
}

void SetCalibrationBaseData(int mode) {
    if(mode==1) {
        float offset[gAxisNum];
        GetCalibrationOffset(offset);
        // change the calibration data of all projects using offset
    }
    
    double currentAxesPos[gAxisNum];
    for(int i=1; i<gAxisNum; i++) {
        smc_get_encoder_unit(0, i, &currentAxesPos[i-1]);
        int rc = smc_set_persistent_reg_float(0, BaseCalibrationDataStartIndex+i-1, 1, (float *)&currentAxesPos[i-1]);
        if(rc!=0) {
            Logger(MLLogError, "<SetCalibration>: fail to set Base Calibration data(%.0f) for axis %d\n", currentAxesPos[i-1], i);
        }
    }
}

void GetAxisState(int axis, WORD *stateMachine, WORD *runMode, double *commandPos, double *encoderPos, double *speed, WORD *ioState) {
    // 0-未启动， 1-启动禁止，2-准备启动，3-启动，4-操作使能，5-停止，6-错误触发，7错误
    nmcs_get_axis_state_machine(gHandle, axis, stateMachine);
    // 0-空闲, 1-Pmove, 2-Vmove, 3-Hmove, 4-Handwheel, 5-Ptt/Pts, 6-Pvt/Pvts, 7-Gear, 8-Cam, 9-Line, 10-Continue
    smc_get_axis_run_mode(gHandle, axis, runMode);
    smc_get_position_unit(gHandle, axis, commandPos);
    smc_get_encoder_unit(gHandle, axis, encoderPos);
    smc_read_current_speed_unit(gHandle, axis, speed);
    *ioState = smc_axis_io_status(gHandle, axis);
}

bool InitializeSystem() {
    // free malloc objects.
    if (duts != NULL) { free(duts); duts = NULL; }
    if (laserPortName != NULL) { free(laserPortName); laserPortName = NULL; }
    if (illuminometerPortName != NULL) { free(illuminometerPortName); illuminometerPortName = NULL; }
    if (anglePortName != NULL) { free(anglePortName); anglePortName = NULL; }

    Logger(MLLogInfo, "\n\n-------------------- load MT driver --------------------");
    Logger(MLLogInfo, "<%s>: Flare library version: {%s}.\n", __func__, GetLibraryVersion());
    
    bool flag = true;
    errmsg = (string)malloc(256);
    
    char filename[128];
    char *homepath = getenv("HOME");
    sprintf(filename, "%s/Documents/Flare/profile.ini", homepath);
    CreateLogPath("/Documents/Flare/");
    SetIniFileName(filename);
    
    int dutTypeCnt = GetIntValue("syscfg", "dut_type_count");
    if (dutTypeCnt <= 0) {
        PutIntValue("syscfg", "dut_type_count", 0);
        dutTypeCnt = 0;
    }
    
    dutCount = dutTypeCnt;
    duts = (MLDUT *)malloc(MLMaxDUTTypeCount * sizeof(MLDUT) + 1);
    for (int cnt = 0; cnt < dutTypeCnt; cnt++) {
        char *title = (char *)malloc(32);
        //        sprintf(title, "%s", "DUTNAME");
        sprintf(title, "duttype_%d", cnt);
        char *name = (char *)malloc(64);
        memset(name, 0, 64);
        dutPrm.type = GetIntValue(title, "type");   if (dutPrm.type == INT_MIN) { PutIntValue(title, "type", 0); }
        GetStringValue(title, "name", name);
        dutPrm.name = name;                         if (0 == strcmp(dutPrm.name, "")) { PutStringValue(title, "name", ""); }
        dutPrm.posHolderX = GetDoubleValue(title, "axisHolderX_position");     if (dutPrm.posHolderX == LONG_MIN) { PutLongValue(title, "axisHolderX_position", 0); }
        dutPrm.posHolderY = GetDoubleValue(title, "axisHolderY_position");       if (dutPrm.posHolderY == LONG_MIN) { PutLongValue(title, "axisHolderY_position", 0); }
        dutPrm.posLifter = GetDoubleValue(title, "lifter_position");          if (dutPrm.posLifter == LONG_MIN) { PutLongValue(title, "lifter_position", 0); }
        dutPrm.posAxisX = GetDoubleValue(title, "axisX_position");          if (dutPrm.posAxisX == LONG_MIN) { PutLongValue(title, "axisX_position", 0); }
        dutPrm.posAxisY = GetDoubleValue(title, "axisY_position");          if (dutPrm.posAxisY == LONG_MIN) { PutLongValue(title, "axisY_position", 0); }
        dutPrm.posLifter = GetDoubleValue(title, "lifter_position");          if (dutPrm.posLifter == LONG_MIN) { PutLongValue(title, "lifter_position", 0); }
        duts[cnt] = dutPrm;
    }
    
    for (int axis = 0; axis < MLMaxAxisCount; axis++) {
        char *title = (char *)malloc(32);
        sprintf(title, "axis_%d", axis);
        double startSpeed = GetDoubleValue(title, "start_speed");           if (LONG_MIN == startSpeed) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double runSpeed =   GetDoubleValue(title, "run_speed");             if (LONG_MIN == runSpeed) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double stopSpeed =  GetDoubleValue(title, "stop_speed");            if (LONG_MIN == stopSpeed) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double homeSpeed =  GetDoubleValue(title, "home_speed");            if (LONG_MIN == homeSpeed) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double accTime =    GetDoubleValue(title, "acc_time");              if (LONG_MIN == accTime) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        int homeDirect =    GetIntValue(title, "home_dir");                 if (INT_MIN == homeDirect) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false;}
        int homeLevel =     GetIntValue(title, "home_level");               if (INT_MIN == homeLevel) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double ppratio =    GetDoubleValue(title, "pp_ratio");              if (LONG_MIN == ppratio) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double posInit =    GetDoubleValue(title, "initialize_positon");    if (LONG_MIN == posInit) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        double posTest =    GetDoubleValue(title, "test_position");         if (LONG_MIN == posTest) { Logger(MLLogError, "%s\n", GetConfigFileErrorMessage()); flag = false; }
        
        AxisParam param;
        param.axis = axis;
        param.isRuning = false;
        param.startSpeed =  (LONG_MIN != startSpeed) ? startSpeed : startSpeedDef[axis];        if (LONG_MIN == startSpeed) { PutDoubleValue(title, "start_speed", startSpeedDef[axis]); }
        param.runSpeed =    (LONG_MIN != runSpeed) ? runSpeed : runSpeedDef[axis];              if (LONG_MIN == runSpeed) { PutDoubleValue(title, "run_speed", runSpeedDef[axis]); }
        param.stopSpeed =   (LONG_MIN != stopSpeed) ? stopSpeed : stopSpeedDef[axis];           if (LONG_MIN == stopSpeed) { PutDoubleValue(title, "stop_speed", stopSpeedDef[axis]); }
        param.homeSpeed =   (LONG_MIN != homeSpeed) ? homeSpeed : homeSpeedDef[axis];           if (LONG_MIN == homeSpeed) { PutDoubleValue(title, "home_speed", homeSpeedDef[axis]); }
        param.accTime =     (LONG_MIN != accTime) ? accTime : accTImeDef[axis];                 if (LONG_MIN == accTime) { PutDoubleValue(title, "acc_time", accTImeDef[axis]); }
        param.homeDirect =  (INT_MIN != homeDirect) ? homeDirect : homeDirDef;                  if (INT_MIN == homeDirect) { PutDoubleValue(title, "home_dir", homeDirDef); }
        param.homeLevel =   (MLLevel)((INT_MIN != homeLevel) ? homeLevel : homeLevelDef);       if (INT_MIN == homeLevel) { PutDoubleValue(title, "home_level", homeLevelDef); }
        param.equiv = 1;
        param.backlash = 0;
        param.ppratio =     (LONG_MIN != ppratio) ? ppratio : ppRatioDef[axis];                 if (ppratio < 0) { PutDoubleValue(title, "pp_ratio", ppRatioDef[axis]); }
        param.posToInit =   (LONG_MIN != posInit) ? posInit : 0;                                if (LONG_MIN == posInit) { PutDoubleValue(title, "initialize_positon", 0); }
        param.posToTest =   (LONG_MIN != posTest) ? posTest : 0;                                if (LONG_MIN == posTest) { PutDoubleValue(title, "test_position", 0); }
        gAxisPrm[axis] = param;
        
        free(title);
        title = NULL;
    }
    
    int count;
    const char **device_names = (const char**)ListUSBDeviceNames(&count);
    
    laserPortName = (string)malloc(128);
    if (!GetStringValue("laser", "portname", laserPortName)) {
        Logger(MLLogError, "No laser port has configed.\n");
        InsertStringValue("laser", NULL, "portname", "/dev/cu.usbserial-Laser");
        flag = false;
    } else {
        if (CheckDeviceName(device_names, count, laserPortName)) {
            Logger(MLLogError, "{laser}, Not find port name '%s' on computer.\n", laserPortName);
            flag = false;
        }
//        if (strlen(laserPortName) <= 4) { flag = false; }
    }
    
    illuminometerPortName = (string)malloc(128);
    if (!GetStringValue("illumionmeter", "portname", illuminometerPortName)) {
        Logger(MLLogError, "No illumionmeter port has configed.\n");
        InsertStringValue("illumionmeter", NULL, "portname", "/dev/cu.usbserial-Luxmeter");
        flag = false;
    } else {
        if (strlen(illuminometerPortName) <= 4) { flag = false; }
    }
    
    anglePortName = (string)malloc(128);
    if (!GetStringValue("angle", "portname", anglePortName)) {
        Logger(MLLogError, "No angle port has configed.\n");
        InsertStringValue("angle", NULL, "portname", "/dev/cu.usbserial-Angle");
        flag = false;
    } else {
        if (CheckDeviceName(device_names, count, anglePortName)) {
            Logger(MLLogError, "{angle}, Not find port name '%s' on computer.\n", anglePortName);
            flag = false;
        }
//        if (strlen(anglePortName) <= 4) { flag = false; }
    }
    hasInitialzed = true;
    return flag;
}

// 判断输入的字符串是否是正确的IP，但不能过滤掉192.168.1..33这种情况
bool IsValidIP(char *ip) {
    bool flag = false;
    
    char ipStr[20];
    memset(ipStr, 0, 20);
    strcpy(ipStr, ip);
    
    int count = 0;
    char *ptr = strtok(ipStr, ".");
    
    do {
        if (ptr) {
            int num = atoi(ptr);
            
            if (num <= 255 && num >= 0) { count++; }
            else { break; }     // 超出ip的数值的最大值
            
            do {
                ptr = strtok(NULL, ".");
                if (ptr) {
                    int num = atoi(ptr);
                    
                    if (num <= 255 && num >= 0) { count++; }
                    else { break; }     // 超出ip的数值的最大值
                }
            } while (ptr != NULL);
        }
    } while (0);
    
    if (count == 4) {
        flag = true;
    }
    
    return flag;
}

bool CheckLimitSensor(int axis)
{
    ASSERT_REPORT(axis >= 0);
    bool flag = true;
    
    if (gHandle != -1) {
        DWORD state = smc_axis_io_status(gHandle, axis);
        
        if ((state & 0x06) == 0x06) {
            flag = false;
        }
    }
    
    return flag;
}

int CheckAllLimitSensor() {
    int errorAxis = 1;
    
    for (int axis = 1; axis < 13; axis++) {
        if (!CheckLimitSensor(axis)) {
            errorAxis = -axis;
            break;
        }
    }
    Logger(MLLogInfo, "<%s>: Check limit sensor status.\n", __func__);
    
    return errorAxis;
}

/*
 *check the axis's specified IO
 *@return 0 -> Servo drive alarm, 1 -> positive limit, 2 -> negative limit, 3 -> EMG, 4 -> ORG
 */
int CheckAxisIOState(int axis) {
    ASSERT_REPORT(axis >= 0);
    
    if (gHandle != -1) {
        DWORD state = smc_axis_io_status(gHandle, axis);
//        DWORD state;
//        smc_axis_io_status_ex(gHandle, axis, &state);
        
        int index = 0;
        state <<= 1;
        
        do {
            state = state >> 1;
            int bit = state & 0x01;
            
            switch (index) {
                case 0:
                    if (bit==1) {
                        DWORD errcode = 0;
                        GetAxisErrCode(axis, &errcode);
                        Logger(MLLogError, "<CheckAxisIOState>: Axis %d Servo drive alarm(error code: 0x%lx).\n", axis, errcode);
                        memset(errmsg, 0, 256);
                        sprintf(errmsg, "Axis %d Servo drive alarm(error code: 0x%lx).\n", axis, errcode);
//                        nmcs_get_node_od(0, 2, axis+1, 0x603F, 0, 32, &errcode);  // 另一种获得error code的方式
                        return 0;
                    } break;
                case 1:
                    if (bit==1) {
                        return 1;
                    } break;
                case 2:
                    if (bit==1) {
                        return 2;
                    } break;
                case 3:
                    if (bit==1) {
                        return 3;
                    } break;
                case 4:
                    if (bit==1) {
                        return 4;
                    } break;
                default: break;
            }
            index++;
        } while (state);
    }
    
    return 10;
}

AxisParam GetAxisParam(int axis) {
    assert(gAxisPrm != NULL);
    assert(axis >= 0);
    AxisParam param;
    pthread_mutex_lock(&_mutex);
    param = gAxisPrm[axis];
    pthread_mutex_unlock(&_mutex);
    
    return param;
}

bool SetAxisParam(int axis, AxisParam param) {
    bool flag = true;
    
    assert(gAxisPrm != NULL);
    assert(axis >= 0);
    gAxisPrm[axis] = param;
    
    return flag;
}

void PowerOn() {
    int errorAxis = CheckAllLimitSensor();
    if (errorAxis < 0) {
        Logger(MLLogError, "<%s>: Limit sensor has broken at axis %d\n", __func__, abs(errorAxis));
    }
    
    Logger(MLLogInfo, "<%s>: Power ON system.\n", __func__);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    SetBitState(MLOutLightStop, MLLow);
    SetBitState(MLOutLightStart, MLLow);
    SetBitState(MLOutLightReset, MLLow);
//    SetBitState(MLOutLeftDoorOpen, MLLow);
    DoorOpen();
    gIsPowerOn = true;
    AllAxisGoHome(true);
    Logger(MLLogInfo, "<%s>: Power ON..., reset all axises.\n", __func__);
}

void PowerOff() {
    Logger(MLLogInfo, "<%s>: Power OFF system.\n", __func__);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    SetBitState(MLOutLightSourcePower, MLHigh);
    SetBitState(MLOutSpotPower, MLHigh);
    SetBitState(MLOutDoorOpen, MLHigh);
    SetBitState(MLOutSunLight, MLHigh);
    SetBitState(MLOutLightStop, MLHigh);
    SetBitState(MLOutLightStart, MLHigh);
    SetBitState(MLOutLightReset, MLHigh);
    SetBitState(MLOutLaserPower, MLHigh);
    SetBitState(MLOutLuxmetePower, MLHigh);
    ResetAllAxis(true);
    gIsPowerOn = false;
}

// 初始化寄存器数据
void InitRegister() {
    // 将默认轴参数写入寄存器
    int size = 1024;
    char *zeroData = malloc(sizeof(char)*size);
    memset(zeroData, 0, size);
    for(int i=0; i<4; i++) {
        smc_set_persistent_reg_byte(0, i*size, size, zeroData);// 清零
    }
    float  defaultAxisParams[][16] = {
        // start_speed, run_speed, stop_speed, home_speed, acc_time, dec_time, home_dir, home_level, pp_ratio, init_pos, test_pos, reserve1, reserve2, reserve3, equiv, backlash
        {   10, 20, 10, 20,     1,  1,  1, 0,   100,    -110,  764,     0, 0, 0,    1, 1 },     // axis 1（Rotation Axis）
        {   2,  3,  2,  3,      1,  1,  1, 0,   2500,   -2500, 0,   0, 0, 0,      1, 1},     // axis 2 (X Rotation)
        {   3,  5,  3,  5,      1,  1,  1, 0,   2500,   0, 0,   0, 0, 0,          1, 1},     // axis 3 (Y Rotation)
        {   5,  5,  5,  5,      1,  1,  1, 0,   15000,  0, 0,   0, 0, 0,          1, 1},     // axis 4 (Lifter)
        {   2, 3,  2,  3,       1,  1,  1, 0,   10000,   0, 36.5,    0, 0, 0,       1, 1},     // axis 5 (DUT X)
        {   2, 3,  2,  3,       1,  1,  1, 0,   10000,   0, 40,  0, 0, 0,         1, 1},     // axis 6 (DUT Y)
        {   1, 8,   1,  5,      1,  1,  1, 0,   40000,  0, 0,   0, 0, 0,           1, 1},     // axis 7 (Light Source)
        {   40, 120, 40, 120,   3,  3,  1, 0,   800,    -12.5, 726,     680, 0, 0,  1, 1},     // axis 8 (Laser)
        {   1, 10, 1, 10,       1,  1,  1, 0,   2000,   16000, 5,   0, 0, 0,      1, 1},     // axis 9 (X)
        {   1, 10, 1, 10 ,      1,  1,  1, 0,   2000,   -10000, -8,     0, 0, 0,    1, 1}      // axis 10 (Y)
    };
    short rc = smc_set_persistent_reg_float(0, AxisParamStartIndex, AxisParamLengthInRegister*10, (float *)defaultAxisParams);
    if(rc!=0) {
        Logger(MLLogError, "<InitRegister>: write initial axes's parameter to registe failed.");
    }
    
    for(int i=0; i<=gAxisNum; i++) {
        char state = 0;  // 默认所有轴启用
        short rc = smc_set_persistent_reg_byte(gHandle, DisabledAxisStartIndex+i, 1, &state);
        if(rc!=0) {
            Logger(MLLogError, "<InitRegister>: initialize disabled axes table(axis: %@).", i);
        }
    }
    
    // set initialized flag
    char initialized = '1';
    smc_set_persistent_reg_byte(0, ControllerInitializedFlagByte, 1, &initialized);
}

void CheckRegister() {
    char initliazed = '\0';
    smc_get_persistent_reg_byte(0, ControllerInitializedFlagByte, 1, &initliazed);
    if(initliazed=='\0') {  // 寄存器数据未被设置
        InitRegister();
    }
}

void InitAxesAvailableStates() {
    for(int axis=0; axis<=gAxisNum; axis++) {
        char disableState = 0;
        smc_get_persistent_reg_byte(gHandle, DisabledAxisStartIndex+axis, 1, &disableState);
        gAxisAvailableStates[axis] = (disableState==0 ? 1 : 0);
    }
}

void InitializeCalibrationData() {
    for(int i=MLAxisRotation; i<=gAxisNum; i++) {
        AxisParam *ap = &gAxisPrm[i];
        ap->ppratio      = GetSingleAxisParamInRegister(i, AP_PPRatio);
        ap->posToInit    = GetSingleAxisParamInRegister(i, AP_InitPos);
        ap->posToTest    = GetSingleAxisParamInRegister(i, AP_TestPos);
        ap->homeDirect   = GetSingleAxisParamInRegister(i, AP_HomeDirection);
        ap->homeLevel    = GetSingleAxisParamInRegister(i, AP_HomeLevel);
        ap->equiv        = GetSingleAxisParamInRegister(i, AP_Equiv);
        ap->backlash     = GetSingleAxisParamInRegister(i, AP_Backlash);
    }
}

bool Connect(char *ip) {
    Logger(MLLogInfo, "\n-----------------------------------\n", __func__);
    Logger(MLLogInfo, "<%s> Connecting machine controller...\n", __func__);
    
    bool flag = IsValidIP(ip);
    gHandle = -1;
    gIsPowerOn = true;
    if (!flag) {
        Logger(MLLogError, "<%s>: Invaid IP {%s}.\n", __func__, ip);
    }
    
    isNeedStop = false;
    isNeedEmgStop = false;
    
    if (flag) {
        memset(errmsg, 0, 256);
        if (0 == smc_board_init(0, Ethernet, ip, 0)) {
            gHandle = 0;
            WORD cardNum;
            DWORD cardTypeList;
            WORD cardIdList;
            DWORD hwVersion, fwType, fwVersion, libVersion;
            smc_get_CardInfList(&cardNum, &cardTypeList, &cardIdList);
            smc_get_card_version(gHandle, &hwVersion);
            smc_get_card_soft_version(gHandle, &fwType, &fwVersion);
            smc_get_card_lib_version(&libVersion);
//            smc_get_total_axes(gHandle, &axisCount);
            Logger(MLLogInfo, "-- <CardNum> = %d, <CardTypeList> = 0x%x, <CardIDList> = 0x%x\n", cardNum, cardTypeList, cardIdList);
            Logger(MLLogInfo, "-- <HardwareVersion> = %ld\n", hwVersion);
            Logger(MLLogInfo, "-- <FirewareType> = %ld\n", fwType);
            Logger(MLLogInfo, "-- <FirewareVersion> = %ld\n", fwVersion);
            Logger(MLLogInfo, "-- <Library Version> = %ld\n", libVersion);
//            if(axisCount!=gAxisNum+1) {
//                Logger(MLLogError, "The count of valid axes is %d(expect: %d{1, %d}, the axis 0 should be a virtual axis)\n", axisCount-1, gAxisNum, gAxisNum);
//            }
            
            Logger(MLLogInfo, "<%s> try to clear alarm and [card] error code.\n", libVersion);
            nmcs_clear_card_errcode(gHandle);   // clear card error
            nmcs_clear_errcode(gHandle,2);      // clear bus error
            nmcs_set_alarm_clear(gHandle,2,0);
            
            for (int axis = 1; axis <= gAxisNum; axis++) {
                nmcs_clear_axis_errcode(gHandle, axis);
                AxisParam ap = gAxisPrm[axis];
                smc_set_profile_unit(gHandle, axis, ap.startSpeed*ap.ppratio, ap.runSpeed*ap.ppratio, ap.accTime, ap.accTime, ap.stopSpeed*ap.ppratio);
                smc_set_home_profile_unit(gHandle, axis, ap.startSpeed*ap.ppratio, ap.homeSpeed*ap.ppratio, ap.accTime, ap.accTime);
                // ioType:      0-PLimit, 1-NLimit, 2-Origin, 3-EmgStop, 4-DecStop, 5-Alarm, 6-Ready, 7-In_Place
                // mapIOType:   0-PLimit, 1-NLimit, 2-Origin, 3-Alarm, 4-Ready, 5-In_Place. 6-Geneal_IO
                smc_set_axis_io_map(gHandle, axis, 3, 6, 13, 0);
                smc_set_emg_mode(gHandle, axis, 1, 1);
            }
            
            pthread_mutex_init(&_mutex, NULL);
            
            sleep(1);   // wait board init finish and stable.
            
            short rtn = 0;
            for (int i = 1; i < 13 ; i++) {
                rtn |= smc_write_sevon_pin(gHandle, i, 0);
                usleep(200000);
            }
            
            SetBitState(MLOutLuxmetePower, MLLow);
            Logger(MLLogInfo, "<%s>:Launch input signal watching thread.\n", __func__);
            CheckInputSignal();
            CheckStart();
            CheckStop();
            CheckEmgStop();
            CheckStopActivedSignal();
            
            flag = true;
            SetEncoderUnit();
            
            CheckRegister();        // 检查寄存器状态，如寄存器数据未设置，自动初始化,
            InitializeCalibrationData();
            InitAxesAvailableStates();
            
            Logger(MLLogInfo, "<%s>:Successfully connect to machine controller.\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Fail to connect controller.\n", __func__);
            sprintf(errmsg, "Fail to connect controller.\n");
            flag = false;
        }
    }

    return flag;
}

double ReadDDAngle() {
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    res=smc_get_encoder_unit(MyCardNo,1,&Myencoder_value);//读编码值

    double angle = 0;
    angle = 1.0 * Myencoder_value/100;
   
    return angle;
}

void ResetSystem(void) {
    nmcs_reset_etc(gHandle);
}

void Disconnect(void) {
    if (gHandle != -1) {
        Logger(MLLogInfo, "<%s>: Disconnect controller.\n", __func__);
        gStopped = true;
        AllAxisStop();                      // stop all axis before disconnect controller.
        usleep(250000);
        nmcs_clear_card_errcode(gHandle);   // clear card error
        nmcs_clear_errcode(gHandle,0);      // clear bus error
        nmcs_set_alarm_clear(gHandle,2,0);
        
        for (int axis = 1; axis <= gAxisNum; axis++) {
            nmcs_clear_axis_errcode(gHandle, axis);
        }
        usleep(200000);
        
        short rtn;
        
        if ((rtn = smc_board_close(gHandle))) {
            Logger(MLLogError, "<Disconnect>: Fail to close controller, rtn:{%d}.\n", rtn);
        } else {
            Logger(MLLogInfo, "<Disconnect>: Success to disconnect controller.\n");
        }
        gHandle = -1;
        pthread_mutex_destroy(&_mutex);
    }
    Logger(MLLogInfo, "----------------- end ----------------------\n\n\n");
}

void ChangeAxisParam(int axis, double startSpeed, double runSpeed, double acc) {
    AxisParam param = gAxisPrm[axis];
    param.startSpeed = (startSpeed > 0) ? startSpeed : param.startSpeed;
    param.runSpeed = (runSpeed >= param.startSpeed) ? runSpeed : param.startSpeed;
    param.accTime = (acc > 0) ? acc : param.accTime;
    gAxisPrm[axis] = param;
    Logger(MLLogInfo, "<ChangeAxisParam>: Config axis %d start speed to %lf, run speed to %lf, acc time to %lf.\n", axis, param.startSpeed, param.runSpeed, param.accTime);
}

void TMoveAxisToPosition(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    double position = arguments->arg2;
    bool blocked = arguments->arg3;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    long currentPos = -1;
    short rtn = 0;
    gIsTesting = true;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    assert(axis >= 0);
    
    // check if the axis is enalbed
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>: disabled axis %d\n", __func__, axis);
        gIsTesting = false;
        return;
    }
    
    if (gHandle != -1) {
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        
        bool paramOK = CheckAxisParameters(axis, 0);
        if (rtn == 0 && paramOK) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, (long)(position*param.ppratio), MLAbsolute);
            
            if (rtn == 0) {
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { break; }  // stop to check state
                        if (isNeedEmgStop) {break;}
                    };
                    CheckAxisIOState(axis);
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                    param.isRuning = false;
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                if(rtn==104 || rtn==105) {
                    Logger(MLLogWarning, "<%s>: Axis %d is in the %s limit, cannot move .\n", __func__, axis, rtn==104?"positive":"negative", rtn);
                } else {
                    Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
                }
            }
        }
    }
    gIsTesting = false;
}

long MoveAxisToPosition(int axis, double position, bool blocked, bool pthread) {
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>:axis %d is disabled.\n", __func__, axis);
        return -1;
    }
    
    AxisParam axisPrm = GetAxisParam(axis);
    if (axisPrm.isRuning) { return 0; }
    if (isNeedStop) {return 0;}
    if (isNeedEmgStop) {return 0;}
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = position;
        args->arg3 = blocked;
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TMoveAxisToPosition, args);
        return 0;
    } else {
        long currentPos = -1;
        short rtn = 0;
        gIsTesting = true;
        short res = 0;
        WORD MyCardNo = 0;
        double encoderValue=0;

        assert(axis >= 0);
        
        if (gHandle != -1) {
            smc_write_sevon_pin(gHandle, axis, 0);
            AxisParam param = gAxisPrm[axis];
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);

            bool paramOK = CheckAxisParameters(axis, 0);
            if (rtn == 0 && paramOK) {
                res=smc_get_encoder_unit(MyCardNo,axis,&encoderValue);//读编码值
                currentPos = encoderValue;
                Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
                rtn |= smc_pmove_unit(gHandle, axis, (long)(position*param.ppratio), MLAbsolute);

                if (rtn == 0) {
                    if (blocked) {
                        while (!smc_check_done(gHandle, axis)) {
                            usleep(200000);     // check axis's state every 200ms
                            if (isNeedStop) {
                                break;
                            }
                            if (isNeedEmgStop) {
                                break;
                            }
                        };
                        CheckAxisIOState(axis);
                        res=smc_get_encoder_unit(MyCardNo,axis,&encoderValue);//读编码值
                        currentPos = encoderValue;
                        Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                    } else {
                        Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                    }
                }  else {
                    if(rtn==104 || rtn==105) {
                        Logger(MLLogWarning, "<%s>: Axis %d is in the %s limit, cannot move .\n", __func__, axis, rtn==104?"positive":"negative", rtn);
                    } else {
                        Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                        memset(errmsg, 0, 256);
                        sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
                    }
                }
            }
        }
        gIsTesting = false;

        return blocked ? currentPos : -997;
    }
}

void AdjustHoldPosition(int *axises, double distance, bool blocked) {
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    MLAxis axis = axises[0];
    MoveAxisDistance(axis,distance, false, false);

    MLAxis axi1 = axises[1];
    MoveAxisDistance(axi1,-distance, false, false);
}

void TMoveAxisDistance(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    double distance = arguments->arg2;
    bool blocked = arguments->arg3;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    long currentPos = -1;
    long validPos = distance;
    short rtn = 0;
    bool flag = false;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    assert(axis >= 0);
    
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
        return;
    }
    if(distance==0) {
        return;
    }
    
    gIsTesting = true;
    
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        
        bool bSuccess = CheckAxisParameters(axis, 0);
        
        if (rtn==0 && bSuccess) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
            
            if (rtn==0) {
                flag = true;
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { break; } // stop to check state.
                        if (isNeedEmgStop) {
                            break;
                        }
                     }
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position>]: %ld\n", __func__, axis, currentPos);
                    param.isRuning = false;
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                if(rtn==104 || rtn==105) {
                    Logger(MLLogWarning, "<%s>: Axis %d is in the %s limit, cannot move .\n", __func__, axis, rtn==104?"positive":"negative", rtn);
                } else {
                    Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", __func__, axis, rtn);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToPosition, rtn: {%d}.\n", axis, rtn);
                }
            }
        }
    }
    gIsTesting = false;
}

bool MoveAxisDistance(int axis, double distance, bool blocked, bool pthread) {
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>: axis %d is disabled.\n", __func__, axis);
        return true;
    }
    if(distance==0) {
        return true;
    }
    
    if (pthread) {
        AxisParam axisPrm = GetAxisParam(axis);
        if (axisPrm.isRuning) { return false; }
        if (isNeedStop) {return 0;}
        if (isNeedEmgStop) {return 0;}
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = distance;
        args->arg3 = blocked;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TMoveAxisDistance, args);
        
        return false;
    } else {
        long currentPos = -1;
        long validPos = distance;
        short rtn = 0;
        gIsTesting = true;
        bool flag = false;
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;

        assert(axis >= 0);
        
        if (gHandle != -1) {
            smc_write_sevon_pin(gHandle, axis, 0);
            AxisParam param = gAxisPrm[axis];
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
            
            bool bSuccess = CheckAxisParameters(axis, 0);
            
            if (rtn==0 && bSuccess) {
                res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值

                currentPos = Myencoder_value;
                validPos += currentPos;
                Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
                
                rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
                
                if (rtn==0) {
                    flag = true;
                    if (blocked) {
                        while (!smc_check_done(gHandle, axis)) {
                            usleep(200000);     // check axis's state every 200ms
                            if (isNeedStop) {
                                break;
                            }
                            if (isNeedEmgStop) {
                                break;
                            }
                        }
                        res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                        currentPos = Myencoder_value;
                        param.isRuning = false;
                        Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__,  axis, currentPos);
                    } else {
                        Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                    }
                } else {
                    flag = false;
                    Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", __func__, axis, rtn);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", axis, rtn);
                }
            } else {
                Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistance.\n", __func__, axis);
            }
        }
        gIsTesting = false;

        return flag;
    }
}

long MoveAxisDistanceQuick(int axis, double distance, bool blocked) {
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
        return -1;
    }
    long currentPos = -1;
    long validPos = distance;
    short rtn = 0;
    gIsTesting = true;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    assert(axis >= 0);
    if (isNeedStop) {return 0;}
    if (isNeedEmgStop) {return 0;}
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        
        if (rtn==0 && CheckAxisParameters(axis, 0)) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLRelative);
            
            if (rtn==0) {
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                    }
                    CheckAxisIOState(axis);
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__,  axis, currentPos);
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d} using MoveAxisToDistance, rtn: {%d}.\n", axis, rtn);
            }
        } else {
            Logger(MLLogError, "<%s>: Fail to move axis{%d} using MoveAxisToDistanceQuick.\n", __func__, axis);
        }
    }
    gIsTesting = false;
    
    return blocked ? currentPos : -997;
}

void TJMoveAxis(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    int direction = arguments->arg2;
    AxisParam param = gAxisPrm[axis];
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    short rtn = 0;
    if (gHandle != -1) {
        if(gAxisAvailableStates[axis]==0) {
            Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
            return;
        }
        gIsTesting = true;
        
        param.isRuning = true;
        gAxisPrm[axis] = param;
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        
        if (direction == 0) {
            if (2 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d is already at negative limit. cancel action.\n", __func__, axis, rtn);
                AxisStop(axis);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        } else if (direction == 1) {
            if (1 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d is already at positive limit. cancel action.\n", __func__, axis, rtn);
                AxisStop(axis);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        }
        
        sleep(1);
        bool bSuccess = CheckAxisParameters(axis, 0);
        
        if (bSuccess && rtn == 0) {
            rtn |= smc_vmove(gHandle, axis, direction);
        }
        
        if (rtn != 0 && rtn!=104 && rtn!=105) {
            Logger(rtn==4?MLLogWarning:MLLogError, "<%s>: Axis %d fail to move axis using JMove, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
//    param.isRuning = false;
    gAxisPrm[axis] = param;
}

void JMoveAxis(int axis, int direction, bool pthread) {
    AxisParam param = gAxisPrm[axis];
    
    if (gIsTesting) { return; }
    if (param.isRuning) {return;}
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    
    if (pthread) {
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TJMoveAxis, NULL);
    } else {
        short rtn = 0;
        
        if (gHandle != -1) {
            if(gAxisAvailableStates[axis]==0) {
                Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
                return;
            }
            gIsTesting = true;
            
            param.isRuning = true;
            gAxisPrm[axis] = param;
            rtn |= smc_write_sevon_pin(gHandle, axis, 0);
            
            if (direction == 0) {
                if (2 == CheckAxisIOState(axis)) {
                    Logger(MLLogInfo, "<%s>: Axis %d at negative limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                    AxisStop(axis);
                    gIsTesting = false;
                    param.isRuning = false;
                    gAxisPrm[axis] = param;
                    return;
                }
            } else if (direction == 1) {
                if (1 == CheckAxisIOState(axis)) {
                    Logger(MLLogInfo, "<%s>: Axis %d at positive limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                    AxisStop(axis);
                    gIsTesting = false;
                    param.isRuning = false;
                    gAxisPrm[axis] = param;
                    return;
                }
            }
            
            usleep(100*1000);
            
            if (CheckAxisParameters(axis, 0)) {
                Logger(MLLogInfo, "to vmove axis(%d)\n", axis);
                CheckAllAxisState();
                rtn |= smc_vmove(gHandle, axis, direction);
                Logger(MLLogInfo, "vmove return: %d\n", rtn);
                CheckAllAxisState();
                usleep(100*1000);
            }
            
            if (rtn != 0 && rtn!=104 && rtn!=105 && rtn!=4) {
                Logger(rtn==4?MLLogWarning:MLLogError, "<%s>: Axis %d fail to move axis using JMove, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
            }
        }
        gIsTesting = false;
        param.isRuning = false;
        gAxisPrm[axis] = param;
    }
}

void JHoldMoveAxis(int axis, int direction) {
    if (gIsTesting) { return; }
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    short rtn = 0;
    gIsTesting = true;

    if (gHandle != -1) {
        AxisParam param = gAxisPrm[axis];
        
        rtn |= smc_write_sevon_pin(gHandle, axis, 0);
        
        if (direction == 0) {
            if (2 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at negative limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        } else if (direction == 1) {
            if (1 == CheckAxisIOState(axis)) {
                Logger(MLLogInfo, "<%s>: Axis %d at positive limit, not need move, rtn: {%d}.\n", __func__, axis, rtn);
                gIsTesting = false;
                param.isRuning = false;
                gAxisPrm[axis] = param;
                return;
            }
        }
        
        sleep(1);
        if (CheckAxisParameters(axis, 0)) {
            rtn |= smc_vmove(gHandle, axis, direction);
        }
        
        if (rtn != 0 && rtn!=104 && rtn!=105) {
            Logger(rtn==4?MLLogWarning:MLLogError, "<%s>: Axis %d fail to move axis using JMove, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to move axis using JMove, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
}

void TJMoveAxisWithBlock(void *args) {
    TArgs *arguments = (TArgs *)args;
    int axis = *((int *)arguments->arg1);
    int direction = arguments->arg2;
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    JMoveAxis(axis, direction, false);
    gIsTesting = true;
    
    if (gHandle != -1) {
        while (!smc_check_done(gHandle, axis)) {
            usleep(200000);     // check axis's state every 200ms
            if (0 == CheckAxisIOState(axis)) {
                Logger(MLLogError, "<%s>: Servo clarm at axis %d.\n", __func__, axis);
                break;
            }
            if (isNeedStop) { break; }  // stop to check state.
            if (isNeedEmgStop) {
                break;
            }
        }
    }
    AxisParam axisPrm = GetAxisParam(axis);
    axisPrm.isRuning = false;
    SetAxisParam(axis, axisPrm);
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Axis %d stop jmove.\n", __func__, axis);
    } else {
        Logger(MLLogInfo, "<%s>: Axis %d finished jmove.\n", __func__, axis);
    }
    gIsTesting = false;
}

void JMoveAxisWithBlock(int axis, int direction, bool pthread) {
    AxisParam axisPrm = GetAxisParam(axis);
    if (axisPrm.isRuning) { return; }
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    Logger(MLLogInfo, "<%s>: Axis %d call `JMoveAxisWithBlock`\n", __func__, axis);
    
    if(gAxisAvailableStates[axis]==0) {
        Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
        return;
    }
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = &axis;
        args->arg2 = direction;
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TJMoveAxisWithBlock, args);
    } else {
        JMoveAxis(axis, direction, false);
        gIsTesting = true;
        
        if (gHandle != -1) {
            while (!smc_check_done(gHandle, axis)) {
                usleep(200000);     // check axis's state every 200ms
                if (0 == CheckAxisIOState(axis)) {
                    Logger(MLLogError, "<%s>: Servo clarm at axis %d.\n", __func__, axis);
                    break;
                }
                if (isNeedStop) { break; }  // stop to check axis state.
                if (isNeedEmgStop) {
                    break;
                }
            }
        }
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Axis %d stop jmove.\n", __func__, axis);
        } else {
            Logger(MLLogInfo, "<%s> Axis %d finished jmove.\n", __func__, axis);
        }
        gIsTesting = false;
    }
}

bool CheckAxisState(int *axises, int axisCount, bool pthread) {
    bool allStopped = true;
    
    bool stopFlags[axisCount];
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        stopFlags[i] = false;
    }
    
    if (gHandle != -1) {
        do {
            allStopped = true;
            for (int index = 0; index < axisCount; index++) {
                int axis = axises[index];
                bool stopped = (1 == smc_check_done(gHandle, axis));
                if(stopped != stopFlags[index]) {
                    stopFlags[index] = stopped;
                    AxisParam axisPrm = GetAxisParam(axis);
                    axisPrm.isRuning = stopped?false:true;
                    SetAxisParam(axis, axisPrm);
                    Logger(MLLogInfo, "<%s> Axis %d %s", axis, stopped ? "finish movement" : "moves again");
                }
                allStopped = allStopped && stopped;
            }
            
            if (isNeedStop) {   // stop to check axis state.
//                Logger(MLLogInfo, "<%s>: stop to check state.\n", __func__);
                break;
            }
            if (isNeedEmgStop) {
                break;
            }
            usleep(100*1000);
        } while (!allStopped && !pthread);
    } else {
        Logger(MLLogInfo, "<%s>: Controller is disconnected.\n", __func__);
    }
    
    return allStopped;
}

bool CheckMutliAxisState(int *axises, int axisCount, func callback, double value1, double value2) {
    bool flag = false;
    int bitflags = 0;
    
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        bitflags = ((bitflags << 1) | 0x1);
    }
    
    while (!flag) {
        for (int index = 0; index < axisCount; index++) {
            int axis = axises[index];
            
            if (((bitflags >> index) & 0x01) != 0) {
                if (1 == smc_check_done(gHandle, axis)) {
                    bitflags = (~bitflags) ^ (1 << index);
                    bitflags = ~bitflags;
                }
            }
        }
        
        if (!bitflags) {    // 轴全部到位
            flag = true;
        }
        
        if (callback != NULL) {
            callback(value1, value2);
        }
    }
    
    return flag;
}

bool CheckAxisHomeState(int *axises, int axisCount) {
    bool flag = false;
    int bitflags = 0;
    
    // 设定标志，默认为1， 如果变为0则表示轴运动到位.
    for (int i = 0; i < axisCount; i++) {
        bitflags = ((bitflags << 1) | 0x1);
    }
    
    while (!flag) {
        for (int index = 0; index < axisCount; index++) {
            int axis = axises[index];
            
            if (((bitflags >> index) & 0x01) != 0) {
                WORD state;
                if ((0==smc_get_home_result(gHandle, axis, &state)) && state) {
                    bitflags = (~bitflags) ^ (1 << index);
                    bitflags = ~bitflags;
                    Logger(MLLogInfo, "<%s>: Axis %d success to go home\n", __func__, axis);
                }
            }
        }
        
        if (!bitflags) {    // 轴全部到位
            flag = true;
        }
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed\n", __func__);
            break;
        }
        if (isNeedEmgStop) {
            break;
        }
    }
    
    return flag;
}

void AxisGoHome(int axis, bool blocked) {
    AxisParam param = gAxisPrm[axis];
//    if (gIsTesting) { return; }
    if (param.isRuning) {return;}
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    assert(axis >= 0);
    gIsTesting = true;
    short rtn = 0;
    
    if (gHandle != -1) {
        if(gAxisAvailableStates[axis]==0) {
            Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
            gIsTesting = false;
            return;
        }
        
        param.isRuning = true;
        gAxisPrm[axis] = param;
        Logger(MLLogInfo, "<%s>: Axis %d call `AxisGoHome`\n", __func__, axis);

        bool bSuccess = CheckAxisParameters(axis, 1);   // check home parameter
        
        if (bSuccess) {
            rtn |= smc_write_sevon_pin(gHandle, axis, 0);
            rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
            rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//            rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
            
            rtn |= smc_set_homemode(gHandle, axis, param.homeDirect, 1, homeModes[axis-1], 0);
            Logger(rtn!=0?MLLogError:MLLogInfo, "<%s>: Axis %d home-mode to {dir:%d mode:%d}, rtn: {%d}\n", __func__, axis, param.homeDirect, homeModes[axis-1], rtn);

            
            while (!smc_check_done(gHandle, axis)) {
                usleep(100000);
            }
            if (axis==MLAxisLightSource) {
                SetBitState(MLOutCylinderHome, 0);
                SetBitState(MLOutCylinderShop, 1);
            }
            rtn = smc_home_move(gHandle, axis);
            
            if (rtn==0) {
                WORD state;
                
                if (blocked) {
                    while (!smc_get_home_result(gHandle, axis, &state)) {
                        if (state) {
                            double orgPosition;
                            smc_get_position_unit(gHandle, axis, &orgPosition);
                            Logger(MLLogInfo, "<%s>: Axis %d success to go home, Position: %lf\n", __func__, axis, orgPosition);
                            break;  // 回原点成功
                        }
                        if (isNeedStop) { return; }  // stop to check axis state.
                        if (isNeedEmgStop) {return;}
                        usleep(100000);
                    }
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d go home in unblock mode\n", __func__, axis);
                }
            } else {
                Logger(MLLogError, "<%s>: Axis %d fail to moving at home mode, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Axis %d fail to moving at home mode, rtn: {%d}.\n", axis, rtn);
            }
        } else  {
            Logger(MLLogError, "<%s>: Axis %d fail to setup home mode, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Axis %d fail to setup home mode, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
    param.isRuning = false;
    gAxisPrm[axis] = param;
}

bool IsStopActived(void) {
    return isNeedStop;
}

bool IsMoveFinished(void) {
    return isActiveFinished;
}

void stopSingleChange(void){
    isNeedStop = false;
}

void InactiveStop(void) {
//    pthread_mutex_lock(&_mutex);
    isNeedStop = false;
    isNeedEmgStop = false;
    isActiveFinished = false;
    gIsTesting = false;
    Logger(MLLogInfo, "<%s>: Reset stop state.\n", __func__);
//    pthread_mutex_unlock(&_mutex);
}

char *GetErrorMessage() {
    char *errorMsg = (char *)malloc(512 * sizeof(char));
    memset(errorMsg, 0, 512);
    
    if (gHandle != -1) {
        do {
            DWORD errorcode = 0;
            nmcs_get_errcode(gHandle, 2, &errorcode);

            if (errorcode != 0) {
                sprintf(errorMsg, "The bus error(error code: 0x%lx)\n", errorcode);
                break;
            }
            
            nmcs_get_card_errcode(gHandle, &errorcode);

            if (errorcode != 0) {
                sprintf(errorMsg, "The bus error(error code: 0x%lx)\n", errorcode);
                break;
            }
            
            for (int axis = 1; axis <= gAxisNum; axis++) {
                int state = CheckAxisIOState(axis);
                
                if (state == 0) {
                    break;
                }
            }
            
        } while (0);
        sprintf(errorMsg, "%s%s", errorMsg, errmsg);
    }
    return errorMsg;
}

void ClearAxisErrorCode(int axis){
    nmcs_clear_axis_errcode(gHandle, axis);
    memset(errmsg, 0, 256);
}

void ClearBusErrorCode() {
    for(int axis=MLAxisRotation; axis<gAxisNum; axis++) {
        ClearAxisErrorCode(axis);
    }
    nmcs_set_alarm_clear(gHandle, 2, 0);    // clear alarm signal
    nmcs_clear_errcode(gHandle, 2);         // clear bus error
    nmcs_clear_card_errcode(gHandle);       // clear bus error code
    memset(errmsg, 0, 256);
}

void TResetAllAxis() {
    isActiveFinished = false;
    SetBitState(MLOutCylinderHome, 0);
    SetBitState(MLOutCylinderShop, 1);
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop reset system.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    JMoveAxisWithBlock(MLAxisLightSource, 0, false);
    int axises0[] = {MLAxisLightSource};
    CheckAxisState(axises0, 1, false);
    
    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    JMoveAxisWithBlock(MLAxisLaser, 0, false);
    int axises1[] = {MLAxisLaser};
    CheckAxisState(axises1, 1, false);
    
    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    MLSwingAngleAbsolute(0, 0, 0, 1, 1, 1, false);
    int axises[] = { MLAxisRotationX, MLAxisRotation };
    MoveToInitPos(axises, 2);
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Power OFF completed\n", __func__);
}

void ResetAllAxis(bool pthread) {
    bool moving = false;
    for (int axis = 0; axis <= gAxisNum; axis++) {
        AxisParam axisPrm = GetAxisParam(axis);
        moving |= axisPrm.isRuning;
    }
    
    if (moving) {
        Logger(MLLogInfo, "<%s>: Some axis is moving, please wait moving finish then call `AllAxisGoHome`.\n", __func__);
        return;
    }
    
    if (pthread) {
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TResetAllAxis, NULL);
    } else {
        TResetAllAxis();
    }
}

void TAllAxisGoHome() {
    CheckAllAxisState();
    Logger(MLLogInfo, "<%s>: Reset system...\n", __func__);
    isActiveFinished = false;
    SetBitState(MLOutCylinderHome, 0);
    if (isNeedStop) {return;}
    /* shield axis 9 */
//    if (isNeedEmgStop) {return;}
    if(gAxisAvailableStates[MLAxisLightSource]==1) {
        Logger(MLLogInfo, "axis %d go negative limit...\n", MLAxisLightSource);
        CheckAllAxisState();
        JMoveAxisWithBlock(MLAxisLightSource, 0, false);
        int axises0[] = {MLAxisLightSource};
        CheckAxisState(axises0, 1, false);
    }
//
//    if (isNeedStop) {
//        isActiveFinished = true;
//        Logger(MLLogInfo, "<%s>: Stop button pressed when light axis go home, at call `JMoveAxisWithBlock`\n", __func__);
//        return;
//    }
    CheckAllAxisState();
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    Logger(MLLogInfo, "Axes {%d} going home...\n", MLAxisLifter);
    AxisGoHome(MLAxisLifter, true); // wait success to go home.

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when lifter axis go home, at call `AxisGoHome`.\n", __func__);
        return;
    }
    Logger(MLLogInfo, "Axes {%d, %d, %d, %d, %d, %d} going home...\n", MLAxisRotation, MLAxisHolderX, MLAxisHolderY, MLAxisLaser, MLAxisX, MLAxisY);
    CheckAllAxisState();
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    int axises1[] = {MLAxisRotation, MLAxisHolderX, MLAxisHolderY, MLAxisLaser, MLAxisX, MLAxisY};
    ManyAxisGoHome(axises1, 6, false);

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home, at call `ManyAxisGoHome`.\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    JMoveAxisWithBlock(MLAxisLaser, 0, false);
    int axises2[] = {MLAxisLaser};
    CheckAxisState(axises2, 1, false);

    if (isNeedStop) {
        isActiveFinished = true;
        Logger(MLLogInfo, "<%s>: Stop button pressed when laser axis go home, at call `JMoveAxisWithBlock`.\n", __func__);
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    
    Logger(MLLogInfo, "Axes {%d, %d} going home...\n", MLAxisRotationX, MLAxisRotationY);
    int axises3[] = {MLAxisRotationX, MLAxisRotationY};
    ManyAxisGoHome(axises3, 2, false);
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when rotate axis go home, at call `ManyAxisGoHome`.\n", __func__);
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    
    Logger(MLLogInfo, "Axes %d going home...\n", MLAxisLightSource);
    int axes4[] = {MLAxisLightSource};
    ManyAxisGoHome(axes4, 1, false);
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when lightsource axis go home, at call `ManyAxisGoHome`.\n", __func__);
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    
    
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Power ON completed and all axises finish go home.\n", __func__);
}

void AllAxisGoHome(bool pthread) {
    bool moving = false;
    for (int axis = 0; axis <= gAxisNum; axis++) {
        AxisParam axisPrm = GetAxisParam(axis);
        moving |= axisPrm.isRuning;
    }

    if (moving) {
        Logger(MLLogInfo, "<%s>: Some axis is moving, please wait moving finish then call `AllAxisGoHome`.\n", __func__);
        return;
    }
    
    if (pthread) {
        Logger(MLLogInfo, "<%s>:start new thread...\n", __func__);
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TAllAxisGoHome, NULL);
         Logger(MLLogInfo, "<%s>: Run function in new thread...\n", __func__);
    } else {
        TAllAxisGoHome();
        /*
        //    if (gIsTesting) { return; }
        //    gIsTesting = true;
        SetBitState(MLOutCylinderHome, 0);
        SetBitState(MLOutCylinderShop, 1);
        
        JMoveAxisWithBlock(MLAxisLight, 0, false);
        int axises0[] = {MLAxisLight};
        CheckAxisState(axises0, 1, false);
        
        AxisGoHome(MLAxisLifter, false);

        int axises1[] = {1,5,6,7,8,10,11,12};
        ManyAxisGoHome(axises1, 8, false);

        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int axises2[] = {MLAxisLaser};
        CheckAxisState(axises2, 1, false);
        
        int axises3[] = {2,3};
        ManyAxisGoHome(axises3, 2, false);
        
    //    gIsTesting = false;
         */
    }
}

void DebugHold1KMove(int *axises, int axisCount,int testCount) {
    gIsTesting = true;
    
    const char *pFileName = "/Users/Shared/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }
    
    for (int i = 0; i < testCount; i++) {
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        JMoveAxis(axises[0],  1, false);
        JMoveAxis(axises[1],  1, false);
        JMoveAxis(axises[2],  1, false);
        JMoveAxis(axises[3],  1, false);
        
        CheckAxisState(axises, axisCount, false);
        
        JMoveAxis(axises[0],  0, false);
        JMoveAxis(axises[1],  0, false);
        JMoveAxis(axises[2],  0, false);
        JMoveAxis(axises[3],  0, false);
        
        CheckAxisState(axises, axisCount, false);
        
        fprintf(pFile,"<%s>: Dut Hold Move %d times\n", __func__, i+61);
    }
    
    fclose(pFile);
    gIsTesting = false;
    
}

void DebugManyAxisMovePAndN(int *axises, int axisCount,int testCount) {
    const char *pFileName = "/Users/Shared/FlareLog.txt";
    FILE *pFile;
    pFile = fopen(pFileName,"a");
    if (NULL == pFile) {
        printf("error");
    }

    for (int i = 0; i < testCount; i++) {
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
        }
        
        CheckAxisState(axises, axisCount, false);
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            JMoveAxis(axis, axisPrm.homeDirect ? 1 : 0, false);
        }

        
        CheckAxisState(axises, axisCount, false);
        
        MLAxis axis = axises[0];
        fprintf(pFile,"<%s>: Axis %d Move %d times\n", __func__, axis, i+1);

    }
    
    fclose(pFile);
}

void TManyAxisGoBack(void *args) {
    TArgs *arguments = (TArgs *)args;
    int *axises = (int *)arguments->arg1;
    int axisCount = (int)arguments->arg2;
    
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
        AxisParam axisPrm = GetAxisParam(axis);
        JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit\n", __func__);
            break;
        }
        if (isNeedEmgStop) {
            Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
            return;
        }
    }
    
    CheckAxisState(axises, axisCount, false);
    Logger(MLLogInfo, "<%s>: Many axises finish go negative limit.\n", __func__);
}

void ManyAxisGoBack(int *axises, int axisCount, bool pthread) {
    bool isRuning = false;
    
    for (int ax = 0; ax < axisCount; ax++) {
        int axis = axises[ax];
        AxisParam axisPrm = GetAxisParam(axis);
        isRuning |= axisPrm.isRuning;
    }
    
    if (isRuning) {
        Logger(MLLogInfo, "<%s>: Some axis is running, please wait it.\n", __func__);
        return;
    }
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = axises;
        args->arg2 = axisCount;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TManyAxisGoBack, args);
        
    } else {
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            AxisParam axisPrm = GetAxisParam(axis);
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            int ioState = CheckAxisIOState(axis);
            int dir = axisPrm.homeDirect ? 0 : 1;
            if((dir==1 && ioState!=1)
               || (dir==0 && ioState!=2)) {
                JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            }
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit\n", __func__);
                break;
            }
            if (isNeedEmgStop) {
                Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
                return;
            }
        }
        
        // need no-block
        CheckAxisState(axises, axisCount, true);
    }
}

void MoveToInitPos(int *axises, int count) {
    short rtn;
    
    for (int i = 0; i < count; i++) {
        int axis = axises[i];
        AxisParam ap = gAxisPrm[axis];
        rtn = smc_pmove_unit(gHandle, axis, ap.posToInit, MLAbsolute);
        Logger(MLLogInfo, "<%s>: Axis %d go to init position %.0lf (in pluse), rtn: (%d}\n", __func__,  axis, ap.posToInit, rtn);
    }
}

void TManyAxisGoHome(void *args) {
    TArgs *arguments;
    arguments = (TArgs *)args;
    int *axises = (int *)arguments->arg1;
    int axisCount = arguments->arg2;
    
    bool block = false;
    
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
        AxisParam axisPrm = GetAxisParam(axis);
        int ioState = CheckAxisIOState(axis);
        int dir = axisPrm.homeDirect ? 0 : 1;
        if((dir==1 && ioState!=1)
           || (dir==0 && ioState!=2)) {
            JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
        }
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) {
            Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__);
            return;
        }
    }
    
    if (CheckAxisState(axises, axisCount, false)) {
        Logger(MLLogInfo, "<%s>: Many axises have moved finish.\n", __func__);
    }
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go back to negative limit.\n", __func__);
        isActiveFinished = true;
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    for (int index = 0; index < axisCount; index++) {
        MLAxis axis = axises[index];
        Logger(MLLogInfo, "<%s>: Axis %d start go home ...\n", __func__, axis);
        AxisGoHome(axis, block);
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    }
    
    if (CheckAxisHomeState(axises, axisCount)) {
        Logger(MLLogInfo, "<%s>: Many axis success to go home\n", __func__);
    }
    
    if (isNeedStop) {
        Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
        isActiveFinished = true;
        return;
    }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (!block) {
        MoveToInitPos(axises, axisCount);
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises move the init position.\n", __func__);
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    }
    
    isActiveFinished = true;
    Logger(MLLogInfo, "<%s>: Many axises finish go home\n", __func__);
}

void ManyAxisGoHome(int *axises, int axisCount, bool pthread) {
    bool isRuning = false;
    
    for (int ax = 0; ax < axisCount; ax++) {
        int axis = axises[ax];
        AxisParam axisPrm = GetAxisParam(axis);
        isRuning |= axisPrm.isRuning;
    }
    
    if (isRuning) {
        Logger(MLLogInfo, "<%s>: Some axis is running, please wait it.\n", __func__);
        return;
    }
    
    if (pthread) {
        TArgs *args = (TArgs *)malloc(sizeof(TArgs));
        args->arg1 = axises;
        args->arg2 = axisCount;
        
        pthread_t thrd;
        pthread_create(&thrd, NULL, (void *)TManyAxisGoHome, args);
    } else {
        Logger(MLLogInfo, "<%s>: Call `ManyAxisGoHome`\n", __func__);
        bool block = false;
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            
            if(gAxisAvailableStates[axis]==0) {
                Logger(MLLogInfo, "<%s>: axis %d is disabled\n", __func__, axis);
                continue;
            }
            
            Logger(MLLogInfo, "<%s>: Axis %d start go to limit ...\n", __func__, axis);
            AxisParam axisPrm = GetAxisParam(axis);
            int ioState = CheckAxisIOState(axis);
            int dir = axisPrm.homeDirect ? 0 : 1;
            if((dir==1 && ioState!=1)
               || (dir==0 && ioState!=2)) {
                JMoveAxis(axis, axisPrm.homeDirect ? 0 : 1, false);
            }
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
                isActiveFinished = true;
                return;
            }
        }
        CheckAllAxisState();
        
        if (CheckAxisState(axises, axisCount, false)) {
            Logger(MLLogInfo, "<%s>: Many axises have moved finish.\n", __func__);
        }
        
        if (isNeedStop) {
            Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
            isActiveFinished = true;
            return;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        
        for (int index = 0; index < axisCount; index++) {
            MLAxis axis = axises[index];
            Logger(MLLogInfo, "<%s>: Axis %d start go home ...\n", __func__, axis);
            AxisGoHome(axis, block);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when some axises go home.\n", __func__);
                isActiveFinished = true;
                return;
            }
            if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        }
        
        if (CheckAxisHomeState(axises, axisCount)) {
            Logger(MLLogInfo, "<%s>: Many axis success to go home\n", __func__);
        }
        
        if (!block) {
            MoveToInitPos(axises, axisCount);
            
            if (isNeedStop) {
                Logger(MLLogInfo, "<%s>: Stop button pressed when go to init position.\n", __func__);
                isActiveFinished = true;
                return;
            }
            if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
        }
        Logger(MLLogInfo, "<%s>: Many axises finish go home\n", __func__);
    }
}

void ResetAxisSystem(void) {
    AllAxisStop();
    AllAxisGoHome(false);
}

void ConfigSpeed(int axis, int speed, int mode) {}

void AxisStop(int axis) {
    assert(axis >= 0);
    
    if (gHandle != -1) {
        short ret = smc_stop(gHandle, axis, 1);
        if(ret!=0) {
            Logger(MLLogError, "fail to stop axis %d.[%d]\n", axis, ret);
        }
    }
}

void TAllAxisStop() {
    isNeedStop = true;
    isNeedEmgStop = true;
    for (int axis = 0; axis < gAxisNum; axis++) {
        AxisStop(axis);
    }
    gIsTesting = false;
    Logger(MLLogInfo, "<%s>: Stop all axes.\n", __func__);
}

void AllAxisStop(void) {
    isNeedStop = true;
    isNeedEmgStop = true;
    for (int axis = 0; axis < gAxisNum; axis++) {
        AxisStop(axis);
    }
    Logger(MLLogInfo, "<%s>: Stop all axes.\n", __func__);
}

void EmgStop(void) {
    isNeedStop = true;
    isNeedEmgStop = true;
    smc_emg_stop(gHandle);
    Logger(MLLogInfo, "<%s>: Emergency Stop.\n", __func__);
}

void SetAxisRatio(int axis, double ratio) {
    assert(axis >= 0);
    
    AxisParam prm = gAxisPrm[axis];
    prm.ppratio = ratio;
    gAxisPrm[axis] = prm;
}

static bool SetOutputBitState(MLOutSensor outbit, MLLevel level)
{
    int rst = -1;
    if (gHandle != -1) {
        rst = smc_write_outbit(gHandle, outbit, level);
        Logger(MLLogInfo, "<%s> set Bit[%d:%s] to %d [result: %d]\n", __func__, outbit, outBitDescs[outbit], level, rst);
        unsigned int reverseLevel = abs((int)(level - 1));
        if(outbit==MLOutCylinderHome || outbit==MLOutCylinderShop) {
            MLOutSensor anotherBit = outbit==MLOutCylinderHome ? MLOutCylinderShop : MLOutCylinderHome;
            LuxMeterState targetState = (level==MLLow ? LuxMeter_Home : LuxMeter_Shop); // default: outbit is MLOutCylinderHome
            if(outbit==MLOutCylinderShop) {
                targetState = (level==MLLow ? LuxMeter_Shop : LuxMeter_Home);
            }
            rst=smc_write_outbit(gHandle, anotherBit, reverseLevel);
            Logger(MLLogInfo, "<%s> set Another-Bit[%d:%s] to %d [result: %d]\n", __func__, anotherBit, outBitDescs[anotherBit], reverseLevel, rst);
            int time = 0;
            do {
                if(luxmeterState==targetState) {
                    break;
                }
                usleep(500000);
                time += 500;
            } while (time < 8000);
            rst = luxmeterState==targetState ? 0 : -1;
        } else if(level==MLLow && (outbit==MLOutDoorClose||outbit==MLOutDoorOpen)) {
            MLOutSensor anotherBit = outbit==MLOutDoorClose ? MLOutDoorOpen : MLOutDoorClose;
            rst = smc_write_outbit(gHandle, anotherBit, reverseLevel);
            Logger(MLLogInfo, "<%s> set Another-Bit[%d:%s] to %d [result: %d]\n", __func__, anotherBit, outBitDescs[anotherBit], reverseLevel, rst);
            usleep(200*1000);
            DoorState targetState = outbit==MLOutDoorClose ? Door_Closed : Door_Opened;
            int time = 0; int timeout=8000;
            while(doorState!=targetState && time<timeout
                  && (outbit==MLOutDoorOpen || (outbit==MLOutDoorClose && gIsRaster==false))) {
                usleep(100*1000);
                time+=100;
            }
            rst = smc_write_outbit(gHandle, outbit, reverseLevel);
            Logger(MLLogInfo, "<%s> set Bit[%d:%s] to %d [result: %d]\n", __func__, outbit, outBitDescs[outbit], reverseLevel, rst);
        }

    }
    
    return rst==0;
}

void SetBitState(int bit, MLLevel level) {
    SetOutputBitState(bit, level);
}

MLLevel GetInBitState(int bit) {
    MLLevel level = MLLow;
    
    if (gHandle != -1) {
        level = smc_read_inbit(gHandle, bit);
    }
    
    return level;
}

MLLevel GetOutBitState(int bit) {
    MLLevel level = MLLow;
    
    if (gHandle != -1) {
        level = smc_read_outbit(gHandle, bit);
    }
    
    return level;
}

MLLevel GetOutBit(int bit) {
    MLLevel lvl = smc_read_outbit(gHandle, bit);
    return lvl;
}

DWORD GetOutbits(int port) {
    DWORD bits=0;
    if(gHandle!=-1) {
        bits = smc_read_outport(gHandle, port);
    }
    return bits;
}

MLLevel GetInBit(int bit) {
    MLLevel lvl = smc_read_inbit(gHandle, bit);
    return lvl;
}

DWORD GetInBits(int port) {
    DWORD bits=0;
    if(gHandle!=-1) {
        bits = smc_read_inport(gHandle, port);
    }
    return bits;
}

void Cylinder(int bit, MLLevel level) {
    if (gHandle != -1) {
        SetBitState(bit, level);
    }
}

// Illuminometer API
bool ConnectIlluminometer(string portName) {
    assert(portName != NULL);   // 断言串口名不为空
    gLuxConnected = false;
    
    if (!JKConnectCL200A(portName, "00")) {
        Logger(MLLogError, "<%s>: Can't connect the illuminometer.\n", __func__);
        gLuxConnected = false;
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Can't connect the luxmeter.\n");
    } else {
        Logger(MLLogInfo, "<%s>: Connect the illuminometer.\n", __func__);
        gLuxConnected = true;
    }

    return gLuxConnected;
}

void DisconnectIlluminometer() {
    short rtn;
    if (-1 != (rtn = JKDisconnectCL200A())) {
        gLuxConnected = false;
        Logger(MLLogInfo, "<%s>: Disconnect the illuminometer.\n", __func__);
    } else {
        Logger(MLLogError, "<%s>: Fail disconnecct illuminometer. Errorcode: %d\n", __func__, rtn);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Fail disconnect luxmeter,errorcode: %d.\n", rtn);
    }
}

double GetIlluminanceValue(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return lv;
}

double MLGetIlluminanceValueNT() {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return lv;
}


double GetColorX(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return x;
}

double GetColorY(int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(&lv, &x, &y);
    
    if (lv == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
    
    return y;
}

void GetColorLvxy(float *lv, float *x, float *y, int timeout) {
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvXY(lv, x, y);
    
    if (*lv == 0 || *x == 0 || *y == 0) {
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    }
}

void ConfigAnglePortName(string portName) {
    assert(portName != NULL);
    sprintf(anglePortName, "%s", portName);
    PutStringValue("angle", "portname", portName);
}

void ConfigIlluminometerPortName(string portName) {
    assert(portName != NULL);
    sprintf(illuminometerPortName, "%s", portName);
    PutStringValue("illumionmeter", "portname", portName);
}

void CheckInputSignal() {
    gStopped = false;
    pthread_create(&checkSensorThread, NULL, (void *)CheckSensor, NULL);
}

bool DoorOpen() {
    if (gHandle != -1) {
        Logger(MLLogInfo, "<%s>: Auto-door opening...\n", __func__);
        SetBitState(MLOutDoorOpen, MLLow);
        SetBitState(MLOutSunLight, MLLow);      // 打开日光灯

        if(doorState==Door_Opened) {
            Logger(MLLogInfo, "<%s>: Auto-door has opened\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Auto-door cannot be opened\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Auto-door cannot be opened\n");
        }
    }
    return doorState==Door_Opened;
}

bool DoorClose() {
    if (gHandle != -1) {
        Logger(MLLogInfo, "<%s>: Auto-door closing...\n", __func__);
        SetBitState(MLOutSunLight, MLHigh);             // 关闭日关灯
        SetBitState(MLOutDoorClose, MLLow);             // 关门
        
        if(doorState==Door_Closed) {
            Logger(MLLogInfo, "<%s>: Auto-door has closed\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Auto-door cannot be closed\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Auto-door cannot be closed\n");
        }
    }
    return doorState==Door_Closed;
}

// union API
int CheckSensor() {
    pthread_mutex_t mutex;
    pthread_mutex_init(&mutex, NULL);
    
    if (gHandle != -1) {
        Logger(MLLogInfo, "-- <%s>: Watching input signal...\n", __func__);
        while (!gStopped) {
            int inStates[32];
            for(int bit=0; bit<32; bit++) {
                inStates[bit] =  smc_read_inbit(gHandle, bit);
            }
            
            for (int i = 0; i < 10; i++) {
                if (i<8 && inStates[i] == MLLow) {
                    DWORD errCode;
                    GetAxisErrCode(i+1, &errCode);
                    Logger(MLLogError, "-- <%s>: Servo {%d} alarm. (error code: 0x%lx)\n", __func__, i+1, errCode);
                    memset(errmsg, 0, 256);
                    sprintf(errmsg, "Servo {%d} alarm. (error code: 0x%lx)\n", i+1, errCode);
                    smc_emg_stop(gHandle);
                    gIsTesting = false;
//                    return 1;
                } else {
                    if(CheckAxisIOState(i+1)==0) {  // serve alarm
                        Logger(MLLogError, "-- <%s>: Servo {%d} alarm.\n", __func__, i+1);
                        smc_emg_stop(gHandle);
                        gIsTesting = false;
                    }
                }
            }
            
            // check door
            if( inStates[MLInDoorLeftClosed]==MLLow ) {
                doorState = Door_Closed;
            } else if( inStates[MLInDoorLeftOpened]==MLLow ) {
                doorState = Door_Opened;
            } else if( inStates[MLInDoorLeftOpened]==MLHigh && inStates[MLInDoorLeftClosed]==MLHigh ) {
                doorState = Door_Moving;
            }
            
            // check cylinder
            if (inStates[MLInCylinderShop] == MLLow && inStates[MLInCylinderHome] == MLHigh) {
                luxmeterState = LuxMeter_Shop;
            } else if(inStates[MLInCylinderShop]== MLHigh && inStates[MLInCylinderHome]== MLLow) {
                luxmeterState = LuxMeter_Home;
            } else {
                luxmeterState = LuxMeter_Moving;
            }
            
            // check side door
            if(inStates[MLInSideDoorOpened]==MLHigh) {
                Logger(MLLogWarning, "-- <%s>: Side door opened.\n", __func__);
            }
            
            // check raster
            gIsRaster = (inStates[MLInRaster]==MLHigh);
            if (gIsRaster) {
                if (doorState==Door_Moving) {
                    DWORD outbits = GetOutbits(0);
                    if(((outbits>>MLOutDoorClose)&0x01)==MLLow) {
                        // open door
                        usleep(200*1000);
                        SetBitState(MLOutDoorOpen, MLLow);
                    }
                }
                if (gIsTesting) {
                    Logger(MLLogWarning, "-- <%s>: Grating alarm.\n", __func__);
                }
            }
            
            // check Emg-Stop
            if (inStates[MLInEmgStop] == MLHigh) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "-- <%s>: EMG stop button pressed.\n", __func__);
                    isNeedEmgStop = true;
                    AllAxisStop();
                    gIsTesting = false;
                }
            } else {
                isNeedEmgStop = false;
            }
            
            // check Stop button(red)
            if (inStates[MLInStop] == MLLow) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "-- <%s>: Stop button pressed.\n", __func__);
                    pthread_t thrd;
                    pthread_create(&thrd, NULL, (void*)AllAxisStop, NULL);
                    gIsStopTest = true;
                    gIsTesting = false;
                    isNeedStop = true;
                }
            }else{
                gIsStopTest = false;
            }
            
            // check Reset button(blue)
            if (inStates[MLInReset] == MLLow) {
                if (gIsPowerOn) {
                    Logger(MLLogInfo, "-- <%s>: Reset button pressed.\n", __func__);
                    usleep(200000);
                    gIsOpenDoor = true;
                    gIsTesting = false;
                }
                
                if (doorState==Door_Opened) {
                    pthread_t thrd;
                    pthread_create(&thrd, NULL, (void*)DoorClose, NULL);
                } else {
                    pthread_t thrd;
                    pthread_create(&thrd, NULL, (void*)DoorOpen, NULL);
                }
            }
            
            // check start button(green)
            if (inStates[MLInStart] == MLLow) {
                if (gIsTesting && gIsPowerOn) {
                    Logger(MLLogInfo, "-- <%s>: start button pressed.\n", __func__);
                    gIsTesting = false;
                }
                if (gIsPowerOn) {
                    gIsStartTest = true;
                }
            } else {
                gIsStartTest = false;
            }
            usleep(100000);
        }
        Logger(MLLogInfo, "-- <%s>: Input signal watching thread has ended \n", __func__);
    }
    
    pthread_mutex_destroy(&mutex);
    
    return 0;
}

void CheckEmgStop(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckEmgStopSignal, NULL);
}

void CheckStopActivedSignal(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)IsStopActived, NULL);
}

bool CheckEmgStopSignal() {
    
    return gIsEmgStopTest;
}

void CheckStart(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckStartSignal, NULL);
}

bool CheckStartSignal() {

//    pthread_mutex_t mutex;
//    pthread_mutex_init(&mutex, NULL);
//    while (!gIsStartTest) {
//        usleep(10000);
//    }
//    pthread_mutex_destroy(&mutex);
   return gIsStartTest;
}

void CheckStop(){
    pthread_t thrd;
    pthread_create(&thrd, NULL, (void*)CheckStopSignal, NULL);
}

bool CheckStopSignal() {
//    pthread_mutex_t mutex;
//    pthread_mutex_init(&mutex, NULL);
//    while (!gIsStopTest) {
//        usleep(10000);
//    }
//    pthread_mutex_destroy(&mutex);
    return gIsStopTest;
}

void CheckDoor(){
    if (gIsOpenDoor) {
        SetBitState(MLOutDoorOpen, MLLow);
    }
}

void AdjustAxisPosistion(int direction, int step) {
    int direct = 1;
    switch (direction) {
        case 0: {   // go left
            int axises[] = { MLAxisHolderX};
            direct = 1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 1: {   // go right
            int axises[] = { MLAxisHolderX };
            direct = -1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 2: {   // go front
            int axises[] = { MLAxisHolderY };
            direct = -1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        case 3: {   // go back
            int axises[] = { MLAxisHolderY };
            direct = 1;
            MoveAxisDistance(axises[0], direct * abs(step), false, false);
            CheckAxisState(axises, 2, false);
            break;
        }
        default: break;
    }
    
    gIsTesting = false;
    
}


void ConfigLaserPortName(string portName) {
    assert(portName != NULL);
    sprintf(laserPortName, "%s", portName);
    PutStringValue("laser", "portname", portName);
}

double MLLaserCalibrator(double offset) {
    assert(laserPortName != NULL);
    
    if (gIsTesting) { return -1; }
    
    int axises1[] = {5,6,7,8,9};
    ManyAxisGoBack(axises1, 5, false);
    
    int axises[] = {10};
    ManyAxisGoHome(axises, 1, false);
    
    int times = 0;
    double height1, height2;
    double diff = -1;
    int direct = 1;
    double off = fabs(offset);
    
    SetBitState(MLOutLaserPower,0);
//    SetBitState(MLOutSpotPower,0);
    
    gIsTesting = true;
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    if (gHandle != -1) {
        AxisParam param = gAxisPrm[MLAxisLaser];
        
        if (!ConnectDLRS1A(laserPortName)) {
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
            MoveAxisToPosition(MLAxisLaser, param.posToInit, true, false);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            MoveAxisToPosition(MLAxisLaser, param.posToTest/param.ppratio, true, false);
            
            do {
                height2 = GetLaserDiffOfHeight(laserPortName);
                diff = fabs(height1 - height2);
                direct = (height1 - height2) > 0 ? 1 : -1;
                
                MoveAxisDistance(MLAxisLifter,diff*direct, true, false);
                if (isNeedStop) {return 0;}
                if (isNeedEmgStop) {return 0;}
                res=smc_get_encoder_unit(MyCardNo,4,&Myencoder_value);//读编码值
                times++;
            } while ((diff > off) && (times < 100));
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Not connect the laser device.\n");
        }
        
        MoveAxisToPosition(MLAxisLaser, param.posToTest, true, false);
        int axes[] = {MLAxisLaser};
        CheckAxisState(axes, 1, false);
        
        Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
    }
    
    gIsTesting = false;
    
    return diff;
}

double LaserCalibrator(double offset) {
    assert(laserPortName != NULL);
    
    if (gIsTesting) { return -1; }
    
    int axises1[] = {5,6,7,8,9};
    ManyAxisGoBack(axises1, 5, false);
    
    int axises[] = {10};
    ManyAxisGoHome(axises, 1, false);
    
    int times = 0;
    double height1, height2;
    double diff = -1;
    int direct = 1;
    double off = fabs(offset);
    
    SetBitState(MLOutLaserPower,MLLow);
//    SetBitState(MLOutSpotPower,0);
    
    gIsTesting = true;
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    if (gHandle != -1) {
        AxisParam param = gAxisPrm[MLAxisLaser];
        
        if (!ConnectDLRS1A(laserPortName)) {
            Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
            JMoveAxis(MLAxisLaser, 0, false);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            usleep(500000);
            height1 = GetLaserDiffOfHeight(laserPortName);
            int axes[] = {MLAxisLaser};
            CheckAxisState(axes, 1, false);
            if (isNeedStop) {return 0;}
            if (isNeedEmgStop) {return 0;}
            MoveAxisToPosition(MLAxisLaser, GetLaserDUTPos(), true, false);
            CheckAxisState(axes, 1, false);
            do {
                height2 = GetLaserDiffOfHeight(laserPortName);
                diff = height1-height2;
//                diff = fabs(height1 - height2);
//                direct = (height1 - height2) > 0 ? 1 : -1;
                
                MoveAxisDistance(MLAxisLifter, diff, true, false);
                res=smc_get_encoder_unit(MyCardNo,4,&Myencoder_value);//读编码值
                times++;
            } while ((diff > off) && (times < 5));
            DisconnectDLRS1A();
            Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
        } else {
            Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Not connect the laser device.\n");
        }
        
        SetBitState(MLOutLaserPower,MLHigh);
        
        MoveAxisToPosition(MLAxisLaser, 0, true, false);
        SetBitState(MLOutLightSourcePower, MLLow);

        //        MoveAxisToPosition(MLAxisLaser, 0, true);   // back to init position
        Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
    }
    
    gIsTesting = false;
    
    return diff;
}



void DebugLaserCalibrator(int testCount) {
    assert(laserPortName != NULL);
    
    short rtn = 0;
    int iostate = CheckAxisIOState(9);
    
    AxisParam param = gAxisPrm[9];
    
    rtn |= smc_write_sevon_pin(gHandle, 9, 0);
    sleep(1);
    rtn |= smc_set_profile_unit(gHandle, 9, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
    rtn |= smc_set_s_profile(gHandle, 9,0,0);
    
    if (iostate != 2) {
        
        rtn |= smc_vmove(gHandle, 9, 0);
    }
    while (smc_check_done(gHandle, 9)==0) {
        usleep(200000);
    };
    
    
    for (int i = 0; i < testCount; i++) {
        
        const char *pFileName = "/Users/Shared/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        int times = 0;
        double height1, height2;
        double diff = -1;
        int direct = 1;
        double off = 1;
        SetBitState(MLOutLaserPower, MLLow);
        SetBitState(MLOutSpotPower, MLLow);
        
        gIsTesting = true;
        
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;
        
        if (gHandle != -1) {
            AxisParam param = gAxisPrm[MLAxisLaser];
            
            if (!ConnectDLRS1A(laserPortName)) {
                Logger(MLLogInfo, "<%s>: Start laser calibrate.\n", __func__);
                MoveAxisToPosition(MLAxisLaser, param.posToInit, true, false);
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                usleep(500000);
                height1 = GetDLRS1AM0MeasureValue();
                MoveAxisToPosition(MLAxisLaser, param.posToTest-40, true, false);
                
                do {
                    height2 = GetDLRS1AM0MeasureValue();
                    diff = fabs(height1 - height2);
                    direct = (height1 - height2) > 0 ? 1 : -1;
                    
                    MoveAxisDistance(MLAxisLifter,2*direct, true, false);
                    res=smc_get_encoder_unit(MyCardNo,9,&Myencoder_value);//读编码值
                    times++;
                } while ((diff > off) && (times < 100));
                
                fprintf(pFile,"<%s>: Laser calibration value %f times %d\n", __func__, diff,i);
                
                fclose(pFile);
                DisconnectDLRS1A();
                Logger(MLLogInfo, "<%s>: Laser calibration finished.\n", __func__);
            } else {
                Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
            }
            
            MoveAxisToPosition(MLAxisLaser, param.posToTest, true, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            MoveAxisToPosition(MLAxisLaser, 0, true, false);   // back to init position
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            Logger(MLLogInfo, "<%s>: Laser calibrate done.\n", __func__);
        }
        
    }
    
    gIsTesting = false;
    
}

double GetAngleValue(string portName,char*bufx,char*bufy,char*buft) {
    assert(portName != NULL);
    uint8_t* buff = (uint8_t*)malloc(1024);
    char* buffXoY = (char*)malloc(1024);

    if (!ConnectSCAPort(portName)) {

        //    SendXAxisCom();
        SendXYAxisCom();
        usleep(200*1000);
        
        XYReadExistingHex(buff,(uint8_t*)buffXoY,"@");
        DisconnectSCAPort();
        printf("value:%s\n",buffXoY);
        
        //    XorYBytetoHex(buffXoY,buf);
        XYBytetoHex(buffXoY,bufx,bufy,buft);
        
        Logger(MLLogInfo, "<%s>: angle-x: {%s}, angle-y: {%s}, angle-t: {%s}.\n", __func__, bufx, bufy, buft);
    } else {
        Logger(MLLogError, "<%s>: Not connect the angle device.\n", __func__);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Not connect the angle device.\n");
    }
    
    return 0;
}

double GetAngleDoubleValue(string portName, double *anglex, double *angley, double *tem) {
    char* buffX = (char*)malloc(1024);
    char* buffY = (char*)malloc(1024);
    char* buffTem = (char*)malloc(1024);
    double v = GetAngleValue(portName, buffX, buffY, buffTem);
    
    *anglex = atof(buffX);
    *angley = atof(buffY);
    *tem = atof(buffTem);
    
    if (buffX != NULL) {
        free(buffX);
    }
    
    if (buffY != NULL) {
        free(buffY);
    }
    
    if (buffTem != NULL) {
        free(buffTem);
    }
    
    return v;
}


double GetLaserDiffOfHeight(string portName) {
    assert(portName != NULL);
    
    double height = 0;
    
    if (!ConnectDLRS1A(portName)) {
        height = GetDLRS1AM0MeasureValue();
        DisconnectDLRS1A();
        Logger(MLLogInfo, "<%s>: Laser calibration finished. height: {%lf}\n", __func__, height);
    } else {
        Logger(MLLogError, "<%s>: Not connect the laser device.\n", __func__);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Not connect the laser device.\n");
    }
    
    return height;
}

void ConfigPPRatio(int axis, double ppratio) {
    assert(axis >= 0);
    if (ppratio <= 0) {
        Logger(MLLogWarning, "<%s>: PP ratio is invalid.\n", __func__);
        return;
    }
    AxisParam axisPrm = gAxisPrm[axis];
    axisPrm.ppratio = ppratio;
}

void MLSwingAngleQuick(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (gIsTesting) { return; }
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    gIsTesting = true;
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceQuick(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceQuick(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceQuick(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void SwingAngleQuick(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (gIsTesting) { return; }
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    gIsTesting = true;
    
    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLightSource);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLightSource, 0, false);
        int swingLightAxis[1] = { MLAxisLightSource};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
    int laserState = CheckAxisIOState(MLAxisLaser);
    
    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceQuick(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceQuick(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceQuick(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

bool MoveAxisDistanceAbsolute(int axis, double distance, bool blocked) {
    long currentPos = -1;
    long validPos = distance;
    short rtn = 0;
    gIsTesting = true;
    bool flag = false;
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop move absolute distance.\n", __func__); return 0;}
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return 0; }
    assert(axis >= 0);
    
    if (gHandle != -1) {
        smc_write_sevon_pin(gHandle, axis, 0);
        AxisParam param = gAxisPrm[axis];
        rtn |= smc_set_pulse_outmode(gHandle, axis, MLPluseOutMode);
        rtn |= smc_set_equiv(gHandle, axis, param.equiv);
//        rtn |= smc_set_backlash_unit(gHandle, axis, param.backlash);
        
        if (rtn==0 && CheckAxisParameters(axis, 0)) {
            res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
            currentPos = Myencoder_value;
            validPos += currentPos;
            Logger(MLLogInfo, "<%s>: Axis %d, [Before_Position]: %ld\n", __func__, axis, currentPos);
            
            rtn |= smc_pmove_unit(gHandle, axis, distance*param.ppratio, MLAbsolute);
            
            if (rtn==0) {
                flag = true;
                if (blocked) {
                    while (!smc_check_done(gHandle, axis)) {
                        usleep(200000);     // check axis's state every 200ms
                        if (isNeedStop) { Logger(MLLogInfo, "<%s>: Stop swing angle.\n", __func__); break; }
                        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); break;}
                    }
                    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
                    currentPos = Myencoder_value;
                    Logger(MLLogInfo, "<%s>: Axis %d, [After_Position]: %ld\n", __func__, axis, currentPos);
                } else {
                    Logger(MLLogInfo, "<%s>: Axis %d running unblock mode\n", __func__, axis);
                }
            } else {
                flag = false;
                Logger(MLLogError, "<%s>: Fail to move axis{%d}, rtn: {%d}.\n", __func__, axis, rtn);
                memset(errmsg, 0, 256);
                sprintf(errmsg, "Fail to move axis{%d}, rtn: {%d}.\n", axis, rtn);
            }
        } else {
            Logger(MLLogError, "<%s>: Fail to set axis{%d}'s parameters, rtn: {%d}.\n", __func__, axis, rtn);
            memset(errmsg, 0, 256);
            sprintf(errmsg, "Fail to set axis{%d}'s parameters, rtn: {%d}.\n", axis, rtn);
        }
    }
    gIsTesting = false;
    
    return flag;
}

void MLSwingDegreesOption(double angleStep,double fromAngle, double toAngle, int delay, bool pthread) {
    int swingValue = fabs(fromAngle)+fabs(toAngle);
    MLSwingAngleQuick(0, fromAngle, 0, 1, 1, 1);
    
    for (int i = 0; i < swingValue; i++) {
        MLSwingAngle(0, angleStep, 0, 1, 1, 1, pthread);
        sleep(delay);
        if (isNeedStop) {
            Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__);
            break;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    }
    isNeedStop = false;
    isNeedEmgStop = false;
}

void SwingDegreesOption(double angleStep, double fromAngle,double toAngle) {
    int swingValue = fabs(fromAngle)+fabs(toAngle);
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    SwingAngleQuick(0, fromAngle, 0, 1, 1, 1);
    
    for (int i = 0; i < swingValue; i++) {
        SwingAngle(0, angleStep, 0, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {
            Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__);
            break;
        }
        if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return;}
    }
    isNeedStop = false;
    isNeedEmgStop = false;
}

void MLSwingAngleAbsolute(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR, bool pthread) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return;}
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;
  
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        if (directionX != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotationX, dirX * angleX, false);
        }
        if (directionY != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotationY, dirY * angleY , false);
        }
        if (directionR != 0) {
            MoveAxisDistanceAbsolute(MLAxisRotation, dirR * rotate, false);
        }
        
        CheckAxisState(swingAxis, 3, pthread);
        if (!pthread) {
            Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
        } else {
            Logger(MLLogInfo, "<%s>: Need query state of rotate axis.\n", __func__);
        }
    }
    
    gIsTesting = false;
}

void SwingAngleAbsolute(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    if (gIsTesting) { return; }
    
    gIsTesting = true;
    
    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLightSource);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLightSource, 0, false);
        int swingLightAxis[1] = { MLAxisLightSource};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
    int laserState = CheckAxisIOState(MLAxisLaser);
    
    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistanceAbsolute(MLAxisRotationX, dirX * angleX, false);
        MoveAxisDistanceAbsolute(MLAxisRotationY, dirY * angleY , false);
        MoveAxisDistanceAbsolute(MLAxisRotation, dirR * rotate, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void MLSwingAngle(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR, bool pthread) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;
    
    
    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        if (directionX != 0) {
            MoveAxisDistance(MLAxisRotationX, dirX * angleX, false, false);
        }
        if (directionY != 0) {
            MoveAxisDistance(MLAxisRotationY, dirY * angleY , false, false);
        }
        if (directionR != 0) {
             MoveAxisDistance(MLAxisRotation, dirR * rotate, false, false);
        }
       
        CheckAxisState(swingAxis, 3, pthread);
        
        if (!pthread) {
            Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
        } else {
            Logger(MLLogInfo, "<%s>: Need qurey state of rotate axis.\n", __func__);
        }
    }
    
    gIsTesting = false;
}

void SwingAngle(double angleX, double angleY, double rotate, int directionX, int directionY, int directionR) {
    int dirX = (directionX >= 0) ? 1 : -1;
    int dirY = (directionY >= 0) ? 1 : -1;
    int dirR = (directionR >= 0) ? 1 : -1;
    
    if (isNeedStop) { Logger(MLLogError, "<%s>: Stop button pressed when swing angle.\n", __func__); return; }
    if (isNeedEmgStop) { Logger(MLLogInfo, "<%s>: Emg stop.\n", __func__); return; }
    if (gIsTesting) { return; }
    
    gIsTesting = true;

    SetBitState(MLOutCylinderHome, MLLow);
    SetBitState(MLOutCylinderShop, MLHigh);
    
    int lightState =  CheckAxisIOState(MLAxisLightSource);
    if (lightState != 2) {
        JMoveAxisWithBlock(MLAxisLightSource, 0, false);
        int swingLightAxis[1] = { MLAxisLightSource};
        CheckAxisState(swingLightAxis, 1, false);
    }
    
   int laserState = CheckAxisIOState(MLAxisLaser);

    if (laserState != 2) {
        JMoveAxisWithBlock(MLAxisLaser, 0, false);
        int swingLaserAxis[1] = { MLAxisLaser};
        CheckAxisState(swingLaserAxis, 1, false);
    }

    if (gHandle != -1) {
        int swingAxis[3] = { MLAxisRotationX, MLAxisRotationY, MLAxisRotation };
        MoveAxisDistance(MLAxisRotationX, dirX * angleX, false, false);
        MoveAxisDistance(MLAxisRotationY, dirY * angleY , false, false);
        MoveAxisDistance(MLAxisRotation, dirR * rotate, false, false);
        CheckAxisState(swingAxis, 3, false);
        Logger(MLLogInfo, "<%s>: Success to rotate an angle.\n", __func__);
    }
    
    gIsTesting = false;
}

void Swing90Degrees(double angleValue) {
    int stepCount = floor(90/angleValue);
    for (int i = 0; i < stepCount; i++) {
        SwingAngle(0, angleValue, 0, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
    }
    
    SwingAngleQuick(0, angleValue*stepCount, 0, 1, -1, 1);
    sleep(1);
    if (isNeedStop) {return;}
    if (isNeedEmgStop) {return;}
    for (int i = 0; i < stepCount; i++) {
        SwingAngle(0, angleValue, 0, 1, -1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
    }
    
    SwingAngleQuick(0, angleValue*stepCount, 0, 1, 1, 1);
}

void SwingAngleTest(int dutType, double force1, double force2,double angleV,int testCount) {
    
    for (int i = 0; i < testCount; i ++) {
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 45, 1, 1, -1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        Swing90Degrees(angleV);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        SwingAngle(0, 0, 135, 1, 1, 1);
        sleep(1);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        // release hold.
        int axises[] = {MLAxisHolderX, MLAxisHolderY};
        ManyAxisGoBack(axises,2, false);
        SetBitState(MLOutLightSourcePower, MLHigh);
    }
}

void DebugSwing45Degrees(double angleValue,int testCount) {
    
    for (int i = 0; i < testCount; i++) {
        
        double axisPosition = 0;
        const char *pFileName = "/Users/Shared/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        axisPosition = ReadEncoderValue(MLAxisRotation);
        fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        
        for (int i = 0; i < 2; i++) {
           SwingAngle(0, 0, angleValue, 1, 1, 1);
           if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
           sleep(1);
           axisPosition = ReadEncoderValue(MLAxisRotation);
           fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
        for (int i = 0; i < 6; i++) {
            SwingAngle(0, 0, 45, 1, 1, -1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(1);
            axisPosition = ReadEncoderValue(MLAxisRotation);
            fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
//        SwingAngle(0, 0, 90, 1, 1, -1);
//        sleep(1);
        
        for (int i = 0; i < 4; i++) {
            SwingAngle(0, 0, 45, 1, 1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(1);
            axisPosition = ReadEncoderValue(MLAxisRotation);
            fprintf(pFile,"Current Rotation Position:%f\n",axisPosition);
        }
        
//        SwingAngle(0, 0, 180, 1, 1, 1);
        fclose(pFile);
    }
}

void DebugSwingAngle(double stepAngle, double limitAngle, int testCount) {
    
}

void DebugSwing90Degrees(double angleValue,int testCount) {
    
    for (int i = 0; i < testCount; i++) {
        
        char* buffX = (char*)malloc(1024);
        char* buffY = (char*)malloc(1024);
        char* buffTem = (char*)malloc(1024);
        
        const char *pFileName = "/Users/Shared/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        int stepCount = floor(90/angleValue);
        for (int i = 0; i < stepCount; i++) {
            SwingAngle(0, angleValue, 0, 1, 1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(2);
            GetAngleValue(anglePortName,buffX,buffY,buffTem);
            fprintf(pFile,"buffX:%s buffY:%s\n",buffX,buffY);
        }
        
        SwingAngleQuick(0, angleValue*stepCount, 0, 1, -1, 1);
        sleep(2);
        
        for (int i = 0; i < stepCount; i++) {
            SwingAngle(0, angleValue, 0, 1, -1, 1);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            sleep(2);
            GetAngleValue(anglePortName,buffX,buffY,buffTem);
            fprintf(pFile,"buffX:%s buffY:%s\n",buffX,buffY);
        }
        
        SwingAngleQuick(0, angleValue*stepCount, 0, 1, 1, 1);
        fclose(pFile);
    }
}

void DebugRotationAngleTest(double angleV,int testCount) {
    
    DebugSwing45Degrees(angleV,testCount);
}

void DebugSwingAngleTest(int dutType, double force1, double force2,double angleV,int testCount) {
    
    DebugSwing90Degrees(angleV,testCount);
}

void RotateAngle(double angle) {
    
}

void SwingOneAngle(int direction) {
    
}


void DebugIlluminanceMeasure(int testCount) {
    double measValue = -1;
    int tmCount = 0;
    
    for (int i = 0; i < testCount; i++) {
        SetBitState(MLOutLuxmetePower, MLLow);
        sleep(3);
        
        const char *pFileName = "/Users/Shared/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        if (ConnectIlluminometer(illuminometerPortName)) {
            if (gHandle != -1) {
                short rtn = 0;
                int iostate = CheckAxisIOState(MLAxisLaser);
                gIsTesting = true;
                
                AxisParam param = gAxisPrm[MLAxisLaser];
                
                rtn |= smc_write_sevon_pin(gHandle, MLAxisLaser, 0);
                sleep(1);
                rtn |= smc_set_profile_unit(gHandle, MLAxisLaser, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
                rtn |= smc_set_s_profile(gHandle, MLAxisLaser,0,0);
                rtn |= smc_vmove(gHandle, MLAxisLaser, 0);
                
                while (smc_check_done(gHandle, 9)==0) {
                    usleep(200000);     // check axis's state every 200ms
                };
                
                SetBitState(MLOutSunLight, MLHigh);
                SetBitState(MLOutLaserPower, MLHigh);
                SetBitState(MLOutSpotPower, MLHigh);
                DoorClose();
                while (doorState!=Door_Closed) { usleep(500000); }
                
                // back to orignal position.
                if (!bCalibrated) {
                    calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                    if (iostate == 2) {
                        SetBitState(MLOutSunLight, MLLow);
                        SetBitState(MLOutLaserPower, MLLow);
                        SetBitState(MLOutSpotPower, MLLow);
                    }
                    while (luxmeterState!=LuxMeter_Shop) { usleep(500000); }       // check state every 500ms.
                    calLuxVals->lv = GetIlluminanceValue(5000);
                    Logger(MLLogInfo, "Illuminometer Calibrate value = %lf.\n", calLuxVals->lv);
                }
                
                Logger(MLLogInfo, "Cylinder has ready, waiting to measure intensity of illumination.\n");
                bool flag = false;
                measValue = GetIlluminanceValue(5000);
                double diff = 0;
                do {
                    diff = fabs(GetIlluminanceValue(5000) - measValue);
                    
                    if (diff <= 1) {
                        flag = true;
                        break;
                    }
                } while (tmCount++ <= 3);
                
                if (flag) {
                    measValue = GetIlluminanceValue(5000);
                } else {
                    bCalibrated = flag;
                    measValue = -1;
                }
                
                fprintf(pFile,"Illuminometer calibration value %f times %d\n",diff,i);
                fclose(pFile);
                
                SetBitState(MLOutCylinderHome, MLLow);
            } else {
                Logger(MLLogWarning, "You need connect controller firstly.\n");
            }
        }
        
        DoorOpen();
        //    DisconnectCL200A();
        gIsTesting = false;
        SetBitState(MLOutLuxmetePower, MLHigh);
    }
}

void LightSourceON(){
    SetBitState(MLOutLightSourcePower,0);
}

void LightSourceOFF(){
    SetBitState(MLOutLightSourcePower,1);
}

double GetLuxmeter(string portName) {
    double measValue = -1;
    
    if (gHandle != -1) {
        gIsTesting = true;
        
        SetBitState(MLOutLuxmetePower, MLLow);
        usleep(500 * 1000);
        
        if (!gLuxConnected) {
            sleep(2);
            if (!ConnectIlluminometer(portName)) {
                Logger(MLLogError, "<%s>: Fail to connect the luxmeter, port name: %s.\n", __func__, portName);
                return measValue;
            }
        }
        
        SetBitState(MLOutSunLight, MLHigh);
        SetBitState(MLOutLaserPower, MLHigh);
        SetBitState(MLOutSpotPower, MLHigh);
        
        // move light source axis to negative limit
        JMoveAxis(MLAxisLightSource, 0, false);
        usleep(100*1000);
        int axes1[] = {MLAxisLightSource};
        CheckAxisState(axes1, 1, false);
        int iostate = CheckAxisIOState(MLAxisLightSource);
        if (iostate != 2) {
            Logger(MLLogWarning, "<%s>: axis {%d} didnot move to negative limit.\n", __func__, MLAxisLightSource);
            return measValue;
        }
        
        // move laser axis to negative limit
        JMoveAxis(MLAxisLaser, 0, false);
        usleep(100*1000);
        int axes2[] = {MLAxisLaser};
        CheckAxisState(axes2, 1, false);
        if (CheckAxisIOState(MLAxisLaser) != 2) {
            Logger(MLLogWarning, "<%s>: axis {%d} didnot move to negative limit.\n", __func__, MLAxisLaser);
            return measValue;
        }
        
        // Close auto-door
        DoorClose();
        if (doorState!=Door_Closed) {
            return measValue;
        }
        
        if (!bCalibrated) {
            calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
//            SetBitState(MLOutCylinderHome, MLLow);
//            while (lightTestReadyed) { usleep(500000); }       // check state every 500ms.
            
            if (!SetOutputBitState(MLOutCylinderHome, MLLow)) {
                Logger(MLLogError, "<%s>: Fail to move luxmeter to init poistion.\n", __func__);
                return measValue;
            }
            calLuxVals->lv = GetIlluminanceValue(5000);
            measValue = calLuxVals->lv;
            Logger(MLLogInfo, "<%s>: Get Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            bCalibrated = true;
            Logger(MLLogInfo, "<%s>: Is calibrated: %s.\n", __func__, bCalibrated ? "Yes" : "No");
            sleep(3);
        } else {
            measValue = calLuxVals->lv;
            Logger(MLLogInfo, "<%s>: Use calibration value: %lf.\n", __func__, calLuxVals->lv);
        }
        
        /* shield axis 9 */
//        if (iostate == 2) {     // negative limit
//            SetBitState(MLOutCylinderShop, MLLow);
//        }
        SetBitState(MLOutCylinderShop, MLLow);
        
//        while (!lightTestReadyed) { usleep(500000); }       // check state every 500ms.
        
        if (!SetOutputBitState(MLOutCylinderShop, MLLow)) {
            Logger(MLLogError, "<%s>: Fail to move luxmeter to test poistion.\n", __func__);
            return measValue;
        }
        
        gIsTesting = false;
    } else {
        Logger(MLLogInfo, "<%s>: Please connect controller firstly.\n", __func__);
    }
    
    return measValue;
}

void LuxmeterMeasure(double measureValues[], int size, int timeout) {
    float lv = 0;
    float x = 0;
    float y = 0;
    
//    double *measureValues = malloc(3 * sizeof(double));
    size = size < 3 ? 3 : size;
    memset(measureValues, 0, 3);
    
    JKSetCL200ATimeout(timeout);
    JKGetCL200AEvTcpDeltaUV(&lv, &x, &y);
    
    if (lv == 0) {
        measureValues[0] = 0;
        measureValues[1] = 0;
        measureValues[2] = 0;
        Logger(MLLogInfo, "<%s>: Fail to get illuminance measure data.\n", __func__);
    } else {
        measureValues[0] = lv;
        measureValues[1] = x;
        measureValues[2] = y;
        Logger(MLLogInfo, "<%s>: Luxmeter measure value, Ev: %lf, Tcp: %lf, Δuv: %lf.\n", __func__, lv, x, y);
    }
    
//    return measureValues;
}

bool MoveBackLuxmeter(void) {
    bool flag = true;
    
    gIsTesting = true;
    SetBitState(MLOutLuxmetePower,MLHigh);
//    SetBitState(MLOutCylinderHome, MLLow);
//    while (lightTestReadyed) { usleep(500000); }   // check state every 500ms.
    if (!SetOutputBitState(MLOutCylinderHome, MLLow)) {
        flag = false;
        Logger(MLLogError, "<%s>: Fail to move luxmeter to home poistion.\n", __func__);
    }
    DoorOpen();
//    JKDisconnectCL200A();
    DisconnectIlluminometer();
    Logger(MLLogInfo, "<%s>: Open auto-door and disconnect luxmeter.\n", __func__);
    gIsTesting = false;
    
    return flag;
}

bool LuxMeterON(string portName) {
    double measValue = -1;
    SetBitState(MLOutLuxmetePower,0);
    sleep(2);
    if (MLConnectIlluminometer(portName)) {
        if (gHandle != -1) {
            
            int iostate = CheckAxisIOState(9);
            gIsTesting = true;
            
            SetBitState(MLOutSunLight, MLHigh);
            SetBitState(MLOutLaserPower, MLHigh);
            SetBitState(MLOutSpotPower, MLHigh);
            
            DoorClose();
            if (doorState!=Door_Closed) {
                return -1;
            }
            
            // back to orignal position.
            if (!bCalibrated) {
                calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                if (iostate == 2) {
                    SetBitState(MLOutCylinderShop,0);
                }
                while (luxmeterState!=LuxMeter_Shop) { usleep(500000); }       // check state every 500ms.
                calLuxVals->lv = MLGetIlluminanceValue();
                Logger(MLLogInfo, "<%s>: Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            }
            
            measValue = MLGetIlluminanceValue();
            Logger(MLLogInfo, "<%s>: Cylinder has ready, waiting to measure intensity of illumination.\n", __func__);
        }
    } else {
        Logger(MLLogError, "<%s>: Fail to connect the luxmeter, port name: %s.\n", __func__, portName);
        memset(errmsg, 0, 256);
        sprintf(errmsg, "Fail to connect the luxmeter.\n");
        return -1;
    }
    gIsTesting = false;
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}


double IlluminanceValue() {
    double measValue = -1;
    measValue = MLGetIlluminanceValue();
    Logger(MLLogInfo, "<%s>: Read luxmeter value: {%lf}.\n", __func__, (measValue != -1) ? (measValue - calLuxVals->lv) : measValue);
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}

void LuxMeterOFF() {
    gIsTesting = true;
    SetBitState(MLOutLuxmetePower,1);
    SetBitState(MLOutCylinderHome,0);
    while (luxmeterState!=LuxMeter_Home) { usleep(500000); }   // check state every 500ms.
    DoorOpen();
    JKDisconnectCL200A();
    Logger(MLLogInfo, "<%s>: Open auto-door and disconnect luxmeter.\n", __func__);
    gIsTesting = false;
}

bool MLConnectIlluminometer(string portName) {
    return ConnectIlluminometer(portName);
}

double MLGetIlluminanceValue() {
    return MLGetIlluminanceValueNT();
}

double IlluminanceMeasure(string portName, double offset, int timeout) {
    double measValue = -1;
    int tmCount = 0;
    SetBitState(MLOutLuxmetePower, MLLow);
    sleep(3);
    if (MLConnectIlluminometer(illuminometerPortName)) {
        if (gHandle != -1) {
            short rtn = 0;
            int iostate = CheckAxisIOState(9);
            gIsTesting = true;

            AxisParam param = gAxisPrm[9];

            rtn |= smc_write_sevon_pin(gHandle, 9, 0);
            sleep(1);
            rtn |= smc_set_profile_unit(gHandle, 9, param.startSpeed*param.ppratio, param.runSpeed*param.ppratio, param.accTime, param.accTime, param.stopSpeed);
            rtn |= smc_set_s_profile(gHandle, 9,0,0);
            rtn |= smc_vmove(gHandle, 9, 0);


            SetBitState(MLOutSunLight, MLHigh);
            SetBitState(MLOutLaserPower, MLHigh);
            SetBitState(MLOutSpotPower, MLHigh);
            
            DoorClose();
            if (doorState!=Door_Closed) { return -1; }
            
            // back to orignal position.
            if (!bCalibrated) {
                calLuxVals = (MLLuxMeterVal *)malloc(sizeof(MLLuxMeterVal));
                if (iostate == 2) {
                    SetBitState(MLOutLightSourcePower, MLLow);
                    SetBitState(MLOutCylinderShop, MLLow);
                }
                calLuxVals->lv = MLGetIlluminanceValue();
                Logger(MLLogInfo, "<%s>: Illuminometer Calibrate value = %lf.\n", __func__, calLuxVals->lv);
            }
            
            Logger(MLLogInfo, "<%s>: Cylinder has ready, waiting to measure intensity of illumination.\n", __func__);
            bool flag = false;
            measValue = MLGetIlluminanceValue();
            
            do {
                double diff = fabs(MLGetIlluminanceValue() - measValue);
                
                if (diff <= offset) {
                    flag = true;
                    break;
                }
            } while (tmCount++ <= 3);

            if (flag) {
                measValue = MLGetIlluminanceValue();
            } else {
                bCalibrated = flag;
                measValue = -1;
            }
            
            SetBitState(MLOutCylinderHome, MLLow);
        } else {
            Logger(MLLogWarning, "<%s>: You need connect controller firstly.\n", __func__);
        }
    }
    DoorOpen();
//    DisconnectCL200A();
    gIsTesting = false;
    SetBitState(MLOutLuxmetePower, MLHigh);
    return (measValue != -1) ? (measValue - calLuxVals->lv) : measValue;
}

void AddDUTType(int type, string name, double posHolderX, double posHolderY, double posAxisX,double posAxisY,double posLifter) {
//    int currentDUTTypeCnt = GetCountOfPtr(duts);
    const char *searchname = GetDUTName(type);
    
    if (searchname == NULL) {
        return;
    }
    
    MLDUT duttype;
    duttype.type = type;
    duttype.name = (char *)malloc(128*sizeof(char));
    sprintf(duttype.name, "%s", name);
    
    duttype.posHolderX = posHolderX;
    duttype.posHolderY = posHolderY;
    
    duttype.posAxisX = posAxisX;
    duttype.posAxisY = posAxisY;
    duttype.posLifter = posLifter;
    if (type < MLMaxDUTTypeCount) {
        duts[type] = duttype;
    }
    MLDUT dut = duts[type];
    char *title = (char *)malloc(32);
    char *dutType = (char *)malloc(32);

    sprintf(title, "duttype_%d", dutCount-1);
    sprintf(dutType, "duttype_%d",dut.type);

    int flag = checkTitle(dutType);
    if (flag) {
        sprintf(title, "duttype_%d",dut.type);
        dutCount--;
    }
    
    PutIntValue(title, "type", dut.type);
    PutStringValue(title, "name", dut.name);
    PutDoubleValue(title, "axisHolderX_position",       dut.posHolderX);
    PutDoubleValue(title, "axisHolderY_position",       dut.posHolderY);
    PutDoubleValue(title, "axisX_position",             dut.posAxisX);
    PutDoubleValue(title, "axisY_position",             dut.posAxisY);
    PutDoubleValue(title, "lifter_position",            dut.posLifter);

//    ModifyKeyString(title,dut.name);
    free(title);
    title = NULL;
    dutCount++;
    PutIntValue("syscfg", "dut_type_count", dutCount);
}

double GetAxisPosition(int axis){

    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    AxisParam param = gAxisPrm[axis];
    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
    double position = Myencoder_value/param.ppratio;
    return position;
}

double GetAxisCmdPosition(int axis){

    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    AxisParam param = gAxisPrm[axis];
    res=smc_get_position_unit(MyCardNo,axis,&Myencoder_value);//读编码值
    double position = Myencoder_value/param.ppratio;
    return position;
}

void AddDUTTypeAndName(int type, string name) {
    if (gHandle != -1) {

        AxisParam param4 = gAxisPrm[4];
        AxisParam param5 = gAxisPrm[5];
        AxisParam param6 = gAxisPrm[6];
        AxisParam param7 = gAxisPrm[7];
        AxisParam param8 = gAxisPrm[8];
        AxisParam param11 = gAxisPrm[11];
        AxisParam param12 = gAxisPrm[12];

        
        short res = 0;
        WORD MyCardNo = 0;
        double Myencoder_value=0;

        res=smc_get_encoder_unit(MyCardNo,MLAxisHolderX,&Myencoder_value);//读编码值
        double posHolderX = Myencoder_value/param5.ppratio;
        res=smc_get_encoder_unit(MyCardNo,MLAxisHolderY,&Myencoder_value);//读编码值
        double posHolderY = Myencoder_value/param6.ppratio;

        res=smc_get_encoder_unit(MyCardNo,MLAxisX,&Myencoder_value);//读编码值
        double posAxisX = Myencoder_value/param11.ppratio;
        res=smc_get_encoder_unit(MyCardNo,MLAxisY,&Myencoder_value);//读编码值
        double posAxisY = Myencoder_value/param12.ppratio;
        
        res=smc_get_encoder_unit(MyCardNo,MLAxisLifter,&Myencoder_value);//读编码值
        double posLifter = Myencoder_value/param4.ppratio;
        
        AddDUTType(type, name, posHolderX,posHolderY,posAxisX,posAxisY,posLifter);
    }
}

void LookDUTTypeAndName(int type, string name) {
    if (gHandle != -1) {

        char *title = (char *)malloc(32);
        sprintf(title, "%s",name);

        int axises1[] = {MLAxisHolderX, MLAxisHolderY};
        long posRightHold = GetLongValue(title, "axisHolderX_position");
        MoveAxisToPosition(MLAxisHolderX, posRightHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        long posBackHold = GetLongValue(title, "axisHolderY_position");
        MoveAxisToPosition(MLAxisHolderY, posBackHold,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
        
        
        int axises2[] = {MLAxisX, MLAxisY};
        long posAxisX = GetLongValue(title, "axisX_position");
        MoveAxisToPosition(MLAxisX,posAxisX,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        long posAxisY = GetLongValue(title, "axisY_position");
        MoveAxisToPosition(MLAxisY,posAxisY,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises2, 2, false)) { usleep(100000); }
        
        
        int axises3[] = {MLAxisLifter};
        long posLifter = GetLongValue(title, "lifter_position");
        MoveAxisToPosition(MLAxisLifter,posLifter,false, false);
        if (isNeedStop) {return;}
        if (isNeedEmgStop) {return;}
        while (!CheckAxisState(axises3, 1, false)) { usleep(100000); }
    }
    
}

double ReadEncoderValue(int axis){
    
    AxisParam param = gAxisPrm[axis];
    
    short res = 0;
    WORD MyCardNo = 0;
    double Myencoder_value=0;
    
    res=smc_get_encoder_unit(MyCardNo,axis,&Myencoder_value);//读编码值
    double posValue = Myencoder_value/param.ppratio;
    Logger(MLLogInfo, "<%s>: Read encoder value: {%d}.\n", __func__, posValue);
    
    return posValue;
    
}

void DebugLookDUTTypeAndName(int type, string name) {
    
    if (gHandle != -1) {
        
        const char *pFileName = "/Users/Shared/FlareLog.txt";
        FILE *pFile;
        pFile = fopen(pFileName,"a");
        if (NULL == pFile) {
            printf("error");
        }
        
        for (int i = 0; i < 10; i++) {
            
            char *title = (char *)malloc(32);
            sprintf(title, "%s",name);
            
            int axises1[] = {MLAxisHolderX, MLAxisHolderY};
            long posRightHold = GetLongValue(title, "axisHolderX_position");
            MoveAxisToPosition(MLAxisHolderX, posRightHold,false, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            long posBackHold = GetLongValue(title, "axisHolderY_position");
            MoveAxisToPosition(MLAxisHolderY, posBackHold,false, false);
            if (isNeedStop) {return;}
            if (isNeedEmgStop) {return;}
            while (!CheckAxisState(axises1, 2, false)) { usleep(100000); }
            
            int axises2[] = {MLAxisX, MLAxisY};
            long posAxisX = GetLongValue(title, "axisX_position");
            MoveAxisToPosition(MLAxisX,posAxisX,false, false);
            long posAxisY = GetLongValue(title, "axisY_position");
            MoveAxisToPosition(MLAxisY,posAxisY,false, false);
            while (!CheckAxisState(axises2, 2, false)) { usleep(100000); }
            
            
            int axises3[] = {MLAxisLifter};
            long posLifter = GetLongValue(title, "lifter_position");
            MoveAxisToPosition(MLAxisLifter,posLifter,false, false);
            while (!CheckAxisState(axises3, 1, false)) { usleep(100000); }
            
            fclose(pFile);
            
            int axisesBack[] = {MLAxisHolderX, MLAxisHolderY};
            ManyAxisGoBack(axisesBack,2, false);
        }
    }
}

int GetCountOfDUTType() {
    return dutCount; //GetCountOfPtr(duts);
}

const char* GetDUTName(int type) {
    bool found = false;
    MLDUT *ptr;
    ptr = duts;
    usleep(50000);
    while (ptr->name != NULL) {
         usleep(50000);
        if (ptr->type == type) {
            found = true;
            break;
        }
        ptr++;
    }
    
    if (!found) {
        Logger(MLLogWarning, "<%s>: No DUT which type is %d, You can add it using AddDUTType function.\n", __func__, type);
    }
    
    return found ? ptr->name : "";
}

const char* GetTypeName(int type) {

    char sTitle[64];
    sprintf(sTitle, "duttype_%d", type);
    char *name = (char *)malloc(64);
    GetStringValue(sTitle, "name", name);
    
    return name;
}

bool SetActivedDUT(int type) {
    bool flag = false;
    int count = GetCountOfDUTType();
    
    if (type >= count || type < 0) {
        Logger(MLLogError, "<%s>: No dut which type is %d\n", __func__, type);
        return flag;
    }
    
    if (gHandle != -1) {
        MLDUT dut = duts[type];
        
        int axises[] = {MLAxisX, MLAxisY};
        MoveAxisToPosition(MLAxisLifter, dut.posLifter, true, false);
        MoveAxisToPosition(MLAxisX, dut.posHolderX, false, false);
        MoveAxisToPosition(MLAxisY, dut.posHolderX, false, false);
        while (!CheckAxisState(axises, 2, false)) { usleep(100000); }
        Logger(MLLogInfo, "<%s>: Success to set active DUT\n.", __func__);
        flag = true;
    } else {
        Logger(MLLogError, "<%s>: Connect controller firstly.\n", __func__);
    }
    
    return flag;
}

MLDUT GetDUT(int dutType) {
    MLDUT dut;
    MLDUT *ptr;
    ptr = duts;
    
    while (1) {
        if (ptr->type == dutType) {
            dut = *ptr;
            break;
        }
        ptr++;
    }
    
    return dut;
}
