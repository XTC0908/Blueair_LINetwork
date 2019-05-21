
#include <p32xxxx.h>
#include "GenericTypeDefs.h"

UINT16 pwmlimit=1024;

void MotorDrvInit(void);

void MotorDrv_DIS(void);
void MotorBreak_L(void);
void MotorBreak_R(void);
void MotorDrv_L(INT16);
void MotorDrv_R(INT16);
extern UINT8 PWM_L,PWM_R;