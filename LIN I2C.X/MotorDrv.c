

#include <HardwareProfile.h>
#include <plib.h>
#include <MotorDrv.h>



void MotorDrv_DIS(void)
{

EN1=0;  // motor L-R disable
EN2=0;
SetDCOC1PWM(0) ;//L_A
SetDCOC2PWM(0) ;//L_B
SetDCOC3PWM(0) ;//R_A
SetDCOC4PWM(0) ;//R_B


}

void MotorBreak_L(void)
{
EN1=1;  // motor L disable
SetDCOC1PWM(0) ;//L_A
SetDCOC2PWM(0) ;//L_B
}
void MotorBreak_R(void)
{
EN2=1;  // motor L disable
SetDCOC3PWM(0) ;//L_A
SetDCOC4PWM(0) ;//L_B
}


void MotorDrvInit(void)
{
EN1_TRIS=0;
EN2_TRIS=0;
MotorDrv_DIS();
TankPWMInit();



}

void MotorDrv_L(INT16 pwm)
{
EN1=1;

	if (pwm>0)
	{
		if (pwm>pwmlimit)pwm=pwmlimit;

		SetPWM2(pwm) ;//L_A
		SetPWM1_OFF() ;//L_B
	}
	else
	{
		pwm=abs(pwm);
		if (pwm>pwmlimit)pwm=pwmlimit;
		SetPWM2_OFF() ;//L_A
		SetPWM1(pwm) ;//L_B
	}
//	PWM_R=pwm;

}


void MotorDrv_R(INT16 pwm)
{
EN2=1;

	if (pwm>0)
	{
		if (pwm>pwmlimit)pwm=pwmlimit;

		SetPWM3(pwm) ;//L_A
		SetPWM4_OFF() ;//L_B
	}
	else
	{
		pwm=abs(pwm);
		if (pwm>pwmlimit)pwm=pwmlimit;
		SetPWM3_OFF() ;//L_A
		SetPWM4(pwm) ;//L_B
	}
//	PWM_L=pwm;


}
