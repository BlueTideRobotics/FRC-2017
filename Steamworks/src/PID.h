#ifndef PID_H
#define PID_H

/*
 * PID.h
 *
 *  Created on: Feb 15, 2013
 *      Author: Sage
 */
#include "WPILib.h"
#include "math.h"
//#include "include_Functions.cpp"
/*
 * Description:
 */
class PID
{
	double P,I,D;
	double pConstant,iConstant,dConstant,encoderMax,encoderMin,setpoint,encoder,error,epsilon,setPoint,oldError;
	double maxChange;
	double oldEncoder;
	Timer encoderTimeDif;
	double oldTime;
	bool startNewPID,enabled;
public:
	PID(double setP,double setI,double setD,double maxEncoderVal,double minEncoderVal);
	~PID();
	void SetSetpoint(double setpointSet);
	double GetOutput(double encoderVal,double *Pval=NULL,double *Ival=NULL,double *Dval=NULL);
	void SetVal(double setP,double setI,double setD,double maxEncoderVal,double minEncoderVal);
	void SetEpsilon(double value);
	void ResetPID();
	void SetPID(double setP,double setI,double setD);
	void Enable();
	void Disable();
	void ResetI();
	void SetMaxChangeSetpoint(double maxChangeSet);
private:
	double Scale(double value);
};
#endif

