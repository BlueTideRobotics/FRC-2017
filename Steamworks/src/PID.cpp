/*#ifndef PID_CPP
#define PID_CPP*/
/*
 * PID.cpp
 *
 *  Created on: Feb 15, 2013
 *      Author: Sage
 */

#include "PID.h"

/*
 * Default constructor
 */
PID::PID(double setP,double setI,double setD,double maxEncoderVal,double minEncoderVal)
{
	pConstant=setP;
	iConstant=setI;
	dConstant=setD;
	encoderMax=maxEncoderVal;
	encoderMin=minEncoderVal;
	oldEncoder=0;
	oldTime=0;
	enabled=true;
	maxChange=0.01;
}

/*
 * Default destructor
 */
PID::~PID()
{
	
}
void PID::SetVal(double setP,double setI,double setD,double maxEncoderVal,double minEncoderVal)
{
	pConstant=setP;
	iConstant=setI;
	dConstant=setD;
	encoderMax=maxEncoderVal;
	encoderMin=minEncoderVal;
}
void PID::SetSetpoint(double setpointSet)
{
	setPoint=PID::Scale(setpointSet);
	if(setPoint+maxChange>setpointSet||setPoint-maxChange<setpointSet)
		startNewPID=true;
}
double PID::Scale(double value)
{
	return((value+fabs(encoderMin))/((encoderMax+fabs(encoderMin))/2))-1;
}
void PID::SetEpsilon(double value)//sets how accurate you want the PID to be
{
	epsilon=value;
}
double PID::GetOutput(double encoderVal,double *Pval,double *Ival,double *Dval)
{
	if(enabled)
	{
		double ret;
		if(startNewPID)
		{
			encoderTimeDif.Stop();
			encoderTimeDif.Reset();
			encoderTimeDif.Start();
			oldTime=0;
			I=0.0;
		}
		encoder=PID::Scale(encoderVal);
		error=setPoint-encoder;
		P=error;
		if(fabs(error)>epsilon)
			I=I+((error)*(encoderTimeDif.Get()-oldTime));
		D=(error-oldError)/(encoderTimeDif.Get()-oldTime);
		ret=(pConstant*P)+(iConstant*I)+(dConstant*D);
		oldTime=encoderTimeDif.Get();
		oldError=error;
		oldEncoder=encoder;
		if(Pval&&Ival&&Dval)
		{
			*Pval=P;
			*Ival=I;
			*Dval=D;
		}
		if(ret>1) ret=1;
		if(ret<-1) ret=-1;
		return ret;
	}
	else
		return 0;
		
}
void PID::ResetPID()
{
	PID::SetVal(0,0,0,encoderMax,encoderMin);
}
void PID::SetPID(double setP,double setI,double setD)
{
	pConstant=setP;
	iConstant=setI;
	dConstant=setD;
}
void PID::Enable()
{
	enabled=true;
}
void PID::Disable()
{
	enabled=false;
}
void PID::ResetI()
{
	I=0;
}
void PID::SetMaxChangeSetpoint(double maxChangeSet)
{
	maxChange=maxChangeSet;
}
//#endif
