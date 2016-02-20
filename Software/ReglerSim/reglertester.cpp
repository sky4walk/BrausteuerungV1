#include <stdio.h>
#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif

///////////////////////////////////////////////////////////////////////////////

unsigned long millis()
{
#ifdef WIN32
	return ( unsigned long ) GetTickCount();
#else
	struct timeval tp;
    gettimeofday(&tp, NULL);
    return ( unsigned long ) tp.tv_sec * 1000L + tp.tv_usec / 1000;	
#endif
}

///////////////////////////////////////////////////////////////////////////////

class HeatingSystem
{
public:
	HeatingSystem()
	{
	}
	void Init(
		unsigned long heatTime,
		float dDegreeUp, 
		unsigned long coolTime, 
		float dDegreeDown,
		float tNachlauf,
		float tVorlauf,
		float startTemp)
	{
		mHeatTime    = heatTime;
		mDDegreeUp   = dDegreeUp;
		mCoolTime    = coolTime;
		mDDegreeDown = dDegreeDown;
		mNachlaufTemp = tNachlauf;
		mVorlaufTemp = tVorlauf;
		mHeaterState = false;
		mStartTemp	 = startTemp;
		mStateChangedTime = millis();
		
		mNachlaufTime = (unsigned long)(tNachlauf * heatTime / dDegreeUp);
		mVorlaufTime  = (unsigned long)(tVorlauf  * coolTime / dDegreeDown);
		
		mStartTemp	  -= getRelTempLinear(-mDDegreeDown,mCoolTime,mVorlaufTime);
                mActStateTemp      = mStartTemp;
		mStateChangedTime -= mNachlaufTime;
	}
	void setHeater(bool onOff)
	{
          if ( mHeaterState != onOff )
          {
		    mActStateTemp = getTemperatur();
		    mStateChangedTime = millis();
		    mHeaterState = onOff;
          }
	}
	bool getHeater()
	{
		return mHeaterState;
	}
	float getTemperatur()
	{
		unsigned long dTime = millis() - mStateChangedTime;
		float temp = mActStateTemp;
		
		if ( mHeaterState )
		{
			if ( dTime <= mVorlaufTime )
			{
				temp += getRelTempLinear(-mDDegreeDown,mCoolTime,dTime);
			}
			else
			{
				temp += getRelTempLinear(-mDDegreeDown,mCoolTime,mVorlaufTime);
				temp += getRelTempLinear(mDDegreeUp,mHeatTime,dTime - mVorlaufTime);
			}
		}
		else
		{
			if ( dTime <= mNachlaufTime )
			{
				temp += getRelTempLinear(mDDegreeUp,mHeatTime,dTime);
			}
			else
			{
				temp += getRelTempLinear(mDDegreeUp,mHeatTime,mNachlaufTime);
				temp += getRelTempLinear(-mDDegreeDown,mCoolTime,dTime - mNachlaufTime);
			}
		}
		return temp; //  < mStartTemp ? mStartTemp : temp;
	}
private:
	
	float getRelTempLinear(float degree, unsigned long dTime, unsigned long measureInterval)
	{
		return degree * measureInterval / dTime;
	}
		
	unsigned long mHeatTime;
	unsigned long mCoolTime;
	float mDDegreeUp;
	float mDDegreeDown;
	float mStartTemp;
	float mActStateTemp;
	float mNachlaufTemp;
	float mVorlaufTemp;
	unsigned long mNachlaufTime;
	unsigned long mVorlaufTime;
	bool mHeaterState;
	unsigned long mStateChangedTime;
	float stateChangedTemp;
};

///////////////////////////////////////////////////////////////////////////////

class ZweiPunktRegler
{
public:
	ZweiPunktRegler()
	{
	}
	void Init(float zielTemp, float dMin, float dMax)
	{
		mZielTemp = zielTemp;
		mDMin = dMin;
		mDMax = dMax;
	}
	bool heating(float actTemp, bool heatOnOff)
	{
		if ( heatOnOff )
		{
			if ( actTemp > ( mZielTemp + mDMax ) )
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			if ( actTemp < ( mZielTemp + mDMin ) )
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
private:
	float mZielTemp;
	float mDMin;
	float mDMax;
};
///////////////////////////////////////////////////////////////////////////////

class PID_Regler
{
public:
	PID_Regler()
	{
	}
	void Init(
		double zielTemp, 
		double outMax,
		double outMin,
		unsigned long sampleTime)
	{
		mZielTemp = zielTemp;
		mMin = outMin;
		mMax = outMax;
		mSampleTime = sampleTime;
		mLastTime = millis() - mSampleTime;
		mLastInput = 0;
	}
	void setTuning(double Kp, double Ki, double Kd)
	{
		double sampleTimeSec = mSampleTime / 1000.0;
		mKp = Kp;
		mKi = Ki * sampleTimeSec;
		mKd = Kd / sampleTimeSec;
	}
	bool Compute(double input)
	{
		unsigned long now = millis();
		unsigned long timeChange = (now - mLastTime);
		if( timeChange >= mSampleTime )
		{
			// P Anteil: schnell
			double error = mZielTemp - input;
			// I Anteil: genau
			ITerm += (mKi * error) * timeChange;
			// D Anteil: daempfung ueberschwinger
			double dInput = (input - mLastInput) / timeChange;
	 
			/*Compute PID Output*/
			mOutput = mKp * error + ITerm - mKd * dInput;
		  
		        if     (mOutput > mMax) mOutput = mMax;
                        else if(mOutput < mMin) mOutput = mMin;
			
			/*Remember some variables for next time*/
			mLastInput = input;
			mLastTime  = now;
			
			printf("output: %f\n",mOutput);

			return true;
		}
		else 
			return false;
	}
	double getOutput()
	{
		return mOutput;
	}
	double getSetPoint()
	{
		return mZielTemp;
	}
private:
	double mZielTemp;
	double mKp;
	double mKi;
	double mKd;
	double mMax;
	double mMin;
	double ITerm;
	unsigned long  mLastTime;
	unsigned long  mSampleTime;
	double mLastInput;
	double mOutput;
};

///////////////////////////////////////////////////////////////////////////////

HeatingSystem kocher;
ZweiPunktRegler regler;
PID_Regler pid;
unsigned long StartTimeMs;
unsigned long systemStart;

bool heatingControllerPID (
	double setPoint,
	double temperatur,
	double deltaPid,
	unsigned long windowSize,
	unsigned long* startTime)
{
    double output;
    if ( (setPoint - temperatur) < deltaPid * 1.2 ) 
    {
	// getting close, start feeding the PID
	pid.Compute(temperatur);
	output = pid.getOutput();
    }
    if ( (setPoint - temperatur) > deltaPid )
    {
        // ignore PID and go full speed
	output = 100;    
    }

    // PWM the output
    unsigned long now = millis();
    unsigned long delta = now - *startTime;
    if ( delta > windowSize ) 
    {
      *startTime = now; 
    }

    if ( (output * (windowSize / 100) ) > delta )
    {  
      return true;
    }
    else
    { 
      return false;
    }
}

void setup()
{
	printf("setup\n");
	kocher.Init(1000,1.0f,1000,1.0f, 1.0f,1.0f, 16.0f);
//	regler.Init(50.0f,-0.2f,0.2f);
	pid.Init(50.0,0.0,100.0,500);
	pid.setTuning(100.0 , 20.0, 5.0);
	StartTimeMs = 0;
	kocher.getTemperatur();
	kocher.setHeater(false);
	systemStart = millis();
}

void loop()
{
	unsigned long actTime = (unsigned long)millis()-systemStart;
	printf("actTime:\t%lu\t",actTime);
	
	/*
	if ( actTime - startTime > 500 )
	{
		startTime = actTime;
		float temp = kocher.getTemperatur();
		bool heat = regler.heating(temp,kocher.getHeater());
		kocher.setHeater(heat);
	}
	*/

	float temp = kocher.getTemperatur();
	bool heat =  heatingControllerPID (
				pid.getSetPoint(), 
				temp,
				5.0,
				100.0,
				&StartTimeMs);


	if ( actTime > 1000) { kocher.setHeater(false); systemStart = millis();}
	else kocher.setHeater(true);

	printf("T:\t%f \t H: %s \n",temp, kocher.getHeater() ? "On" : "Off" );		

}

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{
        printf("main\n");
	setup();
	while(1) 
	{ 
		loop();
#ifdef WIN32		
		Sleep(1);
#else
		
		usleep(10000);
#endif		
	}
	return 0;
}
