#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

#define MINIMUM_RSSI -80

#define RADAR_BOOTING -16000
#define CONNECTION_LOST -4000000
#define RADAR_RSSI_TOO_LOW -3000000
#define RADAR_UNINITIALIZED -2000000

#define MAX_SAMPLEBUFFERSIZE 256
int * sampleBuffer;
int sampleBufferSize = 64;
int sampleBufferIndex = 0;
int sampleBufferValid = 0;

#define MAX_AVERAGEBUFFERSIZE 64
int mobileAverageFilterSize = 16;
int mobileAverageBufferSize = MAX_AVERAGEBUFFERSIZE;
int mobileAverage = 0;
int mobileAverageTemp = 0;
int * mobileAverageBuffer;
int mobileAverageBufferIndex = 0;

int bufferIndex = 0; 

#define MAX_VARIANCE 65535

int motionSensorLevel = RADAR_BOOTING; 
int presenceSensorLevel = RADAR_BOOTING; 

int varianceSample = 0; 

int varianceIntegral = 0; 

int varianceIntegratorLimitMax = MAX_SAMPLEBUFFERSIZE;
int varianceIntegratorLimit = 3;
int varianceBufferSize = MAX_SAMPLEBUFFERSIZE;
int *varianceBuffer; 
int varianceBufferIndex = 0;

int MeasuredPower = 0;
int presenceNormal = 0;
bool presenceInit = false;

// --- Service ---

int* MemoryInit(int* address, int size) 
{
  if (address == NULL) 
  {
    address = (int*)malloc(sizeof(int) * size); 
    for (int i = 0; i < size; i++) 
    {   
      address[i] = 0x00;
    }
  }

  return address;
}

int CheckErrors(int RSSILevel){
  if (RSSILevel == 0)
    return CONNECTION_LOST;

  if ((sampleBuffer == NULL) || (mobileAverageBuffer == NULL) || (varianceBuffer == NULL)) 
    return RADAR_UNINITIALIZED;

  if (RSSILevel < MINIMUM_RSSI) 
    return RADAR_RSSI_TOO_LOW;

  return 1;
}

void Setup(){
  sampleBuffer = MemoryInit(sampleBuffer, sampleBufferSize);  
  mobileAverageBuffer = MemoryInit(mobileAverageBuffer, mobileAverageBufferSize);  
  varianceBuffer = MemoryInit(varianceBuffer, varianceBufferSize);  

  varianceBufferSize = sampleBufferSize;
}

// --- Init ---

bool MPS_StartWiFiConnection(const char* SSID, const char* password) 
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    delay(5000);
    ESP.restart();
  }
  
  return true;
}

void MPS_Setup(int currnetMeasuredPower) {
  Setup();

  if (currnetMeasuredPower < 0)
    MeasuredPower = currnetMeasuredPower;
  else
    MeasuredPower = 1000;
}

void MPS_Setup() {
  Setup();

  MeasuredPower = 1000;
}

// --- Distance ---

int MPS_RSSI() { return (int)WiFi.RSSI(); }

float filtValbegushie_srednee = 0;
float MPS_GetDistance()
{ 
  if (MeasuredPower == 1000)
    return -1;
    
  float newVal = pow(10, (MeasuredPower - MPS_RSSI()) * 0.05);
  filtValbegushie_srednee += (newVal - filtValbegushie_srednee) * 0.1;
  return filtValbegushie_srednee; 
}

// --- Main ---

int MPS_PresenceSensorLevel()
{
  int RSSIlevel = MPS_RSSI();
  
  int error = CheckErrors(RSSIlevel);
  if (error < 0)
    return error;

  RSSIlevel = abs(RSSIlevel);

  int diffrence = abs(presenceNormal - RSSIlevel);
  return diffrence*diffrence;
}

int MPS_MotionSensorLevel() 
{ 
  int RSSIlevel = MPS_RSSI();
  
  int error = CheckErrors(RSSIlevel);
  if (error < 0)
    return error;

  sampleBuffer[sampleBufferIndex] = RSSIlevel;
  sampleBufferIndex++;

  if (sampleBufferIndex >= sampleBufferSize) 
  {
    sampleBufferIndex = 0;
    sampleBufferValid = 1;
  }
  
  if (sampleBufferValid >= 1) 
  {
    mobileAverageTemp = 0;
    int mobilePointer = 0;

    for (int mobileAverageSampleIndex = 0; mobileAverageSampleIndex < mobileAverageFilterSize; mobileAverageSampleIndex++) 
    {
      mobilePointer = sampleBufferIndex - mobileAverageSampleIndex;

      if (mobilePointer <= 0) 
      {
        mobilePointer = mobilePointer + (sampleBufferSize - 1);
      }

      mobileAverageTemp += sampleBuffer[mobilePointer];
    }

    mobileAverage = mobileAverageTemp / mobileAverageFilterSize;
    mobileAverageBuffer[mobileAverageBufferIndex] = mobileAverage;
    
    if (!presenceInit)
    {
      presenceInit = true;
      presenceNormal = abs(mobileAverage);
    }

    varianceSample = (RSSIlevel - mobileAverageBuffer[mobileAverageBufferIndex])*(RSSIlevel - mobileAverageBuffer[mobileAverageBufferIndex]);
    varianceBuffer[varianceBufferIndex] = varianceSample;
    
    varianceIntegral = 0;
    int variancePointer = 0;

    for (int varianceBufferIndexTemp = 0; varianceBufferIndexTemp < varianceIntegratorLimit; varianceBufferIndexTemp++) 
    {
      variancePointer = varianceBufferIndex - varianceBufferIndexTemp;

      if (variancePointer <= 0) 
      {
        variancePointer = variancePointer + (varianceBufferSize -1);
      }

      varianceIntegral = varianceIntegral + varianceBuffer[variancePointer];
    }

    varianceBufferIndex++;
    if (varianceBufferIndex >= varianceBufferSize) 
    { 
      varianceBufferIndex = 0;
    }
    
    motionSensorLevel = (varianceIntegral + motionSensorLevel) / 2; 

    mobileAverageBufferIndex++;
    if ( mobileAverageBufferIndex >= mobileAverageBufferSize )
    {
      mobileAverageBufferIndex = 0;
    }
  }
  else
  {
    return -(sampleBufferSize - sampleBufferIndex);
  }

  return motionSensorLevel;
}