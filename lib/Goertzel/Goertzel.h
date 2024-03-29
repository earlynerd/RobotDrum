/*
  The Goertzel algorithm is long standing so see 
  http://en.wikipedia.org/wiki/Goertzel_algorithm for a full description.
  It is often used in DTMF tone detection as an alternative to the Fast 
  Fourier Transform because it is quick with low overheard because it
  is only searching for a single frequency rather than showing the 
  occurrence of all frequencies.
  
  This work is entirely based on the Kevin Banks code found at
  http://www.eetimes.com/design/embedded/4024443/The-Goertzel-Algorithm 
  so full credit to him for his generic implementation and breakdown. I've
  simply massaged it into an Arduino library. I recommend reading his article
  for a full description of whats going on behind the scenes.

  See Contributors.md and add yourself for pull requests
  Released into the public domain.
*/

// ensure this library description is only included once
#ifndef Goertzel_h
#define Goertzel_h

// include types & constants of Wiring core API
#include "Arduino.h"

#define MAXN 2048
#define ADCCENTER 512

// library interface description
class Goertzel
{
  // user-accessible "public" interface
  public:
    Goertzel(float,float,float);
    Goertzel(float,float);
	void sample(int32_t* buf, int32_t len);
	float detect(float *realPart, float *imagPart, double *mag2);

  // library-accessible "private" interface
  private:
  float Q0;
  float Q1;
  float Q2;
  float coeff;
  float sine;
  float cosine;
  float _SAMPLING_FREQUENCY;
  float _TARGET_FREQUENCY;
  int underscoreN;
  int32_t testData[MAXN];
	void GetRealImag(float*,float*);
	void ProcessSample(int);
	void ResetGoertzel(void);
	
};

#endif

