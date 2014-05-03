#include <SoftPWMServo.h>
//#include <SoftPWM.h>
//#include <avr/pgmspace.h>

/* fix_fft.c - Fixed-point in-place Fast Fourier Transform */
/*
All data are fixed-point short integers, in which -32768
 to +32768 represent -1.0 to +1.0 respectively. Integer
 arithmetic is used for speed, instead of the more natural
 floating-point.
 
 Written by: Tom Roberts 11/8/89
 Made portable: Malcolm Slaney 12/15/94 malcolm@interval.com
 Enhanced: Dimitrios P. Bouras 14 Jun 2006 dbouras@ieee.org
 Adapted for Arduino: Anatoly Kuzmenko 6 Feb 2011 k_anatoly@hotmail.com
 Adapted for ChipKit by Mason, Jared and Kevin Dec 2012
 */

//**************************************************************************************************
#define N_WAVE 1024 /* full length of Sinewave[] */
#define LOG2_N_WAVE 10 /* log2(N_WAVE) */

#define FFT_SIZE 128
#define log2FFT 7
#define N (2 * FFT_SIZE)
#define log2N (log2FFT + 1)

int debug_state = 0;
float ledLevel1 = 0;
int AledLevel[] = { 
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // used in test sub
int ledLevel = 250; // used in test sub

char command[0x20];

////////////////////////////////////////////////////////////////
// Analog Input
////////////////////////////////////////////////////////////////
int AnalogInput = A14;

unsigned int micros_delay = 50; 
// micros_delay values for 80MHz PIC32
//  0 = 180KHz
// 10 = 65KHz
// 20 = 37.5KHz
// 30 = 27.5KHz
// 40 = 20KHz

////////////////////////////////////////////////////////////////
// The led[] array defines the 16 output pins used for the PWM
////////////////////////////////////////////////////////////////
//Originally, only pins capable of PWM throughput were
//3,5,6,9
//int led[] = { A1,A2,A3,A4,13,29,30,31,32,33,36,37,38,39,40,41};
//int led[] = { 32,33,31,35,38,36,34,37,16,15,27,17,30,26,29,28}; // Changed by Rod
//int led[] = { 86,64,5,70,22,76,9,2,23,39,8,21,78,79,10,20}; // Quick Drive Kard Outputs (2-5) (Kard2 has issue with SoftPWM maybe is uses the clock)
//int led[] = { 57,56,63,54,22,76,9,2,23,39,8,21,78,79,10,20 }; // Quick Drive Kard Outputs (2-5)
int led[] = { 20,78,21,57,8,79,10,63,76,39,2,56,54,9,22,23 }; // Quick Drive Ping Pong Ball Order
//possible pins -- A1, A2, A3, A4, 13, 29, 30, 31, 32, 33
// 34, 35, 36, 37, 38, 39, 40, 41

const int Sinewave[N_WAVE-N_WAVE/4] PROGMEM = {
  0, 201, 402, 603, 804, 1005, 1206, 1406,
  1607, 1808, 2009, 2209, 2410, 2610, 2811, 3011,
  3211, 3411, 3611, 3811, 4011, 4210, 4409, 4608,
  4807, 5006, 5205, 5403, 5601, 5799, 5997, 6195,
  6392, 6589, 6786, 6982, 7179, 7375, 7571, 7766,
  7961, 8156, 8351, 8545, 8739, 8932, 9126, 9319,
  9511, 9703, 9895, 10087, 10278, 10469, 10659, 10849,
  11038, 11227, 11416, 11604, 11792, 11980, 12166, 12353,
  12539, 12724, 12909, 13094, 13278, 13462, 13645, 13827,
  14009, 14191, 14372, 14552, 14732, 14911, 15090, 15268,
  15446, 15623, 15799, 15975, 16150, 16325, 16499, 16672,
  16845, 17017, 17189, 17360, 17530, 17699, 17868, 18036,
  18204, 18371, 18537, 18702, 18867, 19031, 19194, 19357,
  19519, 19680, 19840, 20000, 20159, 20317, 20474, 20631,
  20787, 20942, 21096, 21249, 21402, 21554, 21705, 21855,
  22004, 22153, 22301, 22448, 22594, 22739, 22883, 23027,
  23169, 23311, 23452, 23592, 23731, 23869, 24006, 24143,
  24278, 24413, 24546, 24679, 24811, 24942, 25072, 25201,
  25329, 25456, 25582, 25707, 25831, 25954, 26077, 26198,
  26318, 26437, 26556, 26673, 26789, 26905, 27019, 27132,
  27244, 27355, 27466, 27575, 27683, 27790, 27896, 28001,
  28105, 28208, 28309, 28410, 28510, 28608, 28706, 28802,
  28897, 28992, 29085, 29177, 29268, 29358, 29446, 29534,
  29621, 29706, 29790, 29873, 29955, 30036, 30116, 30195,
  30272, 30349, 30424, 30498, 30571, 30643, 30713, 30783,
  30851, 30918, 30984, 31049, 31113, 31175, 31236, 31297,
  31356, 31413, 31470, 31525, 31580, 31633, 31684, 31735,
  31785, 31833, 31880, 31926, 31970, 32014, 32056, 32097,
  32137, 32176, 32213, 32249, 32284, 32318, 32350, 32382,
  32412, 32441, 32468, 32495, 32520, 32544, 32567, 32588,
  32609, 32628, 32646, 32662, 32678, 32692, 32705, 32717,
  32727, 32736, 32744, 32751, 32757, 32761, 32764, 32766,
  32767, 32766, 32764, 32761, 32757, 32751, 32744, 32736,
  32727, 32717, 32705, 32692, 32678, 32662, 32646, 32628,
  32609, 32588, 32567, 32544, 32520, 32495, 32468, 32441,
  32412, 32382, 32350, 32318, 32284, 32249, 32213, 32176,
  32137, 32097, 32056, 32014, 31970, 31926, 31880, 31833,
  31785, 31735, 31684, 31633, 31580, 31525, 31470, 31413,
  31356, 31297, 31236, 31175, 31113, 31049, 30984, 30918,
  30851, 30783, 30713, 30643, 30571, 30498, 30424, 30349,
  30272, 30195, 30116, 30036, 29955, 29873, 29790, 29706,
  29621, 29534, 29446, 29358, 29268, 29177, 29085, 28992,
  28897, 28802, 28706, 28608, 28510, 28410, 28309, 28208,
  28105, 28001, 27896, 27790, 27683, 27575, 27466, 27355,
  27244, 27132, 27019, 26905, 26789, 26673, 26556, 26437,
  26318, 26198, 26077, 25954, 25831, 25707, 25582, 25456,
  25329, 25201, 25072, 24942, 24811, 24679, 24546, 24413,
  24278, 24143, 24006, 23869, 23731, 23592, 23452, 23311,
  23169, 23027, 22883, 22739, 22594, 22448, 22301, 22153,
  22004, 21855, 21705, 21554, 21402, 21249, 21096, 20942,
  20787, 20631, 20474, 20317, 20159, 20000, 19840, 19680,
  19519, 19357, 19194, 19031, 18867, 18702, 18537, 18371,
  18204, 18036, 17868, 17699, 17530, 17360, 17189, 17017,
  16845, 16672, 16499, 16325, 16150, 15975, 15799, 15623,
  15446, 15268, 15090, 14911, 14732, 14552, 14372, 14191,
  14009, 13827, 13645, 13462, 13278, 13094, 12909, 12724,
  12539, 12353, 12166, 11980, 11792, 11604, 11416, 11227,
  11038, 10849, 10659, 10469, 10278, 10087, 9895, 9703,
  9511, 9319, 9126, 8932, 8739, 8545, 8351, 8156,
  7961, 7766, 7571, 7375, 7179, 6982, 6786, 6589,
  6392, 6195, 5997, 5799, 5601, 5403, 5205, 5006,
  4807, 4608, 4409, 4210, 4011, 3811, 3611, 3411,
  3211, 3011, 2811, 2610, 2410, 2209, 2009, 1808,
  1607, 1406, 1206, 1005, 804, 603, 402, 201,
  0, -201, -402, -603, -804, -1005, -1206, -1406,
  -1607, -1808, -2009, -2209, -2410, -2610, -2811, -3011,
  -3211, -3411, -3611, -3811, -4011, -4210, -4409, -4608,
  -4807, -5006, -5205, -5403, -5601, -5799, -5997, -6195,
  -6392, -6589, -6786, -6982, -7179, -7375, -7571, -7766,
  -7961, -8156, -8351, -8545, -8739, -8932, -9126, -9319,
  -9511, -9703, -9895, -10087, -10278, -10469, -10659, -10849,
  -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
  -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827,
  -14009, -14191, -14372, -14552, -14732, -14911, -15090, -15268,
  -15446, -15623, -15799, -15975, -16150, -16325, -16499, -16672,
  -16845, -17017, -17189, -17360, -17530, -17699, -17868, -18036,
  -18204, -18371, -18537, -18702, -18867, -19031, -19194, -19357,
  -19519, -19680, -19840, -20000, -20159, -20317, -20474, -20631,
  -20787, -20942, -21096, -21249, -21402, -21554, -21705, -21855,
  -22004, -22153, -22301, -22448, -22594, -22739, -22883, -23027,
  -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
  -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201,
  -25329, -25456, -25582, -25707, -25831, -25954, -26077, -26198,
  -26318, -26437, -26556, -26673, -26789, -26905, -27019, -27132,
  -27244, -27355, -27466, -27575, -27683, -27790, -27896, -28001,
  -28105, -28208, -28309, -28410, -28510, -28608, -28706, -28802,
  -28897, -28992, -29085, -29177, -29268, -29358, -29446, -29534,
  -29621, -29706, -29790, -29873, -29955, -30036, -30116, -30195,
  -30272, -30349, -30424, -30498, -30571, -30643, -30713, -30783,
  -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
  -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735,
  -31785, -31833, -31880, -31926, -31970, -32014, -32056, -32097,
  -32137, -32176, -32213, -32249, -32284, -32318, -32350, -32382,
  -32412, -32441, -32468, -32495, -32520, -32544, -32567, -32588,
  -32609, -32628, -32646, -32662, -32678, -32692, -32705, -32717,
  -32727, -32736, -32744, -32751, -32757, -32761, -32764, -32766
};

int fix_fft(int fr[], int fi[], int m )
{
  int mr, nn, i, j, l, k, istep, n, scale, shift;
  int qr, qi, tr, ti, wr, wi;

  n = 1 << m;

  /* max FFT size = N_WAVE */
  if (n > N_WAVE)
    return -1;

  mr = 0;
  nn = n - 1;
  scale = 0;

  /* decimation in time - re-order data */
  for (m=1; m<=nn; ++m) {
    l = n;
    do {
      l >>= 1;
    }
    while (mr+l > nn);
    mr = (mr & (l-1)) + l;

    if (mr <= m)
      continue;
    tr = fr[m];
    fr[m] = fr[mr];
    fr[mr] = tr;
    ti = fi[m];
    fi[m] = fi[mr];
    fi[mr] = ti;
  }

  l = 1;
  k = LOG2_N_WAVE-1;
  while (l < n) {
    shift = 1;
    istep = l << 1;
    for (m=0; m<l; ++m) {
      j = m << k;
      /* 0 <= j < N_WAVE/2 */
      wr = pgm_read_word(&Sinewave[j+N_WAVE/4]);
      wi = -pgm_read_word(&Sinewave[j]);

      wr >>= 1;
      wi >>= 1;

      for (i=m; i<n; i+=istep) {
        j = i + l;
        tr = ((long)wr*(long)fr[j] - (long)wi*(long)fi[j])>>15;
        ti = ((long)wr*(long)fi[j] + (long)wi*(long)fr[j])>>15;
        qr = fr[i];
        qi = fi[i];

        qr >>= 1;
        qi >>= 1;

        fr[j] = qr - tr;
        fi[j] = qi - ti;
        fr[i] = qr + tr;
        fi[i] = qi + ti;
      }
    }
    --k;
    l = istep;
  }
  return scale;
}

int fix_fftr(int f[], int m )
{
  int i, Nt = 1<<(m-1), scale = 0;
  int tt, *fr=f, *fi=&f[Nt];

  scale = fix_fft(fi, fr, m-1 );
  return scale;
}
//**************************************************************************************************

int x[N], fx[N];
int incomingByte;
int i, count, scale;

const int PING_PONG_BINS = 16;
int accum_s[PING_PONG_BINS];
int accum_n[PING_PONG_BINS];
// remove this once refactor is complete
//int kraccums = 0, zlaccums = 0, snaccums = 0;
//int kraccumn = 0, zlaccumn = 0, snaccumn = 0;

int sdvig = 32768; //DC bias of the ADC, approxim +2.5V. (kompensaciya post. sostavlyauschei).
int minim = 0;
int maxim = 254;
int vrem;
float kdmp = 0.95; //Smoothing constant.
float kary = 0.999; //AGC time constant.
//AGC affects visual display only, no AGC on analog part of the system
const int ledCountKr = 4; // the number of levels on the display for red color band
const int ledCountZl = 4; // the number of levels on the display for green color band
const int ledCountSn = 4; // the number of levels on the display for blue color band

int ledPinsKr[] = {
  32, 33, 31, 35 }; // an array of pin numbers to which red LEDs are attached
int ledPinsZl[] = {
  38, 36, 34, 37 }; // an array of pin numbers to which green LEDs are attached
int ledPinsSn[] = {
  16, 15, 27, 17 };// an array of pin numbers to which blue LEDs are attached
// int led[] = { 32,33,31,35,38,36,34,37,16,15,27,17,30,26,29,28}; // Changed by Rod

void setup() {
  //SoftPWMServoInit();
  //SoftPWMBegin();
  // loop over the pin array and set them all to output:
  for (int thisLed = 0; thisLed < ledCountKr; thisLed++) {
    pinMode(ledPinsKr[thisLed], OUTPUT);
  }
  for (int thisLed = 0; thisLed < ledCountZl; thisLed++) {
    pinMode(ledPinsZl[thisLed], OUTPUT);
  }
  for (int thisLed = 0; thisLed < ledCountSn; thisLed++) {
    pinMode(ledPinsSn[thisLed], OUTPUT);
  }

  // initialize serial communication, for debug purposes mostly,
  //be careful with serial on Linux (Ubuntu), it hangs up periodicaly:
  Serial.begin(115200);
  delay(3000);
  Serial.println("MAG Lab's Ping Pong FFT");
}

void loop()
{
  //Filling up input raw data array x[];
  // 14.6 msec for ADC and 12.4 msec everything else, TOTAL = 27 msec if FFT size = 64 (N=128).
  // 28.8 msec for ADC and 27.1 msec everything else, TOTAL = 55.9 msec if FFT size = 128 (N).

  // ADCSRA = 0x87;
  // // turn on adc, freq = 1/128 , 125 kHz.
  // ADMUX = 0x60;
  // //Bit 5 â€“ ADLAR: ADC Left Adjust Result
  // ADCSRA |= (1<<ADSC);
  // // while((ADCSRA&(1<<ADIF)) == 0); //Discard first conversion result.
  // while(!(ADCSRA & 0x10));
  //
  // for(i=0; i<N; i++ ) {
  // ADCSRA |= (1<<ADSC);
  // // while((ADCSRA&(1<<ADIF)) == 0);
  // while(!(ADCSRA & 0x10));
  //
  // x[i] = ADCL;
  // x[i] += (ADCH << 8);
  // }
  //
  // ADCSRA = 0x00;

  ////////////////////////////////////////////////////
  // Sample the ADC N times
  ////////////////////////////////////////////////////

  //unsigned long micros_next = micros() + 100; // 10kHz sample rate
  for (i=0; i<N; i++){
    //while( micros() < micros_next );  // Do nothing // todo: no time get really accurate timing working
    //micros_next = micros() + 100; // 10kHz sample rate
    x[i]=(analogRead(AnalogInput) - 512) << 1;
    delayMicroseconds(micros_delay);
    if (i & 0x01)
      fx[(N+i)>>1] = x[i] ;
    else
      fx[i>>1] = x[i] ;
  }

  //Performing FFT, getting fx[] array, where each element represents
  //frequency bin with width 65 Hz.

  fix_fftr( fx, log2N );

  // Calculation of the magnitude:
  for (i=0; i<N/2; i++)
  {
    fx[i] = sqrt((long)fx[i] * (long)fx[i] + (long)fx[i+N/2] * (long)fx[i+N/2]);
    fx[i] *= 12;
    if (fx[i] < 0) 
    {
      fx[i] = 0;
    }
    if (fx[i] > 255) 
    {
      fx[i] = 255;
    }
  }

  //Show data on three color LEDs display:
  for ( i = 0; i < PING_PONG_BINS; i++ )
  {
    // Save old data for each band RGB, for smoothing;
    accum_s[i] = accum_n[i];
    // Reset
    accum_n[i] = 0;
  }

  // Start at 1 to skip bin 0 with the DC component
  for ( count = 1; count < 32; count++ )
  {
    accum_n[count/2] = accum_n[count/2] + fx[count];
  }

  for ( i = 0; i < PING_PONG_BINS; i++ )
  {
    // Smoothing, so it fall down gradually.
    if ( accum_n[i] < (accum_s[i] * kdmp) )
      accum_n[i] = (accum_s[i] * kdmp);
  }

  boolean any_greater_than_maxim = false;
  vrem = accum_n[0];
  for ( i = 0; i < PING_PONG_BINS; i++ )
  {
    any_greater_than_maxim |= (accum_n[i] > maxim);
    vrem = max( vrem, accum_n[i] );
  }

  //Visual Display AGC for all three band
  if ( any_greater_than_maxim )
  {
    maxim = vrem ;
  }
  else {
    maxim *= kary;
  }


  // Output fft to leds
  //maxim = maxim >> 16;
  //if(maxim > 0) {
  for ( i = 0; i < PING_PONG_BINS; i++ )
  {
    //accum_n[i] = accum_n[i] >> 16;
    ledLevel1 = ((float)(accum_n[i])) ; //(float)(maxim))*160.0;
    SoftPWMServoPWMWrite(led[i],(int)(fx[i])); //changed ledlevel
    //SoftPWMSet(led[i],(int)(ledLevel1));
  }
  //}
  //else {
  // analogWrite(led1,0);
  // analogWrite(led2,0);
  // analogWrite(led3,0);
  //}
#ifdef GONE
#endif

  switch ( debug_state ) {
  case 0: // no debug
    break;
  case 1: // spectrum
    for (i=N/4-1; i>=0; i--){
      int jj;
      int ji;
      Serial.print(i,DEC);
      Serial.print("*");
      //    Serial.println((fx[i]/2 > 255) ? 255 : fx[i]/2);
      //    ji = (fx[i]/2 > 255) ? 255 : fx[i]/2; // change by Rod
      //    ji = (fx[i]/2 > 255) ? 255 : fx[i]/2; // change by Rod
      ji = (x[i]/2 > 255) ? 255 : (x[i]/2 < -255) ? -255 : x[i]/2; // change by Rod
      //    Serial.println(ji);
      if (ji < 0) {
        Serial.println("-");
      } 
      else {
        //    for( jj = 0; jj < (fx[i]/2 > 255) ? 255 : fx[i]/2; jj++ )
        for( jj = 0; jj < ji; jj++ ) // to fix negative result
        {
          Serial.print("*");
        } 
      }
      Serial.println("+");
    }
    delay(500);
    break;
  case 2: // signal
    for (i=1; i<N/2; i+=2){
      int jj;
      for( jj = 0; jj < abs(x[i]/2); jj++ )
      {
        Serial.print(".");
      }
      Serial.println("*");
    }
    delay(500);
    break;
  case 3: // ping pong spectrum
    for ( i = 0; i < PING_PONG_BINS; i++ )
    {
      Serial.println(accum_n[i]);
    }
    Serial.println(maxim);
    Serial.println(ledLevel1);
    delay(500);
    break;
  }

  //Debugging monitor, allow to check processing data on each stage.
  // x command - printout data received from ADC (input raw data).
  if( receive_function(command,sizeof(command)) ){

    if( strcmp(command,"v?") == 0 ){
      Serial.println("Mag Labs PPBASA version tu point oh");
    }
    if( strcmp(command,"reset") == 0 ){
      Serial.println("Close serial terminal, resetting board in..."); 
      char sec = 5; 
      while(sec >= 1) 
      { 
        Serial.println(sec, DEC); 
        delay(1000);
        sec--;
      }
      Reset();
    }
    if( strcmp(command,"md") == 0 ){
      micros_delay += 10;
      if( micros_delay > 40 ) micros_delay = 0;
      Serial.println(micros_delay, DEC);
    }
    if( strcmp(command,"a") == 0 ) {
      debug_state = 0;
    }
    if( strcmp(command,"b") == 0 ) {
      debug_state = 1;
    }
    if( strcmp(command,"c") == 0 ) {
      debug_state = 2;
    }
    if( strcmp(command,"d") == 0 ) {
      debug_state = 3;
    }

    // f command - printout data after FFT. Clear view of each bin in the spectrum.
    // Plus printing summary accumulator for each band and variable MAXIM.
    if( strcmp(command,"f") == 0 ){
      for (i=1; i<N/4; i++){
        Serial.print(fx[i], DEC);
        Serial.print(",");
        //if ((i+1)%10 == 0) Serial.print("\n");
      }
      Serial.println("");

      for ( i = 0; i < PING_PONG_BINS; i++ )
      {
        Serial.print("accum_n[");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(accum_n[i], DEC);
        Serial.println("");
      }
      Serial.print("\n MAXIM: ");
      Serial.print(maxim, DEC);
      Serial.print("\n");
      Serial.println(" Array FFT printed.");
      delay(200);
    }

    if( strcmp(command,"q") == 0 ){
      Serial.print("maxim=");
      Serial.print(maxim);
      Serial.println("");
      for ( i = 0; i < PING_PONG_BINS; i++ )
      {
        Serial.print("accum_n[");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(accum_n[i]);
        Serial.println("");
      }
    }
    // t command - test out fan throughput using SoftPWM
    /*     if (incomingByte == 't') {
     for (i=0; i < PING_PONG_BINS; i++){
     SoftPWMServoPWMWrite(led[i],0);
     //SoftPWMSetFadeTime(led[i],1000,1000);
     }
     delay(5000);
     for(i=0; i < PING_PONG_BINS; i++){
     Serial.print("Fade-in on pin ");
     Serial.print(led[i]);
     Serial.println("...");
     SoftPWMServoPWMWrite(led[i],225); // change by Rod
     delay(500);
     }
     delay(500);
     for(i=0; i < PING_PONG_BINS; i++){
     Serial.print("Fade-out on pin ");
     Serial.print(led[i]);
     Serial.println("...");
     SoftPWMServoPWMWrite(led[i],0);
     delay(500);
     }
     delay(500);
     } */
    if( strcmp(command,"pwm.test") == 0 ){
      int iTimeDelay = 100;
      int iStep = 20;
      int iWidth = 4;
      int iPoints = 16;

      for (int d = 0; d < iPoints; d++)
      {
        Serial.print("d:");
        Serial.print(d);
        Serial.println(" ");

        for (int16_t  i = 0; i < 250; i+=25 )
        {
          SoftPWMServoPWMWrite(led[d], (uint8_t)i);
          delay(iTimeDelay);
        }
        delay(5000);
        for (int16_t  i = 250; i >= 0; i-=25 )
        {
          SoftPWMServoPWMWrite(led[d], (uint8_t)i);
          delay(iTimeDelay);
        }
      }
    }
    if( strcmp(command,"t") == 0 ){
      int iTimeDelay = 70;
      int iStep = 20;
      int iWidth = 4;
      int iPoints = 15 + iWidth;
      ledLevel = 250 - ((iWidth - 1) * iStep);
      Serial.print("W ");
      Serial.print("D-");
      for (int d = 0; d <= iPoints; d++)
      {
        Serial.print("2-");
        Serial.print(d);
        Serial.println(" ");
        if (d < iWidth) // width
        {
          AledLevel[0] = ledLevel;
          ledLevel = ledLevel + iStep;
        }
        for ( i = 15; i >= 0; i-- )
        {
          Serial.print(AledLevel[i]);
          Serial.print(" ");
          Serial.println(i);
          SoftPWMServoPWMWrite(led[i],(int)(AledLevel[i]));
          AledLevel[i] = AledLevel[i - 1];
          delay(iTimeDelay);
        }
        AledLevel[i] = 0;
      }
      incomingByte = 0;
    }
    // x command - 
    if( strcmp(command,"x") == 0 ){
      for (i=0; i<N; i++){
        Serial.println(x[i], DEC);
        //Serial.print(", ");
        //if ((i+1)%10 == 0) Serial.print("\n");
      }
      Serial.print("\n");
      incomingByte = ' ';
      delay(2000);
    }
    // z command - stop any communication, handy command on Ubuntu.
    if( strcmp(command,"z") == 0 ){
      Serial.println(" STOP in 2 sec.");
      delay(2000);
      Serial.end();
    }

    //Adjustment Time constant of the AGC, depends on nature of the music.
    if( strcmp(command,"1") == 0 ){
      kary = 0.99;
      Serial.print("\n");
      Serial.println(" New value kary = 0.99.");
      delay(200);
    }
    if( strcmp(command,"2") == 0 ){
      kary = 0.999;
      Serial.print("\n");
      Serial.println(" New value kary = 0.999.");
      delay(200);
    }
    if( strcmp(command,"3") == 0 ){
      kary = 0.9999;
      Serial.print("\n");
      Serial.println(" New value kary = 0.9999.");
      delay(200);
    }
  }

  // Display, 3 band (RGB), 4 level Original
  // int ledLevelKr = map(kraccumn, minim, maxim, 0, ledCountKr);
  //
  // for (int thisLed = 0; thisLed < ledCountKr; thisLed++) {
  // if (thisLed < ledLevelKr)
  // digitalWrite(ledPinsKr[thisLed], HIGH);
  // else
  // digitalWrite(ledPinsKr[thisLed], LOW);
  // }
  //
  // int ledLevelZl = map(zlaccumn, minim, maxim, 0, ledCountZl);
  //
  // for (int thisLed = 0; thisLed < ledCountZl; thisLed++) {
  //// if (thisLed < ledLevelZl)
  //// digitalWrite(ledPinsZl[thisLed], HIGH);
  //// else
  //// digitalWrite(ledPinsZl[thisLed], LOW);
  // }
  //
  // int ledLevelSn = map(snaccumn, minim, maxim, 0, ledCountSn);
  //
  // for (int thisLed = 0; thisLed < ledCountSn; thisLed++) {
  //// if (thisLed < ledLevelSn)
  //// digitalWrite(ledPinsSn[thisLed], HIGH);
  //// else
  //// digitalWrite(ledPinsSn[thisLed], LOW);
  // }
}






