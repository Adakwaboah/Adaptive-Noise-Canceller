/* Author: Akwasi D. Akwaboah
 * Institution: Johns Hopkins University
 * Project Name: Adaptive Noise Canceller
 * Description: Implementation of the Widrow-Hopf algorithm as described in 
 * Widrow, B., Glover, J.R., McCool, J.M., Kaunitz, J., Williams, C.S., Hearn, R.H., Zeidler, J.R., Dong, J.E. 
 * and Goodlin, R.C., 1975. Adaptive noise cancelling: Principles and applications. 
 * Proceedings of the IEEE, 63(12), pp.1692-1716.
 * Version 0.0: 05/10/2021
 */


//Arduino Nano 33 BLE Sense used

#include <math.h>

int analog_Pin = A0;    // microphone A
int OutputPin = 9;      // analogwrite output
double sensorCache [4] = {0.0,0.0,0.0,0.0}; //sensor Cache for adaptive filter
//                           0.0,0.0,0.0,0.0,
//                           0.0,0.0,0.0,0.0, 
//                           0.0,0.0,0.0,0.0,}; //allocate memory for storing 16 time-shifted audio samples. 
double W [4] = {0.0,0.0,0.0,0.0};  // weiner filter weights
//                 0.0,0.0,0.0,0.0,
//                 0.0,0.0,0.0,0.0, 
//                 0.0,0.0,0.0,0.0,}; 
double y = 0.0; //weighted sum of shifted noise signals
double sig_in = 0.0; //temp storage for signal+ noise from mic A
double sig_out = 0.0; //temp storage for filter signal
double mu = 0.5e-2; //learning rate - to do: implement only hyperparameter tuning similar to momentum    
double pi = 3.14159265359; 
double n = 0.0;
double f1 = 5.0;
double f2 = 3.0;
double Ts = 1.0/10000.0; //fs = 10000.0
double noisy_sine = 0.0;
double noise = 0.0;
int analog_out = 0.0; // value output to the PWM (analog out)
int noise_outPin = 8;
int in_byte = 0; // trigger read from matlab
int N = 4; //filter size

void setup() {
    // initialize serial communications at 9600 bps:
    Serial.begin(9600);
    randomSeed(analogRead(A2));
    pinMode(noise_outPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    in_byte = Serial.read();

    // for debugging
//    Serial.print("I received: ");
//    Serial.println(in_byte, DEC);
  }

  if (in_byte == 0){
    analogWrite(OutputPin, 0); //keep indicator off
  }

  else{
    //read data from sensor
    sig_in = ((double)analogRead(analog_Pin)-310.0)/310.0; //read and deal with offset
    //emulation of white noise
    noise = 1.0*(double)(random(-100, 100))/310.0;
    //mix signal with noise to emulate the micA input
    noisy_sine = sig_in + noise;
  
    //overall loop for FIFO buffer
    sensorCache[0] = noise; //read noise map to -1 to 1
    //Compute dot product and Shift filter cache
    for (int i = 0; i < N-1; i++){
      sensorCache[i+1] = sensorCache[i];
    }
  
    y = 0.0; //reset dot product for next
    //Weighted sum
    for (int i = 0; i < N; i++){
      y += W[i]*sensorCache[i];
    }
    sig_out  = noisy_sine - y; //output/ error
  
    //Weight update
    for (int i = 0; i < N; i++){
      W[i] += 2.0*mu*sig_out*sensorCache[i];
    }
    
    analog_out = map(sig_out*100, -127, 127, 0, 255);
    analogWrite(OutputPin, analog_out);
  
  //   print the results to the Serial Monitor:
//    Serial.print("noisy_signal:"); Serial.print(noisy_sine, 10);  Serial.print(", ");
//    Serial.print("denoised_signal:"); Serial.print(sig_out, 10);  Serial.print(", ");
//    Serial.print("noise:"); Serial.println(noise, 10);  //Serial.println();
  
//    //for Matlab
    Serial.println(sig_in, 10); //ground truth
    Serial.println(noisy_sine, 10);  //Serial.print(",");
    Serial.println(sig_out, 10);  //Serial.print(",");
    Serial.println(noise, 10); //;  Serial.println();
  
  //  Serial.print(sig_out, 10);
  //  Serial.println();
    
    //update time instance count
    n += 1.0;
    delay(Ts*1000);
  }
}
