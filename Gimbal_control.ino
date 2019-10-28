/*  
 *  Bayangtoys X21 BGC Gimbal Control for Ardunio
 *  by kr0k0f4nt (2017)
 *  PPM Processing based on Code by Sijjim (https://forum.arduino.cc/index.php?topic=182681.0)
 *  Inspired by Muhammad Imam Zulkarnaen (https://www.youtube.com/watch?v=pYitT60Frjc)
 */ 

// *********************** SETTINGS ****************************
/*
 * Angle can be 0-2000, but only Values between 1500-2000 make sense
 * 1000 = Camera looks straight up to the drone body, lower values make it look backwards
 * 1500 = Neutal Setting looking straight forward
 * 2000 = Cameras looks straight down
 */
#define MAX           2000     // Max Angle
#define STD           1650     // Standard Angle for MODE 1
#define MIN           1500     // Min Angle
#define STEP          25       // Steps for MODE 2

/*
 * Interrputs are not available on all Pins, for the Nano only on 2 & 3
 */
#define PPM_PIN       2        // PIN with X21 Signal

/*
 * This only needs to be a Pin that is capable of PWM, for the Nano this is 3,5-6,9-11
 */
#define GIMBAL_PIN    6       // PIN with Gimbal Signal

/*
 * Setting Up Pin Pair for MODE Control
 */
#define MODE_PIN_OUT  7        // MODE Pin Output
#define MODE_PIN_IN   8        // MODE Pin Input
#define MODE_1_FIXED  1        // Mode for fixed Angles on Video Button
#define MODE_2_STEP   2        // Mode for Stepping Angles video Video (Down) and Photo Up

/*
 * This will set the number of channels in the PPM signal
 */

#define CHANNELS        3         // X21 works with 3 Channels
#define CHANNEL_SIGNAL  3         // X21 Signal is on Channel 3

/*
 * These are the Signal average Values for the Channel 3, which varies on the Video/Photo Buttons being pressed
 */

#define SIGNAL_BASE    500
#define SIGNAL_PHOTO   1100
#define SIGNAL_VIDEO   1600

// ****************** GLOBAL VARIABLES ******************

#include <Servo.h>
Servo Gimbal;

// State Variables for handling Signal toggles
boolean VideoMode = false;
int LastSignal = SIGNAL_BASE;

int GimbalMode = 0;
int GimbalState = STD;

// Variables for MODE 1 Control by Photo Button
int GimbalSteps = 0; 
int GimbalLastSteps = 5;

// Variables for PPM Processing
volatile int Values[CHANNELS + 1] = {0};

// *****************************************************

void setup() {
  // Serial Communcication Output 
  Serial.begin(115200);

  // Settings up MODE pins
  pinMode(PPM_PIN, INPUT);
  pinMode(MODE_PIN_OUT, OUTPUT);
  pinMode(MODE_PIN_IN, INPUT_PULLUP);
  digitalWrite(MODE_PIN_OUT, LOW);

  // Setting up Gimbal
  Gimbal.attach(GIMBAL_PIN);
  Gimbal.writeMicroseconds(GimbalState);
}

void loop(){

  // MODE Selection
  if (digitalRead(MODE_PIN_IN) == HIGH) {
    if (GimbalMode != MODE_1_FIXED) {
      GimbalMode = MODE_1_FIXED;
      Serial.println("MODE 1 - Fixed Angles");
    }
  } else {
    if (GimbalMode != MODE_2_STEP) {
      GimbalMode = MODE_2_STEP;
      Serial.println("MODE 2 - Stepping Angles");
    }
  }

  // Wait for Sync on Signal
  while(pulseIn(PPM_PIN, HIGH) < 2500){}
  
  // Processing PPM Signal
  for (int Channel = 1; Channel <= CHANNELS; Channel++) {
    Values[Channel] = pulseIn(PPM_PIN, HIGH);
  }

  // Determinig the Sigal for easier handling
  int Signal = Values[CHANNEL_SIGNAL];
  if (Signal <= 1800 && Signal >= 1500) {
    Signal = SIGNAL_VIDEO;
  } else if (Signal <= 1200 && Signal >= 900) {
    Signal = SIGNAL_PHOTO;
  } else {
    Signal = SIGNAL_BASE;
  }


  // Only do something whenever Signal changes
  if (LastSignal != Signal) {
    
    /* 
     *  Figuring out which Key was actually pressed ...
     *  This is a bit tricky as there are only 3 Signal States and the SIGNAL_BASE is shared by both Buttons
     *  To solve this we need to maintain the State of the VideoMode as well as the LastSignal.
    */
    
    int KeyPressed;
    
    switch(Signal) {
      case SIGNAL_VIDEO:
        if (VideoMode) {
          VideoMode = false;
          KeyPressed = SIGNAL_VIDEO;
        } else {
          VideoMode = true;
          KeyPressed = SIGNAL_VIDEO;  
        }
        break;
      case SIGNAL_PHOTO:
        KeyPressed = SIGNAL_PHOTO;
        break;
      case SIGNAL_BASE:
        if (VideoMode == true && LastSignal == SIGNAL_VIDEO) {
          VideoMode = false;
          KeyPressed = SIGNAL_VIDEO;
        } else if (VideoMode == false && LastSignal == SIGNAL_VIDEO) {
          VideoMode = true;
          KeyPressed = SIGNAL_VIDEO;
        } else {
          KeyPressed = SIGNAL_PHOTO;
        }
        break;
    }

    DebugPrintStates(KeyPressed, Signal, VideoMode);

    if (GimbalMode == MODE_1_FIXED) {
      // MODE 1 - Using Fixed Values on Video Button / Toggle Control on Photo

      // Toggle between Standard and Maximum Angle with Video Mode
      if (KeyPressed == SIGNAL_VIDEO) {
        // Video Mode represents the Angles on the Remote, it toggels between MAX and STD Settings
        if (VideoMode) {
          GimbalState = MAX;
        } else {            
          GimbalState = STD;
        }
      } else {
        // Photo Button controls the Gimbal by starting or stopping the motion and cycle direction each time
        if (GimbalSteps != 0) {
          // Gimbal is currently moving ... Stop now!
          GimbalSteps = 0; 
        } else {
          // Gimbal is stopped ... Start moving and switch direction!
          GimbalSteps = GimbalLastSteps * (-1);
          GimbalLastSteps = GimbalSteps;
        }
      }
      
    } else {
      // MODE 2 - Using Photo Button for Step Up / Video Button for Down

      if (KeyPressed == SIGNAL_VIDEO) {
        // Moving the Gimbal downwards ...
        if (GimbalState + STEP <= MAX) {
          GimbalState += STEP;
        } else {
          GimbalState = MAX;
        }
      } else {
        // Moving the Gimbal upwards ...
        if (GimbalState - STEP >= MIN) {
          GimbalState -= STEP;
        } else {
          GimbalState = MIN;
        }
      }
        
    }

    // Overwrite Last Signal
    LastSignal = Signal;
    
  }

  // Write GimbalState but maintain MIN/MAX Angles and stop movment if exceeded
  if (GimbalState + GimbalSteps > MAX) {
    GimbalState = MAX;
    GimbalSteps = 0;
  } else if (GimbalState + GimbalSteps < MIN) {
    GimbalState = MIN;
    GimbalSteps = 0;
  } else {
    Gimbal.writeMicroseconds(GimbalState += GimbalSteps);
  }
  
}

void DebugPrintStates(int KeyPressed, int Signal, boolean VideoMode) {
    Serial.print("Key pressed: ");
    if (KeyPressed == SIGNAL_VIDEO) {
      Serial.print("VIDEO - ");
      Serial.println(Signal);
    } else {
      Serial.print("PHOTO - ");
      Serial.println(Signal);
    }
    Serial.print("Video Mode: ");
    if (VideoMode) {
      Serial.println("ON");
    } else {
      Serial.println("OFF");
    }  
}
