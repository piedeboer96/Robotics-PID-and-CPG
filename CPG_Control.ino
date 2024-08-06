//Tutorial07 CPG Control Template

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Calibration of Servo Limits
//Please enter our specific servo limits for servo 1, 2, (and 3) here
//You can find the corresponding limits in your EDMO box
int SERVOMIN[]  {108, 108}; //PLEASE ENTER: The lower motor PPM limit (Servo Min) as noted in your EDMO box
int SERVOMAX[]  {528, 530}; //PLEASE ENTER: The upper motor PPM limit (Servo Max) as noted in your EDMO box

//////////////////////////////////////////////////////////////////////////////////////////////////////// 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
const unsigned int NUM_OSCILLATORS = 2; // this number has to match entries in array osc[] (do NOT modify!!)

//Timing variables
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long timeStep = 10; // period used to update CPG state variables and servo motor control (do NOT modify!!)
double interval = 0; //variable to store actual measured update time of the PID

//CPG parameter
double frequency = 0.5; // oscillator frequency
double rateOfFrequency = 0;
double targetFrequency = 2.5;

double w = 5; // we assume that all oscillators use same coupling weight
double a = 5; // we assume that all oscillators use same adaptation rate for amplitude
double c = 2; // adaptation rate for frequency and offset

//float calib[NUM_OSCILLATORS];


typedef struct 
{
    double phase;                       // phase of the oscillation
    double amplitude;                   // amplitude of the oscillation 
    double targetAmplitude;             // amplitude to gradually change to
    double offset;                      // offset for the oscillation (in range of servo 0-180)
    double targetOffset;                // added parameter to offset smoothly
    double rateOfPhase;                 // current rate of change of the phase parameter
    double rateOfAmplitude;             // current rate of change of the amplitude parameter
    double rateOfOffset;                // current rate of change of the offset parameter
    double pos;                         // oscillator output = servos angular position in degrees
    uint16_t angle_motor;                  // mapped motor PPM value used to set motor position according to CPG pos
    double phaseBias[NUM_OSCILLATORS];  // controls pairwise coupling phase bias
    double coupling[NUM_OSCILLATORS];   // controls topology of the network
} oscillator;

// initalisation with offset 90 for all motors (since servos operate in the range -90 to 90)
oscillator osc[NUM_OSCILLATORS] = 
{
    {0,30,15,90,170,0,0,0,0,0,{0,90},{0,1}},
    {0,30,65,90,145,0,0,0,0,0,{-90,0},{1,0}}
};

// strings for reading input
String commandString, valueString, indexString;

////////////////////////////// setup //////////////////////////////////////
//(do NOT modify!!)
void setup() 
{
    //////////////Initialize Serial Communication//////////////////
    Serial.begin(9600); 
    Serial.flush();
    while (!Serial); //wait for the USB to serial chip to get ready (really quick!!)Does NOT! wait for the serial monitor to open!!!!   
    pwm.begin();  
    pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz update
    delay(4);
//    zeroCalib();
//    setCalib(0,0);
//    setCalib(1,0);
//    setCalib(2,0);
}


void loop(){    
  
    currentMillis = millis(); //update the current time (do NOT modify!!)
    if (currentMillis - previousMillis >= timeStep){//CPG update interval (10ms)(do NOT modify!!)
      interval=currentMillis - previousMillis; //calculate actual interval time (do NOT modify!!)
      previousMillis = currentMillis; //update the previous time step (do NOT modify!!)
      readInput(); //read input command from serial monitor (do NOT modify!!)
      

      interval *= 0.001;
      //+++++++++++++++IMPLEMENT your CPG control code here BELOW !!!++++++++++++++++++++++++++++++++++++
      

      
       for (int i = 0; i < NUM_OSCILLATORS; i++) {

          // TODO: check!

          // Calculate CPG here
          osc[i].rateOfPhase =  getPhaseRateControl(i); 
          osc[i].rateOfAmplitude = a*(osc[i].targetAmplitude - osc[i].amplitude);
          osc[i].rateOfOffset = c*(osc[i].targetOffset - osc[i].offset);


          // update location in angles
          osc[i].pos = osc[i].amplitude*sin(osc[i].phase * M_PI/180) + osc[i].offset;


          // Update control parameters
          osc[i].phase = osc[i].phase + osc[i].rateOfPhase*interval;
          osc[i].amplitude = osc[i].amplitude + osc[i].rateOfAmplitude*interval;
          osc[i].offset = osc[i].offset + osc[i].rateOfOffset*interval;
  
          // set motor to new position (do NOT modify!!)
          //osc[i].pos+= calib[i]; //servo calibration. Not required anymore!!
          osc[i].angle_motor = map(osc[i].pos,0,180,SERVOMIN[i],SERVOMAX[i]);//(do NOT modify!!)
          osc[i].angle_motor = constrain(osc[i].angle_motor,SERVOMIN[i],SERVOMAX[i]); //(do NOT modify!!)          
          pwm.setPWM(i, 0, osc[i].angle_motor);//(do NOT modify!!)
  
          
          Serial.print(osc[i].pos);
          Serial.print(" ");
       }
       //+++++++++++++++IMPLEMENT your CPG control code here ABOVE!!!++++++++++++++++++++++++++++++++++++
       Serial.println();


      rateOfFrequency = c*(targetFrequency - frequency);
      frequency = frequency + rateOfFrequency*interval; 
    }
}


// compute the rate of phase for oscillator with id "osc_id"
// TODO: check!!!
float getPhaseRateControl(int osc_id){

    float rateOfPhase= 360*frequency;

    for(int i = 0; i < NUM_OSCILLATORS; ++i){
      //rateOfPhase += w*osc[i].amplitude*osc[osc_id].coupling[i]*sin( (osc[i].amplitude - osc[osc_id].amplitude - osc[osc_id].phaseBias[i]) * M_PI/180 );
      rateOfPhase += w*osc[i].amplitude*osc[osc_id].coupling[i]*sin( (osc[i].phase - osc[osc_id].phase - osc[osc_id].phaseBias[i]) * M_PI/180 );
    } 
    return rateOfPhase;
}



/////////////////Function for Reading Inputs via the Serial Monitor//////////////////////////////
///////////////////////////- PLEASE DO NOT MODIFY!/////////////////////////////////
void readInput() 
{
    //if there is an input via the serial monitor read and parse it
    if (Serial.available()) 
    {
        commandString = Serial.readStringUntil('\n');//read received string 
        /////////////////PARSE INPUT//////////////////////////////////////////
        if (commandString.startsWith("amp")) 
        {
            // change the target amplitude for the specified oscillator
            indexString = commandString.substring(4, 5);
            valueString = commandString.substring(6, commandString.length());
            osc[(int) indexString.toInt()].targetAmplitude = (int) valueString.toInt();
        } else if (commandString.startsWith("off")) 
        {
            // change the target offset for the specified oscillator
            indexString = commandString.substring(4, 5);
            valueString = commandString.substring(6, commandString.length());
            osc[(int) indexString.toInt()].targetOffset = (int) valueString.toInt();
        } else if (commandString.startsWith("freq")) 
        {
            // change the target frequency for all oscillators
            valueString = commandString.substring(5, commandString.length());
            targetFrequency = (float) valueString.toFloat();
        } else if (commandString.startsWith("phb")) 
        {
            // change the phase bias between the two specified oscillators
            indexString = commandString.substring(4, 5);
            int index1 = (int) indexString.toInt();
            indexString = commandString.substring(6, 7);
            int index2 = (int) indexString.toInt();
            valueString = commandString.substring(8, commandString.length());
            osc[index1].phaseBias[index2] = (float) valueString.toFloat();
            osc[index2].phaseBias[index1] = -osc[index1].phaseBias[index2];
        } else if (commandString.startsWith("weight")) 
        {
            // change the weight for the adaptation of the rate of change of the phase
            valueString = commandString.substring(7, commandString.length());
            w = (float) valueString.toFloat();
        } else if (commandString.startsWith("print")) 
        {
            // print information about the current state of the oscillators
            Serial.print("Frequency: ");
            Serial.println(frequency);
            for (int i = 0; i < NUM_OSCILLATORS; i++) 
            {
                Serial.print(i);
                Serial.print(": ");
                Serial.print("[");
                Serial.print(osc[i].phase);
                Serial.print(", ");
                Serial.print(osc[i].amplitude);
                Serial.print(", ");
                Serial.print(osc[i].targetAmplitude);
                Serial.print(", ");
                Serial.print(osc[i].offset);
                Serial.print(", ");
                Serial.print(osc[i].targetOffset);
                Serial.print(", ");
                Serial.print(osc[i].rateOfPhase);
                Serial.print(", ");
                Serial.print(osc[i].rateOfAmplitude);
                Serial.print(", ");
                Serial.print(osc[i].rateOfOffset);
                Serial.print(", ");
                Serial.print(osc[i].pos);
                Serial.print(", [");
                Serial.print(osc[i].phaseBias[0]);
                Serial.print(", ");
                Serial.print(osc[i].phaseBias[1]);
                Serial.print(", ");
                Serial.print(osc[i].phaseBias[2]);
                Serial.print("]]");
                Serial.println();
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////
//void zeroCalib()
//{
//    for (byte j = 0 ; j < NUM_OSCILLATORS ; j++)
//      calib[j] = 0;
//}
//
///////////////////////////////////////////////////////////////////////////////////
//void setCalib(int motor,int val)
//{
//    if(motor < NUM_OSCILLATORS)
//        calib[motor] = val;
//    else
//       Serial.println("Enter a valid motor number"); 
//}
