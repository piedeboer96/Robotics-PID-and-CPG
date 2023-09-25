//PID Control Template

// *********************************************************************************************************************** //
// Variables                                                                                                               //
// *********************************************************************************************************************** //

//pin definition
#define ENA 9
#define IN1 21
#define IN2 20
#define SENSOR_PIN A0

// ******** <TODO> **********************
// ******** define interval between recomputing error and adjusting feedback (in milliseconds) ********************** 
const int INTERVAL = 15;  // to start with, we experiment using 15 ms
unsigned long previousTime = 0;

int motorSpeed = 0; // speed of the motor, values between 0 and 255
int target = 512; // position (as read by potentiometer) to move the motor to, default value 512

// ******** <TODO> **********************
// ******** define the different gains **********************
float kp = 0.0; // proportional gain
float ki = 0.0; // integral gain
float kd = 0.0; // derivative gain

int pos = 0; // current position for plotting
//serial communication variables
float PID_values[4];
byte i = 0;
char record[100];
char recvchar;
byte indx = 0;


// additional variables we defined
float error = 0;
float u = 0;
unsigned long currentTime;        // 
unsigned long previousTime;       // last point in time measured

int integrationSize = 5;          // integral range , we use the last 5 intervals 
float[] errorBuffer = float(integrationSize); // ewmwmbwe rhw mesured errors from previous timestep to compute the integral and derivative
int bufferpPosition = 0;          // where in the errorBuffer we are

// setup code, setting pin modes and initialising the serial connection
void setup() 
{
    Serial.begin(115200);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);    
    pinMode(SENSOR_PIN, INPUT);   
}

void loop() 
{
        //  ******** <TODO> **********************
        //  ******** implement your code  here **********************

        // current time at start of loop
        currentTime = millis();

        // start reading the input
        readInput()

        // take sample based on defined interval
        if(currentTime - previousTime > INTERVAL) {

              // advance time 
              previousTime = currentTime;

              // read the (current) position of potentiometer
              pos = analogRead(SENSOR_PIN);
        
              // compute the error using  'p_obs - p_set' 
              error = (pos - target)

              errorBuffer[errorPosition] = error;
              if(++errorPosition == integrationSize){
                errorPosition = 0;
              }

              // compute speed in order to practice control
              u = -kp * error * -ki * computeIntegral() * - kd * computeDerivative()

              // investigate both the DIRECTION based on sign and RANGE
              if (u<0 && u<255) {
                setMovement(-1,u)
              } else if(u>0 && u<255) {
                setMovement(0,u)
              } else {
                setMovement(0,255)
              }
       
        } 

        //print actual motor position and target value to serial-monitor/plotter
        Serial.print(pos);
        Serial.print(" ");   
        Serial.println(target);
    
}

// Lukas is cool and smart <3
float computeDerivative(){
  // TODO
}

// Matthew is stronger than Lukas.. hihi
float computeIntegral(){
  // TODO
}

// method to set direction and speed of the motor
void setMovement(int dir, int speed1) 
{

        // set the speed to value of 'speed1' range 0~255
        analogWrite(ENA, speed1);

        //  tell the motor to go forward (may depend by your wiring)
        //  NOTE: see emperical what direction it goes...
        if(dir==0){
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
        } else {  
            // alternative direction...
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
}
// *********************************************************************************************************************** //
// method for receiving commands over the serial port
void readInput() 
{
      if (Serial.available())
    {
        recvchar = Serial.read();
        if (recvchar != '\n')
        { 
            record[indx++] = recvchar;
        }
        else if (recvchar == '\n')
        {
          record[indx] = '\0';
          indx = 0;
          
          convertData(record);
          if(i==4){
          target = PID_values[0];
          kp = PID_values[1];
          ki = PID_values[2];
          kd = PID_values[3];
          Serial.print("Entered Values:");
          printData(PID_values);
          }
          else
          {
          Serial.println("Enter correct number of values separated by commas!!");            
          }
        }
    }
}
// *********************************************************************************************************************** //
//method for reading/interpreting serial input 
void convertData(char record[])
{
    i = 0;
    char *index = strtok(record, ",");
    while(index != NULL)
    {
       PID_values[i++] = atof(index); 
        index = strtok(NULL, ",");
    }
}
// *********************************************************************************************************************** //
//method for printing values entered via the serial monitor
void printData(float data[])
{
    for (byte j = 0 ; j < 4 ; j++)
    {
      Serial.print(data[j]);
      Serial.print('\t');
    }
    Serial.println(); 
}



