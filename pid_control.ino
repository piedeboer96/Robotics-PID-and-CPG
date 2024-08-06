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
// ******** defining interval between measurements of servo (in milliseconds) ********************** 
const int INTERVAL = 15;  // each INTERVAL miliseconds, a new correction is done using the PID controller
unsigned long currentTime;        // current time 
unsigned long previousTime = 0;       // last point in time measured

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


float i_err = 0; // integration error (summed up after every iteration)
float i_err_lim = 10000; // maximum integration error - prevents windup
float last_err = 0; // last error - used for computing derivative
float d_err = 0; // derivative error 
float zero_err_threshold = 3; // if error is this small, reset the integration error to 0

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
        readInput();

        // take sample if INTERVAL time has passed
        if(currentTime - previousTime > INTERVAL) {

     
              // read the (current) position of potentiometer
              pos = analogRead(SENSOR_PIN);
        
              // compute the error using  'p_obs - p_set' 
              error = (pos - target);
              

              // compute the derivative error   
              d_err = (error  - last_err)/(millis() - previousTime); // to make the derivative as accurate as possible, we take the actual current time using "millis()" function
                                                                     // this makes a difference because the time when computing the derivative may differ from the error measurement
                                                                     // made

              // Add to the integration part of PID
              // to prevent windup, the error can at most be i_err_lim
              if (abs(i_err + error*(millis() - previousTime)) <= i_err_lim){
                i_err += error*(millis() - previousTime); // again, millis() is used for accurate time window
              }
              else{
                i_err = i_err_lim;
                if (i_err + error*(millis() - previousTime) < 0){
                  i_err = -i_err_lim;
                }
                
              }

              if(abs(error) < zero_err_threshold){ // if error is small, reset the integration error to 0
                i_err = 0;
              }

      
              
                  
            
              // compute speed in order to practice control using PID, the kp, ki, and kd params are inputted using serial monitor
              u = -kp * error -ki * i_err - kd * d_err;

              // now if the control value u is negative, we need to set the movement in the opposite direction
              if (u<0 && u>=-255) {
                setMovement(-1,abs(u));
              }
              else if(u < 0){
                setMovement(-1,255);
              }
              // if the u param is positive, the direction of movement goes again the other way
              else if(u>0 && u<255) {
                setMovement(0,u);
              } 
              else if(u > 0) {
                setMovement(0,255);
              }
              // if u = 0, no movement is done
              else{
                setMovement(0,0);
              }
              // the conditions above also handle the fact that we can at most set 255 as the speed parameter, hence if it is larger, we ignore that
              // NOTE: the param 'u' is float, but the corresponding input param in "setMovement()" is an unsigned int. C however does the conversion for us automatically.

              // advance time, remember the time when we entered the outermost if condition, as the previous_time, so that even if the code inside the if condition takes e.g. 10ms, it will not delay the time at which we enter the outermost if condition again
              previousTime = currentTime;
              last_err = error; // remember the last error, so that we can compute the derivative

              //print actual motor position and target value to serial-monitor/plotter
              Serial.print(pos);
              Serial.print(" ");   
              Serial.println(target);

              
       
        } 

        
    
}


// method to set direction and speed of the motor
void setMovement(int dir, int speed1) 
{

        // set the speed to value of 'speed1' range 0~255
        analogWrite(ENA, speed1);

        //  tell the motor to go forward 
        if(dir==0){
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
        } 
        // tell the monitor to go backward
        else {  
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

          i_err = 0; // NOTE!!! - here there is a change in the code
          // to handle integration windup, we reset the integration error to 0, each time new target is set
          // this is because, assuming we do not reset it, and the integration error is already at the maximum (hit the limit we defined), it would only behave as a constant further on for each new target
          // specifically, adding constant of "integral_lim" to the control value "u" all the time. This esentially means, there would be no I part of the PID controller anymore 

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


