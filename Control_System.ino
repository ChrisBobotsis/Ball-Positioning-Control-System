// Pin Setup

#define Sprint(a)   //(Serial.print(a))
#define Sprintln(a) //(Serial.println(a))

#define Xplus   A1                        
#define Xminus  3     // Pin 3
#define Yplus   A0                        
#define Yminus  5     // Pin 5

////Variable Setups //////////////////////////////////////////////////////////////////////////////////

unsigned int total_x = 0;
unsigned int total_y = 0;

////Servo Setups /////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>

#define xServo_pin 9
#define yServo_pin 10

Servo xServo;
Servo yServo;

int Servo_midpoint = 1500;                    // CHOSEN BY USER!!!    NOW IN uSeconds

int servo_output;
////////////////////////////////////
//      New Servo Convention
//
//               ::  + Max Servo
//               ::
//               O  :: 0
//               ::
//               ::   - Min Servo
//
///////////////////////////////////////////////////////////////////////////////////////////////

// Time Setups ////////////////////////////////////////////////////////////////////////////////
unsigned long t_now = 0;
unsigned long t_previous = 0;
unsigned long timeChange = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////

// Input Setups ////////////////////////////////////////////////////////////////////////////////
int x_Input = 500;                          // CHOSEN BY USER!!!
int y_Input = 501;                          // CHOSEN BY USER!!!

int x_pos;
int y_pos;

int x_error = 0;
int y_error = 0;

int x_error_prev = 0;
int y_error_prev = 0;

double x_errSum = 0;
double y_errSum = 0;

double x_maxSum = 10000;             // CHOSEN BY USER
double y_maxSum = 10000;             // CHOSEN BY USER

double x_errDer = 0;
double y_errDer = 0;

double PID_x = 0;
double PID_y = 0;

unsigned int PID_delay = 0;       // CHOSEN BY USER => delay to use after each PID output is given
///////////////////////////////////////////////////////////////////////////////////////////////


// Controller Setups //////////////////////////////////////////////////////////////////////////
double x_Kp = 0.8;                    // CHOSEN BY USER!!!      0.9         0.9
double x_Ki = 0.001;                    // CHOSEN BY USER!!!     0.00        0.00
double x_Kd = 550;                    // CHOSEN BY USER!!!      250         450

double y_Kp = 0.8;                    // CHOSEN BY USER!!!      0.8         0.8
double y_Ki = 0.001;                    // CHOSEN BY USER!!!     0.00        0.00
double y_Kd = 550;                    // CHOSEN BY USER!!!      250         300
////////////////////////////////////////////////////////////////////////////////////////////////



// X Error Value Setups ////////////////////////////////////////////////////////////////////////

// Max adc input is 1023
int x_Max_Error;
int x_Min_Error;

int x_max_Error_mag;

int x_servo_Max = 45;                      // CHOSEN BY USER!!!
int x_servo_Min= -45;                      // CHOSEN BY USER!!!

int x_us_Max, x_us_Min;

float x_Slope;
float x_Intercept;

int x_Max_adc = 1023;//932;                        // CHOSEN BY USER!!!
int x_Min_adc = 0;//115;                        // CHOSEN BY USER!!!
/////////////////////////////////////////////////////////////////////////////////////////////////


// Y Error Value Setups /////////////////////////////////////////////////////////////////////////
int y_Max_Error;
int y_Min_Error;

int y_max_Error_mag;

int y_servo_Max = 45;                    // CHOSEN BY USER!!!
int y_servo_Min = -45;                   // CHOSEN BY USER!!!

int y_us_Max, y_us_Min;

float y_Slope;
float y_Intercept;

int y_Max_adc = 1023;//772;                      // CHOSEN BY USER!!!
int y_Min_adc = 0;//230;                      // CHOSEN BY USER!!!
///////////////////////////////////////////////////////////////////////////////////////////////////





void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);

        xServo.attach(xServo_pin);
        yServo.attach(yServo_pin);

        xServo.writeMicroseconds(Servo_midpoint);
        yServo.writeMicroseconds(Servo_midpoint);


// X Error Value Setups

// Max adc input is 1023

x_Max_Error = x_Input - x_Max_adc;     // 1023 represents the max adc
x_Min_Error = x_Input - x_Min_adc;        // 0 represents the min adc

x_max_Error_mag = max(abs(x_Max_Error),abs(x_Min_Error));

x_Max_Error = -1*x_max_Error_mag;         // This is to ensure that both sides are balanced even when the x position is shifted
x_Min_Error = x_max_Error_mag;

x_Slope = (float)(x_servo_Max - x_servo_Min)/(x_Max_Error - x_Min_Error);
x_Intercept = (float)(x_servo_Max) - (x_Slope)*(x_Max_Error);               // Always going to be zero    

x_us_Max = (x_servo_Max/90)*(500) + 1500;
x_us_Min = (x_servo_Min/90)*(500) + 1500;

// Y Error Value Setups
y_Max_Error = y_Input - y_Max_adc;     // 1023 represents the max adc
y_Min_Error = y_Input - y_Min_adc;        // 0 represents the min adc

y_max_Error_mag = max(abs(y_Max_Error),abs(y_Min_Error));

y_Max_Error = -1*y_max_Error_mag;
y_Min_Error = y_max_Error_mag;


y_Slope = (float)(y_servo_Max - y_servo_Min)/(y_Max_Error - y_Min_Error);
y_Intercept = (float)(y_servo_Max) - (y_Slope)*(y_Max_Error); 

y_us_Max = (y_servo_Max/90)*(500) + 1500;
y_us_Min = (y_servo_Min/90)*(500) + 1500;




pinMode(13,INPUT);


//Serial.print("X Slope:   ");
//Serial.println(x_Slope,4);
//Serial.print("X Intercept:   ");
//Serial.println(x_Intercept);
//Serial.print("Y Slope:   ");
//Serial.println(y_Slope,4);
//Serial.print("Y Intercept:   ");
//Serial.println(y_Intercept);


while(digitalRead(13)==LOW)     // checks for a button to be pressed on my breadboard
{
  // DO NOTHING, WAIT FOR USER TO PRESS BUTTON TO START
  
  }

}


void loop() {
  // put your main code here, to run repeatedly:




while(true){
  
if(isTouched()==false){
  break;
}

x_pos = median_xposition();  

if(isTouched()==false){
  break;
}

y_pos = median_yposition();   


if(isTouched()==false){
  break;
}

Serial.print("x Position: ");
Serial.print(x_pos);
Serial.print("     ");
  Sprint("X Position:   ");
  Sprintln(x_pos);

Serial.print("y Position:   ");
Serial.println(y_pos);
  Sprint("Y Position: ");
  Sprintln(y_pos);


t_now = millis();   // checks current time


timeChange = t_now - t_previous;


x_error = x_Input - x_pos;       
  Sprint("X Error:   ");
  Sprintln(x_error);                   
y_error = y_Input - y_pos;
  Sprint("Y Error:   ");
  Sprintln(y_error);

x_errSum += (double)(x_error*timeChange);
    if(x_errSum > 0){
    
        x_errSum = min(x_errSum,x_maxSum);
    }
    else if(x_errSum < 0){
    
        x_errSum = min(abs(x_errSum),x_maxSum);
        x_errSum *= -1;
    }

y_errSum += (double)(y_error*timeChange);
    if(y_errSum > 0){
    
        y_errSum = min(y_errSum,y_maxSum);
    }
    else if(y_errSum < 0){
    
        y_errSum = min(abs(y_errSum),y_maxSum);
        y_errSum *= -1;
    }

    Sprint("X Error Sum:   ");
    Sprintln(x_errSum);
    Sprint("Y Error Sum:   ");
    Sprintln(y_errSum);

x_errDer = (double)(x_error-x_error_prev)/timeChange;
y_errDer = (double)(y_error-y_error_prev)/timeChange;

    Sprint("X Error Derivative:   ");
    Sprintln(x_errDer);
    Sprint("Y Error Derivative:   ");
    Sprintln(y_errDer);

PID_x = x_Kp*x_error + x_Ki*x_errSum + x_Kd*x_errDer;

if ( PID_x > x_Min_Error ) {              // if error exceeds more than minimum (largest positive error) reassign to lower limit

    PID_x = x_Min_Error;
}

else if ( PID_x < x_Max_Error ) {         // if error exceeds more that maximum (largest negative error) reassign to upper limit

    PID_x = x_Max_Error;
}

  Sprint("PID X:   ");
  Sprintln(PID_x);


PID_y = y_Kp*y_error + y_Ki*y_errSum + y_Kd*y_errDer;

if ( PID_y > y_Min_Error ) {              // if error exceds more than the minimum (largest positive error) reassign to lower limit

    PID_y = y_Min_Error;
}

else if ( PID_y < y_Max_Error ) {

    PID_y = y_Max_Error;
}

    Sprint("PID Y:   ");
    Sprintln(PID_y);

servo_output = servoOutput_x_error(PID_x);

//Serial.print("Servo X:   ");
//Serial.print(servo_output);
//Serial.print("    ");

xServo.writeMicroseconds(servo_output);

  Sprint("X Servo Output:   ");
  Sprintln(servoOutput_x_error(PID_x));

servo_output = servoOutput_y_error(PID_y);

//Serial.print("Servo Y:   ");
//Serial.print(servo_output);
//Serial.println();

yServo.writeMicroseconds(servo_output);

  Sprint("Y Servo Output:   ");
  Sprintln(servoOutput_y_error(PID_y));



x_error_prev = x_error;
y_error_prev = y_error;
t_previous = t_now;

delay(PID_delay);

}

// This is what happens if the ball is taken off of the touch screen
//Serial.println("Out of loop");
//x_errSum = 0;                       // Need to zero out the integral error
//y_errSum = 0;

//xServo.writeMicroseconds(Servo_midpoint);       // Resets servos to zero position to keep the plane level
//yServo.writeMicroseconds(Servo_midpoint);

//t_previous = millis();              // Updates previous time for when it jumps back in the while loop


}








int servoOutput_x_error (double x_err)
{
    float servo_output_x = (float)( x_Slope*x_err + x_Intercept);     // outputs what servo should be in degrees!

    Sprint("X Slope:   ");
    Sprintln(x_Slope);
    Sprint("X Intercept:   ");
    Sprintln(x_Intercept);

    servo_output_x = (int)((servo_output_x/90)*(500) + 1500);

    return servo_output_x;
}


int servoOutput_y_error (double y_err)                        //This has been changed for uS function!!!!
{
    float servo_output_y = (float)( y_Slope*y_err + y_Intercept);

    Sprint("Y Slope:   ");
    Sprintln(y_Slope);
    Sprint("Y Intercept:   ");
    Sprintln(y_Intercept);

    servo_output_y = (int)((servo_output_y/90)*(500) + 1500);      // The midpoint is 1500 

    return servo_output_y;
}



bool isTouched()
{

  
  bool touched = false;         // Assume it is not touched

  
  pinMode(Yplus,INPUT_PULLUP);  // Pull up connected to HIGH
  pinMode(Xminus,OUTPUT);       // Must be set to GRND

  pinMode(Xplus,INPUT);         // High Impedance
  pinMode(Yminus,INPUT);        // High Impedance
          
  digitalWrite(Xminus,LOW);     // if it is touched,Yplus is low
                                // if not touched, Yplus is high
  
  delayMicroseconds(1000);                     // Allow voltage to settle

  if (digitalRead(Yplus)==LOW)
  {
    touched = true;
  }


  //setPinInput(Xplus,Xminus,Yplus,Yminus);

  return touched;
}


void sort(int list[], int listlength)
{
  for (int top = listlength-1; top > 0 ; top --)      // after each iteration list[top] will have been sorted
  {
     int largestLoc = 0; // assume largest number is at list[0]
     for (int i = 0; i <= top ; i++)
     {
      if (list[i]>list[largestLoc])
      {
        largestLoc = i;
      }
     }
  
    int temp = list[top];
    list[top] = list[largestLoc];
    list[largestLoc] = temp;
  
  
  }


}


void setPinInput(int a, int b, int c, int d){

  pinMode(a,INPUT);
  pinMode(b,INPUT);
  pinMode(c,INPUT);
  pinMode(d,INPUT);
}


int median_xposition()
{

  int numsamples = 11;         // how many samples we want to get

  int midpoint = (numsamples-1)/2;    // calculates the midpoint in the array
  
  unsigned int x = 0;

  int xtst[numsamples];

  int us_delay = 20;

  
  for (int i = 0; i<numsamples; i++){
    
  pinMode (Xplus,OUTPUT);     // Set to 5V
  pinMode (Xminus,OUTPUT);    // Set to 0V
  pinMode (Yplus,INPUT);      // High Impedance
  pinMode (Yminus,INPUT);     // High Impedance

  digitalWrite(Xplus,HIGH);
  digitalWrite(Xminus,LOW);

  delayMicroseconds(us_delay);                   // Gives time for voltage to settle

  xtst[i]  = analogRead(Yplus);  

  /*pinMode(Xplus,INPUT);
  pinMode(Xminus,INPUT);
  pinMode(Yplus,INPUT);
  pinMode(Yminus,INPUT);

  delayMicroseconds(us_delay);*/

  }


  sort(xtst,numsamples);      // sorts in increasing order

  x = xtst[midpoint];         // get median value

  return x;
  
}



int median_yposition()
{
  int numsamples = 11;

  int midpoint = (int)(numsamples-1)/2;
  
  unsigned int y = 0;

  int ytst[numsamples];

  int us_delay = 20;
  

  for (int i = 0; i< numsamples; i++){ 
  
  pinMode (Yplus,OUTPUT);      // Set to 5V
  pinMode (Yminus,OUTPUT);     // Set to 0V
  pinMode (Xplus,INPUT);       // High Impedance
  pinMode (Xminus,INPUT);      // High Impedance
  
  digitalWrite(Yplus,HIGH);
  digitalWrite(Yminus,LOW);

  delayMicroseconds(us_delay);                     // Gives time for voltage to settle

  
  ytst[i] = analogRead(Xplus);
  

  /*pinMode(Xplus,INPUT);
  pinMode(Xminus,INPUT);
  pinMode(Yplus,INPUT);
  pinMode(Yminus,INPUT);

  delayMicroseconds(us_delay);*/

  }

 
  sort(ytst,numsamples);        // sorts in increasing order

  y = ytst[midpoint];           // get median value


  return y;

}


