#include <QTRSensors.h>
#include<util/delay.h>
#define NUM_SENSORS             5  // number of sensors used ///
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4}, 
  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int line_position=0; // value from 0-5000 to indicate position of line between sensor 0 - 4
//black is greater than 550 and white is less than 300
#define WT 300
#define BT 550

int pwm_a = 9;  //PWM control for motor outputs 1 and 2 is on digital pin 10  (Left motor)
int pwm_b = 10;  //PWM control for motor outputs 3 and 4 is on digital pin 11  (Right motor)
int speed_lt = 90;
int dir_a = 12;  //direction control for motor outputs 1 and 2 is on digital pin 12  (Left motor)
int dir_b = 6;  //direction control for motor outputs 3 and 4 is on digital pin 13  (Right motor)
int dir_ac = 4;  //direction control for motor outputs 1 and 2 is on digital pin 12  (Left motor)
int dir_bc = 7;  //direction control for motor outputs 3 and 4 is on digital pin 13  (Right motor)

// motor tuning vars for maze navigating
int calSpeed = 165;   // tune value motors will run while auto calibration sweeping turn (0-255)
int turnSpeed = 120;  // tune value motors will run while turning (0-255)
int turnSpeedSlow = 100;  // tune value motors will run as they slow down from turning cycle to avoid overrun (0-255)
int drivePastDelay = 180; // tune value in mseconds motors will run past intersection to align wheels for turn

// pid loop vars
float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;  // tune value in follow_line() function
//float ki = 0; // ki is not currently used
float kd =0;   // tune value in follow_line() function
int m1Speed=0;
int m2Speed=0;
int motorspeed=0;


// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your 
// maze design does not exceed the bounds of the array.
char path[100] = "";
unsigned char path_length = 0; // the length of the path


void calibrate_LDR()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to ind  icate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  delay(1000);
}

void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  pinMode(dir_ac, OUTPUT);
  pinMode(dir_bc, OUTPUT);
  /*pinMode(explore, INPUT);
  pinMode(solve, INPUT);*/
  Serial.begin(9600);
  calibrate_LDR();
  _delay_ms(500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//line following subroutine
// PD Control
void follow_line()  //follow the line  //added constant speed_lt
{
  lastError = 0;
  while(1)
  {
    line_position = qtra.readLine(sensorValues);
    Serial.println(line_position);
    switch(line_position)
    {
      // case 0 and case 5000 are used in the instructable for PD line following code.
      // kept here as reference.  Otherwise switch function could be removed.   
      //Line has moved off the left edge of sensor
      case 0:
             digitalWrite(dir_a, HIGH);
             digitalWrite(dir_ac, LOW); 
             analogWrite(pwm_a, 150);
             digitalWrite(dir_b, LOW);
             digitalWrite(dir_b, HIGH);  
             analogWrite(pwm_b, 90);
             //Serial.println("Rotate Left\n");
      break;
      // Line had moved off the right edge of sensor
      case 4000:
             digitalWrite(dir_a, LOW);
             digitalWrite(dir_ac, HIGH); 
             analogWrite(pwm_a, 90);
             digitalWrite(dir_b, HIGH);
             digitalWrite(dir_b, LOW);  
             analogWrite(pwm_b, 150);
             //Serial.println("Rotate Right\n");
      //break;  
      default:
        error = (float)line_position - 2000;
        // set the motor speed based on proportional and derivative PID terms
        // kp is the a floating-point proportional constant (maybe start with a value around 0.5)
        // kd is the floating-point derivative constant (maybe start with a value around 1)
        // note that when doing PID, it's very important you get your signs right, or else the
        // control loop will be unstable
        kp=.2;
        kd=3;    
        PV = kp * error + kd * (error - lastError);
        lastError = error;   
        //this codes limits the PV (motor speed pwm value)  
        // limit PV to 55
        if (PV > 55)
        {      
          PV = 55;
        }   
        if (PV < -55)
        {
          PV = -55;
        }        
        m1Speed = 200 + PV - speed_lt;
        m2Speed = 200 - PV - speed_lt;     
        //set motor speeds
        digitalWrite(dir_a, LOW);
        digitalWrite(dir_ac, HIGH);
        analogWrite(pwm_a, m2Speed);
        digitalWrite(dir_b, LOW);
        digitalWrite(dir_bc, HIGH);
        analogWrite(pwm_b, m1Speed);
        break;
    }
    // We use the inner 3 sensors (1 thru 4) topid works perfectly !
    // determine if there is a line straight ahead, and the
    // sensors 0 and 5 if the path turns.
    //as of now a big assumption that 
    if(sensorValues[0] < WT && sensorValues[1] < WT && sensorValues[2] < WT && sensorValues[3] < WT && sensorValues[4] < WT) // read raw ssensor values and chage them
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      //delay(1000);
      Serial.print(" dead end");
      return;
    }

    else if((sensorValues[0] > BT && sensorValues[1] > BT) || (sensorValues[3] > BT && sensorValues[4] > BT)) // change according to the raw sensor values
    {
      Serial.print(" intersection found");
      // Found an intersection.
      //delay(1000);
      return;
    }

  } 

} // end follow_line


/////////////////////////////////////////////////////////////////////////////////////////
// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if(found_left)
    return 'L';
  else if(found_straight)
    return 'S';
  else if(found_right)
    return 'R';
  else
    return 'B';
} // end select_turn

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn(char dir)
{
  switch(dir)
  {
    // Turn left 90deg
    case 'L':    
      digitalWrite(dir_a, HIGH);
      digitalWrite(dir_ac, LOW);     
      digitalWrite(dir_b, LOW);
      digitalWrite(dir_bc, HIGH); 
      
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow);       
      // find center
      while (/*line_position < 1200 && */sensorValues[1] < BT)  // tune - wait for line position to find near center
      {
        line_position = qtra.readLine(sensorValues);
      }    
      // stop both motors
      analogWrite(pwm_b, 0);  // stop right motor first to better avoid over run
      analogWrite(pwm_a, 0);  
      break;      
    // Turn right 90deg
      
    case 'R':        
      digitalWrite(dir_a, LOW);
      digitalWrite(dir_ac, HIGH);     
      digitalWrite(dir_b, HIGH);
      digitalWrite(dir_bc, LOW); 
      
      // slow down speed
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow);   
      // find center
      while (/*line_position > 3800 && */sensorValues[3] < BT)  // tune - wait for line position to find near center
      {
        line_position = qtra.readLine(sensorValues);
      }    
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);      
      break;   
    // Turn right 180deg to go back
      
    case 'B':    
      digitalWrite(dir_a, LOW);
      digitalWrite(dir_ac, HIGH); 
      digitalWrite(dir_b, HIGH);
      digitalWrite(dir_bc, LOW);
      
      // slow down speed*/
      analogWrite(pwm_a, turnSpeedSlow);
      analogWrite(pwm_b, turnSpeedSlow);      
      // find center
      while (/*line_position > 3800 && */sensorValues[3] < BT)  // tune - wait for line position to find near center
      {
        line_position = qtra.readLine(sensorValues);
      }     
      // stop both motors
      analogWrite(pwm_a, 0);  
      analogWrite(pwm_b, 0);           
      break;

    // Straight ahead
    case 'S':
      // do nothing
      break;
  }
} // end turn

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length-2] != 'B')
    return;
  
  int total_angle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[path_length-i])
    {
      case 'R':
        total_angle += 90;
  break;
      case 'L':
  total_angle += 270;
  break;
      case 'B':
  total_angle += 180;
  break;
    }
  }
  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;
  // Replace all of those turns with a single one.
  switch(total_angle)
  {
    case 0:
  path[path_length - 3] = 'S';
  break;
    case 90:
  path[path_length - 3] = 'R';
  break;
    case 180:
  path[path_length - 3] = 'B';
  break;
    case 270:
  path[path_length - 3] = 'L';
  break;
  }
  // The path is now two steps shorter.
  path_length -= 2;  
} // end simplify_path

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MazeSolve()
{
  // Loop until we have solved the maze.
  while(1)
  {
    // FIRST MAIN LOOP BODY  
    follow_line();
    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, while examining the current intersection
    unsigned char found_left=0;
    unsigned char found_straight=0;
    unsigned char found_right=0;
    
    // Now read the sensors and check the intersection type.
    line_position = qtra.readLine(sensorValues);

    // Check for left and right exits.
    if(sensorValues[4] > 300) ///
    found_right = 1;
    if(sensorValues[0] > 300) ///
    found_left = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    digitalWrite(dir_a, LOW);  
    digitalWrite(dir_ac, HIGH);
    
    digitalWrite(dir_b, LOW);
    digitalWrite(dir_bc, HIGH);  
    analogWrite(pwm_b, 130);
    analogWrite(pwm_a, 130);
    delay(drivePastDelay);
    digitalWrite(pwm_a, LOW);
    digitalWrite(pwm_b, LOW);
    delay(400);
    line_position = qtra.readLine(sensorValues); ///
    if(sensorValues[0] > BT || sensorValues[1] > BT || sensorValues[2] > BT || sensorValues[3] > BT || sensorValues[4] > BT)
    found_straight = 1;

    // Check for the ending spot.
    // If all six middle sensors are on dark black, we have
    // solved the maze.
    if(sensorValues[0] > 600 && sensorValues[1] > 600 && sensorValues[2] > 600 && sensorValues[3] > 600 && sensorValues[4] > 600)
  break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);
    Serial.print("turning");
    Serial.println(dir);
    // Make the turn indicated by the path.
    turn(dir);
    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length ++;
    // Simplify the learned path.
    simplify_path();
  }
  analogWrite(pwm_a, 0);  // stop both motors
  analogWrite(pwm_b, 0);
  //write indication of path solved
  for(int i=0 ; i<5; i++)
  {
    digitalWrite(13, HIGH);
    _delay_ms(250);
  }
  // Solved the maze!
  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  while(1)
  {
    analogWrite(pwm_a, 0);  // stop both motors
    analogWrite(pwm_b, 0);
    // delay to give you time to let go of the robot
    delay(10000); ///  remove  or decrease this to save time
    // Re-run the now solved maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    int i;
    for(i=0;i<path_length;i++)
    {
      // SECOND MAIN LOOP BODY  
      follow_line();
      // drive past intersection slightly slower and timed delay to align wheels on line
      digitalWrite(dir_a, LOW);  
      analogWrite(pwm_a, 200);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, 200);
      delay(drivePastDelay); // tune time to allow wheels to position for correct turning
      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_line();
      digitalWrite(dir_a, LOW);  
      analogWrite(pwm_a, 200);
      digitalWrite(dir_b, LOW);  
      analogWrite(pwm_b, 200);
      delay(drivePastDelay); // tune time to allow wheels to position for correct turning        
      // Now we should be at the finish!  Now move the robot again and it will re-run this loop with the solution again.  
  } // end running solved  
} // end MazeSolve
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  line_position = qtra.readLine(sensorValues);
  // begin maze solving
  MazeSolve(); // comment out and run serial monitor to test sensors while manually sweeping across line
  delay(250);
}
