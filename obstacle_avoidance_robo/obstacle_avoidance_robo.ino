/////////////////////OBSTACLE_AVOIDANCE_ROBO/////////////////////////
// 25/5/2026 ->datatypes of all variables saving pin no.s is changed to uint8_t from int
#include <Servo.h>

//SEVO
 uint8_t servo_pin=11;
Servo myservo;


//ULTRASONIC_SENSOR
 uint8_t echo = 2;
 uint8_t trig = 12;

//motor A (left Motor)
 uint8_t In1=4;
 uint8_t In2=5;
 uint8_t EnA=3;

//motor B(Right Motor)
 uint8_t In3=6;
 uint8_t In4=7;
 uint8_t EnB=9;

int motorSpeed = 190;// ranges from 0 to 255 (PWM at EN pins of L298N)
int min_safe_dist = 20;//  (in cm ) minimum safe distance the robot has to maintain from obstacles
int turn_base_time = 350;//  time to overcome internal and external friction get the motor start spinnning
void setup() {

       //MOTORs
       pinMode(In1,OUTPUT);
       pinMode(In2,OUTPUT);
       pinMode(In3,OUTPUT);
       pinMode(In4,OUTPUT);

       pinMode(EnA,OUTPUT);
       pinMode(EnB,OUTPUT);


       //SERVO
       myservo.attach(servo_pin);


       //ULTRASONIC_SENSOR
       pinMode(echo, INPUT);
       pinMode(trig, OUTPUT);

}








void moveforward(){

       analogWrite(EnA,motorSpeed); 
       digitalWrite(In1,HIGH);
       digitalWrite(In2,LOW);

       analogWrite(EnB,motorSpeed); 
       digitalWrite(In3,HIGH);
       digitalWrite(In4,LOW);
       
}

void reverse(){

       analogWrite(EnA,motorSpeed); 
       digitalWrite(In1,LOW);
       digitalWrite(In2,HIGH);

       analogWrite(EnB,motorSpeed); 
       digitalWrite(In3,LOW);
       digitalWrite(In4,HIGH);
 
}

void turn_right(int strength){

       // left has to move faster than right
       analogWrite(EnA,motorSpeed); //LEFT
       digitalWrite(In1,HIGH);
       digitalWrite(In2,LOW);

       analogWrite(EnB,strength); //RIGHT
       digitalWrite(In3,LOW);
       digitalWrite(In4,HIGH);
 
}

void turn_left(int strength){

       // right has to move faster than left
       analogWrite(EnA,strength); //LEFT
       digitalWrite(In1,LOW);
       digitalWrite(In2,HIGH);

       analogWrite(EnB,motorSpeed); //RIGHT
       digitalWrite(In3,HIGH);
       digitalWrite(In4,LOW);
   
}

void stopRobot(){
       //setting both motor speed to 0

       analogWrite(EnA,0); //LEFT
       digitalWrite(In1,LOW);
       digitalWrite(In2,LOW);

       analogWrite(EnB,0); //RIGHT
       digitalWrite(In3,LOW);
       digitalWrite(In4,LOW);
      
}

//Determines distance of a obstacle using ultrasonic sensor 
//returns distance
int getdist(){

              int duration;
              int distance;

              digitalWrite(trig, LOW);
              delayMicroseconds(2);
              digitalWrite(trig, HIGH);
              delayMicroseconds(10);
              digitalWrite(trig, LOW);

              duration = pulseIn(echo, HIGH, 25000);
                     if (duration == 0) return 400;
              distance = duration / 58 ;

                     return distance;
    
} 





//Scan evironment at 5 angles and store the distance at these angles
int scanAngles[5] = { 20 , 60 , 90 , 120 , 160};
int scanned_dist[5];//scanned results at the given angles

void scanEnv(){
       for(int i = 0 ; i < 5 ; i++ )
       {
              myservo.write(scanAngles[i]);
              delay(100);
              scanned_dist[i] = getdist();
       }
       myservo.write(90); delay(300);

}


// find best direction --> direction (among the scanned 5 directions) in which obstacle detected is farthest

int findBestDir(){

              int bestDir_index = 2; // 90 deg as default best direction
              int best_dist = scanned_dist[2];

              for(int i = 0 ; i < 5 ; i++ )
                     {
                            if( scanned_dist[i] > best_dist )
                            {
                                   bestDir_index = i;
                                   best_dist = scanned_dist[i];
                            }
                     }
              return bestDir_index;       
}

void loop() {

/////////////////////////Check the front side //////////////////////////// 
       myservo.write(90);
       delay(200);
       int front_dist = getdist();

       if(front_dist > min_safe_dist)
       {
              moveforward();
              return;
       }

////////////////////// !!!! OBSTACLE-DETECTED !!!! //////////////
              stopRobot();
              delay(300);
              reverse();
              delay(1000);
              stopRobot();

/////////////////////// Scan surroundings//////////////////////////// 
scanEnv();// about 550 ms of time has lapsed in scanning environment

int bestDir_index = findBestDir();
int best_dist = scanned_dist[bestDir_index];
int best_angle = scanAngles[bestDir_index];

// if no direction is safe to move then take a reverse and scan again (restart loop)
if( best_dist < min_safe_dist + 5)
{
       reverse();
       delay(500);
       return;
}


//---------TURN CALCULATIONS-----------------
int angleOffset = best_angle - 90 ;
int turn_strength = map( abs(angleOffset) , 0 , 70 , 100 , 180);// how fast it will turn
int turn_duration = turn_base_time + ( abs(angleOffset) * 3 ) ;//it has to be dynamic since for angles turn duration cant be the same

       // Make a TURN based on the obstacle location 
       if (angleOffset > 0){//best path is towards left of center -> turn left
                     turn_left(turn_strength);
       }
       else{//turn right otherwise
                     turn_right(turn_strength);
       }
       delay(turn_duration);
       stopRobot();
       delay(120);

       // Set servo at 90 deg and reSCAN
        myservo.write(90);
        delay(200);
        front_dist = getdist();
        if(front_dist > min_safe_dist)
        {
               moveforward();
        }
              /*else   ---------------->> commented since this  else block is reduntant 
              {     return;
              }  */    

}





