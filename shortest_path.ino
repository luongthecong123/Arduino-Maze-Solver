int mode = 0;

# define STOPPED 0
# define PID_FORWARD 1
# define NO_LINE 2
# define HORIZONTAL_LINE 3
# define POS_LINE 4
# define RIGHT_TURN 5
# define LEFT_TURN 6

////////////////////////////Sensor Pins/////////////////////////
  //Digital Pin
const int lineFollowsensor0 = 2;  //left most/5
const int lineFollowsensor1 = 3;  
const int lineFollowsensor2 = 4; 
const int lineFollowsensor3 = 5; 
const int lineFollowsensor4 = 12; //right most/5
  //Analog Pin
const int farRightsensorPin = A0;  
const int farLeftsensorPin = A5;   
///////////////////////Initial Values of sensors////////////////
int sensor[5]={0, 0, 0, 0, 0},sensorL=0,sensorR=0; 
int farRightsensor = 0;
int farLeftsensor = 0; 

//////////////////////////////Motors Pin///////////////////////  
  //right wheel
int ENA = 10;
int motorInput1 = 6;
int motorInput2 = 7;  
  //left wheel
int motorInput3 = 9;
int motorInput4 = 8;
int ENB = 11;

//////////////////////////Motor speed variables/////////////////
int mspeed = 29;              //PID_write_speed()
int rotspeed=60;
int rotspeedsharp=60;

float leftMotorSpeed=0;
float rightMotorSpeed=0;

int maxspeed =60;             //calculatePID(), PID_write_speed()

/////////////////////////PID variables//////////////////////////        calculatePID()
int Kp=12.5;
float Kd=30;
float Ki=0;
//initial values
int  P=0, I=0,D=0, PIDvalue=0;
float previousError=0,previousI=0, error=0;

///////////////////////Variables to find SECOND PASS///////////////////
char path[100];
int pathLength=0;
int index=0;
int i,runner, countB;
int second_pass_steps=0;
int state=1;  ///If state=1: loop will run, after the robot reaches the finish line, state will become 0 and escape the loop
void setup() {

  pinMode(lineFollowsensor0, INPUT);
  pinMode(lineFollowsensor1, INPUT);
  pinMode(lineFollowsensor2, INPUT);
  pinMode(lineFollowsensor3, INPUT);
  pinMode(lineFollowsensor4, INPUT);
  pinMode(farRightsensorPin, INPUT);  
  pinMode(farLeftsensorPin, INPUT);  
  
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);                     //setting serial monitor at rate of 9600
    Serial.print("Started!!!");
  delay(1500);
  startEngine();                          //low operating speed requires high speed kick start to escape inertia
}



///////////////////////////////////////////////////MAIN LOOP//////////////////////////////////////////

void loop()
{
  Serial.println("First Pass !!");
  first_pass(); //run the maze and return:shortest path, path[], length of shortest path
  i=0;
  stop_bot();
  delay(5000);
  /////////////PLACE THE ROBOT BACK TO START POINT////////////////
  Serial.println("Second Pass !!");
  state=1;
  startEngine(); 
  second_pass();
  stop_bot();delay(20000);  
}



///////////////////////////////////////////////////FIRST PASS////////////////////////////////////////////////
void first_pass() {
while(state){    //while(1) = while(true)
    errorRead();
    //Serial.println(" ");
    //Printsensor();        
    switch (mode)
  {
    case HORIZONTAL_LINE: //CONT_LINE
      //Printsensor();Serial.println("HORIZEN");
      forward_a_bit();                   // go foward a bit      
      errorRead();
      if (mode==HORIZONTAL_LINE){                           //stop and waiting for second pass
        stop_bot;
        printPath();shortenPath();delay(1000);
        state=0;        
        }              
      else 
      {         
         left90();                  //left90 itself has a delay function of 500 millisecs      
         path[index]='L';index=index+1; pathLength=pathLength+1;   //save left path to path[]
         engine_start_mini(); // engine start mini version
      }
      previousError = error;
      break;

    case NO_LINE:  

      forward_a_bit(); // go foward a bit
      left90();left90_2();              //left90 itself has a delay function of 500 millisecs
      path[index]='B';index=index+1; pathLength=pathLength+1;   //save back path to path[]
      engine_start_mini(); // engine start mini version
      previousError = 0;
      break;
    case RIGHT_TURN:
      //Printsensor();Serial.println("RIGHT");
      analogWrite(ENA,mspeed);analogWrite(ENB,mspeed);forward();delay(550);                 // go foward a bit    
      errorRead();  
      if (mode == NO_LINE){
         //Printsensor();Serial.print(" ");Serial.println("mode:NO_LINE");
         stop_bot();delay(200);
         right90();
         path[index]='R';index=index+1; pathLength=pathLength+1;   //save right path to path[]
         engine_start_mini(); // engine start mini version 
      }
      else if (mode==PID_FORWARD)
      {
         path[index]='S';index=index+1; pathLength=pathLength+1;   //save straight path to path[]
             
      }
      break;    
    case LEFT_TURN:

      forward_a_bit();                 // go foward a bit 
      left90();
      path[index]='L';index=index+1; pathLength=pathLength+1;   //save left path to path[]
      engine_start_mini(); // engine start mini version
      break;
    case PID_FORWARD:     

      calculatePID();
      PID_write_speed();
      forward();    
      break;     
  }
}  
}
/////////////////////////////////////READ SENSOR VALUES, RETURN ERROR AND TYPE OF LINE THE ROBOT CURRENTLY FACING/////////////////////////////
void errorRead(){
  sensor[0] = !digitalRead(lineFollowsensor0);
  sensor[1] = !digitalRead(lineFollowsensor1);
  sensor[2] = !digitalRead(lineFollowsensor2);
  sensor[3] = !digitalRead(lineFollowsensor3);
  sensor[4] = !digitalRead(lineFollowsensor4);
  sensorR   =  digitalRead(farRightsensorPin);
  sensorL   =  digitalRead(farLeftsensorPin);
  
  if((sensor[0]== 1 )&&(sensor[1]== 1 )&&(sensor[2]== 1 )&&(sensor[3]== 1 )&&(sensor[4]== 1 ))       {mode = HORIZONTAL_LINE; error = 0;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = NO_LINE;     error = 0;}  
  else if((sensorR == 1)&&(sensor[0] ==0 ))                                                          {mode = RIGHT_TURN;  error = 0;}
  else if((sensorL == 1)&&(sensor[4] ==0 ))                                                          {mode = LEFT_TURN;   error = 0;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 1 ))  {mode = PID_FORWARD; error = 4;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 1 ))  {mode = PID_FORWARD; error = 3;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 1 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = 2;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 1 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = 1;}
  else if((sensor[0]== 0 )&&(sensor[1]== 0 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = 0;}
  else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 1 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error =-1;}
  else if((sensor[0]== 0 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = -2;}
  else if((sensor[0]== 1 )&&(sensor[1]== 1 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = -3;}
  else if((sensor[0]== 1 )&&(sensor[1]== 0 )&&(sensor[2]== 0 )&&(sensor[3]== 0 )&&(sensor[4]== 0 ))  {mode = PID_FORWARD; error = -4;}

}

/////////////////////////////////////////////////////////PID CONTROLS//////////////////////////////////////////////////////////////
void calculatePID()
{
  P = error;
  if (leftMotorSpeed!=maxspeed || rightMotorSpeed!=maxspeed) //anti-winding 
  {I = I + previousI;previousI=I;}
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);  
  previousError = error;
}

void PID_write_speed()
{
  leftMotorSpeed = mspeed + PIDvalue;
  rightMotorSpeed = mspeed - PIDvalue;
  leftMotorSpeed=constrain(round(leftMotorSpeed), 0, maxspeed);
  rightMotorSpeed=constrain(round(rightMotorSpeed), 0, maxspeed);
  analogWrite(ENA,leftMotorSpeed);analogWrite(ENB,rightMotorSpeed);
    //Serial.print(0);Serial.print(",");Serial.print(80);Serial.print(",");
    //Serial.println(leftMotorSpeed);
    
}

///////////////////////////////////////////////////BASIC MOVEMENT CONTROLS/////////////////////////////////////////////
void forward_a_bit(){
  analogWrite(ENA,mspeed);analogWrite(ENB,mspeed);forward();delay(550);stop_bot();delay(200);
}

void startEngine(){
  analogWrite(ENA,60);analogWrite(ENB,60);
  forward();
  delay(200);
  analogWrite(ENA,0);analogWrite(ENB,0);
  delay(50);
}
void engine_start_mini(){
  analogWrite(ENA,60);analogWrite(ENB,60);forward();delay(100);
}
void forward()
{

  //Serial.println("forward");
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void stop_bot()
{
  //Serial.println("stop_bot");
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
void left90()
{
  //Serial.println("left90");
  analogWrite(ENA,rotspeed);analogWrite(ENB,rotspeed);
  sharpLeftTurn();
  delay(320);
  stop_bot();delay(500);
}
void left90_2()
{
  //Serial.println("left90");
  analogWrite(ENA,rotspeed);analogWrite(ENB,rotspeed);
  sharpLeftTurn();
  delay(320);
  stop_bot();delay(500);
}
void left180()
{
  //Serial.println("left180");
  analogWrite(ENA,rotspeed);analogWrite(ENB,rotspeed);
  sharpLeftTurn();
  delay(720);
  stop_bot();delay(500);  
}
void right90()
{
  //Serial.println("right90");
  analogWrite(ENA,rotspeed);analogWrite(ENB,rotspeed);
  sharpRightTurn();
  delay(340);
  stop_bot();delay(500);
}
void sharpLeftTurn() {
 //Serial.println("sharpLeftTurn");
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
 //Serial.println("sharpRightTurn");
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
///////////////////////////////////////////////////////////////PRINTING STUFFS/////////////////////////////////////////////////////
void Printsensor(){
  Serial.print(sensorL);Serial.print(" ");Serial.print(sensor[0]);Serial.print(sensor[1]);Serial.print(sensor[2]);Serial.print(sensor[3]);Serial.print(sensor[4]);
  Serial.print(" "); Serial.print(sensorR);Serial.print(" ");
}
void printPath(){               //print the path after first pass
  Serial.print("Final path: ");
  for (i=0;i<=pathLength-1;i++)
  {
    Serial.print(path[i]);
  }
}
////////////////////////////////////////////////////////////////SHORTEST_PATH///////////////////////////////////////////////////////
void del_replace(char path[],char replacement,int length,int position)  // replace 3 element in an array with 'replacement' 
{                                                                       //based on delete element in array algorithm used in Data structure
    int i=0;
    int pointer=0;
    int a=length;
    pointer=position;
    for(i=pointer-1;i<=a-3;i++)
    {
        path[i+1]=path[i+3];
    }
    //int new_size = a-2;
    path[pointer-1]=replacement;

}
void shortenPath(){
while(true){
    for (i=0;i<=pathLength-1;i++)
    {
        if (path[i] == 'B') countB=countB+1; //check if B is gone
    }
    Serial.print("number of Bs: ");Serial.println(countB);
    if (countB != 0)
    {


        for (i=0;i<=pathLength-1;i++)  //index=B, look for behind & after
        {
            if (path[i] == 'B')
            {
                if (i==0) continue;  //skip first element
                else if (i==pathLength-1) break; //exit loop at end loop
                else
                {
                    if(path[i-1]=='L')
                    {
                        if(path[i+1]=='S')
                            {del_replace(path,'R',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='R')
                            {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='L')
                            {del_replace(path,'S',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }
                    if(path[i-1]=='R')
                    {
                        if(path[i+1]=='L')
                           {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }
                    if(path[i-1]=='S')
                    {
                        if(path[i+1]=='L')
                            {del_replace(path,'R',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                        else if(path[i+1]=='S')
                            {del_replace(path,'B',pathLength,i);countB=0;pathLength=pathLength-2;break;}
                    }

                }
            }
        }
    for (i=0;i<=pathLength-1;i++)
    {
        Serial.print(path[i]);
    }   Serial.println(" ");Serial.print("pathLength: ");Serial.println(pathLength);
    }
    else break;
}
Serial.println("---------------------------");
Serial.print("SHORTEST PATH: ");

    for (i=0;i<=pathLength-1;i++)
    {
        Serial.print(path[i]);
    }
    Serial.println("  ");
    Serial.print("--------------------------- ");  
}
////////////////////////////////////////////////////////////SECOND PASS//////////////////////////////////////////////////
void second_pass(){
  second_pass_steps = pathLength;
  
  while(state)
  {
    errorRead();
    switch(mode)
    {
      case HORIZONTAL_LINE:
        analogWrite(ENA,mspeed);analogWrite(ENB,mspeed);forward();delay(550);
        errorRead();
        if (mode==HORIZONTAL_LINE) {Serial.print("CONGRATULATIONS !!!");state=0;}
        else
        {
        if(i != second_pass_steps  ) {Serial.print("Path: ");Serial.println(path[i]);what_to_do(path[i]);i=i+1;}
        
        break;
      case RIGHT_TURN:
        analogWrite(ENA,mspeed);analogWrite(ENB,mspeed);forward();delay(550);
        if(i != second_pass_steps ) {Serial.print("Path: ");Serial.println(path[i]);what_to_do(path[i]);i=i+1;}
        
        break;
      case LEFT_TURN:
        analogWrite(ENA,mspeed);analogWrite(ENB,mspeed);forward();delay(550);
        if(i != second_pass_steps ) {Serial.print("Path: ");Serial.println(path[i]);what_to_do(path[i]);i=i+1;}

        break;
      case PID_FORWARD:
        calculatePID();
        PID_write_speed();
        forward();    
        break;   
      }
      
    }
  }  
}
void what_to_do(char instruction){
    if (instruction == 'L') {Serial.println("Turn Left here");stop_bot();delay(200);left90();engine_start_mini();}
    else if (instruction == 'R') {Serial.println("TurnRight here");stop_bot();delay(200);right90();engine_start_mini();}
    else if (instruction == 'S') {Serial.println("Go straight here");}  
}
