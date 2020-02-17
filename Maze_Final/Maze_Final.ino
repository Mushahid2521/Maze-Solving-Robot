#define maxLeftSpeed 250
#define maxRightSpeed 250


int trig=2;
int echo=4;
int led=53;
long duration,distance;

const int motorPin1A= 8;        //connect with input1 of L293D
const int motorPin1B =  9;       //connect with input2 of L293D
const int motorPin2A = 3;       //connect with input3 of L293D
const int motorPin2B =  4;

int sensor[8] = { 22,23,24,25,26,27,28,29 };
int sensorread[8] = { 0,0,0,0,0,0,0,0 };
int backsensor[2]={38,39};
int backsensorread[2]={0,0};

char mapping[400];
char result[400];
char path[400];
char L,S,B,R;
char b1,b2,b3,c;


int b=1,s=0;
int i=0,j=0,m=0,k,p;
bool isComplete= 0;

float avg=0;
float avgval=0;

float kp = 100;                                      // Max deviation = 3-1 = 2 ||  255/2 = 72                                       //no need of Ki
float kd =90;
float error = 0;
float preverror = 0;
float correction = 0;

float activesen=0;
int leftBaseSpeed = 100;
int rightBaseSpeed = 100;
int PWM_Right , PWM_Left;
void setup()
{
  Serial.begin(9600);

pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);
  pinMode(led,OUTPUT);

  
  pinMode(motorPin1A,OUTPUT);
  pinMode(motorPin1B,OUTPUT);
  pinMode(motorPin2A,OUTPUT);
  pinMode(motorPin2B,OUTPUT);
  for(int i=0; i<8; i++) {
    pinMode(sensor[i], INPUT);
  }
  pinMode(backsensor[0],INPUT);
  pinMode(backsensor[1],INPUT);
  
  digitalWrite(motorPin1A,LOW);
  digitalWrite(motorPin1B,LOW);
  digitalWrite(motorPin2A,LOW);
  digitalWrite(motorPin2B,LOW);

  digitalWrite(led,LOW);

  
  delay(1000);
}

void loop() {
  if(isComplete==0) {
    readsensor();
    if(sensorread[6]==0 || sensorread[7]==0) {
      int f=0;
      readsensor();
      while(sensorread[6]!=1) {
        readsensor();
        if(sensorread[1]==0 || sensorread[0]==0) {
          readsensor();
          while(sensorread[1]==0 || sensorread[6]==0) {
            readsensor();
            if(backsensorread[0]==0 && backsensorread[1]==0) {
              isComplete=1;
               standStill();
               delay(300);
               stopHere();
               delay(1000);
               turnAround(0);
               break;
              
            }
            moveForward();
            delay(10);
          }
          stopHere();
          delay(10);
          
          if ( sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 ) {
          mapping[m++]='R';
          turnRightless(0);
          pid();
          f=1;
          break;
        }
          else {
          mapping[m++]='R';
          turnRight(1);
          right(0);
          pid();
          f=1;
          break;
        }
        }
        moveForward();
      }
      if(f==0) {
          stopHere();
          delay(25);
          if(  sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 ) {
          mapping[m++]='R';
          turnRightless(0);
          pid();
      }
      else {
        mapping[m++]='R';
        turnRight(1);
        right(0);
        pid();
      }
    }
  }

  else if(sensorread[1]==0 || sensorread[0]==0) {
    int f=0;
    while(sensorread[1]!=1) {
       readsensor();
       if( sensorread[7]==0 || sensorread[6]==0 ) {
        readsensor();
        while( sensorread[1]==0 || sensorread[6]==0 ) {
          readsensor();
          if(backsensorread[0]==0 && backsensorread[1]==0) {
            isComplete=1;
           
            standStill();
            delay(300);
            stopHere();
            delay(1000);
            turnAround(0);
            break;
          }
          moveForward();   
        }
         if (  sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 ) {
              mapping[m++]='R';
              turnRightless(0);
              pid();
              f=1;
              break;
        }
         else {
               mapping[m++]='R';
               turnRight(1);
               right(0);
               pid();
               f=1;
              break;
        }
       }
       moveForward();
    }
    if(f==0) {
        stopHere();
        delay(25);
         readsensor();

      if ( sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 ) {
          mapping[m++]='L';
          turnLeftless(0);
          pid();
      }
      else {
        mapping[m++]='S';
        pid();
      }
    }
  }
   else if ( sensorread[0]==1 && sensorread[1]==1 && sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 && sensorread[6]==1 && sensorread[7]==1 ) {
    mapping[m++]='B';
    stopHere();
    delay(20);
    turnAround(0);
    pid();
  }
  else pid(); 
}
else if(isComplete==1){
     k=lengthmap(mapping);

        for(int i=0; i<k; i++ ) {                                   // Find Shortest path


        result[j]=mapping[i];
        while (1) {
            c=ans(result[j],result[j-1],result[j-2]);
            if( c !='F' ) {
                j-=2;
                result[j]= c;

            }
            else {
                j++;
                break;
            }
        }
    }
    int p=0;
    for(int i=j; i>=0; i--) {                   // Reverse it 
      if(result[i]=='L') {
            path[p]='R';
            p++;
        }
      else if(result[i]=='R') {
            path[p]='L';
            p++;
        }
      else if(result[i]=='S') {
       // blinkLed();
        path[p]='S';
        p++;
      }
    }

    int t=1;
    while(1) {
      readsensor();
      if((sensorread[7]==0 || sensorread[6]==0 )|| (sensorread[0]==0 || sensorread[1]==0)) {
          stopHere();
          while( sensorread[0]!=1 || sensorread[7]==0) {
            moveForward();
            readsensor();
          }
          analogWrite(motorPin1B, 0);
          digitalWrite(motorPin1A, LOW);
          analogWrite(motorPin2B, 0);
          digitalWrite(motorPin2A, LOW);
          delay(100);
          moveForward();
          delay(100);

          if(path[t]=='S') {
            blinkLed();
            moveForward();
            delay(150);
            stopHere();
            delay(40);
            pid();
            t++;
          }
          else {
            readsensor();
            if( sensorread[1]==1 && sensorread[2]==1 && sensorread[3]==1 && sensorread[4]==1 && sensorread[5]==1 && sensorread[6]==1) {
              if(path[t]=='R') {
              turnRight(0);
              pid();
              t++;
            }
            else if(path[t]=='L') {
              turnLeft(0);
              pid();
              t++;
             }
            }
           else {
            if(path[t]=='R') {
              turnRight(1);
              turnRight(0);
              pid();
              t++;
            }
            else if(path[t]=='L') {
              turnLeft(1);
              turnLeft(0);
              pid();
              t++;
            }
          }
      }
    }
    else pid();
    }
   } 
}

void readsensor(){
  for(int i;i<8;i++)
  {
    sensorread[i]=digitalRead(sensor[i]);
    Serial.print(sensorread[i]);
    if(sensorread[i]==0){
    avg += (i+1);  
    activesen++;
    }  
  }
  backsensorread[0]=digitalRead(backsensor[0]);
  backsensorread[1]=digitalRead(backsensor[1]);
  Serial.print(backsensorread[0]);
  Serial.print(backsensorread[1]);
  Serial.println();
 
  avgval=avg/activesen;
  //Serial.print(avgval);
  activesen=0;
  avg=0;
}

void pid(){
  readsensor();
  error = avgval - 4.5 ;
  correction = (kp*error)+ (kd*(error-preverror));
  preverror=error;
  if( correction > 255 ) { correction = 255.0; }
  if( correction < -255.0 ) { correction = -255.0; }
  
      PWM_Right = 0;
    PWM_Left = 0;
    // Serial.println(correction);
   
  if(correction >=0) // Turn right
    {
    PWM_Right = rightBaseSpeed- (int(correction)) ;
      PWM_Left = leftBaseSpeed ;
      
    }
        else // Turn left
    {
      PWM_Right = rightBaseSpeed;
      PWM_Left = leftBaseSpeed  + (int(correction));
    }
   // Serial.print(PWM_Right);
    //Serial.print("\t");
    //Serial.println(PWM_Left); 
   
   //Serial.println(error);
    if(PWM_Left > 255) PWM_Left = maxLeftSpeed;
    if(PWM_Right > 255) PWM_Right = maxRightSpeed;
    if(PWM_Left < -255) PWM_Left = maxLeftSpeed * (-1);
    if(PWM_Right < -255) PWM_Right = maxRightSpeed * (-1); 
    
    moveforward(PWM_Right,PWM_Left);
}

void moveforward(int x,int y)
{
  if(x >= 0 && y >= 0)
  {
    analogWrite(motorPin1A,x);
    digitalWrite(motorPin1B,LOW);
    analogWrite(motorPin2A,y);
    digitalWrite(motorPin2B,LOW);
  }
  else if(x < 0 && y >= 0)
  {
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin1B,abs(x));
    analogWrite(motorPin2A,y);
    digitalWrite(motorPin2B,LOW);
  }
  else if(x >= 0 && y < 0)
  {
    analogWrite(motorPin1A,x);
    digitalWrite(motorPin1B,LOW);
    digitalWrite(motorPin2A,LOW);
    analogWrite(motorPin2B,abs(y));
  } 
  /*analogWrite(motorPin1A,x);
  digitalWrite(motorPin1B,LOW);
  analogWrite(motorPin2A,y);
  digitalWrite(motorPin2B,LOW); */
//  else if(x < 0 && y < 0)
//  {
//    digitalWrite(motorPin1A,LOW);
//    analogWrite(motorPin1B,abs(x));
//    digitalWrite(motorPin2A,LOW);
//    analogWrite(motorPin2B,abs(y));
//  }
}

 void turnRight(int a) {
  //stopHere();
  //delay(50);
  moveForward();
  delay(200);
  readsensor();
  while(sensorread[4]!=a || sensorread[5]!=a) {
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin1B,120);
    analogWrite(motorPin2A,90);
    digitalWrite(motorPin2B,LOW);
    readsensor();
  }
  stopHere();
  delay(20);
 }

  void turnRightless(int a) {
  //stopHere();
  //delay(30);
  moveForward();
  delay(200);
  readsensor();
  while(sensorread[4]!=a || sensorread[5]!=a) {
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin1B,120);
    analogWrite(motorPin2A,90);
    digitalWrite(motorPin2B,LOW);
    readsensor();
  }
  stopHere();
  delay(20);
 }
 void right(int a) {
//  stopHere();
//  delay(50);
//  moveForward();
//  delay(200);
  readsensor();
  while(sensorread[4]!=a || sensorread[5]!=a) {
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin1B,120);
    analogWrite(motorPin2A,90);
    digitalWrite(motorPin2B,LOW);
    readsensor();
  }
  stopHere();
  delay(20);
 }

  void turnAround(int a) {
    while(sensorread[3]!=a && sensorread[4]!=a) {
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin1B,100);
    analogWrite(motorPin2A,70);
    digitalWrite(motorPin2B,LOW);
    readsensor();
  }
 }
   void moveForward() {
    analogWrite(motorPin1A,50);
    digitalWrite(motorPin1B,LOW);
    analogWrite(motorPin2A,50);
    digitalWrite(motorPin2B,LOW);
 }
  void standStill() {
  analogWrite(motorPin1A,0);
  digitalWrite(motorPin1B,LOW);
  analogWrite(motorPin2A,0);
  digitalWrite(motorPin2B,LOW);
 }

  void stepBack() {
  analogWrite(motorPin1B,100);
    digitalWrite(motorPin1A,LOW);
    analogWrite(motorPin2B,100);
    digitalWrite(motorPin2A,LOW);
 }

   void turnLeft(int a) {
    //stopHere();
    //delay(50);
    moveForward();
    delay(200);
  readsensor();
  while(sensorread[2]!=a ||sensorread[3]!=a  ) {
    digitalWrite(motorPin2A,LOW);
    analogWrite(motorPin2B,120);
    analogWrite(motorPin1A,90);
    digitalWrite(motorPin1B,LOW);
    readsensor();
  }
  stopHere();
  delay(20);
 }

    void turnLeftless(int a) {
    //stopHere();
    //delay(50);
    moveForward();
    delay(200);
  readsensor();
  while(sensorread[2]!=a ||sensorread[3]!=a  ) {
    digitalWrite(motorPin2A,LOW);
    analogWrite(motorPin2B,120);
    analogWrite(motorPin1A,90);
    digitalWrite(motorPin1B,LOW);
    readsensor();
  }
  stopHere();
  delay(20);
 }

    void stopHere(){
  analogWrite(motorPin1B, 60);
  digitalWrite(motorPin1A, LOW);
  analogWrite(motorPin2B, 60);
  digitalWrite(motorPin2A, LOW);
  }
    void blinkLed() {
    digitalWrite(led,HIGH);
    delay(100);
    digitalWrite(led,LOW);
  }

   char ans(char a1, char a2, char a3) {                         // Right hand    Left Hand

    if(a1=='R' && a2=='B' && a3=='L') return 'B';            // RBL= B          LBL=B
    else if(a1=='R' && a2=='B' && a3=='S') return 'L';       // RBS= L          LBS=R
    else if(a1=='L' && a2=='B' && a3=='R') return 'B';       // LBR= B          RBL=B
    else if(a1=='S' && a2=='B' && a3=='R') return 'L';       // SBR= L          SBL=R
    else if(a1=='S' && a2=='B' && a3=='S') return 'B';       // SBS= B          SBS=B
    else if(a1=='R' && a2=='B' && a3=='R') return 'S';       // RBR= S          LBL=S
    else return 'F';

}
int lengthmap(char a[]) {
    int i=0;
    while(a[i]!= '\0') i++;
    return i;
}
  
