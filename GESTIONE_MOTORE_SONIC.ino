const short TRIG1=8;
const short ECHO1=9;
const short M1_1=0;
const short M1_2=1;
const short M2_1=3;
const short M2_2=4;

void setup() {
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(M1_1, OUTPUT);
  pinMode(M1_2, OUTPUT);
  pinMode(M2_1, OUTPUT);
  pinMode(M2_2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(TRIG1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
    
  unsigned long dur= pulseIn(ECHO1, HIGH);
  unsigned long dis=dur/29/2;
  
  //scrittua su seriale
  Serial.print("distance 1 :");
  Serial.print(dis);
  Serial.println(" cm");
  
  if(dis<30){
    digitalWrite(M1_1, HIGH);
    digitalWrite(M2_1, LOW);
    digitalWrite(M1_2, HIGH);
    digitalWrite(M2_2, LOW);
  }else{
    digitalWrite(M1_1, HIGH);
    digitalWrite(M2_1, HIGH);
    digitalWrite(M1_2, LOW);
    digitalWrite(M2_2, LOW);
  }
}
