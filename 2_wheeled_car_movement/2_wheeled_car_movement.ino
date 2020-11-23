int IN4 = 10;       
int IN3 = 9;     
int IN2 = 6;      
int IN1 = 5;    
int Speed=255;
void forward() 
  {
    analogWrite(IN1, 0);         
    analogWrite(IN2, Speed);
    analogWrite(IN3, 0);                                
    analogWrite(IN4, Speed);
  } 

void backward() 
  {
    Speed=-1*Speed;
    analogWrite(IN1, Speed);         
    analogWrite(IN2, 0);
    analogWrite(IN3, Speed);                                
    analogWrite(IN4, 0);
  } 
void setup() {

   Serial.begin(9600);
   pinMode(IN4, OUTPUT);      
   pinMode(IN3, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN1, OUTPUT);
   

}

void loop() {
    forward(); 
    delay(1000);

}
