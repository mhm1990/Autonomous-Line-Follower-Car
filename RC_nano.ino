#define RC_IN 7 // RC ontroller ch3
#define mode 8  // To send RC mode status

void setup() 
{
  pinMode(RC_IN,INPUT);
  pinMode(mode, OUTPUT);
  Serial.begin(115200);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  //Serial.println(pulseIn(RC_IN,HIGH));
  if (pulseIn(RC_IN,HIGH)<1000)
  {
    //Serial.println(pulseIn(RC_IN,HIGH));
    digitalWrite(mode,LOW); //RC Mode
  }
  else
  {
    //Serial.println("1");
    digitalWrite(mode,HIGH); // Autonomous Mode
  }
    
}
