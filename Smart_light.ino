
#define LED_PIN 11       //LED PIN NUMBER
int light;

void setup()
{
  
  pinMode(LED_PIN,OUTPUT);
                                                                                                                                                                                                                                                                               Serial.begin(9600);
}
void loop()
{
  light = analogRead(A0);

  
  analogWrite(LED_PIN,light);


 
 
  
  Serial.println(light);


  delay(100);
}
