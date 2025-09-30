#define led1Pin 21
#define led2Pin 19
#define alarmPin 18
#define fan1Pin 16
#define fan2Pin 17
#define statusPin 5

char led1=0, led2=0, scream=0, fan1=0, fan2=0, status=0;
uint8_t cmd=0;
unsigned long screamStart = 0;

void action(void);

void setup() {
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(alarmPin, OUTPUT);
  pinMode(fan1Pin, OUTPUT);
  pinMode(fan2Pin, OUTPUT);
  pinMode(statusPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {

  if (scream)
  {
    if(millis() - currentMillis >= 5000)
    {
      currentMillis = millis();
      scream=0;
    }
  }

  if(Serial.available())
  {
    cmd = Serial.read();
    if(!cmd)//all off
  {
    led1=0;
    led2=0;
    scream=0;
    fan1=0;
    fan2=0;
  }
  if(cmd&0x01)//led on
  {
    led1=1;
    led2=1;
  }
  if(cmd&0x02)//fan 50%
  {
    fan1=127;
    fan2=127;
  }
  if(cmd&0x04)//fan 100%
  {
    fan1=255;
    fan2=255;
  }
  if(cmd&0x08)//alarm on
  {
    scream=1;
    currentMillis = millis();
  }
  if(cmd&0x10)//led off
  {
    led1=0;
    led2=0;
  }
  if(cmd&0x20)//fan off
  {
    fan1=0;
    fan2=0;
  }
  if(cmd&0x40)//green on
  {
    status=1;
  }
  if(cmd&0x80)//green off
  {
    status=0;
  }
  action();
  }

  digitalWrite(led1Pin, led1);
  digitalWrite(led2Pin, led2);
  digitalWrite(alarmPin, 0);
  delay(100);
  digitalWrite(led2Pin, 0);
  digitalWrite(led1Pin, 0);
  digitalWrite(alarmPin, scream);
  delay(300);
}

void action(void)
{
  analogWrite(fan1Pin, fan1);
  analogWrite(fan2Pin, fan2);
  digitalWrite(statusPin, status);
}

  