#define ENA 5
#define ENB 10
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
char Data;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);     
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite(BL,HIGH);
  if (Serial.available()) {
    Data = Serial.read();
    Serial.println(Data);
    
    switch (Data) {
      case 'F':
        forward();
        Serial.write("x");
        //delay(100);
        Data = 0;
        
        break;
      case 'B':
        back();
        //delay(100);
        Data = 0;
       
        break;
      case 'R':
        right();
        //delay(100);
        Data = 0;
        
        break;
      case 'L':
        left();
        //delay(100);
        Data = 0;
        break;
      case 'S':
        stop();
        //delay(100);
        Data = 0;
        break;
        default:
        Data = 0;
        break;
    }
    
  }
}

void forward() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 100);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
}

void back() {
  analogWrite(ENA, 200);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void right() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
}

void left() {
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, LOW);
}