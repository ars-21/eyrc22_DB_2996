
void setup()
{
  Serial.begin(9600);
  Serial.println("TRSensor example");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

}


void loop()
{

  Serial.print(analogRead(A0));
  Serial.print("  ");
  Serial.print(analogRead(A1));
  Serial.print("  ");
  Serial.print(analogRead(A2));
  Serial.print("  ");
  Serial.print(analogRead(A3));
  Serial.print("  ");
  Serial.print(analogRead(A4));
  Serial.println();
}
