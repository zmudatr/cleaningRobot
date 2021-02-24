/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */
 
 void sterowanieW()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  float predkoscLm = predkoscLmForward;
  float predkoscRm = predkoscRmForward;

  do
  {
    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);

    predkoscL = liczeniePredkosciLeft();
    if (predkoscL > 60) predkoscLm--;
    else if (predkoscL < 60) predkoscLm++;

    predkoscR = liczeniePredkosciRight();
    if (predkoscR > 60) predkoscRm--;
    else if (predkoscR < 60) predkoscRm++;

  } while (BTserial.available() == 0);

  sterowanieE();
  delay(500);
}

void sterowanieA() // skret Anti-clockwise
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  analogWrite(enL, predkoscLmBackward);
  analogWrite(enR, predkoscRmForward);

  while (BTserial.available() == 0)
  {  }
  
  sterowanieE();
  delay(500);
}

void sterowanieD()  // skret Clock-wise
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);

  analogWrite(enL, predkoscLmForward);
  analogWrite(enR, predkoscRmBackward);
  while (BTserial.available() == 0)
  {  }

  sterowanieE();
  delay(500);
}

void sterowanieS()
{
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);

  float predkoscLm = predkoscLmBackward;
  float predkoscRm = predkoscRmBackward;

  do
  {
    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);

    predkoscL = liczeniePredkosciLeft();
    if (predkoscL > 60) predkoscLm--;
    else if (predkoscL < 60) predkoscLm++;

    predkoscR = liczeniePredkosciRight();
    if (predkoscR > 60) predkoscRm--;
    else if (predkoscR < 60) predkoscRm++;

  } while (BTserial.available() == 0);

  sterowanieE();
  delay(500);
}

void sterowanie1()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  flagaStop = 0;

  while (flagaStop == 0)
  {
    if (BTserial.available()) flagaStop = 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) flagaStop = 1;
    }
    fuzzy2012();
  }
  sterowanieE();
  delay(500);
}

void sterowanie2()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  flagaStop = 0;

  while (flagaStop == 0)
  {
    if (BTserial.available()) flagaStop = 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) flagaStop = 1;
    }
    fuzzy2020();
  }
  sterowanieE();
  delay(500);
}

void sterowanieI()
{
  if (wentylatorState == LOW)     // if the wentylator is off turn it on and vice-versa:
    wentylatorState = HIGH;
  else
    wentylatorState = LOW;

  digitalWrite(wentylator, wentylatorState);
}

void sterowanieM()
{
  if (virtualWallState == LOW)     // if Virtual Wall is off then turn it on and vice-versa:
    virtualWallState = HIGH;
  else
    virtualWallState = LOW;
}

void sterowanieE()
{
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}

void sterowanieL()
{
  while (losowy() == 0) {    }
  
  sterowanieE();
  delay(500);
}

void sterowanieO()
{
  int x = 0;
  do
  {
    x = spiralny();
    
    if (x == 0)
      x = losowySpirala();
  }
  while (x == 0);
  sterowanieE();
  delay(500);
}

void sterowanieP()
{
  while ( snaking() == 0) {    }
  sterowanieE();
  delay(500);
}

//void sterowanieK()
//{
//  BTserial.println("Kalibracja");
//  kalibruj();
//}
