/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */
 
 void kalibruj()
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  int predkosc = 60;

  bool flagaLm = false;
  bool flagaRm = false;

  int tempLm = predkoscLmForward;
  int tempRm = predkoscRmForward;
  analogWrite(enL, tempLm);
  analogWrite(enR, tempRm);
  delay(200);
  while (!(flagaLm) || !(flagaRm))
  {
    predkoscL = liczeniePredkosciLeft();
    predkoscR = liczeniePredkosciRight();

    if (flagaLm == 0)
    {
      if (predkoscL < predkosc)  tempLm++;
      else  if (predkoscL > predkosc)  tempLm--;
      else  if (predkoscL == predkosc) {
        flagaLm = 1;
      }
    }

    if (flagaRm == 0)
    {
      if (predkoscR < predkosc)  tempRm++;
      else  if (predkoscR > predkosc)  tempRm--;
      else  if (predkoscR == predkosc) {
        flagaRm = 1;
      }
    }
    analogWrite(enL, tempLm);
    analogWrite(enR, tempRm);
  }
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  predkoscLmForward = tempLm;
  predkoscRmForward = tempRm;

  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  tempLm = predkoscLmBackward;
  tempRm = predkoscRmBackward;

  flagaLm = false;
  flagaRm = false;

  analogWrite(enL, tempLm);
  analogWrite(enR, tempRm);
  delay(200);
  while (!(flagaLm) || !(flagaRm))
  {
    predkoscL = liczeniePredkosciLeft();
    predkoscR = liczeniePredkosciRight();

    if (flagaLm == 0)
    {
      if (predkoscL < predkosc)  tempLm++;
      else  if (predkoscL > predkosc)  tempLm--;
      else  if (predkoscL == predkosc) {
        flagaLm = 1;
      }
    }

    if (flagaRm == 0)
    {
      if (predkoscR < predkosc)  tempRm++;
      else  if (predkoscR > predkosc)  tempRm--;
      else  if (predkoscR == predkosc) {
        flagaRm = 1;
      }
    }
    analogWrite(enL, tempLm);
    analogWrite(enR, tempRm);
  }
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  predkoscLmBackward = tempLm;
  predkoscRmBackward = tempRm;
  
// -------------- Debug -------------- //
//  BTserial.print(" Koniec, wyniki kalibracji ");
//  BTserial.print(" predkoscLmForward: ");
//  BTserial.print(predkoscLmForward);
//  BTserial.print(" predkoscRmForward: ");
//  BTserial.print(predkoscRmForward);
//  BTserial.print(" predkoscLmBackward: ");
//  BTserial.print(predkoscLmBackward);
//  BTserial.print(" predkoscRmBackward: ");
//  BTserial.println(predkoscRmBackward);
  doBlink(5, 200);
}
