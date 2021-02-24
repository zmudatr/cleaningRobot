/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */

void doBlink(int times, int Millis)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(ledRedIrPin, HIGH);   // turn the red LED on (HIGH is the voltage level)
    delay(Millis);                     // wait for a second

    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(ledRedIrPin, LOW);    // turn the red LED off by making the voltage LOW
    delay(Millis);                     // wait for a second
  }
}

bool checkIR()
{
  unsigned long currentMillis = millis();
  unsigned long StartMillis = currentMillis;
  bool flaga = false;
  digitalWrite(ledRedIrPin, HIGH);

  irrecv.enableIRIn();
  while (currentMillis - StartMillis < 60)
  {
    decode_results results;
    if (irrecv.decode(&results)) {
      //      BTserial.print(results.value);
      if (results.value == 48)
      {
        flaga = true;
        sterowanieE();
        StartMillis = 0;
      }
      else
      {
        flaga = false;
      }
    }
    currentMillis = millis();
  }
  
  if (flaga == false)
    digitalWrite(ledRedIrPin, LOW);
  else
    digitalWrite(ledRedIrPin, HIGH);


//    irrecv.disableIRIn();
  return flaga;
}

//  12(impulsów) * 298(przekladnia enkodera) = 3576 kroków = 1 rpm = 17.90 cm(obwod kola) dokladnie=== 17,907078125463
// 2r = 23,5 * PI = 73,827427359365    obwod roomby
// 73,827427359365 / 2 = 36,9137136796825
// 36,9137136796825 / 17,907078125463 = 2,06140350877193
// 2,06140350877193 * 3576 = 7371,578947368421\
// (-) opoznienia hamowania ->
//  obrot(6900, 0, 100);  - obrot ok 90 stopni /2 = 3450
void obrot (int kroki, int predkoscLm, int predkoscRm)
{
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  cnt = 0;
  while (cnt < kroki)
  {
    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);
    newPositionLeft = encLeft.read();
    if (newPositionLeft != oldPositionLeft) {
      oldPositionLeft = newPositionLeft;
      cnt = cnt + 1;
    }

    newPositionRight = encRight.read();
    if (newPositionRight != oldPositionRight) {
      oldPositionRight = newPositionRight;
      cnt = cnt + 1;
    }
  }
  analogWrite(enL, 0);
  analogWrite(enR, 0);
}

void obrotTime (int czasMillis, int predkoscLm, int predkoscRm)
{
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  unsigned long currentMillis = millis();
  unsigned long StartMillis = currentMillis;
  do
  {
    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);
    currentMillis = millis();
  } while (currentMillis - StartMillis < czasMillis);

  analogWrite(enL, 0);
  analogWrite(enR, 0);
}

int losowySpirala()
{
  randomSeed(analogRead(0));
  int kat = random(8000, 17000);
  int kierunek = random(100);

  // jesli czujniki boczne przy scianie to skret zawsze w prawo
  FS = sonar[0].ping_cm();
  sonar[0].timer_stop();
  if (!(sonar[0].check_timer()))   FS = 60;
  delay(PING_INTERVAL);

  BS = sonar[1].ping_cm();
  sonar[1].timer_stop();
  if (!(sonar[1].check_timer()))   BS = 60;

  if (FS < 5 || BS < 5) kierunek = 0;

  if (kierunek < 50) // kierunek obrotu w prawo
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    obrot(kat, predkoscLmForward, predkoscRmBackward);
  }
  else // kierunek obrotu w lewo
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    obrot(kat, predkoscLmBackward, predkoscRmForward);
  }
  sterowanieE();

  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  int odleglosc = random(4000, 10000);

  unsigned long currentMillis = millis();
  unsigned long StartMillis = currentMillis;
  while (currentMillis - StartMillis < odleglosc)     // licz kroki w xxx milisekundach
  {
    if (BTserial.available() != 0) return 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) return 1;
    }

    analogWrite(enL, predkoscLmForward);
    analogWrite(enR, predkoscRmForward);
    currentMillis = millis();

    HS = sonar[2].ping_cm();
    sonar[2].timer_stop();
    if (!(sonar[2].check_timer()))   HS = 60;

    if (HS < 60) odleglosc = 0; // w przypadku gdy do sciany/przeszkody mniej niz 60cm, zacznij wykonywac spirale
  }
  return 0;
}
