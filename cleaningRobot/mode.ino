/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */
 
 int losowy() {
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  do
  {
    if (BTserial.available() != 0) return 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) return 1;
    }

    analogWrite(enL, predkoscLmForward);
    analogWrite(enR, predkoscRmForward);
    HS = sonar[2].ping_median(3, 20) / 57;      // 3 tests, max 20cm,  NewPing.h US_ROUNDTRIP_CM -> 57
    if (!(sonar[2].check_timer()))   HS = 60;
  } while ( HS > 9);
  sterowanieE();
  delay(1000);

  randomSeed(analogRead(0));
  int kat = random(4000, 12000);
  int kierunek = random(100);

  FS = sonar[0].ping_cm();  
  sonar[0].timer_stop();
  if (!(sonar[0].check_timer()))   FS = 60;
  delay(PING_INTERVAL);

  BS = sonar[1].ping_cm();
  sonar[1].timer_stop();
  if (!(sonar[1].check_timer()))   BS = 60;

  if (FS < 5 || BS < 5) kierunek = 0;// if both sensor near wall then turn right

  if (kierunek < 50) // prawo
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
    obrot(kat, predkoscLmForward, predkoscRmBackward);
  }
  else // lewo
  {
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    obrot(kat, predkoscLmBackward, predkoscRmForward);
  }
  analogWrite(enL, 0);
  analogWrite(enR, 0);
  delay(700);
  return 0;
}



// ----------------------------  SPIRALNY ------------------- //
int  spiralny() {
  FS = sonar[0].ping_cm();
  sonar[0].timer_stop();
  if (!(sonar[0].check_timer()))   FS = 60;

  BS = sonar[1].ping_cm();
  sonar[1].timer_stop();
  if (!(sonar[1].check_timer()))   BS = 60;

  HS = sonar[2].ping_cm();
  sonar[2].timer_stop();
  if (!(sonar[2].check_timer()))   HS = 60;

  if (FS > 3 && BS > 3 && HS > 3)
  {
    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    int enRspeed = 33;
    int counter = 100;
    int x = 0;

    analogWrite(enL, predkoscLmForward);
    analogWrite(enR, enRspeed);
    while (FS > 3 && BS > 3 && HS > 3)
    {
      x++;
      if (BTserial.available() != 0) return 1;

      if (virtualWallState == true)
      {
        if (checkIR() == true) return 1;
      }

      FS = sonar[0].ping_cm();
      sonar[0].timer_stop();
      if (!(sonar[0].check_timer()))   FS = 60;
      delay(PING_INTERVAL);

      BS = sonar[1].ping_cm();
      sonar[1].timer_stop();
      if (!(sonar[1].check_timer()))   BS = 60;
      delay(PING_INTERVAL);

      HS = sonar[2].ping_cm();
      sonar[2].timer_stop();
      if (!(sonar[2].check_timer()))   HS = 60;

      if (x > counter)
      {
        counter = counter * 1.15;
        x = 0;
        enRspeed = enRspeed + 6;
        analogWrite(enR, enRspeed);
      }
    }
  }
  sterowanieE();
  return 0;
}


// ----------------------------  SNAKING ------------------- //
int snaking () {
  float predkoscLm = predkoscLmForward;
  float predkoscRm = predkoscRmForward;
  float predkoscL;
  float predkoscR;

  // jedz prosto
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  analogWrite(enL, predkoscLm);
  analogWrite(enR, predkoscRm);

  do
  { // prosto az do sciany
    HS = sonar[2].ping_median(3, 20) / 57;      // 3 tests, max 20cm,  NewPing.h US_ROUNDTRIP_CM -> 57
    if (!(sonar[2].check_timer()))   HS = 60;
    
    if (BTserial.available() != 0) return 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) return 1;
    }

    predkoscL = liczeniePredkosciLeft();
    if (predkoscL > 60) predkoscLm--;
    else if (predkoscL < 60) predkoscLm++;

    predkoscR = liczeniePredkosciRight();
    if (predkoscR > 60) predkoscRm--;
    else if (predkoscR < 60) predkoscRm++;

    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);
  }  while (HS > 12 );

  sterowanieE();
  delay(700);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  obrot(6500, predkoscLmForward, predkoscRmBackward);  
  delay(700);

  HS = sonar[2].ping_median(3, 20) / 57;      // 3 tests, max 20cm,  NewPing.h US_ROUNDTRIP_CM -> 57
  if (!(sonar[2].check_timer()))   HS = 60;
  if (HS < 10) return 1;  // end if obstacle near

  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  analogWrite(enL, predkoscLmForward);
  analogWrite(enR, predkoscRmForward);
  delay(1000); 
  sterowanieE();
  delay(700);

  digitalWrite(LM1, HIGH); // second turn 90 degrees
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);

  obrot(6450, predkoscLmForward, predkoscRmBackward);   // turn right 90 degrees
  delay(700);

  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  analogWrite(enL, predkoscLm);
  analogWrite(enR, predkoscRm);

  do {
    HS = sonar[2].ping_median(3, 20) / 57;      // 3 tests, max 20cm,  NewPing.h US_ROUNDTRIP_CM -> 57
    if (!(sonar[2].check_timer()))   HS = 60;
     
    if (BTserial.available() != 0) return 1;

    if (virtualWallState == true)
    {
      if (checkIR() == true) return 1;
    }
    
    predkoscL = liczeniePredkosciLeft();
    if (predkoscL > 60) predkoscLm--;
    else if (predkoscL < 60) predkoscLm++;

    predkoscR = liczeniePredkosciRight();
    if (predkoscR > 60) predkoscRm--;
    else if (predkoscR < 60) predkoscRm++;

    analogWrite(enL, predkoscLm);
    analogWrite(enR, predkoscRm);

  } while (HS > 12);

  sterowanieE();

  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  delay(700);
  
  obrot(6500, predkoscLmBackward, predkoscRmForward);   // turn left 90 degrees
  delay(700);

  HS = sonar[2].ping_median(3, 20) / 57;      // 3 tests, max 20cm,  NewPing.h US_ROUNDTRIP_CM -> 57
  if (!(sonar[2].check_timer()))   HS = 60;
  if (HS < 10) return 1;  // end if obstacle near

  digitalWrite(LM1, HIGH);   
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  analogWrite(enL, predkoscLmForward);
  analogWrite(enR, predkoscRmForward);
  delay(1000);    
  sterowanieE();
  delay(700);

  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  obrot(6500, predkoscLmBackward, predkoscRmForward);   // turn left 90 degrees
  delay(700);
  return 0;
}
