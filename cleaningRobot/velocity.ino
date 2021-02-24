/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */
 
 float liczeniePredkosciRight()
{
  cnt = 0;
  rpm = 0;
  unsigned long currentMillis = millis();
  unsigned long StartMillis = currentMillis;
  while (currentMillis - StartMillis < 100)     // licz kroki w xxx milisekundach
  {
    newPositionRight = encRight.read();
    if (newPositionRight != oldPositionRight) {
      oldPositionRight = newPositionRight;
      cnt = cnt + 1;
    }
    currentMillis = millis();
  }
//  rpm = cnt * 60UL / 3576;       // dla 1000 milisekund
  //  rpm = cnt * 150UL / 3576;   // dla 400 milisekund
  //  rpm = cnt * 300UL / 3576;   // dla 200 milisekund
    rpm = cnt * 600UL / 3576;       // dla 100 milisekund
  BTserial.print(" RPM R ");
  BTserial.println(rpm);
  return rpm;
}

float liczeniePredkosciLeft()
{
  cnt = 0;
  rpm = 0;
  unsigned long currentMillis = millis();
  unsigned long StartMillis = currentMillis;
  while (currentMillis - StartMillis < 100)     // licz kroki w 100 milisekundach
  {
    newPositionLeft = encLeft.read();
    if (newPositionLeft != oldPositionLeft) {
      oldPositionLeft = newPositionLeft;
      cnt = cnt + 1;
    }
    currentMillis = millis();
  }

//  rpm = cnt * 60UL / 3576;       // dla 1000 milisekund
  //  rpm = cnt * 150UL / 3576;   // dla 400 milisekund
  //  rpm = cnt * 300UL / 3576;   // dla 200 milisekund
    rpm = cnt * 600UL / 3576;       // dla 100 milisekund
  BTserial.print(" RPM L ");
  BTserial.print(rpm);
  return rpm;
}



//float sterujPredkosciaRightTestKalibracji(float oczekiwana_predkosc, float smieci)
//{
//  // aktualnyPWM niepotrzebny
//  //float predkosc = liczeniePredkosciRight();
//  //  BTserial.print(" RPM R ");
//  //  BTserial.print(predkosc);
//  int RPMtoPWMmap = map(oczekiwana_predkosc, 0, 50, 0, 127);
//  BTserial.print(" OP: ");
//  BTserial.print(oczekiwana_predkosc);
//  BTserial.print(" PWM R ");
//  BTserial.println(RPMtoPWMmap);
//  return RPMtoPWMmap;
//}
//
//float sterujPredkosciaLeftTestKalibracji(float oczekiwana_predkosc, float smieci)
//{
//  // float predkosc = liczeniePredkosciLeft();
//  //  float PWM = aktualnyPWM;
//  //  BTserial.print(" RPM L ");
//  //  BTserial.print(predkosc);
//  int RPMtoPWMmap = map(oczekiwana_predkosc, 0, 50, 0, 127);
//  BTserial.print(" OczekP: ");
//  BTserial.print(oczekiwana_predkosc);
//  BTserial.print(" PWM L ");
//  BTserial.println(RPMtoPWMmap);
//  return RPMtoPWMmap;
//}
