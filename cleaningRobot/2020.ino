/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */
 
 void fuzzy2020()
{
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

// ------- Debug ------- //
//  BTserial.print(" FS=");
//  BTserial.print(FS);
//  BTserial.print(" BS=");
//  BTserial.print(BS);
//  BTserial.print(" HS=");
//  BTserial.print(HS);

  fuzzyII->setInput(1, FS);
  fuzzyII->setInput(2, BS);
  fuzzyII->setInput(3, HS);

  fuzzyII->fuzzify();

  float output1 = fuzzyII->defuzzify(1);
  float output2 = fuzzyII->defuzzify(2);

// ------- Debug ------- //
//  BTserial.print(" Fuzzy L ");
//  BTserial.print(output1);
//  BTserial.print(" Fuzzy R");
//  BTserial.println(output2);

  enableA_obecnePWM = map(output2, 0, 50, 0, 127);
  enableB_obecnePWM = map(output1, 0, 50, 0, 127);
  analogWrite(enR, enableA_obecnePWM);
  analogWrite(enL, enableB_obecnePWM);

  if (output1 > 99) delay (1100);
}
