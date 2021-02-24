/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 *
 * Algorithm implemented:
 * Farooq U., Hasan K. M., Usman Asad M., Saleh S. O.: Fuzzy Logic Based Wall Tracking
 * Controller for Mobile Robot Navigation, 7th IEEE Conference on Industrial Electronics and
 * Applications (ICIEA), 18-20 lipca 2012, Singapur.
 * 
 */
 void fuzzy2012()
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

  fuzzy->setInput(1, FS);
  fuzzy->setInput(2, BS);
  fuzzy->setInput(3, HS);

  fuzzy->fuzzify();

  float output1 = fuzzy->defuzzify(1);
  float output2 = fuzzy->defuzzify(2);
  
// ------- Debug ------- //
//  BTserial.print(" Fuzzy L ");
//  BTserial.print(output1);
//  BTserial.print(" Fuzzy R");
//  BTserial.println(output2);

  enableA_obecnePWM = map(output2, 0, 50, 0, 127);
  enableB_obecnePWM = map(output1, 0, 50, 0, 127);
  analogWrite(enR, enableA_obecnePWM);
  analogWrite(enL, enableB_obecnePWM);
}
