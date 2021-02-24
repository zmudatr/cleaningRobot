/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 */

void sterowanie(char bt_msg)
{
  digitalWrite(ledRedIrPin, LOW);
  switch (bt_msg) {
    case 'W':     // Forward
      sterowanieW();
      break;
    case 'A':     // Anti-Clockwise
      sterowanieA();
      break;
    case 'D':     // Clockwise
      sterowanieD();
      break;
    case 'S':     // Backward
      sterowanieS();
      break;
    case '1':     // alg. Fuzzy2012
      sterowanie1();
      break;
    case '2':     // alg. Fuzzy2020
      sterowanie2();
      break;
    case 'I':     // wentylator ON/OFF
      sterowanieI();
      break;
    case 'M':     // wirtualna sciana ON/OFF
      sterowanieM();
      break;
    case 'E':     // STOP
      sterowanieE();
      break;
    case 'L':     // alg. losowy
      sterowanieL();
      break;
    case 'O':     // alg. spiralny
      sterowanieO();
      break;
    case 'P':     // alg. Snaking
      sterowanieP();
      break;
    //    case 'K':     // Kalibracja
    //    sterowanieK();
    //    break;
    default:
      BTserial.print("Command: ");
      BTserial.print(bt_msg);
      BTserial.println(" not found.");
      break;
  }
}
