/*
 *  Author:
 *  Robert Zmuda Trzebiatowski 
 *  
 *  Designed and tested on ATmega2560
 *  
 *  The code below allows you to control the cleaning robot. It’s possible to manually control the robot, 
 *  activate the fan, virtual wall or run one of implemented algorithms - random, spiral, with a trajectory 
 *  in the shape of the letter "S" and two wall following algorithms that use fuzzy logic.
 *  
 *  The standard eFLL library (Embedded Fuzzy Logic Library) allows the construction of fuzzy logic systems
 *  in which the rules may contain only two linguistic variables. The functionality of the library has been
 *  extended to enable the construction of a rule containing three linguistic variables combined with the logical AND operator.
 *  
 *  For correctly working fuzzy logic system, you need to replace the eFLL library from project.
 *  
 */
 
#include <Fuzzy.h>
#include <NewPing.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <IRremote.h>

Fuzzy *fuzzy =  new Fuzzy();
Fuzzy *fuzzyII = new Fuzzy();

#define LM1 3
#define LM2 4
#define enL 2   // lewy

#define RM1 5
#define RM2 6
#define enR 7   // prawy

#define BTRX 10  // 11
#define BTTX 11  // 10
SoftwareSerial BTserial(BTRX, BTTX);

#define RECV_PIN 52                  // IR receiver
IRrecv irrecv(RECV_PIN);
bool virtualWallState = false;
bool flagaStop = false;
#define ledRedIrPin 53               // red LED pin

#define trigPin1 23                  // head sensor
#define echoPin1 22

#define trigPin3 25                  // front sensor
#define echoPin3 24

#define trigPin2 27                  // back sensor
#define echoPin2 26

#define PING_INTERVAL 33              // interwal pingu czujnikow
#define SONAR_NUM 3                   // liczba skanow 
#define MAX_DISTANCE 60               // Maximum distance (in cm) to ping.

#define wentylator 8                  // stan wentylatora on/off
int wentylatorState = LOW;

int enableA_obecnePWM = 0;
int enableB_obecnePWM = 0;

char bt_msg;

float predkoscL = 0;
float predkoscR = 0;

float predkoscLmForward = 72;
float predkoscRmForward = 84;
float predkoscLmBackward = 78;
float predkoscRmBackward = 78;

unsigned int cnt = 0;
unsigned long rpm = 0;

long oldPositionRight  = -999;
long newPositionRight = 0;
long oldPositionLeft  = -999;
long newPositionLeft = 0;

int HS = 0;
int FS = 0;
int BS = 0;

Encoder encLeft(A12, A13); // lewy
Encoder encRight(A14, A15); // prawy

unsigned long  pingTimer;

// -------------  Inputs Fuzzy 2012 ------------- //
FuzzySet *nearFs = new FuzzySet(0, 0, 30, 35);
FuzzySet *medFs = new FuzzySet(30, 35, 35, 40);
FuzzySet *farFs = new FuzzySet(35, 40, 60, 60);

FuzzySet *nearBs = new FuzzySet(0, 0, 30, 35);
FuzzySet *medBs = new FuzzySet(30, 35, 35, 40);
FuzzySet *farBs = new FuzzySet(35, 40, 60, 60);

FuzzySet *nearHs = new FuzzySet(0, 0, 30, 35);
FuzzySet *medHs = new FuzzySet(30, 35, 35, 40);
FuzzySet *farHs = new FuzzySet(35, 40, 60, 60);

// -------------  Inputs Fuzzy 2020 ------------- //
FuzzySet *nearFsII = new FuzzySet(0, 0, 10, 15);
FuzzySet *medFsII = new FuzzySet(10, 15, 15, 20);
FuzzySet *farFsII = new FuzzySet(15, 20, 60, 60);

FuzzySet *nearBsII = new FuzzySet(0, 0, 10, 15);
FuzzySet *medBsII = new FuzzySet(10, 15, 15, 20);
FuzzySet *farBsII = new FuzzySet(15, 20, 60, 60);

FuzzySet *nearHsII = new FuzzySet(0, 0, 30, 35);
FuzzySet *farHsII = new FuzzySet(30, 35, 60, 60);

// -------------  Outputs Fuzzy 2012 ------------- //
FuzzySet *slowLm = new FuzzySet(0, 0, 0, 25);
FuzzySet *medLm = new FuzzySet(0, 25, 25, 50);
FuzzySet *fastLm = new FuzzySet(25, 50, 50, 50);

FuzzySet *slowRm = new FuzzySet(0, 0, 0, 25);
FuzzySet *medRm = new FuzzySet(0, 25, 25, 50);
FuzzySet *fastRm = new FuzzySet(25, 50, 50, 50);

// -------------  Outputs Fuzzy 2020 ------------- //
FuzzySet *slowLmII = new FuzzySet(0, 0, 34, 35);
FuzzySet *medLmII = new FuzzySet(34, 35, 41, 50);
FuzzySet *fastLmII = new FuzzySet(41, 50, 50, 50);
FuzzySet *superFastLmII = new FuzzySet(99, 100, 100, 100);

FuzzySet *slowRmII = new FuzzySet(0, 0, 34, 35);
FuzzySet *medRmII = new FuzzySet(34, 35, 41, 50);
FuzzySet *fastRmII = new FuzzySet(41, 50, 50, 50);

NewPing sonar[SONAR_NUM] = {                                // Define a Newping array to measure the distance
  NewPing(trigPin3, echoPin3, MAX_DISTANCE),
  NewPing(trigPin2, echoPin2, MAX_DISTANCE),
  NewPing(trigPin1, echoPin1, MAX_DISTANCE)
};

void setup()
{
  BTserial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledRedIrPin, OUTPUT);
  pinMode(wentylator, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(ledRedIrPin, LOW);
  digitalWrite(wentylator, LOW);

  //irrecv.enableIRIn(); // Start the receiver

  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);

  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);

  FuzzyInput *fs = new FuzzyInput(1);
  fs->addFuzzySet(nearFs);
  fs->addFuzzySet(medFs);
  fs->addFuzzySet(farFs);
  fuzzy->addFuzzyInput(fs);

  FuzzyInput *fsII = new FuzzyInput(1);
  fsII->addFuzzySet(nearFsII);
  fsII->addFuzzySet(medFsII);
  fsII->addFuzzySet(farFsII);
  fuzzyII->addFuzzyInput(fsII);

  FuzzyInput *bs = new FuzzyInput(2);
  bs->addFuzzySet(nearBs);
  bs->addFuzzySet(medBs);
  bs->addFuzzySet(farBs);
  fuzzy->addFuzzyInput(bs);

  FuzzyInput *bsII = new FuzzyInput(2);
  bsII->addFuzzySet(nearBsII);
  bsII->addFuzzySet(medBsII);
  bsII->addFuzzySet(farBsII);
  fuzzyII->addFuzzyInput(bsII);

  FuzzyInput *hs = new FuzzyInput(3);
  hs->addFuzzySet(nearHs);
  hs->addFuzzySet(medHs);
  hs->addFuzzySet(farHs);
  fuzzy->addFuzzyInput(hs);

  FuzzyInput *hsII = new FuzzyInput(3);
  hsII->addFuzzySet(nearHsII);
  hsII->addFuzzySet(farHsII);
  fuzzyII->addFuzzyInput(hsII);

  FuzzyOutput *leftMotor = new FuzzyOutput(1);
  leftMotor->addFuzzySet(slowLm);
  leftMotor->addFuzzySet(medLm);
  leftMotor->addFuzzySet(fastLm);
  fuzzy->addFuzzyOutput(leftMotor);

  FuzzyOutput *leftMotorII = new FuzzyOutput(1);
  leftMotorII->addFuzzySet(slowLmII);
  leftMotorII->addFuzzySet(medLmII);
  leftMotorII->addFuzzySet(fastLmII);
  leftMotorII->addFuzzySet(superFastLmII);
  fuzzyII->addFuzzyOutput(leftMotorII);

  FuzzyOutput *rightMotor = new FuzzyOutput(2);
  rightMotor->addFuzzySet(slowRm);
  rightMotor->addFuzzySet(medRm);
  rightMotor->addFuzzySet(fastRm);
  fuzzy->addFuzzyOutput(rightMotor);

  FuzzyOutput *rightMotorII = new FuzzyOutput(2);
  rightMotorII->addFuzzySet(slowRmII);
  rightMotorII->addFuzzySet(medRmII);
  rightMotorII->addFuzzySet(fastRmII);
  fuzzyII->addFuzzyOutput(rightMotorII);


  // -------------  Building consequent Fuzzy 2012 ------------- //
  //consequent z 1-4 + 12-13 + 16 - LM fast, RM Slow
  FuzzyRuleConsequent *thenLmFastAndRmSlow = new FuzzyRuleConsequent();
  thenLmFastAndRmSlow->addOutput(fastLm);
  thenLmFastAndRmSlow->addOutput(slowRm);

  //consequent z 5-11 + 17  - LM fast, RM Med
  FuzzyRuleConsequent *thenLmFastAndRmMed = new FuzzyRuleConsequent();
  thenLmFastAndRmMed->addOutput(fastLm);
  thenLmFastAndRmMed->addOutput(medRm);

  //consequent z 14 + 18 - LM slow, RM fast
  FuzzyRuleConsequent *thenLmSlowAndRmFast = new FuzzyRuleConsequent();
  thenLmSlowAndRmFast->addOutput(slowLm);
  thenLmSlowAndRmFast->addOutput(fastRm);

  //consequent z 15 - LM fast, RM fast
  FuzzyRuleConsequent *thenLmFastAndRmFast = new FuzzyRuleConsequent();
  thenLmFastAndRmFast->addOutput(fastLm);
  thenLmFastAndRmFast->addOutput(fastRm);

  //consequent z 19 - LM med, RM fast
  FuzzyRuleConsequent *thenLmMedAndRmFast = new FuzzyRuleConsequent();
  thenLmMedAndRmFast->addOutput(medLm);
  thenLmMedAndRmFast->addOutput(fastRm);


  // -------------  Building consequent Fuzzy 2020 ------------- //
  //consequent z 1 - LM superFast, RM slow
  FuzzyRuleConsequent *thenLmSuperFastAndRmSlowII = new FuzzyRuleConsequent();
  thenLmSuperFastAndRmSlowII->addOutput(superFastLmII);
  thenLmSuperFastAndRmSlowII->addOutput(slowRmII);

  //consequent z 2, 3, 4  - LM fast, RM Slow
  FuzzyRuleConsequent *thenLmFastAndRmSlowII = new FuzzyRuleConsequent();
  thenLmFastAndRmSlowII->addOutput(fastLmII);
  thenLmFastAndRmSlowII->addOutput(slowRmII);

  //consequent z 5 - LM slow, RM fast
  FuzzyRuleConsequent *thenLmSlowAndRmFastII = new FuzzyRuleConsequent();
  thenLmSlowAndRmFastII->addOutput(slowLmII);
  thenLmSlowAndRmFastII->addOutput(fastRmII);

  //consequent z 6 - LM fast, RM fast
  FuzzyRuleConsequent *thenLmFastAndRmFastII = new FuzzyRuleConsequent();
  thenLmFastAndRmFastII->addOutput(fastLmII);
  thenLmFastAndRmFastII->addOutput(fastRmII);

  //consequent z 7 - LM med, RM fast
  FuzzyRuleConsequent *thenLmMedAndRmFastII = new FuzzyRuleConsequent();
  thenLmMedAndRmFastII->addOutput(medLmII);
  thenLmMedAndRmFastII->addOutput(fastRmII);

  //consequent z 8, 9, 10 - LM slow, RM med
  FuzzyRuleConsequent *thenLmSlowAndRmMedII = new FuzzyRuleConsequent();
  thenLmSlowAndRmMedII->addOutput(slowLmII);
  thenLmSlowAndRmMedII->addOutput(medRmII);

  // -------------Building FuzzyRule 2012 ------------- //
  //1   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifHsNear = new FuzzyRuleAntecedent();
  ifHsNear->joinSingle(nearHs);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifHsNear, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule1);

  //2   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsNearAndHsMed = new FuzzyRuleAntecedent();
  ifFsNearAndBsNearAndHsMed->joinWithAND(nearFs, nearBs, medHs);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifFsNearAndBsNearAndHsMed, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule2);

  //3   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsMedAndHsMed = new FuzzyRuleAntecedent();
  ifFsNearAndBsMedAndHsMed->joinWithAND(nearFs, medBs, medHs);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifFsNearAndBsMedAndHsMed, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule3);

  //4   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsFarAndHsMed = new FuzzyRuleAntecedent();
  ifFsNearAndBsFarAndHsMed->joinWithAND(nearFs, farBs, medHs);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifFsNearAndBsFarAndHsMed, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule4);

  //5   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsMedAndBsNearAndHsMed = new FuzzyRuleAntecedent();
  ifFsMedAndBsNearAndHsMed->joinWithAND(medFs, nearBs, medHs);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifFsMedAndBsNearAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule5);

  //6   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsMedAndBsMedAndHsMed = new FuzzyRuleAntecedent();
  ifFsMedAndBsMedAndHsMed->joinWithAND(medFs, medBs, medHs);
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifFsMedAndBsMedAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule6);

  //7   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsMedAndBsFarAndHsMed = new FuzzyRuleAntecedent();
  ifFsMedAndBsFarAndHsMed->joinWithAND(medFs, farBs, medHs);
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, ifFsMedAndBsFarAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule7);

  //8   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsNearAndHsMed = new FuzzyRuleAntecedent();
  ifFsFarAndBsNearAndHsMed->joinWithAND(farFs, nearBs, medHs);
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, ifFsFarAndBsNearAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule8);

  //9   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsMedAndHsMed = new FuzzyRuleAntecedent();
  ifFsFarAndBsMedAndHsMed->joinWithAND(farFs, medBs, medHs);
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, ifFsFarAndBsMedAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule9);

  //10   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsFarAndHsMed = new FuzzyRuleAntecedent();
  ifFsFarAndBsFarAndHsMed->joinWithAND(farFs, farBs, medHs);
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, ifFsFarAndBsFarAndHsMed, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule10);

  //11   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsNearAndBsNearAndHsFar = new FuzzyRuleAntecedent();
  ifFsNearAndBsNearAndHsFar->joinWithAND(nearFs, nearBs, farHs);
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, ifFsNearAndBsNearAndHsFar, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule11);

  //12   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsMedAndHsFar = new FuzzyRuleAntecedent();
  ifFsNearAndBsMedAndHsFar->joinWithAND(nearFs, medBs, farHs);
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, ifFsNearAndBsMedAndHsFar, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule12);

  //13   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsFarAndHsFar = new FuzzyRuleAntecedent();
  ifFsNearAndBsFarAndHsFar->joinWithAND(nearFs, farBs, farHs);
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, ifFsNearAndBsFarAndHsFar, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule13);

  //14   LM Slow, RM Fast
  FuzzyRuleAntecedent *ifFsMedAndBsNearAndHsFar = new FuzzyRuleAntecedent();
  ifFsMedAndBsNearAndHsFar->joinWithAND(medFs, nearBs, farHs);
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, ifFsMedAndBsNearAndHsFar, thenLmSlowAndRmFast);
  fuzzy->addFuzzyRule(fuzzyRule14);

  //15   LM Fast, RM Fast
  FuzzyRuleAntecedent *ifFsMedAndBsMedAndHsFar = new FuzzyRuleAntecedent();
  ifFsMedAndBsMedAndHsFar->joinWithAND(medFs, medBs, farHs);
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, ifFsMedAndBsMedAndHsFar, thenLmFastAndRmFast);
  fuzzy->addFuzzyRule(fuzzyRule15);

  //16   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsMedAndBsFarAndHsFar = new FuzzyRuleAntecedent();
  ifFsMedAndBsFarAndHsFar->joinWithAND(medFs, farBs, farHs);
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, ifFsMedAndBsFarAndHsFar, thenLmFastAndRmSlow);
  fuzzy->addFuzzyRule(fuzzyRule16);

  //17   LM Fast, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsNearAndHsFar = new FuzzyRuleAntecedent();
  ifFsFarAndBsNearAndHsFar->joinWithAND(farFs, nearBs, farHs);
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, ifFsFarAndBsNearAndHsFar, thenLmFastAndRmMed);
  fuzzy->addFuzzyRule(fuzzyRule17);

  //18   LM Slow, RM Fast
  FuzzyRuleAntecedent *ifFsFarAndBsMedAndHsFar = new FuzzyRuleAntecedent();
  ifFsFarAndBsMedAndHsFar->joinWithAND(farFs, medBs, farHs);
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, ifFsFarAndBsMedAndHsFar, thenLmSlowAndRmFast);
  fuzzy->addFuzzyRule(fuzzyRule18);

  //19   LM Med, RM Fast
  FuzzyRuleAntecedent *ifFsFarAndBsFarAndHsFar = new FuzzyRuleAntecedent();
  ifFsFarAndBsFarAndHsFar->joinWithAND(farFs, farBs, farHs);
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, ifFsFarAndBsFarAndHsFar, thenLmMedAndRmFast);
  fuzzy->addFuzzyRule(fuzzyRule19);

  // ------------------ Building FuzzyRule 2020 ------------------ //
  //1   LM SuperFast, RM Slow
  FuzzyRuleAntecedent *ifHsNearII = new FuzzyRuleAntecedent();
  ifHsNearII->joinSingle(nearHsII);
  FuzzyRule *fuzzyIIRule1 = new FuzzyRule(1, ifHsNearII, thenLmSuperFastAndRmSlowII);
  fuzzyII->addFuzzyRule(fuzzyIIRule1);

  //2   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsNear = new FuzzyRuleAntecedent();
  ifFsNearAndBsNear->joinWithAND(nearFsII, nearBsII, farHsII);
  FuzzyRule *fuzzyIIRule2 = new FuzzyRule(2, ifFsNearAndBsNear, thenLmFastAndRmSlowII);
  fuzzyII->addFuzzyRule(fuzzyIIRule2);

  //3   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsMed = new FuzzyRuleAntecedent();
  ifFsNearAndBsMed->joinWithAND(nearFsII, medBsII, farHsII);
  FuzzyRule *fuzzyIIRule3 = new FuzzyRule(3, ifFsNearAndBsMed, thenLmFastAndRmSlowII);
  fuzzyII->addFuzzyRule(fuzzyIIRule3);

  //4   LM Fast, RM Slow
  FuzzyRuleAntecedent *ifFsNearAndBsFar = new FuzzyRuleAntecedent();
  ifFsNearAndBsFar->joinWithAND(nearFsII, farBsII, farHsII);
  FuzzyRule *fuzzyIIRule4 = new FuzzyRule(4, ifFsNearAndBsFar, thenLmFastAndRmSlowII);
  fuzzyII->addFuzzyRule(fuzzyIIRule4);

  //5   LM Slow, RM Fast
  FuzzyRuleAntecedent *ifFsMedAndBsNear = new FuzzyRuleAntecedent();
  ifFsMedAndBsNear->joinWithAND(medFsII, nearBsII, farHsII);
  FuzzyRule *fuzzyIIRule5 = new FuzzyRule(5, ifFsMedAndBsNear, thenLmSlowAndRmFastII);
  fuzzyII->addFuzzyRule(fuzzyIIRule5);

  //6   LM Fast, RM Fast
  FuzzyRuleAntecedent *ifFsMedAndBsMed = new FuzzyRuleAntecedent();
  ifFsMedAndBsMed->joinWithAND(medFsII, medBsII, farHsII);
  FuzzyRule *fuzzyIIRule6 = new FuzzyRule(6, ifFsMedAndBsMed, thenLmFastAndRmFastII);
  fuzzyII->addFuzzyRule(fuzzyIIRule6);

  //7   LM Med, RM Fast
  FuzzyRuleAntecedent *ifFsMedAndBsFar = new FuzzyRuleAntecedent();
  ifFsMedAndBsFar->joinWithAND(medFsII, farBsII, farHsII);
  FuzzyRule *fuzzyIIRule7 = new FuzzyRule(7, ifFsMedAndBsFar, thenLmMedAndRmFastII);
  fuzzyII->addFuzzyRule(fuzzyIIRule7);

  //8   LM Slow, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsNear = new FuzzyRuleAntecedent();
  ifFsFarAndBsNear->joinWithAND(farFsII, nearBsII, farHsII);
  FuzzyRule *fuzzyIIRule8 = new FuzzyRule(8, ifFsFarAndBsNear, thenLmSlowAndRmMedII);
  fuzzyII->addFuzzyRule(fuzzyIIRule8);

  //9   LM Slow, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsMed = new FuzzyRuleAntecedent();
  ifFsFarAndBsMed->joinWithAND(farFsII, medBsII, farHsII);
  FuzzyRule *fuzzyIIRule9 = new FuzzyRule(9, ifFsFarAndBsMed, thenLmSlowAndRmMedII);
  fuzzyII->addFuzzyRule(fuzzyIIRule9);

  //10   LM Slow, RM Med
  FuzzyRuleAntecedent *ifFsFarAndBsFarAndHsNotNear = new FuzzyRuleAntecedent();
  ifFsFarAndBsFarAndHsNotNear->joinWithAND(farFsII, farBsII, farHsII);
  FuzzyRule *fuzzyIIRule10 = new FuzzyRule(10, ifFsFarAndBsFarAndHsNotNear, thenLmSlowAndRmMedII);
  fuzzyII->addFuzzyRule(fuzzyIIRule10);

  pingTimer = millis(); // Start now.
}

void loop()
{
    if (BTserial.available() != 0)
    {
      bt_msg = BTserial.read();
           sterowanie(bt_msg);    
    }
}
