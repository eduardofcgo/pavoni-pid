#include "Thread.h"
#include "Relay.h"
#include "PID_v1.h"

#define defaultGoalTemp 93//92
#define steamTemp 150
#define sensorAdjustement -8//-8

#define boilerTempPin A3
#define relayPin 12
#define steamModePin 2
#define steamModePinHigh 3
#define steamModePinLow 4

#define relayPeriodSeconds 1
#define readingsIntervalMs 250

double goal = defaultGoalTemp;
double pidInput;
double pidControlOutput;

Thread timerThread = Thread();
Relay relay(relayPin, relayPeriodSeconds);
PID pidControl(&pidInput, &pidControlOutput, &goal, 2., 1., 1.8, DIRECT); //0, 1, 2
//PID pidControl(&pidInput, &pidControlOutput, &goal, 0., 2., 5., DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(relayPin , OUTPUT);
  pinMode(steamModePinHigh , OUTPUT);
  pinMode(steamModePinLow , OUTPUT);

  digitalWrite(steamModePinHigh, HIGH);
  digitalWrite(steamModePinLow, LOW);

  timerThread.onRun(updatePID);
  timerThread.setInterval(readingsIntervalMs);
  relay.setRelayMode(relayModeAutomatic);
  relay.setDutyCyclePercent(0.);
  pidControl.SetMode(AUTOMATIC);
}

double readTemp(int pin) {
  int reading = analogRead(pin);
  double voltage = reading * (5.0 / 1024.0);
  double celcious = (voltage - 0.5) * 100;

  return celcious - sensorAdjustement;
}

void updatePID() {
  float dutyCycle = relay.getDutyCyclePercent();
  pidInput = readTemp(boilerTempPin);

  Serial.print(pidInput);
  Serial.print(", ");
  Serial.println(pidControlOutput / 255);

  if (pidInput > goal) {
    relay.setDutyCyclePercent(0);
    relay.setRelayPosition(relayPositionOpen);
  } else {
    relay.setDutyCyclePercent(pidControlOutput / 255);
    Serial.println("wowow!!");
    Serial.println(relay.getDutyCyclePercent());
  }

  pidControl.Compute();
  relay.loop();
}

bool isSteamMode() {
  return digitalRead(steamModePin);
}

void loop() {
  if (isSteamMode())
    goal = steamTemp;
  else
    goal = defaultGoalTemp;

  if (timerThread.shouldRun())
    timerThread.run();

  delay(100);
}
