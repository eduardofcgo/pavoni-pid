#include "Thread.h"
#include "Relay.h"
#include "PID_v1.h"

#define groupTempPin A0
#define relayPin 12
#define relayControlSeconds 1
#define defaultGoalTemp 76 //83=100, 76=95.2, 72=90?
#define readingsIntervalMs 1000

double goal = defaultGoalTemp;
double pidInput;
double pidControlOutput;

Thread timerThread = Thread();
Relay relay(relayPin, relayControlSeconds);
PID pidControl(&pidInput, &pidControlOutput, &goal, 20.0, 10.0, 18.0, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(relayPin , OUTPUT);

  timerThread.onRun(updateReadings);
  timerThread.setInterval(readingsIntervalMs);
  relay.setRelayMode(relayModeAutomatic);
  pidControl.SetMode(AUTOMATIC);
}

double readGroupTemp() {
  int reading = analogRead(groupTempPin);
  double voltage = reading * (5.0 / 1024.0);
  double celcious = (voltage - 0.5) * 100;

  return celcious;
}

void updateReadings() {
  float dutyCycle = relay.getDutyCyclePercent();
  pidInput = readGroupTemp();

  if (pidInput > goal) {
    relay.setDutyCyclePercent(0);
    relay.setRelayPosition(relayPositionOpen);
  } else
    relay.setDutyCyclePercent(pidControlOutput / 255.0);

  pidControl.Compute();
  relay.loop();

  Serial.print("Goal: ");
  Serial.print(goal);
  Serial.print(" Input: ");
  Serial.print(pidInput);
  Serial.print(" dutyCycle: ");
  Serial.println(dutyCycle * 100);
}

void loop() {
  delay(100);

  if (timerThread.shouldRun())
    timerThread.run();
}