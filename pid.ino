#include "Thread.h"
#include "Relay.h"
#include "PID_v1.h"

#define defaultGoalTemp 92
#define sensorAdjustement -15

#define boilerTempPin A3
#define relayPin 12

#define relayControlSeconds 0.5
#define readingsIntervalMs 500

double goal = defaultGoalTemp;
double pidInput;
double pidControlOutput;

Thread timerThread = Thread();
Relay relay(relayPin, relayControlSeconds);
PID pidControl(&pidInput, &pidControlOutput, &goal, 20.0, 10.0, 18.0, DIRECT);

void setup() {
  Serial.begin(9600);

  pinMode(relayPin , OUTPUT);

  timerThread.onRun(updatePID);
  timerThread.setInterval(readingsIntervalMs);
  relay.setRelayMode(relayModeAutomatic);
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

  Serial.println(pidInput);

  if (pidInput > goal) {
    relay.setDutyCyclePercent(0);
    relay.setRelayPosition(relayPositionOpen);
  } else {
    relay.setDutyCyclePercent(pidControlOutput / 255.0);
    relay.setRelayPosition(relayPositionClosed);
  }

  pidControl.Compute();
  relay.loop();
}

void loop() {
  delay(500);

  if (timerThread.shouldRun())
    timerThread.run();

}
