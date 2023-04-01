#include "Thread.h"
#include "Relay.h"
#include "PID_v1.h"

/*
 83: 100, 76: 94-96, 72: 90, 74: 92-93,
 
 92: 0.5bar, 94: 0.6bar
 */
#define defaultGoalTemp 96

#define groupTempReference 92 

#define boilerTempPin A1
#define groupTempPin A2
#define relayPin 12
#define groupTempPinHigh 8
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
  pinMode(groupTempPinHigh , OUTPUT);
  digitalWrite(groupTempPinHigh, HIGH);

  timerThread.onRun(updatePID);
  timerThread.setInterval(readingsIntervalMs);
  relay.setRelayMode(relayModeAutomatic);
  pidControl.SetMode(AUTOMATIC);
}

double readTemp(int pin) {
  int reading = analogRead(pin);
  double voltage = reading * (5.0 / 1024.0);
  double celcious = (voltage - 0.5) * 100;

  return celcious;
}

void updatePID() {
  float dutyCycle = relay.getDutyCyclePercent();
  pidInput = readTemp(boilerTempPin);

  if (pidInput > goal) {
    relay.setDutyCyclePercent(0);
    relay.setRelayPosition(relayPositionOpen);
  } else
    relay.setDutyCyclePercent(pidControlOutput / 255.0);

  pidControl.Compute();
  relay.loop();

  double groupTemp = readTemp(groupTempPin);

  Serial.print(pidInput);
  Serial.print(", ");
  Serial.print(groupTemp);
  Serial.print(", ");
  Serial.print(goal);
  Serial.print(", ");
  Serial.println(groupTempReference);
}

void loop() {
  delay(5);

  if (timerThread.shouldRun())
    timerThread.run();
}
