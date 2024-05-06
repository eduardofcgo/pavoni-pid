#include "Thread.h"
#include "Relay.h"
#include "PID_v1.h"

//96.5 - 92 at group
//99.5 - 95 at group
#define defaultGoalTemp 96.5

#define steamTemp 130
#define sensorAdjustement -4

#define boilerTempPin A3
#define relayPin 2
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
//PID pidControl(&pidInput, &pidControlOutput, &goal, 100.0, 0., 0., DIRECT);
//PID pidControl(&pidInput, &pidControlOutput, &goal, 2.0, 200., 75., DIRECT);
//PID pidControl(&pidInput, &pidControlOutput, &goal, 2.0, 0., 0., DIRECT);
PID pidControl(&pidInput, &pidControlOutput, &goal, 4.0, 0., 0., DIRECT);
//PID pidControl(&pidInput, &pidControlOutput, &goal, 32.0, 0., 0., DIRECT);


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

  return celcious;
}

void updatePID() {
  float dutyCycle = relay.getDutyCyclePercent();
  pidInput = readTemp(boilerTempPin) - sensorAdjustement;

  if (pidInput > goal) {
    relay.setDutyCyclePercent(0);
    relay.setRelayPosition(relayPositionOpen);
  } else {
    float newDutyCycle = pidControlOutput / 255;
    relay.setDutyCyclePercent(newDutyCycle);
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

  if (timerThread.shouldRun()) {
      Serial.print((int) pidInput);
      Serial.print(", ");
      Serial.println(relay.getDutyCyclePercent());

      timerThread.run();
  }

  delay(100);
}
