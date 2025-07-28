#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>

// Pin definitions
const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200 // Max sonar distance in cm

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servodown;
Servo servoup;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

// Action definition
struct Action {
    int down;
    int up;
};
const int ACTION_COUNT = 6;
Action actions[ACTION_COUNT] = {
    {0, 180}, {45, 135}, {90, 90},
    {0, 90}, {45, 180}, {90, 45}
};

int actionScores[ACTION_COUNT] = {0};
float lastDistance = 0;
int trainingRound = 0;
const int EXPLORATION_ROUNDS = 30;
const float DISTANCE_THRESHOLD = 1.0;
const float MIN_EFFECTIVE_DIFF = 2.0;

float getDistance() {
    float d = sonar.ping_cm();
    if (d == 0) {
        Serial.println("No echo.");
        return -1;
    }
    Serial.print("Distance: ");
    Serial.print(d);
    Serial.println(" cm");
    return d;
}

void healthCheck() {
    lcd.clear();
    lcd.print("Health Check");
    delay(500);

    float d = getDistance();
    lcd.clear();
    lcd.print(d > 0 ? "D: " + String(d) + "cm" : "Sensor Fail");
    delay(700);

    int downs[] = {0, 45, 90};
    int ups[] = {180, 90, 0};
    for (int i = 0; i < 3; i++) {
        servodown.write(downs[i]);
        delay(300);
        for (int j = 0; j < 3; j++) {
            servoup.write(ups[j]);
            delay(300);
        }
    }

    servodown.write(0);
    servoup.write(180);
    delay(400);
    lcd.clear();
    lcd.print("Check Done");
    delay(400);
}

void moveServoSmooth(Servo &servo, int from, int to, int step = 5, int pause = 10) {
    if (from == to) return;
    if (from < to) {
        for (int pos = from; pos <= to; pos += step) {
            servo.write(pos);
            delay(pause);
        }
    } else {
        for (int pos = from; pos >= to; pos -= step) {
            servo.write(pos);
            delay(pause);
        }
    }
}

void doTraining() {
    Action act;
    int selected = 0;

    if (trainingRound < EXPLORATION_ROUNDS) {
        selected = trainingRound % ACTION_COUNT;
    } else {
        int total = 0;
        for (int i = 0; i < ACTION_COUNT; i++)
            total += max(1, actionScores[i]);
        int r = random(total), cum = 0;
        for (int i = 0; i < ACTION_COUNT; i++) {
            cum += max(1, actionScores[i]);
            if (r < cum) {
                selected = i;
                break;
            }
        }
    }

    act = actions[selected];

    // Either smooth or direct move:
    servodown.write(act.down);
    delay(150);
    servoup.write(act.up);
    delay(250);

    float newDistance = getDistance();
    float diff = newDistance - lastDistance;

    lcd.clear();
    lcd.print("A");
    lcd.print(selected);
    lcd.print(": D=");
    lcd.print(diff, 1);

    if (newDistance > 0) {
        lcd.setCursor(0, 1);
        if (abs(diff) <= DISTANCE_THRESHOLD) {
            lcd.print("Neutral");
        } else if (diff >= MIN_EFFECTIVE_DIFF) {
            actionScores[selected] += 2;
            lcd.print("Reward ++");
        } else if (diff < -DISTANCE_THRESHOLD) {
            actionScores[selected]--;
            lcd.print("Penalty --");
        } else {
            lcd.print("Small Gain");
        }
        lastDistance = newDistance;
    } else {
        lcd.setCursor(0, 1);
        lcd.print("No Distance");
    }

    trainingRound++;
    delay(600);  // Slightly slower learning cycle
}

void doLearnedBehavior() {
    lcd.clear();
    lcd.print("Best Action");

    int bestIndex = 0;
    for (int i = 1; i < ACTION_COUNT; i++) {
        if (actionScores[i] > actionScores[bestIndex])
            bestIndex = i;
    }

    Action best = actions[bestIndex];
    servodown.write(best.down);
    delay(150);
    servoup.write(best.up);
    delay(250);

    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print(best.down);
    lcd.print(" U:");
    lcd.print(best.up);
    delay(500);  // Give enough time to observe
}

void setup() {
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("RL Worm Init");
    delay(600);

    servodown.attach(control_servo_down, 600, 2400);
    servoup.attach(control_servo_up, 600, 2400);
    servodown.write(0);
    servoup.write(180);

    lcd.clear();
    lcd.print("Setup Done");
    delay(500);

    healthCheck();
    lastDistance = getDistance();
    Serial.println("Initial distance: " + String(lastDistance));
}

void loop() {
    doTraining();
    delay(300);           // Short pause between cycles
    doLearnedBehavior();
    delay(500);           // Let the best action rest
}
