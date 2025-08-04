#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <math.h>

// --- Pin definitions ---
const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200 // Max sonar distance in cm

// --- Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servodown;
Servo servoup;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE);

// --- Q-Learning Parameters ---
const int ACTION_COUNT = 6;
const float LEARNING_RATE = 0.1; // Alpha
const float DISCOUNT_FACTOR = 0.9; // Gamma
const float EPSILON_START = 1.0;
const float EPSILON_END = 0.1;
const float EPSILON_DECAY = 0.9995; // A small decay rate

float epsilon = EPSILON_START;
float qTable[ACTION_COUNT] = {0.0}; // The Q-table, a single state with 6 actions

// --- Bot State and Action Definitions ---
struct Action {
    int down_angle;
    int up_angle;
};
Action actions[ACTION_COUNT] = {
    {0, 180}, {45, 135}, {90, 90},
    {0, 90}, {45, 180}, {90, 45}
};

float lastDistance = 0.0;
const float MIN_EFFECTIVE_DIFF = 2.0;
const int MOVE_DELAY_MS = 200; // Delay between servo movements

// --- Function Prototypes ---
float getDistance();
void moveServoSmooth(Servo &servo, int from, int to, int step, int pause);
void doAction(int actionIndex);
float getReward(float oldDistance, float newDistance);
int selectAction();
void updateQTable(int actionIndex, float reward);
void lcdDisplay(const String& line1, const String& line2);

// --- Main Functions ---
void setup() {
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();
    lcdDisplay("RL Crawler Bot", "Initializing...");

    servodown.attach(control_servo_down, 600, 2400);
    servoup.attach(control_servo_up, 600, 2400);
    servodown.write(0);
    servoup.write(180);

    delay(1000);
    lcdDisplay("Ready!", "Get last dist...");

    lastDistance = getDistance();
    if (lastDistance <= 0) {
        lcdDisplay("Sensor Error!", "Restarting...");
        delay(2000);
        ESP.restart(); // Restart if the sensor fails initially
    }
    lcdDisplay("Initial Dist:", String(lastDistance) + " cm");
    delay(2000);
}

void loop() {
    int selectedAction = selectAction();
    float oldDistance = lastDistance;
    
    doAction(selectedAction);
    
    lastDistance = getDistance();
    float reward = getReward(oldDistance, lastDistance);

    updateQTable(selectedAction, reward);
    
    epsilon = max(EPSILON_END, epsilon * EPSILON_DECAY);
    
    // Display current state
    lcdDisplay("Action: " + String(selectedAction), "Reward: " + String(reward, 2));
    
    // Serial debug output
    Serial.print("Epsilon: "); Serial.print(epsilon);
    Serial.print(" | Action: "); Serial.print(selectedAction);
    Serial.print(" | Reward: "); Serial.print(reward);
    Serial.print(" | Q_Val: "); Serial.println(qTable[selectedAction]);
    
    delay(1000);
}

// --- Helper Functions ---
float getDistance() {
    float d = sonar.ping_cm();
    return d > 0 ? d : -1.0;
}

void moveServoSmooth(Servo &servo, int from, int to, int step, int pause) {
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

void doAction(int actionIndex) {
    Action act = actions[actionIndex];
    int currentDown = servodown.read();
    int currentUp = servoup.read();
    
    // Smoothly move servos to the new position
    moveServoSmooth(servodown, currentDown, act.down_angle, 5, 10);
    moveServoSmooth(servoup, currentUp, act.up_angle, 5, 10);
    
    delay(MOVE_DELAY_MS);
}

float getReward(float oldDistance, float newDistance) {
    // Large penalty for sensor failure or no echo
    if (newDistance <= 0) {
        return -10.0;
    }
    
    float diff = newDistance - oldDistance;

    // A small dead zone to ignore minor sensor noise
    const float DISTANCE_NOISE_THRESHOLD = 0.5; // e.g., 0.5 cm
    
    // Check for significant forward movement
    if (diff > DISTANCE_NOISE_THRESHOLD) {
        // Reward is proportional to the distance gained.
        // The more it moves forward, the higher the reward.
        // We'll scale it so that a 10cm move gives a reward of 1.0, for example.
        // This is a form of shaping the reward.
        return diff * 0.5; // You can adjust this scaling factor
    } 
    // Check for significant backward movement
    else if (diff < -DISTANCE_NOISE_THRESHOLD) {
        // Penalty is also proportional to how much it moved backward
        return diff * 1.0; // diff is negative, so this is a penalty
    } 
    // If movement is within the noise threshold (neutral zone)
    else {
        // A very small, constant penalty for not making meaningful progress.
        // This encourages the bot to keep trying to move forward.
        return -0.05;
    }
}

int selectAction() {
    if ((float)random(1000) / 1000.0 < epsilon) {
        // Explore: take a random action
        return random(ACTION_COUNT);
    } else {
        // Exploit: take the best action from the Q-table
        int bestAction = 0;
        for (int i = 1; i < ACTION_COUNT; i++) {
            if (qTable[i] > qTable[bestAction]) {
                bestAction = i;
            }
        }
        return bestAction;
    }
}

void updateQTable(int actionIndex, float reward) {
    float maxQ = 0.0;
    for (int i = 0; i < ACTION_COUNT; i++) {
        if (qTable[i] > maxQ) {
            maxQ = qTable[i];
        }
    }
    
    // Q-learning update formula
    // Q(s, a) = Q(s, a) + alpha * (reward + gamma * max(Q(s')) - Q(s, a))
    qTable[actionIndex] += LEARNING_RATE * (reward + DISCOUNT_FACTOR * maxQ - qTable[actionIndex]);
}

void lcdDisplay(const String& line1, const String& line2) {
    lcd.clear();
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}