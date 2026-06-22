/*
 * FFAI Robothon 2026 - Arduino电机控制
 * 双路H桥驱动
 */

// 电机引脚
#define LEFT_IN1  7
#define LEFT_IN2  8
#define LEFT_EN   5

#define RIGHT_IN1 9
#define RIGHT_IN2 10
#define RIGHT_EN  6

// 传感器引脚（5路）
#define SENSOR_0 A0
#define SENSOR_1 A1
#define SENSOR_2 A2
#define SENSOR_3 A3
#define SENSOR_4 A4

// PID参数
float Kp = 20.0;
float Ki = 0.0;
float Kd = 14.0;

float prevError = 0;
float integral = 0;

// 速度参数
int baseSpeed = 75;
int turnSpeed = 60;

void setup() {
    Serial.begin(115200);
    
    // 电机引脚
    pinMode(LEFT_IN1, OUTPUT);
    pinMode(LEFT_IN2, OUTPUT);
    pinMode(LEFT_EN, OUTPUT);
    pinMode(RIGHT_IN1, OUTPUT);
    pinMode(RIGHT_IN2, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
    
    // 传感器引脚
    for (int i = 0; i < 5; i++) {
        pinMode(A0 + i, INPUT);
    }
    
    Serial.println("Robot Ready");
}

void loop() {
    // 读取传感器
    int values[5];
    for (int i = 0; i < 5; i++) {
        values[i] = analogRead(A0 + i);
    }
    
    // 计算加权误差
    float error = weightedError(values);
    
    // PID计算
    float correction = computePID(error);
    
    // 动态速度
    int speed = dynamicSpeed(error);
    
    // 差速控制
    int left = speed - correction;
    int right = speed + correction;
    
    // 设置电机
    setMotor(left, right);
    
    delay(10);
}

float weightedError(int values[]) {
    float weights[] = {-2, -1, 0, 1, 2};
    float error = 0;
    for (int i = 0; i < 5; i++) {
        error += weights[i] * values[i];
    }
    return error;
}

float computePID(float error) {
    integral += error;
    integral = constrain(integral, -100, 100);
    
    float derivative = error - prevError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    prevError = error;
    return output;
}

int dynamicSpeed(float error) {
    float absError = abs(error);
    if (absError < 1) return 85;
    else if (absError < 3) return 65;
    else return 45;
}

void setMotor(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    
    // 左电机
    digitalWrite(LEFT_IN1, left > 0);
    digitalWrite(LEFT_IN2, left <= 0);
    analogWrite(LEFT_EN, abs(left));
    
    // 右电机
    digitalWrite(RIGHT_IN1, right > 0);
    digitalWrite(RIGHT_IN2, right <= 0);
    analogWrite(RIGHT_EN, abs(right));
}
