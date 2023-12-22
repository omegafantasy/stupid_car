#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <string.h>

#include "Emakefun_MotorDriver.h"
#include "Openjumper_IICMotorDriver.h"

#define TrigL 2
#define EchoL 3

#define TrigR 7
#define EchoR 8

#define Blocked LOW
#define Unimpeded HIGH

#define InfLF 5
#define InfRF 6
// #define InfLB 7
// #define InfRB 8

#define Forw 1
#define Back 2
#define Left 3
#define Right 4
#define LForw 5
#define RForw 6
#define LBack 7
#define RBack 8
#define LSpin 9
#define RSpin 10

#define LLight 3
#define MLight 2
#define RLight 1

#define WINDOW 10
#define ACTWINDOW 50
#define ACTTHRES 5

#define THRES1 60
#define THRES2 40
#define THRES3 20

int maxs = 3000;  // 最大速度

PS2X *ps2x;
Openjumper_IICMotorDriver pwm = Openjumper_IICMotorDriver();
Emakefun_MotorDriver mMotorDriver = Emakefun_MotorDriver(0x60, MOTOR_DRIVER_BOARD_V5);
Adafruit_NeoPixel rgb_display_A1 = Adafruit_NeoPixel(3, A1, NEO_GRB + NEO_KHZ800);
// SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

float checkDistance(int idx) {
    /*
    超声波测距, idx=1左边，idx=2右边，返回距离(cm）
    注意需要防两个超声波干扰
    设置了最大超时时间为20ms，若超时则返回100cm
    */
    float distance;
    if (idx == 1) {
        digitalWrite(TrigL, LOW);
        delayMicroseconds(2);
        digitalWrite(TrigL, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigL, LOW);
        distance = pulseIn(EchoL, HIGH, 1000 * 20) / 58.00;
        if (distance == 0) {
            distance = 100;
        }
    } else {
        digitalWrite(TrigR, LOW);
        delayMicroseconds(2);
        digitalWrite(TrigR, HIGH);
        delayMicroseconds(10);
        digitalWrite(TrigR, LOW);
        distance = pulseIn(EchoR, HIGH, 1000 * 20) / 58.00;
        if (distance == 0) {
            distance = 100;
        }
    }
    delay(20);
    return distance;
}

int infSensor(byte pin) {
    /*
    红外传感器,返回Blocked或Unimpeded
    */
    int data = digitalRead(pin);
    return data;
}

void setColor(int idx, byte r, byte g, byte b) {
    /*
    设置RGB灯颜色，调试用
    */
    rgb_display_A1.setPixelColor(idx - 1, (((r / 20) & 0xffffff) << 16) | (((g / 20) & 0xffffff) << 8) | (b / 20));
    rgb_display_A1.show();
}

void setMotor(int speed, int dir) {
    /*
    设置电机转速，dir为方向,参考上面的宏定义
    旋转转速恒定为2000
    */
    switch (dir) {
        case Forw:
            pwm.setAllMotor(speed);
            break;
        case Back:
            pwm.setAllMotor(-speed);
            break;
        case Left:
            pwm.setAllMotor(-speed, speed, -speed, speed);
            break;
        case Right:
            pwm.setAllMotor(speed, -speed, speed, -speed);
            break;
        case LForw:
            pwm.setAllMotor(0, speed, 0, speed);
            break;
        case RForw:
            pwm.setAllMotor(speed, 0, speed, 0);
            break;
        case LBack:
            pwm.setAllMotor(-speed, 0, -speed, 0);
            break;
        case RBack:
            pwm.setAllMotor(0, -speed, 0, -speed);
            break;
        case LSpin:
            pwm.setAllMotor(-2000, -2000, 2000, 2000);
            break;
        case RSpin:
            pwm.setAllMotor(2000, 2000, -2000, -2000);
            break;
        default:
            break;
    }
}

int mode = 2;  // 0自动，1手动，2通信
int ld_list[WINDOW];
int rd_list[WINDOW];
int ldis, rdis, ld, rd, lf, rf, last_ldis, last_rdis;
bool act[ACTWINDOW];

void handleBlocked() {
    /*
    左前右前都被堵住
    */
    setMotor(maxs, Back);
    delay(100);
    int rand = random(0, 40);
    if (rand == 0) {
        setMotor(maxs, RSpin);
        delay(500);
    } else if (rand < 8) {
        setMotor(maxs, LSpin);
        delay(500);
    } else {
        if (ldis < rdis) {
            while (1) {
                setMotor(maxs, RSpin);
                delay(100);
                lf = infSensor(InfLF);
                rf = infSensor(InfRF);
                if (lf == Unimpeded || rf == Unimpeded) {
                    break;
                }
            }
        } else {
            while (1) {
                setMotor(maxs, LSpin);
                delay(100);
                lf = infSensor(InfLF);
                rf = infSensor(InfRF);
                if (lf == Unimpeded || rf == Unimpeded) {
                    break;
                }
            }
        }
    }
}

void Escape() {
    /*
    很可能车卡住了，尝试逃逸
    */
    setColor(MLight, 255, 255, 255);
    int randdis = random(0, 500);
    int rand = random(0, 8);
    if (rand == 0) {
        setMotor(maxs, RSpin);
        delay(500 + randdis);
    } else {
        setMotor(maxs, LSpin);
        delay(500 + randdis);
    }
}

void randomMove() {
    /*
    随机移动以尝试破除某些条件导致的死循环
    */
    setColor(MLight, 255, 255, 255);
    int rand = random(0, 40);
    int randdis = random(0, 200);
    if (rand == 0) {
        setMotor(maxs, LSpin);
        delay(300 + randdis);
    } else if (rand < 8) {
        setMotor(maxs, RSpin);
        delay(300 + randdis);
    } else if (rand < 16) {
        setMotor(maxs, Left);
        delay(300);
    } else if (rand < 24) {
        setMotor(maxs, Right);
        delay(300);
    } else {
        setMotor(maxs, Back);
        delay(300);
    }
}

void normalMove() {
    /*
    正常避障
    */
    int maxdis = ldis > rdis ? ldis : rdis;
    int mindis = ldis < rdis ? ldis : rdis;
    if (ldis > THRES1) {  // green
        setColor(LLight, 0, 255, 0);
    } else if (ldis > THRES2) {  // yellow
        setColor(LLight, 255, 255, 0);
    } else if (ldis > THRES3) {  // blue
        setColor(LLight, 0, 0, 255);
    } else {  // red
        setColor(LLight, 255, 0, 0);
    }
    if (rdis > THRES1) {  // green
        setColor(RLight, 0, 255, 0);
    } else if (rdis > THRES2) {  // yellow
        setColor(RLight, 255, 255, 0);
    } else if (rdis > THRES3) {  // blue
        setColor(RLight, 0, 0, 255);
    } else {  // red
        setColor(RLight, 255, 0, 0);
    }

    bool actflag = false;
    if (lf == Blocked && rf == Blocked) {
        setColor(MLight, 255, 255, 255);
        handleBlocked();
        actflag = true;
    } else if (lf == Blocked) {
        setColor(MLight, 0, 0, 255);
        setMotor(maxs, Right);
        delay(100);
        setMotor(maxs, RSpin);
        delay(50);
        actflag = true;
    } else if (rf == Blocked) {
        setColor(MLight, 255, 0, 0);
        setMotor(maxs, Left);
        delay(100);
        setMotor(maxs, LSpin);
        delay(50);
        actflag = true;
    } else if (mindis > THRES1) {
        setMotor(maxs, Forw);
        setColor(MLight, 0, 255, 0);
    } else if (mindis > THRES2) {
        setMotor(0.75 * maxs + 0.25 * maxs * (mindis - THRES2) / (THRES1 - THRES2), Forw);
        setColor(MLight, 0, 255, 0);
    } else if (ldis > THRES2 && rdis > THRES3) {
        setMotor(0.75 * maxs, LForw);
        setColor(MLight, 255, 255, 0);
    } else if (ldis > THRES3 && rdis > THRES2) {
        setMotor(0.75 * maxs, RForw);
        setColor(MLight, 0, 255, 255);
    } else if (mindis > THRES3) {
        if (ldis > rdis) {
            setMotor(0.6 * maxs + 0.15 * maxs * (mindis - THRES3) / (THRES2 - THRES3), LForw);
            setColor(MLight, 255, 255, 0);
        } else {
            setMotor(0.6 * maxs + 0.15 * maxs * (mindis - THRES3) / (THRES2 - THRES3), RForw);
            setColor(MLight, 0, 255, 255);
        }
    } else if (ldis <= THRES3 && rdis > THRES3) {
        setColor(MLight, 0, 0, 255);
        setMotor(maxs, Right);
        delay(100);
        setMotor(maxs, RSpin);
        delay(50);
        actflag = true;
    } else if (ldis > THRES3 && rdis <= THRES3) {
        setColor(MLight, 255, 0, 0);
        setMotor(maxs, Left);
        delay(100);
        setMotor(maxs, LSpin);
        delay(50);
        actflag = true;
    } else {
        setColor(MLight, 255, 255, 255);
        handleBlocked();
        actflag = true;
    }

    for (int i = 0; i < ACTWINDOW - 1; i++) {
        act[i] = act[i + 1];
    }
    act[ACTWINDOW - 1] = actflag;
}

void autoMove() {
    /*
    自动避障
    */
    lf = infSensor(InfLF);
    rf = infSensor(InfRF);
    ld = checkDistance(1);
    rd = checkDistance(2);

    for (int i = 0; i < WINDOW - 1; i++) {
        ld_list[i] = ld_list[i + 1];
        rd_list[i] = rd_list[i + 1];
    }
    ld_list[WINDOW - 1] = ld;
    rd_list[WINDOW - 1] = rd;

    // 超声波测距容易受干扰，取半个窗口内的平均值
    int avg_ld = 0, avg_rd = 0;
    for (int i = WINDOW / 2; i < WINDOW; i++) {
        avg_ld += ld_list[i];
        avg_rd += rd_list[i];
    }
    avg_ld /= (WINDOW - WINDOW / 2);
    avg_rd /= (WINDOW - WINDOW / 2);

    ldis = max(min(min(last_ldis + 3, avg_ld), ld), last_ldis - 10);  // 设置变化上下限
    rdis = max(min(min(last_rdis + 3, avg_rd), rd), last_rdis - 10);
    last_ldis = ldis;
    last_rdis = rdis;
    // int lb = infSensor(InfLB);
    // int rb = infSensor(InfRB);
    // Serial.print("ldis:");
    // Serial.print(ldis);
    // Serial.print(" rdis:");
    // Serial.print(rdis);
    // Serial.print(" ");
    // Serial.print(lf);
    // Serial.print(" ");
    // Serial.print(rf);
    // Serial.print("||");

    // 判断是否被卡住
    bool stopflag = true;
    int baseld = ld_list[0], baserd = rd_list[0];
    for (int i = 1; i < WINDOW; i++) {
        if (abs(ld_list[i] - baseld) > 1 || abs(rd_list[i] - baserd) > 1) {
            stopflag = false;
            break;
        }
    }

    // 判断是否陷入局部困境(一段时间内频繁异常避障)
    int actcount = 0;
    for (int i = 0; i < ACTWINDOW; i++) {
        if (act[i]) {
            actcount++;
        }
    }
    bool actflag = actcount >= ACTTHRES;

    if (stopflag) {
        randomMove();
    } else if (actflag) {
        Escape();
        for (int i = 0; i < ACTWINDOW; i++) {
            act[i] = false;
        }
    } else {
        normalMove();
    }
}

void manualMove() {
    /*
    手动控制
    */
    if (ps2x->ButtonDataByte()) {
        if (ps2x->Button(PSB_PAD_UP)) {
            setMotor(maxs, Forw);
        } else if (ps2x->Button(PSB_PAD_DOWN)) {
            setMotor(maxs, Back);
        } else if (ps2x->Button(PSB_PAD_RIGHT)) {
            setMotor(maxs, Right);
        } else if (ps2x->Button(PSB_PAD_LEFT)) {
            setMotor(maxs, Left);
        } else if (ps2x->Button(PSB_L1)) {
            setMotor(maxs, LSpin);
        } else if (ps2x->Button(PSB_R1)) {
            setMotor(maxs, RSpin);
        } else {
            setMotor(0, Forw);
        }
    } else {
        setMotor(0, Forw);
    }
}

void autoMoveInit() {
    /*
    对窗口进行初始化
    */
    ld = checkDistance(1);
    rd = checkDistance(2);
    for (int i = 0; i < WINDOW; i++) {
        ld_list[i] = ld;
        rd_list[i] = rd;
    }
    last_ldis = ld_list[WINDOW - 1];
    last_rdis = rd_list[WINDOW - 1];
    ld_list[1] = ld_list[WINDOW - 1] - 2;
    rd_list[1] = rd_list[WINDOW - 1] - 2;

    for (int i = 0; i < ACTWINDOW; i++) {
        act[i] = false;
    }
}

// const int bufferSize = 1024;
// char jsonBuffer[bufferSize];

// void executeVUI(String actionArray[], int durationArray[], int numActions) {
//     for (int i = 0; i < numActions; i++) {
//         // 根据动作类型执行相应的动作
//         int action = actionArray[i].toInt();
//         setMotor(maxs, action);
//         // 持续相应的时间
//         delay(durationArray[i]);
//         setMotor(0, Forw);

//         // 需不需要再delay一下之类的，以确保在执行下一个动作之前停止？
//     }
// }

void networkMove() {
    if (Serial.available()) {
        // // setMotor(3000, Forw);
        // // delay(50);
        // // setMotor(0, Forw);
        // Serial.print("len:");
        // Serial.print(Serial.available());
        // Serial.print(" ");
        // char buffer[200];
        // int count = 0;
        // while (Serial.available() > 0) {
        //     buffer[count++] = Serial.read();
        //     if (buffer[count - 1] == '$') {
        //         count--;
        //         break;
        //     }
        // }
        // buffer[count] = '\0';
        // // String command = Serial.readStringUntil('$');
        // for (int i = 0; i < count; i++) {
        //     Serial.print(buffer[i]);
        // }
        // String command = String(buffer);
        // Serial.print("Received command from ESP32: ");
        // Serial.print(command);
        // // 解析JSON数据
        // StaticJsonDocument<bufferSize> doc;
        // DeserializationError error = deserializeJson(doc, command);

        // if (error) {
        //     Serial.println("Failed to parse JSON$");
        //     return;
        // }

        // int numActions = doc["numActions"];
        // JsonArray actionArray = doc["actionArray"];
        // JsonArray durationArray = doc["durationArray"];

        // // 检查数组是否有效
        // if (actionArray.size() < numActions || durationArray.size() < numActions) {
        //     Serial.println("Invalid array sizes$");
        //     return;
        // }

        // // 将数据存储在int和数组中
        // int numActionsInt = numActions;
        // String actionString[numActions];
        // int durationInt[numActions];

        // for (int i = 0; i < numActions; i++) {
        //     actionString[i] = actionArray[i].as<String>();
        //     durationInt[i] = durationArray[i].as<int>();
        // }

        // // 执行动作
        // executeVUI(actionString, durationInt, durationInt);

        // // 打印解析的数据
        // Serial.print("Parsed data:");
        // Serial.print("numActions: ");
        // Serial.print(numActionsInt);

        // for (int i = 0; i < numActionsInt; i++) {
        //     Serial.print("Action ");
        //     Serial.print(i);
        //     Serial.print(": ");
        //     Serial.println(actionString[i]);
        //     Serial.print("Duration ");
        //     Serial.print(i);
        //     Serial.print(": ");
        //     Serial.print(durationInt[i]);
        // }
        // Serial.print("Received:");
        char num = Serial.read();
        // Serial.print(num);
        // Serial.print("$");
        if (num < '0' || num > ':') {
            return;
        }
        setMotor(maxs, num - '0');
        // 持续相应的时间
        delay(400);
        setMotor(0, Forw);
    }
}

void setup() {
    Serial.begin(9600);
    // Serial.begin(9600);
    // mySerial.begin(9600);
    randomSeed(analogRead(0));
    pinMode(TrigL, OUTPUT);
    pinMode(EchoL, INPUT);
    pinMode(TrigR, OUTPUT);
    pinMode(EchoR, INPUT);

    while (!Serial) {
        delay(100);
    }
    // Serial.print("Start$");
    pwm.begin();
    pwm.motorConfig(DIRN, DIRP, DIRN, DIRP);
    pwm.setAllMotor(0);

    ps2x = mMotorDriver.getSensor(E_PS2X);

    rgb_display_A1.begin();
}

void loop() {
    ps2x->read_gamepad(false, 0);
    if (ps2x->ButtonDataByte()) {
        if (ps2x->Button(PSB_CROSS) && ps2x->Button(PSB_CIRCLE) && ps2x->Button(PSB_SQUARE) &&
            ps2x->Button(PSB_TRIANGLE)) {
        } else if (ps2x->Button(PSB_CROSS) && ps2x->Button(PSB_TRIANGLE)) {  // 切换为手动模式
            mode = 1;
        } else if (ps2x->Button(PSB_CIRCLE) && ps2x->Button(PSB_TRIANGLE)) {  // 切换为自动模式
            autoMoveInit();
            mode = 0;
        } else if (ps2x->Button(PSB_SQUARE) && ps2x->Button(PSB_TRIANGLE)) {  // 切换为通信模式
            mode = 2;
        }
        if (ps2x->Button(PSB_CROSS) && ps2x->Button(PSB_CIRCLE) && ps2x->Button(PSB_SQUARE) &&
            ps2x->Button(PSB_TRIANGLE)) {
        } else if (ps2x->Button(PSB_L2)) {  // 降速
            maxs -= 200;
        } else if (ps2x->Button(PSB_R2)) {  // 加速
            maxs += 200;
        }
    }
    if (mode == 0) {
        autoMove();
    } else if (mode == 1) {
        manualMove();
        delay(50);
    } else {
        networkMove();
    }
    // networkMove();
}