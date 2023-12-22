// #include <Adafruit_NeoPixel.h>
// #include <Wire.h>
// #include <string.h>

// #include "Emakefun_MotorDriver.h"
// #include "Openjumper_IICMotorDriver.h"

// #define TrigL 2
// #define EchoL 3

// #define TrigR 7
// #define EchoR 8

// #define Blocked LOW
// #define Unimpeded HIGH

// #define InfLF 5
// #define InfRF 6
// #define InfLB 7
// #define InfRB 8

// #define Forw 1
// #define Back 2
// #define Left 3
// #define Right 4
// #define LForw 5
// #define RForw 6
// #define LBack 7
// #define RBack 8
// #define LSpin 9
// #define RSpin 10

// #define LLight 3
// #define MLight 2
// #define RLight 1

// #define WINDOW 10
// #define ACTWINDOW 50
// #define ACTTHRES 5

// #define THRES1 60
// #define THRES2 40
// #define THRES3 20

// int maxs = 3000;  // 最大速度

// PS2X *ps2x;
// Openjumper_IICMotorDriver pwm = Openjumper_IICMotorDriver();
// Emakefun_MotorDriver mMotorDriver = Emakefun_MotorDriver(0x60, MOTOR_DRIVER_BOARD_V5);
// Adafruit_NeoPixel rgb_display_A1 = Adafruit_NeoPixel(3, A1, NEO_GRB + NEO_KHZ800);

// float checkDistance(int idx) {
//     /*
//     超声波测距, idx=1左边，idx=2右边，返回距离(cm）
//     注意需要防两个超声波干扰
//     设置了最大超时时间为20ms，若超时则返回100cm
//     */
//     float distance;
//     if (idx == 1) {
//         digitalWrite(TrigL, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TrigL, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TrigL, LOW);
//         distance = pulseIn(EchoL, HIGH, 1000 * 20) / 58.00;
//         if (distance == 0) {
//             distance = 100;
//         }
//     } else {
//         digitalWrite(TrigR, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TrigR, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TrigR, LOW);
//         distance = pulseIn(EchoR, HIGH, 1000 * 20) / 58.00;
//         if (distance == 0) {
//             distance = 100;
//         }
//     }
//     delay(20);
//     return distance;
// }

// int infSensor(byte pin) {
//     /*
//     红外传感器,返回Blocked或Unimpeded
//     */
//     int data = digitalRead(pin);
//     return data;
// }

// void setColor(int idx, byte r, byte g, byte b) {
//     /*
//     设置RGB灯颜色，调试用
//     */
//     rgb_display_A1.setPixelColor(idx - 1, (((r / 20) & 0xffffff) << 16) | (((g / 20) & 0xffffff) << 8) | (b / 20));
//     rgb_display_A1.show();
// }

// void setMotor(int speed, int dir) {
//     /*
//     设置电机转速，dir为方向,参考上面的宏定义
//     旋转转速恒定为2000
//     */
//     switch (dir) {
//         case Forw:
//             pwm.setAllMotor(speed);
//             break;
//         case Back:
//             pwm.setAllMotor(-speed);
//             break;
//         case Left:
//             pwm.setAllMotor(-speed, speed, -speed, speed);
//             break;
//         case Right:
//             pwm.setAllMotor(speed, -speed, speed, -speed);
//             break;
//         case LForw:
//             pwm.setAllMotor(0, speed, 0, speed);
//             break;
//         case RForw:
//             pwm.setAllMotor(speed, 0, speed, 0);
//             break;
//         case LBack:
//             pwm.setAllMotor(-speed, 0, -speed, 0);
//             break;
//         case RBack:
//             pwm.setAllMotor(0, -speed, 0, -speed);
//             break;
//         case LSpin:
//             pwm.setAllMotor(-2000, -2000, 2000, 2000);
//             break;
//         case RSpin:
//             pwm.setAllMotor(2000, 2000, -2000, -2000);
//             break;
//         default:
//             break;
//     }
// }

// bool autoMode = false;  // 是否自动模式
// int ld_list[WINDOW];
// int rd_list[WINDOW];
// int ldis, rdis, ld, rd, lf, rf, last_ldis, last_rdis;
// bool act[ACTWINDOW];

// void handleBlocked() {
//     /*
//     左前右前都被堵住
//     */
//     setMotor(maxs, Back);
//     delay(100);
//     int rand = random(0, 40);
//     if (rand == 0) {
//         setMotor(maxs, RSpin);
//         delay(500);
//     } else if (rand < 8) {
//         setMotor(maxs, LSpin);
//         delay(500);
//     } else {
//         if (ldis < rdis) {
//             while (1) {
//                 setMotor(maxs, RSpin);
//                 delay(100);
//                 lf = infSensor(InfLF);
//                 rf = infSensor(InfRF);
//                 if (lf == Unimpeded || rf == Unimpeded) {
//                     break;
//                 }
//             }
//         } else {
//             while (1) {
//                 setMotor(maxs, LSpin);
//                 delay(100);
//                 lf = infSensor(InfLF);
//                 rf = infSensor(InfRF);
//                 if (lf == Unimpeded || rf == Unimpeded) {
//                     break;
//                 }
//             }
//         }
//     }
// }

// void Escape() {
//     /*
//     很可能车卡住了，尝试逃逸
//     */
//     setColor(MLight, 255, 255, 255);
//     int randdis = random(0, 500);
//     int rand = random(0, 8);
//     if (rand == 0) {
//         setMotor(maxs, RSpin);
//         delay(500 + randdis);
//     } else {
//         setMotor(maxs, LSpin);
//         delay(500 + randdis);
//     }
// }

// void randomMove() {
//     /*
//     随机移动以尝试破除某些条件导致的死循环
//     */
//     setColor(MLight, 255, 255, 255);
//     int rand = random(0, 40);
//     int randdis = random(0, 200);
//     if (rand == 0) {
//         setMotor(maxs, LSpin);
//         delay(300 + randdis);
//     } else if (rand < 8) {
//         setMotor(maxs, RSpin);
//         delay(300 + randdis);
//     } else if (rand < 16) {
//         setMotor(maxs, Left);
//         delay(300);
//     } else if (rand < 24) {
//         setMotor(maxs, Right);
//         delay(300);
//     } else {
//         setMotor(maxs, Back);
//         delay(300);
//     }
// }

// void normalMove() {
//     /*
//     正常避障
//     */
//     int maxdis = ldis > rdis ? ldis : rdis;
//     int mindis = ldis < rdis ? ldis : rdis;
//     if (ldis > THRES1) {  // green
//         setColor(LLight, 0, 255, 0);
//     } else if (ldis > THRES2) {  // yellow
//         setColor(LLight, 255, 255, 0);
//     } else if (ldis > THRES3) {  // blue
//         setColor(LLight, 0, 0, 255);
//     } else {  // red
//         setColor(LLight, 255, 0, 0);
//     }
//     if (rdis > THRES1) {  // green
//         setColor(RLight, 0, 255, 0);
//     } else if (rdis > THRES2) {  // yellow
//         setColor(RLight, 255, 255, 0);
//     } else if (rdis > THRES3) {  // blue
//         setColor(RLight, 0, 0, 255);
//     } else {  // red
//         setColor(RLight, 255, 0, 0);
//     }

//     bool actflag = false;
//     if (lf == Blocked && rf == Blocked) {
//         setColor(MLight, 255, 255, 255);
//         handleBlocked();
//         actflag = true;
//     } else if (lf == Blocked) {
//         setColor(MLight, 0, 0, 255);
//         setMotor(maxs, Right);
//         delay(100);
//         setMotor(maxs, RSpin);
//         delay(50);
//         actflag = true;
//     } else if (rf == Blocked) {
//         setColor(MLight, 255, 0, 0);
//         setMotor(maxs, Left);
//         delay(100);
//         setMotor(maxs, LSpin);
//         delay(50);
//         actflag = true;
//     } else if (mindis > THRES1) {
//         setMotor(maxs, Forw);
//         setColor(MLight, 0, 255, 0);
//     } else if (mindis > THRES2) {
//         setMotor(0.75 * maxs + 0.25 * maxs * (mindis - THRES2) / (THRES1 - THRES2), Forw);
//         setColor(MLight, 0, 255, 0);
//     } else if (ldis > THRES2 && rdis > THRES3) {
//         setMotor(0.75 * maxs, LForw);
//         setColor(MLight, 255, 255, 0);
//     } else if (ldis > THRES3 && rdis > THRES2) {
//         setMotor(0.75 * maxs, RForw);
//         setColor(MLight, 0, 255, 255);
//     } else if (mindis > THRES3) {
//         if (ldis > rdis) {
//             setMotor(0.6 * maxs + 0.15 * maxs * (mindis - THRES3) / (THRES2 - THRES3), LForw);
//             setColor(MLight, 255, 255, 0);
//         } else {
//             setMotor(0.6 * maxs + 0.15 * maxs * (mindis - THRES3) / (THRES2 - THRES3), RForw);
//             setColor(MLight, 0, 255, 255);
//         }
//     } else if (ldis <= THRES3 && rdis > THRES3) {
//         setColor(MLight, 0, 0, 255);
//         setMotor(maxs, Right);
//         delay(100);
//         setMotor(maxs, RSpin);
//         delay(50);
//         actflag = true;
//     } else if (ldis > THRES3 && rdis <= THRES3) {
//         setColor(MLight, 255, 0, 0);
//         setMotor(maxs, Left);
//         delay(100);
//         setMotor(maxs, LSpin);
//         delay(50);
//         actflag = true;
//     } else {
//         setColor(MLight, 255, 255, 255);
//         handleBlocked();
//         actflag = true;
//     }

//     for (int i = 0; i < ACTWINDOW - 1; i++) {
//         act[i] = act[i + 1];
//     }
//     act[ACTWINDOW - 1] = actflag;
// }

// void autoMove() {
//     /*
//     自动避障
//     */
//     lf = infSensor(InfLF);
//     rf = infSensor(InfRF);
//     ld = checkDistance(1);
//     rd = checkDistance(2);

//     for (int i = 0; i < WINDOW - 1; i++) {
//         ld_list[i] = ld_list[i + 1];
//         rd_list[i] = rd_list[i + 1];
//     }
//     ld_list[WINDOW - 1] = ld;
//     rd_list[WINDOW - 1] = rd;

//     // 超声波测距容易受干扰，取半个窗口内的平均值
//     int avg_ld = 0, avg_rd = 0;
//     for (int i = WINDOW / 2; i < WINDOW; i++) {
//         avg_ld += ld_list[i];
//         avg_rd += rd_list[i];
//     }
//     avg_ld /= (WINDOW - WINDOW / 2);
//     avg_rd /= (WINDOW - WINDOW / 2);

//     ldis = max(min(min(last_ldis + 3, avg_ld), ld), last_ldis - 10);  // 设置变化上下限
//     rdis = max(min(min(last_rdis + 3, avg_rd), rd), last_rdis - 10);
//     last_ldis = ldis;
//     last_rdis = rdis;
//     // int lb = infSensor(InfLB);
//     // int rb = infSensor(InfRB);
//     Serial.print("ldis:");
//     Serial.print(ldis);
//     Serial.print(" rdis:");
//     Serial.print(rdis);
//     // Serial.print(" ");
//     // Serial.print(lf);
//     // Serial.print(" ");
//     // Serial.print(rf);
//     // Serial.print("||");
//     // // Serial.print(" ");
//     // // Serial.print(lb);
//     // // Serial.print(" ");
//     // // Serial.print(rb);

//     // 判断是否被卡住
//     bool stopflag = true;
//     int baseld = ld_list[0], baserd = rd_list[0];
//     for (int i = 1; i < WINDOW; i++) {
//         if (abs(ld_list[i] - baseld) > 1 || abs(rd_list[i] - baserd) > 1) {
//             stopflag = false;
//             break;
//         }
//     }

//     // 判断是否陷入局部困境(一段时间内频繁异常避障)
//     int actcount = 0;
//     for (int i = 0; i < ACTWINDOW; i++) {
//         if (act[i]) {
//             actcount++;
//         }
//     }
//     bool actflag = actcount >= ACTTHRES;

//     if (stopflag) {
//         randomMove();
//     } else if (actflag) {
//         Escape();
//         for (int i = 0; i < ACTWINDOW; i++) {
//             act[i] = false;
//         }
//     } else {
//         normalMove();
//     }
// }

// void manualMove() {
//     /*
//     手动控制
//     */
//     if (ps2x->ButtonDataByte()) {
//         if (ps2x->Button(PSB_PAD_UP)) {
//             setMotor(maxs, Forw);
//         } else if (ps2x->Button(PSB_PAD_DOWN)) {
//             setMotor(maxs, Back);
//         } else if (ps2x->Button(PSB_PAD_RIGHT)) {
//             setMotor(maxs, Right);
//         } else if (ps2x->Button(PSB_PAD_LEFT)) {
//             setMotor(maxs, Left);
//         } else if (ps2x->Button(PSB_L1)) {
//             setMotor(maxs, LSpin);
//         } else if (ps2x->Button(PSB_R1)) {
//             setMotor(maxs, RSpin);
//         } else {
//             setMotor(0, Forw);
//         }
//     } else {
//         setMotor(0, Forw);
//     }
// }

// void autoMoveInit() {
//     /*
//     对窗口进行初始化
//     */
//     ld = checkDistance(1);
//     rd = checkDistance(2);
//     for (int i = 0; i < WINDOW; i++) {
//         ld_list[i] = ld;
//         rd_list[i] = rd;
//     }
//     last_ldis = ld_list[WINDOW - 1];
//     last_rdis = rd_list[WINDOW - 1];
//     ld_list[1] = ld_list[WINDOW - 1] - 2;
//     rd_list[1] = rd_list[WINDOW - 1] - 2;

//     for (int i = 0; i < ACTWINDOW; i++) {
//         act[i] = false;
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     randomSeed(analogRead(0));
//     pinMode(TrigL, OUTPUT);
//     pinMode(EchoL, INPUT);
//     pinMode(TrigR, OUTPUT);
//     pinMode(EchoR, INPUT);

//     pwm.begin();
//     pwm.motorConfig(DIRN, DIRP, DIRN, DIRP);
//     pwm.setAllMotor(0);

//     ps2x = mMotorDriver.getSensor(E_PS2X);

//     rgb_display_A1.begin();
// }

// void loop() {
//     ps2x->read_gamepad(false, 0);
//     if (ps2x->ButtonDataByte()) {
//         if (ps2x->Button(PSB_CIRCLE)) {  // 切换为自动模式
//             autoMoveInit();
//             autoMode = true;
//         }
//         if (ps2x->Button(PSB_CROSS)) {  // 切换为手动模式
//             autoMode = false;
//         }
//         if (ps2x->Button(PSB_SQUARE)) {  // 降速
//             maxs -= 200;
//         }
//         if (ps2x->Button(PSB_TRIANGLE)) {  // 加速
//             maxs += 200;
//         }
//     }
//     if (autoMode) {
//         autoMove();
//     } else {
//         manualMove();
//         delay(50);
//     }
// }