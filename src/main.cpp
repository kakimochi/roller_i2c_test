#include "unit_rolleri2c.hpp"
#include <M5Unified.h>
#include <MadgwickAHRS.h>

// GUI
#define APP_NAME "iPendulum"
#define APP_VERSION "ver.1.0"

// I2C
#define UNIT_ROLLERI2C_ADDR_L (0x64)
#define UNIT_ROLLERI2C_ADDR_R (0x65)

// Beep Sound
#define TONE_C5 523.251
#define TONE_E5 659.255
#define TONE_G5 783.991
#define TONE_C6 (TONE_C5 * 2)
#define TONE_E6 (TONE_E5 * 2)
#define TONE_G6 (TONE_G5 * 2)

void beep()
{
    M5.Speaker.tone(TONE_E5, 200);
}

void beep_init_done()
{
    M5.Speaker.tone(TONE_C5, 500);
    M5.Speaker.tone(TONE_E5, 500);
    M5.Speaker.tone(TONE_G5, 500);
}

// application timer
const unsigned long interval_10sec = 10000;
const unsigned long interval_3sec  =  3000;
unsigned long pre_ms_10sec = 0;
unsigned long pre_ms_3sec = 0;
unsigned long current_ms = 0;
bool print_enable_10sec = false;
bool print_enable_3sec = false;

// IMU
float Pitch_ahrs, Roll_ahrs, Yaw_ahrs, Roll_bias, Roll;
float Gyro_x, Gyro_y, Gyro_z;
float Acc_x, Acc_y, Acc_z;
int32_t Imu_time,_Imu_time, Imu_dtime;
int32_t Current_ref_r, Current_ref_l;
int32_t Current_r, Current_l;
int32_t Pos_r, Pos_l, Pos_bias_r, Pos_bias_l;
int32_t Speed_r, Speed_l;
int32_t Voltage_r,Voltage_l;
int32_t current_time, previous_time, finished_time, diff_time;
float f1,f2,f3,f4;
float k1;
float U0, U_yaw, U_v;
uint8_t Start_flag = 0;
float commpass_x,commpass_y,commpass_z;

// madgwick filter
#define IMU_MADGWICK_SAMPLE_FREQ_HZ 100 // 200
Madgwick madwick;
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Posture;
Posture posture;

// roller
UnitRollerI2C RollerI2C_L;  // Create a UNIT_ROLLERI2C object for LEFT
UnitRollerI2C RollerI2C_R;  // Create a UNIT_ROLLERI2C object for RIGHT
// uint32_t p, i, d;         // Defines a variable to store the PID value
// uint8_t r, g, b;

typedef enum {
    SPEED = 1,
    POSITION,
    CURRENT,
    ENCODER
} CtrlMode;

static uint8_t ctrl_mode = (uint8_t) CtrlMode::CURRENT;
// static bool motion_enable = false;
static int ctrl_mode_color[1+4] = {
    TFT_WHITE,      // NONE
    TFT_YELLOW,     // CtrlMode::SPEED
    TFT_BLUE,       // CtrlMode::POSITION
    TFT_PURPLE,     // CtrlMode::CURRENT
    TFT_GREENYELLOW // CtrlMode::ENCODER
};

// Battery
int battery_level = -1;

void gui_disp_batterylevel()
{
    battery_level = M5.Power.getBatteryLevel();

    int pos_x = M5.Lcd.width() - 14/2 * (4+1);
    int pos_y = 7;
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setTextColor(TFT_GOLD);
        M5.Display.setTextSize(1);
        M5.Display.fillRect(pos_x, pos_y, 320, 14*1, BLACK);    // clear
        M5.Display.drawString(String(battery_level)+"%", pos_x, pos_y);
    M5.Display.endWrite();
}

void gui_disp_ctrl_mode(uint8_t ctrl_mode)
{
    std::string str_ctrl_mode = "";
    switch (ctrl_mode)
    {
    case CtrlMode::SPEED:
        str_ctrl_mode = "SPEED";
        break;
    case CtrlMode::POSITION:
        str_ctrl_mode = "POSITION";
        break;
    case CtrlMode::CURRENT:
        str_ctrl_mode = "CURRENT";
        break;
    case CtrlMode::ENCODER:
        str_ctrl_mode = "ENCODER";
        break;
    default:
        str_ctrl_mode = "UNKNOWN";
        break;
    }

    int pos_x = 7*2;
    int pos_y = 7 + 7 + (14*2)*2+14*2+(14);
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.fillRect(pos_x, pos_y, 320, 14*2, BLACK);    // clear
        M5.Display.drawString(str_ctrl_mode.c_str(), pos_x, pos_y);
    M5.Display.endWrite();
}

void task_monitor(void *pvParameters) {
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(50));

        M5.update();
        if(M5.BtnA.wasPressed()) {
            Serial.printf("[Info] Button A was pressed.\n");
            Start_flag = !Start_flag;
            Roll_bias = Roll;
            Pos_bias_r = Pos_r;
            Pos_bias_l = Pos_l;
        }

        if(print_enable_3sec) {
            printf("[Info]  Acc: %3.2f, %3.2f, %3.2f\n", Acc_x, Acc_y, Acc_z);
            printf("[Info] Gyro: %3.2f, %3.2f, %3.2f\n", Gyro_x, Gyro_y, Gyro_z);
            printf("----\n");
            M5.Display.fillRect(7, 7 + 12 + 14*2, 320, 14*2, BLACK); // clear the area
            M5.Display.drawString("Acc: "+String(Acc_x)+", "+String(Acc_y)+", "+String(Acc_z), 7, 7 + 12 + 14*2);
            print_enable_3sec = false;
        }
    }
}

void task_control(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 5ms の周期
    xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 次の周期まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        previous_time = current_time;
        current_time = micros();
        // IMU
        M5.Imu.update();
        auto imudata = M5.Imu.getImuData();
        Gyro_x = imudata.gyro.x;
        Gyro_y = imudata.gyro.y;
        Gyro_z = imudata.gyro.z;
        Acc_x = imudata.accel.x;
        Acc_y = imudata.accel.y;
        Acc_z = imudata.accel.z;
        madwick.update(Gyro_x, Gyro_y, Gyro_z, Acc_x, Acc_y, Acc_z, commpass_x, commpass_y, commpass_z);
        Roll_ahrs = madwick.getRoll();
        Pitch_ahrs = madwick.getPitch();
        Yaw_ahrs = madwick.getYaw();
        // MadgwickAHRSupdateIMU(Gyro_x * DEG_TO_RAD, Gyro_y * DEG_TO_RAD, Gyro_z * DEG_TO_RAD, Acc_x, Acc_y, Acc_z, &Pitch_ahrs, &Roll_ahrs, &Yaw_ahrs);

        // Control
        k1 = 0.3;
        Roll = k1*Roll +(1-k1)*Roll_ahrs;
        Current_r =  RollerI2C_R.getCurrentReadback();
        Current_l = -RollerI2C_L.getCurrentReadback();
        Pos_r =  RollerI2C_R.getPosReadback();
        Pos_l = -RollerI2C_L.getPosReadback();
        Speed_r =  RollerI2C_R.getSpeedReadback();
        Speed_l = -RollerI2C_L.getSpeedReadback();
        Voltage_r = RollerI2C_R.getVin();
        Voltage_l = RollerI2C_L.getVin();

        // current control
        if(Start_flag==1){
            f1 = 7500.0;    //4200.0;//振子の角度に比例して電流を制御
            f2 = 150.0;     //200.0;//振子の角速度に比例して電流を制御
            f3 = 0.0;       //モータの角度に比例して電流を制御 0.1
            f4 = 0.0;       //モータの角速度に比例して電流を制御
            float yaw_ref = -360.0*0.0f;
            // float yaw_ref = -360.0*Stick[RUDDER];
            float yaw_err = yaw_ref - Gyro_z;
            U_yaw = yaw_err * 300.0;
            U_v = -0.0f * 200000.0;
            // U_v = -Stick[THROTTLE] * 200000.0;
            //State feedback control
            U0 = (-f1 * (Roll-Roll_bias) - f2 * Gyro_x - f3 * ((float)(Pos_r-Pos_bias_r)/1.0) - f4 * Speed_r);
            Current_ref_r = (int32_t)(U0 + U_v + U_yaw);
            Current_ref_l = (int32_t)(U0 + U_v - U_yaw);
            //limit current
            if (Current_ref_r>120000)Current_ref_r=120000;
            else if (Current_ref_r<-120000)Current_ref_r=-120000;
            if (Current_ref_l>120000)Current_ref_l=120000;
            else if (Current_ref_l<-120000)Current_ref_l=-120000;

            RollerI2C_R.setCurrent(Current_ref_r);
            RollerI2C_L.setCurrent(-Current_ref_l);
        }
        else{
            RollerI2C_R.setCurrent(0);
            RollerI2C_L.setCurrent(0);
        }
        finished_time = micros();
        diff_time = finished_time - current_time;
    }
}



void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    delay(100);

    if(!RollerI2C_L.begin(&Wire, UNIT_ROLLERI2C_ADDR_L, M5.getPin(m5::pin_name_t::port_a_sda), M5.getPin(m5::pin_name_t::port_a_scl), 400000)) {
        printf("[Error] UnitRoller LEFT I2C(%02X) not found\n", UNIT_ROLLERI2C_ADDR_L);
        // while(1);
    }
    if(!RollerI2C_R.begin(&Wire, UNIT_ROLLERI2C_ADDR_R, M5.getPin(m5::pin_name_t::port_a_sda), M5.getPin(m5::pin_name_t::port_a_scl), 400000)) {
        printf("[Error] UnitRoller RIGHT I2C(%02X) not found\n", UNIT_ROLLERI2C_ADDR_R);
        while(1);
    }

    #if 0 // Set I2C address process
    if (RollerI2C_L.setI2CAddress(0x65)) {
        Serial.println("[Info] I2C address set to 0x65 successfully");
    } else {
        Serial.println("[Error] Failed to set I2C address to 0x65");
    }
    while(1);
    #endif

    // GUI
    // battery_level = M5.Power.getBatteryLevel();

    M5.Display.begin();
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setColorDepth(1); // mono color
        M5.Display.fillScreen(BLACK);
        M5.Display.setFont(&fonts::efontCN_10);
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.drawString(APP_NAME, 7, 7 + 12);
        // M5.Display.setTextSize(1);  // 14
        // M5.Display.drawString(APP_VERSION, 7 + 14*2 * 7, 7);     // 8 characters in "ver.1.0 "
        // M5.Display.drawString(String(battery_level)+"%", M5.Lcd.width() - 14/2 * (4+1), 7);
        // M5.Display.drawString("- push BtnA to XXX", 7*2, 7 + (14*2)*2);
        // M5.Display.drawString("- push BtnB to XXX", 7*2, 7 + (14*2)*2+14);
        // M5.Display.drawString("- push BtnC to XXX", 7*2, 7 + (14*2)*2+14*2);
        // M5.Display.drawRect( 20, 220, 80, 20, GOLD);
        // M5.Display.drawString(" A:XXX ", 20+5, 220+2);
        // M5.Display.drawRect(120, 220, 80, 20, GOLD);
        // M5.Display.drawString(" B:XXX ", 120+5, 220+2);
        // M5.Display.drawRect(220, 220, 80, 20, GOLD);
        // M5.Display.drawString(" C:XXX ", 220+5, 220+2);
    M5.Display.endWrite();

    // IMU
    Acc_x = Acc_y = Acc_z = 0.0f;
    Gyro_x = Gyro_y = Gyro_z = 0.0f;
    commpass_x = commpass_y = commpass_z = 0.0f;
    posture = {0};
    madwick.begin(IMU_MADGWICK_SAMPLE_FREQ_HZ);

    // roller
    ctrl_mode = CtrlMode::CURRENT;
    // motion_enable = false;
    gui_disp_ctrl_mode(ctrl_mode);
    RollerI2C_L.setDialCounter(0);
    RollerI2C_L.setRGBMode(ROLLER_RGB_MODE_USER_DEFINED);
    RollerI2C_L.setRGB(TFT_GOLD);
    RollerI2C_R.setDialCounter(0);
    RollerI2C_R.setRGBMode(ROLLER_RGB_MODE_USER_DEFINED);
    RollerI2C_R.setRGB(TFT_GOLD);

    // Task
    BaseType_t result = xTaskCreateUniversal(task_control, "5ms Periodic Task", 8192, NULL, 5, NULL, APP_CPU_NUM);
    if (result != pdPASS) {
        printf("[Error] Task creation failed: %d\n", result);
        printf("[Error] into the infinite loop\n");
        while (1);
    }

    result = xTaskCreateUniversal(task_monitor, "monitor_task", 8192, NULL, 1, NULL, APP_CPU_NUM);
    if (result != pdPASS) {
        printf("[Error] Task creation failed: %d\n", result);
        printf("[Error] into the infinite loop\n");
        while(1);
    }

    // application timer
    print_enable_10sec = false;
    print_enable_3sec = false;

    pre_ms_3sec = millis();

    // init done
    printf("[Info] init done.\n");
    beep_init_done();
}

void loop()
{
    // M5.update();

    #if 0 // for imu debug
    M5.Imu.update();
    auto imu_data = M5.Imu.getImuData();
    Acc_x = imu_data.accel.x;
    Acc_y = imu_data.accel.y;
    Acc_z = imu_data.accel.z;
    Gyro_x = imu_data.gyro.x;
    Gyro_y = imu_data.gyro.y;
    Gyro_z = imu_data.gyro.z;
    #endif
    // if(M5.BtnA.isPressed()) {
    // // if(M5.BtnA.wasPressed()) {
    //     Serial.printf("[Info] Button A was pressed.\n");
    //     Start_flag = !Start_flag;
    //     Roll_bias = Roll;
    //     Pos_bias_r = Pos_r;
    //     Pos_bias_l = Pos_l;
    //     beep();
    // }
    if(print_enable_3sec) {
        printf("[Info]  Acc: %3.2f, %3.2f, %3.2f\n", Acc_x, Acc_y, Acc_z);
        printf("[Info] Gyro: %3.2f, %3.2f, %3.2f\n", Gyro_x, Gyro_y, Gyro_z);
        printf("----\n");
        M5.Display.fillRect(7, 7 + 12 + 14*2, 320, 14*2, BLACK); // clear the area
        M5.Display.drawString("Acc: "+String(Acc_x)+", "+String(Acc_y)+", "+String(Acc_z), 7, 7 + 12 + 14*2);
        print_enable_3sec = false;
    }
    
    // application timer
    current_ms = millis();
    if(current_ms - pre_ms_10sec > interval_10sec) {
        // gui_disp_batterylevel();
        pre_ms_10sec = current_ms;
    }
    if(current_ms - pre_ms_3sec > interval_3sec) {
        print_enable_3sec = true;
        pre_ms_3sec = current_ms;
    }

    vTaskDelay(1);
}