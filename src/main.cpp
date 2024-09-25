#include "unit_rolleri2c.hpp"
#include <M5Unified.h>

// GUI
#define APP_NAME "M5Unit-RollerI2C"
#define APP_VERSION "ver.1.0"

// I2C
#define PIN_ROLLERI2C_SDA GPIO_NUM_32
#define PIN_ROLLERI2C_SCL GPIO_NUM_33

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

// roller
UnitRollerI2C RollerI2C;  // Create a UNIT_ROLLERI2C object
uint32_t p, i, d;         // Defines a variable to store the PID value
uint8_t r, g, b;

typedef enum {
    SPEED = 1,
    POSITION,
    CURRENT,
    ENCODER
} CtrlMode;

static uint8_t ctrl_mode = (uint8_t) CtrlMode::SPEED;
static bool motion_enable = false;
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

// GFX
M5Canvas canvas;

void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    delay(100);

    if(!RollerI2C.begin(&Wire, UNIT_ROLLERI2C_ADDR, PIN_ROLLERI2C_SDA, PIN_ROLLERI2C_SCL, 400000)) {
        Serial.println("[Error] UnitRoller I2C not found");
        while(1);
    }

    // GUI
    battery_level = M5.Power.getBatteryLevel();

    M5.Display.begin();
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setColorDepth(1); // mono color
        M5.Display.fillScreen(BLACK);
        M5.Display.setFont(&fonts::efontCN_14);
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.drawString(APP_NAME, 7, 7 + 12);
        M5.Display.setTextSize(1);  // 14
        M5.Display.drawString(APP_VERSION, 7 + 14*2 * 7, 7);     // 8 characters in "ver.1.0 "
        M5.Display.drawString(String(battery_level)+"%", M5.Lcd.width() - 14/2 * (4+1), 7);
        M5.Display.drawString("- push BtnA to XXX", 7*2, 7 + (14*2)*2);
        M5.Display.drawString("- push BtnB to XXX", 7*2, 7 + (14*2)*2+14);
        M5.Display.drawString("- push BtnC to XXX", 7*2, 7 + (14*2)*2+14*2);
        M5.Display.drawRect( 20, 220, 80, 20, GOLD);
        M5.Display.drawString(" A:XXX ", 20+5, 220+2);
        M5.Display.drawRect(120, 220, 80, 20, GOLD);
        M5.Display.drawString(" B:XXX ", 120+5, 220+2);
        M5.Display.drawRect(220, 220, 80, 20, GOLD);
        M5.Display.drawString(" C:XXX ", 220+5, 220+2);
    M5.Display.endWrite();

    // roller
    ctrl_mode = CtrlMode::SPEED;
    motion_enable = false;
    gui_disp_ctrl_mode(ctrl_mode);
    RollerI2C.setDialCounter(0);
    RollerI2C.setRGBMode(1);
    RollerI2C.setRGB(TFT_WHITE);

    // GFX
    canvas.createSprite(M5.Display.width(), M5.Display.height());
    canvas.setTextSize(2);
    canvas.setCursor(M5.Display.width()/2, M5.Display.height()/2);
    canvas.printf("ROLLER");

    // application timer
    print_enable_10sec = false;
    print_enable_3sec = false;

    pre_ms_3sec = millis();

    // init done
    Serial.println("[Info] init done");
    beep_init_done();
}

void loop()
{
    M5.update();

    if(M5.BtnA.wasPressed()) {
        // Serial.println("[Info] Button A was pressed");
        if(motion_enable) {
            motion_enable = false;
            RollerI2C.setOutput(0);
            RollerI2C.setRGB(TFT_GREEN);
        } else {
            motion_enable = true;
            RollerI2C.setOutput(1);
            RollerI2C.setRGB(ctrl_mode_color[ctrl_mode]);
        }
        beep();
    }
    if(M5.BtnB.wasPressed()) {
        // Serial.println("[Info] Button B was pressed");
        ctrl_mode++;
        if(ctrl_mode > CtrlMode::ENCODER) {
            ctrl_mode = CtrlMode::SPEED;
        }
        gui_disp_ctrl_mode(ctrl_mode);
        RollerI2C.setRGB(ctrl_mode_color[ctrl_mode]);
        beep();
    }
    if(M5.BtnC.wasPressed()) {
        // Serial.println("[Info] Button C was pressed");
        beep();
    }
    if(M5.Touch.getDetail().isPressed()) {
        static float angle_deg = 0.0f;
        canvas.pushRotated(&M5.Display, angle_deg);
        angle_deg += 45.0;
        Serial.printf("[Info] Touch was pressed, angle=%f deg\n", angle_deg);
    }

    if(motion_enable) {
        switch (ctrl_mode) {
        case CtrlMode::CURRENT:
            RollerI2C.setMode(3);
            RollerI2C.setCurrent(120000);
            RollerI2C.setOutput(1);

            if(print_enable_3sec) {
                Serial.printf("[Info] -- Mode: Current --\n");
                Serial.printf("[Info] current: %d\n", RollerI2C.getCurrent());
                Serial.printf("[Info] actualCurrent: %d\n", RollerI2C.getCurrentReadback());
                Serial.println();
                print_enable_3sec = false;
            }
            break;
        case CtrlMode::POSITION:
            RollerI2C.setOutput(0);
            RollerI2C.setMode(2);
            RollerI2C.setPos(2000000);
            RollerI2C.setPosMaxCurrent(100000);
            RollerI2C.setOutput(1);
            RollerI2C.getPosPID(&p, &i, &d);

            if(print_enable_3sec) {
                Serial.printf("[Info] -- Mode: Position --\n");
                Serial.printf("[Info] PosPID  P: %3.8f  I: %3.8f  D: %3.8f\n", p / 100000.0, i / 10000000.0, d / 100000.0);
                Serial.printf("[Info] pos: %d\n", RollerI2C.getPos());
                Serial.printf("[Info] posMaxCurrent: %d\n", RollerI2C.getPosMaxCurrent());
                Serial.printf("[Info] actualPos: %d\n", RollerI2C.getPosReadback());
                Serial.println();
                print_enable_3sec = false;
            }
            break;
        case CtrlMode::SPEED:
            RollerI2C.setOutput(0);
            RollerI2C.setMode(1);
            RollerI2C.setSpeed(240000);
            RollerI2C.setSpeedMaxCurrent(100000);
            RollerI2C.setOutput(1);
            RollerI2C.getSpeedPID(&p, &i, &d);

            if(print_enable_3sec) {
                Serial.printf("[Info] -- Mode: Speed --\n");
                Serial.printf("SpeedPID  P: %3.8f  I: %3.8f  D: %3.8f\n", p / 100000.0, i / 10000000.0, d / 100000.0);
                Serial.printf("speed: %d\n", RollerI2C.getSpeed());
                Serial.printf("speedMaxCurrent: %d\n", RollerI2C.getSpeedMaxCurrent());
                Serial.printf("actualSpeed: %d\n", RollerI2C.getSpeedReadback());
                Serial.println();
                print_enable_3sec = false;
            }
            break;
        case CtrlMode::ENCODER:
            RollerI2C.setOutput(0);
            RollerI2C.setMode(4);
            // RollerI2C.setDialCounter(240000);
            // RollerI2C.setOutput(1);

            // RollerI2C.setRGBBrightness(100);
            // delay(100);
            // RollerI2C.setRGBMode(1);
            // delay(1000);
            // RollerI2C.setRGB(TFT_WHITE);
            // delay(1000);
            // RollerI2C.setRGB(TFT_BLUE);
            // delay(2000);
            // RollerI2C.setRGB(TFT_YELLOW);
            // delay(2000);
            // RollerI2C.setRGB(TFT_RED);
            // delay(2000);
            // RollerI2C.setRGBMode(0);
            // delay(100);
            // RollerI2C.setKeySwitchMode(1);
            // delay(100);
            // printf("I2CAddress:%d\n", RollerI2C.getI2CAddress());
            // delay(100);
            // printf("485 BPS:%d\n", RollerI2C.getBPS());
            // delay(100);
            // printf("485 motor id:%d\n", RollerI2C.getMotorID());
            // delay(100);
            // printf("motor output:%d\n", RollerI2C.getOutputStatus());
            // delay(100);
            // printf("SysStatus:%d\n", RollerI2C.getSysStatus());
            // delay(100);
            // printf("ErrorCode:%d\n", RollerI2C.getErrorCode());
            // delay(100);
            // printf("Button switching mode enable:%d\n", RollerI2C.getKeySwitchMode());
            // delay(100);
            // RollerI2C.getRGB(&r, &g, &b);
            // printf("RGB-R: 0x%02X  RGB-G: 0x%02X  RGB-B: 0x%02X\n", r, g, b);

            if(print_enable_3sec) {
                Serial.printf("[Info] -- Mode: Encoder --\n");
                Serial.printf("DialCounter:%d\n", RollerI2C.getDialCounter());
                Serial.printf("temp:%d\n", RollerI2C.getTemp());
                Serial.printf("Vin:%3.2f\n", RollerI2C.getVin() / 100.0);
                // Serial.printf("RGBBrightness:%d\n", RollerI2C.getRGBBrightness());
                Serial.println();
                print_enable_3sec = false;
            }
            break;
        }
    }
    
    // application timer
    current_ms = millis();
    if(current_ms - pre_ms_10sec > interval_10sec) {
        gui_disp_batterylevel();
        pre_ms_10sec = current_ms;
    }
    if(current_ms - pre_ms_3sec > interval_3sec) {
        print_enable_3sec = true;
        pre_ms_3sec = current_ms;
    }

    vTaskDelay(50);
}