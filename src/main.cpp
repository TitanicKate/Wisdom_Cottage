#include <Arduino.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <DHT_U.h>

// 定义温湿度传感器
#define DHT_TYPE DHT11
#define DHT_PIN 4
DHT_Unified dht(DHT_PIN, DHT_TYPE);

// 初始化OLED：软件I2C，SCL和SDA引脚需在实际接线时定义（如#define SCL 22, #define SDA 21）
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);

// 执行器引脚
#define LED_PIN 25
#define WIFI_STATUS_LED_PIN 2
#define FAN_RELAY_PIN 27

// 按键引脚
#define MODE_KEY_PIN 13
#define LIGHT_KEY_PIN 14
#define CURTAIN_KEY_PIN 15
#define FAN_KEY_PIN 26

// 系统状态变量
float temperature = 25.5; // 温度（℃）
float humidity = 50.0; // 湿度（%）
int light = 450; // 光照强度（lux）
bool autoMode = true; // 系统模式（true=自动，false=手动）
bool ledState = false; // LED灯状态
bool fanStatus = false; // 风扇状态
String curtainState = "Open"; // 窗帘状态（Open/Close/Half）
String windowState = "Open"; // 窗户状态（Open/Close）
bool isAlarmActive = false; // 报警状态（true=报警中）
unsigned long alarmDisplayTime = 0; // 报警信息切换计时
const unsigned long PRINT_INTERVAL = 2000; // 打印间隔（毫秒）
unsigned long previousPrintTime = 0; // 上一次打印的时间戳
int alarmLine = 0; // 报警显示行切换标记


// 主程序功能函数
void getTempHum();
void updateOledDisplay();
void isAutoControl();
void fanControl();
void printAllDataToSerial();

void setup()
{
    init();
    delay(3000);
}

void loop()
{
    getTempHum();
    isAutoControl();
    updateOledDisplay();
    printAllDataToSerial();
}

/**
 * 初始化传感器
 */
void init()
{
    // 初始化引脚
    pinMode(LED_PIN, OUTPUT);
    pinMode(WIFI_STATUS_LED_PIN, OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);

    pinMode(MODE_KEY_PIN, INPUT_PULLDOWN);
    pinMode(LIGHT_KEY_PIN, INPUT_PULLDOWN);
    pinMode(CURTAIN_KEY_PIN, INPUT_PULLDOWN);
    pinMode(FAN_KEY_PIN, INPUT_PULLDOWN);


    // 初始化串口
    Serial.begin(9600);


    // 初始化DHT11
    dht.begin();


    // 初始化OLED
    u8g2.begin();
    // 启用UTF-8打印
    u8g2.enableUTF8Print();
    // 设置中文字体
    u8g2.setFont(u8g2_font_ncenB12_tr);
    // 初始化显示标题
    u8g2.firstPage();
    do
    {
        u8g2.drawFrame(0, 0, 127, 63); // 屏幕边框（x,y,w,h）
        u8g2.drawStr(36, 24, "Smart");
        u8g2.drawStr(9, 48, "Living Room");
    }
    while (u8g2.nextPage());
}

/**
 * 获取温度和湿度
 */
void getTempHum()
{
    sensors_event_t event;
    // 获取温度
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature))
    {
        temperature = 0.0;
    }
    else
    {
        temperature = event.temperature;
    }
    // 获取湿度
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity))
    {
        humidity = 0.0;
    }
    else
    {
        humidity = event.relative_humidity;
    }
}

/**
 * 更新OLED显示
 */
void updateOledDisplay()
{
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.firstPage();

    do
    {
        char buf[20];
        sprintf(buf, "Temp: %.0f", temperature);
        u8g2.drawStr(0, 12, buf);
        sprintf(buf, "Hum: %.0f", humidity);
        u8g2.drawStr(60, 12, buf);

        sprintf(buf, "Light: %d", light);
        u8g2.drawStr(0, 24, buf);
        sprintf(buf, "Mode: %s", autoMode ? "Auto" : " Manu");
        u8g2.drawStr(60, 24, buf);

        if (isAlarmActive)
        {
            // 每1秒切换报警信息行（闪烁效果）
            if (millis() - alarmDisplayTime > 1000)
            {
                alarmLine = !alarmLine;
                alarmDisplayTime = millis();
            }
            if (alarmLine)
            {
                u8g2.drawStr(0, 36, "Alarm: Smoke detected!");
            }
            else
            {
                u8g2.drawStr(0, 36, "Emergency: Be Safety!");
            }
            u8g2.drawStr(0, 48, "LED is blinking!");
        }
        else
        {
            sprintf(buf, "LED: %s", ledState ? "On" : "Off");
            u8g2.drawStr(0, 36, buf);
            sprintf(buf, "Fan: %s", fanStatus ? "On" : "Off");
            u8g2.drawStr(60, 36, buf);

            sprintf(buf, "Curtain: %s", curtainState.c_str());
            u8g2.drawStr(0, 48, buf);
            sprintf(buf, "Window: %s", windowState.c_str());
            u8g2.drawStr(0, 60, buf);
        }
    }
    while (u8g2.nextPage());
}

/**
 * 自动模式控制
 */
void isAutoControl()
{
    if (autoMode)
    {
        fanControl();
    }
}

/**
 * 风扇控制
 */
void fanControl()
{
    /*
     *温度判断：
     *若温度＞28℃：控制继电器吸合，开启风扇通风；同时 OLED 显示 “风扇开启，降温模式”。
     *若温度≤26℃：控制继电器断开，关闭风扇；OLED 显示 “风扇关闭，温度适宜”。
     *湿度判断（辅助逻辑）：
     *若湿度＞60% 且温度＞25℃：开启风扇加速除湿；若湿度＜40%：风扇保持关闭，避免过度干燥。
    */
    if (temperature > 29)
    {
        digitalWrite(FAN_RELAY_PIN, HIGH);
        fanStatus = true;
    }
    else if (temperature <= 26)
    {
        digitalWrite(FAN_RELAY_PIN, LOW);
        fanStatus = false;
    }
    if (humidity > 60 && temperature > 25)
    {
        digitalWrite(FAN_RELAY_PIN, HIGH);
        fanStatus = true;
    }
    else if (humidity < 50)
    {
        digitalWrite(FAN_RELAY_PIN, LOW);
        fanStatus = false;
    }
}

void printAllDataToSerial()
{
    unsigned long currentTime = millis(); // 获取当前时间
    if (currentTime - previousPrintTime >= PRINT_INTERVAL)
    {
        // 时间到，更新上一次打印时间
        previousPrintTime = currentTime;
        Serial.println(F("==================="));
        Serial.print(F("Temperature: "));
        Serial.println(temperature);
        Serial.print(F("Humidity: "));
        Serial.println(humidity);
        Serial.print(F("Light: "));
        Serial.println(light);
        Serial.print(F("Mode: "));
        Serial.println(autoMode ? "Auto" : "Manual");
        Serial.print(F("LED: "));
        Serial.println(ledState ? "On" : "Off");
        Serial.print(F("Fan: "));
        Serial.println(fanStatus ? "On" : "Off");
        Serial.print(F("Curtain: "));
        Serial.println(curtainState);
        Serial.print(F("Window: "));
        Serial.println(windowState);
        Serial.print(F("Alarm: "));
        Serial.println(isAlarmActive ? "On" : "Off");
        Serial.println(F("==================="));
    }
}
