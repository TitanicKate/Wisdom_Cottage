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

// 传感器引脚
#define BODY_SENSOR_PIN 5
#define LIGHT_SENSOR_PIN 34
#define SMOKE_SENSOR_PIN 35
#define FLAME_SENSOR_PIN 36
#define RAIN_SENSOR_PIN 32
#define SOUND_SENSOR_PIN 33

// 执行器引脚
#define WIFI_STATUS_LED_PIN 2
#define LAMP_RELAY_PIN 25
#define FAN_RELAY_PIN 27
#define BUZZER_PIN 18

// 按键引脚
#define MODE_KEY_PIN 13
#define LAMP_KEY_PIN 14
#define CURTAIN_KEY_PIN 15
#define FAN_KEY_PIN 26

// 系统状态变量
float temperature = 0.0; // 温度（℃）
float humidity = 0.0; // 湿度（%）
int lux = 0; // 光照强度（lux）
int body = 0; // 人体 0=无人, 1=有人
int flame = 0; // 火焰 1=无火焰，0=有火焰
int smoke = 0; // 烟雾 1=无烟雾，0=有烟雾
int rain = 0; // 雨量 1=无雨，0=有雨
int sound = 0; // 1=无声，0=有声
int curtainAngle = 0; // 0=关闭，90=打开
int windowAngle = 0; // 0=打开，90=关闭
int alarmLine = 0; // 报警显示行切换标记
bool autoMode = true; // 系统模式（true=自动，false=手动）
bool ledState = false; // LED灯状态
bool fanStatus = false; // 风扇状态
bool isAlarmActive = false; // 报警状态（true=报警中）
String curtainState = "Open"; // 窗帘状态（Open/Close/Half）
String windowState = "Open"; // 窗户状态（Open/Close）
unsigned long alarmDisplayTime = 0; // 报警信息切换计时
unsigned long lastPirTime = 0; // 上次检测 Motion Sensor 的时间
unsigned long previousPrintTime = 0; // 上一次打印的时间戳
unsigned long PRINT_INTERVAL = 2000; // 打印间隔（毫秒）
unsigned long alarmStartTime = 0; // 报警开始时间戳
unsigned long ALARM_DURATION = 5000; // 报警持续时间（5秒）


// 主程序功能函数

void getSensorsData();
void getTempHum();
void getLux();
void getBody();
void getSmoke();
void getFlame();
void getRain();
void getSound();

void isAutoControl();
void bodyControl();
void fanControl();
void isAlarmActived();

void updateOledDisplay();
void printAllDataToSerial();

void isModeKeyPressed();
void isLightKeyPressed();
void isCurtainKeyPressed();
void isFanKeyPressed();

void setup()
{
    init();
    delay(3000);
}

void loop()
{
    getSensorsData();
    isAlarmActived();
    isAutoControl();
    updateOledDisplay();
    printAllDataToSerial();
}

/**
 * 初始化传感器
 */
void init()
{
    // 初始化输入引脚
    pinMode(BODY_SENSOR_PIN, INPUT);
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    pinMode(SOUND_SENSOR_PIN, INPUT);
    pinMode(SMOKE_SENSOR_PIN, INPUT);
    pinMode(FLAME_SENSOR_PIN, INPUT);
    pinMode(RAIN_SENSOR_PIN, INPUT);
    // 初始化输出引脚
    pinMode(WIFI_STATUS_LED_PIN, OUTPUT);
    pinMode(LAMP_RELAY_PIN, OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    pinMode(MODE_KEY_PIN, INPUT_PULLDOWN);
    pinMode(LAMP_KEY_PIN, INPUT_PULLDOWN);
    pinMode(CURTAIN_KEY_PIN, INPUT_PULLDOWN);
    pinMode(FAN_KEY_PIN, INPUT_PULLDOWN);

    // 配置ADC分辨率
    // analogReadResolution(12);
    // 配置ADC衰减值
    // analogSetAttenuation(ADC_11db);

    // 配置按键引脚中断
    attachInterrupt(digitalPinToInterrupt(MODE_KEY_PIN), isModeKeyPressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(LAMP_KEY_PIN), isLightKeyPressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(CURTAIN_KEY_PIN), isCurtainKeyPressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(FAN_KEY_PIN), isFanKeyPressed, FALLING);


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

void getSensorsData()
{
    getTempHum();
    getLux();
    getBody();
    getSmoke();
    getFlame();
    getRain();
    getSound();
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
 * 获取光照强度
 */
void getLux()
{
    // 将ADC值转换为光照强度
    // lux = (float)analogRead(LIGHT_SENSOR_PIN) / 4095 * 100;
    // lux = analogRead(LIGHT_SENSOR_PIN);
    lux = digitalRead(LIGHT_SENSOR_PIN);
}

/**
 * 获取有人状态
 */
void getBody()
{
    body = digitalRead(BODY_SENSOR_PIN);
}

/**
 * 获取烟雾状态
 */
void getSmoke()
{
    smoke = digitalRead(SMOKE_SENSOR_PIN);
}

/**
 * 获取火焰状态
 */
void getFlame()
{
    flame = digitalRead(FLAME_SENSOR_PIN);
}

/**
 * 获取雨量
 */
void getRain()
{
    rain = digitalRead(RAIN_SENSOR_PIN);
}

/**
 * 获取声音状态
 */
void getSound()
{
    sound = digitalRead(SOUND_SENSOR_PIN);
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

        sprintf(buf, "Lux: %d", lux);
        u8g2.drawStr(0, 24, buf);
        sprintf(buf, "Mode: %s", autoMode ? "Auto" : "Manu");
        u8g2.drawStr(60, 24, buf);

        sprintf(buf, "LED: %s", ledState ? "On" : "Off");
        u8g2.drawStr(0, 36, buf);
        sprintf(buf, "Fan: %s", fanStatus ? "On" : "Off");
        u8g2.drawStr(60, 36, buf);

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
                u8g2.drawStr(0, 48, "Alarm: Flame detected!");
            }
            else
            {
                u8g2.drawStr(0, 48, "Alarm: Smoke detected!");
            }

            u8g2.drawStr(0, 60, "Emergency: Be Safety!");
        }
        else
        {
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
        bodyControl();
        fanControl();
    }
}

/**
 * 人体控制
 */
void bodyControl()
{
    if (body)
    {
        lastPirTime = millis();
        // int lightVal = analogRead(LIGHT_SENSOR_PIN);
        // 光照不足时开灯
        if (lux && !ledState)
        {
            ledState = true;
            // analogWrite(LAMP_RELAY_PIN, 178);
            digitalWrite(LAMP_RELAY_PIN, HIGH);
        }
        // 打开窗帘
        if (curtainAngle == 0)
        {
            curtainAngle = 90;
            // curtainServo.write(curtainAngle);
        }
    }
    else
    {
        // 30秒无人关闭设备
        if (millis() - lastPirTime > 30000)
        {
            if (ledState)
            {
                ledState = false;
                // analogWrite(LAMP_RELAY_PIN, 0);
                digitalWrite(LAMP_RELAY_PIN, LOW);
            }
            if (curtainAngle == 90)
            {
                curtainAngle = 0;
                // curtainServo.write(curtainAngle);
            }
        }
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


void isAlarmActived()
{
    // 检测是否有火焰或烟雾（任意一个触发报警）
    bool isDanger = (flame == 0) || (smoke == 0);

    if (isDanger)
    {
        // 触发报警（首次触发或再次检测到危险时更新开始时间）
        if (!isAlarmActive)
        {
            isAlarmActive = true;
            alarmStartTime = millis(); // 记录报警开始时间
            digitalWrite(BUZZER_PIN, HIGH); // 启动蜂鸣器
            Serial.println("检测到危险，开始报警！");
        }
        else
        {
            // 持续处于危险状态，刷新报警时间（避免10秒后误解除）
            alarmStartTime = millis();
        }
    }
    else
    {
        // 无危险时，判断是否已报警且达到10秒
        if (isAlarmActive)
        {
            unsigned long currentTime = millis();
            // 检查是否已报警满10秒
            if (currentTime - alarmStartTime >= ALARM_DURATION)
            {
                // 10秒后无危险，解除报警
                isAlarmActive = false;
                digitalWrite(BUZZER_PIN, LOW); // 关闭蜂鸣器
                Serial.println("10秒内无危险，解除报警");
            }
            // 若未满10秒，继续保持报警状态
        }
    }
}

/**
 * 串口打印数据
 */
void printAllDataToSerial()
{
    unsigned long currentTime = millis(); // 获取当前时间
    if (currentTime - previousPrintTime >= PRINT_INTERVAL)
    {
        // 时间到，更新上一次打印时间
        previousPrintTime = currentTime;
        Serial.println(F("=======Data======="));
        Serial.print(F("Temperature: "));
        Serial.print(temperature);
        Serial.print(F("  Humidity: "));
        Serial.print(humidity);
        Serial.print(F("  Lux: "));
        Serial.println(lux);
        Serial.print(F("Body: "));
        Serial.print(body ? "Yes" : "No ");
        Serial.print(F("           Smoke: "));
        Serial.print(smoke ? "No " : "Yes");
        Serial.print(F("      Flame: "));
        Serial.println(flame ? "No " : "Yes");
        Serial.print(F("Rain: "));
        Serial.print(rain ? "No " : "Yes");
        Serial.print(F("          Sound: "));
        Serial.println(sound ? "No" : "Yes");
        Serial.println(F("------Status-------"));
        Serial.print(F("Mode: "));
        Serial.println(autoMode ? "Auto" : "Manual");
        Serial.print(F("LED: "));
        Serial.print(ledState ? "On" : "Off");
        Serial.print(F("  Fan: "));
        Serial.println(fanStatus ? "On" : "Off");
        Serial.print(F("Curtain: "));
        Serial.print(curtainState);
        Serial.print(F("  Window: "));
        Serial.println(windowState);
        Serial.print(F("Alarm: "));
        Serial.println(isAlarmActive ? "On" : "Off");
        Serial.println(F("==================="));
    }
}

void isModeKeyPressed()
{
    autoMode = !autoMode;
}

void isLightKeyPressed()
{
    if (!autoMode)
    {
        ledState = !ledState;
        digitalWrite(LAMP_RELAY_PIN, ledState ? HIGH : LOW);
    }
}

void isFanKeyPressed()
{
    if (!autoMode)
    {
        fanStatus = !fanStatus;
        digitalWrite(FAN_RELAY_PIN, fanStatus ? HIGH : LOW);
    }
}

void isCurtainKeyPressed()
{
    if (!autoMode)
    {
    }
}
