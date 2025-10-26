#include <Arduino.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>  // 新增WiFi库

// WiFi配置 - 新增WiFi参数
namespace WiFiConfig
{
    const char* SSID = "TitanicKate"; // 替换为你的WiFi名称
    const char* PASSWORD = "gat040506"; // 替换为你的WiFi密码
    const unsigned long RECONNECT_INTERVAL = 5000; // 重连间隔（5秒）
}

// 引脚定义 - 保留原有并增加WiFi状态LED
namespace Pins
{
    // 原有引脚定义...
    constexpr int WIFI_LED = 2; // 用于指示WiFi状态（板载LED）
    // 其他引脚保持不变...
    constexpr int DHT_PIN = 4;
    constexpr int BODY_SENSOR = 5;
    constexpr int LIGHT_SENSOR = 34;
    constexpr int SMOKE_SENSOR = 35;
    constexpr int FLAME_SENSOR = 36;
    constexpr int RAIN_SENSOR = 32;
    constexpr int SOUND_SENSOR = 33;
    constexpr int LAMP_RELAY = 25;
    constexpr int FAN_RELAY = 27;
    constexpr int BUZZER = 18;
    constexpr int DOOR_SERVO = 12;
    constexpr int CURTAIN_SERVO = 19;
    constexpr int MODE_KEY = 13;
    constexpr int LAMP_KEY = 14;
    constexpr int CURTAIN_KEY = 15;
    constexpr int FAN_KEY = 26;
}

// 常量定义 - 保留原有
namespace Constants
{
    // 原有常量保持不变...
    constexpr auto DHT_TYPE = DHT11;
    constexpr int DOOR_CHANNEL = 0;
    constexpr int CURTAIN_CHANNEL = 1;
    constexpr int FREQ = 50;
    constexpr int RESOLUTION = 12;
    constexpr int MIN_ANGLE = 0;
    constexpr int MAX_ANGLE = 180;
    constexpr unsigned long PRINT_INTERVAL = 2000;
    constexpr unsigned long CURTAIN_MOVE_DURATION = 8000;
    constexpr unsigned long ALARM_DURATION = 5000;
    constexpr unsigned long NO_BODY_TIMEOUT = 30000;
    constexpr unsigned long ALARM_DISPLAY_INTERVAL = 1000;
    constexpr float TEMP_HIGH = 29.0f;
    constexpr float TEMP_LOW = 26.0f;
    constexpr float HUMIDITY_HIGH = 60.0f;
    constexpr float HUMIDITY_LOW = 50.0f;
    constexpr float TEMP_HUMIDITY_TRIGGER = 25.0f;
}

// 系统状态 - 新增WiFi相关状态
struct SystemState
{
    // 原有状态变量...
    float temperature = 0.0f;
    float humidity = 0.0f;
    int lux = 0;
    bool bodyDetected = false;
    bool flameDetected = false;
    bool smokeDetected = false;
    bool rainDetected = false;
    bool soundDetected = false;
    bool autoMode = true;
    bool ledState = false;
    bool fanState = false;
    bool alarmActive = false;
    bool doorOpen = false;
    bool curtainOpen = false;
    int curtainAngle = 90;
    bool isCurtainMoving = false;
    unsigned long curtainActionTime = 0;
    unsigned long lastPirTime = 0;
    unsigned long previousPrintTime = 0;
    unsigned long alarmStartTime = 0;
    unsigned long alarmDisplayTime = 0;
    int alarmLine = 0;
    bool lastCurtainState = false;

    // 新增WiFi状态变量
    bool wifiConnected = false; // WiFi连接状态
    unsigned long lastReconnectAttempt = 0; // 上次重连时间
} state;

// 全局对象 - 保留原有
DHT_Unified dht(Pins::DHT_PIN, Constants::DHT_TYPE);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);

// 函数声明 - 新增WiFi相关函数
void initSystem();
void readSensors();
void controlDevices();
void updateDisplay();
void handleAlarms();
void printDebugInfo();
void connectWiFi(); // 新增：连接WiFi
void checkWiFiConnection(); // 新增：检查WiFi连接状态

// 原有函数保持不变...
void setServoAngle(int angle, int channel)
{
    angle = constrain(angle, Constants::MIN_ANGLE, Constants::MAX_ANGLE);
    const float pulseWidthMs = 0.5f + (angle / 180.0f) * 2.0f;
    const int duty = static_cast<int>((pulseWidthMs / 20.0f) * ((1 << Constants::RESOLUTION) - 1));
    ledcWrite(channel, duty);
}

void controlCurtain()
{
    if (state.curtainOpen == state.lastCurtainState) return;

    if (state.curtainOpen)
    {
        if (!state.rainDetected)
        {
            if (!state.isCurtainMoving && state.curtainAngle == 90)
            {
                setServoAngle(80, Constants::CURTAIN_CHANNEL);
                state.curtainAngle = 80;
                state.isCurtainMoving = true;
                state.curtainActionTime = millis();
            }
            else if (state.isCurtainMoving && state.curtainAngle == 80)
            {
                if (millis() - state.curtainActionTime >= Constants::CURTAIN_MOVE_DURATION)
                {
                    setServoAngle(90, Constants::CURTAIN_CHANNEL);
                    state.curtainAngle = 90;
                    state.isCurtainMoving = false;
                    state.lastCurtainState = state.curtainOpen;
                }
            }
        }
        else
        {
            state.lastCurtainState = state.curtainOpen;
        }
    }
    else
    {
        if (!state.isCurtainMoving && state.curtainAngle == 90)
        {
            setServoAngle(100, Constants::CURTAIN_CHANNEL);
            state.curtainAngle = 100;
            state.isCurtainMoving = true;
            state.curtainActionTime = millis();
        }
        else if (state.isCurtainMoving && state.curtainAngle == 100)
        {
            if (millis() - state.curtainActionTime >= Constants::CURTAIN_MOVE_DURATION)
            {
                setServoAngle(90, Constants::CURTAIN_CHANNEL);
                state.curtainAngle = 90;
                state.isCurtainMoving = false;
                state.lastCurtainState = state.curtainOpen;
            }
        }
    }
}

void controlDoor()
{
    setServoAngle(state.doorOpen ? 90 : 180, Constants::DOOR_CHANNEL);
}

void bodySensorControl()
{
    if (state.bodyDetected)
    {
        state.lastPirTime = millis();
        if (state.lux && !state.ledState)
        {
            state.ledState = true;
            digitalWrite(Pins::LAMP_RELAY, HIGH);
        }
        state.curtainOpen = true;
    }
    else
    {
        if (millis() - state.lastPirTime > Constants::NO_BODY_TIMEOUT)
        {
            if (state.ledState)
            {
                state.ledState = false;
                digitalWrite(Pins::LAMP_RELAY, LOW);
            }
            state.curtainOpen = false;
        }
    }
}

void fanControl()
{
    bool fanNeeded = false;
    if (state.temperature > Constants::TEMP_HIGH) fanNeeded = true;
    else if (state.temperature <= Constants::TEMP_LOW) fanNeeded = false;
    if (state.humidity > Constants::HUMIDITY_HIGH && state.temperature > Constants::TEMP_HUMIDITY_TRIGGER)
        fanNeeded =
            true;
    else if (state.humidity < Constants::HUMIDITY_LOW) fanNeeded = false;
    if (fanNeeded != state.fanState)
    {
        state.fanState = fanNeeded;
        digitalWrite(Pins::FAN_RELAY, state.fanState ? HIGH : LOW);
    }
}

// 中断服务函数保持不变...
void IRAM_ATTR onModeKeyPress() { state.autoMode = !state.autoMode; }

void IRAM_ATTR onLightKeyPress()
{
    if (!state.autoMode)
    {
        state.ledState = !state.ledState;
        digitalWrite(Pins::LAMP_RELAY, state.ledState ? HIGH : LOW);
    }
}

void IRAM_ATTR onCurtainKeyPress() { if (!state.autoMode) state.curtainOpen = !state.curtainOpen; }

void IRAM_ATTR onFanKeyPress()
{
    if (!state.autoMode)
    {
        state.fanState = !state.fanState;
        digitalWrite(Pins::FAN_RELAY, state.fanState ? HIGH : LOW);
    }
}

// 新增：WiFi连接函数
void connectWiFi()
{
    Serial.print("Connect to WiFi: ");
    Serial.println(WiFiConfig::SSID);
    WiFi.begin(WiFiConfig::SSID, WiFiConfig::PASSWORD);

    // 等待连接（超时10秒）
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000)
    {
        delay(500);
        Serial.print(".");
        digitalWrite(Pins::WIFI_LED, !digitalRead(Pins::WIFI_LED)); // 闪烁指示正在连接
    }

    // 连接结果判断
    if (WiFi.status() == WL_CONNECTED)
    {
        state.wifiConnected = true;
        digitalWrite(Pins::WIFI_LED, HIGH); // 常亮表示连接成功
        Serial.println("\nWiFi connect successful!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        state.wifiConnected = false;
        digitalWrite(Pins::WIFI_LED, LOW); // 熄灭表示连接失败
        Serial.println("\nWiFi connect failed, Please retry...");
    }
}

// 新增：检查WiFi连接状态，断线重连
void checkWiFiConnection()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        state.wifiConnected = false;
        digitalWrite(Pins::WIFI_LED, LOW);

        // 每隔5秒尝试重连
        if (millis() - state.lastReconnectAttempt >= WiFiConfig::RECONNECT_INTERVAL)
        {
            state.lastReconnectAttempt = millis();
            Serial.println("WiFi disconnected, attempting to reconnect...");
            connectWiFi(); // 调用连接函数
        }
    }
    else
    {
        state.wifiConnected = true;
        digitalWrite(Pins::WIFI_LED, HIGH);
    }
}

// 系统初始化 - 新增WiFi初始化
void initSystem()
{
    // 原有初始化代码...
    pinMode(Pins::WIFI_LED, OUTPUT);
    pinMode(Pins::BODY_SENSOR, INPUT);
    pinMode(Pins::LIGHT_SENSOR, INPUT);
    pinMode(Pins::SOUND_SENSOR, INPUT);
    pinMode(Pins::SMOKE_SENSOR, INPUT);
    pinMode(Pins::FLAME_SENSOR, INPUT);
    pinMode(Pins::RAIN_SENSOR, INPUT);
    pinMode(Pins::LAMP_RELAY, OUTPUT);
    pinMode(Pins::FAN_RELAY, OUTPUT);
    pinMode(Pins::BUZZER, OUTPUT);
    pinMode(Pins::MODE_KEY, INPUT_PULLDOWN);
    pinMode(Pins::LAMP_KEY, INPUT_PULLDOWN);
    pinMode(Pins::CURTAIN_KEY, INPUT_PULLDOWN);
    pinMode(Pins::FAN_KEY, INPUT_PULLDOWN);

    ledcSetup(Constants::DOOR_CHANNEL, Constants::FREQ, Constants::RESOLUTION);
    ledcAttachPin(Pins::DOOR_SERVO, Constants::DOOR_CHANNEL);
    ledcSetup(Constants::CURTAIN_CHANNEL, Constants::FREQ, Constants::RESOLUTION);
    ledcAttachPin(Pins::CURTAIN_SERVO, Constants::CURTAIN_CHANNEL);

    attachInterrupt(digitalPinToInterrupt(Pins::MODE_KEY), onModeKeyPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pins::LAMP_KEY), onLightKeyPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pins::CURTAIN_KEY), onCurtainKeyPress, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pins::FAN_KEY), onFanKeyPress, FALLING);

    Serial.begin(9600);
    dht.begin();
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_ncenB12_tr);

    // 初始化显示
    u8g2.firstPage();
    do
    {
        u8g2.drawFrame(0, 0, 127, 63);
        u8g2.drawStr(36, 24, "Smart");
        u8g2.drawStr(9, 48, "Living Room");
    }
    while (u8g2.nextPage());
    delay(3000);

    // 新增：上电时启动WiFi连接
    connectWiFi();
}

// 读取传感器 - 保持不变
void readSensors()
{
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    state.temperature = isnan(event.temperature) ? 0.0f : event.temperature;
    dht.humidity().getEvent(&event);
    state.humidity = isnan(event.relative_humidity) ? 0.0f : event.relative_humidity;
    state.lux = digitalRead(Pins::LIGHT_SENSOR);
    state.bodyDetected = digitalRead(Pins::BODY_SENSOR);
    state.smokeDetected = !digitalRead(Pins::SMOKE_SENSOR);
    state.flameDetected = !digitalRead(Pins::FLAME_SENSOR);
    state.rainDetected = !digitalRead(Pins::RAIN_SENSOR);
    state.soundDetected = !digitalRead(Pins::SOUND_SENSOR);
}

// 设备控制 - 保持不变
void controlDevices()
{
    if (state.autoMode)
    {
        bodySensorControl();
        fanControl();
    }
    controlCurtain();
    controlDoor();
}

// 报警处理 - 保持不变
void handleAlarms()
{
    const bool isDanger = state.flameDetected || state.smokeDetected;
    if (isDanger)
    {
        if (!state.alarmActive)
        {
            state.alarmActive = true;
            state.alarmStartTime = millis();
            digitalWrite(Pins::BUZZER, HIGH);
        }
        else
        {
            state.alarmStartTime = millis();
        }
    }
    else if (state.alarmActive && millis() - state.alarmStartTime >= Constants::ALARM_DURATION)
    {
        state.alarmActive = false;
        digitalWrite(Pins::BUZZER, LOW);
    }
    if (state.alarmActive && millis() - state.alarmDisplayTime > Constants::ALARM_DISPLAY_INTERVAL)
    {
        state.alarmLine = !state.alarmLine;
        state.alarmDisplayTime = millis();
    }
}

// 更新显示 - 新增WiFi状态显示
void updateDisplay()
{
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.firstPage();
    do
    {
        char buf[20];
        // 第1行：温湿度 + WiFi状态
        sprintf(buf, "Temp: %.0f°C", state.temperature);
        u8g2.drawStr(0, 12, buf);
        sprintf(buf, "Hum: %.0f%%", state.humidity);
        u8g2.drawStr(60, 12, buf);

        // 第2行：光照 + 模式
        sprintf(buf, "Lux: %d", state.lux);
        u8g2.drawStr(0, 24, buf);
        sprintf(buf, "Mode: %s", state.autoMode ? "Auto" : "Manu");
        u8g2.drawStr(60, 24, buf);

        // 第3行：LED + 风扇
        sprintf(buf, "LED: %s", state.ledState ? "On" : "Off");
        u8g2.drawStr(0, 36, buf);
        sprintf(buf, "Fan: %s", state.fanState ? "On" : "Off");
        u8g2.drawStr(60, 36, buf);

        // 第4行：窗帘 + 门（报警时显示报警信息）
        if (state.alarmActive)
        {
            u8g2.drawStr(0, 48, state.alarmLine ? "Alarm: Flame!" : "Alarm: Smoke!");
            u8g2.drawStr(0, 56, "Emergency!");
        }
        else
        {
            sprintf(buf, "Curtain: %s", state.curtainOpen ? "Open" : "Close");
            u8g2.drawStr(0, 48, buf);
            sprintf(buf, "Door: %s", state.doorOpen ? "Open" : "Close");
            u8g2.drawStr(0, 56, buf);
        }
    }
    while (u8g2.nextPage());
}

// 打印调试信息 - 新增WiFi状态
void printDebugInfo()
{
    const unsigned long currentTime = millis();
    if (currentTime - state.previousPrintTime >= Constants::PRINT_INTERVAL)
    {
        state.previousPrintTime = currentTime;
        Serial.println("=======Sensor Data=======");
        Serial.printf("Temp: %.1f°C  Hum: %.1f%%  Lux: %d\n",
                      state.temperature, state.humidity, state.lux);
        Serial.printf("Body: %s  Smoke: %s  Flame: %s\n",
                      state.bodyDetected ? "Yes" : "No",
                      state.smokeDetected ? "Yes" : "No",
                      state.flameDetected ? "Yes" : "No");
        Serial.printf("Rain: %s  Sound: %s\n",
                      state.rainDetected ? "Yes" : "No",
                      state.soundDetected ? "Yes" : "No");

        Serial.println("------Device Status------");
        Serial.printf("Mode: %s  LED: %s  Fan: %s\n",
                      state.autoMode ? "Auto" : "Manual",
                      state.ledState ? "On" : "Off",
                      state.fanState ? "On" : "Off");
        Serial.printf("Curtain: %s  Door: %s  Alarm: %s\n",
                      state.curtainOpen ? "Open" : "Close",
                      state.doorOpen ? "Open" : "Close",
                      state.alarmActive ? "On" : "Off");
        // 新增WiFi状态打印
        Serial.printf("WiFi: %s  IP: %s\n",
                      state.wifiConnected ? "Connected" : "Disconnected",
                      state.wifiConnected ? WiFi.localIP().toString().c_str() : "N/A");
        Serial.println("=========================");
    }
}

// 主函数 - 新增WiFi检查
void setup()
{
    initSystem();
}

void loop()
{
    readSensors();
    handleAlarms();
    controlDevices();
    checkWiFiConnection(); // 循环检查WiFi连接状态
    updateDisplay();
    printDebugInfo();
}
