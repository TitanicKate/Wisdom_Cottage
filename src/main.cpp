#include <Arduino.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <DHT_U.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

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
    constexpr int L_MIN = 0;
    constexpr int L_MAX = 10000;
    constexpr int V_MIN = 0;
    constexpr int V_MAX = 3300;
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
    constexpr unsigned long NO_RAIN_TIMEOUT = 60000;
    constexpr unsigned long ALARM_DISPLAY_INTERVAL = 1000;
    constexpr float TEMP_HIGH = 29.0f;
    constexpr float TEMP_LOW = 26.0f;
    constexpr float HUMIDITY_HIGH = 60.0f;
    constexpr float HUMIDITY_LOW = 50.0f;
    constexpr float TEMP_HUMIDITY_TRIGGER = 25.0f;
}

// ThingsBoard配置 (添加到WiFiConfig命名空间后)
namespace ThingsBoardConfig
{
    const char* SERVER = "106.53.71.21";
    const uint16_t PORT = 1883U;
    const char* TOKEN = "qvSsCLND8g5ufB13ngPg";
    const uint32_t MAX_MESSAGE_SIZE = 1024U;
    const uint32_t REQUEST_TIMEOUT = 5000U * 1000U;
    const size_t MAX_ATTRIBUTES = 5U;
    const int TELEMETRY_INTERVAL = 2000U; // 与原有打印间隔保持一致
}

// 传感器和执行器属性名称定义 (添加到Pins命名空间后)
namespace AttrNames
{
    const char* TEMP = "temperature";
    const char* HUMIDITY = "humidity";
    const char* LUX = "lux";
    const char* BODY = "bodyDetected";
    const char* SMOKE = "smokeDetected";
    const char* FLAME = "flameDetected";
    const char* RAIN = "rainDetected";
    const char* SOUND = "soundDetected";
    const char* LED = "ledState";
    const char* FAN = "fanState";
    const char* CURTAIN = "curtainOpen";
    const char* DOOR = "doorOpen";
    const char* ALARM = "alarmActive";
    const char* RSSI = "rssi";
    const char* CHANNEL = "channel";
    const char* BSSID = "bssid";
    const char* LOCALIP = "localIp";
    const char* SSID = "ssid";
    const char* SYSTEM_MODE = "autoMode";
    const char* DEVICE_NAME = "deviceName";
    const char* MAC_ADDRESS = "macAddress";
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
    unsigned long lastLightReadTime = 0;
    int alarmLine = 0;
    bool lastCurtainState = false;
    bool modeChanged = false;

    // 新增WiFi状态变量
    bool wifiConnected = false; // WiFi连接状态
    unsigned long lastReconnectAttempt = 0; // 上次重连时间
} state;

// 存储获取到的时间
struct
{
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} networkTime;

// 全局对象 - 保留原有
DHT_Unified dht(Pins::DHT_PIN, Constants::DHT_TYPE);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
Server_Side_RPC<5U, 10U> rpc;
Attribute_Request<2U, ThingsBoardConfig::MAX_ATTRIBUTES> attr_request;
Shared_Attribute_Update<3U, ThingsBoardConfig::MAX_ATTRIBUTES> shared_update;
const std::array<IAPI_Implementation*, 3U> apis = {
    &rpc,
    &attr_request,
    &shared_update
};

WiFiUDP ntpUDP; // 创建UDP实例
NTPClient timeClient(ntpUDP,
                     "ntp.aliyun.com", // 阿里云NTP服务器（推荐）
                     8 * 3600, // 时区偏移：北京时间UTC+8，单位秒
                     60000 // 同步间隔：1分钟（60000毫秒）
);

ThingsBoard tb(mqttClient, ThingsBoardConfig::MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);
uint32_t previousTbSend = 0;

// 函数声明 - 新增WiFi相关函数
void initSystem();
void readSensors();
void controlDevices();
void updateDisplay();
void handleAlarms();
void printDebugInfo();
void connectWiFi(); // 新增：连接WiFi
void checkWiFiConnection(); // 新增：检查WiFi连接状态
void updateNetworkTime();

// 初始化NTP并同步时间
void initNTP()
{
    timeClient.begin(); // 启动NTP客户端
    // 等待第一次同步成功（最多等待5秒）
    unsigned long startSyncTime = millis();
    while (!timeClient.update() && millis() - startSyncTime < 5000)
    {
        delay(500);
        Serial.print("Synchronizing time...");
    }

    if (timeClient.update())
    {
        Serial.println("\nThe first time synchronization was successful!");
        updateNetworkTime(); // 更新时间到结构体
    }
    else
    {
        Serial.println("\nThe first time synchronization fails, it will be retried at the next interval...");
    }
}

// 更新时间到networkTime结构体
void updateNetworkTime()
{
    if (timeClient.update())
    {
        // 检查是否需要更新（按设置的间隔）
        // networkTime.year = timeClient.getYear();
        // networkTime.month = timeClient.getMonth();
        networkTime.day = timeClient.getDay();
        networkTime.hour = timeClient.getHours(); // 已自动叠加时区偏移（UTC+8）
        networkTime.minute = timeClient.getMinutes();
        networkTime.second = timeClient.getSeconds();
    }
}

// 原有函数保持不变...
void setServoAngle(int angle, int channel)
{
    angle = constrain(angle, Constants::MIN_ANGLE, Constants::MAX_ANGLE);
    const float pulseWidthMs = 0.5f + (angle / 180.0f) * 2.0f;
    const int duty = static_cast<int>((pulseWidthMs / 20.0f) * ((1 << Constants::RESOLUTION) - 1));
    ledcWrite(channel, duty);
}

void curtainControl()
{
    // --------------------------
    // 1. 最高优先级：下雨时强制关闭窗帘
    // --------------------------
    if (state.rainDetected)
    {
        // 目标状态设为关闭
        state.curtainOpen = false;
    }
    // --------------------------
    // 2. 次高优先级：无雨时按时间段判断基础状态
    // --------------------------
    else
    {
        int currentHour = networkTime.hour; // 获取当前小时（24小时制，0-23）
        bool isNight = (currentHour >= 18) || (currentHour < 7); // 18:00-7:00为夜晚

        // 时间段基础状态：夜晚关，白天开
        bool timeBasedOpen = !isNight;

        // --------------------------
        // 3. 最低优先级：按光照值修正时间段状态
        // --------------------------
        if (timeBasedOpen)
        {
            // 白天基础状态为开，若光照≥400则修正为关
            state.curtainOpen = (state.lux < 400);
        }
        else
        {
            // 夜晚基础状态为关，若光照<400也不打开（保持关闭）
            state.curtainOpen = false;
        }
    }
}

void curtainAction()
{
    if (state.curtainOpen == state.lastCurtainState) return;

    if (state.curtainOpen)
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

void doorControl()
{
    setServoAngle(state.doorOpen ? 90 : 180, Constants::DOOR_CHANNEL);
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

void ledControl()
{
    // 人体检测消抖参数（可根据传感器特性调整）
    static const int DEBOUNCE_COUNT = 3;  // 连续检测次数（稳定触发阈值）
    static int bodyDetectedCount = 0;     // 连续检测到有人的计数

    // 获取当前小时（24小时制，依赖NTP同步的时间）
    int currentHour = networkTime.hour;
    // 判断是否为晚上时间段（18:00-次日7:00）
    bool isNight = (currentHour >= 18) || (currentHour < 7);

    if (state.bodyDetected) {
        // 每次检测到有人，计数+1（不超过阈值）
        if (bodyDetectedCount < DEBOUNCE_COUNT) {
            bodyDetectedCount++;
        }
    } else {
        // 一旦检测到无人，计数清零（重置验证）
        bodyDetectedCount = 0;
    }

    // 消抖后判定：连续DEBOUNCE_COUNT次检测到有人，才视为真实有人
    bool isBodyReallyDetected = (bodyDetectedCount >= DEBOUNCE_COUNT);

    // --------------------------
    // 灯光控制逻辑（基于消抖后的结果）
    // --------------------------

    if (isBodyReallyDetected) {
        // 检测到人体时，更新最后活动时间
        state.lastPirTime = millis();

        if (isNight) {
            // 晚上：优先开启灯光（无论光照，只要没人关闭过）
            if (!state.ledState) {
                state.ledState = true;
                digitalWrite(Pins::LAMP_RELAY, HIGH);
            }
        } else {
            // 白天：按光照值判断（≤300开，>300关）
            if (state.lux <= 300 && !state.ledState) {
                state.ledState = true;
                digitalWrite(Pins::LAMP_RELAY, HIGH);
            } else if (state.lux > 300 && state.ledState) {
                state.ledState = false;
                digitalWrite(Pins::LAMP_RELAY, LOW);
            }
        }
    } else {
        // 未检测到人体，超时后关闭灯光
        if (millis() - state.lastPirTime > Constants::NO_BODY_TIMEOUT) {
            if (state.ledState) {
                state.ledState = false;
                digitalWrite(Pins::LAMP_RELAY, LOW);
            }
        }
    }
}

// 中断服务函数保持不变...
void IRAM_ATTR onModeKeyPress()
{
    state.autoMode = !state.autoMode;
    state.modeChanged = true; // 标记模式已变化
}

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

        initNTP();
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

    unsigned long currentTime = millis(); // 获取当前时间（毫秒）
    if (currentTime - state.lastLightReadTime >= 1000)
    {
        // 间隔≥1秒
        // 计算光照值（保持原公式逻辑）
        state.lux = (Constants::L_MIN + (Constants::V_MAX - analogReadMilliVolts(Pins::LIGHT_SENSOR))
            * (Constants::L_MAX - Constants::L_MIN) / Constants::V_MAX) / 10;
        state.lastLightReadTime = currentTime; // 更新时间戳为当前时间
    }
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
        curtainControl();
        fanControl();
        ledControl();
    }
    curtainAction();
    doorControl();
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
        sprintf(buf, "Temp: %.0f", state.temperature);
        u8g2.drawStr(0, 12, buf);
        sprintf(buf, "Hum: %.0f", state.humidity);
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
            u8g2.drawStr(0, 60, "Emergency!");
        }
        else
        {
            sprintf(buf, "Curtain: %s", state.curtainOpen ? "Open" : "Close");
            u8g2.drawStr(0, 48, buf);
            sprintf(buf, "Door: %s", state.doorOpen ? "Open" : "Close");
            u8g2.drawStr(0, 60, buf);
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
        Serial.println("\n=======Sensor Data=======");
        Serial.println(timeClient.getFormattedTime());
        Serial.printf("Temp: %.1f  Hum: %.1f  Lux: %d\n",
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
        Serial.println("=========================\n");
    }
}

void checkModeChanged()
{
    if (state.modeChanged)
    {
        // 关闭中断，安全读取标记和新值（避免中断中途修改）
        noInterrupts();
        bool currentNewMode = state.autoMode;
        state.modeChanged = false; // 清除标记
        interrupts(); // 恢复中断

        // 更新系统模式
        bool oldMode = state.autoMode;
        state.autoMode = currentNewMode;
        Serial.printf("System mode switches to: %s\n", state.autoMode ? "Auto" : "Manu");

        // 仅在模式实际变化且TB连接时，发送属性和遥测
        if (state.autoMode != oldMode && tb.connected())
        {
            // 发送属性（更新当前状态）
            tb.sendAttributeData(AttrNames::SYSTEM_MODE, state.autoMode);
            // 发送遥测（记录历史变化）
            tb.sendTelemetryData(AttrNames::SYSTEM_MODE, state.autoMode);
            Serial.println("System mode changes are synchronized to the server");
        }
    }
}

// RPC回调函数实现
void processSetAutoMode(const JsonVariantConst& data, JsonDocument& response)
{
    bool newMode = data.as<bool>();
    state.autoMode = newMode;
    // 发送属性更新服务器状态
    tb.sendAttributeData(AttrNames::SYSTEM_MODE, newMode);
    // 返回响应
    JsonObject responseDoc = response.to<JsonObject>();
    responseDoc["success"] = true;
    responseDoc["currentMode"] = newMode ? "auto" : "manual";
}

void processSetLedState(const JsonVariantConst& data, JsonDocument& response)
{
    bool newState = data.as<bool>();
    state.ledState = newState;
    digitalWrite(Pins::LAMP_RELAY, newState ? HIGH : LOW);

    JsonObject responseDoc = response.to<JsonObject>();
    responseDoc["success"] = true;
    responseDoc["newState"] = newState;
    response.set(responseDoc);
    Serial.printf("LED state set to %s\n", newState ? "ON" : "OFF");
}

void processSetFanState(const JsonVariantConst& data, JsonDocument& response)
{
    bool newState = data.as<bool>();
    state.fanState = newState;
    digitalWrite(Pins::FAN_RELAY, newState ? HIGH : LOW);

    JsonObject responseDoc = response.to<JsonObject>();
    responseDoc["success"] = true;
    responseDoc["newState"] = newState;
    response.set(responseDoc);
}

void processSetCurtainState(const JsonVariantConst& data, JsonDocument& response)
{
    bool newState = data.as<bool>();
    state.curtainOpen = newState;

    JsonObject responseDoc = response.to<JsonObject>();
    responseDoc["success"] = true;
    responseDoc["newState"] = newState;
    response.set(responseDoc);
}

void processSetDoorState(const JsonVariantConst& data, JsonDocument& response)
{
    bool newState = data.as<bool>();
    state.doorOpen = newState;

    JsonObject responseDoc = response.to<JsonObject>();
    responseDoc["success"] = true;
    responseDoc["newState"] = newState;
    response.set(responseDoc);
}

// RPC回调列表 (添加到RPC回调函数后)
const std::array<RPC_Callback, 5U> tbCallbacks = {
    RPC_Callback{"setAutoMode", processSetAutoMode},
    RPC_Callback{"setLedState", processSetLedState},
    RPC_Callback{"setFanState", processSetFanState},
    RPC_Callback{"setCurtainState", processSetCurtainState},
    RPC_Callback{"setDoorState", processSetDoorState}
};

// ThingsBoard连接和数据发送函数
void connectThingsBoard()
{
    if (!tb.connected())
    {
        Serial.print("Connecting to ThingsBoard server: ");
        Serial.println(ThingsBoardConfig::SERVER);

        if (tb.connect(ThingsBoardConfig::SERVER, ThingsBoardConfig::TOKEN, ThingsBoardConfig::PORT))
        {
            Serial.println("Connected to ThingsBoard!");

            // 订阅RPC命令
            if (!rpc.RPC_Subscribe(tbCallbacks.cbegin(), tbCallbacks.cend()))
            {
                Serial.println("Failed to subscribe for RPC");
            }
            else
            {
                Serial.println("Successfully subscribed for RPC");
                for (const auto& callback : tbCallbacks)
                {
                    Serial.printf("RPC callback for %s registered\n", callback.Get_Name());
                }
            }

            // 发送设备信息
            tb.sendAttributeData(AttrNames::DEVICE_NAME, "SmartLivingRoom");
            tb.sendAttributeData(AttrNames::MAC_ADDRESS, WiFi.macAddress().c_str());
            tb.sendAttributeData(AttrNames::SYSTEM_MODE, state.autoMode);
            Serial.printf("Initial system mode: %s, Synced to the server\n", state.autoMode ? "Auto" : "Manual");
        }
        else
        {
            Serial.print("Failed to connect to ThingsBoard");
        }
    }
}

void sendTelemetryData()
{
    if (tb.connected() && millis() - previousTbSend > ThingsBoardConfig::TELEMETRY_INTERVAL)
    {
        previousTbSend = millis();

        // 发送传感器数据
        tb.sendTelemetryData(AttrNames::TEMP, state.temperature);
        tb.sendTelemetryData(AttrNames::HUMIDITY, state.humidity);
        tb.sendTelemetryData(AttrNames::LUX, state.lux);
        tb.sendTelemetryData(AttrNames::BODY, state.bodyDetected);
        tb.sendTelemetryData(AttrNames::SMOKE, state.smokeDetected);
        tb.sendTelemetryData(AttrNames::FLAME, state.flameDetected);
        tb.sendTelemetryData(AttrNames::RAIN, state.rainDetected);
        tb.sendTelemetryData(AttrNames::SOUND, state.soundDetected);

        // 发送执行器状态
        tb.sendTelemetryData(AttrNames::LED, state.ledState);
        tb.sendTelemetryData(AttrNames::FAN, state.fanState);
        tb.sendTelemetryData(AttrNames::CURTAIN, state.curtainOpen);
        tb.sendTelemetryData(AttrNames::DOOR, state.doorOpen);
        tb.sendTelemetryData(AttrNames::ALARM, state.alarmActive);

        // 发送属性数据
        tb.sendAttributeData(AttrNames::SYSTEM_MODE, state.autoMode);
        tb.sendAttributeData(AttrNames::LED, state.ledState);
        tb.sendAttributeData(AttrNames::FAN, state.fanState);
        tb.sendAttributeData(AttrNames::CURTAIN, state.curtainOpen);
        tb.sendAttributeData(AttrNames::DOOR, state.doorOpen);

        tb.sendAttributeData(AttrNames::RSSI, WiFi.RSSI());
        tb.sendAttributeData(AttrNames::CHANNEL, WiFi.channel());
        tb.sendAttributeData(AttrNames::BSSID, WiFi.BSSIDstr().c_str());
        tb.sendAttributeData(AttrNames::LOCALIP, WiFi.localIP().toString().c_str());
        tb.sendAttributeData(AttrNames::SSID, WiFi.SSID().c_str());
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
    checkWiFiConnection();

    // 新增ThingsBoard相关操作
    if (state.wifiConnected)
    {
        connectThingsBoard();
        checkModeChanged();
        tb.loop();
        sendTelemetryData();
    }

    updateDisplay();
    printDebugInfo();
}
