#include <Arduino.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHT_TYPE DHT11
#define DHT_PIN 4

// 初始化OLED：软件I2C，SCL和SDA引脚需在实际接线时定义（如#define SCL 22, #define SDA 21）
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);
// 初始化DHT11
DHT_Unified dht(DHT_PIN, DHT_TYPE);

void initOLED();
void initDHT();
void getTempHumShow();

void setup()
{
    // write your initialization code here
    initOLED();
    initDHT();
}

void loop()
{
    // write your code here
    getTempHumShow();
    delay(1000);
}

void initOLED()
{
    // 初始化OLED
    u8g2.begin();
    // 启用UTF-8打印
    u8g2.enableUTF8Print();
    // 设置中文字体
    u8g2.setFont(u8g2_font_ncenB12_tr);
}

void initDHT()
{
    // 初始化DHT11
    dht.begin();
}

void getTempHumShow()
{
    // 获取温度和湿度
    float temperature;
    float humidity;
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
    u8g2.firstPage();
    do
    {
        // 显示温度和湿度
        char buf[20];
        sprintf(buf, "Temp:%.1f", temperature);
        u8g2.drawStr(0, 12, buf);
        sprintf(buf, "Hum：%.1f", humidity);
        u8g2.drawStr(0, 32, buf);
    }
    while (u8g2.nextPage());
}
