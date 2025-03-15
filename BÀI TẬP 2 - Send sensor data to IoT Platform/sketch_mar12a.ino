#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Wi-Fi configuration
const char *ssid = "Redmi K60 Ultra";  // SSID mạng Wi-Fi
const char *password = "asdfghjk";    // Mật khẩu Wi-Fi

// ThingSpeak configuration
unsigned long channelID = 2875470;     // Thay bằng Channel ID của bạn
const char *writeAPIKey = "DPKNW8FDVT9J0GTI"; // Thay bằng Write API Key của bạn
const char *readAPIKey = "8FKK46L7ZUQU79P1";  // Thay bằng Read API Key của bạn

LiquidCrystal_I2C lcd(0x27, 16, 2);   // Màn hình LCD địa chỉ 0x27
Adafruit_MLX90614 mlx;                // Đối tượng cảm biến MLX90614

WiFiClient client;
unsigned long lastTime = 0;
long timeout = 30000;  // Thời gian timeout cho ThingSpeak

SemaphoreHandle_t dataMutex;
struct SensorData {
  float temperature;
  float humidity;
  String dateTime;
} sensorData;

// NTP configuration
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 25200, 60000);  // Múi giờ Việt Nam (UTC+7) là 25200 giây

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Kết nối Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Khởi tạo cảm biến nhiệt độ MLX90614
  Wire.begin(21, 22);  // Các chân SDA và SCL của ESP32
  if (!mlx.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MLX90614 not found");
    while (1)
      ;
  }

  // Tạo semaphore để đồng bộ dữ liệu
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Failed to create mutex");
    while (1)
      ;
  }

  // Khởi tạo ThingSpeak
  ThingSpeak.begin(client);

  // Khởi tạo NTPClient
  timeClient.begin();
  timeClient.setTimeOffset(25200);  // Múi giờ Việt Nam UTC+7

  lcd.clear();
  Serial.println("Place your finger on the sensor.");

  // Tạo task cho đọc nhiệt độ và cập nhật LCD
  xTaskCreate(taskReadTemperature, "Read Temperature", 2048, NULL, 1, NULL);
  xTaskCreate(taskUpdateLCD, "Update LCD", 2048, NULL, 1, NULL);
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastTime >= timeout) {
    // Cập nhật thời gian từ NTP
    timeClient.update();
    String dateTime = timeClient.getFormattedTime();  // Lấy thời gian theo định dạng "HH:MM:SS"

    // Gửi dữ liệu lên ThingSpeak (nhiệt độ, độ ẩm và ngày giờ)
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      ThingSpeak.setField(1, sensorData.temperature);  // Gửi nhiệt độ vào Field 1
      ThingSpeak.setField(2, sensorData.humidity);    // Gửi độ ẩm vào Field 2
      ThingSpeak.setField(3, dateTime);               // Gửi ngày giờ vào Field 3

      int httpCode = ThingSpeak.writeFields(channelID, writeAPIKey);

      // Hiển thị HTTP response code
      if (httpCode == 200) {
        Serial.println("Data sent successfully!");
        Serial.println("HTTP Response: OK");
      } else {
        Serial.print("Error: ");
        Serial.println(httpCode);
      }

      xSemaphoreGive(dataMutex);
    }
    lastTime = currentMillis;
  }

  // Nhận dữ liệu từ ThingSpeak (độ ẩm và ngày giờ)
  if (WiFi.status() == WL_CONNECTED) {
    // Đọc độ ẩm từ ThingSpeak Field 2
    float humidity = ThingSpeak.readFloatField(channelID, 2, readAPIKey);
    if (humidity != NAN) {
      sensorData.humidity = humidity; // Lưu độ ẩm vào biến
    }
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Thời gian chờ 1 giây
}

void taskReadTemperature(void *pvParameters) {
  while (1) {
    float temperature = mlx.readObjectTempC();
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      sensorData.temperature = temperature;  // Cập nhật nhiệt độ
      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS);  // Đọc nhiệt độ mỗi 30 giây
  }
}

void taskUpdateLCD(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      // Hiển thị nhiệt độ, độ ẩm và ngày giờ trên LCD
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(sensorData.temperature, 1);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(sensorData.humidity, 1);
      lcd.print("");

      // Hiển thị ngày giờ
      lcd.setCursor(0, 1);
      lcd.print("Time: ");
      lcd.print(timeClient.getFormattedTime());  // Hiển thị thời gian từ NTP

      xSemaphoreGive(dataMutex);
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS);  // Cập nhật LCD mỗi 30 giây
  }
}
