#include <esp_now.h>
#include <WiFi.h>

#define CW_PIN 5   // Pin to activate for CW
#define CCW_PIN 12 // Pin to activate for CCW

typedef struct test_struct {
  int rotationDirection;  // 1 for CW, 2 for CCW, 0 for no rotation
  bool buttonEvent;       // True if button is pressed
} test_struct;

test_struct receivedData;

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  Serial.println("Data received callback triggered.");
  Serial.print("Data length: ");
  Serial.println(len);

  if (len == sizeof(receivedData)) {

    memcpy(&receivedData, data, sizeof(receivedData));

    Serial.print("Rotation Direction: ");
    Serial.println(receivedData.rotationDirection);

    Serial.print("Button Event: ");
    Serial.println(receivedData.buttonEvent);

    if (receivedData.rotationDirection == 1) {

      Serial.println("Activating CW_PIN");
      digitalWrite(CW_PIN, HIGH);
      delay(500);  // Keep the pin active for 500ms
      digitalWrite(CW_PIN, LOW);

    } else if (receivedData.rotationDirection == 2) {

      Serial.println("Activating CCW_PIN");
      digitalWrite(CCW_PIN, HIGH);
      delay(500);  // Keep the pin active for 500ms
      digitalWrite(CCW_PIN, LOW);

    } else {

      Serial.println("No rotation detected.");

    }
  } else {

    Serial.println("Invalid data length received!");

  }
}

void setup() {
  
  Serial.begin(115200);

  Serial.println("Setting up pins...");

  pinMode(CW_PIN, OUTPUT);
  pinMode(CCW_PIN, OUTPUT);

  digitalWrite(CW_PIN, LOW);
  digitalWrite(CCW_PIN, LOW);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("ESP-NOW initialized.");

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Receiver setup complete, waiting for data...");
}

void loop() {
  // The receiver runs in the background and triggers OnDataRecv when data is received
}
