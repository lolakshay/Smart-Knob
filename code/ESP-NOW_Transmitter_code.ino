#include <esp_now.h>
#include <WiFi.h>

#define ROTARY_CLK 15  // CLK pin of KY-040 connected to GPIO15
#define ROTARY_DT  4   // DT pin of KY-040 connected to GPIO4
#define ROTARY_SW  12  // SW pin of KY-040 connected to GPIO16 (button switch)
#define BUZZER_PIN 14  // D5 (GPIO 14) pin connected to the buzzer

// Replace with the receiver MAC address
uint8_t receiverMAC[] = {0x8C, 0x4F, 0x00, 0x12, 0x09, 0xE4};

typedef struct test_struct {
  int rotationDirection;  // 1 for CW, 2 for CCW, 0 for no rotation
  bool buttonEvent;       // True if button is pressed
} test_struct;

test_struct myData = {0, false};

// Variables for rotary encoder
int lastStateCLK;
int currentStateCLK;

// Variables for button debounce
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Rotary encoder pins as input
  pinMode(ROTARY_CLK, INPUT);
  pinMode(ROTARY_DT, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP);  // Pull-up for button switch

  // Buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Ensure the buzzer is off initially

  // Read the initial state of the rotary encoder
  lastStateCLK = digitalRead(ROTARY_CLK);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for send callback
  esp_now_register_send_cb(OnDataSent);

  // Add peer (receiver MAC address)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);  // Set receiver's MAC address
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Rotary Encoder Logic
  currentStateCLK = digitalRead(ROTARY_CLK);

  if (currentStateCLK != lastStateCLK) {
    // Check the direction of rotation
    if (digitalRead(ROTARY_DT) != currentStateCLK) {
      Serial.println("Rotated clockwise");
      myData.rotationDirection = 1;  // CW
    } else {
      Serial.println("Rotated counterclockwise");
      myData.rotationDirection = 2;  // CCW
    }

    // Send rotation direction
    myData.buttonEvent = false;  // Ensure button event is false
    esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

    // Activate buzzer for rotation event
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
    delay(100);                      // Keep buzzer on for 100ms
    digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer

    delay(50);  // Prevent duplicate signals
  }
  lastStateCLK = currentStateCLK;

  // Button Logic with Debounce
  bool currentButtonState = digitalRead(ROTARY_SW);
  if (currentButtonState != lastButtonState) {
    lastDebounceTime = millis();  // Reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      // Button is pressed
      Serial.println("Button pressed!");

      // Send button press event
      myData.rotationDirection = 0;  // No rotation
      myData.buttonEvent = true;
      esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));

      // Activate buzzer for button press event
      digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer
      delay(100);                      // Keep buzzer on for 100ms
      digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer
    }
  }
  lastButtonState = currentButtonState;

  // Reset rotation direction after sending
  myData.rotationDirection = 0;
}
