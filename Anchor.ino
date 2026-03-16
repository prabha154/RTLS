#define BUTTON_PIN 26

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Enable internal pull-up resistor
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {   // Button pressed
    Serial.println("Button Pressed");
    delay(300);  // small debounce delay
  }
}