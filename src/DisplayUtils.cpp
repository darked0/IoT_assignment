#include "DisplayUtils.h"
#include "Globals.h"

void initDisplay() {
  // Start display power (Vext LOW)
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW);
  delay(50); // stabilization

  // Button reset for OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  // Initialize I2C
  Wire.begin(OLED_SDA, OLED_SCL);

  // Start display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Display Initialization Failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.display();
}

int autoCalibrateADCFrequency() {
  Serial.println("--- BOOT: Calibrating Hardware ---");
  Serial.println("Measuring max ADC frequency (1 sec)...");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("--- BOOT SEQUENCE ---");
  display.println("");
  display.println("Calibrating ADC...");
  display.println("Please wait 1 sec");
  display.display();

  uint32_t start_time = micros();
  uint32_t count = 0;

  // Execute reads for exactly 1.000.000 microseconds (1 second)
  while (micros() - start_time < 1000000) {
    volatile int val = analogRead(4); // 'volatile' force CPU to read phisically the pin
    count++;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("--- BOOT SEQUENCE ---");
  display.println("");
  display.print("Max ADC: ");
  display.print(count);
  display.println(" Hz");
  display.println("");
  display.println("Set RTOS to: 1000 Hz");
  display.println("Starting OS...");
  display.display();

  return count;
}
