#include <M5Unified.h>

// Step 1: scaffold only — boots, shows splash screen, does nothing else.
// BLE keyboard functionality will be added in a later step.

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 20);
    M5.Display.println("BT Keyboard");

    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 55);
    M5.Display.println("v0.1.0");
    M5.Display.setCursor(10, 70);
    M5.Display.println("Step 1: scaffold");
}

void loop() {
    M5.update();
    delay(10);
}
