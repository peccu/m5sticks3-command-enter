#include <M5Unified.h>
// https://github.com/T-vK/ESP32-BLE-Keyboard (ZIP install via Arduino IDE)
#include <BleKeyboard.h>

// Step 2: BLE HID init + connection status display
// Button handling will be added in Step 3

BleKeyboard bleKeyboard("Claude Keyboard", "M5StickS3", 100);

void drawStatus(bool connected) {
    M5.Display.fillRect(0, 70, M5.Display.width(), 20, BLACK);
    M5.Display.setCursor(10, 70);
    if (connected) {
        M5.Display.setTextColor(GREEN, BLACK);
        M5.Display.println("Connected!");
    } else {
        M5.Display.setTextColor(YELLOW, BLACK);
        M5.Display.println("Waiting...");
    }
    M5.Display.setTextColor(WHITE, BLACK);
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 20);
    M5.Display.println("BT Keyboard");

    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 55);
    M5.Display.println("v0.2.0");

    drawStatus(false);
    bleKeyboard.begin();
}

bool lastConnected = false;

void loop() {
    M5.update();

    bool connected = bleKeyboard.isConnected();
    if (connected != lastConnected) {
        lastConnected = connected;
        drawStatus(connected);
    }

    delay(100);
}
