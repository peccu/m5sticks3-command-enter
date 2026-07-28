#include <M5Unified.h>
// NimBLE-Arduino — install via Arduino Library Manager
#include <NimBLEDevice.h>

// Step 3: press Button A to send Command+Enter via BLE HID

// Keyboard HID report descriptor
//   Report ID 1, 8-byte input: [modifier][reserved][key x6]
//   Report ID 1, 1-byte output: [LED bits x5 | padding x3]
static const uint8_t hidReportMap[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0x01,  // Report ID (1)
    // Modifier keys — 8 bits (Left/Right Ctrl/Shift/Alt/GUI)
    0x05, 0x07,  0x19, 0xE0,  0x29, 0xE7,
    0x15, 0x00,  0x25, 0x01,  0x75, 0x01,
    0x95, 0x08,  0x81, 0x02,
    // Reserved byte
    0x95, 0x01,  0x75, 0x08,  0x81, 0x01,
    // LED output — 5 LED bits (Num/Caps/Scroll/Compose/Kana) + 3 padding
    0x05, 0x08,  0x19, 0x01,  0x29, 0x05,
    0x95, 0x05,  0x75, 0x01,  0x91, 0x02,
    0x95, 0x01,  0x75, 0x03,  0x91, 0x01,
    // Key array — 6 simultaneous keys
    0x95, 0x06,  0x75, 0x08,
    0x15, 0x00,  0x25, 0x65,
    0x05, 0x07,  0x19, 0x00,  0x29, 0x65,
    0x81, 0x00,
    0xC0         // End Collection
};

// HID modifier bits
#define MOD_LEFT_GUI  0x08  // Command on macOS

// HID key codes
#define KEY_ENTER     0x28

static NimBLECharacteristic* inputReport = nullptr;
static bool bleConnected = false;

class BLECallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo& connInfo) override {
        bleConnected = true;
        NimBLEDevice::startSecurity(connInfo.getConnHandle());
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        bleConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

void setupBLE() {
    NimBLEDevice::init("Claude Keyboard");
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());

    // Device Information Service (0x180A)
    NimBLEService* dis = server->createService("180A");
    dis->createCharacteristic("2A29", NIMBLE_PROPERTY::READ)->setValue("M5Stack");
    dis->createCharacteristic("2A24", NIMBLE_PROPERTY::READ)->setValue("M5StickS3");
    uint8_t pnpId[] = {0x01, 0xE5, 0x02, 0x00, 0x00, 0x01, 0x00};
    dis->createCharacteristic("2A50", NIMBLE_PROPERTY::READ)->setValue(pnpId, sizeof(pnpId));
    dis->start();

    // HID Service (0x1812)
    NimBLEService* hid = server->createService("1812");

    uint8_t mode = 1;
    hid->createCharacteristic("2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR)
       ->setValue(&mode, 1);

    hid->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC)
       ->setValue(hidReportMap, sizeof(hidReportMap));

    inputReport = hid->createCharacteristic(
        "2A4D",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC
    );
    uint8_t inRefVal[] = {0x01, 0x01};
    inputReport->createDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC, 2)
               ->setValue(inRefVal, 2);

    NimBLECharacteristic* outputRpt = hid->createCharacteristic(
        "2A4D",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC
    );
    uint8_t outRefVal[] = {0x01, 0x02};
    outputRpt->createDescriptor("2908", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC, 2)
             ->setValue(outRefVal, 2);

    uint8_t info[] = {0x11, 0x01, 0x00, 0x03};
    hid->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ)->setValue(info, sizeof(info));
    hid->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);
    hid->start();

    // Battery Service (0x180F)
    NimBLEService* batt = server->createService("180F");
    uint8_t lvl = 100;
    batt->createCharacteristic(
        "2A19",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ_ENC)
        ->setValue(&lvl, 1);
    batt->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName("Claude Keyboard");
    adv->addServiceUUID("1812");
    adv->setAppearance(0x03C1);
    adv->start();
}

// Send a key press and release. report is 8 bytes: [modifier][reserved][key x6]
void sendKey(uint8_t modifier, uint8_t keycode) {
    if (!bleConnected || inputReport == nullptr) return;

    uint8_t down[8] = {modifier, 0x00, keycode, 0x00, 0x00, 0x00, 0x00, 0x00};
    inputReport->setValue(down, sizeof(down));
    inputReport->notify();

    delay(10);

    uint8_t up[8] = {};
    inputReport->setValue(up, sizeof(up));
    inputReport->notify();
}

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

void drawAction(const char* label) {
    M5.Display.fillRect(0, 95, M5.Display.width(), 15, BLACK);
    M5.Display.setCursor(10, 95);
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.println(label);
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
    M5.Display.println("v0.3.0");

    drawStatus(false);
    setupBLE();
}

bool lastConnected = false;

void loop() {
    M5.update();

    if (bleConnected != lastConnected) {
        lastConnected = bleConnected;
        drawStatus(bleConnected);
        drawAction("");
    }

    if (M5.BtnA.wasPressed()) {
        if (bleConnected) {
            sendKey(MOD_LEFT_GUI, KEY_ENTER); // Command+Enter
            drawAction("Cmd+Enter sent");
            delay(500);
            drawAction("");
        } else {
            drawAction("Not connected");
            delay(500);
            drawAction("");
        }
    }

    delay(10);
}
