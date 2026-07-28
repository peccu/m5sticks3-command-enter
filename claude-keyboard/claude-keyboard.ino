#include <M5Unified.h>
// NimBLE-Arduino — install via Arduino Library Manager
#include <NimBLEDevice.h>

// Step 2: BLE HID keyboard — advertises and accepts pairing, shows status
// Key sending will be added in Step 3

// Standard keyboard HID report descriptor (8-byte report, no Report ID)
static const uint8_t hidReportMap[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x06,  // Usage (Keyboard)
    0xA1, 0x01,  // Collection (Application)
    // Modifier keys — 8 bits (Ctrl/Shift/Alt/GUI left+right)
    0x05, 0x07,  0x19, 0xE0,  0x29, 0xE7,
    0x15, 0x00,  0x25, 0x01,  0x75, 0x01,
    0x95, 0x08,  0x81, 0x02,
    // Reserved byte
    0x95, 0x01,  0x75, 0x08,  0x81, 0x01,
    // Key array — 6 slots
    0x95, 0x06,  0x75, 0x08,
    0x15, 0x00,  0x25, 0x65,
    0x05, 0x07,  0x19, 0x00,  0x29, 0x65,
    0x81, 0x00,
    0xC0         // End Collection
};

static NimBLECharacteristic* inputReport = nullptr;
static bool bleConnected = false;

class BLECallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        bleConnected = true;
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        bleConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

void setupBLE() {
    NimBLEDevice::init("Claude Keyboard");
    NimBLEDevice::setSecurityAuth(true, false, true); // bonding, no MITM, SC

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());

    // HID Service (0x1812)
    NimBLEService* hid = server->createService("1812");

    // Protocol Mode: Report Protocol (1)
    uint8_t mode = 1;
    hid->createCharacteristic("2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR)
       ->setValue(&mode, 1);

    // Report Map
    hid->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ)
       ->setValue(hidReportMap, sizeof(hidReportMap));

    // Input Report
    inputReport = hid->createCharacteristic(
        "2A4D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t refVal[] = {0x00, 0x01}; // Report ID 0, Input type
    inputReport->createDescriptor("2908", NIMBLE_PROPERTY::READ, 2)
               ->setValue(refVal, 2);

    // HID Information: HID 1.11, not localized, remote wake + normally connectable
    uint8_t info[] = {0x11, 0x01, 0x00, 0x03};
    hid->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ)
       ->setValue(info, sizeof(info));

    // HID Control Point (write only, required by spec)
    hid->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);

    hid->start();

    // Battery Service — required by many BLE HID hosts
    NimBLEService* batt = server->createService("180F");
    uint8_t lvl = 100;
    batt->createCharacteristic("2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY)
        ->setValue(&lvl, 1);
    batt->start();

    // Advertise as a BLE HID keyboard
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID("1812");
    adv->setAppearance(0x03C1); // Keyboard
    adv->start();
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
    setupBLE();
}

bool lastConnected = false;

void loop() {
    M5.update();

    if (bleConnected != lastConnected) {
        lastConnected = bleConnected;
        drawStatus(bleConnected);
    }

    delay(100);
}
