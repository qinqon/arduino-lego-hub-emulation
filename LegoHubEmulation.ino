#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>
#include <NimBLEScan.h>
#include <NimBLEUtils.h>

#include "motors.h"


// LWP3 Service UUID (00001623-1212-EFDE-1623-785FEABCD123)
// This is typically transmitted in little-endian order in BLE advertisements.
// So, it should appear as 2316-DEEF-1212-2316-2378-BCDA-23D1-5F (reversed bytes in groups)
// Or more simply, as an array of bytes as used by Pybricks (big-endian but reversed for comparison)
// Pybricks' pbio_lwp3_hub_service_uuid:
// 0x00, 0x00, 0x16, 0x23, 0x12, 0x12, 0xEF, 0xDE, 0x16, 0x23, 0x78, 0x5F, 0xEA, 0xBC, 0xD1, 0x23
// The NimBLE library typically handles UUID parsing from advertisements, so we'll provide the string representation.
static NimBLEUUID lwp3ServiceUUID("00001623-1212-EFDE-1623-785FEABCD123");

// LWP3 Characteristic UUID (00001624-1212-EFDE-1623-785FEABCD123)
static NimBLEUUID lwp3CharUUID("00001624-1212-EFDE-1623-785FEABCD123");

// LEGO Company ID for Manufacturer Specific Data
const uint16_t LEGO_COMPANY_ID = 0x0397; // Little Endian: 0x9703
// LWP3 Hub Kind for Handset/Remote
const uint8_t LWP3_HUB_KIND_HANDSET = 0x42;

// Connected Remote
static NimBLERemoteCharacteristic* pLwp3Characteristic;
static NimBLEClient* pClient = nullptr;
static NimBLEAdvertisedDevice* pDevice = nullptr;
bool deviceConnected = false;
bool doConnect = false;

uint8_t speed = 70;
uint8_t max_speed = 100;
uint8_t min_speed = 5;
uint8_t speed_interval = 10;

// Scan for BLE servers and connect to the first matching one
class AdvertisedDeviceCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        // We only care about devices advertising the LWP3 service UUID
        if (!advertisedDevice->isAdvertisingService(lwp3ServiceUUID)) {
            return;
        }

        // Check for Manufacturer Data (0xFF type)
        std::string manufacturerData = advertisedDevice->getManufacturerData();
        // Manufacturer data structure from Pybricks analysis:
        // Byte 0-1: Company ID (0x97, 0x03)
        // Byte 2: Button State / System capability
        // Byte 3: Hub Kind (0x42 for Handset)
        if (manufacturerData.length() >= 4) {
            uint16_t companyId = (manufacturerData[0] & 0xFF) | ((manufacturerData[1] & 0xFF) << 8); // Little Endian
            if (companyId == LEGO_COMPANY_ID) {
                if (manufacturerData[3] == LWP3_HUB_KIND_HANDSET) {
                    Serial.printf("Found Powered Up Remote: %s \n", advertisedDevice->toString().c_str());

                    // Stop scanning and save the device to connect in the loop
                    NimBLEDevice::getScan()->stop();
                    if (pDevice != nullptr) {
                        delete pDevice;
                    }
                    pDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
                    doConnect = true;
                }
            }
        }
    }
};

void decreaseSpeed() {
  if (speed - speed_interval > min_speed) {
    speed -= speed_interval;
  } else {
    speed = min_speed;
  }
}

void increasSpeed() {
  if (speed + speed_interval < max_speed) {
    speed += speed_interval;
  } else {
    speed = max_speed;
  }
}

// Callback for characteristic notifications
void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (isNotify) {
        // Handle button presses (LWP3 Port Value messages)
        if (length >= 4 && pData[0] == 0x07 && pData[2] == 0x45) {
            if (pData[3] == 0x00) { // Left Buttons -> Maps to Port A on Train Hub
                Serial.printf("Left Buttons (Port A): %02X %02X %02X\n", pData[4], pData[5], pData[6]);
                if (pData[4] == 1) {
                    moveCar(UP, speed);
                } else if (pData[6] == 1) {
                    moveCar(DOWN, speed);
                } else {
                    moveCar(STOP, 0);
                }
                if (pData[5] == 1) {
                  increasSpeed();
                }
            } else if (pData[3] == 0x01) { // Right Buttons -> Maps to Port B on Train Hub
                Serial.printf("Right Buttons (Port B): %02X %02X %02X\n", pData[4], pData[5], pData[6]);
                if (pData[4] == 1) {
                    moveCar(LEFT, speed);
                } else if (pData[6] == 1) {
                    moveCar(RIGHT, speed);
                } else {
                    moveCar(STOP, 0);
                }
                if (pData[5] == 1) {
                  decreaseSpeed();
                }
            }
        }
    }
}

// Function to send LWP3 commands
bool sendLWP3Command(const uint8_t* command, size_t length) {
    if (pLwp3Characteristic && pLwp3Characteristic->canWrite()) {
        if (pLwp3Characteristic->writeValue(command, length, false)) { // false for no response needed
            return true;
        }
        else {
            Serial.println("Failed to send LWP3 command.");
            return false;
        }
    }
    Serial.println("LWP3 Characteristic not available for writing.");
    return false;
}

void setup() {

    mcpwm_init();

    Serial.begin(115200);
    Serial.println("Starting BLE Powered Up Hub Emulator...");

    NimBLEDevice::init(""); // Init BLE, no device name
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9dbm */

    // Create a BLE scan
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new AdvertisedDeviceCallbacks(), false);
    pScan->setActiveScan(true); // Active scan to get scan response data
    pScan->setInterval(48); // 30ms (48 * 0.625ms)
    pScan->setWindow(48); // 30ms (48 * 0.625ms)
    pScan->start(0, false); // Scan indefinitely, not blocking
}

void loop() {
    // Handle connection request from callback
    if (doConnect) {
        doConnect = false;

        pClient = NimBLEDevice::createClient();
        Serial.println("Connecting to remote...");

        // Try to connect
        if (pClient->connect(pDevice)) {
            Serial.println("Connected to remote.");
            deviceConnected = true;

            // Update connection parameters for stability
            // Min Interval: 24 (30ms), Max Interval: 40 (50ms), Latency: 0, Timeout: 200 (2s)
            pClient->updateConnParams(24, 40, 0, 200);
        } else {
            Serial.println("Failed to connect to remote.");
            NimBLEDevice::getScan()->start(0, false); // Restart scan
        }
    }

    if (deviceConnected && pClient && pClient->isConnected()) {
        if (pLwp3Characteristic == nullptr) {
            // Service and Characteristic Discovery
            NimBLERemoteService* pRemoteService = pClient->getService(lwp3ServiceUUID);
            if (!pRemoteService) {
                Serial.println("Failed to get LWP3 service, disconnecting.");
                pClient->disconnect();
                return;
            }

            pLwp3Characteristic = pRemoteService->getCharacteristic(lwp3CharUUID);
            if (!pLwp3Characteristic) {
                Serial.println("Failed to get LWP3 characteristic, disconnecting.");
                pClient->disconnect();
                return;
            }

            // Enable notifications
            if (pLwp3Characteristic->canNotify()) {
                if (!pLwp3Characteristic->subscribe(true, notifyCallback, false)) {
                    Serial.println("Failed to subscribe to notifications, disconnecting.");
                    pClient->disconnect();
                    return;
                }
                Serial.println("Subscribed to LWP3 notifications.");
            } else {
                Serial.println("LWP3 Characteristic does not support notifications!");
                pClient->disconnect();
                return;
            }

            // Perform the LWP3 handshake to enable button data and set LED
            Serial.println("Performing LWP3 handshake...");

            delay(100);

            // 1. Enable Left Button Notifications (Port 0, Mode KEYSD)
            const uint8_t leftButtonCmd[] = {0x0A, 0x00, 0x41, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x01};
            if (!sendLWP3Command(leftButtonCmd, sizeof(leftButtonCmd))) { Serial.println("Handshake failed at step 1"); }
            delay(100);

            // 2. Enable Right Button Notifications (Port 1, Mode KEYSD)
            const uint8_t rightButtonCmd[] = {0x0A, 0x00, 0x41, 0x01, 0x04, 0x01, 0x00, 0x00, 0x00, 0x01};
            if (!sendLWP3Command(rightButtonCmd, sizeof(rightButtonCmd))) { Serial.println("Handshake failed at step 2"); }
            delay(100);

            // 3. Set Status Light to RGB Mode (Port 52, Mode RGB_0)
            const uint8_t lightSetupCmd[] = {0x0A, 0x00, 0x41, 0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00};
            if (!sendLWP3Command(lightSetupCmd, sizeof(lightSetupCmd))) { Serial.println("Handshake failed at step 3"); }
            delay(100);

            // 4. Set Status Light Color to Green (Train Hub Standard)
            // R: 0, G: 255, B: 0
            const uint8_t lightColorCmd[] = {0x0A, 0x00, 0x81, 0x34, 0x00, 0x51, 0x01, 0x00, 0xFF, 0x00};
            if (!sendLWP3Command(lightColorCmd, sizeof(lightColorCmd))) { Serial.println("Handshake failed at step 4"); }
            delay(100);

            Serial.println("LWP3 handshake complete. Waiting for inputs...");
        }

        // Main loop idle
        delay(10);

    } else if (deviceConnected || (pClient && !pClient->isConnected())) {
        // Handle disconnection
        Serial.println("Disconnected from remote, restarting scan.");

        if (pClient) {
            NimBLEDevice::deleteClient(pClient);
            pClient = nullptr;
        }

        deviceConnected = false;
        doConnect = false;
        pLwp3Characteristic = nullptr;

        NimBLEDevice::getScan()->start(0, false); // Restart scan
    }
}
