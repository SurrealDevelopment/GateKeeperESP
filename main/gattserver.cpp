#include "gattserver.h"
#include "esp_gap_ble_api.h"
#include "fdompin.h"


static char LOG_TAG[] = "BLE";
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t value = 0;



class MyServerCallbacks: public BLEServerCallbacks {

    RgbControl * rgb1;

    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        rgb1->set(8191, 8191, 3000, true, 1500, 1000, 500);

    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        rgb1->set(4000, 8191, 4000, true, 1500, 1000, 500);


    }
public:
    MyServerCallbacks(RgbControl * rgb)
    {
        this->rgb1 = rgb;
    }
};


class SecurityCallbacks : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest(){
        ESP_LOGI(LOG_TAG, "PassKeyRequest");
        return 123456;
    }
    void onPassKeyNotify(uint32_t pass_key){
        ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", pass_key);
    }
    bool onConfirmPIN(uint32_t pass_key){
        ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
        vTaskDelay(5000);
        return true;
    }
    bool onSecurityRequest(){
        ESP_LOGI(LOG_TAG, "SecurityRequest");
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
        ESP_LOGI(LOG_TAG, "Starting BLE work!");
        if(cmpl.success){
            uint16_t length;
            esp_ble_gap_get_whitelist_size(&length);
            ESP_LOGD(LOG_TAG, "size: %d", length);
        }
    }
};


class bleCharacteristicCallback: public BLECharacteristicCallbacks {

    RgbControl * rgb1;
public:
    bleCharacteristicCallback(RgbControl * rgb)
    {
        this->rgb1 = rgb;

    }
    //Fdom * fdom = Fdom::getInstance();
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string msg = pCharacteristic->getValue();
        ESP_LOGI(LOG_TAG, "BLE received: %s", msg.c_str());
                esp_log_buffer_char(LOG_TAG, msg.c_str(), msg.length());
                esp_log_buffer_hex(LOG_TAG, msg.c_str(), msg.length());

        rgb1->set(8192,8192,0, true, 100, 75, 70);


    }

    void onRead(BLECharacteristic* pCharacteristic) {
        std::string msg = pCharacteristic->getValue();
        ESP_LOGI(LOG_TAG, "BLE received: %s, %i", msg.c_str(), msg.length());
                esp_log_buffer_char(LOG_TAG, msg.c_str(), msg.length());
                esp_log_buffer_hex(LOG_TAG, msg.c_str(), msg.length());
    }


};


GattServer::GattServer(RgbControl * rgb, std::string name)
{
    this->rgb1 = rgb;
    BLEDevice::init(name);

   // BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);


    //BLEDevice::setSecurityCallbacks(new SecurityCallbacks());



    ESP_LOGI(LOG_TAG, "Starting BLE Server with name %s", name.c_str());

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(rgb));


    BLEService* pService = pServer->createService("95580c9a-c6f0-47e6-a2af-f71b7d16131f");

    BLECharacteristic* pCharacteristic = pService->createCharacteristic(
            BLEUUID("0d563a58-196a-48ce-ace2-dfec78acc814"),
            BLECharacteristic::PROPERTY_READ  |
            BLECharacteristic::PROPERTY_NOTIFY    |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_INDICATE
    );
    pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    pCharacteristic->setCallbacks(new bleCharacteristicCallback(rgb1));
    pCharacteristic->setValue("Hello World!");

    BLE2902* p2902Descriptor = new BLE2902();
    p2902Descriptor->setNotifications(true);
    pCharacteristic->addDescriptor(p2902Descriptor);
    p2902Descriptor->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);

    pService->start();

    BLEAdvertising* pAdvertising = pServer->getAdvertising();

    pAdvertising->addServiceUUID(BLEUUID(pService->getUUID()));

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setKeySize();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    //pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    pAdvertising->start();

    ESP_LOGD(LOG_TAG, "Advertising started!");








}