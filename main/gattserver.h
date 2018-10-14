#ifndef _GATT_SERVER_H_
#define _GATT_SERVER_H_




#include <string>
#include "ble/BLEDevice.h"
#include "ble/BLE2902.h"
#include "esp_log.h"
#include "ble/BLESecurity.h"
#include "rgbcontrol.h"




#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class GattServer {

    private:
        RgbControl * rgb1;

    public:
        ~GattServer();

        GattServer(RgbControl * rgb, std::string name);
};


#endif

