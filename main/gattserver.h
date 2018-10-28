/**
 *  Copyright (C) 2018 Surreal Development LLC
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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

