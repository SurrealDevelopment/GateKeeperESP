//
// Created by Justin Hoogestraat on 10/31/18.
//

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
#ifndef FDOMESP_ICANLISTENER_H
#define FDOMESP_ICANLISTENER_H

#include "CanMessage.h"

/**
 * Listener object for CAN.
 */
class ICANListener {

    virtual void onCANMessager(CanMessage message);


};

#endif //FDOMESP_ICANLISTENER_H
