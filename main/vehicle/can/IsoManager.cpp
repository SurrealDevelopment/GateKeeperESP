//
// Created by Justin Hoogestraat on 10/29/18.
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
#include "IsoManager.h"
#include "IsoFragment.h"

void IsoManager::onCANMessager(CanMessage message) {

}

IsoManager::IsoManager(ICAN *CAN) {
    this->mIcan = CAN;
}

void IsoManager::setICAN(ICAN *CAN) {
    this->mIcan = CAN;
}
