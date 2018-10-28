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
 /**
 * Contains variosu fdom objects for cross comm
 */


#ifndef _FDOM_H_
#define _FDOM_H_

#include "stdlib.h"

// forward dec
#include "gattserver.h"
#include "rgbcontrol.h"




class Fdom {
     private:          

    public:
        Fdom() {}           

        RgbControl * rgb1;

        GattServer * gatt;

};




#endif
