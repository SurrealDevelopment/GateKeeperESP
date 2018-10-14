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
