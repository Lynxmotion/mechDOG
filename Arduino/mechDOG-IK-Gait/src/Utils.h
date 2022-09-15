/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Description:	Library with functions to synchronize motion control.
 */

#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"

class DTime
{
    public:
        DTime(int16_t dt = 0);
        void updateDT(int16_t dt);
        bool getDT(bool debug = false);
        void reset(void);
        int16_t dt = 0;
        
    private:
        int32_t new_sample = 0;
        int32_t old_sample = 0;
        
};

#endif