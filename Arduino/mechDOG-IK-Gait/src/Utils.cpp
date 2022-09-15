/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Description:	Library with functions to synchronize motion control.
 */

#include "Utils.h"

DTime::DTime(int16_t dt = 0){
    this->old_sample = millis();
    this->dt = dt;

}

void DTime::reset(void){
    this->old_sample = millis();
}   

void DTime::updateDT(int16_t dt){
    if(dt > 0) this->dt = dt;
}      

bool DTime::getDT(bool debug = false){ 
    this->new_sample = millis();
    int16_t dt = this->new_sample - this->old_sample;

    if (debug){
        Serial.print("Time Error: ");
        Serial.println(dt-this->dt);
        Serial.print("Time elapsed: ");
        Serial.println(dt);
    }       

    if (dt-this->dt >= 0){
        this->old_sample = this->new_sample;
        return true;
    }else{
        return false;
    }
}
       