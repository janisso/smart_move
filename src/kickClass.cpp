//
//  kickClass.cpp
//  receiveSensors
//
//  Created by Janis on 07/02/2018.
//
//

#include "kickClass.h"

void kickClass::setup(){
    kick_intensity = 0.0f;
    isKicking = false;
    acc_intensity_norm_median = 0.f;
    LoopIndex = 0;
    LoopIndexPeriod = lcm(lcm(2, 3), KICK_MEDIAN_FILTERSIZE);
    a_x = 0;    a_y = 0;    a_z = 0;
    send_kick = false;
}

void kickClass::kick(){
    
}

void kickClass::update(double ElapsedTime_, float a_x_, float a_y_, float a_z_){
    ElapsedTime = ElapsedTime_;
    a_x = a_x_;
    a_y = a_y_;
    a_z = a_z_;
    
    //KICK
    acc_last_three[0][LoopIndex % 3] = a_x;
    acc_last_three[1][LoopIndex % 3] = a_y;
    acc_last_three[2][LoopIndex % 3] = a_z;
    
    acc_intensity_x = intensity1D(a_x, acc_last_three[0][(LoopIndex+1) % 3], acc_intensity_last_two[0][(LoopIndex+1) % 2], ACC_INTENSITY_PARAM1, ACC_INTENSITY_PARAM2, 1.0f);
    acc_intensity_last_two[0][LoopIndex % 2] = acc_intensity_x;
    acc_intensity_y = intensity1D(a_y, acc_last_three[1][(LoopIndex+1) % 3], acc_intensity_last_two[1][(LoopIndex+1) % 2], ACC_INTENSITY_PARAM1, ACC_INTENSITY_PARAM2, 1.0f);
    acc_intensity_last_two[1][LoopIndex % 2] = acc_intensity_y;
    acc_intensity_z = intensity1D(a_z, acc_last_three[2][(LoopIndex+1) % 3], acc_intensity_last_two[2][(LoopIndex+1) % 2], ACC_INTENSITY_PARAM1, ACC_INTENSITY_PARAM2, 1.0f);
    acc_intensity_last_two[2][LoopIndex % 2] = acc_intensity_z;
    
    acc_intensity_norm = acc_intensity_x + acc_intensity_y + acc_intensity_z;
    
    //*** Median filter implementation ***
    //The median over the N last samples is computed by inserting the next sample value into an array (median_values) which contains the N last elements already sorted
    //The current median is the value in the middle of the array after this insertion
    i3 = LoopIndex % KICK_MEDIAN_FILTERSIZE; //offset for using the array median_fifo as a circular buffer of size N
    i1 = median_fifo[i3]; //where to insert the new value
    i2 = 1; //index to add/substract to i1
    //Check first to the right
    if(i1 < KICK_MEDIAN_FILTERSIZE-1 && acc_intensity_norm > median_values[i1+i2]){
        //Serial.println("right");
        while (i1+i2 < KICK_MEDIAN_FILTERSIZE && acc_intensity_norm > median_values[i1+i2]){  //Move median_values[i1+i2] to median_values[i1+i2-1]
            //Serial.println(i1+i2);
            median_fifo[median_linking[i1+i2]] = median_fifo[median_linking[i1+i2]] - 1;
            median_values[i1+i2-1] = median_values[i1+i2];
            median_linking[i1+i2-1] = median_linking[i1+i2];
            i2++;
        }
        median_values[i1+i2-1] = acc_intensity_norm;
        median_linking[i1+i2-1] = i3;
        median_fifo[i3] = i1+i2-1;
    }
    else {
        //Check to the left
        //Serial.println("left");
        while (i2 < i1+1 && acc_intensity_norm < median_values[i1-i2]){ //second operand is not evaluated is first operand is false
            //Serial.println(i1-i2);
            median_fifo[median_linking[i1-i2]] = median_fifo[median_linking[i1-i2]] + 1;
            median_values[i1-i2+1] = median_values[i1-i2];
            median_linking[i1-i2+1] = median_linking[i1-i2];
            i2++;
        }
        median_values[i1-i2+1] = acc_intensity_norm;
        median_linking[i1-i2+1] = i3;
        median_fifo[i3] = i1-i2+1;
    }
    
    // The current intensity norm is compared with the previous median value
    if (acc_intensity_norm - acc_intensity_norm_median > KICK_THRESHOLD){
        if (isKicking == true){ //Still kicking
            if (kick_intensity < acc_intensity_norm){
                kick_intensity = acc_intensity_norm;
            }
        }
        else {//New kick detected
            isKicking = true;
            send_kick = true;
            //cout << "kick " << ElapsedTime << endl;
            kick_intensity = acc_intensity_norm;
            cout << "kick " << ElapsedTime << endl;
            
            //
            //kick_intensity = 0.;
            //Update last kick time only at kick onset
            LastKick = ElapsedTime;
        }
    }
    else {
        //Do we really want this? No: update kick intensity only within a kick event
        /*if (kick_intensity < acc_intensity_norm){
         kick_intensity = acc_intensity_norm;
         }*/
        //Set isKicking to 0 only after the speedgate duration
        if (ElapsedTime - LastKick > KICK_SPEEDGATE){
            isKicking = false;
        }
    }
    
    acc_intensity_norm_median = median_values[KICK_MEDIAN_FILTERSIZE/2]; //KICK_MEDIAN_FILTERSIZE should be odd
    
    LoopIndex = (LoopIndex + 1) % LoopIndexPeriod;
}
