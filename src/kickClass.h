//
//  kickClass.h
//  receiveSensors
//
//  Created by Janis on 07/02/2018.
//
//

#ifndef __receiveSensors__kickClass__
#define __receiveSensors__kickClass__

//#include <stdio.h>
#include "ofMain.h"
#include "features.h"

class kickClass{
public:
    void setup();
    void update(double ElapsedTime_, float a_x_, float a_y_, float a_z_);
    void kick();
    
    bool send_kick;
    double ElapsedTime;
    float a_x, a_y, a_z;
    
    // For the object "kick"
    float acc_last_three[3][3] = {{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}};
    float acc_intensity_last_two[3][2] = {{0., 0.}, {0., 0.}, {0., 0.}};
    float acc_intensity_x, acc_intensity_y, acc_intensity_z;
    float acc_intensity_norm;
    float kick_intensity;// = 0.0f;
    double LastKick; //Time of the last sample of the last kick in milliseconds
    bool isKicking;// = 0.0f;
    //Median computation over N running points (N odd):
    //- Insert current value at the location of the value to be ejected, using a fifo
    //- Move it to the right place & update the indices concurrently using a layer to establish the link with fifo indices
    //- End of loop: median array is sorted, median value is median_values[N/2]
    float median_values[KICK_MEDIAN_FILTERSIZE] = {0., 0., 0., 0., 0., 0., 0., 0., 0.}; //sorted array of the 9 latest intensity values
    int median_linking[KICK_MEDIAN_FILTERSIZE] = {3, 4, 1, 5, 7, 8, 0, 2, 6}; //linking layer
    int median_fifo[KICK_MEDIAN_FILTERSIZE] = {6, 2, 7, 0, 1, 3, 8, 4, 5}; //fifo buffer for the corresponding indices, to update every time an element is moved in the median array
    //The initialization  of median_linking and median_fifo must comply with the following invariant property: median_fifo[median_linking[i]] == median_linking[median_fifo[i]] == i for all i in [0, MEDIAN_FILTER_SIZE-1]
    //
    int i1, i2, i3;
    float acc_intensity_norm_median;// = 0.f;
    unsigned short int LoopIndex;// = 0;
    unsigned short int LoopIndexPeriod;// = lcm(lcm(2, 3), KICK_MEDIAN_FILTERSIZE);
    
};

#endif /* defined(__receiveSensors__kickClass__) */
