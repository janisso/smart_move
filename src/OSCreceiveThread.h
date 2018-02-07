/*
 * ImageSaverThread.h
 *
 *  Created on: Oct 14, 2014
 *      Author: arturo
 */
#pragma once
#include "ofMain.h"
class OSCreceiveThread: public ofThread{
public:
	OSCreceiveThread();
	~OSCreceiveThread();

	//void save(unsigned char * pixels);
	void waitReady();
	void threadedFunction();

private:
	ofPixels pixels;
	ofThreadChannel<unsigned char *> channel;
	ofThreadChannel<bool> channelReady;
	bool firstMessage;
};
