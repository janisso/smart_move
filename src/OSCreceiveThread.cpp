/*
 * ImageSaverThread.cpp
 *
 *  Created on: Oct 14, 2014
 *      Author: arturo
 */

#include "OSCreceiveThread.h"

OSCreceiveThread::OSCreceiveThread()
:firstMessage(true){
	startThread();
}

OSCreceiveThread::~OSCreceiveThread(){
	channel.close();
	channelReady.close();
	waitForThread(true);
}

/*void OSCreceiveThread::save(unsigned char * pixels){
	// send the pixels to save to the thread
	channel.send(pixels);
}*/

void OSCreceiveThread::waitReady(){
	// wait till the thread is done saving the
	// previous frame
	bool ready;
	channelReady.receive(ready);
}

void OSCreceiveThread::threadedFunction(){
	// wait to receive some pixels,
	// save them as jpeg and then tell the main
	// thread that we are done
	// if the channel closes go out of the thread
	unsigned char * p;
	while(channel.receive(p)){
		pixels.setFromPixels(p,1024,768,OF_PIXELS_RGB);
		ofSaveImage(pixels,ofToString(ofGetFrameNum())+".jpg");
		channelReady.send(true);
	}
}
