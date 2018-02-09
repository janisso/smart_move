#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "attitude_estimator.h"
#include "kickClass.h"

// listen on port 12345
#define PORT 54321
#define NUM_MSG_STRINGS 20
#define HOST "localhost"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        ofVec3f normalize3(ofVec3f quat3_);
        double getLength(ofVec3f quat3_);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void displayGraph(vector <ofVec3f> _history, int _mult);
    
    ofTrueTypeFont font;
    ofxOscReceiver receiver;
    
    int current_msg_string;
    string msg_strings[NUM_MSG_STRINGS];
    float timers[NUM_MSG_STRINGS];
    
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    float lax,lay,laz,lgx,lgy,lgz,lmx,lmy,lmz;
    //string t;
    //long t, l_t;
    float t, l_t, dt, d_ts;
    long ts, l_ts;
    
    
    vector <ofVec3f> accHistory;
    vector <ofVec3f> gyrHistory;
    vector <ofVec3f> magHistory;
    
    int 	bufferCounter;
    int 	drawCounter;
    
    float smoothedVol;
    float scaledVol;
    
    ofBoxPrimitive box,box1,box2;
    ofSpherePrimitive sphere;
    float pitch, roll;
    
    ofQuaternion slerpRot;
    
    stateestimation::AttitudeEstimator Est;
    
    ofQuaternion curRot;
    //double aQ[4];
    ofVec3f quat3, normQuat;
    double q[4], l_q[4], dq[4];
    
    ofFile sensorData, quaternionData;
    ofFile beatData;
    
    int sendNote;
    
    ofxOscSender sender;
    
    kickClass kicker;
		
};
