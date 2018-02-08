#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    //ofSetFrameRate(60);
    //cout << "listening for osc messages on port " << PORT << "\n";
    receiver.setup(PORT);
    sender.setup(HOST, 5555);
    
    current_msg_string = 0;
    
    ax = 0;    ay = 0;    az = 0;
    gx = 0;    gy = 0;    gz = 0;
    mx = 0;    my = 0;    mz = 0;
    
    lax = 0; lay = 0; laz = 0;
    
    for (int i = 0; i < 924; i++){
        accHistory.push_back(ofVec3f(0.0,0.0,0.0));
        gyrHistory.push_back(ofVec3f(0.0,0.0,0.0));
        magHistory.push_back(ofVec3f(0.0,0.0,0.0));
    }
    //volHistory.assign(1024-100, 0.0);
    
    float width = ofGetWidth() * .12;
    box.set(400, 20, 200);
    box.setSideColor(box.SIDE_BACK, ofColor::red);
    box.setSideColor(box.SIDE_LEFT, ofColor::black);
    box.setSideColor(box.SIDE_RIGHT, ofColor::blue);
    box.setSideColor(box.SIDE_TOP, ofColor::green);
    box.setSideColor(box.SIDE_FRONT, ofColor::yellow);
    box.setSideColor(box.SIDE_BOTTOM, ofColor::maroon);
    pitch = 0;  roll = 0;
    
    sphere.set( 50, 60, OF_PRIMITIVE_TRIANGLE_STRIP );
    //sphere.setColor(0,255,0);
    
    t = 0;
    l_t = 0;
    dt = 0.0;
    
    ts = 0;
    l_ts = 0;
    d_ts = 0;
    
    //ESTIMATION SETUP
    // Initialise the estimator (e.g. in the class constructor, none of these are actually strictly required for the estimator to work, and can be set at any time)
    Est.setMagCalib(1.1, 26.3, -37.2);         // Recommended: Use if you want absolute yaw information as opposed to just relative yaw (Default: (0.68, -1.32, 0.0))
    /*Est.setPIGains(2.2, 2.65, 10, 1.25);       // Recommended: Use if the default gains (shown) do not provide optimal estimator performance (Note: Ki = Kp/Ti)*/
    Est.setQLTime(3);                        // Optional: Use if the default quick learning time is too fast or too slow for your application (Default: 3.0)
    /*Est.setAttitude(0.5, 0.5, 0.5, 0.5);       // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
    Est.setAttitudeEuler(M_PI, 0.0, 0.0);      // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
    Est.setAttitudeFused(M_PI, 0.0, 0.0, 1.0); // Optional: Use if you have prior knowledge about the orientation of the robot (Default: Identity orientation)
    Est.setGyroBias(0, 0, 0);     // Optional: Use if you have prior knowledge about the gyroscope bias (Default: (0.0, 0.0, 0.0))
    Est.setAccMethod(Est.ME_FUSED_YAW);        // Optional: Use if you wish to experiment with varying acc-only resolution methods*/
    void ofResetElapsedTimeCounter();
    
    
    for(int i =0; i< 4; i++){
        q[i]=0;
        dq[i]=0;
        l_q[i]=0;
        //aQ[0]=0;
    }
    //q[4] = {0,0,0,0};
    
    
    sensorData.open("sensor_data.csv",ofFile::WriteOnly);
    quaternionData.open("quaternion_data.csv",ofFile::WriteOnly);
    
    kicker.setup();
}

//--------------------------------------------------------------
void ofApp::update(){
    // hide old messages
    for(int i = 0; i < NUM_MSG_STRINGS; i++){
        if(timers[i] < ofGetElapsedTimef()){
            msg_strings[i] = "";
        }
    }
    
    // check for waiting messages
    while(receiver.hasWaitingMessages()){
        // get the next message
        ofxOscMessage m;
        receiver.getNextMessage(m);
        if(m.getAddress() == "/a"){
            //atof(num.c_str());
            ts = m.getArgAsInt64(0);
            //cout << t << endl;
            // both the arguments are int32's
            ax = m.getArgAsFloat(1);
            ay = m.getArgAsFloat(2);
            az = m.getArgAsFloat(3);
            
            gx = m.getArgAsFloat(4);
            gy = m.getArgAsFloat(5);
            gz = m.getArgAsFloat(6);
            
            mx = m.getArgAsFloat(7);
            my = m.getArgAsFloat(8);
            mz = m.getArgAsFloat(9);
        }
        
        /*if(m.getAddress() == "/q"){

            aQ[0] = m.getArgAsFloat(0);
            aQ[1] = m.getArgAsFloat(1);
            aQ[2] = m.getArgAsFloat(2);
            aQ[3] = m.getArgAsFloat(3);
        }*/
    }
    

    
    //dt = float(t - l_t)/1000;
    
    
    lax = ax * 0.5 + (lax * (1.0 - 0.5));
    lay = ay * 0.5 + (lay * (1.0 - 0.5));
    laz = az * 0.5 + (laz * (1.0 - 0.5));
    
    lmx = mx * 0.5 + (lmx * (1.0 - 0.5));
    lmy = my * 0.5 + (lmy * (1.0 - 0.5));
    lmz = mz * 0.5 + (lmz * (1.0 - 0.5));
    
    lgx = gx * 0.5 + (lgx * (1.0 - 0.5));
    lgy = gy * 0.5 + (lgy * (1.0 - 0.5));
    lgz = gz * 0.5 + (lgz * (1.0 - 0.5));
    
    //roll  = (atan2(-lay, laz)*180.0)/M_PI;
    //pitch = (atan2(lax, sqrt(lay*lay + laz*laz))*180.0)/M_PI;
    
    //pitch = atan(ax/((ax*ax)+(az*az)));
    
    //lets record the volume into an array
    accHistory.push_back(ofVec3f(lax, lay, laz));
    gyrHistory.push_back(ofVec3f(lgx, lgy, lgz));
    magHistory.push_back(ofVec3f(lmx, lmy, lmz));
    
    //if we are bigger the the size we want to record - lets drop the oldest value
    if( accHistory.size() >= 924 ){
        accHistory.erase(accHistory.begin(), accHistory.begin()+1);
        gyrHistory.erase(gyrHistory.begin(), gyrHistory.begin()+1);
        magHistory.erase(magHistory.begin(), magHistory.begin()+1);
    }
    t = ofGetElapsedTimef();
    kicker.update(t,lax,lay,laz);
    if(kicker.send_kick){
        ofxOscMessage k;
        k.setAddress("/k");
        k.addFloatArg(kicker.kick_intensity);
        sender.sendMessage(k, false);
        kicker.send_kick = false;
    }
    //string s(t.str());
    sensorData << to_string(t) << ", " << to_string(ax) << ", " << to_string(ay) <<  ", " << to_string(az) << ", " << to_string(gx) << ", " << to_string(gy) << ", " << to_string(gz) << ", " << to_string(mx) << ", " << to_string(my) << ", " <<  to_string(mz) << endl;
    quaternionData << to_string(t) << ", " << to_string(q[0]) << ", " << to_string(q[1]) << ", " << to_string(q[2]) << ", " << to_string(q[3]) << endl;
    dt = t-l_t;
    kicker.update(t,lax,lay,laz);
    l_t = t;
    //cout << dt <<endl;
    
    d_ts = float(ts-l_ts)/1000.f;
    
    
    
    //cout << dt << " " << d_ts << endl;
    l_ts = ts;
    Est.update(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
    //Est.update(dt, lgx, lgy, lgz, lax, lay, laz, lmx, lmy, lmz);
    //double q[4];
    Est.getAttitude(q);
    /*for (int i = 0; i < 4; i++){
        aQ[i]=q[i];
    }*/
    //cout << "My attitude is (quaternion): (" << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ")" << endl;
    //cout <<  << endl;
    //cout << "My attitude is (ZYX Euler): (" << Est.eulerYaw() << "," << Est.eulerPitch() << "," << Est.eulerRoll() << ")" << endl;
    //cout << "My attitude is (fused): (" << Est.fusedYaw() << "," << Est.fusedPitch() << "," << Est.fusedRoll() << "," << (Est.fusedHemi() ? 1 : -1) << ")" << endl;
    ofxOscMessage a;
    a.setAddress("/a");
    a.addFloatArg(lax);
    a.addFloatArg(lay);
    a.addFloatArg(laz);
    sender.sendMessage(a, false);
    
    ofxOscMessage g;
    g.setAddress("/g");
    g.addFloatArg(lgx);
    g.addFloatArg(lgy);
    g.addFloatArg(lgz);
    sender.sendMessage(g, false);
    
    ofxOscMessage m;
    m.setAddress("/m");
    m.addFloatArg(lmx);
    m.addFloatArg(lmy);
    m.addFloatArg(lmz);
    sender.sendMessage(m, false);
    
    ofxOscMessage e;
    e.setAddress("/e");
    e.addFloatArg(Est.fusedYaw());
    e.addFloatArg(Est.fusedPitch());
    e.addFloatArg(Est.fusedRoll());
    e.addFloatArg((Est.fusedHemi() ? 1 : -1));
    sender.sendMessage(e, false);
    
    ofxOscMessage aQ;
    aQ.setAddress("/q");
    aQ.addFloatArg(q[0]);
    aQ.addFloatArg(q[1]);
    aQ.addFloatArg(q[2]);
    aQ.addFloatArg(q[3]);
    sender.sendMessage(aQ, false);

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableDepthTest();
    // Box //
    //puts center of everything to center of screen
    //ofPushMatrix();
    ofTranslate(ofGetWidth()*.5, ofGetHeight()*.5, ofGetWidth()*.5);
    sphere.setPosition(200, 0, 0);
    //cout << q[0] << q[1] << q[2] << q[3] << endl;
    //
    //ofVec3f axis;
    //float angle;
    for(int i =0; i< 4; i++){
        dq[i]=q[i]-l_q[i];
    }
    for(int i =0; i< 4; i++){
        l_q[i]=q[i];
    }
    
    //cout << "My attitude is (quaternion): (" << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << ")" << endl;
    //ofSetOrientation(curRot);
    //curRot.getRotate(angle, axis);
    //ofRotate(angle, 0, 0,  0);
    
    /*ofRotateX(-Est.fusedRoll()*(360/(M_PI*2)));
    ofRotateZ(Est.fusedPitch()*(360/(M_PI*2)));
    ofRotateY(Est.fusedYaw()*(360/(M_PI*2)));
    ofRotateX(q[1]*(360/(M_PI*2)));
    ofRotateZ(q[2]*(360/(M_PI*2)));
    ofRotateY(q[3]*(360/(M_PI*2)));*/
    //ofRotate(q[0]*(360/(M_PI*2)), q[1]*(360/(M_PI*2)), q[2]*(360/(M_PI*2)), q[3]*(360/(M_PI*2)));
    
    ofNoFill();
    //ofQuaternion curRot(q[3],q[2],q[0],q[1]);
    
    ofQuaternion curRot(q[3],q[2],q[0],q[1]);
    //ofQuaternion curRot(aQ[3],aQ[0],aQ[1],aQ[2]);
    //vector<ofMeshFace> triangles = box.getMesh().getUniqueFaces();
    box.setOrientation(curRot);
    //sphere.setOrientation(curRot);
    //ofDrawAxis(400);
    ofPushMatrix();
    ofRotateY(90);
    box.draw();
    
    
    
    float theta = 4*M_PI-2*acos(q[3]);
    float x = q[0]*(1.0/sin(theta/2));
    float y = q[1]*(1.0/sin(theta/2));
    float z = q[2]*(1.0/sin(theta/2));
    
    cout << asin(z)*180 << endl;
    
    ofRotateZ(Est.fusedRoll()*(360/(M_PI*2)));
    ofRotateY(Est.fusedYaw()*(360/(M_PI*2)));
    ofRotateX(-Est.fusedPitch()*(360/(M_PI*2)));
    //ofRotateY(Est.fusedYaw()*(360/(M_PI*2)));
    //ofTranslate(0,200,0);
    //ofTranslate(0,0,z*200);//,z*200);
    //ofRotateY();
    sphere.draw();
    ofPopMatrix();
    
    
    //cout <<  << endl;
    /*ofPushMatrix();
    ofTranslate(200, 200);
    
    float theta = acos(q[3])*2;
    //float rotY = asin(q[1])*2;
    
    ofRotateY(theta*180);
    cout << Est.fusedYaw()*(360/(M_PI*2)) << endl;
    ofDrawAxis(200);
    ofPopMatrix();*/
    //ofDisableDepthTest();
    //ofPopMatrix();
    
    
    //ofBox(160,120,10,120);
    //-------ACCELEROMETER DATA-------
    ofPushMatrix();
    ofTranslate(50-ofGetWidth()*.5, 1440/6-ofGetHeight()*.5, -ofGetWidth()*.5);
    displayGraph(accHistory, 10);
    ofPopMatrix();
    
    
    //-------GYRO DATA-------
    ofPushMatrix();
    ofTranslate(50-ofGetWidth()*.5, 1440/2-ofGetHeight()*.5, -ofGetWidth()*.5);
    displayGraph(gyrHistory, 10);
    ofPopMatrix();
    
    
    //-------MAG DATA-------
    ofPushMatrix();
    ofTranslate(50-ofGetWidth()*.5, (1440/6)*5-ofGetHeight()*.5, -ofGetWidth()*.5);
    displayGraph(magHistory, 2);
    ofPopMatrix();
    
    //ofRotateX(pitch);
    //ofDrawBox()
    //ofPushMatrix();
    //ofTranslate(1024/2, 1440/2, 0);

    //ofPopMatrix();
}

void ofApp::displayGraph(vector <ofVec3f> _history, int _mult){

    ofSetColor(255, 0, 0);
    ofNoFill();
    ofBeginShape();
    for (unsigned int i = 0; i < _history.size(); i++){
        ofVertex(i, _history[i].x * _mult);
    }
    ofEndShape(false);
    
    ofSetColor(0, 255, 0);
    ofNoFill();
    ofBeginShape();
    for (unsigned int i = 0; i < _history.size(); i++){
        ofVertex(i, _history[i].y * _mult);
    }
    ofEndShape(false);
    
    ofSetColor(0, 0, 255);
    ofNoFill();
    ofBeginShape();
    for (unsigned int i = 0; i < _history.size(); i++){
        ofVertex(i, _history[i].z * _mult);
    }
    ofEndShape(false);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key == 'r'){
        Est.reset(true, false);                 // Reset into quick learning mode, but preserve the current gyro bias estimate
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
