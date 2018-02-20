/*
 *  CommThread.h
 *  EPuckMonitor
 *
 *  Created by Stefano Morgani on 11/18/08.
 *
 *	Copyright 2008 GCtronic
 *
 *  This file is part of EPuckMonitor.
 *
 *  EPuckMonitor is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  EPuckMonitor is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with EPuckMonitor; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 */

/*! \class CommThread
 *  \brief Thread designed to send binary commands
 *  \author Stefano Morgani
 *  \version 1.0
 *  \date 11/18/08
 *
 * This class contains the commands that will be sent to the E-Puck robot in binary mode; the sensors inquiried are: accelerometer, proximity sensors (for proximity and ambient light) and microphones.
 */

#define ZERO_THRESHOLD 3            /**< discrimination window. */

#ifndef COMMTHREAD_H_
#define COMMTHREAD_H_

#include <QThread>
#include <QMutex>
#ifdef __WIN32__
	#include "comm.h"
#else
	#include "SerialComm.h"
#endif
#include <math.h>
#include <iostream>
#include <stdio.h>

#define IMAGE_BUFF_SIZE (4056+3)

class CommThread : public QThread
{
    Q_OBJECT

        protected:
		void run();

    public:
        ~CommThread();

		#ifdef __WIN32__
			void setComm(TCommPort *sc){comm=sc;}
		#else
			void setComm(SerialComm *sc){comm=sc;}
        #endif
        void init();
        void closeCommunication();
        float getAcceleration(){return acceleration;}
        float getOrientation(){return orientation;}
        float getInclination(){return inclination;}
        int getAccX(){return acc_x;}
        int getAccY(){return acc_y;}
        int getAccZ(){return acc_z;}
		int getIr0(){return ir0;}
		int getIr1(){return ir1;}
		int getIr2(){return ir2;}
		int getIr3(){return ir3;}
		int getIr4(){return ir4;}
		int getIr5(){return ir5;}
		int getIr6(){return ir6;}
		int getIr7(){return ir7;}
		int getLight(){return lightAvg;}
        uint16_t getMic(uint8_t id){return micVolume[id];}
        void getSensors(bool b) { getSensorsData = b; }
        void getCamera(bool b) { getCameraData = b; }
        void getImg(unsigned char *img);
        int getType(){return type;}
        int getWidth(){return width;}
        int getHeight(){return height;}
        int getPixNum(){return pixNum;}
        char* getSelector(){return selectorStr;}
        char* getIrCheck(){return irCheckStr;}
        char* getIrAddress(){return irAddressStr;}
        char* getIrData(){return irDataStr;}
        void setImgType(int t) { type = t; }
        void setImgWidth(int w) { width = w; }
        void setImgHeight(int h) { height = h; }
        void setImgPixNum(int p) { pixNum = p; }
        uint16_t getBatteryRaw(){return batteryRaw;}
        char* getBatteryRawStr(){return batteryRawStr;}
        int16_t getGyroRaw(uint8_t axis){return gyroRaw[axis];}
        uint8_t getAsercomVer(void){return asercomVer;}
        uint16_t getDistanceCm(void){return distanceCm;}
        char* getDistanceCmStr(void){return distanceCmStr;}
        uint8_t buttonIsPressed(void){return buttonState==1;}
		uint8_t getMicroSdState(void){return microSdState;}

        void sendGoUp(int motorSpeed);					/**< called when the "F" button is clicked; send the command to move forward the robot*/
        void sendGoDown(int motorSpeed);					/**< called when the "B" button is clicked; send the command to move backward the robot*/
        void sendGoLeft(int motorSpeed);					/**< called when the "L" button is clicked; send the command to turn left the robot*/
        void sendGoRight(int motorSpeed);					/**< called when the "R" button is clicked; send the command to turn right the robot*/
        void sendStop();					/**< called when the "S" button is clicked; send the command to stop the robot*/
        void sendUpdateLed0(int state);					/**< called when the "LED0" checkbox is checked/unchecked; send the command to turn on/off the LED0*/
        void sendUpdateLed1(int state);					/**< called when the "LED1" checkbox is checked/unchecked; send the command to turn on/off the LED1*/
        void sendUpdateLed2(int state);					/**< called when the "LED2" checkbox is checked/unchecked; send the command to turn on/off the LED2*/
        void sendUpdateLed3(int state);					/**< called when the "LED3" checkbox is checked/unchecked; send the command to turn on/off the LED3*/
        void sendUpdateLed4(int state);					/**< called when the "LED4" checkbox is checked/unchecked; send the command to turn on/off the LED4*/
        void sendUpdateLed5(int state);					/**< called when the "LED5" checkbox is checked/unchecked; send the command to turn on/off the LED5*/
        void sendUpdateLed6(int state);					/**< called when the "LED6" checkbox is checked/unchecked; send the command to turn on/off the LED6*/
        void sendUpdateLed7(int state);					/**< called when the "LED7" checkbox is checked/unchecked; send the command to turn on/off the LED7*/
        void sendUpdateLed8(int state);					/**< called when the "LED8" checkbox is checked/unchecked; send the command to turn on/off the body led*/
        void sendUpdateLed9(int state);					/**< called when the "LED9" checkbox is checked/unchecked; send the command to turn on/off the front led*/
        void sendSound1();					/**< called when the "1" button is clicked; send the command to play the first sound*/
        void sendSound2();					/**< called when the "2" button is clicked; send the command to play the second sound*/
        void sendSound3();					/**< called when the "3" button is clicked; send the command to play the third sound*/
        void sendSound4();					/**< called when the "4" button is clicked; send the command to play the fourth sound*/
        void sendSound5();					/**< called when the "5" button is clicked; send the command to play the fifth sound*/
        void sendAudioOff();

        bool headerReceived;					/**< boolean indicating whether or not the first three bytes of the image data (header) was received*/
        bool imgReceived;						/**< boolean indicating whether or not the image was received completely*/
        bool wrongAnswer;						/**< boolean indicating whether or not a wrong header was received*/
        bool abortThread;

	private:
        //QMutex *mutex;
		float acceleration, orientation, inclination;		/**< acceleration data*/
        int acc_x, acc_y, acc_z, roll_acc, pitch_acc;
		int ir0, ir1, ir2, ir3, ir4, ir5, ir6, ir7;			/**< proximity sensors data*/
		int lightAvg;										/**< light sensor data*/
        uint16_t micVolume[4];								/**< microphone data*/
        char selectorStr[3];								/**< selector data*/
        char irCheckStr[8], irAddressStr[8], irDataStr[8];	/**< IR data*/
        char RxBuffer[45];
        char command[20];
        unsigned char imgBuffer[IMAGE_BUFF_SIZE];				/**< image data; IMAGE_BUFF_SIZE is the maximum number of bytes that can be received at one time from the robot.*/
        unsigned int type;						/**< type of the image: color (value 1) or grayscale (value 0)*/
        unsigned int width;						/**< width of the image to be received*/
        unsigned int height;					/**< height of the image to be received*/
        unsigned int pixNum;					/**< total number of pixels (bytes) to be received; in case of grayscale image it is width*height, in case of color image it is width*height*2 (RGB565)*/
        unsigned int zoom;
        bool getSensorsData;
        bool getCameraData;
        bool goUpNow, goDownNow, goLeftNow, goRightNow, stopNow;
        bool updateLed0Now, updateLed1Now, updateLed2Now, updateLed3Now, updateLed4Now, updateLed5Now, updateLed6Now, updateLed7Now, updateLed8Now, updateLed9Now;
        bool sound1Now, sound2Now, sound3Now, sound4Now, sound5Now, audioOffNow;
        int motorSpeed;
        int stateLed0, stateLed1, stateLed2, stateLed3, stateLed4, stateLed5, stateLed6, stateLed7, stateLed8, stateLed9;
        uint16_t batteryRaw;
        int16_t gyroRaw[3];
        char batteryRawStr[5];
        uint8_t asercomVer;
        uint16_t distanceCm;
        char distanceCmStr[5];
        uint8_t rgbLedValue[3];
        uint8_t rgbLedState[12];
        uint8_t buttonState;
		uint8_t microSdState;
        char portName[50];
		#ifdef __WIN32__
			TCommPort *comm;								/**< pointer to the serial port for the bluetooth device (Windows)*/
		#else
			SerialComm *comm;								/**< pointer to the serial port for the bluetooth device (Linux, MacOS)*/
		#endif


    public slots:
        void updateParameters(int t, int w, int h, int z);		/**< called when the "Send Parameters" button is clicked; send the command to the robot to change the camera parameters*/
        void initConnection(char* portName);
        void goUpSlot(int speed);
        void goDownSlot(int speed);
        void goLeftSlot(int speed);
        void goRightSlot(int speed);
        void stopSlot();
        void led0Slot(int state);
        void led1Slot(int state);
        void led2Slot(int state);
        void led3Slot(int state);
        void led4Slot(int state);
        void led5Slot(int state);
        void led6Slot(int state);
        void led7Slot(int state);
        void led8Slot(int state);
        void led9Slot(int state);
        void sound1Slot();
        void sound2Slot();
        void sound3Slot();
        void sound4Slot();
        void sound5Slot();
        void audioOffSlot();
        void updateRed(int value);
        void updateGreen(int value);
        void updateBlue(int value);

    signals:
        void newBinaryData();
        void newAsciiData();
        void newImage();
        void cannotOpenPort(QString s);
        void portOpened();
        void showVersion(QString, int);
        void reconnect();
        void portClosed();

};

#endif
