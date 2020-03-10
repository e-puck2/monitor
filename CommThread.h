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

#include <QMainWindow>
#include <QMutex>
#include <QSerialPort>
#include <QTimer>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#define IMAGE_BUFF_SIZE (4056+3)
#define TIMEOUT_MS 3000
#define MAX_CONN_TRIALS 5
#define RX_BUFF_SIZE 64

class CommThread : public QObject
{
    Q_OBJECT

    public:
        ~CommThread();
        void init();
        void setSpeed(int16_t speed) { motorSpeed=speed; }
        void closeCommunication(bool reconnect_flag, bool from_btn_disconnect);
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

	private:
        //QMutex *mutex;
		float acceleration, orientation, inclination;		/**< acceleration data*/
        int acc_x, acc_y, acc_z, roll_acc, pitch_acc;
		int ir0, ir1, ir2, ir3, ir4, ir5, ir6, ir7;			/**< proximity sensors data*/
		int lightAvg;										/**< light sensor data*/
        uint16_t micVolume[4];								/**< microphone data*/
        char selectorStr[3];								/**< selector data*/
        char irCheckStr[8], irAddressStr[8], irDataStr[8];	/**< IR data*/
        char RxBuffer[RX_BUFF_SIZE];
        char command[20];
        unsigned char imgBuffer[IMAGE_BUFF_SIZE];				/**< image data; IMAGE_BUFF_SIZE is the maximum number of bytes that can be received at one time from the robot.*/
        unsigned int type;						/**< type of the image: color (value 1) or grayscale (value 0)*/
        unsigned int width;						/**< width of the image to be received*/
        unsigned int height;					/**< height of the image to be received*/
        unsigned int pixNum;					/**< total number of pixels (bytes) to be received; in case of grayscale image it is width*height, in case of color image it is width*height*2 (RGB565)*/
        unsigned int zoom;
        bool getSensorsData;
        bool getCameraData;
        int stateLed0, stateLed1, stateLed2, stateLed3, stateLed4, stateLed5, stateLed6, stateLed7, stateLed8, stateLed9;
        uint16_t batteryRaw;
        int16_t gyroRaw[3];
        char batteryRawStr[5];
        uint8_t asercomVer;
        uint16_t distanceCm;
        char distanceCmStr[5];
        uint8_t rgbLedDesiredValue[3];
        uint8_t rgbLedValue[12];
        int rgbLedState[4];
        uint8_t buttonState;
		uint8_t microSdState;
        char portName[50];
        QSerialPort *serialPort = nullptr;
        uint8_t read_state;
        uint8_t temp_buffer[100];
        uint16_t packet_index;
        uint16_t bytesToRead = 0;
        uint16_t bytesRead = 0;
        long  mantis=0;
        short  exp=0;
        float flt=0;
        int selector;
        int ir_check, ir_address, ir_data;
        QTimer m_timer;
        char async_command[20];
        int16_t motorSpeed;
        uint8_t test_state = 0;
        std::vector<std::string> cmd_list;
        std::vector<int> cmd_list_len;
        std::vector<bool> cmd_list_need_answer;
        std::vector<int> cmd_list_answer_len;
        char command_sensors[20];
        uint8_t bytesToSendSensors = 0;
        char command_cam[20];
        bool sending_async_commands = false;
        uint8_t conn_state = 0;
        uint8_t conn_trials = 0;
        bool reading_ascii_data = false;

    public slots:
        void handleTimeout();
        void handleBytesWritten(qint64 bytes);
        void readyRead();
        void handleError(QSerialPort::SerialPortError error);
        void updateParameters(int t, int w, int h, int z);		/**< called when the "Send Parameters" button is clicked; send the command to the robot to change the camera parameters*/
        void initConnection(char* portName);
        void goForward();   /**< called when the "F" button is clicked; send the command to move forward the robot*/
        void goBackward();  /**< called when the "B" button is clicked; send the command to move backward the robot*/
        void goLeft();      /**< called when the "L" button is clicked; send the command to turn left the robot*/
        void goRight();     /**< called when the "R" button is clicked; send the command to turn right the robot*/
        void stopMotors();  /**< called when the "S" button is clicked; send the command to stop the robot*/
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
        void printDialog(QString s);
        void portOpened();
        void showVersion(QString, int);
        void disconnect();
        void reconnect();
        void portClosed();

};

#endif
