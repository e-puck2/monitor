/*
 *  EpuckMonitor.h
 *  EPuckMonitor
 *
 *  Created by Stefano Morgani on 10/2/08.
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

/*! \class EpuckMonitor
 *  \brief Main window class
 *  \author Stefano Morgani
 *  \version 1.0
 *  \date 10/2/08
 *
 * This class contains all the graphical objects that are visualized in the main window; moreover it starts and stops all the threads used to communicate with the E-Puck robot.
 */

#ifndef EPUCKMONITOR_H_
#define EPUCKMONITOR_H_

#include <QMessageBox>
#include "ui_main.h"
#include "CommThread.h"
#include "glwidget.h"
#include <stdio.h>
#include <QtMath>
#include <QTimer>

#define TEST_STOPPED 0
#define TEST_STARTED 1

class EpuckMonitor : public QMainWindow
{
    Q_OBJECT
	
    public:
        EpuckMonitor(QMainWindow *parent=0);
        ~EpuckMonitor();
        void setCommThread(CommThread *thread);

        Ui::MainWindow ui;
        int motorSpeed;					/**< used to store the value of the velocity changed with the slider*/
        CommThread *commThread;
        bool imgReceived;				/**< indicate whether or not the image was received completely*/
        QImage img;
        QLabel *lblCamera;
        unsigned int type;						/**< type of the image: color (value 1) or grayscale (value 0)*/
        unsigned int width;						/**< width of the image to be received*/
        unsigned int height;					/**< height of the image to be received*/
        unsigned int pixNum;					/**< total number of pixels (bytes) to be received; in case of grayscale image it is width*height, in case of color image it is width*height*2 (RGB565)*/
        unsigned int zoom;
        char command[20];
        char response[3];
        unsigned char imgBuffer[IMAGE_BUFF_SIZE];
        bool isReceiving;
        GLWidget *glWidget;
        char backgroundColor[100];
        uint8_t rgbState;
        uint8_t testState;
        QTimer *testTimer;

    public slots:
        void connect();					/**< called when the "Connect" button is clicked; initialize the connection and the threads*/
        void disconnect();				/**< called when the "Disconnect" button is clicked; stop the connection and the threads if they are running*/
        void updateParameters();		/**< called when the "Send Parameters" button is clicked; send the command to the robot to change the camera parameters*/
        void goUp();					/**< called when the "F" button is clicked; send the command to move forward the robot*/
        void goDown();					/**< called when the "B" button is clicked; send the command to move backward the robot*/
        void goLeft();					/**< called when the "L" button is clicked; send the command to turn left the robot*/
        void goRight();					/**< called when the "R" button is clicked; send the command to turn right the robot*/
        void sensorActivation(int state);		/**< called when the "Activate Sensors" checkbox is checked/unchecked; activate/deactivate the threads for receiveing data from the sensors*/
        void updateSpeed();				/**< called when the slider changes its state; update the "motorSpeed" variable accordingly*/
        void binarySensorsUpdate();		/**< called when the "binaraySensorThread" terminates; update the graphical objects accordingly to the received data*/
        void asciiSensorsUpdate();		/**< called when the "asciiSensorThread" terminates; update the graphical objects accordingly to the received data*/
        void cameraUpdate();			/**< called when the "cameraThread" terminates; update the image accordingly to the received data*/
        void getImages();				/**< called when the "Get Image" button is clicked; start the "cameraThread" in order to receive an image from the robot*/
        void printMessage(QString s);
        void portOpened();
        void test();
        void updateRgbLeds();

    signals:
        void newParameters(int t, int w, int h, int z);
        void moveForward(int motorSpeed);
        void moveBackward(int motorSpeed);
        void moveLeft(int motorSpeed);
        void moveRight(int motorSpeed);
        void connectToRobot(char* portName);
        void new_x_angle(int value);
        void new_y_angle(int value);
        void new_z_angle(int value);

};

#endif /*EPUCKMONITOR_H_*/
