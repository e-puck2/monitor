/*
 *  main.cpp
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
 
#include "EpuckMonitor.h"

int main(int argc, char *argv[]) {

    QApplication app(argc, argv);
	  
    EpuckMonitor main;
    CommThread *commThread = new CommThread();
    commThread->init();
    main.setCommThread(commThread);
	 
    main.ui.chkRotate->setVisible(false);	//feature not implemented
	
    main.show();
    QObject::connect(main.ui.btnConnect, SIGNAL(clicked()), &main, SLOT(connect()));
    QObject::connect(main.ui.btnDisconnect, SIGNAL(clicked()), &main, SLOT(disconnect()));
    QObject::connect(commThread, SIGNAL(reconnect()), &main, SLOT(connect()));
    QObject::connect(main.ui.btnParameters, SIGNAL(clicked()), &main, SLOT(updateParameters()));
    QObject::connect(&main, SIGNAL(newParameters(int,int,int,int)), commThread, SLOT(updateParameters(int,int,int,int)));
    QObject::connect(main.ui.btnUp, SIGNAL(clicked()), &main, SLOT(goUp()));
    QObject::connect(main.ui.btnDown, SIGNAL(clicked()), &main, SLOT(goDown()));
    QObject::connect(main.ui.btnLeft, SIGNAL(clicked()), &main, SLOT(goLeft()));
    QObject::connect(main.ui.btnRight, SIGNAL(clicked()), &main, SLOT(goRight()));
    QObject::connect(main.ui.btnStop, SIGNAL(clicked()), commThread, SLOT(stopSlot()));
    QObject::connect(&main, SIGNAL(moveForward(int)), commThread, SLOT(goUpSlot(int)));
    QObject::connect(&main, SIGNAL(moveBackward(int)), commThread, SLOT(goDownSlot(int)));
    QObject::connect(&main, SIGNAL(moveRight(int)), commThread, SLOT(goRightSlot(int)));
    QObject::connect(&main, SIGNAL(moveLeft(int)), commThread, SLOT(goLeftSlot(int)));
    QObject::connect(main.ui.btn1, SIGNAL(clicked()), commThread, SLOT(sound1Slot()));
    QObject::connect(main.ui.btn2, SIGNAL(clicked()), commThread, SLOT(sound2Slot()));
    QObject::connect(main.ui.btn3, SIGNAL(clicked()), commThread, SLOT(sound3Slot()));
    QObject::connect(main.ui.btn4, SIGNAL(clicked()), commThread, SLOT(sound4Slot()));
    QObject::connect(main.ui.btn5, SIGNAL(clicked()), commThread, SLOT(sound5Slot()));
    QObject::connect(main.ui.btnAudioOff, SIGNAL(clicked()), commThread, SLOT(audioOffSlot()));
    QObject::connect(main.ui.checkLed0, SIGNAL(stateChanged(int)), commThread, SLOT(led0Slot(int)));
    QObject::connect(main.ui.checkLed1, SIGNAL(stateChanged(int)), commThread, SLOT(led1Slot(int)));
    QObject::connect(main.ui.checkLed2, SIGNAL(stateChanged(int)), commThread, SLOT(led2Slot(int)));
    QObject::connect(main.ui.checkLed3, SIGNAL(stateChanged(int)), commThread, SLOT(led3Slot(int)));
    QObject::connect(main.ui.checkLed4, SIGNAL(stateChanged(int)), commThread, SLOT(led4Slot(int)));
    QObject::connect(main.ui.checkLed5, SIGNAL(stateChanged(int)), commThread, SLOT(led5Slot(int)));
    QObject::connect(main.ui.checkLed6, SIGNAL(stateChanged(int)), commThread, SLOT(led6Slot(int)));
    QObject::connect(main.ui.checkLed7, SIGNAL(stateChanged(int)), commThread, SLOT(led7Slot(int)));
    QObject::connect(main.ui.checkLed8, SIGNAL(stateChanged(int)), commThread, SLOT(led8Slot(int)));
    QObject::connect(main.ui.checkLed9, SIGNAL(stateChanged(int)), commThread, SLOT(led9Slot(int)));
    QObject::connect(main.ui.chkSensors, SIGNAL(stateChanged(int)), &main, SLOT(sensorActivation(int)));
    QObject::connect(main.ui.sliderVel, SIGNAL(valueChanged(int)), &main, SLOT(updateSpeed()));
    QObject::connect(&main, SIGNAL(connectToRobot(char*)), commThread, SLOT(initConnection(char*)));
    QObject::connect(main.ui.btnImage, SIGNAL(clicked()), &main, SLOT(getImages()));
    QObject::connect(commThread, SIGNAL(cannotOpenPort(QString)), &main, SLOT(printMessage(QString)));
    QObject::connect(commThread, SIGNAL(portClosed()), &main, SLOT(disconnect()));
    QObject::connect(commThread, SIGNAL(portOpened()), &main, SLOT(portOpened()));
    QObject::connect(commThread, SIGNAL(showVersion(QString,int)), main.ui.statusbar, SLOT(showMessage(QString,int)));
    QObject::connect(main.ui.slideRed, SIGNAL(valueChanged(int)), commThread, SLOT(updateRed(int)));
    QObject::connect(main.ui.slideGreen, SIGNAL(valueChanged(int)), commThread, SLOT(updateGreen(int)));
    QObject::connect(main.ui.slideBlue, SIGNAL(valueChanged(int)), commThread, SLOT(updateBlue(int)));
    QObject::connect(main.ui.btnTest, SIGNAL(clicked()), &main, SLOT(test()));

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();

}

