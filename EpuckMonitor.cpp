/*
 *  EpuckMonitor.cpp
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

EpuckMonitor::EpuckMonitor(QMainWindow *parent) : QMainWindow(parent)
{
    ui.setupUi(this);
    motorSpeed=500;
    lblCamera = new QLabel();
    type = 1;
    width = 40;
    height = 40;
    pixNum = 3200;
    zoom = 8;
    commThread=NULL;
    isReceiving = false;
    testState = TEST_STOPPED;

    //disable all the graphical objects before the connection is established
    ui.chkSensors->setEnabled(false);
    ui.btnParameters->setEnabled(false);
    ui.btnImage->setEnabled(false);
    ui.btnUp->setEnabled(false);
    ui.btnDown->setEnabled(false);
    ui.btnRight->setEnabled(false);
    ui.btnLeft->setEnabled(false);
    ui.btnStop->setEnabled(false);
    ui.btn1->setEnabled(false);
    ui.btn2->setEnabled(false);
    ui.btn3->setEnabled(false);
    ui.btn4->setEnabled(false);
    ui.btn5->setEnabled(false);
    ui.btnAudioOff->setEnabled(false);
    ui.checkLed0->setEnabled(false);
    ui.checkLed1->setEnabled(false);
    ui.checkLed2->setEnabled(false);
    ui.checkLed3->setEnabled(false);
    ui.checkLed4->setEnabled(false);
    ui.checkLed5->setEnabled(false);
    ui.checkLed6->setEnabled(false);
    ui.checkLed7->setEnabled(false);
    ui.checkLed8->setEnabled(false);
    ui.checkLed9->setEnabled(false);
    ui.sliderVel->setEnabled(false);
    ui.txtHeight->setEnabled(false);
    ui.txtWidth->setEnabled(false);
    ui.txtZoom->setEnabled(false);
    ui.radioColor->setEnabled(false);
    ui.radioGrayscale->setEnabled(false);

    ui.txtPort->setFocus();

    glWidget = new GLWidget;
    ui.layoutOpenGL->addWidget(glWidget);
    QObject::connect(this, SIGNAL(new_x_angle(int)), glWidget, SLOT(setXRotation(int)));
    QObject::connect(this, SIGNAL(new_y_angle(int)), glWidget, SLOT(setYRotation(int)));
    QObject::connect(this, SIGNAL(new_z_angle(int)), glWidget, SLOT(setZRotation(int)));

    testTimer = new QTimer(this);
    QObject::connect(testTimer, SIGNAL(timeout()), this, SLOT(updateRgbLeds()));
}

EpuckMonitor::~EpuckMonitor()
{
    disconnect();
    return;
}

void EpuckMonitor::connect() {
    ui.btnConnect->setEnabled(false);
    emit connectToRobot(ui.txtPort->text().toLatin1().data());
    return;
}


void EpuckMonitor::cameraUpdate() {

    switch(type) {
        case 0: {	//grayscale
                    commThread->getImg(imgBuffer);
                    img = QImage(width, height, QImage::Format_RGB32);
                    int i=0;
                    for(int y=0; y<height; y++) {
                        for(int x=0; x<width; x++) {
                            int r = (int)imgBuffer[i];
                            int g = (int)imgBuffer[i];
                            int b = (int)imgBuffer[i];
                            img.setPixel(x, y, qRgb (r, g, b));
                            i++;
                        }
                    }
                    //the size of the label is 200*200, the maximum size of the image is 45*45
                    lblCamera->setFixedSize(width*200/45, height*200/45);
                    lblCamera->setScaledContents(true);
                    lblCamera->setPixmap(QPixmap::fromImage(img));
                    ui.scrollCamera->setWidget(lblCamera);
                    break;
                }

        case 1: {	// RGB565 image
                    commThread->getImg(imgBuffer);
                    img = QImage(width, height, QImage::Format_RGB16);
                    int i=0;
                    for(int y=0; y<height; y++) {
                        for(int x=0; x<width; x++) {
                            int r = (int)imgBuffer[i*2]&0xF8;
                            int g = (int)(imgBuffer[i*2]&0x07)<<5 | (imgBuffer[i*2+1]&0xE0)>>3;
                            int b = (int)(imgBuffer[i*2+1]&0x1F)<<3;
                            img.setPixel(x, y, qRgb (r, g, b));
                            i++;
                        }
                    }
                    lblCamera->setFixedSize(width*200/45, height*200/45);
                    lblCamera->setScaledContents(true);
                    lblCamera->setPixmap(QPixmap::fromImage(img));
                    ui.scrollCamera->setWidget(lblCamera);
                    break;
                }

        default: break;
    }

    return;

}

void EpuckMonitor::binarySensorsUpdate() {

    //accelerometer
    ui.lcdAcceleration->display(commThread->getAcceleration());
    double orientRad = commThread->getOrientation()/180.0*M_PI;
    emit new_y_angle(-sin(orientRad)*commThread->getInclination());
    emit new_x_angle(cos(orientRad)*commThread->getInclination());

    //emit new_x_angle(int(atan2((double)commThread->getAccZ(),(double)commThread->getAccY())/M_PI*180.0)); //pitch
    //emit new_y_angle(int(atan2((double)commThread->getAccZ(),(double)commThread->getAccX())/M_PI*180.0)); //roll

    //proximity sensors
    ui.progressIR0->setValue(commThread->getIr0());
    ui.progressIR1->setValue(commThread->getIr1());
    ui.progressIR2->setValue(commThread->getIr2());
    ui.progressIR3->setValue(commThread->getIr3());
    ui.progressIR4->setValue(commThread->getIr4());
    ui.progressIR5->setValue(commThread->getIr5());
    ui.progressIR6->setValue(commThread->getIr6());
    ui.progressIR7->setValue(commThread->getIr7());

    //light sensor data
    memset(backgroundColor, 0x0, 100);
    sprintf(backgroundColor, "background-color: rgb(%d, %d, %d);", 255-int((double)commThread->getLight()/4000.0*255.0), 255-int((double)commThread->getLight()/4000.0*255.0), 255-int((double)commThread->getLight()/4000.0*255.0));
    ui.lblLight->setStyleSheet(backgroundColor);

    //microphone data
    char str[5];
    ui.progressMic0->setValue(commThread->getMic(0));
    sprintf(str, "%4d", commThread->getMic(0));
    ui.lblMic0Val->setText(str);
    ui.progressMic1->setValue(commThread->getMic(1));
    memset(str, 0x0, 5);
    sprintf(str, "%4d", commThread->getMic(1));
    ui.lblMic1Val->setText(str);
    ui.progressMic2->setValue(commThread->getMic(2));
    memset(str, 0x0, 5);
    sprintf(str, "%4d", commThread->getMic(2));
    ui.lblMic2Val->setText(str);
    ui.progressMic3->setValue(commThread->getMic(3));
    memset(str, 0x0, 5);
    sprintf(str, "%4d", commThread->getMic(3));
    ui.lblMic3Val->setText(str);

    // Battery data.
    ui.lblBattAdc->setText(commThread->getBatteryRawStr());

    // Gyro data.
    ui.progressGyroX->setValue(commThread->getGyroRaw(0));
    ui.progressGyroY->setValue(commThread->getGyroRaw(1));
    ui.progressGyroZ->setValue(commThread->getGyroRaw(2));

    // ToF data.
    ui.lblDist->setText(commThread->getDistanceCmStr());
    ui.progressDistance->setValue((commThread->getDistanceCm()>200)?200:commThread->getDistanceCm());

    // Button state
    if(commThread->buttonIsPressed()) {
        ui.lblBtnState->setText("pressed");
        ui.pushButton->setChecked(true);
    } else {
        ui.lblBtnState->setText("released");
        ui.pushButton->setChecked(false);
    }

	// Micro sd state
	if(commThread->getMicroSdState()) {
        ui.lblSdStateColor->setStyleSheet("background-color: rgb(0, 180, 0);");
        //pLabel->setStyleSheet("QLabel { background-color : red; color : blue; }");
        ui.lblSdStateTxt->setText("pass");
	} else {
        ui.lblSdStateColor->setStyleSheet("background-color: rgb(180, 0, 0);");
        ui.lblSdStateTxt->setText("fail");
	}

    return;
}

void EpuckMonitor::asciiSensorsUpdate() {

    //selector data
    ui.lblSelector->setText(commThread->getSelector());
    ui.dialSelector->setValue(atoi(commThread->getSelector()));
    //IR data
    ui.lblIRcheck->setText(commThread->getIrCheck());
    ui.lblIRaddress->setText(commThread->getIrAddress());
    ui.lblIRdata->setText(commThread->getIrData());

    return;
}

void EpuckMonitor::updateParameters() {

    width = ui.txtWidth->text().toInt();
    height = ui.txtHeight->text().toInt();
    zoom = ui.txtZoom->text().toInt();

    if(ui.radioColor->isChecked()) {
        if((width*height)>(IMAGE_BUFF_SIZE-3)/2) {	//at most 2028 pixels allowed
            printMessage("Height and Width not allowed, at most 2025 pixels!");
            return;
        } else {
            type = 1;
        }
    }
    else if(ui.radioGrayscale->isChecked()) {
        if((width*height)>(IMAGE_BUFF_SIZE-3)/2) {
            printMessage("Height and Width not allowed, at most 2028 pixels!");
            return;
        } else {
            type = 0;
        }
    }

    emit newParameters(type, width, height, zoom);

    return;
}

void EpuckMonitor::goUp() {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    emit moveForward(motorSpeed);

    return;
}

void EpuckMonitor::goDown() {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    emit moveBackward(motorSpeed);

    return;
}

void EpuckMonitor::goLeft() {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    emit moveLeft(motorSpeed);

    return;
}

void EpuckMonitor::goRight() {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    emit moveRight(motorSpeed);

    return;
}

void EpuckMonitor::updateSpeed() {
    char str[5];
    motorSpeed = ui.sliderVel->value();
    sprintf(str, "%4d", motorSpeed);
    ui.lblVelValue->setText(str);
}

void EpuckMonitor::disconnect() {

    commThread->getSensors(false);
    commThread->getCamera(false);
    commThread->abortThread = true;
    commThread->wait();

    //disable all graphical objects
    ui.btnConnect->setEnabled(true);
    ui.btnDisconnect->setEnabled(false);
    ui.chkSensors->setEnabled(false);
    ui.btnParameters->setEnabled(false);
    ui.btnImage->setEnabled(false);
    ui.btnUp->setEnabled(false);
    ui.btnDown->setEnabled(false);
    ui.btnRight->setEnabled(false);
    ui.btnLeft->setEnabled(false);
    ui.btnStop->setEnabled(false);
    ui.btn1->setEnabled(false);
    ui.btn2->setEnabled(false);
    ui.btn3->setEnabled(false);
    ui.btn4->setEnabled(false);
    ui.btn5->setEnabled(false);
    ui.btnAudioOff->setEnabled(false);
    ui.checkLed0->setEnabled(false);
    ui.checkLed1->setEnabled(false);
    ui.checkLed2->setEnabled(false);
    ui.checkLed3->setEnabled(false);
    ui.checkLed4->setEnabled(false);
    ui.checkLed5->setEnabled(false);
    ui.checkLed6->setEnabled(false);
    ui.checkLed7->setEnabled(false);
    ui.checkLed8->setEnabled(false);
    ui.checkLed9->setEnabled(false);
    ui.sliderVel->setEnabled(false);
    ui.txtHeight->setEnabled(false);
    ui.txtWidth->setEnabled(false);
    ui.txtZoom->setEnabled(false);
    ui.radioColor->setEnabled(false);
    ui.radioGrayscale->setEnabled(false);
    ui.btnImage->setText("Start receiving images");
    ui.btnTest->setEnabled(false);

    return;
}

void EpuckMonitor::sensorActivation(int state) {

    if(state == Qt::Checked) {
        commThread->getSensors(true);
    } else {
        commThread->getSensors(false);
    }

    return;
}

void EpuckMonitor::getImages() {
    if(isReceiving) {
        isReceiving = false;
        ui.btnImage->setText("Start receiving images");
        commThread->getCamera(false);
    } else {
        isReceiving = true;
        ui.btnImage->setText("Stop receiving images");
        commThread->getCamera(true);
    }

    return;
}

void EpuckMonitor::printMessage(QString s) {
    QMessageBox::critical( this, "E-Puck Monitor", s);
}

void EpuckMonitor::setCommThread(CommThread *thread) {
    commThread = thread;
    QObject::connect(commThread, SIGNAL(newBinaryData()), this, SLOT(binarySensorsUpdate()));
    QObject::connect(commThread, SIGNAL(newAsciiData()), this, SLOT(asciiSensorsUpdate()));
    QObject::connect(commThread, SIGNAL(newImage()), this, SLOT(cameraUpdate()));
}

void EpuckMonitor::portOpened() {
    ui.btnConnect->setEnabled(false);
    ui.btnDisconnect->setEnabled(true);
    ui.chkSensors->setCheckState(Qt::Checked);

    //enable all the graphical objects after the connection was established
    ui.chkSensors->setEnabled(true);
    ui.btnParameters->setEnabled(true);
    ui.btnImage->setEnabled(true);
    ui.btnUp->setEnabled(true);
    ui.btnDown->setEnabled(true);
    ui.btnRight->setEnabled(true);
    ui.btnLeft->setEnabled(true);
    ui.btnStop->setEnabled(true);
    ui.btn1->setEnabled(true);
    ui.btn2->setEnabled(true);
    ui.btn3->setEnabled(true);
    ui.btn4->setEnabled(true);
    ui.btn5->setEnabled(true);
    ui.btnAudioOff->setEnabled(true);
    ui.checkLed0->setEnabled(true);
    ui.checkLed1->setEnabled(true);
    ui.checkLed2->setEnabled(true);
    ui.checkLed3->setEnabled(true);
    ui.checkLed4->setEnabled(true);
    ui.checkLed5->setEnabled(true);
    ui.checkLed6->setEnabled(true);
    ui.checkLed7->setEnabled(true);
    ui.checkLed8->setEnabled(true);
    ui.checkLed9->setEnabled(true);
    ui.sliderVel->setEnabled(true);
    ui.txtHeight->setEnabled(true);
    ui.txtWidth->setEnabled(true);
    ui.txtZoom->setEnabled(true);
    ui.radioColor->setEnabled(true);
    ui.radioGrayscale->setEnabled(true);
    ui.btnTest->setEnabled(true);
}

void EpuckMonitor::test() {
    if(testState == TEST_STOPPED) {
        testState = TEST_STARTED;
        ui.btnTest->setText("Stop test");

        // Turn on all leds.
        ui.checkLed0->setChecked(false);
        ui.checkLed0->setChecked(true);
        ui.checkLed1->setChecked(false);
        ui.checkLed1->setChecked(true);
        ui.checkLed2->setChecked(false);
        ui.checkLed2->setChecked(true);
        ui.checkLed3->setChecked(false);
        ui.checkLed3->setChecked(true);
        ui.checkLed4->setChecked(false);
        ui.checkLed4->setChecked(true);
        ui.checkLed5->setChecked(false);
        ui.checkLed5->setChecked(true);
        ui.checkLed6->setChecked(false);
        ui.checkLed6->setChecked(true);
        ui.checkLed7->setChecked(false);
        ui.checkLed7->setChecked(true);
        ui.checkLed8->setChecked(false);
        ui.checkLed8->setChecked(true);
        ui.checkLed9->setChecked(false);
        ui.checkLed9->setChecked(true);
        ui.slideRed->setValue(100);
        ui.slideGreen->setValue(0);
        ui.slideBlue->setValue(0);
        rgbState = 1;

        // Turn on sound.
        ui.btn3->click();

        // Turn on motors.
        ui.sliderVel->setValue(150);
        ui.btnUp->click();

        testTimer->start(1000);
    } else {
        testState = TEST_STOPPED;
        ui.btnTest->setText("Start test");

        // Turn off motors.
        ui.btnStop->click();

        testTimer->stop();
    }

    return;
}

void EpuckMonitor::updateRgbLeds() {
    switch(rgbState) {
        case 0:
            ui.slideRed->setValue(100);
            ui.slideGreen->setValue(0);
            ui.slideBlue->setValue(0);
            rgbState = 1;
            break;
        case 1:
            ui.slideRed->setValue(0);
            ui.slideGreen->setValue(100);
            ui.slideBlue->setValue(0);
            rgbState = 2;
            break;
        case 2:
            ui.slideRed->setValue(0);
            ui.slideGreen->setValue(0);
            ui.slideBlue->setValue(100);
            rgbState = 0;
            break;
    }
}

