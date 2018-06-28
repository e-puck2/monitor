/*
 *  CommThread.cpp
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

#include "CommThread.h"
#include "EpuckMonitor.h"

void CommThread::init() {   
    type = 1;						/**< type of the image: color (value 1) or grayscale (value 0)*/
    width = 40;						/**< width of the image to be received*/
    height = 40;					/**< height of the image to be received*/
    pixNum = 3200;
    zoom = 8;

    socket = new QTcpSocket(this);
    connect(socket, SIGNAL(connected()),this, SLOT(connected()));
    connect(socket, SIGNAL(disconnected()),this, SLOT(disconnected()));
    connect(socket, SIGNAL(bytesWritten(qint64)),this, SLOT(bytesWritten(qint64)));
    connect(socket, SIGNAL(readyRead()),this, SLOT(readyRead()));
}

void CommThread::initConnection(QString ip) {

    packet_index = 0;
    read_state = 0;

    socket->connectToHost(ip, 1000);

    fpsTimer = new QTimer(this);
    QObject::connect(fpsTimer, SIGNAL(timeout()), this, SLOT(timerEvent()));
    fpsTimer->start(2000);

}

CommThread::~CommThread() {

}

void CommThread::connected()
{
    qDebug() << "connected...";
    emit updateUiState(UI_STATE_ROBOT_CONNECTED);

    output_buffer[0] = 0x80;
    output_buffer[1] = 0x02; // Bit0: start/stop image stream; bit1: start/stop sensors stream.
    output_buffer[2] = 0x00; // Left speed LSB
    output_buffer[3] = 0x00; // Left speed MSB
    output_buffer[4] = 0x00; // Right speed LSB
    output_buffer[5] = 0x00; // Right speed MSB
    output_buffer[6] = 0x00; // LED1
    output_buffer[7] = 0x00; // LED3
    output_buffer[8] = 0x00; // LED5

    next_request = output_buffer[1];

    socket->skip(MAX_BUFF_SIZE); // Flush input buffer.
    socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
}

void CommThread::disconnected()
{
    qDebug() << "disconnected...";
    emit updateUiState(UI_STATE_ROBOT_DISCONNECTED);
    fpsTimer->stop();
}

void CommThread::bytesWritten(qint64 bytes)
{
    //qDebug() << bytes << " bytes written...";
}

void CommThread::readyRead()
{
    //qDebug() << "reading...";

    // read the data from the socket
    //qDebug() << socket->readAll();

    if(packet_index == 0) {
        memset(input_buffer, 0x0, MAX_BUFF_SIZE);
    }

    while(socket->bytesAvailable() > 0) {

        switch(read_state) {
            case 0: // Read header
                socket->read((char*)&input_buffer[0], 1);
                //qDebug() << "available = " << socket->bytesAvailable();
                //qDebug() << "id = " << input_buffer[0];
                if(input_buffer[0] == 0x01) {
                    read_state = 1;
                } else if(input_buffer[0] == 0x02) {
                    read_state = 2;
                } else if(input_buffer[0] == 0x03) {
                    if(send_cmd) {
                        send_cmd = false;
                        output_buffer[1] = next_request;
                        //qDebug() << "next req 0 = " << next_request;
                        socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
                    }
                }
                packet_index = 0;
                break;

            case 1: // Read image
                packet_index += socket->read((char*)&input_buffer[packet_index], MAX_BUFF_SIZE-packet_index);
                //qDebug() << "ind = " << packet_index;

                if(packet_index == MAX_BUFF_SIZE) {
                    packet_index = 0;
                    img_count++;
                    emit newImage();
                    if((output_buffer[1]&0x02) == 0x00) { // If only image is streamed.
                        if(send_cmd) {
                            send_cmd = false;
                            output_buffer[1] = next_request;
                            //qDebug() << "next req 1 = " << next_request;
                            socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
                        }
                    }
                    read_state = 0;
                }
                break;

            case 2: // Read sensors
                packet_index += socket->read((char*)&input_buffer[packet_index], 64-packet_index);
                //qDebug() << "ind = " << packet_index;

                if(packet_index == 64) {
                    packet_index = 0;

                    //for(int i=0; i<64; i++) {
                    //    qDebug() << std::dec << (int)input_buffer[i] << ", ";
                    //}

                    long  mantis=0;
                    short  exp=0;
                    float flt=0;

                    // Compute acceleration
                    mantis = (input_buffer[0] & 0xff) + ((input_buffer[1] & 0xffl) << 8) + (((input_buffer[2] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[3] & 0x7f) * 2 + ((input_buffer[2] & 0x80) ? 1 : 0);
                    if (input_buffer[3] & 0x80) {
                        mantis = -mantis;
                    }
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    acceleration=flt;

                    // Compute orientation.
                    mantis = (input_buffer[4] & 0xff) + ((input_buffer[5] & 0xffl) << 8) + (((input_buffer[6] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[7] & 0x7f) * 2 + ((input_buffer[6] & 0x80) ? 1 : 0);
                    if (input_buffer[7] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    orientation=flt;
                    if (orientation < 0.0 )
                        orientation=0.0;
                    if (orientation > 360.0 )
                        orientation=360.0;

                    // Compute inclination.
                    mantis = (input_buffer[8] & 0xff) + ((input_buffer[9] & 0xffl) << 8) + (((input_buffer[10] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[11] & 0x7f) * 2 + ((input_buffer[10] & 0x80) ? 1 : 0);
                    if (input_buffer[11] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    inclination=flt;
                    if (inclination < 0.0 )
                        inclination=0.0;
                    if (inclination > 180.0 )
                        inclination=180.0;

                    // Proximity sensors data.
                    ir0 = (input_buffer[12]+input_buffer[13]*256>2000)?2000:input_buffer[12]+input_buffer[13]*256;
                    ir1 = (input_buffer[14]+input_buffer[15]*256>2000)?2000:input_buffer[14]+input_buffer[15]*256;
                    ir2 = (input_buffer[16]+input_buffer[17]*256>2000)?2000:input_buffer[16]+input_buffer[17]*256;
                    ir3 = (input_buffer[18]+input_buffer[19]*256>2000)?2000:input_buffer[18]+input_buffer[19]*256;
                    ir4 = (input_buffer[20]+input_buffer[21]*256>2000)?2000:input_buffer[20]+input_buffer[21]*256;
                    ir5 = (input_buffer[22]+input_buffer[23]*256>2000)?2000:input_buffer[22]+input_buffer[23]*256;
                    ir6 = (input_buffer[24]+input_buffer[25]*256>2000)?2000:input_buffer[24]+input_buffer[25]*256;
                    ir7 = (input_buffer[26]+input_buffer[27]*256>2000)?2000:input_buffer[26]+input_buffer[27]*256;
                    if(ir0<0) {
                        ir0=0;
                    }
                    if(ir1<0) {
                        ir1=0;
                    }
                    if(ir2<0) {
                        ir2=0;
                    }
                    if(ir3<0) {
                        ir3=0;
                    }
                    if(ir4<0) {
                        ir4=0;
                    }
                    if(ir5<0) {
                        ir5=0;
                    }
                    if(ir6<0) {
                        ir6=0;
                    }
                    if(ir7<0) {
                        ir7=0;
                    }

                    // Compute abmient light.
                    lightAvg += (input_buffer[28]+input_buffer[29]*256);
                    lightAvg += (input_buffer[30]+input_buffer[31]*256);
                    lightAvg += (input_buffer[32]+input_buffer[33]*256);
                    lightAvg += (input_buffer[34]+input_buffer[35]*256);
                    lightAvg += (input_buffer[36]+input_buffer[37]*256);
                    lightAvg += (input_buffer[38]+input_buffer[39]*256);
                    lightAvg += (input_buffer[40]+input_buffer[41]*256);
                    lightAvg += (input_buffer[42]+input_buffer[43]*256);
                    lightAvg = (int) (lightAvg/8);
                    lightAvg = (lightAvg>4000)?4000:lightAvg;
                    if(lightAvg<0) {
                        lightAvg=0;
                    }

                    // Microphone
                    micVolume[0] = ((uint8_t)input_buffer[44]+(uint8_t)input_buffer[45]*256>1500)?1500:((uint8_t)input_buffer[44]+(uint8_t)input_buffer[45]*256);
                    micVolume[1] = ((uint8_t)input_buffer[46]+(uint8_t)input_buffer[47]*256>1500)?1500:((uint8_t)input_buffer[46]+(uint8_t)input_buffer[47]*256);
                    micVolume[2] = ((uint8_t)input_buffer[48]+(uint8_t)input_buffer[49]*256>1500)?1500:((uint8_t)input_buffer[48]+(uint8_t)input_buffer[49]*256);
                    micVolume[3] = ((uint8_t)input_buffer[50]+(uint8_t)input_buffer[51]*256>1500)?1500:((uint8_t)input_buffer[50]+(uint8_t)input_buffer[51]*256);

                    // Battery
                    batteryRaw = (uint8_t)input_buffer[52]+(uint8_t)input_buffer[53]*256;
                    memset(batteryRawStr, 0x0, 5);
                    sprintf(batteryRawStr, "%d", batteryRaw);

                    // Gyro
                    gyroRaw[0] = input_buffer[54]+input_buffer[55]*256;
                    gyroRaw[1] = input_buffer[56]+input_buffer[57]*256;
                    gyroRaw[2] = input_buffer[58]+input_buffer[59]*256;

                    // ToF
                    distanceCm = (uint16_t)(((uint8_t)input_buffer[61]<<8)|((uint8_t)input_buffer[60]))/10;
                    memset(distanceCmStr, 0x0, 5);
                    sprintf(distanceCmStr, "%d", (distanceCm>200)?200:distanceCm);

                    // Micro sd state.
                    microSdState = input_buffer[62];

                    // Button state.
                    buttonState = input_buffer[63];

                    sensors_count++;
                    emit newBinaryData();
                    if(send_cmd) {
                        send_cmd = false;
                        output_buffer[1] = next_request;
                        //qDebug() << "next req 2 = " << next_request;
                        socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
                    }
                    read_state = 0;
                }
                break;

        }

    }

}

void CommThread::enableSensors(bool state) {
    if(state) {
        next_request |= 0x02;
    } else {
        next_request &= ~(0x02);
    }
    send_cmd = true;
}

void CommThread::enableCamera(bool state) {
    if(state) {
        next_request |= 0x01;
    } else {
        next_request &= ~(0x01);
    }
    send_cmd = true;
}


void CommThread::closeCommunication() {
    img_count = 0;
    sensors_count = 0;
    socket->disconnectFromHost();
}

void CommThread::timerEvent() {
    fps = (double)img_count/2.0;
    refresh_rate = (double)sensors_count/2.0;
    //qDebug() << "sensors rate = " << refresh_rate;
    img_count = 0;
    sensors_count = 0;
    emit updateFps();
}

void CommThread::getImg(unsigned char *img) {
    memcpy(img, &input_buffer[0], MAX_BUFF_SIZE);
}

void CommThread::updateParameters(int t, int w, int h, int z) {

    type = t;
    width = w;
    height = h;
    zoom = z;
    switch(type) {
        case 0: pixNum = width*height;
                break;
        case 1: pixNum = width*height*2;
                break;
    }

//    memset(command, 0x0, 20);
//    sprintf(command,"J,%d,%d,%d,%d\r", type, width, height, zoom);

//    comm->writeData(command, strlen(command), 100000);
//    comm->readData(RxBuffer, 3, 100000);    //response is j\r\n

    return;
}

void CommThread::goForward() {
    //qDebug() << "go fw";
    int speed_left = motorSpeed;
    output_buffer[2] = speed_left & 0xFF;
    output_buffer[3] = (speed_left>>8) & 0xFF;
    int speed_right = motorSpeed;
    output_buffer[4] = speed_right & 0xFF;
    output_buffer[5] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goBackward() {
    //qDebug() << "go bw";
    int speed_left = -motorSpeed;
    output_buffer[2] = speed_left & 0xFF;
    output_buffer[3] = (speed_left>>8) & 0xFF;
    int speed_right = -motorSpeed;
    output_buffer[4] = speed_right & 0xFF;
    output_buffer[5] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goLeft() {
    //qDebug() << "go sx";
    int speed_left = -motorSpeed;
    output_buffer[2] = speed_left & 0xFF;
    output_buffer[3] = (speed_left>>8) & 0xFF;
    int speed_right = motorSpeed;
    output_buffer[4] = speed_right & 0xFF;
    output_buffer[5] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goRight() {
    //qDebug() << "go dx";
    int speed_left = motorSpeed;
    output_buffer[2] = speed_left & 0xFF;
    output_buffer[3] = (speed_left>>8) & 0xFF;
    int speed_right = -motorSpeed;
    output_buffer[4] = speed_right & 0xFF;
    output_buffer[5] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::stopMotors() {
    //qDebug() << "stop";
    output_buffer[2] = 0;
    output_buffer[3] = 0;
    output_buffer[4] = 0;
    output_buffer[5] = 0;
    send_cmd = true;
}

void CommThread::led0Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[6] = 1;
    } else {
        output_buffer[6] = 0;
    }
    send_cmd = true;
}

void CommThread::led1Slot(int state) {
//    uint8_t bytesToSend = 0;
//    memset(command, 0x0, 20);
//
//    if(state == Qt::Checked) {
//        rgbLedState[0] = rgbLedValue[0];
//        rgbLedState[1] = rgbLedValue[1];
//        rgbLedState[2] = rgbLedValue[2];
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 1, 1, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    } else {
//        rgbLedState[0] = 0;
//        rgbLedState[1] = 0;
//        rgbLedState[2] = 0;
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 1, 0, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    }

//    comm->writeData(command, bytesToSend, 12000);
}

void CommThread::led2Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] = 1;
    } else {
        output_buffer[7] = 0;
    }
    send_cmd = true;
}

void CommThread::led3Slot(int state) {
//    uint8_t bytesToSend = 0;
//    memset(command, 0x0, 20);

//    if(state == Qt::Checked) {
//        rgbLedState[3] = rgbLedValue[0];
//        rgbLedState[4] = rgbLedValue[1];
//        rgbLedState[5] = rgbLedValue[2];
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 3, 1, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    } else {
//        rgbLedState[3] = 0;
//        rgbLedState[4] = 0;
//        rgbLedState[5] = 0;
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 3, 0, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    }

//    comm->writeData(command, bytesToSend, 12000);
}

void CommThread::led4Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[8] = 1;
    } else {
        output_buffer[8] = 0;
    }
    send_cmd = true;
}

void CommThread::led5Slot(int state) {
//    uint8_t bytesToSend = 0;
//    memset(command, 0x0, 20);

//    if(state == Qt::Checked) {
//        rgbLedState[6] = rgbLedValue[0];
//        rgbLedState[7] = rgbLedValue[1];
//        rgbLedState[8] = rgbLedValue[2];
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 5, 1, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    } else {
//        rgbLedState[6] = 0;
//        rgbLedState[7] = 0;
//        rgbLedState[8] = 0;
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 5, 0, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    }

//    comm->writeData(command, bytesToSend, 12000);
}

void CommThread::led6Slot(int state) {
    if(state == Qt::Checked) {
        //output_buffer[...] = 1;
    } else {
        //output_buffer[...] = 0;
    }
    send_cmd = true;
}

void CommThread::led7Slot(int state) {
//    uint8_t bytesToSend = 0;
//    memset(command, 0x0, 20);

//    if(state == Qt::Checked) {
//        rgbLedState[9] = rgbLedValue[0];
//        rgbLedState[10] = rgbLedValue[1];
//        rgbLedState[11] = rgbLedValue[2];
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 7, 1, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    } else {
//        rgbLedState[9] = 0;
//        rgbLedState[10] = 0;
//        rgbLedState[11] = 0;
//        if((int)asercomVer == 1) {
//            bytesToSend = 4;
//            sprintf(command, "%c%c%c%c",-'L', 7, 0, 0);
//        } else {
//            bytesToSend = 13;
//            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
//        }
//    }

//    comm->writeData(command, bytesToSend, 12000);
}

void CommThread::led8Slot(int state) {
    if(state == Qt::Checked) {
        //output_buffer[...] = 1;
    } else {
        //output_buffer[...] = 0;
    }
    send_cmd = true;
}

void CommThread::led9Slot(int state) {
    if(state == Qt::Checked) {
        //output_buffer[...] = 1;
    } else {
        //output_buffer[...] = 0;
    }
    send_cmd = true;
}

void CommThread::sound1Slot() {
    //output_buffer[...] = 1;
    send_cmd = true;
}

void CommThread::sound2Slot() {
    //output_buffer[...] = 2;
    send_cmd = true;
}

void CommThread::sound3Slot() {
    //output_buffer[...] = 3;
    send_cmd = true;
}

void CommThread::sound4Slot() {
    //output_buffer[...] = 4;
    send_cmd = true;
}

void CommThread::sound5Slot() {
    //output_buffer[...] = 5;
    send_cmd = true;
}

void CommThread::audioOffSlot() {
    //output_buffer[...] = 6;
    send_cmd = true;
}

void CommThread::updateRed(int value) {
    rgbLedValue[0] = value;
    //updateLed1Now = true;
    //updateLed3Now = true;
    //updateLed5Now = true;
    //updateLed7Now = true;
}

void CommThread::updateGreen(int value) {
    rgbLedValue[1] = value;
    //updateLed1Now = true;
    //updateLed3Now = true;
    //updateLed5Now = true;
    //updateLed7Now = true;
}

void CommThread::updateBlue(int value) {
    rgbLedValue[2] = value;
    //updateLed1Now = true;
    //updateLed3Now = true;
    //updateLed5Now = true;
    //updateLed7Now = true;
}


