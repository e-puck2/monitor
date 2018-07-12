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
    output_buffer[2] = 0x00; // Behavior / others
    output_buffer[3] = 0x00; // Left speed LSB
    output_buffer[4] = 0x00; // Left speed MSB
    output_buffer[5] = 0x00; // Right speed LSB
    output_buffer[6] = 0x00; // Right speed MSB
    output_buffer[7] = 0x00; // LEDs
    output_buffer[8] = 0x00; // LED2 red
    output_buffer[9] = 0x00; // LED2 green
    output_buffer[10] = 0x00; // LED2 blue
    output_buffer[11] = 0x00; // LED4 red
    output_buffer[12] = 0x00; // LED4 green
    output_buffer[13] = 0x00; // LED4 blue
    output_buffer[14] = 0x00; // LED6 red
    output_buffer[15] = 0x00; // LED6 green
    output_buffer[16] = 0x00; // LED6 blue
    output_buffer[17] = 0x00; // LED8 red
    output_buffer[18] = 0x00; // LED8 green
    output_buffer[19] = 0x00; // LED8 blue
    output_buffer[20] = 0x00; // sound

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
                        output_buffer[20] = 0; // Clear sound value.
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
                            output_buffer[20] = 0; // Clear sound value.
                        }
                    }
                    read_state = 0;
                }
                break;

            case 2: // Read sensors
                packet_index += socket->read((char*)&input_buffer[packet_index], INPUT_BUFFER_SIZE-packet_index);
                //qDebug() << "ind = " << packet_index;

                if(packet_index == INPUT_BUFFER_SIZE) {
                    packet_index = 0;

                    //for(int i=0; i<INPUT_BUFFER_SIZE; i++) {
                    //    qDebug() << std::dec << (int)input_buffer[i] << ", ";
                    //}

                    //acc_x = input_buffer[0] + input_buffer[1]*256;
                    //acc_y = input_buffer[2] + input_buffer[3]*256;
                    //acc_z = input_buffer[4] + input_buffer[5]*256;
                    //qDebug() << "acc_x = " << acc_x;
                    //qDebug() << "acc_y = " << acc_y;
                    //qDebug() << "acc_z = " << acc_z;

                    long  mantis=0;
                    short  exp=0;
                    float flt=0;

                    // Compute acceleration
                    mantis = (input_buffer[6] & 0xff) + ((input_buffer[7] & 0xffl) << 8) + (((input_buffer[8] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[9] & 0x7f) * 2 + ((input_buffer[8] & 0x80) ? 1 : 0);
                    if (input_buffer[9] & 0x80) {
                        mantis = -mantis;
                    }
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    acceleration=flt;

                    // Compute orientation.
                    mantis = (input_buffer[10] & 0xff) + ((input_buffer[11] & 0xffl) << 8) + (((input_buffer[12] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[13] & 0x7f) * 2 + ((input_buffer[12] & 0x80) ? 1 : 0);
                    if (input_buffer[13] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    orientation=flt;
                    if (orientation < 0.0 )
                        orientation=0.0;
                    if (orientation > 360.0 )
                        orientation=360.0;

                    //qDebug() << "orientation = " << orientation;

                    // Compute inclination.
                    mantis = (input_buffer[14] & 0xff) + ((input_buffer[15] & 0xffl) << 8) + (((input_buffer[16] &0x7fl) | 0x80) << 16);
                    exp = (input_buffer[17] & 0x7f) * 2 + ((input_buffer[16] & 0x80) ? 1 : 0);
                    if (input_buffer[17] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    inclination=flt;
                    if (inclination < 0.0 )
                        inclination=0.0;
                    if (inclination > 180.0 )
                        inclination=180.0;

                    //qDebug() << "inclination = " << inclination;

                    // Gyro
                    gyroRaw[0] = input_buffer[18]+input_buffer[19]*256;
                    gyroRaw[1] = input_buffer[20]+input_buffer[21]*256;
                    gyroRaw[2] = input_buffer[22]+input_buffer[23]*256;

                    // Magnetometer
                    magneticField[0] = *((float*)&input_buffer[24]);
                    magneticField[1] = *((float*)&input_buffer[28]);
                    magneticField[2] = *((float*)&input_buffer[32]);

                    // Temperature.
                    //temperature = input_buffer[36];
                    //qDebug() << "temp = " << temperature;

                    // Proximity sensors data.
                    ir0 = (input_buffer[37]+input_buffer[38]*256>2000)?2000:input_buffer[37]+input_buffer[38]*256;
                    ir1 = (input_buffer[39]+input_buffer[40]*256>2000)?2000:input_buffer[39]+input_buffer[40]*256;
                    ir2 = (input_buffer[41]+input_buffer[42]*256>2000)?2000:input_buffer[41]+input_buffer[42]*256;
                    ir3 = (input_buffer[43]+input_buffer[44]*256>2000)?2000:input_buffer[43]+input_buffer[44]*256;
                    ir4 = (input_buffer[45]+input_buffer[46]*256>2000)?2000:input_buffer[45]+input_buffer[46]*256;
                    ir5 = (input_buffer[47]+input_buffer[48]*256>2000)?2000:input_buffer[47]+input_buffer[48]*256;
                    ir6 = (input_buffer[49]+input_buffer[50]*256>2000)?2000:input_buffer[49]+input_buffer[50]*256;
                    ir7 = (input_buffer[51]+input_buffer[52]*256>2000)?2000:input_buffer[51]+input_buffer[52]*256;
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
                    lightAvg += (input_buffer[53]+input_buffer[54]*256);
                    lightAvg += (input_buffer[55]+input_buffer[56]*256);
                    lightAvg += (input_buffer[57]+input_buffer[58]*256);
                    lightAvg += (input_buffer[59]+input_buffer[60]*256);
                    lightAvg += (input_buffer[61]+input_buffer[62]*256);
                    lightAvg += (input_buffer[63]+input_buffer[64]*256);
                    lightAvg += (input_buffer[65]+input_buffer[66]*256);
                    lightAvg += (input_buffer[67]+input_buffer[68]*256);
                    lightAvg = (int) (lightAvg/8);
                    lightAvg = (lightAvg>4000)?4000:lightAvg;
                    if(lightAvg<0) {
                        lightAvg=0;
                    }

                    // ToF
                    distanceCm = (uint16_t)(((uint8_t)input_buffer[70]<<8)|((uint8_t)input_buffer[69]))/10;
                    memset(distanceCmStr, 0x0, 5);
                    sprintf(distanceCmStr, "%d", (distanceCm>200)?200:distanceCm);

                    // Microphone
                    micVolume[0] = ((uint8_t)input_buffer[71]+(uint8_t)input_buffer[72]*256>1500)?1500:((uint8_t)input_buffer[71]+(uint8_t)input_buffer[72]*256);
                    micVolume[1] = ((uint8_t)input_buffer[73]+(uint8_t)input_buffer[74]*256>1500)?1500:((uint8_t)input_buffer[73]+(uint8_t)input_buffer[74]*256);
                    micVolume[2] = ((uint8_t)input_buffer[75]+(uint8_t)input_buffer[76]*256>1500)?1500:((uint8_t)input_buffer[75]+(uint8_t)input_buffer[76]*256);
                    micVolume[3] = ((uint8_t)input_buffer[77]+(uint8_t)input_buffer[78]*256>1500)?1500:((uint8_t)input_buffer[77]+(uint8_t)input_buffer[78]*256);

                    // Left steps
                    //leftSteps = (input_buffer[79]+input_buffer[80]*256);
                    //qDebug() << "left steps = " << leftSteps;

                    // Right steps
                    //rightSteps = (input_buffer[81]+input_buffer[82]*256);

                    // Battery
                    batteryRaw = (uint8_t)input_buffer[83]+(uint8_t)input_buffer[84]*256;
                    memset(batteryRawStr, 0x0, 5);
                    sprintf(batteryRawStr, "%d", batteryRaw);

                    // Micro sd state.
                    microSdState = input_buffer[85];

                    // Tv remote.
                    irCheck = input_buffer[86];
                    irAddress = input_buffer[87];
                    irData = input_buffer[88];
                    memset(irCheckStr, 0x0, 8);
                    memset(irAddressStr, 0x0, 8);
                    memset(irDataStr, 0x0, 8);
                    sprintf(irCheckStr, "%x", irCheck);
                    sprintf(irAddressStr, "%x", irAddress);
                    sprintf(irDataStr, "%x", irData);

                    // Selector.
                    selector = input_buffer[89];
                    memset(selectorStr, 0x0, 3);
                    sprintf(selectorStr, "%d", selector);

                    // Ground sensor proximity.
                    //groundProx[0] = input_buffer[90]+input_buffer[91]*256;
                    //groundProx[1] = input_buffer[92]+input_buffer[93]*256;
                    //groundProx[2] = input_buffer[94]+input_buffer[95]*256;

                    // Ground sensor ambient light.
                    //groundAmbient[0] = input_buffer[96]+input_buffer[97]*256;
                    //groundAmbient[1] = input_buffer[98]+input_buffer[99]*256;
                    //groundAmbient[2] = input_buffer[100]+input_buffer[101]*256;

                    // Button state.
                    buttonState = input_buffer[102];

                    sensors_count++;
                    emit newBinaryData();
                    if(send_cmd) {
                        send_cmd = false;
                        output_buffer[1] = next_request;
                        //qDebug() << "next req 2 = " << next_request;
                        socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
                        output_buffer[20] = 0; // Clear sound value.
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
    output_buffer[3] = speed_left & 0xFF;
    output_buffer[4] = (speed_left>>8) & 0xFF;
    int speed_right = motorSpeed;
    output_buffer[5] = speed_right & 0xFF;
    output_buffer[6] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goBackward() {
    //qDebug() << "go bw";
    int speed_left = -motorSpeed;
    output_buffer[3] = speed_left & 0xFF;
    output_buffer[4] = (speed_left>>8) & 0xFF;
    int speed_right = -motorSpeed;
    output_buffer[5] = speed_right & 0xFF;
    output_buffer[6] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goLeft() {
    //qDebug() << "go sx";
    int speed_left = -motorSpeed;
    output_buffer[3] = speed_left & 0xFF;
    output_buffer[4] = (speed_left>>8) & 0xFF;
    int speed_right = motorSpeed;
    output_buffer[5] = speed_right & 0xFF;
    output_buffer[6] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::goRight() {
    //qDebug() << "go dx";
    int speed_left = motorSpeed;
    output_buffer[3] = speed_left & 0xFF;
    output_buffer[4] = (speed_left>>8) & 0xFF;
    int speed_right = -motorSpeed;
    output_buffer[5] = speed_right & 0xFF;
    output_buffer[6] = (speed_right>>8) & 0xFF;
    send_cmd = true;
}

void CommThread::stopMotors() {
    //qDebug() << "stop";
    output_buffer[3] = 0;
    output_buffer[4] = 0;
    output_buffer[5] = 0;
    output_buffer[6] = 0;
    send_cmd = true;
}

void CommThread::led0Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x01;
    } else {
        output_buffer[7] &= ~(0x01);
    }
    send_cmd = true;
}

void CommThread::led1Slot(int state) {
    rgbLedState[0] = state;
    if(state == Qt::Checked) {
        output_buffer[8] = rgbLedValue[0];
        output_buffer[9] = rgbLedValue[1];
        output_buffer[10] = rgbLedValue[2];
    } else {
        output_buffer[8] = 0;
        output_buffer[9] = 0;
        output_buffer[10] = 0;
    }
    send_cmd = true;
}

void CommThread::led2Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x02;
    } else {
        output_buffer[7] &= ~(0x02);
    }
    send_cmd = true;
}

void CommThread::led3Slot(int state) {
    rgbLedState[1] = state;
    if(state == Qt::Checked) {
        output_buffer[11] = rgbLedValue[0];
        output_buffer[12] = rgbLedValue[1];
        output_buffer[13] = rgbLedValue[2];
    } else {
        output_buffer[11] = 0;
        output_buffer[12] = 0;
        output_buffer[13] = 0;
    }
    send_cmd = true;
}

void CommThread::led4Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x04;
    } else {
        output_buffer[7] &= ~(0x04);
    }
    send_cmd = true;
}

void CommThread::led5Slot(int state) {
    rgbLedState[2] = state;
    if(state == Qt::Checked) {
        output_buffer[14] = rgbLedValue[0];
        output_buffer[15] = rgbLedValue[1];
        output_buffer[16] = rgbLedValue[2];
    } else {
        output_buffer[14] = 0;
        output_buffer[15] = 0;
        output_buffer[16] = 0;
    }
    send_cmd = true;
}

void CommThread::led6Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x08;
    } else {
        output_buffer[7] &= ~(0x08);
    }
    send_cmd = true;
}

void CommThread::led7Slot(int state) {
    rgbLedState[3] = state;
    if(state == Qt::Checked) {
        output_buffer[17] = rgbLedValue[0];
        output_buffer[18] = rgbLedValue[1];
        output_buffer[19] = rgbLedValue[2];
    } else {
        output_buffer[17] = 0;
        output_buffer[18] = 0;
        output_buffer[19] = 0;
    }
    send_cmd = true;
}

void CommThread::led8Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x10;
    } else {
        output_buffer[7] &= ~(0x10);
    }
    send_cmd = true;
}

void CommThread::led9Slot(int state) {
    if(state == Qt::Checked) {
        output_buffer[7] |= 0x20;
    } else {
        output_buffer[7] &= ~(0x20);
    }
    send_cmd = true;
}

void CommThread::sound1Slot() {
    output_buffer[20] |= 0x01;
    send_cmd = true;
}

void CommThread::sound2Slot() {
    output_buffer[20] |= 0x02;
    send_cmd = true;
}

void CommThread::sound3Slot() {
    output_buffer[20] |= 0x04;
    send_cmd = true;
}

void CommThread::sound4Slot() {
    output_buffer[20] |= 0x08;
    send_cmd = true;
}

void CommThread::sound5Slot() {
    output_buffer[20] |= 0x10;
    send_cmd = true;
}

void CommThread::audioOffSlot() {
    output_buffer[20] |= 0x20;
    send_cmd = true;
}

void CommThread::updateRed(int value) {
    rgbLedValue[0] = value;
    if(rgbLedState[0] == Qt::Checked) {
        output_buffer[8] = value;
        send_cmd = true;
    }
    if(rgbLedState[1] == Qt::Checked) {
        output_buffer[11] = value;
        send_cmd = true;
    }
    if(rgbLedState[2] == Qt::Checked) {
        output_buffer[14] = value;
        send_cmd = true;
    }
    if(rgbLedState[3] == Qt::Checked) {
        output_buffer[17] = value;
        send_cmd = true;
    }
}

void CommThread::updateGreen(int value) {
    rgbLedValue[1] = value;
    if(rgbLedState[0] == Qt::Checked) {
        output_buffer[9] = value;
        send_cmd = true;
    }
    if(rgbLedState[1] == Qt::Checked) {
        output_buffer[12] = value;
        send_cmd = true;
    }
    if(rgbLedState[2] == Qt::Checked) {
        output_buffer[15] = value;
        send_cmd = true;
    }
    if(rgbLedState[3] == Qt::Checked) {
        output_buffer[18] = value;
        send_cmd = true;
    }
}

void CommThread::updateBlue(int value) {
    rgbLedValue[2] = value;
    if(rgbLedState[0] == Qt::Checked) {
        output_buffer[10] = value;
        send_cmd = true;
    }
    if(rgbLedState[1] == Qt::Checked) {
        output_buffer[13] = value;
        send_cmd = true;
    }
    if(rgbLedState[2] == Qt::Checked) {
        output_buffer[16] = value;
        send_cmd = true;
    }
    if(rgbLedState[3] == Qt::Checked) {
        output_buffer[19] = value;
        send_cmd = true;
    }
}


