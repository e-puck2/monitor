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
    getSensorsData = true;
    getCameraData = false;
    motorSpeed = 500;
    memset(command_sensors, 0x00, 20);
    memset(command_cam, 0x00, 20);
    command_cam[0]=-'I';      //binary image receiving command
    command_cam[1]= 0;        //end command
    conn_trials = 0;

    m_timer.setSingleShot(true);
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(handleTimeout()));

    serialPort = new QSerialPort(this);
    connect(serialPort, SIGNAL(bytesWritten(qint64)), this,  SLOT(handleBytesWritten(qint64)));
    connect(serialPort, SIGNAL(errorOccurred(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));
    connect(serialPort, SIGNAL(readyRead()),this, SLOT(readyRead()));
}

void CommThread::initConnection(char* portName) {
    read_state = 0;
    packet_index = 0;

    memset(this->portName, 0x0, 50);
    memcpy(this->portName, portName, strlen(portName));

    serialPort->setPortName(this->portName);
    serialPort->setBaudRate(QSerialPort::Baud115200);

    if (!serialPort->open(QIODevice::ReadWrite)) {
        std::cerr << "Unable to open serial port " << portName << " (err = " << serialPort->error() << ")" << std::endl;
        emit printDialog("Unable to open serial port.");
        closeCommunication(false, false); // Do not reconnect
        return;
    }

    serialPort->clear(QSerialPort::AllDirections);

    memset(command, 0x0, 20);
    sprintf(command, "x\r");
    serialPort->write(command, 2); // Send a fake request to clear the communication with the robot
    //serialPort->flush();
    //serialPort->waitForBytesWritten(-1);
    std::cerr << "Clearing the communication" << std::endl;
    conn_state = 0; // Clearing communication
    m_timer.start(TIMEOUT_MS);

}

CommThread::~CommThread() {
    if(serialPort->isOpen()) {
        serialPort->clear(QSerialPort::AllDirections);
        serialPort->close();
    }
}

void CommThread::handleTimeout() {
    // Actually the same procedure is done for all states, thus the states could be eliminated...
    // At the moment the states are maintained separated in case different actions are needed for them.
    std::cerr << "timeout occurred (" << (int)conn_state << ")" << std::endl;
    switch(conn_state) {
        case 0: // Clearing communication => no answer received from the robot at the beginning
            conn_trials++;
            if(conn_trials == MAX_CONN_TRIALS) { // The robot never answered, thus give up...
                closeCommunication(false, false);
                conn_trials = 0;
            } else { // Restart the communication with the robot
                closeCommunication(true, false);
            }
            break;
        case 1: // Getting protocol version => no answer received from the robot after issueing the request
            conn_trials++;
            if(conn_trials == MAX_CONN_TRIALS) { // The robot never answered, thus give up...
                closeCommunication(false, false);
                conn_trials = 0;
            } else { // Restart the communication with the robot
                closeCommunication(true, false);
            }
            break;
        case 2: // Getting sensors/camera data => no answer received from the robot after issueing the request
            std::cerr << "Only " << (int)bytesRead << "/" << (int)bytesToRead << " bytes received for the previous request, restart the communication..." << std::endl;
            for(int i=0; i<bytesRead; i++) {
                std::cerr << (int)RxBuffer[i] << ", ";
            }
            std::cerr << std::endl;
            //conn_trials++;
            //if(conn_trials == MAX_CONN_TRIALS) { // The robot never answered, thus give up...
            //    closeCommunication(false, false);
            //    conn_trials = 0;
            //} else { // Restart the communication with the robot
                closeCommunication(true, false);
            //}
            break;
    }
}

void CommThread::handleBytesWritten(qint64 bytes) {
    std::cerr << "handleBytesWritten = " << bytes << std::endl;
    //std::cerr << "cmd size = " << cmd_list.size() << std::endl;
    //std::cerr << "answer size = " << cmd_list_need_answer.size() << std::endl;
    //if(cmd_list_need_answer.size() > 0) {
    if(sending_async_commands) {
        if(cmd_list_need_answer.front()) { // The last async command need an answer
            //read_state = 4;
            //std::cerr << "need answer" << std::endl;
        } else {
            cmd_list_need_answer.erase(cmd_list_need_answer.begin());
            cmd_list_answer_len.erase(cmd_list_answer_len.begin());
            if(cmd_list.size() > 0) { // Continue sending async commands if needed
                //std::cerr << "no need answer, send next command" << std::endl;
                memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                packet_index = 0;
                serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                m_timer.start(TIMEOUT_MS);
                cmd_list.erase(cmd_list.begin());
                cmd_list_len.erase(cmd_list_len.begin());
                if(cmd_list_need_answer.front()) {
                    read_state = 4;
                }
            } else { // Re-activate sensors reading
                //std::cerr << "restart sensors" << std::endl;
                sending_async_commands = false;
                memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                packet_index = 0;
                serialPort->write(command_sensors, bytesToSendSensors);
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                m_timer.start(TIMEOUT_MS);
                if((int)asercomVer == 1) {
                    bytesToRead = 58; //12+16+16+6+2+6;
                } else {
                    bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                }
                read_state = 2;
            }
        }
    }
}

void CommThread::handleError(QSerialPort::SerialPortError error) {
    //std::cerr << "handleError" << std::endl;
    //std::cerr << "err = " << error << std::endl;
}

void CommThread::readyRead() {
    std::cerr << "readyRead (" << (int)read_state << "), [" << (int)serialPort->bytesAvailable() << "]" << std::endl;

    while(serialPort->bytesAvailable() > 0) {

        switch(read_state) {
            case 0: // Clear communication
                //m_timer.stop();
                bytesRead = serialPort->read((char *)temp_buffer, 100);
                std::cerr << "Garbage = " << temp_buffer << std::endl;
                bytesToRead = 0;
                std::cerr << (int)bytesRead << "/" << (int)bytesToRead << std::endl;
                serialPort->waitForReadyRead(20);
                break;

            case 1: // Getting asercom version
                //m_timer.stop();
                packet_index += serialPort->read((char *)&temp_buffer[packet_index], 100-packet_index);
                bytesRead = packet_index;
                bytesToRead = 0;
                std::cerr << (int)bytesRead << "/" << (int)bytesToRead << std::endl;
                serialPort->waitForReadyRead(20);
                break;

            case 2: // Read sensors (binary)
               // m_timer.stop();
                packet_index += serialPort->read((char *)&RxBuffer[packet_index], bytesToRead-packet_index);
                bytesRead = packet_index;
                std::cerr << (int)bytesRead << "/" << (int)bytesToRead << std::endl;
                if(packet_index == bytesToRead) {
                    // Acceleration
                    mantis=0;
                    exp=0;
                    flt=0;
                    mantis = (RxBuffer[0] & 0xff) + ((RxBuffer[1] & 0xffl) << 8) + (((RxBuffer[2] &0x7fl) | 0x80) << 16);
                    exp = (RxBuffer[3] & 0x7f) * 2 + ((RxBuffer[2] & 0x80) ? 1 : 0);
                    if (RxBuffer[3] & 0x80) {
                        mantis = -mantis;
                    }
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    acceleration=flt;

                    // Orientation
                    mantis = (RxBuffer[4] & 0xff) + ((RxBuffer[5] & 0xffl) << 8) + (((RxBuffer[6] &0x7fl) | 0x80) << 16);
                    exp = (RxBuffer[7] & 0x7f) * 2 + ((RxBuffer[6] & 0x80) ? 1 : 0);
                    if (RxBuffer[7] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    orientation=flt;
                    if (orientation < 0.0 )
                        orientation=0.0;
                    if (orientation > 360.0 )
                        orientation=360.0;

                    // Inclination
                    mantis = (RxBuffer[8] & 0xff) + ((RxBuffer[9] & 0xffl) << 8) + (((RxBuffer[10] &0x7fl) | 0x80) << 16);
                    exp = (RxBuffer[11] & 0x7f) * 2 + ((RxBuffer[10] & 0x80) ? 1 : 0);
                    if (RxBuffer[11] & 0x80)
                        mantis = -mantis;
                    flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                    inclination=flt;
                    if (inclination < 0.0 )
                        inclination=0.0;
                    if (inclination > 180.0 )
                        inclination=180.0;

                    // Proximity
                    ir0 = (RxBuffer[12]+RxBuffer[13]*256>2000)?2000:RxBuffer[12]+RxBuffer[13]*256;
                    ir1 = (RxBuffer[14]+RxBuffer[15]*256>2000)?2000:RxBuffer[14]+RxBuffer[15]*256;
                    ir2 = (RxBuffer[16]+RxBuffer[17]*256>2000)?2000:RxBuffer[16]+RxBuffer[17]*256;
                    ir3 = (RxBuffer[18]+RxBuffer[19]*256>2000)?2000:RxBuffer[18]+RxBuffer[19]*256;
                    ir4 = (RxBuffer[20]+RxBuffer[21]*256>2000)?2000:RxBuffer[20]+RxBuffer[21]*256;
                    ir5 = (RxBuffer[22]+RxBuffer[23]*256>2000)?2000:RxBuffer[22]+RxBuffer[23]*256;
                    ir6 = (RxBuffer[24]+RxBuffer[25]*256>2000)?2000:RxBuffer[24]+RxBuffer[25]*256;
                    ir7 = (RxBuffer[26]+RxBuffer[27]*256>2000)?2000:RxBuffer[26]+RxBuffer[27]*256;
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

                    // Ambient
                    lightAvg += (RxBuffer[28]+RxBuffer[29]*256);
                    lightAvg += (RxBuffer[30]+RxBuffer[31]*256);
                    lightAvg += (RxBuffer[32]+RxBuffer[33]*256);
                    lightAvg += (RxBuffer[34]+RxBuffer[35]*256);
                    lightAvg += (RxBuffer[36]+RxBuffer[37]*256);
                    lightAvg += (RxBuffer[38]+RxBuffer[39]*256);
                    lightAvg += (RxBuffer[40]+RxBuffer[41]*256);
                    lightAvg += (RxBuffer[42]+RxBuffer[43]*256);
                    lightAvg = (int) (lightAvg/8);
                    lightAvg = (lightAvg>4000)?4000:lightAvg;
                    if(lightAvg<0) {
                        lightAvg=0;
                    }

                    if((int)asercomVer == 1) {
                        // Microphone
                        micVolume[0] = ((uint8_t)RxBuffer[44]+(uint8_t)RxBuffer[45]*256>1500)?1500:((uint8_t)RxBuffer[44]+(uint8_t)RxBuffer[45]*256);
                        micVolume[1] = ((uint8_t)RxBuffer[46]+(uint8_t)RxBuffer[47]*256>1500)?1500:((uint8_t)RxBuffer[46]+(uint8_t)RxBuffer[47]*256);
                        micVolume[2] = ((uint8_t)RxBuffer[48]+(uint8_t)RxBuffer[49]*256>1500)?1500:((uint8_t)RxBuffer[48]+(uint8_t)RxBuffer[49]*256);
                        micVolume[3] = 0;

                        // Battery
                        batteryRaw = (uint8_t)RxBuffer[50]+(uint8_t)RxBuffer[51]*256;
                        memset(batteryRawStr, 0x0, 5);
                        sprintf(batteryRawStr, "%d", batteryRaw);

                        // Gyro
                        gyroRaw[0] = RxBuffer[52]+RxBuffer[53]*256;
                        gyroRaw[1] = RxBuffer[54]+RxBuffer[55]*256;
                        gyroRaw[2] = RxBuffer[56]+RxBuffer[57]*256;

                    } else {
                        // Microphone
                        micVolume[0] = ((uint8_t)RxBuffer[44]+(uint8_t)RxBuffer[45]*256>1500)?1500:((uint8_t)RxBuffer[44]+(uint8_t)RxBuffer[45]*256);
                        micVolume[1] = ((uint8_t)RxBuffer[46]+(uint8_t)RxBuffer[47]*256>1500)?1500:((uint8_t)RxBuffer[46]+(uint8_t)RxBuffer[47]*256);
                        micVolume[2] = ((uint8_t)RxBuffer[48]+(uint8_t)RxBuffer[49]*256>1500)?1500:((uint8_t)RxBuffer[48]+(uint8_t)RxBuffer[49]*256);
                        micVolume[3] = ((uint8_t)RxBuffer[50]+(uint8_t)RxBuffer[51]*256>1500)?1500:((uint8_t)RxBuffer[50]+(uint8_t)RxBuffer[51]*256);

                        // Battery
                        batteryRaw = (uint8_t)RxBuffer[52]+(uint8_t)RxBuffer[53]*256;
                        memset(batteryRawStr, 0x0, 5);
                        sprintf(batteryRawStr, "%d", batteryRaw);

                        // Gyro
                        gyroRaw[0] = RxBuffer[54]+RxBuffer[55]*256;
                        gyroRaw[1] = RxBuffer[56]+RxBuffer[57]*256;
                        gyroRaw[2] = RxBuffer[58]+RxBuffer[59]*256;

                        // Distance sensor
                        distanceCm = (uint16_t)(((uint8_t)RxBuffer[61]<<8)|((uint8_t)RxBuffer[60]))/10;
                        memset(distanceCmStr, 0x0, 5);
                        sprintf(distanceCmStr, "%d", (distanceCm>200)?200:distanceCm);

                        // Button
                        buttonState = RxBuffer[62];

                        // Micro sd
                        microSdState = RxBuffer[63];
                    }

                    emit newBinaryData();

                    /*
                    memset(async_command, 0x0, 20);
                    sprintf(async_command, "%c%c",'C', '\r');
                    cmd_list.push_back(std::string(async_command, 2));
                    cmd_list_len.push_back(2);
                    cmd_list_need_answer.push_back(true);
                    cmd_list_answer_len.push_back(5);
                    */

                    serialPort->write("C\r",2); // Request selector
                    packet_index = 0;
                    memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                    m_timer.start(TIMEOUT_MS);
                    bytesToRead = 5;
                    read_state = 5;

                }

                break;

            case 3: // Read camera
                //m_timer.stop();
                //std::cerr << "Reading camera: " << packet_index << "/"  << (int)bytesToRead << std::endl;
                packet_index += serialPort->read((char*)&imgBuffer[packet_index], bytesToRead-packet_index);
                bytesRead = packet_index;
                std::cerr << (int)bytesRead << "/" << (int)bytesToRead << std::endl;
                if(bytesRead == bytesToRead) {
                    emit newImage();
                }
                break;

            case 4:
                //m_timer.stop();
                packet_index += serialPort->read((char*)&RxBuffer[packet_index], cmd_list_answer_len.front()-packet_index);
                bytesToRead = cmd_list_answer_len.front();
                bytesRead = packet_index;
                std::cerr << (int)bytesRead << "/" << (int)bytesToRead << std::endl;
                if(packet_index == cmd_list_answer_len.front()) {
                    //std::cerr << "answer received" << std::endl;
                    cmd_list_need_answer.erase(cmd_list_need_answer.begin());
                    cmd_list_answer_len.erase(cmd_list_answer_len.begin());
                    if(cmd_list.size() > 0) { // Continue sending async commands if needed
                        memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                        packet_index = 0;
                        serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());
                        //serialPort->flush();
                        //serialPort->waitForBytesWritten(-1);
                        //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                        //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                        m_timer.start(TIMEOUT_MS);
                        cmd_list.erase(cmd_list.begin());
                        cmd_list_len.erase(cmd_list_len.begin());
                    } else { // Re-activate sensors reading
                        sending_async_commands = false;
                        read_state = 2;
                    }
                }
                break;

            case 5: // Read selecor in ASCII
                packet_index += serialPort->read((char *)&RxBuffer[packet_index], bytesToRead-packet_index);
                bytesRead = packet_index;
                if(packet_index == bytesToRead) {
                    memset(selectorStr, 0x0, 3);
                    sscanf((char*)RxBuffer,"c,%d\r\n",&selector);
                    sprintf(selectorStr, "%d", selector);

                    serialPort->write("G\r",2); // Request tv remote
                    packet_index = 0;
                    memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                    m_timer.start(TIMEOUT_MS);
                    bytesToRead = 45;
                    read_state = 6;
                }
                break;

            case 6: // Read tv remote
                packet_index += serialPort->read((char *)&RxBuffer[packet_index], bytesToRead-packet_index);
                bytesRead = packet_index;
                if(packet_index == bytesToRead) {
                    memset(irCheckStr, 0x0, 8);
                    memset(irAddressStr, 0x0, 8);
                    memset(irDataStr, 0x0, 8);
                    sscanf((char*)RxBuffer,"g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n",&ir_check,&ir_address,&ir_data);
                    sprintf(irCheckStr, "%x", ir_check);
                    sprintf(irAddressStr, "%x", ir_address);
                    sprintf(irDataStr, "%x", ir_data);

                    packet_index = 0;
                    memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                    emit newAsciiData();
                    read_state = 2;
                }
                break;

            default:
                std::cerr << "Uknown read_state (1)" << std::endl;
                break;
        }

        //serialPort->waitForReadyRead(20); // This is needed because not all data are always received in one shot. This delay limits the maximum refresh rate

    }

    if((bytesToRead>0) && (bytesRead!=bytesToRead)) {
        return;
    }

    //if(bytesRead == 0) {  // Can't happen because this function "readyRead" is called only when there is something to read. If no data is received, then the timer will timeout instead.
    //} else
    //if(bytesToRead>0 && bytesToRead!=bytesRead) { // Not all expected bytes are received from the robot, restart communication with the robot.
    //    std::cerr << "Not all data recevied for the previous request, restart the communication..." << std::endl;
    //    closeCommunication(true, false);
    //    return;
    //}

    switch(read_state) {
        case 0: // Garbage collected, now request the protocol version
            sprintf(command, "V\r");
            serialPort->write(command, 2);
            //serialPort->flush();
            //serialPort->waitForBytesWritten(-1);
            packet_index = 0;
            read_state = 1;
            conn_state = 1; // Getting protocol version
            std::cerr << "Getting protocol version" << std::endl;
            m_timer.start(TIMEOUT_MS);
            break;

        case 1:
            std::cerr << "version = " << temp_buffer << std::endl;
            emit showVersion((char *)&temp_buffer[2], 0);

            asercomVer = temp_buffer[10]-'0';
            std::cerr << "asercom version = " << (int)asercomVer << std::endl;

            if(asercomVer!=1 && asercomVer!=2) {
                std::cerr << "Unable to work with this protocol version, restart the communication" << std::endl;
                closeCommunication(true, false);
                return;
            }

            emit portOpened(); // Call EPuckMonitor->portOpened to "enable" the interface

            conn_trials = 0;
            conn_state = 2; // Getting sensors/camera data.
            std::cerr << "Getting sensors/camera data " << std::endl;
            //getSensorsData = true;
            //getCameraData = false;

            packet_index = 0;
            memset(RxBuffer, 0x0, RX_BUFF_SIZE);
            if(getSensorsData) {
                if((int)asercomVer == 1) {
                    bytesToSendSensors = 7;
                    bytesToRead = 58; //12+16+16+6+2+6;
                    command_sensors[0]=-'A';                        //accelerometer request
                    command_sensors[1]=-'N';                        //proximity sensors request
                    command_sensors[2]=-'O';                        //ambient light request
                    command_sensors[3]=-'u';                        //microphones request (3 x mic)
                    command_sensors[4]=-'b';                        // Battery raw value.
                    command_sensors[5]=-'g';                        // Gyro raw value.
                    command_sensors[6]=0;                           //binary command ending
                } else {
                    bytesToSendSensors = 10;
                    bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                    command_sensors[0]=-'A';                        //accelerometer request
                    command_sensors[1]=-'N';                        //proximity sensors request
                    command_sensors[2]=-'O';                        //ambient light request
                    command_sensors[3]=-0x0C;                       //microphones request (4 x mic)
                    command_sensors[4]=-'b';                        // Battery raw value.
                    command_sensors[5]=-'g';                        // Gyro raw value.
                    command_sensors[6]=-0x0D;                       // ToF sensor value.
                    command_sensors[7]=-0x0B;                       // User button state.
                    command_sensors[8]=-0x0E;                       // Micro sd state.
                    command_sensors[9]=0;                           //binary command ending
                }

                //send all the request in one shot
                serialPort->write(command_sensors, bytesToSendSensors);
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                m_timer.start(TIMEOUT_MS);
                std::cerr << "Request binary sensors" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
                read_state = 2;
            } else if(getCameraData) {

                headerReceived=false;
                imgReceived=false;
                wrongAnswer=true;

                //send command
                //comm->flush();
                serialPort->write(command_cam, 2);
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                m_timer.start(TIMEOUT_MS);

                headerReceived=true;
                imgReceived=true;
                wrongAnswer=false;
                bytesToRead = pixNum+3;
                read_state = 3;
                std::cerr << "Request camera" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
            }
            break;

        case 2:
            packet_index = 0;
            memset(RxBuffer, 0x0, RX_BUFF_SIZE);
            if(reading_ascii_data) {
                break;
            }
            if(cmd_list.size() > 0) { // Start sending async commands if needed
                sending_async_commands = true;
                serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());              
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                //std::cerr << "sending = ";
                //for(int i=0; i<cmd_list.front().length(); i++) {
                //    std::cerr << int(cmd_list.front()[i]) << ",";
                //}
                //std::cerr << std::endl;
                m_timer.start(TIMEOUT_MS);
                cmd_list.erase(cmd_list.begin());
                cmd_list_len.erase(cmd_list_len.begin());
                bytesToRead = cmd_list_answer_len.front();
                std::cerr << "Request async (" << int(cmd_list.front()[0]) << ")" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
                if(cmd_list_need_answer.front()) {
                    read_state = 4;
                }
                //read_state = 5; // Because it happens that "readyRead" is called before "handleBytesWritten" and this can lead to problems. By setting 5, "readyRead" is in a wait state.

                /*
                if(cmd_list_need_answer.front()) { // The last async command need an answer
                    read_state = 4;
                    //std::cerr << "need answer" << std::endl;
                } else {
                    cmd_list_need_answer.erase(cmd_list_need_answer.begin());
                    cmd_list_answer_len.erase(cmd_list_answer_len.begin());
                    if(cmd_list.size() > 0) { // Continue sending async commands if needed
                        //std::cerr << "no need answer, send next command" << std::endl;
                        memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                        packet_index = 0;
                        serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());
                        //serialPort->flush();
                        //serialPort->waitForBytesWritten(-1);
                        //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                        //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                        m_timer.start(TIMEOUT_MS);
                        cmd_list.erase(cmd_list.begin());
                        cmd_list_len.erase(cmd_list_len.begin());
                    } else { // Re-activate sensors reading
                        //std::cerr << "restart sensors" << std::endl;
                        sending_async_commands = false;
                        memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                        packet_index = 0;
                        serialPort->write(command_sensors, bytesToSendSensors);
                        //serialPort->flush();
                        //serialPort->waitForBytesWritten(-1);
                        m_timer.start(TIMEOUT_MS);
                        if((int)asercomVer == 1) {
                            bytesToRead = 58; //12+16+16+6+2+6;
                        } else {
                            bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                        }
                        read_state = 2;
                    }
                }
                */
                break;
            }
            if(getCameraData) { // If enabled, read the camera now since we just read the sensors
                headerReceived=false;
                imgReceived=false;
                wrongAnswer=true;

                serialPort->write(command_cam, 2);
                m_timer.start(TIMEOUT_MS);

                headerReceived=true;
                imgReceived=true;
                wrongAnswer=false;
                bytesToRead = pixNum+3;
                read_state = 3;
                std::cerr << "Request camera" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
            } else if(getSensorsData) { // Otherwise read once more the sensors if enabled
                //send all the request in one shot               
                serialPort->write(command_sensors, bytesToSendSensors);
                m_timer.start(TIMEOUT_MS);
                if((int)asercomVer == 1) {
                    bytesToRead = 58; //12+16+16+6+2+6;
                } else {
                    bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                }
                read_state = 2;
                std::cerr << "Request binary sensors" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
            }
            break;

        case 3:
            packet_index = 0;
            memset(RxBuffer, 0x0, RX_BUFF_SIZE);
            if(cmd_list.size() > 0) { // Start sending async commands if needed
                sending_async_commands = true;
                serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                //std::cerr << "sending = ";
                //for(int i=0; i<cmd_list.front().length(); i++) {
                //    std::cerr << int(cmd_list.front()[i]) << ",";
                //}
                //std::cerr << std::endl;
                m_timer.start(TIMEOUT_MS);
                cmd_list.erase(cmd_list.begin());
                cmd_list_len.erase(cmd_list_len.begin());
                bytesToRead = cmd_list_answer_len.front();
                std::cerr << "Request async (" << int(cmd_list.front()[0]) << ")" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
                if(cmd_list_need_answer.front()) {
                    read_state = 4;
                }
                //read_state = 5; // Because it happens that "readyRead" is called before "handleBytesWritten" and this can lead to problems. By setting 5, "readyRead" is in a wait state.

                /*
                if(cmd_list_need_answer.front()) { // The last async command need an answer
                    read_state = 4;
                    //std::cerr << "need answer" << std::endl;
                } else {
                    cmd_list_need_answer.erase(cmd_list_need_answer.begin());
                    cmd_list_answer_len.erase(cmd_list_answer_len.begin());
                    if(cmd_list.size() > 0) { // Continue sending async commands if needed
                        //std::cerr << "no need answer, send next command" << std::endl;
                        memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                        packet_index = 0;
                        serialPort->write(cmd_list.front().c_str(), cmd_list_len.front());
                        //serialPort->flush();
                        //serialPort->waitForBytesWritten(-1);
                        //memcpy(temp_buffer, &cmd_list.front()[0], cmd_list_len.front());
                        //serialPort->write((char*)temp_buffer, cmd_list_len.front());
                        m_timer.start(TIMEOUT_MS);
                        cmd_list.erase(cmd_list.begin());
                        cmd_list_len.erase(cmd_list_len.begin());
                    } else { // Re-activate sensors reading
                        //std::cerr << "restart sensors" << std::endl;
                        sending_async_commands = false;
                        memset(RxBuffer, 0x0, RX_BUFF_SIZE);
                        packet_index = 0;
                        serialPort->write(command_sensors, bytesToSendSensors);
                        //serialPort->flush();
                        //serialPort->waitForBytesWritten(-1);
                        m_timer.start(TIMEOUT_MS);
                        if((int)asercomVer == 1) {
                            bytesToRead = 58; //12+16+16+6+2+6;
                        } else {
                            bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                        }
                        read_state = 2;
                    }
                }
                */
                break;
            }
            if(getSensorsData) { // If enaled, read the sensors now since we just read the camera
                //send all the request in one shot
                serialPort->write(command_sensors, bytesToSendSensors);
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                m_timer.start(TIMEOUT_MS);
                if((int)asercomVer == 1) {
                    bytesToRead = 58; //12+16+16+6+2+6;
                } else {
                    bytesToRead = 64; //12+16+16+8+2+6+2+1+1;
                }
                read_state = 2;
                std::cerr << "Request binary sensors" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
            } else if(getCameraData) { // Otherwise read once more the camera if enabled
                headerReceived=false;
                imgReceived=false;
                wrongAnswer=true;

                serialPort->write(command_cam, 2);
                //serialPort->flush();
                //serialPort->waitForBytesWritten(-1);
                m_timer.start(TIMEOUT_MS);

                headerReceived=true;
                imgReceived=true;
                wrongAnswer=false;
                bytesToRead = pixNum+3;
                read_state = 3;
                std::cerr << "Request camera" << std::endl;
                std::cerr << "Waiting for " << (int)bytesToRead << " bytes" << std::endl;
            }
            break;

        case 4:
            break;

        case 5: // Reading ASCII sensors.
        case 6:
            break;

        default:
            std::cerr << "Uknown read_state (2)" << std::endl;
            break;

    }

}

void CommThread::closeCommunication(bool reconnect_flag, bool from_btn_disconnect) {
    if(serialPort->isOpen()) {
        serialPort->clear(QSerialPort::AllDirections);
        serialPort->close();
    }
    m_timer.stop();
    if(reconnect_flag) {
        emit reconnect(); // Call CommThread->initConnection()
    } else {
        if(!from_btn_disconnect) {
            emit portClosed(); // Call EpuckMonitor->disconnect() to "disable" interface
        }
    }
}

void CommThread::getImg(unsigned char *img) {
    memcpy(img, &imgBuffer[3], IMAGE_BUFF_SIZE-3);
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

    memset(command, 0x0, 20);
    sprintf(command,"J,%d,%d,%d,%d\r", type, width, height, zoom);

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, strlen(command));
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,100000);
    comm->FlushCommPort();
#else
//    serialPort->write(command, strlen(command));
//    if(serialPort->waitForReadyRead(READ_TIMEOUT_MS)) {
//        bytes=serialPort->read(RxBuffer, 3); //response is j\r\n
//    } else {
//        bytes=0;
//    }
#endif

    return;
}

void CommThread::goForward() {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(async_command, 0x0, 20);
    sprintf(async_command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    cmd_list.push_back(std::string(async_command, 6));
    cmd_list_len.push_back(6);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::goBackward() {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(async_command, 0x0, 20);
    sprintf(async_command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    cmd_list.push_back(std::string(async_command, 6));
    cmd_list_len.push_back(6);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::goLeft() {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(async_command, 0x0, 20);
    sprintf(async_command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    cmd_list.push_back(std::string(async_command, 6));
    cmd_list_len.push_back(6);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::goRight() {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(async_command, 0x0, 20);
    sprintf(async_command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    cmd_list.push_back(std::string(async_command, 6));
    cmd_list_len.push_back(6);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::stopMotors() {

    int speed_left = 0;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = 0;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(async_command, 0x0, 20);
    sprintf(async_command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);
    cmd_list.push_back(std::string(async_command, 6));
    cmd_list_len.push_back(6);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led0Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 0, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 0, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led1Slot(int state) {
    memset(async_command, 0x0, 20);
    int num_bytes = 0;
    rgbLedState[0] = state;

    if(state == Qt::Checked) {
        rgbLedValue[0] = rgbLedDesiredValue[0];
        rgbLedValue[1] = rgbLedDesiredValue[1];
        rgbLedValue[2] = rgbLedDesiredValue[2];
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 1, 1, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    } else {
        rgbLedValue[0] = 0;
        rgbLedValue[1] = 0;
        rgbLedValue[2] = 0;
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 1, 0, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    }

    cmd_list.push_back(std::string(async_command, num_bytes));
    cmd_list_len.push_back(num_bytes);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);
    return;
}

void CommThread::led2Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 2, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 2, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led3Slot(int state) {
    memset(async_command, 0x0, 20);
    int num_bytes = 0;
    rgbLedState[1] = state;

    if(state == Qt::Checked) {
        rgbLedValue[3] = rgbLedDesiredValue[0];
        rgbLedValue[4] = rgbLedDesiredValue[1];
        rgbLedValue[5] = rgbLedDesiredValue[2];
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 3, 1, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    } else {
        rgbLedValue[3] = 0;
        rgbLedValue[4] = 0;
        rgbLedValue[5] = 0;
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 3, 0, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    }

    cmd_list.push_back(std::string(async_command, num_bytes));
    cmd_list_len.push_back(num_bytes);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led4Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 4, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 4, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led5Slot(int state) {
    memset(async_command, 0x0, 20);
    int num_bytes = 0;
    rgbLedState[2] = state;

    if(state == Qt::Checked) {
        rgbLedValue[6] = rgbLedDesiredValue[0];
        rgbLedValue[7] = rgbLedDesiredValue[1];
        rgbLedValue[8] = rgbLedDesiredValue[2];
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 5, 1, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    } else {
        rgbLedValue[6] = 0;
        rgbLedValue[7] = 0;
        rgbLedValue[8] = 0;
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 5, 0, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    }

    cmd_list.push_back(std::string(async_command, num_bytes));
    cmd_list_len.push_back(num_bytes);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led6Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 6, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 6, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led7Slot(int state) {
    memset(async_command, 0x0, 20);
    int num_bytes = 0;
    rgbLedState[3] = state;

    if(state == Qt::Checked) {
        rgbLedValue[9] = rgbLedDesiredValue[0];
        rgbLedValue[10] = rgbLedDesiredValue[1];
        rgbLedValue[11] = rgbLedDesiredValue[2];
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 7, 1, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    } else {
        rgbLedValue[9] = 0;
        rgbLedValue[10] = 0;
        rgbLedValue[11] = 0;
        if((int)asercomVer == 1) {
            num_bytes = 4;
            sprintf(async_command, "%c%c%c%c",-'L', 7, 0, 0);
        } else {
            num_bytes = 13;
            sprintf(async_command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedValue[0], rgbLedValue[1], rgbLedValue[2], rgbLedValue[3], rgbLedValue[4], rgbLedValue[5], rgbLedValue[6], rgbLedValue[7], rgbLedValue[8], rgbLedValue[9], rgbLedValue[10], rgbLedValue[11]);
        }
    }

    cmd_list.push_back(std::string(async_command, num_bytes));
    cmd_list_len.push_back(num_bytes);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led8Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 8, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 8, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::led9Slot(int state) {

    memset(async_command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(async_command, "%c%c%c%c",-'L', 9, 1, 0);
    } else {
        sprintf(async_command, "%c%c%c%c",-'L', 9, 0, 0);
    }

    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(false);
    cmd_list_answer_len.push_back(0);

    return;
}

void CommThread::sound1Slot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '1', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::sound2Slot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '2', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::sound3Slot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '3', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::sound4Slot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '4', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::sound5Slot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '5', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::audioOffSlot() {
    sprintf(async_command, "%c%c%c%c",'T', ',', '6', '\r');
    cmd_list.push_back(std::string(async_command, 4));
    cmd_list_len.push_back(4);
    cmd_list_need_answer.push_back(true);
    cmd_list_answer_len.push_back(3);    //response is t\r\n
    return;
}

void CommThread::updateRed(int value) {
    rgbLedDesiredValue[0] = value;
    led1Slot(rgbLedState[0]);
    led3Slot(rgbLedState[1]);
    led5Slot(rgbLedState[2]);
    led7Slot(rgbLedState[3]);
}

void CommThread::updateGreen(int value) {
    rgbLedDesiredValue[1] = value;
    led1Slot(rgbLedState[0]);
    led3Slot(rgbLedState[1]);
    led5Slot(rgbLedState[2]);
    led7Slot(rgbLedState[3]);
}

void CommThread::updateBlue(int value) {
    rgbLedDesiredValue[2] = value;
    led1Slot(rgbLedState[0]);
    led3Slot(rgbLedState[1]);
    led5Slot(rgbLedState[2]);
    led7Slot(rgbLedState[3]);
}


