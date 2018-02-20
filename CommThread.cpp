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
    goUpNow = false; goDownNow = false; goLeftNow = false; goRightNow = false; stopNow = false;
    updateLed0Now = false; updateLed1Now = false; updateLed2Now = false; updateLed3Now = false; updateLed4Now = false; updateLed5Now = false; updateLed6Now = false; updateLed7Now = false; updateLed8Now = false; updateLed9Now = false;
    sound1Now = false; sound2Now = false; sound3Now = false; sound4Now = false; sound5Now = false; audioOffNow = false;
    comm = NULL;    
}

void CommThread::initConnection(char* portName) {

    char version[100];
    int err = 0;

    memset(this->portName, 0x0, 50);
    memcpy(this->portName, portName, strlen(portName));

#ifdef __WIN32__

    comm = new TCommPort();
    comm->SetCommPort(portName);
    comm->SetBaudRate(115200);
    comm->SetParity(NOPARITY);      // NOPARITY and friends are #defined in windows.h
    comm->SetByteSize(8);
    comm->SetStopBits(ONESTOPBIT);  // ONESTOPBIT is also from windows.h
    err = comm->OpenCommPort();
    if(err==-1) {
        std::cerr << "Unable to open serial port." << std::endl;
        emit cannotOpenPort("Unable to open serial port.");
        emit portClosed();
        return;
    }
    emit portOpened();

    comm->WriteBuffer((BYTE*)"\r", 1);
    comm->FlushCommPort();
    Sleep(100);
    comm->PurgeCommPort();

    sprintf(command, "V\r");
    comm->WriteBuffer((BYTE*)command, 2);
    memset(version, 0x0, 100);
    comm->ReadBytes((BYTE*)version, 36, 200000);
    emit showVersion(&version[2], 0);

#else
    comm = new SerialComm();
    err = comm->connect(portName);
    if(err==-1) {
        std::cerr << "Unable to open serial port " << portName << std::endl;
        emit cannotOpenPort("Unable to open serial port.");
        emit portClosed();
        return;
    }

    usleep(10000);
    comm->flush();
    while(comm->readData(version, 100, 200000)>0) {
        std::cerr << version << std::endl;
        memset(version, 0, 100);
        usleep(10000);
    }

    memset(command, 0x0, 20);
    sprintf(command, "\r");
    comm->writeData(command, 1, 10000);        // clear output buffer
    usleep(10000);
    comm->readData(version, 100, 200000);

    comm->flush();
    while(comm->readData(version, 100, 200000)>0) {
        std::cerr << version << std::endl;
        memset(version, 0, 100);
        usleep(10000);
    }

    sprintf(command, "V\r");
    comm->writeData(command, 2, 10000);
    usleep(10000);
    memset(version, 0x0, 100);
    comm->readData(version, 100, 200000);
    std::cerr << "version = " << version << std::endl;
    emit showVersion(&version[2], 0);

#endif

    asercomVer = version[10]-'0';
    std::cerr << "asercom version = " << (int)asercomVer << std::endl;

    if(asercomVer!=1 && asercomVer!=2) {
        std::cerr << "Unable to work with this protocol version" << std::endl;
        closeCommunication();
        emit reconnect();
        return;
    }

    emit portOpened();

/*
    //initialize camera parameters
    memset(command, 0x0, 20);
    sprintf(command,"J,%d,%d,%d,%d\r\n", type, width, height, zoom);
#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, strlen(command));
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3);
    comm->FlushCommPort();
#else
    comm->writeData(command, strlen(command), 100000);
    comm->readData(RxBuffer, 3, 300000);    //response is j\r\n
#endif

    std::cerr << "response = " << RxBuffer << std::endl;
*/

    getSensorsData = true;
    getCameraData = false;

    start();
}

CommThread::~CommThread() {

}

void CommThread::closeCommunication() {
#ifdef __WIN32__
    if(comm!=NULL) {
        comm->FlushCommPort();
        comm->PurgeCommPort();
        comm->CloseCommPort();
        comm=NULL;
    }
#else
    if(comm!=NULL) {
        comm->disconnect();
        comm=NULL;
    }
#endif
    emit portClosed();
}

void CommThread::run() {

    char msg[50];
    int bytes=0;
    long  mantis=0;
    short  exp=0;
    float flt=0;
    uint8_t bytesToRead = 0;
    uint8_t bytesToSend = 0;
    bool reconnectFlag = false;

    abortThread = false;

    while(!abortThread) {

        msleep(20); // Keep main thread (GUI) responsive.

        if(getSensorsData) {

            if((int)asercomVer == 1) {
                bytesToSend = 7;
                command[0]=-'A';                        //accelerometer request
                command[1]=-'N';                        //proximity sensors request
                command[2]=-'O';                        //ambient light request
                command[3]=-'u';                        //microphones request (3 x mic)
                command[4]=-'b';                        // Battery raw value.
                command[5]=-'g';                        // Gyro raw value.
                command[6]=0;                           //binary command ending
            } else {
                bytesToSend = 10;
                command[0]=-'A';                        //accelerometer request
                command[1]=-'N';                        //proximity sensors request
                command[2]=-'O';                        //ambient light request
                command[3]=-0x0C;                       //microphones request (4 x mic)
                command[4]=-'b';                        // Battery raw value.
                command[5]=-'g';                        // Gyro raw value.
                command[6]=-0x0D;                       // ToF sensor value.
                command[7]=-0x0B;                       // User button state.
                command[8]=-0x0E;                       // Micro sd state.
                command[9]=0;                           //binary command ending
            }

            //send all the request in one shot
        #ifdef __WIN32__
            comm->PurgeCommPort();
            comm->WriteBuffer((BYTE*)command, bytesToSend);
            comm->FlushCommPort();
            Sleep(100);
        #else
            comm->flush();
            bytes=comm->writeData(command, bytesToSend, 1000000);
        #endif

            //ACCELEROMETER DATA
/*
            memset(RxBuffer, 0x0, 45);
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,6,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,6,1000000);
        #endif
            if(bytes<6) {
                sprintf(msg, "Accelerometer: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                orientation=0.0;
            } else {
                acc_x = RxBuffer[0]+RxBuffer[1]*256;
                acc_y = RxBuffer[2]+RxBuffer[3]*256;
                acc_z = RxBuffer[4]+RxBuffer[5]*256;

            }

            std::cerr << "acc x = " << acc_x << std::endl;
            std::cerr << "acc y = " << acc_y << std::endl;
            std::cerr << "acc z = " << acc_z << std::endl;
            std::cerr << "roll = " << atan2((double)acc_z,(double)acc_x)/M_PI*180.0 << std::endl;
            std::cerr << "pitch = " << atan2((double)acc_z,(double)acc_y)/M_PI*180.0 << std::endl;
*/


            //read acceleration
            memset(RxBuffer, 0x0, 45);            
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,4,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,4,1000000);
        #endif
            mantis=0;
            exp=0;
            flt=0;
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<4) {	//data not received completely
                sprintf(msg, "Acceleration: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                acceleration=0.0;
            } else {
                mantis = (RxBuffer[0] & 0xff) + ((RxBuffer[1] & 0xffl) << 8) + (((RxBuffer[2] &0x7fl) | 0x80) << 16);
                exp = (RxBuffer[3] & 0x7f) * 2 + ((RxBuffer[2] & 0x80) ? 1 : 0);
                if (RxBuffer[3] & 0x80) {
                    mantis = -mantis;
                }
                flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                acceleration=flt;
            }            

            //read orientation
            memset(RxBuffer, 0x0, 45);            
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,4,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,4,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<4) {
                sprintf(msg, "Orientation: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                orientation=0.0;
            } else {
                mantis = (RxBuffer[0] & 0xff) + ((RxBuffer[1] & 0xffl) << 8) + (((RxBuffer[2] &0x7fl) | 0x80) << 16);
                exp = (RxBuffer[3] & 0x7f) * 2 + ((RxBuffer[2] & 0x80) ? 1 : 0);
                if (RxBuffer[3] & 0x80)
                    mantis = -mantis;
                flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                orientation=flt;
                if (orientation < 0.0 )
                    orientation=0.0;
                if (orientation > 360.0 )
                    orientation=360.0;
            }            

            //std::cout << "orientation = " << orientation << std::endl;

            //read inclination
            memset(RxBuffer, 0x0, 45);
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,4,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,4,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<4) {
                sprintf(msg, "Inclination: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                inclination=0.0;
            } else {
                mantis = (RxBuffer[0] & 0xff) + ((RxBuffer[1] & 0xffl) << 8) + (((RxBuffer[2] &0x7fl) | 0x80) << 16);
                exp = (RxBuffer[3] & 0x7f) * 2 + ((RxBuffer[2] & 0x80) ? 1 : 0);
                if (RxBuffer[3] & 0x80)
                    mantis = -mantis;
                flt = (mantis || exp) ? ((float) ldexp (mantis, (exp - 127 - 23))): 0;
                inclination=flt;
                if (inclination < 0.0 )
                    inclination=0.0;
                if (inclination > 180.0 )
                    inclination=180.0;
            }
            //std::cout << "inclination = " << inclination << std::endl;


            //PROXIMITY SENSORS DATA
            memset(RxBuffer, 0x0, 45);
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,16,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,16,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<16) {
                sprintf(msg, "IRs: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                ir0=0;
                ir1=0;
                ir2=0;
                ir3=0;
                ir4=0;
                ir5=0;
                ir6=0;
                ir7=0;
            } else {
                ir0 = (RxBuffer[0]+RxBuffer[1]*256>2000)?2000:RxBuffer[0]+RxBuffer[1]*256;
                ir1 = (RxBuffer[2]+RxBuffer[3]*256>2000)?2000:RxBuffer[2]+RxBuffer[3]*256;
                ir2 = (RxBuffer[4]+RxBuffer[5]*256>2000)?2000:RxBuffer[4]+RxBuffer[5]*256;
                ir3 = (RxBuffer[6]+RxBuffer[7]*256>2000)?2000:RxBuffer[6]+RxBuffer[7]*256;
                ir4 = (RxBuffer[8]+RxBuffer[9]*256>2000)?2000:RxBuffer[8]+RxBuffer[9]*256;
                ir5 = (RxBuffer[10]+RxBuffer[11]*256>2000)?2000:RxBuffer[10]+RxBuffer[11]*256;
                ir6 = (RxBuffer[12]+RxBuffer[13]*256>2000)?2000:RxBuffer[12]+RxBuffer[13]*256;
                ir7 = (RxBuffer[14]+RxBuffer[15]*256>2000)?2000:RxBuffer[14]+RxBuffer[15]*256;
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
            }

            //LIGHT SENSORS DATA
            memset(RxBuffer, 0x0, 45);
            lightAvg=0;
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,16,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,16,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<16) {
                sprintf(msg, "Light: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                lightAvg=0;
            } else {
                lightAvg += (RxBuffer[0]+RxBuffer[1]*256);
                lightAvg += (RxBuffer[2]+RxBuffer[3]*256);
                lightAvg += (RxBuffer[4]+RxBuffer[5]*256);
                lightAvg += (RxBuffer[6]+RxBuffer[7]*256);
                lightAvg += (RxBuffer[8]+RxBuffer[9]*256);
                lightAvg += (RxBuffer[10]+RxBuffer[11]*256);
                lightAvg += (RxBuffer[12]+RxBuffer[13]*256);
                lightAvg += (RxBuffer[14]+RxBuffer[15]*256);
                lightAvg = (int) (lightAvg/8);
                lightAvg = (lightAvg>4000)?4000:lightAvg;
                if(lightAvg<0) {
                    lightAvg=0;
                }
            }

            //MICROPHONE DATA
            memset(RxBuffer, 0x0, 45);
            if((int)asercomVer == 1) {
                bytesToRead = 6;
            } else {
                bytesToRead = 8;
            }
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,bytesToRead,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,bytesToRead,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<bytesToRead) {
                sprintf(msg, "MICs: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                micVolume[0]=micVolume[1]=micVolume[2]=micVolume[3]=0;
            } else {
                micVolume[0] = ((uint8_t)RxBuffer[0]+(uint8_t)RxBuffer[1]*256>1500)?1500:((uint8_t)RxBuffer[0]+(uint8_t)RxBuffer[1]*256);
                micVolume[1] = ((uint8_t)RxBuffer[2]+(uint8_t)RxBuffer[3]*256>1500)?1500:((uint8_t)RxBuffer[2]+(uint8_t)RxBuffer[3]*256);
                micVolume[2] = ((uint8_t)RxBuffer[4]+(uint8_t)RxBuffer[5]*256>1500)?1500:((uint8_t)RxBuffer[4]+(uint8_t)RxBuffer[5]*256);
                if((int)asercomVer == 1) {
                    micVolume[3] = 0;
                } else {
                    micVolume[3] = ((uint8_t)RxBuffer[6]+(uint8_t)RxBuffer[7]*256>1500)?1500:((uint8_t)RxBuffer[6]+(uint8_t)RxBuffer[7]*256);
                }
            }

            //BATTERY DATA
            memset(RxBuffer, 0x0, 45);
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,2,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,2,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<2) {
                sprintf(msg, "Battery: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                batteryRaw=0;
                memset(batteryRawStr, 0x0, 5);
                sprintf(batteryRawStr, "%d", 0);
            } else {
                batteryRaw = (uint8_t)RxBuffer[0]+(uint8_t)RxBuffer[1]*256;
                memset(batteryRawStr, 0x0, 5);
                sprintf(batteryRawStr, "%d", batteryRaw);
            }

            //GYRO DATA
            memset(RxBuffer, 0x0, 45);
        #ifdef __WIN32__
            bytes=comm->ReadBytes((BYTE*)RxBuffer,6,1000000);
        #else
            bytes=comm->readData((char*)RxBuffer,6,1000000);
        #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<6) {
                sprintf(msg, "Gyro: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                gyroRaw[0]=0;
                gyroRaw[1]=0;
                gyroRaw[2]=0;
            } else {
                gyroRaw[0] = RxBuffer[0]+RxBuffer[1]*256;
                gyroRaw[1] = RxBuffer[2]+RxBuffer[3]*256;
                gyroRaw[2] = RxBuffer[4]+RxBuffer[5]*256;
            }


            if((int)asercomVer == 2) {

                // ToF DATA
                memset(RxBuffer, 0x0, 45);
            #ifdef __WIN32__
                bytes=comm->ReadBytes((BYTE*)RxBuffer,2,1000000);
            #else
                bytes=comm->readData((char*)RxBuffer,2,1000000);
            #endif
                if(bytes == 0) {
                    reconnectFlag = true;
                    break;
                } else if(bytes<2) {
                    sprintf(msg, "ToF: only %d bytes red", bytes);
                    std::cerr << msg << std::endl;
                    distanceCm = 0;
                    memset(distanceCmStr, 0x0, 5);
                    sprintf(distanceCmStr, "%d", 0);
                } else {
                    distanceCm = (uint16_t)(((uint8_t)RxBuffer[1]<<8)|((uint8_t)RxBuffer[0]))/10;
                    memset(distanceCmStr, 0x0, 5);
                    sprintf(distanceCmStr, "%d", (distanceCm>200)?200:distanceCm);
                }

                // Button state.
                memset(RxBuffer, 0x0, 45);
            #ifdef __WIN32__
                bytes=comm->ReadBytes((BYTE*)RxBuffer,1,1000000);
            #else
                bytes=comm->readData((char*)RxBuffer,1,1000000);
            #endif
                if(bytes == 0) {
                    reconnectFlag = true;
                    break;
                } else if(bytes<1) {
                    sprintf(msg, "Button: only %d bytes red", bytes);
                    std::cerr << msg << std::endl;
                } else {
                    buttonState = RxBuffer[0];
                }
				
                // Micro sd state.
                memset(RxBuffer, 0x0, 45);
            #ifdef __WIN32__
                bytes=comm->ReadBytes((BYTE*)RxBuffer,1,1000000);
            #else
                bytes=comm->readData((char*)RxBuffer,1,1000000);
            #endif
                if(bytes == 0) {
                    reconnectFlag = true;
                    break;
                } else if(bytes<1) {
                    sprintf(msg, "Micro sd: only %d bytes red", bytes);
                    std::cerr << msg << std::endl;
                } else {
                    microSdState = RxBuffer[0];
                }
            }

            emit newBinaryData();

            //SELECTOR DATA (ascii mode)
            memset(RxBuffer, 0x0, 45);
            int selector;
            #ifdef __WIN32__
                    comm->PurgeCommPort();
                    comm->WriteBuffer((BYTE*)"C\r", 2);
                    comm->FlushCommPort();
                    Sleep(100);
                    bytes = comm->ReadBytes((BYTE*)RxBuffer,5,1000000);
            #else
                    comm->flush();
                    comm->writeData("C\r",2,1000000);
                    bytes = comm->readData((char*)RxBuffer, 5, 1000000);
            #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<5) {
                sprintf(msg, "Selector: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                    //comm->discard(5-bytes);
            } else {
                memset(selectorStr, 0x0, 3);
                sscanf((char*)RxBuffer,"c,%d\r\n",&selector);
                sprintf(selectorStr, "%d", selector);
            }

            //IR DATA (ascii mode)
            memset(RxBuffer, 0x0, 45);
            int ir_check, ir_address, ir_data;
            #ifdef __WIN32__
                    comm->PurgeCommPort();
                    comm->WriteBuffer((BYTE*)"G\r", 2);
                    comm->FlushCommPort();
                    Sleep(100);
                    bytes = comm->ReadBytes((BYTE*)RxBuffer,45,1000000);
            #else
                    comm->flush();
                    comm->writeData("G\r",2,1000000);
                    bytes = comm->readData((char*)RxBuffer, 45, 10000000);
            #endif
            if(bytes == 0) {
                reconnectFlag = true;
                break;
            } else if(bytes<45) {
                sprintf(msg, "IRs data: only %d bytes red", bytes);
                std::cerr << msg << std::endl;
                    //comm->discard(45-bytes);
            } else {
                memset(irCheckStr, 0x0, 8);
                memset(irAddressStr, 0x0, 8);
                memset(irDataStr, 0x0, 8);
                sscanf((char*)RxBuffer,"g IR check : 0x%x, address : 0x%x, data : 0x%x\r\n",&ir_check,&ir_address,&ir_data);
                sprintf(irCheckStr, "%x", ir_check);
                sprintf(irAddressStr, "%x", ir_address);
                sprintf(irDataStr, "%x", ir_data);
            }

            emit newAsciiData();

        }   // end get sensor data

        if(getCameraData) {

            headerReceived=false;
            imgReceived=false;
            wrongAnswer=true;

            command[0]=-'I';      //binary image receiving command
            command[1]= 0;        //end command

            //send command
        #ifdef __WIN32__
            comm->PurgeCommPort();
            comm->WriteBuffer((BYTE*)command, 2);
            comm->FlushCommPort();
            Sleep(100);
        #else
            comm->flush();
            bytes = comm->writeData(command,2,6000);
        #endif

            headerReceived=true;
            imgReceived=true;
            wrongAnswer=false;

            //read image header
        #ifdef __WIN32__
            bytes = comm->ReadBytes((BYTE*)imgBuffer,pixNum+3,1000000);
        #else
            bytes = comm->readData((char*)imgBuffer, pixNum+3, 10000000);
        #endif

            emit newImage();
        }   // end get camera data


        if(goUpNow) {
            goUpNow = false;
            sendGoUp(motorSpeed);
            usleep(20000);
        }

        if(goDownNow) {
            goDownNow = false;
            sendGoDown(motorSpeed);
            usleep(20000);
        }

        if(goLeftNow) {
            goLeftNow = false;
            sendGoLeft(motorSpeed);
            usleep(20000);
        }

        if(goRightNow) {
            goRightNow = false;
            sendGoRight(motorSpeed);
            usleep(20000);
        }

        if(stopNow) {
            stopNow = false;
            sendStop();
            usleep(20000);
        }

        if(updateLed0Now) {
            updateLed0Now = false;
            sendUpdateLed0(stateLed0);
            usleep(20000);
        }

        if(updateLed1Now) {
            updateLed1Now = false;
            sendUpdateLed1(stateLed1);
            usleep(20000);
        }

        if(updateLed2Now) {
            updateLed2Now = false;
            sendUpdateLed2(stateLed2);
            usleep(20000);
        }

        if(updateLed3Now) {
            updateLed3Now = false;
            sendUpdateLed3(stateLed3);
            usleep(20000);
        }

        if(updateLed4Now) {
            updateLed4Now = false;
            sendUpdateLed4(stateLed4);
            usleep(20000);
        }

        if(updateLed5Now) {
            updateLed5Now = false;
            sendUpdateLed5(stateLed5);
            usleep(20000);
        }

        if(updateLed6Now) {
            updateLed6Now = false;
            sendUpdateLed6(stateLed6);
            usleep(20000);
        }

        if(updateLed7Now) {
            updateLed7Now = false;
            sendUpdateLed7(stateLed7);
            usleep(20000);
        }

        if(updateLed8Now) {
            updateLed8Now = false;
            sendUpdateLed8(stateLed8);
            usleep(20000);
        }

        if(updateLed9Now) {
            updateLed9Now = false;
            sendUpdateLed9(stateLed9);
            usleep(20000);
        }

        if(sound1Now) {
            sound1Now = false;
            sendSound1();
        }

        if(sound2Now) {
            sound2Now = false;
            sendSound2();
        }

        if(sound3Now) {
            sound3Now = false;
            sendSound3();
        }

        if(sound4Now) {
            sound4Now = false;
            sendSound4();
        }

        if(sound5Now) {
            sound5Now = false;
            sendSound5();
        }

        if(audioOffNow) {
            audioOffNow = false;
            sendAudioOff();
        }

    }   // end infinite loop

    closeCommunication();
    msleep(300);
    if(reconnectFlag) {
        emit reconnect();
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
    comm->writeData(command, strlen(command), 100000);
    comm->readData(RxBuffer, 3, 100000);    //response is j\r\n
#endif

    return;
}

void CommThread::sendGoUp(int motorSpeed) {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);


#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 6);
    comm->FlushCommPort();
#else
    int bytes = comm->writeData(command, 6, 500000);
#endif

    return;
}

void CommThread::sendGoDown(int motorSpeed) {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);


#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 6);
    comm->FlushCommPort();
#else
    comm->writeData(command, 6, 20000);
#endif

    return;
}

void CommThread::sendGoLeft(int motorSpeed) {

    int speed_left = -motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);


#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 6);
    comm->FlushCommPort();
#else
    comm->writeData(command, 6, 20000);
#endif

    return;
}

void CommThread::sendGoRight(int motorSpeed) {

    int speed_left = motorSpeed;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = -motorSpeed;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 6);
    comm->FlushCommPort();
#else
    comm->writeData(command, 6, 20000);
#endif

    return;
}

void CommThread::sendStop() {

    int speed_left = 0;
    char high_left = (speed_left>>8) & 0xFF;
    char low_left = speed_left & 0xFF;
    int speed_right = 0;
    char high_right = (speed_right>>8) & 0xFF;
    char low_right = speed_right & 0xFF;

    memset(command, 0x0, 20);
    sprintf(command, "%c%c%c%c%c%c",-'D', low_left, high_left, low_right, high_right,0);

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 6);
    comm->FlushCommPort();
#else
    comm->writeData(command, 6, 20000);
#endif

    return;
}

void CommThread::sendUpdateLed0(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 0, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 0, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendUpdateLed1(int state) {
    uint8_t bytesToSend = 0;
    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        rgbLedState[0] = rgbLedValue[0];
        rgbLedState[1] = rgbLedValue[1];
        rgbLedState[2] = rgbLedValue[2];
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 1, 1, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    } else {
        rgbLedState[0] = 0;
        rgbLedState[1] = 0;
        rgbLedState[2] = 0;
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 1, 0, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    }


#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, bytesToSend);
    comm->FlushCommPort();
#else
    comm->writeData(command, bytesToSend, 12000);
#endif

    return;
}

void CommThread::sendUpdateLed2(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 2, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 2, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendUpdateLed3(int state) {
    uint8_t bytesToSend = 0;
    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        rgbLedState[3] = rgbLedValue[0];
        rgbLedState[4] = rgbLedValue[1];
        rgbLedState[5] = rgbLedValue[2];
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 3, 1, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    } else {
        rgbLedState[3] = 0;
        rgbLedState[4] = 0;
        rgbLedState[5] = 0;
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 3, 0, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, bytesToSend);
    comm->FlushCommPort();
#else
    comm->writeData(command, bytesToSend, 12000);
#endif

    return;
}

void CommThread::sendUpdateLed4(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 4, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 4, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendUpdateLed5(int state) {
    uint8_t bytesToSend = 0;
    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        rgbLedState[6] = rgbLedValue[0];
        rgbLedState[7] = rgbLedValue[1];
        rgbLedState[8] = rgbLedValue[2];
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 5, 1, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    } else {
        rgbLedState[6] = 0;
        rgbLedState[7] = 0;
        rgbLedState[8] = 0;
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 5, 0, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, bytesToSend);
    comm->FlushCommPort();
#else
    comm->writeData(command, bytesToSend, 12000);
#endif

    return;
}

void CommThread::sendUpdateLed6(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 6, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 6, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendUpdateLed7(int state) {
    uint8_t bytesToSend = 0;
    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        rgbLedState[9] = rgbLedValue[0];
        rgbLedState[10] = rgbLedValue[1];
        rgbLedState[11] = rgbLedValue[2];
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 7, 1, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    } else {
        rgbLedState[9] = 0;
        rgbLedState[10] = 0;
        rgbLedState[11] = 0;
        if((int)asercomVer == 1) {
            bytesToSend = 4;
            sprintf(command, "%c%c%c%c",-'L', 7, 0, 0);
        } else {
            bytesToSend = 13;
            sprintf(command, "%c%c%c%c%c%c%c%c%c%c%c%c%c",-0x0A, rgbLedState[0], rgbLedState[1], rgbLedState[2], rgbLedState[3], rgbLedState[4], rgbLedState[5], rgbLedState[6], rgbLedState[7], rgbLedState[8], rgbLedState[9], rgbLedState[10], rgbLedState[11]);
        }
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, bytesToSend);
    comm->FlushCommPort();
#else
    comm->writeData(command, bytesToSend, 12000);
#endif

    return;
}

void CommThread::sendUpdateLed8(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 8, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 8, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendUpdateLed9(int state) {

    memset(command, 0x0, 20);

    if(state == Qt::Checked) {
        sprintf(command, "%c%c%c%c",-'L', 9, 1, 0);
    } else {
        sprintf(command, "%c%c%c%c",-'L', 9, 0, 0);
    }

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)command, 4);
    comm->FlushCommPort();
#else
    comm->writeData(command, 4, 12000);

#endif

    return;
}

void CommThread::sendSound1() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,1\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,1\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif       

    return;
}

void CommThread::sendSound2() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,2\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,2\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif

    return;
}

void CommThread::sendSound3() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,3\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,3\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif

    return;
}

void CommThread::sendSound4() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,4\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,4\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif

    return;
}

void CommThread::sendSound5() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,5\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,5\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif

    return;
}

void CommThread::sendAudioOff() {

#ifdef __WIN32__
    comm->PurgeCommPort();
    comm->WriteBuffer((BYTE*)"T,6\r", 4);
    comm->FlushCommPort();
    comm->ReadBytes((BYTE*)RxBuffer,3,200000);
#else
    comm->writeData("T,6\r",4,12000);
    comm->readData(RxBuffer,3,200000);
#endif

    return;
}

void CommThread::goUpSlot(int speed) {
    goUpNow = true;
    motorSpeed = speed;
}

void CommThread::goDownSlot(int speed) {
    goDownNow = true;
    motorSpeed = speed;
}

void CommThread::goLeftSlot(int speed) {
    goLeftNow = true;
    motorSpeed = speed;
}

void CommThread::goRightSlot(int speed) {
    goRightNow = true;
    motorSpeed = speed;
}

void CommThread::stopSlot() {
    stopNow = true;
}

void CommThread::led0Slot(int state) {
    stateLed0 = state;
    updateLed0Now = true;
}

void CommThread::led1Slot(int state) {
    stateLed1 = state;
    updateLed1Now = true;
}

void CommThread::led2Slot(int state) {
    stateLed2 = state;
    updateLed2Now = true;
}

void CommThread::led3Slot(int state) {
    stateLed3 = state;
    updateLed3Now = true;
}

void CommThread::led4Slot(int state) {
    stateLed4 = state;
    updateLed4Now = true;
}

void CommThread::led5Slot(int state) {
    stateLed5 = state;
    updateLed5Now = true;
}

void CommThread::led6Slot(int state) {
    stateLed6 = state;
    updateLed6Now = true;
}

void CommThread::led7Slot(int state) {
    stateLed7 = state;
    updateLed7Now = true;
}

void CommThread::led8Slot(int state) {
    stateLed8 = state;
    updateLed8Now = true;
}

void CommThread::led9Slot(int state) {
    stateLed9 = state;
    updateLed9Now = true;
}

void CommThread::sound1Slot() {
    sound1Now = true;
}

void CommThread::sound2Slot() {
    sound2Now = true;
}

void CommThread::sound3Slot() {
    sound3Now = true;
}

void CommThread::sound4Slot() {
    sound4Now = true;
}

void CommThread::sound5Slot() {
    sound5Now = true;
}

void CommThread::audioOffSlot() {
    audioOffNow = true;
}

void CommThread::updateRed(int value) {
    rgbLedValue[0] = value;
    updateLed1Now = true;
    updateLed3Now = true;
    updateLed5Now = true;
    updateLed7Now = true;
}

void CommThread::updateGreen(int value) {
    rgbLedValue[1] = value;
    updateLed1Now = true;
    updateLed3Now = true;
    updateLed5Now = true;
    updateLed7Now = true;
}

void CommThread::updateBlue(int value) {
    rgbLedValue[2] = value;
    updateLed1Now = true;
    updateLed3Now = true;
    updateLed5Now = true;
    updateLed7Now = true;
}


