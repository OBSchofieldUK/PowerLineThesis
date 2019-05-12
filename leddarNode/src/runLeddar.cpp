/*
// leddarNode
Written by Oscar Bowen Schofield osbow17@student.sdu.dk

Node inspiration taken from LeddarTech SDK (support.leddartech.com)
________________________________________________________________________________________

* THIS SOFTWARE IS PROVIDED BY LEDDARTECH "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
* EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL LEDDARTECH BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.

*/
#include <iostream>
#include <iomanip>

#include "LdConnectionFactory.h"
#include "LdConnectionInfoModbus.h"
#include "LdDeviceFactory.h"
#include "LdLibModbusSerial.h"
#include "LdPropertyIds.h"
#include "LdResultEchoes.h"
#include "LdSensorVu8.h"
#include "LdSensorVu8Modbus.h"
#include "LdSpiFTDI.h"
#include "LdSensorOneModbus.h"
#include "LdSensorM16Modbus.h"
#include "LdLibUsb.h"
#include "LdProtocolLeddartechUSB.h"

#include "LtExceptions.h"
#include "LtKeyboardUtils.h"
#include "LtStringUtils.h"
#include "LtTimeUtils.h"

#include "ros/ros.h"

#include <inspec_msg/lidardat.h>
#include <inspec_msg/head.h>

#include <std_msgs/Float64MultiArray.h>

#define LEDDAR_DEV "/dev/ttyACM0"           //Location of LEDDAR VU8
#define LEDDAR_MODBUS 1                     //Leave as 1!

int count = 0;

int main( int argc, char *argv[] )
{
    //ROS Setup
    ros::init(argc, argv,"leddarNode");
    ros::NodeHandle n;
    ros::Publisher lidarPub = n.advertise<inspec_msg::lidardat>("/inspec/daq/lidarDat", 10);
    ros::Rate loop_rate(20);

    // Lidar Pointer Setup
    LeddarDevice::LdSensor              *lSensor = nullptr;
    LeddarConnection::LdConnection      *lConnection = nullptr;
    LeddarConnection::LdConnectionInfo  *lConnectionInfo = nullptr;

    std::vector<LeddarConnection::LdConnectionInfo *> lConnList =  LeddarConnection::LdLibModbusSerial::GetDeviceList();
    

    for (size_t i = 0; i < lConnList.size(); i++)
    {   
        if (lConnList[i]->GetDisplayName() == LEDDAR_DEV){
            lConnectionInfo = lConnList[i];
            break;
        }
    }
    if( lConnectionInfo == nullptr){
        std::cout << "Error: no device found!" << std::endl;
        return 0;
    }else{
        dynamic_cast<LeddarConnection::LdConnectionInfoModbus *>(lConnectionInfo)->SetModbusAddr((uint8_t) LEDDAR_MODBUS);
        lConnection = dynamic_cast<LeddarConnection::LdConnectionUniversal *>(LeddarConnection::LdConnectionFactory::CreateConnection(lConnectionInfo));
        lSensor = LeddarDevice::LdDeviceFactory::CreateSensor(lConnection);
    }

    lSensor->GetConstants();
    lSensor->GetConfig();
    lSensor->SetDataMask(LeddarDevice::LdSensor::DM_ALL);   //Set mask to read data
    
    // display info on screen
    while(!ros::isShuttingDown()){

        inspec_msg::lidardat msg;
        msg.distance.resize(8);
        msg.amplitude.resize(8);

        bool lNewData = lSensor->GetData();
        if( lNewData ){
            LeddarConnection::LdResultEchoes *lResultEchoes = lSensor->GetResultEchoes();
            uint32_t lDistanceScale = lResultEchoes->GetDistanceScale();
            uint32_t lAmplitudeScale = lResultEchoes->GetAmplitudeScale();
            lResultEchoes->Lock( LeddarConnection::B_GET );
            std::vector<LeddarConnection::LdEcho> &lEchoes = *( lResultEchoes->GetEchoes() );

            std::vector<double> Distances(8,-1.0);
            std::vector<double> Amplitudes(8,-1.0);
            // double Distances[8] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};
            // double Amplitude[8] = {-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0,-1.0};

            for( uint32_t i = 0; i < lResultEchoes->GetEchoCount(); ++i ){           
                int idx = lEchoes[i].mChannelIndex;
                Distances[idx] = (double) lEchoes[idx].mDistance / (double) lDistanceScale;
                Amplitudes[idx] = (double) lEchoes[idx].mAmplitude / (double) lAmplitudeScale;
            }
            
            inspec_msg::head msgHead;
            
            msgHead.seq = count;
            msgHead.stamp = ros::Time::now();
            
            msg.header = msgHead;
            msg.distance = Distances;
            msg.amplitude = Amplitudes;

            lResultEchoes->UnLock( LeddarConnection::B_GET );
            lidarPub.publish(msg);
            count++;

        }
        ros::spinOnce();
        loop_rate.sleep();

        // LeddarUtils::LtTimeUtils::Wait( 10 );
    }

    lSensor->SetDataMask( LeddarDevice::LdSensor::DM_NONE );
    lSensor->Disconnect();
    delete lSensor;
    
    return 0;
}


