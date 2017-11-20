/*
* VLPCommunication.cpp
* Manage communication between velodyne sensor with UDP socket
* Author: Binyamin Appelbaum
*/

#include "../include/VLPCommunication.h"

#include <iostream>
#include <map>
#include <boost/assign.hpp> // boost::assign::map_list_of
#include <boost/asio.hpp> // boost::asio::io_service
#include <boost/range/irange.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>

#define RESET_COL "\033[0m"
#define RED "\033[31m"
#define ERR(msg) printError(__func__, msg)

static const int DEGREES = 360;
static const int DISTANCE_MULT = 500;
static const int AZIMUTH_MULT = 100;
static const int SECOND_TO_MICROSECOND = 1000000;
static const unsigned long HOUR_TO_MICRO_SEC = 360 * SECOND_TO_MICROSECOND;
static const std::string PORT = "2368";
static const std::string IP_ADDRESS = "192.168.1.77";

std::map<ReturnMode, std::string> retModeToStr = boost::assign::map_list_of(_STRONGEST_, "strongest")(_LAST_, "last")(_DUAL_, "dual");
std::map<DataSource, std::string> dataSourceToStr = boost::assign::map_list_of(_HDL32E_, "HDL_32E")(_VLP16_, "VLP16");

void printError(const std::string& funcName, const std::string& message) { 
    std::cout << RED << funcName << ":: " << message << RESET_COL << std::endl;
}

VLPCommunication::VLPCommunication(const VLPConfig& vlpConfig) : m_vlpConfig(vlpConfig) {
    InitVelodyneData();
}

VLPCommunication::~VLPCommunication() {
    m_sendDataThread.interrupt();
}

VLPCommunication::VLPConfig::VLPConfig(const std::string& ipAddress, const std::string& port, Resolution horizontalResolution,
             NumOfDataChannels numOfRowsInColumn, ReturnMode returnMode, DataSource dataSource, int sensorFrequency) :
             m_ipAddress(ipAddress), m_port(port), m_horizontalResolution(horizontalResolution), m_numOfRowsInColumn(numOfRowsInColumn),
              m_returnMode(returnMode), m_dataSource(dataSource), m_sensorFrequency(sensorFrequency) {
    m_realHorizontalResolution = m_horizontalResolution / 1000.0;        
}

void VLPCommunication::InitVelodyneData() {
    int numOfColumns = (DEGREES / m_vlpConfig.m_realHorizontalResolution);
    for (int i : boost::irange(0,numOfColumns)) {
       m_velodyneData.push_back(VLPData(0, t_channel_data(), boost::posix_time::ptime()));
    }
}

void VLPCommunication::SendPacket(const VLPDataPacket& packet) const {
    using namespace boost::asio;
    char buf[sizeof(VLPDataPacket)]{};
    memcpy(buf, &packet, sizeof(packet));

    boost::asio::io_service io_service;
    ip::udp::socket socket(io_service);
    socket.open(ip::udp::v4());

    ip::udp::endpoint remote_endpoint = ip::udp::endpoint(ip::address::from_string(m_vlpConfig.m_ipAddress),
         boost::lexical_cast<int>(m_vlpConfig.m_port));
    // set the ip address of the configuration
    remote_endpoint.address(ip::address::from_string(m_vlpConfig.m_ipAddress));
    boost::system::error_code err;
    socket.send_to(buffer(buf, sizeof(packet)), remote_endpoint, 0, err);
    if (err.value() != boost::system::errc::success) {
        ERR("Failed to send packet. " + err.message());
    }
    socket.close();
}

void VLPCommunication::SendData() const {
    // transmittion frequency is the degrees*10 / <degrees range of packet>
    static const unsigned int TRANSMISSION_FREQ = (m_vlpConfig.m_sensorFrequency * DEGREES) /
                                                 (m_vlpConfig.m_realHorizontalResolution * 2*NUM_OF_VLP_DATA_BLOCKS);
    // sleep time is 1/transmission time * 1000 (milliseconds) * 1000 (to microseconds)
    static const unsigned int SLEEP_TIME = 1000000/TRANSMISSION_FREQ;
    // send data loop
   /* while (true) {
        unsigned int lastTimeStamp = 0, currTimeStamp = 0;
        for (int i = 0; i < (DEGREES / m_vlpConfig.m_realHorizontalResolution); i += 2*NUM_OF_VLP_DATA_BLOCKS) {
            VLPDataPacket packet = CreateVLPPacket(i);
             // check a case when lastTimeStamp = 3599 and currTimeStamp = 1, but currTimeStamp is in the new hour    
            if (IsPacketHasBeenSent(packet, lastTimeStamp, currTimeStamp)) {
                continue;
            }
            lastTimeStamp = currTimeStamp;
            SendPacket(packet);
            usleep(SLEEP_TIME);
            //printPacketData(packet);
        }
    }*/

    VLPDataPacket packet = CreateVLPPacket(0);
    printPacketData(packet);
}

bool VLPCommunication::IsPacketHasBeenSent(const VLPDataPacket& packet, unsigned int lastTimeStamp,/* out */ unsigned int& currTimeStamp) const {
    auto tsFunc = [](double num) -> double {return (double)num/SECOND_TO_MICROSECOND; };
    currTimeStamp = FormatBlock(packet.timeStamp, sizeof(packet.timeStamp), tsFunc);
    return currTimeStamp < lastTimeStamp;
}

bool VLPCommunication::CheckDataValidation(const VLPData& data) const {
    double angle = data.m_azimuth;
    // avoid 360 Degrees and above
    if ((angle >= DEGREES) || (angle < 0)) {
        ERR("Angle is not valid: " + std::to_string(angle));
        return false;
    }
    // check that the angle is divided with the resolution
    if ((int(1000*angle + 0.5) % int(1000*m_vlpConfig.m_realHorizontalResolution + 0.5)) != 0) {
        ERR("Angle is not valid: " + std::to_string(angle));
        return false;
    }
    // check that the data size corresponds to the number of columns
    if (data.m_channels.size() != m_vlpConfig.m_numOfRowsInColumn) {
        ERR("Channels size is not valid: " + std::to_string(data.m_channels.size()));
        return false;
    }
    return true;
}

void VLPCommunication::SetData(const std::vector<VLPData>& data) {
    for (auto block : data) {
        if (!CheckDataValidation(block)) {
            ERR("received invalid block");
            continue;
        }
        // index is (angle / resolution) + 0.5 - to round up
        double index = block.m_azimuth / m_vlpConfig.m_realHorizontalResolution + 0.5f;
        m_velodyneData[index].m_channels = block.m_channels;
        m_velodyneData[index].m_timeStamp = block.m_timeStamp;
        m_velodyneData[index].m_azimuth = block.m_azimuth;
    }
}

VLPCommunication::VLPDataPacket VLPCommunication::CreateVLPPacket(int dataStartIndex) const{
    VLPDataPacket packet;
    FillFactory(packet);
    FillTimeStamp(packet, dataStartIndex);
    for (int dataIndex = dataStartIndex, packetIndex = 0; dataIndex < dataStartIndex + 2*NUM_OF_VLP_DATA_BLOCKS; dataIndex+=2, packetIndex++) {
        // fill in the azimuth
        FillAzimuth(packet, dataIndex, packetIndex);
        // fill in the dataRecords
        FillDataRecords(packet, dataIndex, packetIndex);
    }
    return packet;
}

void VLPCommunication::FillTimeStamp(VLPDataPacket& packet, int dataIndex) const {
    boost::posix_time::ptime timeStamp = m_velodyneData[dataIndex].m_timeStamp;
    boost::posix_time::time_duration td = timeStamp - boost::posix_time::from_time_t(0);
    // reduce hours from the time stamp (time stamp is how much time passed after the last round hour)
    unsigned long microseconds = td.total_microseconds() - (td.hours() * HOUR_TO_MICRO_SEC);
    ToByteArray((unsigned long)microseconds, packet.timeStamp, sizeof(packet.timeStamp));
}

void VLPCommunication::FillFactory(VLPDataPacket& packet) const {
    packet.factory[0] = m_vlpConfig.m_returnMode;
    packet.factory[1] = m_vlpConfig.m_dataSource;
}

void VLPCommunication::FillAzimuth(VLPDataPacket& packet, int dataIndex, int packetIndex) const {
    // convert the angle * 100 (in order to save double information) to array on the suitable block of the packet
    ToByteArray((unsigned int) (m_velodyneData[dataIndex].m_azimuth * AZIMUTH_MULT),
        packet.dataBlocks[packetIndex].azimuth, sizeof(packet.dataBlocks[packetIndex].azimuth));
}

void VLPCommunication::FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const {
    FillSingleDataRecord(dataIndex, 0, packet.dataBlocks[packetIndex].dataRecords);
    FillSingleDataRecord(dataIndex + 1, 0 + m_vlpConfig.m_numOfRowsInColumn, packet.dataBlocks[packetIndex].dataRecords);
}

void VLPCommunication::FillSingleDataRecord(int dataIndex, int dataRecordsIndex, VLPDataPacket::VLPDataBlock::DataChannel* dataRecords) const {
    auto values = m_velodyneData[dataIndex].m_channels;
    int i = dataRecordsIndex; // CheckDataValidation validated that the number of values == number of data records
    for (auto value : values) {
        // convert the distance * 500 (in order to save double information) to array on the suitable block of the packet
        ToByteArray((unsigned int)(value.first * DISTANCE_MULT), dataRecords[i].distance, sizeof(dataRecords[i].distance));
        i++;
    }
}

void VLPCommunication::Run() {
    m_sendDataThread = boost::thread(&VLPCommunication::SendData, this);
}

template <typename Func>
double VLPCommunication::FormatBlock(const unsigned char* arr, size_t size, Func func) const {
    long num = 0;
    for (int i = 0; i < size; i++) {
        num += ((long)arr[i] << i*8);
    }
    return func(num);
}

template <typename T>
bool VLPCommunication::ToByteArray(T num, unsigned char* ret, size_t size) const {
    if (ret == nullptr) {
        ERR("nullptr");
        return false;
    }
    // drop the right-most bytes and convert to new right most byte. after the loop - the bytes will be reversed
    for (int i = 0; i < size; i++) {
        ret[i] = (int)((num >> (8*i)) & 0xFF);
    }
    return true;
}

void VLPCommunication::printVelData() const {
    for (auto data : m_velodyneData) {
        auto values = data.m_channels;
        if (values.empty()) {
            continue;
        }
        std::cout << "Angle: ****" << data.m_azimuth << "****. Data: " << std::endl;
        for (auto val : values) {
            std::cout << "(" << val.first << "," << val.second << ") ";
        }
        std::cout << std::endl;
    }
}

void VLPCommunication::printPacketData(const VLPDataPacket& packet) const {

    auto azimuthFunc = [](double num) -> double {return (double)num/AZIMUTH_MULT; };
    auto distanceFunc = [](double num) -> double {return (double)num/DISTANCE_MULT; };
    auto tsFunc = [](double num) -> double {return (double)num/SECOND_TO_MICROSECOND; };
    auto defFunc = [](double num) -> double {return num; };

    for (auto block : packet.dataBlocks) {
        std::cout << "Azimuth: " << FormatBlock(block.azimuth, sizeof(block.azimuth), azimuthFunc)<< std::endl;
        int i = 0;
        for (auto channel : block.dataRecords) {
            std::cout << "  Distance " << i++ << ": " << FormatBlock(channel.distance, sizeof(channel.distance), distanceFunc);
            std::cout << "  Reflectivity: " << channel.reflectivity << std::endl;
        }
        std::cout << std::endl;
    }
    std::cout << "Return mode: " << retModeToStr[(ReturnMode)packet.factory[0]] << std::endl;
    std::cout << "Data source: " << dataSourceToStr[(DataSource)packet.factory[1]] << std::endl;
    std::cout << "*********** Time After hour: " << FormatBlock(packet.timeStamp, sizeof(packet.timeStamp), tsFunc) << " *********************" << std::endl;
}


#include <random>
std::vector<VLPCommunication::VLPData> generateData() {
    std::vector<VLPCommunication::VLPData> v;
    /*std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 60.0);*/
    for (int i = 0; i < 1800; i++) {
        VLPCommunication::t_channel_data channel;
        for (int j = 0; j < 16; j++) {
            channel.push_back(std::pair<double,short>(10, 0));
        }
        VLPCommunication::VLPData data((double)i/5, channel, boost::posix_time::microsec_clock::local_time());
        v.push_back(data);
    }

    return v;
}

int main() {
    VLPCommunication::VLPConfig conf(IP_ADDRESS, PORT);
    VLPCommunication vlp(conf);
    std::vector<VLPCommunication::VLPData> data = generateData();
    for (int i = 0; i < 1800; i+=30) {
        std::vector<VLPCommunication::VLPData> newVec(data.begin() + i, data.begin() + i + 30);
        vlp.SetData(newVec);
    }
    vlp.Run();
    while (true) { sleep(1);}
}

