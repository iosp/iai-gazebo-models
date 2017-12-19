/**
* VLPCommunication.cpp
* Manage communication between velodyne sensor with UDP socket
* Author: Binyamin Appelbaum
* Date: 7.11.17
*/

#include "VLPCommunication.h"
#include "Logger.h"

#include <boost/assign.hpp> // boost::assign::map_list_of
#include <boost/asio.hpp> // boost::asio::io_service
#include <boost/range/irange.hpp> // boost::irange
#include <boost/lexical_cast.hpp>

static const int DISTANCE_MULT = 500;
static const int AZIMUTH_MULT = 100;
static const unsigned long HOUR_TO_MICRO_SEC = 360 * SECOND_TO_MICROSECOND;

const std::map<VLPCommunication::ReturnMode, std::string> VLPCommunication::retModeToStr = 
            boost::assign::map_list_of(VLPCommunication::_STRONGEST_, "strongest")(VLPCommunication::_LAST_, "last")(VLPCommunication::_DUAL_, "dual");
const std::map<VLPCommunication::DataSource, std::string> VLPCommunication::dataSourceToStr = 
            boost::assign::map_list_of(VLPCommunication::_HDL32E_, "HDL_32E")(VLPCommunication::_VLP16_, "VLP16");

VLPCommunication::VLPCommunication(const VLPConfig& vlpConfig) : m_vlpConfig(vlpConfig) {
    InitVelodyneData();
    // transmittion frequency is the degrees*10 / <degrees range of packet>
    size_t transmissionFrequency = (m_vlpConfig.m_sensorFrequency * DEGREES) /
                                    (m_vlpConfig.m_realHorizontalResolution * 2*NUM_OF_VLP_DATA_BLOCKS);
    // sleep time is 1/transmission time * 1000 (milliseconds) * 1000 (to microseconds)
    m_sleepTimeBetweenEverySend = SECOND_TO_MICROSECOND / transmissionFrequency;
}

VLPCommunication::~VLPCommunication() {
    m_sendDataThread.interrupt();
}

VLPCommunication::VLPDataPacket::VLPDataPacket() {

}

void VLPCommunication::VLPDataPacket::InitVLPDataPacket() {
    std::fill_n(dataBlocks, NUM_OF_VLP_DATA_BLOCKS, VLPDataBlock());
    std::fill_n(timeStamp, 4, 0);
}

VLPCommunication::VLPConfig::VLPConfig(const std::string& ipAddress, const std::string& port, Resolution horizontalResolution,
             ReturnMode returnMode, DataSource dataSource, int sensorFrequency) :
             m_ipAddress(ipAddress), m_port(port), m_horizontalResolution(horizontalResolution),
              m_returnMode(returnMode), m_dataSource(dataSource), m_sensorFrequency(sensorFrequency) {
    m_realHorizontalResolution = m_horizontalResolution / 1000.0;     
}

void VLPCommunication::InitVelodyneData() {
    int numOfColumns = (DEGREES / m_vlpConfig.m_realHorizontalResolution);
    for (int i : boost::irange(0,numOfColumns)) {
       m_velodyneData.push_back(VLPData(0, t_channel_data(), boost::posix_time::microseconds(0)));
    }
}

bool VLPCommunication::CheckDataValidation(const VLPData& data) const {
    double angle = data.m_azimuth;
    // avoid 360 Degrees and above
    if ((angle >= DEGREES) || (angle < 0)) {
        LOG(_ERROR_, "Angle is not valid: " + std::to_string(angle));
        return false;
    }
    // check that the data size corresponds to the number of columns
    if (data.m_channels.size() != GetNumOfrowsInColumn()) {
        LOG(_ERROR_, "Channels size is not valid: " + std::to_string(data.m_channels.size()));
        return false;
    }
    return true;
}

bool VLPCommunication::IsDataZeroed(int dataIndex) const {
    return std::all_of(m_velodyneData[dataIndex].m_channels.begin(), m_velodyneData[dataIndex].m_channels.end(), 
            [](const std::pair<double, short>& p) { return p.first == 0; });
}

void VLPCommunication::SetData(const std::vector<VLPData>& data) {
    for (auto const& block : data) {
        m_velodyneDataMutex.lock();
        if (!CheckDataValidation(block)) {
            LOG(_ERROR_, "received invalid block");
            m_velodyneDataMutex.unlock();
            continue;
        }
        // index is (angle / resolution) + 0.5 - to round up
        double index = block.m_azimuth / m_vlpConfig.m_realHorizontalResolution + 0.5f; // HANDLE CASTING!!
        m_velodyneData[index].m_channels = block.m_channels;
        m_velodyneData[index].m_durationAfterLastHour = block.m_durationAfterLastHour;
        m_velodyneData[index].m_azimuth = block.m_azimuth;
        m_velodyneDataMutex.unlock();
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
        LOG(_ERROR_, "Failed to send packet. " + err.message());
    }
    socket.close();
}

void VLPCommunication::SendData() const {
    using namespace boost::posix_time;
    
    long packetsCounter = 0;
    VLPDataPacket packet;
    FillFactory(packet);
    int packetIndex = 0;
    time_duration lastDuration = microseconds(0);
    ptime startTime = microsec_clock::local_time();
    // thread works until interrupt
    while (true) {
        int dataIndex = 0; 
        // run over m_velodyneData
        while (dataIndex < (DEGREES / m_vlpConfig.m_realHorizontalResolution)) {
            // send packet when it contains the required number of blocks
            if (packetIndex == NUM_OF_VLP_DATA_BLOCKS) {
                packetsCounter++;
                SendPacket(packet);
                // printPacketData(packet);
                packet.InitVLPDataPacket();
                packetIndex = 0;
                // calculate sleep time: (<time to sleep> - <time of the iteration>)
                ptime endTime = microsec_clock::local_time();
                time_duration diff = endTime - startTime;
                int sleepTime = m_sleepTimeBetweenEverySend - diff.total_microseconds();
                if (sleepTime > 0) {
                    usleep(sleepTime);
                }
                startTime = microsec_clock::local_time();
            }
            // critical section - try to fill a block
            m_velodyneDataMutex.lock();
            // fill block only if the time stamps of the blocks are valid
            if (CanAddToPacket(lastDuration, dataIndex)) {
                FillBlockInPacket(dataIndex, packetIndex, packet);
                // take the last duration from the last cell that was inserted
                lastDuration = m_velodyneData[dataIndex + DataIndexIncrement() - 1].m_durationAfterLastHour;
                packetIndex++;
            }
            dataIndex += DataIndexIncrement();
            m_velodyneDataMutex.unlock();
        }
    }
}

void VLPCommunication::FillBlockInPacket(int dataIndex, int packetIndex, VLPDataPacket& packet) const {
    // fill time stamp only for the first index
    if (packetIndex == 0) {
        FillTimeStamp(packet, dataIndex);
    }
    FillAzimuth(packet, dataIndex, packetIndex);
    FillDataRecords(packet, dataIndex, packetIndex);
}

void VLPCommunication::FillTimeStamp(VLPDataPacket& packet, int dataIndex) const {
    boost::posix_time::time_duration td = m_velodyneData[dataIndex].m_durationAfterLastHour;
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

void VLPCommunication::FillChannelsInPacket(VLPDataPacket& packet, const t_channel_data& channels, int packetIndex) const {
    for (auto i : boost::irange<size_t>(0, channels.size())) {
         // convert the distance * 500 (in order to save double information) to array on the suitable block of the packet
        ToByteArray((unsigned int)(channels[i].first * DISTANCE_MULT), 
            packet.dataBlocks[packetIndex].dataRecords[i].distance, sizeof(packet.dataBlocks[packetIndex].dataRecords[i].distance));
    }
}

VLPCommunication::t_channel_data VLPCommunication::MapChannels(const t_channel_data& channels) const {
    int channelsSize = channels.size();
    t_channel_data newChannels(channelsSize);
    // put the size/2 first elements in the correspoindig 2*i indexes on the new vector (0 -> 0, 1 -> 2, 2 -> 4, etc)
    for (auto i : boost::irange(0,channelsSize / 2)) {
        newChannels[i*2] = channels[i];
    }
    // fill the rest indexes of the new vector with the last size/2 elements of the original vector ( 1 -> 8, 3 -> 9, 5 -> 10, etc)
    for (int i = 1, j = channelsSize / 2; i < channelsSize; i += 2, j++) {
        newChannels[i] = channels[j];
    }
    return newChannels;
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
        LOG(_ERROR_, "nullptr");
        return false;
    }
    // drop the right-most bytes and convert to new right most byte. after the loop - the bytes will be reversed
    for (int i = 0; i < size; i++) {
        ret[i] = (int)((num >> (8*i)) & 0xFF);
    }
    return true;
}

void VLPCommunication::printVelData() const {       
    for (auto const& data : m_velodyneData) {
        auto values = data.m_channels;
        if (values.empty()) {
            continue;
        }
        std::cout << "Angle: ****" << data.m_azimuth << "****. Data: " << std::endl;
        for (auto const& val : values) {
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

    for (auto const& block : packet.dataBlocks) {
        LOG(_NORMAL_, "Azimuth: " + std::to_string(FormatBlock(block.azimuth, sizeof(block.azimuth), azimuthFunc)));
        int i = 0;
        for (auto const& channel : block.dataRecords) {
            std::stringstream ss;
            ss << "  Distance " << i++ << ": "  << FormatBlock(channel.distance, sizeof(channel.distance), distanceFunc);
            //ss << "  Reflectivity: " << channel.reflectivity;
            LOG(_NORMAL_, ss.str());
        }
        LOG(_NORMAL_, "\n");
    }
    LOG(_DEBUG_, "Return mode: " + retModeToStr.find((ReturnMode)packet.factory[0])->second);
    LOG(_DEBUG_, "Data source: " + dataSourceToStr.find((DataSource)packet.factory[1])->second);
    LOG(_NORMAL_, "*********** Time After startup: " + 
        std::to_string(FormatBlock(packet.timeStamp, sizeof(packet.timeStamp), tsFunc)) + " *********************");
}
