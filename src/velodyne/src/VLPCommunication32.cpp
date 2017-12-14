/*
* VLPCommunication32.cpp
* Manage communication between velodyne sensor with UDP socket - for VLP 32
* Author: Binyamin Appelbaum
* Date: 13.12.17
*/

#include "VLPCommunication32.h"

VLPCommunication32::VLPCommunication32(const VLPConfig& vlpConfig) : VLPCommunication(vlpConfig) {

}
void VLPCommunication32::FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const {
    auto values = MapChannels(m_velodyneData[dataIndex].m_channels);

    FillChannelsInPacket(packet, values, packetIndex);
}

bool VLPCommunication32::CanAddToPacket(const boost::posix_time::time_duration& lastDuration, int dataIndex) const {
    return (lastDuration < m_velodyneData[dataIndex].m_durationAfterLastHour);
}

int VLPCommunication32::DataIndexIncrement() const {
    return 1;
}

int VLPCommunication32::GetNumOfrowsInColumn() const {
    return 32;
}