/*
* VLPCommunication16.cpp
* Manage communication between velodyne sensor with UDP socket - for VLP 16
* Author: Binyamin Appelbaum
* Date: 13.12.17
*/

#include "VLPCommunication16.h"

VLPCommunication16::VLPCommunication16(const VLPConfig& vlpConfig) : VLPCommunication(vlpConfig) {
}

void VLPCommunication16::FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const {
    auto values = MapChannels(m_velodyneData[dataIndex].m_channels);

    auto additionalValues = MapChannels(m_velodyneData[dataIndex + 1].m_channels);
    values.insert(values.end(), additionalValues.begin(), additionalValues.end());

    FillChannelsInPacket(packet, values, packetIndex);
}

bool VLPCommunication16::CanAddToPacket(const boost::posix_time::time_duration& lastDuration, int dataIndex) const {
    return (lastDuration < m_velodyneData[dataIndex].m_durationAfterLastHour) && 
                (m_velodyneData[dataIndex].m_durationAfterLastHour < m_velodyneData[dataIndex + 1].m_durationAfterLastHour) &&
                !IsDataZeroed(dataIndex);
}

int VLPCommunication16::DataIndexIncrement() const {
    return 2;
}

int VLPCommunication16::GetNumOfrowsInColumn() const {
    return 16;
}