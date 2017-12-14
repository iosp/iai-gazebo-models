#ifndef VLP_COMMUNICATION32
#define VLP_COMMUNICATION32

/*
* VLPCommunication32.h
* Manage communication between velodyne sensor with UDP socket - for VLP 32
* Author: Binyamin Appelbaum
* Date: 13.12.17
*/

#include "VLPCommunication.h"


class VLPCommunication32 : public VLPCommunication {
protected:

    virtual void FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const;

    virtual int GetNumOfrowsInColumn() const;

    virtual bool CanAddToPacket(const boost::posix_time::time_duration& lastDuration, int dataIndex) const;

    virtual int DataIndexIncrement() const;

public:
    VLPCommunication32(const VLPConfig& vlpConfig);
};



#endif // VLP_COMMUNICATION32