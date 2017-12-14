#ifndef VLP_COMMUNICATION16
#define VLP_COMMUNICATION16

/*
* VLPCommunication16.h
* Manage communication between velodyne sensor with UDP socket - for VLP 16
* Author: Binyamin Appelbaum
* Date: 13.12.17
*/

#include "VLPCommunication.h"


class VLPCommunication16 : public VLPCommunication {
protected:

    virtual void FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const;

    virtual int GetNumOfrowsInColumn() const;

    virtual bool CanAddToPacket(const boost::posix_time::time_duration& lastDuration, int dataIndex) const;

    virtual int DataIndexIncrement() const;

public:
    VLPCommunication16(const VLPConfig& vlpConfig);
};



#endif // VLP_COMMUNICATION16