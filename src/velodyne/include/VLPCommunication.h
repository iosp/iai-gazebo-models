#ifndef VLP_COMMUNICATION
#define VLP_COMMUNICATION

/*
* VLPCommunication.h
* Manage communication between velodyne sensor with UDP socket
* Author: Binyamin Appelbaum
* VLP = Velodyne Lidar Puck
*/

#include <string>
#include <vector>
#include <boost/date_time/posix_time/posix_time.hpp> // boost::posix_time::ptime
#include <boost/thread.hpp> // boost::thread

static const int SENSOR_FREQ = 10;
static const int NUM_OF_VLP_DATA_CHANNELS_IN_BLOCK = 32;
static const int NUM_OF_VLP_DATA_BLOCKS = 12;
enum NumOfDataChannels {_VEL16_ = 16, _VEL32_ = 32};
enum Resolution { _RES02_ = 200, _RES04_ = 400};
enum ReturnMode { _STRONGEST_ = 37, _LAST_ = 38, _DUAL_ = 39};
enum DataSource {_HDL32E_ = 21, _VLP16_ = 22};

class VLPCommunication {
public:
    /**
     * Hold VLP configuration
     */
    struct VLPConfig {
        std::string m_ipAddress;
        std::string m_port;
        Resolution m_horizontalResolution;
        double m_realHorizontalResolution;
        NumOfDataChannels m_numOfRowsInColumn;
        ReturnMode m_returnMode;
        DataSource m_dataSource;
        int m_sensorFrequency;
        VLPConfig() {}
        VLPConfig(const std::string& ipAddress, const std::string& port, Resolution horizontalResolution = _RES02_,
             NumOfDataChannels numOfRowsInColumn = _VEL16_, ReturnMode returnMode = _DUAL_, DataSource dataSource = _VLP16_,
             int sensorFrequency = SENSOR_FREQ);
        };

    typedef std::vector<std::pair<double, short> > t_channel_data;
    /**
     * VLP data to get and save 
     */  
    struct VLPData {
        double m_azimuth;
        t_channel_data m_channels;
        boost::posix_time::ptime m_timeStamp;
        VLPData(){}
        VLPData(double azimuth, const t_channel_data& channels, const boost::posix_time::ptime& timeStamp) :
            m_azimuth(azimuth), m_channels(channels), m_timeStamp(timeStamp) {} 
    };

private:
    /**
     * VLP packet that defined by Velodyne
     */
    struct VLPDataPacket {
        struct VLPDataBlock {
            struct DataChannel {
                unsigned char distance[2]{};
                unsigned char reflectivity{};
            };
    
            short flag = 0xEEFF; // same for every block
            unsigned char azimuth[2]{};
            DataChannel dataRecords[NUM_OF_VLP_DATA_CHANNELS_IN_BLOCK];
        };
        
        VLPDataBlock dataBlocks[NUM_OF_VLP_DATA_BLOCKS];
        unsigned char timeStamp[4]{}; // time stamp is how much seconds passed after the last round hour
        unsigned char factory[2]{};
    };

    /**
     * velodyne data to save on process  
    */
    std::vector<VLPData> m_velodyneData; // TODO mutex!!
    /**
     * VLP configuration values
     */ 
    VLPConfig m_vlpConfig;
    /**
     * thread of data send
     */ 
    boost::thread m_sendDataThread;
    /**
     * Send data via UDP socket
     */
    void SendData() const;

    /**
     * Send packet via UDP socket
     * @param packet - struct of VLP packet 
     */
    void SendPacket(const VLPDataPacket& packet) const;

    /**
     * Initialize inner velodyne data 
     */
    void InitVelodyneData();

    /**
     * Create VLP packet to send
     * @param dataStartIndex - index to velodyne data vector, to start sending from this point
     * @return struct of VLPDataPacket 
     */
    VLPDataPacket CreateVLPPacket(int dataStartIndex) const;

    /**
     * Fill time stamp on VLP packet, on dataIndex in velodyne vector
     * @param packet - struct of VLP packet
     * @param dataIndex - the index on velodyne data vector to get the time from
     */ 
    void FillTimeStamp(VLPDataPacket& packet, int dataIndex) const;

    /**
     * Fill factory field on VLP packet
     * @param packet - struct of VLP packet
     */
    void FillFactory(VLPDataPacket& packet) const;

    /**
     * Fill azimuth on VLP packet (on suitable block - according to packetIndex)
     * @param packet - struct of VLP packet
     * @param dataIndex - the index on velodyne data vector to get the azimuth from
     * @param packetIndex - the index on VLP packet struct to put the data on
     */
    void FillAzimuth(VLPDataPacket& packet, int dataIndex, int packetIndex) const;

    /**
     * Fill data records on VLP packet (on suitable block - according to packetIndex)
     * @param packet - struct of VLP packet
     * @param dataIndex - the index on velodyne data vector to get the data from
     * @param packetIndex - the index on VLP packet struct to put the data on
     */
    void FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const;

    /**
     * Fill signee data record on VLP packet
     * @param dataIndex - the index on velodyne data vector to get the data from
     * @param dataRecords - reference to the data records on VLP packet struct
     */
    void FillSingleDataRecord(int dataIndex, int dataRecordsIndex, VLPDataPacket::VLPDataBlock::DataChannel* dataRecords) const;

    /**
     * Check if current packet has been sent already (by its time stamp).
     * The method also returns the current timeStamp of the packet by the reference member.
     * @param packet - struct of VLP packet
     * @param lastTimeStamp - last sent packet time stamp
     * @param currTimeStamp - out member, current timestamp of the packet
     * @return true if currTimeStamp < lastTimeStamp and false O.W 
     */
    bool IsPacketHasBeenSent(const VLPDataPacket& packet, unsigned int lastTimeStamp,/* out */ unsigned int& currTimeStamp) const;

    /**
     * Check validation of VLP data
     * @param data - VLP data struct
     * @return true if data is valid and false otherwise
    */
    bool CheckDataValidation(const VLPData& data) const;

    /**
     * convert number to unsigned char array with HEX values of this number. the array bytes are reversed.
     * This function works only for unsigned types!
     * @param num - unsinged long / int / short number
     * @param ret - return buffer
     * @size - size of ret array
     * @return bool - true for success, false for wrong input (ret == nullptr)
     */ 
    template <typename T>
    bool ToByteArray(T num, unsigned char* ret, size_t size) const;

    /**
     * convert block of unsigned char array with HEX values to number. the array bytes are reversed.
     * This function works only for unsigned types!
     * @param arr - the array with HEX values
     * @size - size of array
     * @func - lambda function to operate on the number
     * @return double - the original number after func operated on it
     */ 
    template <typename Func>
    double FormatBlock(const unsigned char* arr, size_t size, Func func) const;

    /**
     * Print vector of veclodyne data. for debug only
     */ 
    void printVelData() const;

    /**
     * Print the packet data (formatted). for debug only
     * @param packet - VLP data packet
     */ 
    void printPacketData(const VLPDataPacket& packet) const;


public:
    /**
     * Ctor
     * @param vlpConfig - struct of VLPConfig
     */ 
    VLPCommunication(const VLPConfig& vlpConfig);
    ~VLPCommunication();

    /**
     * Set data on inner velodyne data vector
     * @param data - vector of VLPData struct
     */ 
    void SetData(const std::vector<VLPData>& data);

    /**
     * Run VLP send data thread
     */ 
    void Run();

    const VLPConfig& GetConfig() const {
        return m_vlpConfig;
    }   

};



#endif // VLP_COMMUNICATION