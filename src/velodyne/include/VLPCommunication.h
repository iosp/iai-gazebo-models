#ifndef VLP_COMMUNICATION
#define VLP_COMMUNICATION

/*
* VLPCommunication.h
* Manage communication between velodyne sensor with UDP socket
* Author: Binyamin Appelbaum
* Date: 7.11.17
* VLP = Velodyne Lidar Puck
*/

#include <string>
#include <vector>
#include <map>
#include <boost/date_time/posix_time/posix_time.hpp> // boost::posix_time::time_duration
#include <boost/thread.hpp> // boost::thread

static const int DEGREES = 360;
static const int SECOND_TO_MICROSECOND  = 1e6;
static const int SENSOR_FREQ = 10;
static const int NUM_OF_VLP_DATA_CHANNELS_IN_BLOCK = 32;
static const int NUM_OF_VLP_DATA_BLOCKS = 12;

class VLPCommunication {
public:
    enum Resolution { _RES02_ = 200, _RES04_ = 400};
    enum ReturnMode { _STRONGEST_ = 37, _LAST_ = 38, _DUAL_ = 39};
    enum DataSource {_HDL32E_ = 21, _VLP16_ = 22};
    /**
     * Hold VLP configuration
     */
    struct VLPConfig {
        std::string m_ipAddress;
        std::string m_port;
        Resolution m_horizontalResolution;
        double m_realHorizontalResolution;
        ReturnMode m_returnMode;
        DataSource m_dataSource;
        int m_sensorFrequency;
        VLPConfig() = default;
        VLPConfig(const std::string& ipAddress, const std::string& port, Resolution horizontalResolution = _RES02_,
            ReturnMode returnMode = _STRONGEST_, DataSource dataSource = _VLP16_,
             int sensorFrequency = SENSOR_FREQ);
        };

    typedef std::vector<std::pair<double, short> > t_channel_data;
    /**
     * VLP data to get and save 
     */  
    struct VLPData {
        double m_azimuth;
        t_channel_data m_channels;
        boost::posix_time::time_duration m_durationAfterLastHour;
        VLPData() = default;
        VLPData(double azimuth, const t_channel_data& channels, const boost::posix_time::time_duration& durationAfterLastHour) :
            m_azimuth(azimuth), m_channels(channels), m_durationAfterLastHour(durationAfterLastHour) {} 
    };

protected:
    /**
     * VLP packet that defined by Velodyne
     */
    class VLPDataPacket {
    public:
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
        VLPDataPacket();
        void InitVLPDataPacket();
    };

    /**
     * velodyne data to save on process  
    */
    std::vector<VLPData> m_velodyneData;
    /**
     * VLP configuration values
     */ 
    VLPConfig m_vlpConfig;
    /**
     * time to sleep between every packet send
    */
    int m_sleepTimeBetweenEverySend;
    /**
     * thread of data send
     */ 
    boost::thread m_sendDataThread;
    /**
     * mutex to save velodyne data
     */ 
    mutable boost::mutex m_velodyneDataMutex;
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
     * Fill one block in packet
     * @param packet - struct of VLP packet
     * @param dataIndex - the index on velodyne data vector to get the time from
     * @param packetIndex - the index on VLP packet struct to put the data on
     */ 
    void FillBlockInPacket(int dataIndex, int packetIndex, VLPDataPacket& packet) const;

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
     * Transform vector of channels to adapt Velodyne format.
     * The method converts vector indexes ( the example is for VLP16 but works also for VLP32)
     * Orig: 0  1  2  3  4  5  6  7  8  9  10  11  12  13  14  15
     * New:  0  8  1  9  2  10 3 11  4  12 5   13  6   14  7   15
     * @param channels - vector of channel pairs (distance and reflectivity)
     * @return dataIndex - new formatted vector
     */
    t_channel_data MapChannels(const t_channel_data& channels) const;

    /**
     * Fill channles in specific packet in packet index
     * @param packet - struct of VLP packet
     * @param channels - the data of the channels
     * @param packetIndex - the index on VLP packet struct to put the data on
     */
    void FillChannelsInPacket(VLPDataPacket& packet, const t_channel_data& channels, int packetIndex) const;

    /**
     * Fill data records on VLP packet (on suitable block - according to packetIndex)
     * @param packet - struct of VLP packet
     * @param dataIndex - the index on velodyne data vector to get the data from
     * @param packetIndex - the index on VLP packet struct to put the data on
     */
    virtual void FillDataRecords(VLPDataPacket& packet, int dataIndex, int packetIndex) const = 0;
    
    /**
     * Check validation of VLP data
     * @param data - VLP data struct
     * @param numOfRowsInColumn - number of rows expected in every column
     * @return true if data is valid and false otherwise
    */
    virtual bool CheckDataValidation(const VLPData& data) const;

    /**
     * Get how many rows in column on the data table
     * @return integer of the number
    */
    virtual int GetNumOfrowsInColumn() const = 0;

    /**
     * Check if the last duration enables us to add the next element to the packet
     * @param lastDuration - last duration that inserted
     * @param dataIndex - the index on velodyne data vector to get the data from
     * @return true if the next element can be added and false O.W
    */
    virtual bool CanAddToPacket(const boost::posix_time::time_duration& lastDuration, int dataIndex) const = 0;

    /**
     * Check if the velodyne data in specific index is all zero values
     * @param dataIndex - the index on velodyne data vector to get the data from
     * @return true if the data is zeroed and false O.W
    */
    virtual bool IsDataZeroed(int dataIndex) const;

    /**
     * get a number to add in every data iteration
     * @return integer of the number
    */
    virtual int DataIndexIncrement() const = 0;

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

    static const std::map<VLPCommunication::ReturnMode, std::string> retModeToStr;
    static const std::map<VLPCommunication::DataSource, std::string> dataSourceToStr;

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

    static void printVelData(const std::vector<VLPCommunication::VLPData>& velData);

};



#endif // VLP_COMMUNICATION