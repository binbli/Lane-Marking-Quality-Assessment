// pcap2kitti
//
// pcap2kitti is based on the code from https://github.com/UnaNancyOwen/VelodyneCapture.git.
// pcap2kitti is the capture class to retrieve the laser data from Velodyne sensors using Boost.Asio and PCAP.
// It will be able to retrieve lasers infomation about azimuth, vertical and distance that capture at one rotation.
// This class supports direct capture form Velodyne sensors, or capture from PCAP files.
// ( This class only supports VLP-16, HDL-32E and HDL-64E sensor, and not supports Dual Return mode. )
//
// If direct capture from sensors, pcap2kitti are requires Boost.Asio and its dependent libraries ( Boost.System, Boost.Date_Time, Boost.Regex ).
// Please define HAVE_BOOST in preprocessor.
//
// If capture from PCAP files, pcap2kitti are requires PCAP.
// Please define HAVE_PCAP in preprocessor.
//
// This orginal source code is licensed under the MIT license.
// Copyright (c) 2017 Tsukasa SUGIURA
// t.sugiura0204@gmail.com

#ifndef VELODYNE_CAPTURE
#define VELODYNE_CAPTURE

#include <string>
#include <sstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <vector>
#include <cassert>
#include <cstdint>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <functional>

#ifdef HAVE_BOOST
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#endif
#ifdef HAVE_PCAP
#include <pcap.h>
#endif

namespace velodyne {
    struct Laser {
        double azimuth;
        double vertical;
        unsigned short distance;
        unsigned char intensity;
        unsigned char id;
        long long time;

        const bool operator<(const struct Laser &laser) {
            if (azimuth == laser.azimuth) {
                return id < laser.id;
            } else {
                return azimuth < laser.azimuth;
            }
        }
    };

    class pcap2kitti {
    protected:
#ifdef HAVE_BOOST
        boost::asio::io_service ioservice;
        boost::asio::ip::udp::socket* socket = nullptr;
        boost::asio::ip::address address;
        unsigned short port = 2368;
#endif

#ifdef HAVE_PCAP
        pcap_t* pcap = nullptr;
        std::string filename = "";
#endif

        std::thread *thread = nullptr;
        std::atomic_bool run = {false};
        std::mutex mutex;
        std::queue<std::vector<Laser>> queue;

        int MAX_NUM_LASERS;
        std::vector<double> lut;

        static const int LASER_PER_FIRING = 32;
        static const int FIRING_PER_PKT = 12;

#pragma pack(push, 1)
        typedef struct LaserReturn {
            uint16_t distance;
            uint8_t intensity;
        } LaserReturn;
#pragma pack(pop)

        struct FiringData {
            uint16_t blockIdentifier;
            uint16_t rotationalPosition;
            LaserReturn laserReturns[LASER_PER_FIRING];
        };

        struct DataPacket {
            FiringData firingData[FIRING_PER_PKT];
            uint32_t gpsTimestamp;
            uint8_t mode;
            uint8_t sensorType;
        };

    public:
        // Constructor
        pcap2kitti() {
        };

#ifdef HAVE_BOOST
        // Constructor ( direct capture from Sensor )
        pcap2kitti( const boost::asio::ip::address& address, const unsigned short port = 2368 )
        {
            open( address, port );
        };
#endif

#ifdef HAVE_PCAP
        // Constructor ( capture from PCAP )
        pcap2kitti( const std::string& filename )
        {
            open( filename );
        };
#endif

        // Destructor
        ~pcap2kitti() {
            close();
        };

#ifdef HAVE_BOOST
        // Open Direct Capture from Sensor
        const bool open( const boost::asio::ip::address& address, const unsigned short port = 2368 )
        {
            // Check Running
            if( isRun() ){
                close();
            }

            // Set IP-Address and Port
            this->address = ( !address.is_unspecified() ) ? address : boost::asio::ip::address::from_string( "255.255.255.255" );
            this->port = port;

            // Create Socket
            try{
                socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( this->address, this->port ) );
            }
            catch( ... ){
                delete socket;
                socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( boost::asio::ip::address_v4::any(), this->port ) );
            }

            // Start IO-Service
            try{
                ioservice.run();
            }
            catch( const std::exception& e ){
                std::cerr << e.what() << std::endl;
                return false;
            }

            // Start Capture Thread
            run = true;
            thread = new std::thread( std::bind( &pcap2kitti::captureSensor, this ) );

            return true;
        };
#endif

#ifdef HAVE_PCAP
        // Open Capture from PCAP
        const bool open( const std::string& filename )
        {
            // Check Running
            if( isRun() ){
                close();
            }

            // Open PCAP File
            char error[PCAP_ERRBUF_SIZE];
            pcap_t* pcap = pcap_open_offline( filename.c_str(), error );
            if( !pcap ){
                throw std::runtime_error( error );
                return false;
            }

            // Convert PCAP_NETMASK_UNKNOWN to 0xffffffff
            struct bpf_program filter;
            std::ostringstream oss;
            if( pcap_compile( pcap, &filter, oss.str().c_str(), 0, 0xffffffff ) == -1 ){
                throw std::runtime_error( pcap_geterr( pcap ) );
                return false;
            }

            if( pcap_setfilter( pcap, &filter ) == -1 ){
                throw std::runtime_error( pcap_geterr( pcap ) );
                return false;
            }

            this->pcap = pcap;
            this->filename = filename;

            // Start Capture Thread
            run = true;
            thread = new std::thread( std::bind( &pcap2kitti::capturePCAP, this ) );

            return true;
        };
#endif

        // Check Open
        const bool isOpen() {
            std::lock_guard<std::mutex> lock(mutex);
            return (
#if defined( HAVE_BOOST ) || defined( HAVE_PCAP )
#ifdef HAVE_BOOST
            ( socket && socket->is_open() )
#endif
#if defined( HAVE_BOOST ) && defined( HAVE_PCAP )
            ||
#endif
#ifdef HAVE_PCAP
            pcap != nullptr
#endif
#else
                    false
#endif
            );
        };

        // Check Run
        const bool isRun() {
            // Returns True when Thread is Running or Queue is Not Empty
            std::lock_guard<std::mutex> lock(mutex);
            return (run || !queue.empty());
        }

        // Close Capture
        void close() {
            std::lock_guard<std::mutex> lock(mutex);
            run = false;
            // Close Capturte Thread
            if (thread && thread->joinable()) {
                thread->join();
                thread->~thread();
                delete thread;
                thread = nullptr;
            }

#ifdef HAVE_BOOST
            // Close Socket
            if( socket && socket->is_open() ){
                socket->close();
                delete socket;
                socket = nullptr;
            }

            // Stop IO-Service
            if( ioservice.stopped() ){
                ioservice.stop();
                ioservice.reset();
            }
#endif

#ifdef HAVE_PCAP
            // Close PCAP
            if( pcap ){
                pcap_close( pcap );
                pcap = nullptr;
                filename = "";
            }
#endif

            // Clear Queue
            std::queue<std::vector<Laser>>().swap(queue);
        };

        // Retrieve Capture Data
        void retrieve(std::vector<Laser> &lasers, const bool sort = false) {
            // Pop One Rotation Data from Queue
            if (mutex.try_lock()) {
                if (!queue.empty()) {
                    lasers = std::move(queue.front());
                    queue.pop();
                    if (sort) {
                        std::sort(lasers.begin(), lasers.end());
                    }
                }
                mutex.unlock();
            }
        };

        // Operator Retrieve Capture Data with Sort
        void operator>>(std::vector<Laser> &lasers) {
            // Retrieve Capture Data
            retrieve(lasers, false);
        };

        size_t getQueueSize() {
            std::lock_guard<std::mutex> lock(mutex);
            return queue.size();
        }

    private:
#ifdef HAVE_BOOST
        // Capture Thread from Sensor
        void captureSensor()
        {
            struct timeval last_time = { 0 };
            double last_azimuth = 0.0;
            std::vector<Laser> lasers;
            unsigned char data[1500];
            boost::asio::ip::udp::endpoint sender;

            while( socket->is_open() && ioservice.stopped() && run ){
                // Receive Packet
                boost::system::error_code error;
                const size_t length = socket->receive_from( boost::asio::buffer( data, sizeof( data ) ), sender, 0, error );
                if( error == boost::asio::error::eof ){
                    break;
                }

                // Check IP-Address and Port
                if( sender.address() != address && sender.port() != port ){
                    continue;
                }

                // Check Packet Data Size
                // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
                if( length != 1206 ){
                    continue;
                }

                // Retrieve Unix Time ( microseconds )
                const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
                const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>( now.time_since_epoch() );
                const long long unixtime = epoch.count();

                // Convert to DataPacket Structure
                // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
                const DataPacket* packet = reinterpret_cast<const DataPacket*>( data );
                assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

                // Caluculate Interpolated Azimuth
                double interpolated = 0.0;
                if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                    interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
                }
                else{
                    interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
                }

                // Processing Packet
                for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                    // Retrieve Firing Data
                    const FiringData firing_data = packet->firingData[firing_index];
                    for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                        // Retrieve Rotation Azimuth
                        double azimuth = static_cast<double>( firing_data.rotationalPosition );

                        // Interpolate Rotation Azimuth
                        if( laser_index >= MAX_NUM_LASERS )
                        {
                            azimuth += interpolated;
                        }

                        // Reset Rotation Azimuth
                        if( azimuth >= 36000 )
                        {
                            azimuth -= 36000;
                        }

                        // Complete Retrieve Capture One Rotation Data
                        if( last_azimuth > azimuth ){
                            // Push One Rotation Data to Queue
                            mutex.lock();
                            queue.push( std::move( lasers ) );
                            mutex.unlock();
                            lasers.clear();
                        }

                        Laser laser;
                        laser.azimuth = azimuth / 100.0;
                        laser.vertical = lut[laser_index % MAX_NUM_LASERS];
#ifdef USE_MILLIMETERS
                        laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0;
#else
                        laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0 / 10;
#endif
                        laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                        laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );
#ifdef HAVE_GPSTIME
                        laser.time = packet->gpsTimestamp;
#else
                        laser.time = unixtime;
#endif
                        lasers.push_back( laser );

                        // Update Last Rotation Azimuth
                        last_azimuth = azimuth;
                    }
                }
            }

            run = false;
        };
#endif

#ifdef HAVE_PCAP
        // Capture Thread from PCAP
        void capturePCAP()
        {
            struct timeval last_time = { 0 };
            double last_azimuth = 0.0;
            std::vector<Laser> lasers;

            while( run ){
                // Retrieve Header and Data from PCAP
                struct pcap_pkthdr* header;
                const unsigned char* data;
                const int ret = pcap_next_ex( pcap, &header, &data );
                if( ret <= 0 ){
                    break;
                }

                // Check Packet Data Size
                // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
                if( ( header->len - 42 ) != 1206 ){
                    continue;
                }

                // Retrieve Unix Time ( microseconds )
                std::stringstream ss;
                ss << header->ts.tv_sec << std::setw( 6 ) << std::left << std::setfill( '0' ) << header->ts.tv_usec;
                const long long unixtime = std::stoll( ss.str() );

                // Convert to DataPacket Structure ( Cut Header 42 bytes )
                // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
                const DataPacket* packet = reinterpret_cast<const DataPacket*>( data + 42 );
                assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );

                // Wait This Thread Difference Time
                if( last_time.tv_sec == 0 )
                {
                    last_time = header->ts;
                }

                if( last_time.tv_usec > header->ts.tv_usec )
                {
                    last_time.tv_usec -= 1000000;
                    last_time.tv_sec++;
                }

                const unsigned long long delay = ( ( header->ts.tv_sec - last_time.tv_sec ) * 1000000 ) + ( header->ts.tv_usec - last_time.tv_usec );
#ifndef HAVE_FAST_PCAP
                std::this_thread::sleep_for( std::chrono::microseconds( delay ) );
#endif
                last_time = header->ts;

                // Caluculate Interpolated Azimuth
                double interpolated = 0.0;
                if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                    interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
                }
                else{
                    interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
                }

                // Processing Packet
                for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                    // Retrieve Firing Data
                    const FiringData firing_data = packet->firingData[firing_index];
                    for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                        // Retrieve Rotation Azimuth
                        double azimuth = static_cast<double>( firing_data.rotationalPosition );

                        // Interpolate Rotation Azimuth
                        if( laser_index >= MAX_NUM_LASERS )
                        {
                            azimuth += interpolated;
                        }

                        // Reset Rotation Azimuth
                        if( azimuth >= 36000 )
                        {
                            azimuth -= 36000;
                        }

                        // Complete Retrieve Capture One Rotation Data
                        if( last_azimuth > azimuth ){
                            // Push One Rotation Data to Queue
                            mutex.lock();
                            queue.push( std::move( lasers ) );
                            mutex.unlock();
                            lasers.clear();
                        }

                        Laser laser;
                        laser.azimuth = azimuth / 100.0;
                        laser.vertical = lut[laser_index % MAX_NUM_LASERS];
#ifdef USE_MILLIMETERS
                        laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0;
#else
                        laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance * 2.0 / 10;
#endif
                        laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                        laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );
#ifdef HAVE_GPSTIME
                        laser.time = packet->gpsTimestamp;
#else
                        laser.time = unixtime;
#endif
                        lasers.push_back( laser );
                        // Update Last Rotation Azimuth
                        last_azimuth = azimuth;
                    }
                }
            }

            run = false;
        };
#endif
    };

    class VLP16Capture : public pcap2kitti {
    private:
        static const int MAX_NUM_LASERS = 16;
        const std::vector<double> lut = {-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0,
                                         13.0, -1.0, 15.0};

    public:
        VLP16Capture() : pcap2kitti() {
            initialize();
        };

#ifdef HAVE_BOOST
        VLP16Capture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : pcap2kitti( address, port )
        {
            initialize();
        };
#endif

#ifdef HAVE_PCAP
        VLP16Capture( const std::string& filename ) : pcap2kitti( filename )
        {
            initialize();
        };
#endif

        ~VLP16Capture() {
        };

    private:
        void initialize() {
            pcap2kitti::MAX_NUM_LASERS = MAX_NUM_LASERS;
            pcap2kitti::lut = lut;
        };
    };

    class HDL32ECapture : public pcap2kitti {
    private:
        static const int MAX_NUM_LASERS = 32;
        const std::vector<double> lut = {-30.67, -9.3299999, -29.33, -8.0, -28, -6.6700001, -26.67, -5.3299999, -25.33,
                                         -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67,
                                         2.6700001, -17.33, 4.0, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0,
                                         9.3299999, -10.67, 10.67};

    public:
        HDL32ECapture() : pcap2kitti() {
            initialize();
        };

#ifdef HAVE_BOOST
        HDL32ECapture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : pcap2kitti( address, port )
        {
            initialize();
        };
#endif

#ifdef HAVE_PCAP
        HDL32ECapture( const std::string& filename ) : pcap2kitti( filename )
        {
            initialize();
        };
#endif

        ~HDL32ECapture() {
        };

    private:
        void initialize() {
            pcap2kitti::MAX_NUM_LASERS = MAX_NUM_LASERS;
            pcap2kitti::lut = lut;
        };
    };

    class HDL64ECapture : public pcap2kitti {
    private:
        static const int MAX_NUM_LASERS = 64;
        const std::vector<double> lut = {-2.0000, 1.6667, 1.3333, 1.0000, 0.6667, 0.3333, 0.0, -0.3333, -0.6667,
                                         -1.0000, -1.3333, -1.6667, -2.0000, -2.3333, -2.6667, -3.0000, -3.3333,
                                         -3.6667, -4.0000, -4.3333, -4.6667, -5.0000, -5.3333, -5.6667, -6.0000,
                                         -6.3333, -6.6667, -7.0000, -7.3333, -7.6667, -8.0000, -8.3333, -8.8300,
                                         -9.3300, -9.8300, -10.3300, -10.8300, -11.3300, -11.8300, -12.3300, -12.8300,
                                         -13.3300, -13.8300, -14.3300, -14.8300, -15.3300, -15.8300, -16.3300, -16.8300,
                                         -17.3300, -17.8300, -18.3300, -18.8300, -19.3300, -19.8300, -20.3300, -20.8300,
                                         -21.3300, -21.8300, -22.3300, -22.8300, -23.3300, -23.8300, -24.3300};

    public:
        HDL64ECapture() : pcap2kitti() {
            initialize();
        };

#ifdef HAVE_BOOST
        HDL64ECapture( const boost::asio::ip::address& address, const unsigned short port = 2368 ) : pcap2kitti( address, port )
            {
                initialize();
            };
#endif

#ifdef HAVE_PCAP
        HDL64ECapture( const std::string& filename ) : pcap2kitti( filename )
            {
                initialize();
            };
#endif

        ~HDL64ECapture() {
        };

    private:
        void initialize() {
            pcap2kitti::MAX_NUM_LASERS = MAX_NUM_LASERS;
            pcap2kitti::lut = lut;
        };
    };

}

#endif
