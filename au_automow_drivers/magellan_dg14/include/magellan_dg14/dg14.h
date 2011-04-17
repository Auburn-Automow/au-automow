#ifndef DG14_H_
#define DG14_H_
#include <serial.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#define CMD_TERMINATOR "\r\n"

#define CMD_POS_ON "$PASHS,NME,POS,A,ON,0.2" CMD_TERMINATOR
#define CMD_POS_OFF "$PASHS,NME,POS,A,OFF,0.2" CMD_TERMINATOR

#define CMD_SAT_ON "$PASHS,NME,SAT,A,ON,0.2" CMD_TERMINATOR
#define CMD_SAT_OFF "$PASHS,NME,SAT,A,OFF,0.2" CMD_TERMINATOR

#define CMD_UTM_ON = "$PASHS,NME,UTM,A,ON,0.2" CMD_TERMINATOR
#define CMD_UTM_OFF = "$PASHS,NME,UTM,A,OFF,0.2" CMD_TERMINATOR

namespace dg14 {
    namespace s = std;
    class Gps {
        public:
            explicit Gps();
            explicit Gps(s::string, int buad = 115200);
            
            void Init(s::string, int baud = 115200);
            void Run();
            void Stop();
            void Cancel();
            
            void RegisterCallback(boost::function<void (s::string)> onsuccess);
        private: 
            void RunLoop();
            
            boost::thread looper_;
            bool loop_flag_;
            boost::mutex mx_;
            
            boost::function<void (s::string)> onsuccess_;
            serial::Serial s_;
    };
}

#endif /* end DG14_H_ */
