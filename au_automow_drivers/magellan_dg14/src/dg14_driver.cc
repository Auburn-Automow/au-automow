#include <ros/ros.h>
#include <algorithm>
#include <iterator>
#include <time.h>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <serial.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <nav_msgs/Odometry.h>
#include <magellan_dg14/UTMFix.h>

using namespace gps_common;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace std;
namespace po = boost::program_options;

string
trim_trailing(string str) {
    string whitespaces(" \t\f\v\n\r");
    size_t found;

    found = str.find_last_not_of(whitespaces);
    if (found != string::npos)
        str.erase(found+1);
    else
        str.clear();
        
    return str;
}

class Gps {
    public:
        double          easting_origin;
        double          northing_origin;
        
        explicit Gps() {
            easting_origin = 641730;
            northing_origin = 3606766;
        }

        // explicit Gps(string port, int buad = 115200) {
        // }

        bool Init(string port, int baud = 115200) {
            /* TODO: Make sure the serial port opened properly */
            s_.setPort(port);
            s_.setBaudrate(baud);
            s_.setTimeoutMilliseconds(250);
            s_.open();
            utc_time = 0.0;
            testing = false;

            gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
            utm_fix_pub = node.advertise<magellan_dg14::UTMFix>("utm_fix", 1);
            navsat_fix_pub = node.advertise<NavSatFix>("/gps/fix", 1);
            gps_odom_pub = node.advertise<Odometry>("/gps/odometry",1);
            gps_timer = node.createTimer(ros::Duration(1.0/5.0), &Gps::publish_callback, this);
            return true;
        }
        
        bool InitTesting() {
            testing = true;
            return testing;
        }

        void Stop() {
            s_.close();
        }

        void Cancel() {
            Stop();
        }
        
        void Step() {
            string line = s_.read_until('\n');
            process_data(line);
        }
        
        void process_data(string &unparsed_tokens) {
            unparsed_tokens = trim_trailing(unparsed_tokens);
            vector<string> tokens;
            double utc_time_old = utc_time;
            size_t last_i = 0;
            for (size_t i = 0; i < unparsed_tokens.size(); i++) {
                if (unparsed_tokens[i] == ',') {
                    tokens.push_back(unparsed_tokens.substr(last_i, i - last_i));
                    last_i = i + 1;
                }
            }
            string last_with_checksum = unparsed_tokens.substr(last_i, unparsed_tokens.length() - 1);
            
            if (last_with_checksum.find('*') != string::npos) {
                if (last_with_checksum.find('*') == 0) {
                    tokens.push_back("");
                }
                else {
                    tokens.push_back(last_with_checksum.substr(0, last_with_checksum.find('*') - 1));
                }
            }
            else {
                tokens.push_back(last_with_checksum);
            }
            
            if (tokens.size() < 2) {
                return;
            }
            
            ros::Time time = ros::Time::now();
            
            status.header.stamp = time;
            fix.header.stamp = time;
            nav_fix.header.stamp = time;
            
            if (tokens[0] == "$PASHR" && 
                tokens[1] == "POS") {
                process_data_pos(tokens);
            }
            else if (tokens[0] == "$PASHR" && 
                     tokens[1] == "SAT") {
                process_data_sat(tokens);
            }
            else if (tokens[0] == "$PASHR" && 
                     tokens[1] == "UTM") {
                process_data_utm(tokens);
            }
            else if (tokens[0] == "$GPGST") {
                process_data_gst(tokens);
            }
            else {
                cout << "Shouldn't happen" << endl;
                cout << tokens[0] << ":" << tokens[1] << endl;
            }
            
            fix.status = status;
            
            // publish
            if (testing) {
                if (utc_time != utc_time_old)
                    cout << "Test: Publishing msg" << endl;
            }
        }
        
        void publish_callback(const ros::TimerEvent& e) {
            gps_fix_pub.publish(fix); // Its a new pos.
            navsat_fix_pub.publish(nav_fix);
        }
    private:
        
        void process_data_utm(vector<string> &tokens) {
            magellan_dg14::UTMFix utm_fix;
            utm_fix.header.stamp = ros::Time::now();
            utm_fix.header.frame_id = "gps_link";
            utm_fix.utc_time = strtod(tokens[2].c_str(), NULL);
            utm_fix.utm_zone = tokens[3];
            utm_fix.easting = strtod(tokens[4].c_str(), NULL);
            utm_fix.northing = strtod(tokens[5].c_str(), NULL);
            /* status msgs 
             * int16 FIX_TYPE_NONE = 0
             * int16 FIX_TYPE_RAW = 1
             * int16 FIX_TYPE_FLOAT = 2
             * int16 FIX_TYPE_FIXED = 3
             */
            int mode = atoi(tokens[6].c_str());
            utm_fix.fix_type = mode;
            // if (mode == 0) {
            //     utm_fix.fix_type = 0;
            // }
            // else if (mode == 1) {
            //     utm_fix.fix_type = 1;
            // }
            // else if (mode == 2) {
            //     utm_fix.fix_type = 2;
            // }
            // else if (mode == 3) {
            //     utm_fix.fix_type = 3;
            // }
            // else if (mode == 4) {
            //     utm_fix.fix_type = 4;
            // }
            // else {
            //     utm_fix.fix_type = -1;
            // }
            utm_fix.num_satellites = atoi(tokens[7].c_str());
            utm_fix.hdop = strtod(tokens[8].c_str(), NULL);
            utm_fix.antenna_height = strtod(tokens[9].c_str(), NULL);
            utm_fix.geoidal_separation = strtod(tokens[11].c_str(), NULL);
            utm_fix.differential_corrections_age = atoi(tokens[13].c_str());
            utm_fix.differential_station_id = tokens[14];
            utm_fix.position_covariance = fix.position_covariance;
            utm_fix_pub.publish(utm_fix);

            gps_odom.header.stamp = utm_fix.header.stamp;
            gps_odom.header.frame_id = "base_footprint";
            gps_odom.pose.pose.position.x = utm_fix.easting - easting_origin;
            gps_odom.pose.pose.position.y = utm_fix.northing - northing_origin;
            gps_odom.pose.pose.position.z = 0;
            gps_odom.pose.pose.orientation.x = 1;
            gps_odom.pose.pose.orientation.y = 0;
            gps_odom.pose.pose.orientation.z = 0;
            gps_odom.pose.pose.orientation.w = 0;
            
            // Use ENU covariance to build XYZRPY covariance
            boost::array<double, 36> covariance = {{
              utm_fix.position_covariance[0],
              utm_fix.position_covariance[1],
              utm_fix.position_covariance[2],
              0, 0, 0,
              utm_fix.position_covariance[3],
              utm_fix.position_covariance[4],
              utm_fix.position_covariance[5],
              0, 0, 0,
              utm_fix.position_covariance[6],
              utm_fix.position_covariance[7],
              utm_fix.position_covariance[8],
              0, 0, 0,
              0, 0, 0, 99999, 0, 0,
              0, 0, 0, 0, 99999, 0,
              0, 0, 0, 0, 0, 99999 
            }};

            gps_odom.pose.covariance = covariance;

            gps_odom_pub.publish(gps_odom);
        }
        
        void process_data_pos(vector<string> &tokens) {
            /* status msgs 
             * int16 STATUS_NO_FIX=-1   # Unable to fix position
             * int16 STATUS_FIX=0       # Normal fix
             * int16 STATUS_SBAS_FIX=1  # Fixed using a satellite-based augmentation system
             * int16 STATUS_GBAS_FIX=2  #          or a ground-based augmentation system
             * int16 STATUS_DGPS_FIX=18 # Fixed with DGPS
             * int16 STATUS_WAAS_FIX=33 # Fixed with WAAS
             */
            int mode = atoi(tokens[2].c_str());
            if (mode == 0) {
                status.status = 0;
                nav_fix.status.status = NavSatStatus::STATUS_FIX;
            }
            else if (mode == 1) {
                status.status = 1;
                nav_fix.status.status = NavSatStatus::STATUS_GBAS_FIX;
            }
            else if (mode == 2) {
                status.status = 2;
            }
            else if (mode == 3) {
                status.status = 18;
            }
            else {
                status.status = -1;
            }
            nav_fix.status.service = NavSatStatus::SERVICE_GPS;
            
            utc_time = strtod(tokens[4].c_str(), NULL);
            
            fix.latitude = strtod(tokens[5].c_str(), NULL);
            nav_fix.latitude = fix.latitude;
            fix.longitude = strtod(tokens[7].c_str(), NULL);
            nav_fix.longitude = fix.longitude;
            fix.altitude = strtod(tokens[9].c_str(), NULL);
            nav_fix.altitude = fix.altitude;
            if (tokens[6] == "S") // If southern hemisphere, it needs to be negative
                fix.latitude *= -1;
            if (tokens[8] == "W") // If West of Prime meridian, it needs to be negative
                fix.longitude *= -1;
            fix.track = strtod(tokens[11].c_str(), NULL);
            fix.speed = 0.5144 * strtod(tokens[12].c_str(), NULL); // DG14 returns knots, gps message expects mps
            fix.climb = strtod(tokens[13].c_str(), NULL);
            // fix.pitch;
            // fix.roll;
            // fix.dip;
            fix.time = strtod(tokens[4].c_str(), NULL);
            fix.pdop = strtod(tokens[14].c_str(), NULL);
            fix.hdop = strtod(tokens[15].c_str(), NULL);
            fix.vdop = strtod(tokens[16].c_str(), NULL);
            fix.tdop = strtod(tokens[17].c_str(), NULL);
            fix.gdop = sqrt(pow(fix.pdop, 2.0) + pow(fix.tdop, 2.0));
            // fix.err;
            // fix.err_horz;
            // fix.err_vert;
            // fix.err_track;
            // fix.err_speed;
            // fix.err_time;
            // fix.err_pitch;
            // fix.err_roll;
            // fix.err_dip;
        }
        
        void process_data_gst(vector<string> &tokens) {
            fix.position_covariance[0] = strtod(tokens[6].c_str(), NULL);
            fix.position_covariance[4] = strtod(tokens[7].c_str(), NULL);
            fix.position_covariance[8] = strtod(tokens[8].c_str(), NULL);
            fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            utc_time = strtod(tokens[1].c_str(), NULL);
            nav_fix.position_covariance[0] = fix.position_covariance[0];
            nav_fix.position_covariance[4] = fix.position_covariance[4];
            nav_fix.position_covariance[8] = fix.position_covariance[8];
            nav_fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        }

        void process_data_sat(vector<string> &tokens) {
            if (tokens.size() < 8)
                return;
            size_t sat_num = atoi(tokens[2].c_str());
            vector<vector<string> > used_tokens;
            for (int i = 1; i < sat_num; i++) {
                if ((tokens.size() >= (3 + (5 * i))) && (!tokens[3 + (5 * i)].empty())) {
                    vector<string> tmp(5, "");
                    tmp[0] = tokens[(3 + (1 * i))];
                    tmp[1] = tokens[(3 + (2 * i))];
                    tmp[2] = tokens[(3 + (3 * i))];
                    tmp[3] = tokens[(3 + (4 * i))];
                    tmp[4] = tokens[(3 + (5 * i))];
                    used_tokens.push_back(tmp);
                }
            }
            
            status.satellites_visible = used_tokens.size();
            
            status.satellite_visible_prn.resize(status.satellites_visible);
            status.satellite_visible_z.resize(status.satellites_visible);
            status.satellite_visible_azimuth.resize(status.satellites_visible);
            status.satellite_visible_snr.resize(status.satellites_visible);
            
            for (int i = 0; i < used_tokens.size(); i++) {                
                status.satellite_visible_prn[i] = atoi(used_tokens[i][0].c_str());
                status.satellite_visible_z[i] = atoi(used_tokens[i][2].c_str());
                status.satellite_visible_azimuth[i] = atoi(used_tokens[i][1].c_str());
                status.satellite_visible_snr[i] = atoi(used_tokens[i][3].c_str());
            }
        }
        
        ros::NodeHandle node;
        ros::NodeHandle privnode;
        ros::Publisher  gps_fix_pub;
        ros::Publisher  utm_fix_pub;
        ros::Publisher  navsat_fix_pub;
        ros::Publisher  gps_odom_pub;
        serial::Serial  s_;
        ros::Timer      gps_timer;
        GPSStatus       status;
        GPSFix          fix;
        NavSatFix       nav_fix;
        Odometry        gps_odom;
        bool            testing;
        double          utc_time;
};

int main (int argc, char *argv[]) {
    ros::init(argc, argv, "dg14_driver");
    
    Gps gps;
    
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "Displays this message")
        ("src", po::value<string>(), "Set the serial source, defaults to /dev/gps")
        ("baud", po::value<int>(), "Set the baud rate, defaults to 115200")
        ("test", po::value<string>(), "Sets a test")
    ;
    string src = "/dev/gps";
    string test = "";
    int baud = 115200;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }
    
    if (vm.count("test")) {
        test = vm["test"].as<string>();
    }
    
    if (test == "") {
        if (vm.count("src")) {
            src = vm["src"].as<string>();
        }

        if (vm.count("baud")) {
            baud = vm["baud"].as<int>();
        }
        
        if (ros::param::has("src")) {
            ros::param::get("src", src);
        }

        if (ros::param::has("baud")) {
            ros::param::get("baud", baud);
        }
        if (ros::param::has("northing_origin")) {
            ros::param::get("northing_origin",gps.northing_origin);
        }
         if (ros::param::has("easting_origin")) {
            ros::param::get("easting_origin",gps.easting_origin);
        }

        gps.Init(src, baud);
        // gps_timer.start();
        while (ros::ok()) {
            ros::spinOnce();
            gps.Step();
        }
        gps.Stop();
    }
    else {
        gps.InitTesting();
        ifstream ifile;
        ifile.open(test.c_str(), ifstream::in);
        while (ifile.good()) {
            string line;
            getline(ifile, line);
            gps.process_data(line);
        }
    }
    
    return 0;
}
