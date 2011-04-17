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
#include <serial.h>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

using namespace gps_common;
using namespace sensor_msgs;
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
        explicit Gps() {
        }

        // explicit Gps(string port, int buad = 115200) {
        // }

        bool Init(string port, int baud = 115200) {
            /* TODO: Make sure the serial port opened properly */
            s_.setPort(port);
            s_.setBaudrate(baud);
            s_.setTimeoutMilliseconds(250);
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
            string s_pos = s_.read_until('\n');
            string s_sat = s_.read_until('\n');
            process_data(s_pos, s_sat);
        }
        
        void process_data(string s_pos, string s_sat) {
            s_pos = trim_trailing(s_pos);
            s_sat = trim_trailing(s_sat);
            vector<string> pos_tokens;
            vector<string> sat_tokens;
            size_t last_i = 0;
            for (size_t i = 0; i < s_pos.size(); i++) {
                if (s_pos[i] == ',') {
                    pos_tokens.push_back(s_pos.substr(last_i, i - last_i));
                    last_i = i;
                }
            }
            last_i = 0;
            for (size_t i = 0; i < s_sat.size(); i++) {
                if (s_sat[i] == ',') {
                    sat_tokens.push_back(s_sat.substr(last_i, i - last_i));
                    last_i = i;
                }
            }

            if (sat_tokens.size() > 2 && 
                sat_tokens[0] == "$PASHR" && 
                sat_tokens[1] == "POS") {
                swap(sat_tokens, pos_tokens);
            }
            
            ros::Time time = ros::Time::now();
            
            GPSStatus status;
            GPSFix fix;
            
            status.header.stamp = time;
            fix.header.stamp = time;
            
            process_data_gps(pos_tokens, fix);
            process_data_sat(sat_tokens, status);
            
            fix.status = status;
            
            // publish
            if (!testing)
                gps_fix_pub.publish(fix);
        }
    private:

        void process_data_gps(vector<string> &tokens, GPSFix &fix) {
            fix.latitude = strtod(tokens[5].c_str(), NULL);
            fix.longitude = strtod(tokens[7].c_str(), NULL);
            fix.track = strtod(tokens[11].c_str(), NULL);
            fix.speed = strtod(tokens[12].c_str(), NULL); // TODO: In knots, convert to mps
            fix.climb = strtod(tokens[13].c_str(), NULL);
            // fix.pitch;
            // fix.roll;
            // fix.dip;
            fix.time = strtod(tokens[4].c_str(), NULL);
            // fix.gdop;
            fix.pdop = strtod(tokens[14].c_str(), NULL);
            fix.hdop = strtod(tokens[15].c_str(), NULL);
            fix.vdop = strtod(tokens[16].c_str(), NULL);
            fix.tdop = strtod(tokens[17].c_str(), NULL);
            // fix.err;
            // fix.err_horz;
            // fix.err_vert;
            // fix.err_track;
            // fix.err_speed;
            // fix.err_time;
            // fix.err_pitch;
            // fix.err_roll;
            // fix.err_dip;
            // fix.position_covariance;
            // fix.position_covariance_type;
        }

        void process_data_sat(vector<string> &tokens, GPSStatus &status) {
            if (tokens.size() < 8)
                return;
            size_t sat_num = atoi(tokens[2].c_str());
            vector<vector<string> > used_tokens; 
            for (int i = 1; i = sat_num; i++) {
                if ((tokens.size() >= (3 + (5 * i))) && (!tokens[3 + (5 * i)].empty())) {
                    vector<string> tmp;
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
            
            if (used_tokens.size() > 0) {
                status.status = 2; // STATUS_GBAS_FIX
            }
            
            for (int i = 0; i < used_tokens.size(); i++) {                
                status.satellite_visible_prn[i] = atoi(used_tokens[i][0].c_str());
                status.satellite_visible_z[i] = atoi(used_tokens[i][2].c_str());
                status.satellite_visible_azimuth[i] = atoi(used_tokens[i][1].c_str());
                status.satellite_visible_snr[i] = atoi(used_tokens[i][3].c_str());
            }
        }
        
        ros::NodeHandle node;
        ros::NodeHandle privnode;
        ros::Publisher gps_fix_pub;
        ros::Publisher navsat_fix_pub;
        serial::Serial s_;
        bool testing;
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
        
        gps.Init(src, baud);
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
            string line_pos, line_sat;
            getline(ifile, line_pos);
            getline(ifile, line_sat);
            gps.process_data(line_pos, line_sat);
        }
    }
    
    return 0;
}
