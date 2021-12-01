/*
* C++ version of /Payload\ Computer/cal_sequence_tcp_server_v2.py
* 
* author: KM
* date: 13th Oct 2021
*/
#include "ros/ros.h"    // for checking ROS flags
#include <stdio.h>
#include <string.h>
#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <iostream>
#include <cstring>
#include <string>
#include <time.h>

// Define namespaces
using namespace std;
using namespace ros;

// Global variables
string telem_source        = "/dev/ttyTELEM";
string heartbeat_check     = "hrt_beat";            // heartbeat every n secs
string heartbeat_conf      = "OK_hrtbt";            // heartbeat confirmation
string startup_initiate    = "pay_INIT";            // check to see if payload is running
string startup_confirm     = "INITconf";            // confirmation msg from payload if running
string handshake_start     = "is_comms";            // begin handshake prior to save data
string handshake_conf      = "serialOK";            // confirmation from payload before save
string toggle_ON           = "start_tx";            // message to payload to start cal                
string toggle_OFF          = "stop_acq";            // message from payload to stop saving
string stop_acq_conf       = "confSTOP";            // confirm that acquisition has stopped
string no_data_tx          = "no__data";            // tell base no cal was done here
string shutdown            = "shutdown";            // force shutdown of all SDRs
string reboot_payload      = "_reboot_";            // reboot payload computer
string pingtest            = "pingtest";            // manually test connection
string update_wp           = "updateWP";            // manual update to WP table
string restart_wp_node     = "rswpnode";            // manual reset of ROS WP nodes
string client_script_name  = "gr_cal_tcp_loopback_client.py";
int togglePoint            = 96;                    // number of pulses after which GPIO is toggled
int sample_packet          = 4096*16;               // Length of one pulse.
int repeat_keyword         = 1;                     // how many times to repeat serial msg
int ser_timeout            = 30;                    // serial timeout when waiting for handshake (deciseconds)
const int buff_size        = handshake_start.length() * repeat_keyword;
bool seq_flag;


//? convert string to char array for serial transmit
char* get_char_array(string str_obj){
    char* char_arr;
    char_arr = &str_obj[0];
    return char_arr;
}

//? take input string, repeat and concatenate it n times
string create_msg(string str_obj, int repeat_keyword){
    string str_obj_a = str_obj;
    for (int i = 1; i < repeat_keyword; i++){
        str_obj_a = str_obj_a.append(str_obj);
    }
    return str_obj_a;
}


//? returns different serial object based on timeout conditions
//? vmin = number of bytes to wait for 
//? vtime = time in deciseconds to wait for
//? reference: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
int get_serial_obj(int vtime, int vmin){
    int serial_port = open(get_char_array(telem_source), O_RDWR);
    // Check for errors
    if (serial_port < 0) {
        printf("Error %i from open: %s\nSerial device not found\n", errno, strerror(errno));
    }
    else {
        printf("Serial device found\n");
    }
    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;
    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    tty.c_cflag &= ~PARENB;     // Clear parity bit, disabling parity (most common)
    //tty.c_cflag |= PARENB;    // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB;     // Clear stop field, only one stop bit used in communication (most common)
    //tty.c_cflag |= CSTOPB;    // Set stop field, two stop bits used in communication
    tty.c_cflag &= ~CRTSCTS;    // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS;   // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CS8;         // 8 bits per byte (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;       // Disable echo
    tty.c_lflag &= ~ECHOE;      // Disable erasure
    tty.c_lflag &= ~ECHONL;     // Disable new-line echo
    tty.c_lflag &= ~ISIG;       // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST;      // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;      // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS;  // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT;  // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    tty.c_cc[VTIME] = vtime;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = vmin;
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B4800);   //* set input baud rate
    cfsetospeed(&tty, B4800);   //* set output baud rate

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    return serial_port;
}

//TODO: add else statements if comms fail
//TODO: logging? or cout received messages at least
//TODO: long term: multithreading
int main(int argc, char **argv){
    int tx_ser          = get_serial_obj(1, 0);                     
    int rx_ser          = get_serial_obj(0, buff_size);    
    int rx_ser_timeout  = get_serial_obj(ser_timeout, buff_size); 
    char read_buf[8];
    ros::init(argc, argv, "cal_sequence");
    ros::NodeHandle node;
    node.setParam("trigger/sequence", false);
    while (ros::ok()){
        usleep(10e3);
        if (node.param("trigger/sequence", seq_flag) == true){
            cout << "Drone has reached WP. Sending handshake to base to begin acquisition." << endl;
            node.setParam("trigger/sequence", false);
            write(tx_ser, get_char_array(create_msg(handshake_start, repeat_keyword)), buff_size);
            int m = read(rx_ser_timeout, &read_buf, buff_size);
            if (strstr(read_buf, "serialOK")){
                cout << "Handshake confirmation received from base. Beginning calibration sequence, and saving metadata." << endl;
                node.setParam("trigger/metadata", true);
                usleep(2e6);
                write(tx_ser, get_char_array(create_msg(toggle_OFF, repeat_keyword)), buff_size);
                int n = read(rx_ser_timeout, &read_buf, buff_size);
                if (strstr(read_buf, "confSTOP")){
                    cout << "Base has stopped acquisition. Sequence complete." << endl;
                    node.setParam("trigger/metadata", false);
                    node.setParam("trigger/waypoint", true);
                }
                else{
                    cout << "No stop acq confirmation from base. serial data: " << read_buf << endl;
                    node.setParam("trigger/metadata", false);
                    node.setParam("trigger/waypoint", true);
                }
            }
            else{
                cout << "No handshake confirmation from base. CAL was NOT performed. Serial data: " << read_buf << endl;
                node.setParam("trigger/waypoint", true);

            }
        }
    }
    close(tx_ser);
    close(rx_ser);
    close(rx_ser_timeout);
    return 0;
}


