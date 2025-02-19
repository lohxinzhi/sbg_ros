#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <math.h>
#include <vector>

#include <ros/ros.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

#include "sbg_ros/sat_data.h"  
#include "sbg_ros/sbgECom_diag.h"  
#include "sbg_ros/sbgECom_ekf_euler.h" 
#include "sbg_ros/sbgECom_ekf_nav.h"
#include "sbg_ros/sbgECom_ekf_vel_body.h"  
#include "sbg_ros/sbgECom_gnss_heading.h"  
#include "sbg_ros/sbgECom_gnss_position.h"  
#include "sbg_ros/sbgECom_gnss_velocity.h"
#include "sbg_ros/sbgECom_gnss_sat.h"  
#include "sbg_ros/sbgECom_imu_data.h"  
#include "sbg_ros/sbgECom_status.h"  
#include "sbg_ros/sbgECom_utc_time.h"  
#include "sbg_ros/sig_data.h"

/* Status */
#define SBG_ECOM_LOG_STATUS_ID      (0x01u)
#define SBG_ECOM_LOG_STATUS_LEN     (0x1Bu)

/* UTC Time. */
#define SBG_ECOM_LOG_UTC_TIME_ID    (0x02u)
#define SBG_ECOM_LOG_UTC_TIME_LEN   (0x21u)

/* IMU Data */
#define SBG_ECOM_LOG_IMU_DATA_ID    (0x03u)
#define SBG_ECOM_LOG_IMU_DATA_LEN   (0x3Au)

/* EKF Euler */
#define SBG_ECOM_LOG_EKF_EULER_ID   (0x06u)
#define SBG_ECOM_LOG_EKF_EULER_LEN  (0x20u)

/* EKF Nav */
#define SBG_ECOM_LOG_EKF_NAV_ID     (0x08u)
#define SBG_ECOM_LOG_EKF_NAV_LEN    (0x48u)

/* EKF Vel Body */
#define SBG_ECOM_LOG_EKF_VEL_BODY_ID     (0x36u)
#define SBG_ECOM_LOG_EKF_VEL_BODY_LEN    (0x20u)

/* GNSS Velocity */
#define SBG_ECOM_LOG_GPS1_VEL_ID    (0x0Du)
#define SBG_ECOM_LOG_GPS1_VEL_LEN   (0x2cu)

/* GNSS Position  */
#define SBG_ECOM_LOG_GPS1_POS_ID    (0x0Eu)
#define SBG_ECOM_LOG_GPS1_POS_LEN   (0x3Eu) // 0x3B in the doc

/* GNSS True Heading */
#define SBG_ECOM_LOG_GPS1_HDT_ID    (0x0Fu)
#define SBG_ECOM_LOG_GPS1_HDT_LEN   (0x20u)

/* GNSS Satellite info */
#define SBG_ECOM_LOG_GPS1_SAT_ID    (0x32u)
// #define SBG_ECOM_LOG_GPS1_SAT_LEN   (0xu)

/* Diagnostic */
#define SBG_ECOM_LOG_DIAG_ID        (0x30u)
// #define SBG_ECOM_LOG_DIAG_LEN       (0xu)


typedef double float64_t;
typedef float float32_t;

using namespace std;

void parse_status(uint8_t * input, sbg_ros::sbgECom_status * output);
void parse_utc_time(uint8_t * input, sbg_ros::sbgECom_utc_time * output);
void parse_imu_data(uint8_t * input, sbg_ros::sbgECom_imu_data * output);
void parse_ekf_nav(uint8_t * input, sbg_ros::sbgECom_ekf_nav * output);
void parse_ekf_euler(uint8_t * input, sbg_ros::sbgECom_ekf_euler * output);
void parse_ekf_vel_body(uint8_t * input, sbg_ros::sbgECom_ekf_vel_body * output);
void parse_gnss_velocity(uint8_t * input, sbg_ros::sbgECom_gnss_velocity * output);
void parse_gnss_position(uint8_t * input, sbg_ros::sbgECom_gnss_position * output);
void parse_gnss_heading(uint8_t * input, sbg_ros::sbgECom_gnss_heading * output);
void parse_gnss_sat(uint8_t * input, sbg_ros::sbgECom_gnss_sat * output);
void parse_diag(uint8_t * input, sbg_ros::sbgECom_diag * output, uint16_t length);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekinox_micro");
    ros::NodeHandle nh("~");

    ros::Publisher utc_time_pub = nh.advertise<sbg_ros::sbgECom_utc_time>("sbg_ECom/utc_time", 1000);
    ros::Publisher imu_data_pub = nh.advertise<sbg_ros::sbgECom_imu_data>("sbg_ECom/imu_data", 1000);
    ros::Publisher ekf_nav_pub = nh.advertise<sbg_ros::sbgECom_ekf_nav>("sbg_ECom/ekf_nav", 1000);
    ros::Publisher ekf_vel_body_pub = nh.advertise<sbg_ros::sbgECom_ekf_vel_body>("sbg_ECom/ekf_vel_body", 1000);
    ros::Publisher ekf_euler_pub = nh.advertise<sbg_ros::sbgECom_ekf_euler>("sbg_ECom/ekf_euler", 1000);
    ros::Publisher gnss_heading_pub = nh.advertise<sbg_ros::sbgECom_gnss_heading>("sbg_ECom/gnss_heading", 1000);
    ros::Publisher gnss_position_pub = nh.advertise<sbg_ros::sbgECom_gnss_position>("sbg_ECom/gnss_position", 1000);
    ros::Publisher gnss_velocity_pub = nh.advertise<sbg_ros::sbgECom_gnss_velocity>("sbg_ECom/gnss_velocity", 1000);
    ros::Publisher gnss_sat_pub = nh.advertise<sbg_ros::sbgECom_gnss_sat>("sbg_ECom/gnss_sat", 1000);
    ros::Publisher status_pub = nh.advertise<sbg_ros::sbgECom_status>("sbg_ECom/status", 1000);
    ros::Publisher diag_pub = nh.advertise<sbg_ros::sbgECom_diag>("sbg_ECom/diag", 1000);
    

    string ip;
    ip = "192.168.3.200";

    int sbg_port;
    sbg_port = 50004; // tbc

    double timeout;
    timeout = 1.0;

    int sockfd[2];
    uint8_t buffer[4096];
    struct sockaddr_in server_addr;
    struct timeval tv;
    int timeout_sec = (int) timeout;
    int timeout_usec = (int) ((timeout - ((double) timeout_sec)) * 1e6);
    tv.tv_sec = (time_t) timeout_sec;
    tv.tv_usec = (suseconds_t) timeout_usec;


    int state = 0;

    ros::Rate loop_rate(200);

    while(ros::ok())
    {

        
        if (state == 0)
        {
            if ((sockfd[1] = socket(AF_INET, SOCK_STREAM, 0)) < 0)
            {
                ROS_ERROR("Failed to create socket.");
            }
            else
            {
                state = 1;
                ROS_INFO("Socket created");
            }
        }

        if(state == 1)
        {
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = inet_addr(ip.c_str());
            server_addr.sin_port = htons(sbg_port);

            if (setsockopt(sockfd[1], SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv) < 0)
            {
                ROS_ERROR("Failed to set send timeout.");
                close(sockfd[1]);
                state = 0;
            }
            else
            {
                ROS_INFO("Send timeout set");
                if (setsockopt(sockfd[1], SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0)
                {
                    ROS_ERROR("Failed to set receive timeout.");
                    close(sockfd[1]);
                    state = 0;
                }
                else
                {
                    ROS_INFO("Receive timeout set");
                    if (connect(sockfd[1], (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
                    {
                        ROS_ERROR("Failed to connect to socket.");
                        // printf("server_addr.sin_port = % \n", server_addr.sin_port);
                        close(sockfd[1]);
                        state = 0;
                    }
                    else
                    {
                        ROS_INFO("Connected to socket");
                        state = 2;
                    }
                }
            }
        }

        if(state == 2)
        {
            int n = recv(sockfd[1], buffer, 4096, 0);
            // printf("n = %d  \n", n);
            if( (n<=0) || (n > 4096))
            {
                ROS_ERROR("Failed to receive sbgECom message.");
                close(sockfd[1]);
                state = 0;
            }
            else
            {
                uint8_t sync_1;
                uint8_t sync_2;
                uint8_t msg_id;
                uint8_t msg_class;
                uint16_t length;
                uint16_t crc;
                uint8_t etx;
                uint8_t * msg;

                msg = &buffer[0];

                int count = 0;

                while(n>count)
                {
                    sync_1 = msg[count + 0];
                    sync_2 = msg[count + 1];
                    msg_id = msg[count + 2];
                    msg_class = msg[count + 3];
                    length = msg[count + 4];
                    length |= (msg[count + 5] << 8);
                    crc = msg[count + 5 + length + 1];
                    crc |= (msg[count + 5 + length + 2] << 8);
                    etx = msg[count + 5 + length + 3];

                    uint16_t check_crc = 0u;
                    uint8_t carry;

                    for (int i = 0; i < length + 4; i++)
                    {
                        check_crc = check_crc ^ msg[count+2+i];
                        for (int j = 0; j<8; j++)
                        {
                            carry = check_crc & 1;
                            check_crc   = check_crc / 2;
                        
                            if (carry)
                            {
                                check_crc = check_crc^(0x8408);
                            }
                        }
                    }

                    if ((sync_1 != 0xff) 
                        ||(sync_2 != 0x5a)
                        ||(crc != check_crc)
                        ||(etx != 0x33))
                    {
                        ROS_WARN("sbgECom message invalid, skipping to next message");
                        ROS_INFO("sync_1 = %x   ", sync_1);
                        ROS_INFO("sync_2 = %x   ", sync_2);
                        ROS_INFO("msg_id = %d   ", msg_id);
                        ROS_INFO("msg_class = %d", msg_class);
                        ROS_INFO("length = %d   ", length);
                        ROS_INFO("crc = %d      ", crc);
                        ROS_INFO("check_crc = %d",check_crc);
                        ROS_INFO("etx = %x      ", etx);
                    }

                    // ROS_INFO("sync_1 = %x   ", sync_1);
                    // ROS_INFO("sync_2 = %x   ", sync_2);
                    // ROS_INFO("msg_id = %d   ", msg_id);
                    // ROS_INFO("msg_class = %d", msg_class);
                    // ROS_INFO("length = %d   ", length);
                    // ROS_INFO("crc = %d      ", crc);
                    // ROS_INFO("check_crc = %d",check_crc);
                    // ROS_INFO("etx = %x      ", etx);
                    switch(msg_id)
                    {
                        
                        case SBG_ECOM_LOG_STATUS_ID:
                        {
                            if (length == SBG_ECOM_LOG_STATUS_LEN || true)
                            {
                                sbg_ros::sbgECom_status status;
                                parse_status(&msg[count + 6], &status);
                                status_pub.publish(status);
                                
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_UTC_TIME_ID:
                        {
                            if (length == SBG_ECOM_LOG_UTC_TIME_LEN)
                            {
                                sbg_ros::sbgECom_utc_time utc_time;
                                parse_utc_time(&msg[count + 6], &utc_time);
                                utc_time_pub.publish(utc_time);
                                
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_IMU_DATA_ID:
                        {
                            if (length == SBG_ECOM_LOG_IMU_DATA_LEN)
                            {
                                sbg_ros::sbgECom_imu_data imu_data;
                                parse_imu_data(&msg[count + 6], &imu_data);
                                imu_data_pub.publish(imu_data);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_EKF_NAV_ID:
                        {
                            if (length == SBG_ECOM_LOG_EKF_NAV_LEN)
                            {
                                sbg_ros::sbgECom_ekf_nav ekf_nav;
                                parse_ekf_nav(&msg[count + 6], &ekf_nav);
                                ekf_nav_pub.publish(ekf_nav);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_EKF_EULER_ID:
                        {
                            if (length == SBG_ECOM_LOG_EKF_EULER_LEN)
                            {
                                sbg_ros::sbgECom_ekf_euler ekf_euler;
                                parse_ekf_euler(&msg[count + 6], &ekf_euler);
                                ekf_euler_pub.publish(ekf_euler);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_EKF_VEL_BODY_ID:
                        {
                            if (length == SBG_ECOM_LOG_EKF_VEL_BODY_LEN)
                            {
                                sbg_ros::sbgECom_ekf_vel_body ekf_vel_body;
                                parse_ekf_vel_body(&msg[count + 6], &ekf_vel_body);
                                ekf_vel_body_pub.publish(ekf_vel_body);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_GPS1_POS_ID:
                        {
                            if (length == SBG_ECOM_LOG_GPS1_POS_LEN)
                            {
                                sbg_ros::sbgECom_gnss_position gnss_position;
                                parse_gnss_position(&msg[count + 6], &gnss_position);
                                gnss_position_pub.publish(gnss_position);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_GPS1_HDT_ID:
                        {
                            if (length == SBG_ECOM_LOG_GPS1_HDT_LEN)
                            {
                                sbg_ros::sbgECom_gnss_heading gnss_heading;
                                parse_gnss_heading(&msg[count + 6], &gnss_heading);
                                gnss_heading_pub.publish(gnss_heading);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_GPS1_VEL_ID:
                        {
                            if (length == SBG_ECOM_LOG_GPS1_VEL_LEN)
                            {
                                sbg_ros::sbgECom_gnss_velocity gnss_velocity;
                                parse_gnss_velocity(&msg[count + 6], &gnss_velocity);
                                gnss_velocity_pub.publish(gnss_velocity);
                            }
                            break;
                        }
                        case SBG_ECOM_LOG_DIAG_ID:
                        {
                            sbg_ros::sbgECom_diag diag;
                            parse_diag(&msg[count + 6], &diag, length);
                            diag_pub.publish(diag);
                            break;
                        }
                        case SBG_ECOM_LOG_GPS1_SAT_ID:
                        {
                            if (true)
                            {
                                sbg_ros::sbgECom_gnss_sat gnss_sat;
                                parse_gnss_sat(&msg[count + 6], &gnss_sat);
                                gnss_sat_pub.publish(gnss_sat);
                            }
                            break;
                        }


                        default:
                            break;
                    }

                    // close(sockfd[1]);
                    count += (length + 9); 

                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    close(sockfd[1]);



}

void parse_status(uint8_t * input, sbg_ros::sbgECom_status * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;   

    tmp_uint16 = (uint16_t) input[4];
    tmp_uint16 |= (uint16_t) input[5] << 8;    
    output->general_status = tmp_uint16; 
    
    tmp_uint16 = (uint16_t) input[6];
    tmp_uint16 |= (uint16_t) input[7] << 8;    
    output->com_status_2 = tmp_uint16; 

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->com_status = tmp_uint32;

    tmp_uint32 = (uint32_t) input[12];
    tmp_uint32 |= (uint32_t) input[13] << 8;
    tmp_uint32 |= (uint32_t) input[14] << 16;
    tmp_uint32 |= (uint32_t) input[15] << 24;
    output->aiding_status = tmp_uint32;

    tmp_uint32 = (uint32_t) input[22];
    tmp_uint32 |= (uint32_t) input[23] << 8;
    tmp_uint32 |= (uint32_t) input[24] << 16;
    tmp_uint32 |= (uint32_t) input[25] << 24;
    output->up_time = tmp_uint32;

    output->cpu_usage = input[26];
}

void parse_imu_data(uint8_t * input, sbg_ros::sbgECom_imu_data * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32; 

    tmp_uint16 = (uint16_t) input[4];
    tmp_uint16 |= (uint16_t) input[5] << 8;    
    output->imu_status = tmp_uint16;

    tmp_uint32 = (uint32_t) input[6];
    tmp_uint32 |= (uint32_t) input[7] << 8;
    tmp_uint32 |= (uint32_t) input[8] << 16;
    tmp_uint32 |= (uint32_t) input[9] << 24;
    output->accel_0_x = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[10];
    tmp_uint32 |= (uint32_t) input[11] << 8;
    tmp_uint32 |= (uint32_t) input[12] << 16;
    tmp_uint32 |= (uint32_t) input[13] << 24;
    output->accel_0_y = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[14];
    tmp_uint32 |= (uint32_t) input[15] << 8;
    tmp_uint32 |= (uint32_t) input[16] << 16;
    tmp_uint32 |= (uint32_t) input[17] << 24;
    output->accel_0_z = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[18];
    tmp_uint32 |= (uint32_t) input[19] << 8;
    tmp_uint32 |= (uint32_t) input[20] << 16;
    tmp_uint32 |= (uint32_t) input[21] << 24;
    output->gyro_0_x = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[22];
    tmp_uint32 |= (uint32_t) input[23] << 8;
    tmp_uint32 |= (uint32_t) input[24] << 16;
    tmp_uint32 |= (uint32_t) input[25] << 24;
    output->gyro_0_y = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[26];
    tmp_uint32 |= (uint32_t) input[27] << 8;
    tmp_uint32 |= (uint32_t) input[28] << 16;
    tmp_uint32 |= (uint32_t) input[29] << 24;
    output->gyro_0_z = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[30];
    tmp_uint32 |= (uint32_t) input[31] << 8;
    tmp_uint32 |= (uint32_t) input[32] << 16;
    tmp_uint32 |= (uint32_t) input[33] << 24;
    output->temperature = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[34];
    tmp_uint32 |= (uint32_t) input[35] << 8;
    tmp_uint32 |= (uint32_t) input[36] << 16;
    tmp_uint32 |= (uint32_t) input[37] << 24;
    output->accel_1_x = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[38];
    tmp_uint32 |= (uint32_t) input[39] << 8;
    tmp_uint32 |= (uint32_t) input[40] << 16;
    tmp_uint32 |= (uint32_t) input[41] << 24;
    output->accel_1_y = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[42];
    tmp_uint32 |= (uint32_t) input[43] << 8;
    tmp_uint32 |= (uint32_t) input[44] << 16;
    tmp_uint32 |= (uint32_t) input[45] << 24;
    output->accel_1_z = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[46];
    tmp_uint32 |= (uint32_t) input[47] << 8;
    tmp_uint32 |= (uint32_t) input[48] << 16;
    tmp_uint32 |= (uint32_t) input[49] << 24;
    output->gyro_1_x = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[50];
    tmp_uint32 |= (uint32_t) input[51] << 8;
    tmp_uint32 |= (uint32_t) input[52] << 16;
    tmp_uint32 |= (uint32_t) input[53] << 24;
    output->gyro_1_y = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[54];
    tmp_uint32 |= (uint32_t) input[55] << 8;
    tmp_uint32 |= (uint32_t) input[56] << 16;
    tmp_uint32 |= (uint32_t) input[57] << 24;
    output->gyro_1_z = *((float32_t *) &tmp_uint32);
}

void parse_utc_time(uint8_t * input, sbg_ros::sbgECom_utc_time * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32; 

    tmp_uint16 = (uint16_t) input[4];
    tmp_uint16 |= (uint16_t) input[5] << 8;    
    output->time_status = tmp_uint16;


    tmp_uint16 = (uint16_t) input[6];
    tmp_uint16 |= (uint16_t) input[7] << 8;    
    output->year = tmp_uint16;

    output->month = input[8];

    output->day = input[9];

    output->hour = input[10];
    
    output->minute = input[11];
    
    output->second = input[12];

    tmp_uint32 = (uint32_t) input[13];
    tmp_uint32 |= (uint32_t) input[14] << 8;
    tmp_uint32 |= (uint32_t) input[15] << 16;
    tmp_uint32 |= (uint32_t) input[16] << 24;
    output->nano_second = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[17];
    tmp_uint32 |= (uint32_t) input[18] << 8;
    tmp_uint32 |= (uint32_t) input[19] << 16;
    tmp_uint32 |= (uint32_t) input[20] << 24;
    output->gps_tow = tmp_uint32;     

    tmp_uint32 = (uint32_t) input[21];
    tmp_uint32 |= (uint32_t) input[22] << 8;
    tmp_uint32 |= (uint32_t) input[23] << 16;
    tmp_uint32 |= (uint32_t) input[24] << 24;
    output->clk_bias_std = *((float32_t *) &tmp_uint32);
    
    tmp_uint32 = (uint32_t) input[25];
    tmp_uint32 |= (uint32_t) input[26] << 8;
    tmp_uint32 |= (uint32_t) input[27] << 16;
    tmp_uint32 |= (uint32_t) input[28] << 24;
    output->clk_sf_error_std = *((float32_t *) &tmp_uint32);
    
    tmp_uint32 = (uint32_t) input[29];
    tmp_uint32 |= (uint32_t) input[30] << 8;
    tmp_uint32 |= (uint32_t) input[31] << 16;
    tmp_uint32 |= (uint32_t) input[32] << 24;
    output->clk_residual_err = *((float32_t *) &tmp_uint32);

}

void parse_ekf_nav(uint8_t * input, sbg_ros::sbgECom_ekf_nav * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;
    uint64_t tmp_uint64;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;  

    tmp_uint32 = (uint32_t) input[4];
    tmp_uint32 |= (uint32_t) input[5] << 8;
    tmp_uint32 |= (uint32_t) input[6] << 16;
    tmp_uint32 |= (uint32_t) input[7] << 24;
    output->velocity_n = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->velocity_e = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[12];
    tmp_uint32 |= (uint32_t) input[13] << 8;
    tmp_uint32 |= (uint32_t) input[14] << 16;
    tmp_uint32 |= (uint32_t) input[15] << 24;
    output->velocity_d = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[16];
    tmp_uint32 |= (uint32_t) input[17] << 8;
    tmp_uint32 |= (uint32_t) input[18] << 16;
    tmp_uint32 |= (uint32_t) input[19] << 24;
    output->velocity_n_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[20];
    tmp_uint32 |= (uint32_t) input[21] << 8;
    tmp_uint32 |= (uint32_t) input[22] << 16;
    tmp_uint32 |= (uint32_t) input[23] << 24;
    output->velocity_e_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[24];
    tmp_uint32 |= (uint32_t) input[25] << 8;
    tmp_uint32 |= (uint32_t) input[26] << 16;
    tmp_uint32 |= (uint32_t) input[27] << 24;
    output->velocity_d_acc = *((float32_t *) &tmp_uint32);

    tmp_uint64 = (uint64_t) input[28];
    tmp_uint64 |= (uint64_t) input[29] << 8;
    tmp_uint64 |= (uint64_t) input[30] << 16;
    tmp_uint64 |= (uint64_t) input[31] << 24;
    tmp_uint64 |= (uint64_t) input[32] << 32;
    tmp_uint64 |= (uint64_t) input[33] << 40;
    tmp_uint64 |= (uint64_t) input[34] << 48;
    tmp_uint64 |= (uint64_t) input[35] << 56;
    output->latitude = *((float64_t *) &tmp_uint64);

    tmp_uint64 = (uint64_t) input[36];
    tmp_uint64 |= (uint64_t) input[37] << 8;
    tmp_uint64 |= (uint64_t) input[38] << 16;
    tmp_uint64 |= (uint64_t) input[39] << 24;
    tmp_uint64 |= (uint64_t) input[40] << 32;
    tmp_uint64 |= (uint64_t) input[41] << 40;
    tmp_uint64 |= (uint64_t) input[42] << 48;
    tmp_uint64 |= (uint64_t) input[43] << 56;
    output->longitude = *((float64_t *) &tmp_uint64);

    tmp_uint64 = (uint64_t) input[44];
    tmp_uint64 |= (uint64_t) input[45] << 8;
    tmp_uint64 |= (uint64_t) input[46] << 16;
    tmp_uint64 |= (uint64_t) input[47] << 24;
    tmp_uint64 |= (uint64_t) input[48] << 32;
    tmp_uint64 |= (uint64_t) input[49] << 40;
    tmp_uint64 |= (uint64_t) input[50] << 48;
    tmp_uint64 |= (uint64_t) input[51] << 56;
    output->altitude = *((float64_t *) &tmp_uint64);

    tmp_uint32 = (uint32_t) input[52];
    tmp_uint32 |= (uint32_t) input[53] << 8;
    tmp_uint32 |= (uint32_t) input[54] << 16;
    tmp_uint32 |= (uint32_t) input[55] << 24;
    output->undulation = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[56];
    tmp_uint32 |= (uint32_t) input[57] << 8;
    tmp_uint32 |= (uint32_t) input[58] << 16;
    tmp_uint32 |= (uint32_t) input[59] << 24;
    output->latitude_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[60];
    tmp_uint32 |= (uint32_t) input[61] << 8;
    tmp_uint32 |= (uint32_t) input[62] << 16;
    tmp_uint32 |= (uint32_t) input[63] << 24;
    output->longitude_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[64];
    tmp_uint32 |= (uint32_t) input[65] << 8;
    tmp_uint32 |= (uint32_t) input[66] << 16;
    tmp_uint32 |= (uint32_t) input[67] << 24;
    output->altitude_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[68];
    tmp_uint32 |= (uint32_t) input[69] << 8;
    tmp_uint32 |= (uint32_t) input[70] << 16;
    tmp_uint32 |= (uint32_t) input[71] << 24;
    output->solution_status = tmp_uint32; 
}

void parse_ekf_euler(uint8_t * input, sbg_ros::sbgECom_ekf_euler * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[4];
    tmp_uint32 |= (uint32_t) input[5] << 8;
    tmp_uint32 |= (uint32_t) input[6] << 16;
    tmp_uint32 |= (uint32_t) input[7] << 24;
    output->roll = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->pitch = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[12];
    tmp_uint32 |= (uint32_t) input[13] << 8;
    tmp_uint32 |= (uint32_t) input[14] << 16;
    tmp_uint32 |= (uint32_t) input[15] << 24;
    output->yaw = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[16];
    tmp_uint32 |= (uint32_t) input[17] << 8;
    tmp_uint32 |= (uint32_t) input[18] << 16;
    tmp_uint32 |= (uint32_t) input[19] << 24;
    output->roll_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[20];
    tmp_uint32 |= (uint32_t) input[21] << 8;
    tmp_uint32 |= (uint32_t) input[22] << 16;
    tmp_uint32 |= (uint32_t) input[23] << 24;
    output->pitch_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[24];
    tmp_uint32 |= (uint32_t) input[25] << 8;
    tmp_uint32 |= (uint32_t) input[26] << 16;
    tmp_uint32 |= (uint32_t) input[27] << 24;
    output->yaw_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[28];
    tmp_uint32 |= (uint32_t) input[29] << 8;
    tmp_uint32 |= (uint32_t) input[30] << 16;
    tmp_uint32 |= (uint32_t) input[31] << 24;
    output->solution_status = tmp_uint32; 
}

void parse_ekf_vel_body(uint8_t * input, sbg_ros::sbgECom_ekf_vel_body * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[4];
    tmp_uint32 |= (uint32_t) input[5] << 8;
    tmp_uint32 |= (uint32_t) input[6] << 16;
    tmp_uint32 |= (uint32_t) input[7] << 24;
    output->solution_status = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->velocity_x = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[12];
    tmp_uint32 |= (uint32_t) input[13] << 8;
    tmp_uint32 |= (uint32_t) input[14] << 16;
    tmp_uint32 |= (uint32_t) input[15] << 24;
    output->velocity_y = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[16];
    tmp_uint32 |= (uint32_t) input[17] << 8;
    tmp_uint32 |= (uint32_t) input[18] << 16;
    tmp_uint32 |= (uint32_t) input[19] << 24;
    output->velocity_z = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[20];
    tmp_uint32 |= (uint32_t) input[21] << 8;
    tmp_uint32 |= (uint32_t) input[22] << 16;
    tmp_uint32 |= (uint32_t) input[23] << 24;
    output->velocity_x_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[24];
    tmp_uint32 |= (uint32_t) input[25] << 8;
    tmp_uint32 |= (uint32_t) input[26] << 16;
    tmp_uint32 |= (uint32_t) input[27] << 24;
    output->velocity_y_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[28];
    tmp_uint32 |= (uint32_t) input[29] << 8;
    tmp_uint32 |= (uint32_t) input[30] << 16;
    tmp_uint32 |= (uint32_t) input[31] << 24;
    output->velocity_z_acc = *((float32_t *) &tmp_uint32); 
}

void parse_gnss_velocity(uint8_t * input, sbg_ros::sbgECom_gnss_velocity * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[4];
    tmp_uint32 |= (uint32_t) input[5] << 8;
    tmp_uint32 |= (uint32_t) input[6] << 16;
    tmp_uint32 |= (uint32_t) input[7] << 24;
    output->status_type = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->tow = tmp_uint32;

    tmp_uint32 = (uint32_t) input[12];
    tmp_uint32 |= (uint32_t) input[13] << 8;
    tmp_uint32 |= (uint32_t) input[14] << 16;
    tmp_uint32 |= (uint32_t) input[15] << 24;
    output->vel_n = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[16];
    tmp_uint32 |= (uint32_t) input[17] << 8;
    tmp_uint32 |= (uint32_t) input[18] << 16;
    tmp_uint32 |= (uint32_t) input[19] << 24;
    output->vel_e = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[20];
    tmp_uint32 |= (uint32_t) input[21] << 8;
    tmp_uint32 |= (uint32_t) input[22] << 16;
    tmp_uint32 |= (uint32_t) input[23] << 24;
    output->vel_d = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[24];
    tmp_uint32 |= (uint32_t) input[25] << 8;
    tmp_uint32 |= (uint32_t) input[26] << 16;
    tmp_uint32 |= (uint32_t) input[27] << 24;
    output->vel_acc_n = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[28];
    tmp_uint32 |= (uint32_t) input[29] << 8;
    tmp_uint32 |= (uint32_t) input[30] << 16;
    tmp_uint32 |= (uint32_t) input[31] << 24;
    output->vel_acc_e = *((float32_t *) &tmp_uint32);     

    tmp_uint32 = (uint32_t) input[32];
    tmp_uint32 |= (uint32_t) input[33] << 8;
    tmp_uint32 |= (uint32_t) input[34] << 16;
    tmp_uint32 |= (uint32_t) input[35] << 24;
    output->vel_acc_d = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[36];
    tmp_uint32 |= (uint32_t) input[37] << 8;
    tmp_uint32 |= (uint32_t) input[38] << 16;
    tmp_uint32 |= (uint32_t) input[39] << 24;
    output->course = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[40];
    tmp_uint32 |= (uint32_t) input[41] << 8;
    tmp_uint32 |= (uint32_t) input[42] << 16;
    tmp_uint32 |= (uint32_t) input[43] << 24;
    output->course_acc = *((float32_t *) &tmp_uint32);
}

void parse_gnss_position(uint8_t * input, sbg_ros::sbgECom_gnss_position * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;
    uint64_t tmp_uint64;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;   

    tmp_uint32 = (uint32_t) input[4];
    tmp_uint32 |= (uint32_t) input[5] << 8;
    tmp_uint32 |= (uint32_t) input[6] << 16;
    tmp_uint32 |= (uint32_t) input[7] << 24;
    output->status_type = tmp_uint32; 

    tmp_uint32 = (uint32_t) input[8];
    tmp_uint32 |= (uint32_t) input[9] << 8;
    tmp_uint32 |= (uint32_t) input[10] << 16;
    tmp_uint32 |= (uint32_t) input[11] << 24;
    output->tow = tmp_uint32;    

    tmp_uint64 = (uint64_t) input[12];
    tmp_uint64 |= (uint64_t) input[13] << 8;
    tmp_uint64 |= (uint64_t) input[14] << 16;
    tmp_uint64 |= (uint64_t) input[15] << 24;
    tmp_uint64 |= (uint64_t) input[16] << 32;
    tmp_uint64 |= (uint64_t) input[17] << 40;
    tmp_uint64 |= (uint64_t) input[18] << 48;
    tmp_uint64 |= (uint64_t) input[19] << 56;
    output->latitude = *((float64_t *) &tmp_uint64);

    tmp_uint64 = (uint64_t) input[20];
    tmp_uint64 |= (uint64_t) input[21] << 8;
    tmp_uint64 |= (uint64_t) input[22] << 16;
    tmp_uint64 |= (uint64_t) input[23] << 24;
    tmp_uint64 |= (uint64_t) input[24] << 32;
    tmp_uint64 |= (uint64_t) input[25] << 40;
    tmp_uint64 |= (uint64_t) input[26] << 48;
    tmp_uint64 |= (uint64_t) input[27] << 56;
    output->longitude = *((float64_t *) &tmp_uint64);

    tmp_uint64 = (uint64_t) input[28];
    tmp_uint64 |= (uint64_t) input[29] << 8;
    tmp_uint64 |= (uint64_t) input[30] << 16;
    tmp_uint64 |= (uint64_t) input[31] << 24;
    tmp_uint64 |= (uint64_t) input[32] << 32;
    tmp_uint64 |= (uint64_t) input[33] << 40;
    tmp_uint64 |= (uint64_t) input[34] << 48;
    tmp_uint64 |= (uint64_t) input[35] << 56;
    output->altitude = *((float64_t *) &tmp_uint64);

    tmp_uint32 = (uint32_t) input[36];
    tmp_uint32 |= (uint32_t) input[37] << 8;
    tmp_uint32 |= (uint32_t) input[38] << 16;
    tmp_uint32 |= (uint32_t) input[39] << 24;
    output->undulation = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[40];
    tmp_uint32 |= (uint32_t) input[41] << 8;
    tmp_uint32 |= (uint32_t) input[42] << 16;
    tmp_uint32 |= (uint32_t) input[43] << 24;
    output->latitude_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[44];
    tmp_uint32 |= (uint32_t) input[45] << 8;
    tmp_uint32 |= (uint32_t) input[46] << 16;
    tmp_uint32 |= (uint32_t) input[47] << 24;
    output->longitude_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[48];
    tmp_uint32 |= (uint32_t) input[49] << 8;
    tmp_uint32 |= (uint32_t) input[50] << 16;
    tmp_uint32 |= (uint32_t) input[51] << 24;
    output->altitude_acc = *((float32_t *) &tmp_uint32);

    output->num_sv_used = input[52];

    tmp_uint16 = (uint16_t) input[53];
    tmp_uint16 |= (uint16_t) input[54] << 8;
    output->base_station_id = tmp_uint16;

    tmp_uint16 = (uint16_t) input[55];
    tmp_uint16 |= (uint16_t) input[56] << 8;
    output->diff_age = tmp_uint16;

    output->num_sv_tracked = input[57];

    tmp_uint32 = (uint32_t) input[58];
    tmp_uint32 |= (uint32_t) input[59] << 8;
    tmp_uint32 |= (uint32_t) input[60] << 16;
    tmp_uint32 |= (uint32_t) input[61] << 24;
    output->status_type = tmp_uint32; 
}

void parse_gnss_heading(uint8_t * input, sbg_ros::sbgECom_gnss_heading * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;
    uint64_t tmp_uint64;


    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;   

    tmp_uint16 = (uint16_t) input[4];
    tmp_uint16 |= (uint16_t) input[5] << 8;
    output->status = tmp_uint16; 

    tmp_uint32 = (uint32_t) input[6];
    tmp_uint32 |= (uint32_t) input[7] << 8;
    tmp_uint32 |= (uint32_t) input[8] << 16;
    tmp_uint32 |= (uint32_t) input[9] << 24;
    output->tow = tmp_uint32;

    tmp_uint32 = (uint32_t) input[10];
    tmp_uint32 |= (uint32_t) input[11] << 8;
    tmp_uint32 |= (uint32_t) input[12] << 16;
    tmp_uint32 |= (uint32_t) input[13] << 24;
    output->true_heading = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[14];
    tmp_uint32 |= (uint32_t) input[15] << 8;
    tmp_uint32 |= (uint32_t) input[16] << 16;
    tmp_uint32 |= (uint32_t) input[17] << 24;
    output->true_heading_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[18];
    tmp_uint32 |= (uint32_t) input[19] << 8;
    tmp_uint32 |= (uint32_t) input[20] << 16;
    tmp_uint32 |= (uint32_t) input[21] << 24;
    output->pitch = *((float32_t *) &tmp_uint32);     

    tmp_uint32 = (uint32_t) input[22];
    tmp_uint32 |= (uint32_t) input[23] << 8;
    tmp_uint32 |= (uint32_t) input[24] << 16;
    tmp_uint32 |= (uint32_t) input[25] << 24;
    output->pitch_acc = *((float32_t *) &tmp_uint32);

    tmp_uint32 = (uint32_t) input[26];
    tmp_uint32 |= (uint32_t) input[27] << 8;
    tmp_uint32 |= (uint32_t) input[28] << 16;
    tmp_uint32 |= (uint32_t) input[29] << 24;
    output->baseline = *((float32_t *) &tmp_uint32);

    output->num_sv_tracked = input[30];

    output->num_sv_used = input[31];
   

}

void parse_gnss_sat(uint8_t * input, sbg_ros::sbgECom_gnss_sat * output)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;
    string tmp_string;

    

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;

    output->num_satellites = input[8];

    std::vector<sbg_ros::sat_data> sat_datas;

    int count = 9;

    for(int i = 0; i<output->num_satellites; i++)
    {
        sbg_ros::sat_data sat_data;

        sat_data.satellite_id = input[count++];

        sat_data.elevation = *((int8_t *) &input[count++]);

        tmp_uint16 = (uint16_t) input[count++];
        tmp_uint16 |= (uint16_t) input[count++] << 8;
        sat_data.azimuth = tmp_uint16;

        tmp_uint16 = (uint16_t) input[count++];
        tmp_uint16 |= (uint16_t) input[count++] << 8;
        sat_data.sat_flags = tmp_uint16;

        sat_data.num_signals = input[count++];

        std::vector<sbg_ros::sig_data> sig_datas;

        for (int j=0; j<sat_data.num_signals; j++)
        {
            sbg_ros::sig_data sig_data;

            sig_data.signal_id = input[count++];

            sig_data.signal_flag = input[count++];

            sig_data.snr = input[count++];

            sig_datas.push_back(sig_data);
        }

        sat_data.signal_data = sig_datas;

        sat_datas.push_back(sat_data);
    }

    output->satellite_data = sat_datas;
}



void parse_diag(uint8_t * input, sbg_ros::sbgECom_diag * output, uint16_t length)
{
    uint16_t tmp_uint16;
    uint32_t tmp_uint32;
    string tmp_string;

    tmp_uint32 = (uint32_t) input[0];
    tmp_uint32 |= (uint32_t) input[1] << 8;
    tmp_uint32 |= (uint32_t) input[2] << 16;
    tmp_uint32 |= (uint32_t) input[3] << 24;
    output->time_stamp = tmp_uint32;

    output->type = input[4];

    output->error_code = input[5];

    for(int i = 6; i < length; i++ )
    {
        tmp_string.push_back((char)input[i]);
        // ROS_INFO("%c", input[i]);
    }

    output->message = tmp_string;

}

