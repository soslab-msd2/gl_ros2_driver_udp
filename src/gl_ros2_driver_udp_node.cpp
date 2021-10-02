
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "gl_driver.h"


using namespace std::chrono_literals;


class GLRos2DriverUdp : public rclcpp::Node
{
public:
    GLRos2DriverUdp() : Node("gl_ros2_driver_udp_node")
    {
        DeclareParam();
        GetParam();
        
        laser_pub = create_publisher<sensor_msgs::msg::LaserScan>(pub_topicname_lidar, 10);
        timer = create_wall_timer(12ms, std::bind(&GLRos2DriverUdp::TimerCallback, this));

        InitGL();
    }

    ~GLRos2DriverUdp()
    {
        gl->SetFrameDataEnable(false);
        delete gl;
    }


private:
    /************************** Functions *****************************/
    void DeclareParam(void);
    void GetParam(void);

    void InitGL(void);
    void PubLidar(const gldriver::GL::framedata_t& frame_data);

    void TimerCallback(void);

private:
    /************************** Launch variables *****************************/
    std::string gl_ip;
    int gl_port;
    int pc_port;
    std::string frame_id;
    std::string pub_topicname_lidar;
    double angle_offset;
    
private:
    /************************** Other variables *****************************/
    gldriver::GL* gl;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
    rclcpp::TimerBase::SharedPtr timer;
};

void GLRos2DriverUdp::DeclareParam(void)
{
    declare_parameter("gl_ip", gl_ip);
    declare_parameter("gl_port", gl_port);
    declare_parameter("pc_port", pc_port);
    declare_parameter("frame_id", frame_id);
    declare_parameter("pub_topicname_lidar", pub_topicname_lidar);
    declare_parameter("angle_offset", angle_offset);
}

void GLRos2DriverUdp::GetParam(void)
{
    gl_ip = get_parameter("gl_ip").as_string();
    gl_port = get_parameter("gl_port").as_int();
    pc_port = get_parameter("pc_port").as_int();
    frame_id = get_parameter("frame_id").as_string();
    pub_topicname_lidar = get_parameter("pub_topicname_lidar").as_string();
    angle_offset = get_parameter("angle_offset").as_double();
}

void GLRos2DriverUdp::InitGL(void)
{
    gl = new gldriver::GL(gl_ip, gl_port, pc_port);
    std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
    gl->SetFrameDataEnable(true);
}

void GLRos2DriverUdp::PubLidar(const gldriver::GL::framedata_t& frame_data) 
{
    int num_lidar_data = frame_data.distance.size();
    
    if(num_lidar_data>0)
    {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = rclcpp::Clock().now();
        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = frame_data.angle[0] + angle_offset*3.141592/180.0;
        scan_msg.angle_max = frame_data.angle[num_lidar_data-1] + angle_offset*3.141592/180.0;
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_lidar_data-1);
        scan_msg.range_min = 0.001;
        scan_msg.range_max = 30.0;
        scan_msg.ranges.resize(num_lidar_data);
	    scan_msg.intensities.resize(num_lidar_data);

        for(size_t i=0; i<num_lidar_data; i++)
        {
            scan_msg.ranges[i] = frame_data.distance[i];
            scan_msg.intensities[i] = frame_data.pulse_width[i];
        }
        laser_pub->publish(scan_msg);
    }
}

void GLRos2DriverUdp::TimerCallback(void)
{
    gldriver::GL::framedata_t frame_data;
    gl->ReadFrameData(frame_data);
    if(frame_data.distance.size()>0) PubLidar(frame_data);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GLRos2DriverUdp>());
    rclcpp::shutdown();
   
    return 0;
}
