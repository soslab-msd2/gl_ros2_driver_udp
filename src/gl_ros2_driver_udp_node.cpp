
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "gl_driver.h"


using namespace std::chrono_literals;


class GlRos2DriverUdp : public rclcpp::Node
{
public:
    GlRos2DriverUdp() : Node("gl_ros2_driver_udp_node")
    {
        DeclareParam();
        GetParam();
        
        laser_pub = create_publisher<sensor_msgs::msg::LaserScan>(pub_topicname_lidar, 10);
        timer = create_wall_timer(12ms, std::bind(&GlRos2DriverUdp::TimerCallback, this));

        InitGl();
    }

    ~GlRos2DriverUdp()
    {
        gl->SetFrameDataEnable(false);
    }


private:
    /************************** Functions *****************************/
    void DeclareParam(void);
    void GetParam(void);

    void TimerCallback(void);

    void InitGl(void);
    void PubLidar(const Gl::framedata_t& frame_data);

private:
    /************************** Launch variables *****************************/
    std::string gl_ip = "10.110.1.2";
    int gl_port = 2000;
    int pc_port = 3000;
    std::string frame_id = "laser";
    std::string pub_topicname_lidar = "scan";
    double angle_offset = 0.0;
    
private:
    /************************** Other variables *****************************/
    Gl* gl;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
    rclcpp::TimerBase::SharedPtr timer;
};

void GlRos2DriverUdp::TimerCallback(void)
{
    Gl::framedata_t frame_data;
    gl->ReadFrameData(frame_data);
    if(frame_data.distance.size()>0) PubLidar(frame_data);
}

void GlRos2DriverUdp::DeclareParam(void)
{
    declare_parameter("gl_ip", gl_ip);
    declare_parameter("gl_port", gl_port);
    declare_parameter("pc_port", pc_port);
    declare_parameter("frame_id", frame_id);
    declare_parameter("pub_topicname_lidar", pub_topicname_lidar);
    declare_parameter("angle_offset", angle_offset);
}

void GlRos2DriverUdp::GetParam(void)
{
    gl_ip = get_parameter("gl_ip").as_string();
    gl_port = get_parameter("gl_port").as_int();
    pc_port = get_parameter("pc_port").as_int();
    frame_id = get_parameter("frame_id").as_string();
    pub_topicname_lidar = get_parameter("pub_topicname_lidar").as_string();
    angle_offset = get_parameter("angle_offset").as_double();
}

void GlRos2DriverUdp::InitGl(void)
{
    gl = new Gl(gl_ip, gl_port, pc_port);
    std::cout << "Serial Num : " << gl->GetSerialNum() << std::endl;
    gl->SetFrameDataEnable(true);
}

void GlRos2DriverUdp::PubLidar(const Gl::framedata_t& frame_data) 
{
    int num_lidar_data = frame_data.distance.size();
    
    if(num_lidar_data>0)
    {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = rclcpp::Clock().now();
        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = frame_data.angle[0] + angle_offset;
        scan_msg.angle_max = frame_data.angle[num_lidar_data-1] + angle_offset;
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(num_lidar_data-1);
        scan_msg.range_min = 0.1;
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlRos2DriverUdp>());
    rclcpp::shutdown();
   
    return 0;
}
