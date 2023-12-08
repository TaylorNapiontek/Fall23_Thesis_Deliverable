#include <rclcpp/rclcpp.hpp>
#include "gps_package/msg/gps_message.hpp"
// #include "/home/ws/matplotlibcpp.h" 
// namespace plt = matplotlibcpp;

using std::placeholders::_1;

// start data 
std::vector<double> latitudes;
std::vector<double> longitudes;

void plotGPSData(){
//     plt::scatter(latitudes,longitudes);
//     plt::title("GPS Data Plot");
//     plt::xlabel("Latitude");
//     plt::ylabel("Longitude");
//     plt::show();
}

class GPSSubscriber : public rclcpp::Node
{
  public:
    GPSSubscriber() : Node("gps_subscriber")
    {
      subscription_ = this->create_subscription<gps_package::msg::GpsMessage>(
      "gpsData", 10, std::bind(&GPSSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const gps_package::msg::GpsMessage::SharedPtr msg) const
    {
      latitudes.push_back(msg->latitude);
      longitudes.push_back(msg->longitude);
      RCLCPP_INFO(this->get_logger(), "I heard Latitude: '%f' Longitude: '%f'", msg->latitude, msg->longitude);
    }

    rclcpp::Subscription<gps_package::msg::GpsMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSSubscriber>());
  rclcpp::shutdown();

  plotGPSData();
  return 0;
}

