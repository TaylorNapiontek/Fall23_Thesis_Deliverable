#include <rclcpp/rclcpp.hpp>
#//include "std_msgs/msg/float64.hpp"
#include "gps_package/msg/gps_message.hpp"
#include <string>
#include <iostream>
#include "serial_driver/serial_driver.hpp"

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

using namespace std::literals;

void parseNMEAMessage(const std::string& nmeaMessage, gps_package::msg::GpsMessage& gpsData)
{
    std::istringstream
iss(nmeaMessage);
    std::string messageType;
    double latitude, longitude, altitude;

    iss >> messageType; //If NMEA msg starts with a message type 

    if (messageType == "$GPGGA") {
        // GGA Message Format:
        // $GPGGA,hhmmss.ll,N,mmmm.dddd,W,Fix,numsats,HDOP,altitude,M

        iss.ignore(7); //ignore message type 
        iss >> latitude;
        char northSouth; // N or S?
        iss >> northSouth;
        if (northSouth == 'S'){
            latitude = -latitude;
        }
        iss.ignore(1); // comma 
        iss >> longitude;
        char eastWest; // E or W ?
        iss >> eastWest;
        if (eastWest == 'W'){
            longitude = -longitude;
        }
        iss.ignore(1); // comma
        iss.ignore(1); // fix
        iss.ignore(1); // comma 
        iss.ignore(2); // number of sats 
        iss.ignore(1); // comma 
        iss.ignore(4); // HDOP
        iss.ignore(1); // comma 
        iss >> altitude;

        gpsData.latitude = latitude;
        gpsData.longitude = longitude;
        gpsData.altitude = altitude;  
    }
}

class GpsDataSource
{
public:
  GpsDataSource(rclcpp::Logger logger_in)
  : dev_name{"dev/ttyUSB0"},
    owned_ctx{new IoContext(2)},
    driver{new SerialDriver(*owned_ctx)},
    logger{logger_in}
  {
    uint32_t baud = 9600;
    FlowControl fc = FlowControl::NONE;
    Parity pt = Parity::NONE;
    StopBits sb = StopBits::ONE;

    SerialPortConfig config(baud, fc, pt, sb);
    driver->init_port(dev_name, config);
  }

  void collect_gps_data(gps_package::msg::GpsMessage message) {
    try {
      if (!driver->port()->is_open()) {
        driver->port()->open();
      }

      std::vector<uint8_t> send_recv_buff;
      driver->port()->receive(send_recv_buff);
      std::string string_buf(send_recv_buff.begin(), send_recv_buff.end());
      parseNMEAMessage(string_buf, message);

    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger, "Error creating serial port: %s - %s",
      dev_name.c_str(), ex.what());
    }
  }

private:
  std::string dev_name;
  IoContext *owned_ctx;
  SerialDriver *driver;
  rclcpp::Logger logger;
};

class GpsPublisher : public rclcpp::Node
{
  public:
    GpsPublisher()
    : Node("GpsPublisher"), count_(0), datasource(new GpsDataSource(get_logger()))
    {
      publisher_ = this->create_publisher<gps_package::msg::GpsMessage>("gpsData", 10);
      timer_ = this->create_wall_timer(
         500ms, std::bind(&GpsPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto gpsData = gps_package::msg::GpsMessage();
      datasource->collect_gps_data(gpsData);
      RCLCPP_INFO(get_logger(), "Publishing: '%f', '%f', '%f' ", gpsData.latitude, gpsData.longitude, gpsData.altitude);
      publisher_->publish(gpsData);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<gps_package::msg::GpsMessage>::SharedPtr publisher_;
    size_t count_;
    std::unique_ptr<GpsDataSource> datasource;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsPublisher>());
  rclcpp::shutdown();
  return 0;
}


