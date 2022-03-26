#pragma once

#include <ros/ros.h>
#include <crow.h>
#include <unordered_set>
#include <mutex>

namespace sir
{

  class gui_server
  {
  public:
    gui_server(ros::NodeHandle &h, const std::string &host = "127.0.0.1", const unsigned short port = 8080);
    ~gui_server();

    void start();
    void wait_for_server_start();
    void stop();

  private:
    ros::NodeHandle &handle;
    const std::string host;
    const unsigned short port;
    crow::SimpleApp app;
    std::unordered_set<crow::websocket::connection *> users;
    std::mutex mtx;
  };
} // namespace sir