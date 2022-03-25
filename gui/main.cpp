#include <ros/ros.h>
#include <ros/package.h>
#include <crow.h>
#include <unordered_set>

#define LOCALHOST_ADDRESS "127.0.0.1"
#define LOCALHOST_PORT 8080

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_gui");
    ros::NodeHandle nh;
    ROS_INFO("Starting the Robot GUI..");

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    const std::string templates_path = ros::package::getPath("robot_gui") + "/templates/";
    const std::string static_path = ros::package::getPath("robot_gui") + "/static/";
    std::unordered_set<crow::websocket::connection *> users;
    std::mutex mtx;

    crow::SimpleApp app;

    CROW_ROUTE(app, "/")
    ([&templates_path]()
     {
            crow::mustache::context ctx;
            ctx["title"] = "SI-Robotics";
            crow::mustache::set_base(templates_path);
            return crow::mustache::load("index.html").render(ctx); });

    CROW_ROUTE(app, "/static/<string>")
    ([&static_path](crow::response &res, std::string path)
     {
            std::ifstream ifl(static_path + path, std::ios_base::binary);
            std::stringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            std::size_t last_dot = path.find_last_of(".");
            std::string extension = path.substr(last_dot + 1);
            std::string mimeType = "";
            res.add_header("Content-length", std::to_string(res.body.size()));
            if (extension != "")
            {
                mimeType = crow::mime_types.at(extension);
                if (mimeType != "")
                    res.add_header("Content-Type", mimeType);
                else
                    res.add_header("content-Type", "text/plain");
            }
            res.end(); });

    CROW_ROUTE(app, "/static/faces/<string>")
    ([&static_path](crow::response &res, std::string path)
     {
            std::ifstream ifl(static_path + "faces/" + path, std::ios_base::binary);
            std::stringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            std::size_t last_dot = path.find_last_of(".");
            std::string extension = path.substr(last_dot + 1);
            std::string mimeType = "";
            res.add_header("Content-length", std::to_string(res.body.size()));
            if (extension != "")
            {
                mimeType = crow::mime_types.at(extension);
                if (mimeType != "")
                    res.add_header("Content-Type", mimeType);
                else
                    res.add_header("content-Type", "text/plain");
            }
            res.end(); });

    CROW_ROUTE(app, "/static/images/<string>")
    ([&static_path](crow::response &res, std::string path)
     {
            std::ifstream ifl(static_path + "images/" + path, std::ios_base::binary);
            std::stringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            std::size_t last_dot = path.find_last_of(".");
            std::string extension = path.substr(last_dot + 1);
            std::string mimeType = "";
            res.add_header("Content-length", std::to_string(res.body.size()));
            if (extension != "")
            {
                mimeType = crow::mime_types.at(extension);
                if (mimeType != "")
                    res.add_header("Content-Type", mimeType);
                else
                    res.add_header("content-Type", "text/plain");
            }
            res.end(); });

    CROW_ROUTE(app, "/solver")
        .websocket()
        .onopen([&](crow::websocket::connection &conn)
                { std::lock_guard<std::mutex> _(mtx);
                users.insert(&conn); });

    ROS_INFO("Starting GUI Server..");
    auto srv_st = std::async(std::launch::async, [&]
                             { app.bindaddr(LOCALHOST_ADDRESS).port(LOCALHOST_PORT).run(); });
    app.wait_for_server_start();
    ROS_INFO("GUI Server started..");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}