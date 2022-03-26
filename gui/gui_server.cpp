#include "gui_server.h"
#include <ros/package.h>

namespace sir
{
    gui_server::gui_server(ros::NodeHandle &h, const std::string &host, const unsigned short port) : handle(h), host(host), port(port)
    {
        const std::string templates_path = ros::package::getPath("robot_gui") + "/templates/";
        const std::string static_path = ros::package::getPath("robot_gui") + "/static/";

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
    }
    gui_server::~gui_server() {}

    void gui_server::start() { app.bindaddr(host).port(port).run(); }
    void gui_server::wait_for_server_start() { app.wait_for_server_start(); }
    void gui_server::stop() { app.stop(); }
} // namespace sir