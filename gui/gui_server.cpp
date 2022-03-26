#include "gui_server.h"
#include <ros/package.h>

namespace sir
{
    gui_server::gui_server(ros::NodeHandle &h, const std::string &host, const unsigned short port) : handle(h), host(host), port(port),
                                                                                                     show_face_server(h.advertiseService("set_face", &gui_server::show_face, this)),
                                                                                                     show_image_server(h.advertiseService("show_image", &gui_server::show_image, this)),
                                                                                                     play_audio_server(h.advertiseService("play_audio", &gui_server::play_audio, this)),
                                                                                                     play_video_server(h.advertiseService("play_video", &gui_server::play_video, this)),
                                                                                                     show_page_server(h.advertiseService("show_page", &gui_server::show_page, this)),
                                                                                                     ask_question_server(h.advertiseService("ask_question", &gui_server::ask_question, this)),
                                                                                                     show_toast_server(h.advertiseService("show_toast", &gui_server::show_toast, this)),
                                                                                                     pronounce_utterance_server(h.advertiseService("text_to_speech", &gui_server::pronounce_utterance, this)),
                                                                                                     recognize_utterance_server(h.advertiseService("speech_to_text", &gui_server::recognize_utterance, this)),
                                                                                                     talk_to_me(h.serviceClient<std_srvs::Trigger>("listen")),
                                                                                                     answer_question(h.serviceClient<deliberative_tier::task_service>("contextualized_speech"))
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

    bool gui_server::show_face(dialogue_manager::face_to_show::Request &req, dialogue_manager::face_to_show::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "show_face"}, {"facial_expression", req.facial_expression}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::show_image(dialogue_manager::image_to_show::Request &req, dialogue_manager::image_to_show::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "show_image"}, {"src", req.src}, {"alt", req.alt}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::play_audio(dialogue_manager::audio_to_play::Request &req, dialogue_manager::audio_to_play::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        std::vector<crow::json::wvalue> audios;
        for (const auto &audio : req.audios)
            audios.push_back(crow::json::wvalue({{"src", audio.src}, {"type", audio.type}}));
        crow::json::wvalue w({{"type", "play_audio"}, {"audios", std::move(audios)}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::play_video(dialogue_manager::video_to_play::Request &req, dialogue_manager::video_to_play::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        std::vector<crow::json::wvalue> videos;
        for (const auto &video : req.videos)
            videos.push_back(crow::json::wvalue({{"src", video.src}, {"type", video.type}}));
        crow::json::wvalue w({{"type", "play_video"}, {"videos", std::move(videos)}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::show_page(dialogue_manager::page_to_show::Request &req, dialogue_manager::page_to_show::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "show_image"}, {"title", req.title}, {"src", req.src}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::ask_question(dialogue_manager::question_to_ask::Request &req, dialogue_manager::question_to_ask::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        std::vector<crow::json::wvalue> buttons;
        for (const auto &button : req.buttons)
            buttons.push_back(crow::json::wvalue({{"text", button.text}, {"intent", button.intent}}));
        crow::json::wvalue w({{"type", "ask_question"}, {"facial_expression", req.facial_expression}, {"text", req.text}, {"buttons", std::move(buttons)}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::show_toast(dialogue_manager::toast_to_show::Request &req, dialogue_manager::toast_to_show::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "show_toast"}, {"text", req.text}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }

    bool gui_server::pronounce_utterance(dialogue_manager::utterance_to_pronounce::Request &req, dialogue_manager::utterance_to_pronounce::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "pronounce_utterance"}, {"text", req.utterance}});
        for (const auto &u : users)
            u->send_text(w.dump());
        res.success = !users.empty();
        return true;
    }
    bool gui_server::recognize_utterance(dialogue_manager::utterance_to_recognize::Request &req, dialogue_manager::utterance_to_recognize::Response &res)
    {
        std::lock_guard<std::mutex> _(mtx);
        crow::json::wvalue w({{"type", "recognize_utterance"}});
        for (const auto &u : users)
            u->send_text(w.dump());
        std::unique_lock<std::mutex> _l(stt_mtx);
        stt_cv.wait(_l);
        res.utterance = utterance;
        res.success = !users.empty();
        return true;
    }
} // namespace sir