#include "gui_server.h"
#include "deliberative_tier/get_state.h"
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
#ifdef SPEECH_API
                                                                                                     pronounce_utterance_server(h.advertiseService("text_to_speech", &gui_server::pronounce_utterance, this)),
                                                                                                     recognize_utterance_server(h.advertiseService("speech_to_text", &gui_server::recognize_utterance, this)),
#endif
                                                                                                     get_state(h.serviceClient<deliberative_tier::get_state>("get_state")),
                                                                                                     talk_to_me(h.serviceClient<std_srvs::Trigger>("talk_to_me")),
                                                                                                     answer_question(h.serviceClient<deliberative_tier::task_service>("contextualized_speech")),
                                                                                                     deliberative_state_sub(h.subscribe("deliberative_state", 100, &gui_server::updated_deliberative_state, this)),
                                                                                                     timelines_sub(h.subscribe("timelines", 100, &gui_server::updated_timelines, this)),
                                                                                                     graph_sub(h.subscribe("graph", 100, &gui_server::updated_graph, this)),
                                                                                                     sequencer_state_sub(h.subscribe("sequencer_state", 100, &gui_server::updated_sequencer_state, this)),
                                                                                                     dialogue_state_sub(h.subscribe("dialogue_state", 100, &gui_server::updated_dialogue_state, this))
    {
        const std::string templates_path = ros::package::getPath("robot_gui") + "/templates/";
        const std::string static_path = ros::package::getPath("robot_gui") + "/static/";

        CROW_ROUTE(app, "/")
        ([templates_path]()
         {
            crow::mustache::context ctx;
            ctx["title"] = "SI-Robotics";
            crow::mustache::set_base(templates_path);
            const auto test = crow::mustache::load("index.html").render(ctx);
            return crow::mustache::load("index.html").render(ctx); });

        CROW_ROUTE(app, "/static/<string>")
        ([static_path](crow::response &res, std::string path)
         {
            std::ifstream ifl(static_path + path);
            std::ostringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            res.add_header("Content-length", std::to_string(res.body.size()));
            std::string mimeType = get_mime_type(path);
            if (mimeType != "")
                res.add_header("Content-Type", mimeType);
            res.end(); });

        CROW_ROUTE(app, "/static/faces/<string>")
        ([static_path](crow::response &res, std::string path)
         {
            std::ifstream ifl(static_path + "faces/" + path);
            std::ostringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            res.add_header("Content-length", std::to_string(res.body.size()));
            std::string mimeType = get_mime_type(path);
            if (mimeType != "")
                res.add_header("Content-Type", mimeType);
            res.end(); });

        CROW_ROUTE(app, "/static/images/<string>")
        ([static_path](crow::response &res, std::string path)
         {
            std::ifstream ifl(static_path + "images/" + path);
            std::ostringstream buffer;
            buffer << ifl.rdbuf();
            res.body = buffer.str();
            res.add_header("Content-length", std::to_string(res.body.size()));
            std::string mimeType = get_mime_type(path);
            if (mimeType != "")
                res.add_header("Content-Type", mimeType);
            res.end(); });

        CROW_ROUTE(app, "/state")
        ([static_path, this]()
         {
            deliberative_tier::get_state get_state_msg;
            get_state.call(get_state_msg);
            std::vector<crow::json::wvalue> reasoners;
            for (size_t i = 0; i < get_state_msg.response.graphs.size(); ++i)
            {
                const auto c_graph = get_state_msg.response.graphs.at(i);
                std::vector<crow::json::wvalue> flaws;
                for (const auto &f : c_graph.flaws)
                    flaws.push_back(flaw_to_json(f));
                std::vector<crow::json::wvalue> resolvers;
                for (const auto &r : c_graph.resolvers)
                    resolvers.push_back(resolver_to_json(r));
                crow::json::wvalue graph({{"flaws", flaws}, {"resolvers", resolvers}, {"current_flaw", c_graph.current_flaw}, {"current_resolver", c_graph.current_resolver}});
                
                const auto c_timelines = get_state_msg.response.timelines.at(i);
                std::vector<crow::json::wvalue> tls;
                for (const auto &tl : c_timelines.timelines)
                    tls.push_back(crow::json::load(tl));
                std::vector<crow::json::wvalue> exec;
                for (const auto &atm_id : c_timelines.executing)
                    exec.push_back(atm_id);
                crow::json::wvalue timelines({{"state", crow::json::load(c_timelines.state)}, {"timelines", std::move(tls)}, {"time", rational_to_json(c_timelines.time)}, {"executing", exec}});

                reasoners.push_back(crow::json::wvalue({{"reasoner_id", c_graph.reasoner_id}, {"graph", graph}, {"timelines", timelines}}));
            }
            return crow::json::wvalue(reasoners); });

        CROW_ROUTE(app, "/solver")
            .websocket()
            .onopen([=](crow::websocket::connection &conn)
                    { std::lock_guard<std::mutex> _(mtx);
                users.insert(&conn); })
            .onclose([=](crow::websocket::connection &conn, const std::string &reason)
                     { std::lock_guard<std::mutex> _(mtx);
                users.erase(&conn); })
            .onmessage([&](crow::websocket::connection &conn, const std::string &data, bool is_binary) {});
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

#ifdef SPEECH_API
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
#endif

    void gui_server::updated_deliberative_state(const deliberative_tier::deliberative_state &msg)
    {
        std::lock_guard<std::mutex> _(mtx);
        if (msg.deliberative_state == msg.destroyed)
            deliberative_state.erase(msg.reasoner_id);
        else
            deliberative_state[msg.reasoner_id] = msg.deliberative_state;
        crow::json::wvalue w({{"type", "deliberative_state"}, {"id", msg.reasoner_id}, {"state", msg.deliberative_state}});
        for (const auto &u : users)
            u->send_text(w.dump());
    }
    void gui_server::updated_timelines(const deliberative_tier::timelines &msg)
    {
        std::lock_guard<std::mutex> _(mtx);
        switch (msg.update)
        {
        case deliberative_tier::timelines::state_changed:
        {
            std::vector<crow::json::wvalue> tls;
            for (const auto &tl : msg.timelines)
                tls.push_back(crow::json::load(tl));
            std::vector<crow::json::wvalue> exec;
            for (const auto &atm_id : msg.executing)
                exec.push_back(atm_id);
            crow::json::wvalue w({{"type", "updated_timelines"}, {"id", msg.reasoner_id}, {"state", crow::json::load(msg.state)}, {"timelines", std::move(tls)}, {"time", rational_to_json(msg.time)}, {"executing", exec}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::timelines::time_changed:
        {
            crow::json::wvalue w({{"type", "time_changed"}, {"id", msg.reasoner_id}, {"time", rational_to_json(msg.time)}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::timelines::executing_changed:
        {
            std::vector<crow::json::wvalue> exec;
            for (const auto &atm_id : msg.executing)
                exec.push_back(atm_id);
            crow::json::wvalue w({{"type", "executing_changed"}, {"id", msg.reasoner_id}, {"executing", exec}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        break;
        default:
            break;
        }
    }
    void gui_server::updated_graph(const deliberative_tier::graph &msg)
    {
        std::lock_guard<std::mutex> _(mtx);
        switch (msg.update)
        {
        case deliberative_tier::graph::graph_changed:
        {
            std::vector<crow::json::wvalue> flaws;
            for (const auto &f : msg.flaws)
                flaws.push_back(flaw_to_json(f));
            std::vector<crow::json::wvalue> resolvers;
            for (const auto &r : msg.resolvers)
                resolvers.push_back(resolver_to_json(r));
            crow::json::wvalue w({{"type", "updated_graph"}, {"flaws", flaws}, {"resolvers", resolvers}, {"current_flaw", msg.current_flaw}, {"current_resolver", msg.current_resolver}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::flaw_created:
        {
            crow::json::wvalue w = flaw_to_json(msg.flaws.at(0));
            w["type"] = "flaw_created";
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::flaw_state_changed:
        {
            crow::json::wvalue w({{"type", "flaw_state_changed"}, {"state", msg.flaws.at(0).state}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::flaw_cost_changed:
        {
            crow::json::wvalue w({{"type", "flaw_cost_changed"}, {"cost", rational_to_json(msg.flaws.at(0).cost)}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::flaw_position_changed:
        {
            crow::json::wvalue w({{"type", "flaw_position_changed"}, {"pos", position_to_json(msg.flaws.at(0).pos)}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::current_flaw:
        {
            crow::json::wvalue w({{"type", "current_flaw"}, {"flaw_id", msg.flaw_id}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::resolver_created:
        {
            crow::json::wvalue w = resolver_to_json(msg.resolvers.at(0));
            w["type"] = "resolver_created";
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::resolver_state_changed:
        {
            crow::json::wvalue w({{"type", "resolver_state_changed"}, {"state", msg.resolvers.at(0).state}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::current_resolver:
        {
            crow::json::wvalue w({{"type", "current_resolver"}, {"resolver_id", msg.resolver_id}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        case deliberative_tier::graph::causal_link_added:
        {
            crow::json::wvalue w({{"type", "causal_link_added"}, {"flaw_id", msg.flaw_id}, {"resolver_id", msg.resolver_id}});
            for (const auto &u : users)
                u->send_text(w.dump());
            break;
        }
        }
    }
    void gui_server::updated_sequencer_state(const sequencer_tier::sequencer_state &msg)
    {
        std::lock_guard<std::mutex> _(mtx);
        sequencer_state = msg.system_state;
        crow::json::wvalue w({{"type", "sequencer_state"}, {"state", msg.system_state}});
        for (const auto &u : users)
            u->send_text(w.dump());
    }
    void gui_server::updated_dialogue_state(const dialogue_manager::dialogue_state &msg)
    {
        std::lock_guard<std::mutex> _(mtx);
        dialogue_state = msg.dialogue_state;
        crow::json::wvalue w({{"type", "dialogue_state"}, {"state", msg.dialogue_state}});
        for (const auto &u : users)
            u->send_text(w.dump());
    }

    crow::json::wvalue gui_server::rational_to_json(const deliberative_tier::rational &r) { return crow::json::wvalue({{"num", r.num}, {"den", r.den}}); }
    crow::json::wvalue gui_server::position_to_json(const deliberative_tier::position &p) { return crow::json::wvalue({{"lb", p.lb}, {"ub", p.ub}}); }
    crow::json::wvalue gui_server::flaw_to_json(const deliberative_tier::flaw &f)
    {
        std::vector<crow::json::wvalue> causes;
        for (const auto &c : f.causes)
            causes.push_back(c);
        return crow::json::wvalue({{"id", f.id}, {"causes", causes}, {"data", crow::json::load(f.data)}, {"state", f.state}, {"pos", position_to_json(f.pos)}, {"cost", rational_to_json(f.cost)}});
    }
    crow::json::wvalue gui_server::resolver_to_json(const deliberative_tier::resolver &r)
    {
        std::vector<crow::json::wvalue> preconditions;
        for (const auto &p : r.preconditions)
            preconditions.push_back(p);
        return crow::json::wvalue({{"id", r.id}, {"preconditions", preconditions}, {"effect", r.effect}, {"data", crow::json::load(r.data)}, {"state", r.state}, {"intrinsic_cost", rational_to_json(r.intrinsic_cost)}});
    }

    std::string gui_server::get_mime_type(const std::string &path)
    {
        std::size_t last_dot = path.find_last_of(".");
        std::string extension = path.substr(last_dot + 1);
        if (extension != "")
        {
            std::string mimeType = crow::mime_types.at(extension);
            if (mimeType != "")
                return mimeType;
            else
                return "text/plain";
        }
        return "";
    }
} // namespace sir