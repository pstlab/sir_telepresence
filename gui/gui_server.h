#pragma once

#include "deliberative_tier/get_state.h"
#include "deliberative_tier/task_service.h"
#include "deliberative_tier/graph.h"
#include "deliberative_tier/timelines.h"
#include "deliberative_tier/deliberative_state.h"
#include "dialogue_manager/face_to_show.h"
#include "dialogue_manager/image_to_show.h"
#include "dialogue_manager/audio_to_play.h"
#include "dialogue_manager/video_to_play.h"
#include "dialogue_manager/page_to_show.h"
#include "dialogue_manager/question_to_ask.h"
#include "dialogue_manager/toast_to_show.h"
#include "dialogue_manager/utterance_to_pronounce.h"
#include "dialogue_manager/utterance_to_recognize.h"
#include "dialogue_manager/dialogue_state.h"
#include "sequencer_tier/sequencer_state.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/UInt64.h"
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
    bool show_face(dialogue_manager::face_to_show::Request &req, dialogue_manager::face_to_show::Response &res);
    bool show_image(dialogue_manager::image_to_show::Request &req, dialogue_manager::image_to_show::Response &res);
    bool play_audio(dialogue_manager::audio_to_play::Request &req, dialogue_manager::audio_to_play::Response &res);
    bool play_video(dialogue_manager::video_to_play::Request &req, dialogue_manager::video_to_play::Response &res);
    bool show_page(dialogue_manager::page_to_show::Request &req, dialogue_manager::page_to_show::Response &res);
    bool ask_question(dialogue_manager::question_to_ask::Request &req, dialogue_manager::question_to_ask::Response &res);
    bool show_toast(dialogue_manager::toast_to_show::Request &req, dialogue_manager::toast_to_show::Response &res);

    bool pronounce_utterance(dialogue_manager::utterance_to_pronounce::Request &req, dialogue_manager::utterance_to_pronounce::Response &res);
    bool recognize_utterance(dialogue_manager::utterance_to_recognize::Request &req, dialogue_manager::utterance_to_recognize::Response &res);

    void updated_deliberative_state(const deliberative_tier::deliberative_state &msg);
    void updated_timelines(const deliberative_tier::timelines &msg);
    void updated_graph(const deliberative_tier::graph &msg);
    void updated_sequencer_state(const sequencer_tier::sequencer_state &msg);
    void updated_dialogue_state(const dialogue_manager::dialogue_state &msg);

  private:
    ros::NodeHandle &handle;
    ros::ServiceServer show_face_server;
    ros::ServiceServer show_image_server;
    ros::ServiceServer play_audio_server;
    ros::ServiceServer play_video_server;
    ros::ServiceServer show_page_server;
    ros::ServiceServer ask_question_server;
    ros::ServiceServer show_toast_server;
    ros::ServiceServer pronounce_utterance_server;
    ros::ServiceServer recognize_utterance_server;
    ros::ServiceClient talk_to_me;
    ros::ServiceClient answer_question;
    const std::string host;
    const unsigned short port;
    crow::SimpleApp app;
    std::unordered_set<crow::websocket::connection *> users;
    std::mutex mtx;
    std::mutex stt_mtx;
    std::string utterance;
    std::condition_variable stt_cv;
    ros::Subscriber deliberative_state_sub;
    std::map<uint64_t, unsigned int> deliberative_state;
    ros::Subscriber timelines_sub, graph_sub;
    ros::Subscriber sequencer_state_sub;
    unsigned int sequencer_state = sequencer_tier::sequencer_state::unconfigured;
    ros::Subscriber dialogue_state_sub;
    unsigned int dialogue_state = dialogue_manager::dialogue_state::idle;
  };
} // namespace sir