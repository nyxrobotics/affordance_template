#ifndef RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
#define RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>

/* qt */
#include <QGraphicsScene>
#include <QTableWidgetItem>

/* Project Include */
#include "Affordance.hpp"
#include "RecognitionObject.hpp"
#include "RobotConfig.hpp"
#include "Controls.hpp"
#include "util.hpp"
#include "ui_rviz_affordance_template_panel.h"

// zmq
#include <zmq.hpp>
#include "AffordanceTemplateServerCmd.pb.h"

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class RVizAffordanceTemplatePanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        typedef boost::shared_ptr<Affordance> AffordanceSharedPtr;
        typedef boost::shared_ptr<RecognitionObject> RecognitionObjectSharedPtr;
        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorPoseConfig> EndEffectorPoseIDConfigSharedPtr;
        typedef boost::shared_ptr<Controls> ControlsSharedPtr;

        // Constructors
        RVizAffordanceTemplatePanel(QWidget* parent = 0);
        ~RVizAffordanceTemplatePanel();

        // Emit configuration change event.
        void configChanged();

    public Q_SLOTS:
        void addAffordanceDisplayItem();
        void addObjectDisplayItem();

        /** \brief Send a ZMQ request to get available template classes and populate the template list.
         */
        void getAvailableInfo();

        /** \brief Send a ZMQ request to get running templates and recognition objects on the server.
         */
        void getRunningItems();

        /** \brief Connect/Disconnect from the Template server.
         */
        void connect_callback();

        /** \brief Load Robot Config.
         */
        void loadConfig();

        /** \brief Load Robot Config.
         */
        void safeLoadConfig();

        /** \brief Robot Selection callback.
         */
        void changeRobot(int id);

        /** \brief Robot Config End Effector Selection callback.
         */
        void changeEndEffector(int id);

        /** \brief Send a ZMQ request to kill a running template.
         */
        void killAffordanceTemplate(QListWidgetItem* item);

        /** \brief Send a ZMQ request to kill a running object recog.
         */
        void killRecognitionObject(QListWidgetItem* item);

        /** \brief Go To Start Command.
         */
        void go_to_start() { controls_->send_command(Command::GO_TO_START); };

        // /** \brief Go To End Command.
        //  */
        void go_to_end() { controls_->send_command(Command::GO_TO_END); };

        // /** \brief Pause Command.
        //  */
        void pause() { controls_->send_command(Command::PAUSE); };

        // /** \brief Stop Command.
        //  */
        void stop() { controls_->send_command(Command::STOP); };

        // * \brief Play Backward Command.

        void play_backward() { controls_->send_command(Command::PLAY_BACKWARD); };

        // /** \brief Play Forward Command.
        //  */
        void play_forward() { controls_->send_command(Command::PLAY_FORWARD); };

        // /** \brief Step Backward Command.
        //  */
        void step_backward() { controls_->send_command(Command::STEP_BACKWARD); };

        // /** \brief Step Forward Command.
        //  */
        void step_forward() { controls_->send_command(Command::STEP_FORWARD); };


    private:
        Ui::RVizAffordanceTemplatePanel* ui_;

        void connect();
        void disconnect();

        void setupWidgets();
        void setupRobotPanel(const string& key);
        void setupEndEffectorConfigPanel(const string& key);

        void removeAffordanceTemplates();
        void sendAffordanceTemplateAdd(const string& class_name);
        void sendAffordanceTemplateKill(const string& class_name, int id);
        void sendPing();
        void sendShutdown();

        void removeRecognitionObjects();
        void sendRecognitionObjectAdd(const string& object_name);
        void sendRecognitionObjectKill(const string& object_name, int id);

        // TODO: template these template functions that keep track of templates
        bool addAffordance(const AffordanceSharedPtr& obj);
        bool removeAffordance(const AffordanceSharedPtr& obj);
        bool checkAffordance(const AffordanceSharedPtr& obj);

        bool addRecognitionObject(const RecognitionObjectSharedPtr& obj);
        bool removeRecognitionObject(const RecognitionObjectSharedPtr& obj);
        bool checkRecognitionObject(const RecognitionObjectSharedPtr& obj);

        bool addRobot(const RobotConfigSharedPtr& obj);
        bool removeRobot(const RobotConfigSharedPtr& obj);
        bool checkRobot(const RobotConfigSharedPtr& obj);

        std::string getRobotFromDescription();
        std::vector<std::string> getSelectedEndEffectors();

        void send_request(const Request& request, Response& response, long timeout=1000000);

        // GUI Widgets
        QGraphicsScene* affordanceTemplateGraphicsScene_;
        QGraphicsScene* recognitionObjectGraphicsScene_;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap_;
        std::map<std::string, RecognitionObjectSharedPtr> recognitionObjectMap_;
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string descriptionRobot_;
        std::string robot_name_;

        // zmq
        zmq::context_t context_;
        zmq::socket_t* socket_;
        bool connected_;

        ros::NodeHandle nh_;

        ControlsSharedPtr controls_;
    };
}
#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
