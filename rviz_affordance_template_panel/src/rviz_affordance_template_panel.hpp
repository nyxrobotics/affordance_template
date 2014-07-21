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
#include "RobotConfig.hpp"
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
        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        typedef boost::shared_ptr<EndEffectorConfig> EndEffectorConfigSharedPtr;

        // Constructors
        RVizAffordanceTemplatePanel(QWidget* parent = 0);
        ~RVizAffordanceTemplatePanel();

        // Emit configuration change event.
        void configChanged();

    public Q_SLOTS:
        void addTemplate();

        /** \brief Send a ZMQ request to get available template classes and populate the template list.
         */
        void getAvailableTemplates();

        /** \brief Send a ZMQ request to get running templates on the server.
         */
        void getRunningTemplates();

        /** \brief Connect/Disconnect to the template server.
         */
        void connect();

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
        void killTemplate(QListWidgetItem* item);

        /** \brief Go To Start Command.
         */
        void go_to_start();

        /** \brief Go To End Command.
         */
        void go_to_end();

        /** \brief Pause Command.
         */
        void pause();

        /** \brief Play Backward Command.
         */
        void play_backward();

        /** \brief Play Forward Command.
         */
        void play_forward();

        /** \brief Step Backward Command.
         */
        void step_backward();

        /** \brief Step Forward Command.
         */
        void step_forward();


    private:
        // Pointer to ui.
        Ui::RVizAffordanceTemplatePanel* _ui;

        void setupWidgets();
        void setupRobotPanel(const string& key);
        void setupEndEffectorConfigPanel(const string& key);

        void removeTemplates();
        void sendAdd(const string& class_name);
        void sendKill(const string& class_name, int id);
        void sendPing();
        void sendShutdown();

        // TODO: template these template functions that keep track of templates
        bool addAffordance(const AffordanceSharedPtr& obj);
        bool removeAffordance(const AffordanceSharedPtr& obj);
        bool checkAffordance(const AffordanceSharedPtr& obj);

        bool addRobot(const RobotConfigSharedPtr& obj);
        bool removeRobot(const RobotConfigSharedPtr& obj);
        bool checkRobot(const RobotConfigSharedPtr& obj);

        std::string getRobotFromDescription();
        std::vector<std::string> getSelectedEndEffectors();

        void send_request(const Request& request, Response& response, long timeout=1000000);
        std::string resolvePackagePath(const string& str);


        // GUI Widgets
        QGraphicsScene* graphicsScene;
        QListWidget* runningList;

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap;
        std::map<std::string, RobotConfigSharedPtr> robotMap;
        std::string descriptionRobot;
        std::string robot_name;
        bool force_load;

        // zmq
        zmq::context_t context;
        zmq::socket_t* socket;
        bool connected;

        ros::NodeHandle _nh;


    };
}
#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
