#ifndef RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
#define RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP

/* ROS Includes */
#include <ros/ros.h>
#include <rviz/panel.h>
#include <ros/package.h>

/* qt */
#include <QGraphicsScene>

/* Project Include */
#include "Affordance.hpp"
#include "RobotConfig.hpp"
#include "ui_rviz_affordance_template_panel.h"

// zmq
#include <zmq.hpp>
#include "AffordanceTemplateServerCmd.pb.h"

/* Forward Declarations */
//class RVizControlsPanel;

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

        /** \brief Robot Selection callback.
         */
        void changeRobot(int id);

        /** \brief Robot Config End Effector Selection callback.
         */
        void changeEndEffector(int id);

        /** \brief Send a ZMQ request to kill a running template.
         */
        void killTemplate(QListWidgetItem* item);


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

        void send_request(const Request& request, Response& response);
        std::string resolvePackagePath(const string& str);


        // GUI Widgets
        QGraphicsScene* graphicsScene;
        QListWidget* runningList;
        /*
        QGridLayout* gridLayout;
        QGraphicsView* graphicsView;
        */

        // map to track instantiated object templates
        std::map<std::string, AffordanceSharedPtr> affordanceMap;
        std::map<std::string, RobotConfigSharedPtr> robotMap;

        // zmq
        zmq::context_t context;
        zmq::socket_t* socket;
        bool connected;

        // AT Controls panel.
        //RVizControlsPanel* _controlsPanel;

        ros::NodeHandle _nh;


    };
}
#endif // RVIZ_AFFORDANCE_TEMPLATE_PANEL_HPP
