#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>

#include "RobotConfig.hpp"
#include "util.hpp"
#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/AffordanceTemplateCommand.h>


namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class Controls
    {
    public:
    	typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        Controls(Ui::RVizAffordanceTemplatePanel* ui);
        ~Controls() {};

        void setService(ros::ServiceClient srv) { controlsService_ = srv; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robotName_ = name; };
        void sendCommand(int command_type);

    private:
        Ui::RVizAffordanceTemplatePanel* ui_;
        void updateTable(vector<int> waypoint_ids, vector<int> waypoint_ns);
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;
        std::vector<std::string> getSelectedEndEffectors();
        ros::ServiceClient controlsService_;
    };
}

#endif // CONTROLS_HPP
