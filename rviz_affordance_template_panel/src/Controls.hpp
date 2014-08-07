#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>

#include "RobotConfig.hpp"
#include "util.hpp"
#include "ui_rviz_affordance_template_panel.h"

#include <zmq.hpp>
#include "AffordanceTemplateServerCmd.pb.h"

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

        void send_command(Command_CommandType command_type);
        void setConnected(bool value) { connected_ = value; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robot_name_ = name; };
        void setSocket(zmq::socket_t* sock) { socket_ = sock; };

    private:
        Ui::RVizAffordanceTemplatePanel* ui_;
        void send_request(const Request& request, Response& response, long timeout_=1000000);
        void update_table(const Response& rep);
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robot_name_;
        std::vector<std::string> getSelectedEndEffectors();
        zmq::socket_t* socket_;
        bool connected_;
        long timeout_;
    };
}

#endif // CONTROLS_HPP
