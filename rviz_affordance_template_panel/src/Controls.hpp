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
        void setConnected(bool value) { connected = value; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap = map; };
        void setRobotName(std::string name) { robot_name = name; };
        void setSocket(zmq::socket_t* sock) { socket = sock; };

    private:
        Ui::RVizAffordanceTemplatePanel* _ui;
        void send_request(const Request& request, Response& response, long timeout=1000000);
        void update_table(const Response& rep);
        std::map<std::string, RobotConfigSharedPtr> robotMap;
        std::string robot_name;
        std::vector<std::string> getSelectedEndEffectors();
        zmq::socket_t* socket;
        bool connected;
        long timeout;
    };
}

#endif // CONTROLS_HPP
