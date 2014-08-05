#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <iostream>
#include <boost/shared_ptr.hpp>

/* qt */
#include <QTableWidgetItem>

/* Project Include */
#include "RobotConfig.hpp"
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
    class Controls
    {
    public:
    	typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;
        // Constructors
        Controls(Ui::RVizAffordanceTemplatePanel* ui);
        ~Controls();

        /** \brief Go To Start Command.
         */
        void go_to_start();

        /** \brief Go To End Command.
         */
        void go_to_end();

        /** \brief Pause Command.
         */
        void pause();

        /** \brief Stop Command.
         */
        void stop();

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

        void setConnected(bool value);
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> robotMap);
        void setRobotName(std::string name);
        void setSocket(zmq::socket_t* socket);

    private:
        // Pointer to ui.
        Ui::RVizAffordanceTemplatePanel* _ui;

        void send_request(const Request& request, Response& response, long timeout=1000000);

        // void send_request(const Request& request, Response& response, long timeout=1000000);
        // map to track instantiated object templates
        std::map<std::string, RobotConfigSharedPtr> robotMap;
        std::string robot_name;

        std::vector<std::string> getSelectedEndEffectors();

        // zmq
        // zmq::context_t context;
        zmq::socket_t* socket;
        bool connected;
    };
}

#endif // CONTROLS_HPP
