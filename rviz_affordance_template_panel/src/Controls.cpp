#include "Controls.hpp"

using namespace rviz_affordance_template_panel;
using namespace std;

Controls::Controls(Ui::RVizAffordanceTemplatePanel* ui) :
    connected(false),
    _ui(ui)
{}

Controls::~Controls()
{}

void Controls::setConnected(bool value) {
    connected = value;
}

void Controls::setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) {
    robotMap = map;
}

void Controls::setRobotName(std::string name) {
    robot_name = name;
}

void Controls::setSocket(zmq::socket_t* sock) {
    socket = sock;
}

void Controls::go_to_start() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::GO_TO_START);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::go_to_end() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::GO_TO_END);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::pause() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::PAUSE);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::stop() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::STOP);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}


void Controls::play_backward() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::PLAY_BACKWARD);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::play_forward() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));
        Command *cmd = req.mutable_command();

        cmd->set_type(Command::PLAY_FORWARD);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::step_backward() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));
        Command *cmd = req.mutable_command();

        cmd->set_type(Command::STEP_BACKWARD);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        cout << "got response" << endl;
        for (auto& c: rep.waypoint_info()) {
            cout << c.id() << endl;
            cout << c.num_waypoints() << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints())));
                        }
                    }
                }
            }
        }
    }
}

void Controls::step_forward() {
    if (connected) {
        Request req;
        req.set_type(Request::COMMAND);

        string key = _ui->control_template_box->currentText().toUtf8().constData();
        vector<string> stuff = util::split(key, ':');
        Template* temp(req.add_affordance_template());
        temp->set_type(stuff[0]);
        temp->set_id(atoi(stuff[1].c_str()));

        Command *cmd = req.mutable_command();
        cmd->set_type(Command::STEP_FORWARD);
        cmd->set_steps(_ui->num_steps->text().toInt());
        cmd->set_execute(_ui->execute_on_plan->isChecked());
        vector<string> ee_list = getSelectedEndEffectors();
        for(int i=0; i<ee_list.size(); i++) {
            cmd->add_end_effector(ee_list[i]);
        }
        Response rep;
        send_request(req, rep, 10000000);
        for (auto& c: rep.waypoint_info()) {
            cout << "end effector: " << c.id() << endl;
            cout << "now at waypoint : " << (c.num_waypoints()+1) << endl;
            for (auto& e: (*robotMap[robot_name]).endeffectorMap) {
                if (e.second->id() == c.id()) {
                    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                        if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                            _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString::number(c.num_waypoints()+1)));
                        }
                    }
                }
            }
        }
    }
}


void Controls::send_request(const Request& request, Response& response, long timeout) {
    if (connected) {
        try {
            string req;
            request.SerializeToString(&req);

            zmq::message_t msg(req.size());
            memcpy((void*) msg.data(), req.data(), req.size());
            socket->send(msg);

            string rep;
            zmq::pollitem_t poller[] = { {*socket, 0, ZMQ_POLLIN, 0} };
            zmq::poll(&poller[0], 1, timeout);

            // poll for 1 second
            if (poller[0].revents & ZMQ_POLLIN) {

                zmq::message_t reply;
                socket->recv(&reply);

                response.ParseFromArray(reply.data(), reply.size());

            } else {
                cout << "RVizAffordanceTemplatePanel::send_request() disconnecting" << endl;
                // disconnect();
            }
        } catch (const zmq::error_t& ex) {
            cerr << ex.what() << endl;
        }
    }
}

vector<string> Controls::getSelectedEndEffectors() {

    vector<string> selectedEndEffectors;
    for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
        if (_ui->end_effector_table->item(r,3)->checkState() == Qt::Checked ) {
            selectedEndEffectors.push_back(_ui->end_effector_table->item(r,0)->text().toStdString());
        }
    }
    return selectedEndEffectors;
}
