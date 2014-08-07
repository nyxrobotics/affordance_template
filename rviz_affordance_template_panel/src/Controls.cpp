#include "Controls.hpp"

using namespace rviz_affordance_template_panel;
using namespace std;

Controls::Controls(Ui::RVizAffordanceTemplatePanel* ui) :
    connected(false),
    timeout(10000000),
    _ui(ui) {}

void Controls::update_table(const Response& rep) {
    for (auto& c: rep.waypoint_info()) {
        for (auto& e: (*robotMap[robot_name]).endeffectorMap) {

            if (e.second->id() != c.id()) {
                continue;
            }

            for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {

                if (e.second->name() != _ui->end_effector_table->item(r,0)->text().toStdString()) {
                    continue;
                }

                QTableWidgetItem* item = _ui->end_effector_table->item(r, 1);
                item->setText(QString::number(c.num_waypoints()));
            }

        }
    }
}

void Controls::send_command(Command_CommandType command_type) {

    Request req;
    req.set_type(Request::COMMAND);

    string key = _ui->control_template_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');
    Template* temp(req.add_affordance_template());
    temp->set_type(stuff[0]);
    temp->set_id(atoi(stuff[1].c_str()));

    Command *cmd = req.mutable_command();
    cmd->set_type(command_type);
    cmd->set_steps(_ui->num_steps->text().toInt());
    cmd->set_execute(_ui->execute_on_plan->isChecked());
    vector<string> ee_list = getSelectedEndEffectors();
    for(int i=0; i<ee_list.size(); i++) {
        cmd->add_end_effector(ee_list[i]);
    }

    Response rep;
    send_request(req, rep, timeout);
    update_table(rep);
}

void Controls::send_request(const Request& request, Response& response, long timeout) {
    if (!connected) {
        return;
    }

    bool success = util::send_request(socket, request, response, timeout);
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
