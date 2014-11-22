#include "Controls.hpp"

using namespace rviz_affordance_template_panel;
using namespace std;

Controls::Controls(Ui::RVizAffordanceTemplatePanel* ui) :
    ui_(ui) {}

void Controls::updateTable(vector<int> waypoint_ids, vector<int> waypoint_ns) {
    for (size_t idx=0 ; idx<waypoint_ids.size(); idx++) {
        for (auto& e: (*robotMap_[robotName_]).endeffectorMap) {
            if (e.second->id() != idx) {
                continue;
            }
            for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
                if (e.second->name() != ui_->end_effector_table->item(r,0)->text().toStdString()) {
                    continue;
                }
                QTableWidgetItem* item = ui_->end_effector_table->item(r, 1);
                item->setText(QString::number(waypoint_ids[idx]));
            }
        }
    }
    for (size_t idx=0 ; idx<waypoint_ns.size(); idx++) {
        for (auto& e: (*robotMap_[robotName_]).endeffectorMap) {
            if (e.second->id() != idx) {
                continue;
            }
            for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
                if (e.second->name() != ui_->end_effector_table->item(r,0)->text().toStdString()) {
                    continue;
                }
                QTableWidgetItem* item = ui_->end_effector_table->item(r, 2);
                item->setText(QString::number(waypoint_ns[idx]));
            }
        }
    }
}

void Controls::sendCommand(int command_type) {


    string key = ui_->control_template_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');
    vector<int> waypoint_ids;
    vector<int> waypoint_ns;

    ROS_INFO("Sending Command request for a %s", key.c_str());      

    affordance_template_msgs::AffordanceTemplateCommand srv;
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
    srv.request.command = command_type;
    srv.request.steps = ui_->num_steps->text().toInt();
    srv.request.execute_on_plan = ui_->execute_on_plan->isChecked();
    srv.request.execute_precomputed_plan = true;

    vector<string> ee_list = getSelectedEndEffectors();
    for(auto &ee : ee_list) {
        srv.request.end_effectors.push_back(ee);
    }

    if (controlsService_.call(srv))
    {
        ROS_INFO("Command successful");
        for(auto &wp : srv.response.waypoint_info) {
            waypoint_ids.push_back(int(wp.waypoint_index));
            waypoint_ns.push_back(int(wp.num_waypoints));
        }
        updateTable(waypoint_ids, waypoint_ns);

    }
    else
    {
        ROS_ERROR("Failed to call service command");
    }
}


vector<string> Controls::getSelectedEndEffectors() {
    vector<string> selectedEndEffectors;
    for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
        if (ui_->end_effector_table->item(r,3)->checkState() == Qt::Checked ) {
            selectedEndEffectors.push_back(ui_->end_effector_table->item(r,0)->text().toStdString());
        }
    }
    return selectedEndEffectors;
}
