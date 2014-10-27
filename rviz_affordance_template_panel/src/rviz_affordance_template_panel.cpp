#include "rviz_affordance_template_panel.hpp"

// TODO: don't like these
#define PIXMAP_SIZE 100
#define XOFFSET 20
#define YOFFSET 20

#define CLASS_INDEX 0
#define WAYPOINT_DATA 1

#define OBJECT_INDEX 0
#define PACKAGE 1
#define LAUNCH_FILE 2

using namespace rviz_affordance_template_panel;
using namespace std;

RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel(QWidget *parent) :
    rviz::Panel(parent),
    context_(1),
    connected_(false),
    ui_(new Ui::RVizAffordanceTemplatePanel),
    descriptionRobot_(""),
    controls_(new Controls(ui_))
{
    // Setup the panel.
    ui_->setupUi(this);

    setupWidgets();
    connect();

    descriptionRobot_ = getRobotFromDescription();

    if (descriptionRobot_ != "") {
        std::string yamlRobotCandidate = descriptionRobot_ + ".yaml";
        cout << "RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel() -- searching for Robot: " << yamlRobotCandidate << endl;
        map<string,RobotConfigSharedPtr>::const_iterator it = robotMap_.find(yamlRobotCandidate);
        if (it != robotMap_.end() ) {
            int idx= ui_->robot_select->findText(QString(yamlRobotCandidate.c_str()));
            ui_->robot_select->setCurrentIndex(idx);
            setupRobotPanel(yamlRobotCandidate);
            loadConfig();
        }
    }

}

RVizAffordanceTemplatePanel::~RVizAffordanceTemplatePanel()
{
    delete ui_;
}

void RVizAffordanceTemplatePanel::setupWidgets() {

    affordanceTemplateGraphicsScene_ = new QGraphicsScene(this);
    ui_->affordanceTemplateGraphicsView->setScene(affordanceTemplateGraphicsScene_);

    recognitionObjectGraphicsScene_ = new QGraphicsScene(this);
    ui_->recognitionObjectGraphicsView->setScene(recognitionObjectGraphicsScene_);

    QObject::connect(ui_->server_output_status, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(killAffordanceTemplate(QListWidgetItem*)));
    QObject::connect(ui_->delete_template_button, SIGNAL(clicked()), this, SLOT(deleteButton()));

    QObject::connect(ui_->connect_button, SIGNAL(clicked()), this, SLOT(connect_callback()));
    QObject::connect(ui_->load_config_button, SIGNAL(clicked()), this, SLOT(safeLoadConfig()));
    QObject::connect(ui_->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));

    QObject::connect(ui_->go_to_start_button, SIGNAL(clicked()), this, SLOT(go_to_start()));
    QObject::connect(ui_->go_to_end_button, SIGNAL(clicked()), this, SLOT(go_to_end()));
    //QObject::connect(ui_->pause_button, SIGNAL(clicked()), this, SLOT(pause()));
    //QObject::connect(ui_->play_backwards_button, SIGNAL(clicked()), this, SLOT(play_backward()));
    //QObject::connect(ui_->play_button, SIGNAL(clicked()), this, SLOT(play_forward()));
    QObject::connect(ui_->step_backwards_button, SIGNAL(clicked()), this, SLOT(step_backward()));
    QObject::connect(ui_->step_forward_button, SIGNAL(clicked()), this, SLOT(step_forward()));

    QObject::connect(ui_->refresh_button, SIGNAL(clicked()), this, SLOT(getAvailableInfo()));
    QObject::connect(ui_->robot_lock, SIGNAL(stateChanged(int)), this, SLOT(enable_config_panel(int)));

    QObject::connect(ui_->robot_name, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->moveit_package, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->gripper_service, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    
    QObject::connect(ui_->frame_id, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_tx, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_ty, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_tz, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_rr, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_rp, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->robot_ry, SIGNAL(textEdited(const QString&)), this, SLOT(update_robot_config(const QString&)));
    QObject::connect(ui_->ee_name, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_id, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_tx, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_ty, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_tz, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_rr, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_rp, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
    QObject::connect(ui_->ee_ry, SIGNAL(textEdited(const QString&)), this, SLOT(update_end_effector_map(const QString&)));
}

void RVizAffordanceTemplatePanel::update_robot_config(const QString& text) {
    // update the robot config
    // note: we ignore the actual updated text passed in, we simply update
    // robot map values with the current text in XYZ/RPY

    // get currently selected robot key
    string key = ui_->robot_select->currentText().toUtf8().constData();
    // now update robotMap with current values
    (*robotMap_[key]).name(ui_->robot_select->currentText().toUtf8().constData());
    (*robotMap_[key]).moveit_config_package(ui_->moveit_package->text().toUtf8().constData());
    (*robotMap_[key]).frame_id(ui_->frame_id->text().toUtf8().constData());
    (*robotMap_[key]).gripper_service(ui_->gripper_service->text().toUtf8().constData());

    float tx, ty, tz, rr, rp, ry;
    tx = ui_->robot_tx->text().toFloat();
    ty = ui_->robot_ty->text().toFloat();
    tz = ui_->robot_tz->text().toFloat();
    rr = ui_->robot_rr->text().toFloat();
    rp = ui_->robot_rp->text().toFloat();
    ry = ui_->robot_ry->text().toFloat();
    vector<float> q = util::RPYToQuaternion(rr, rp, ry);
    vector<float> root_offset(7);
    root_offset[0] = tx;
    root_offset[1] = ty;
    root_offset[2] = tz;
    root_offset[3] = q[0];
    root_offset[4] = q[1];
    root_offset[5] = q[2];
    root_offset[6] = q[3];
    (*robotMap_[key]).root_offset(root_offset);
}

void RVizAffordanceTemplatePanel::update_end_effector_map(const QString& text) {
    // update the end effector robot map
    // note: we ignore the actual updated text passed in, we simply update
    // robot map values with the current text in XYZ/RPY of the current EE
    string robot_key = ui_->robot_select->currentText().toUtf8().constData();
    string key = ui_->end_effector_select->currentText().toUtf8().constData();
    for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            e.second->name(ui_->ee_name->text().toUtf8().constData());
            e.second->id(ui_->ee_id->text().toInt());
            float tx, ty, tz, rr, rp, ry;
            tx = ui_->ee_tx->text().toFloat();
            ty = ui_->ee_ty->text().toFloat();
            tz = ui_->ee_tz->text().toFloat();
            rr = ui_->ee_rr->text().toFloat();
            rp = ui_->ee_rp->text().toFloat();
            ry = ui_->ee_ry->text().toFloat();
            vector<float> q = util::RPYToQuaternion(rr, rp, ry);
            vector<float> pose_offset(7);
            pose_offset[0] = tx;
            pose_offset[1] = ty;
            pose_offset[2] = tz;
            pose_offset[3] = q[0];
            pose_offset[4] = q[1];
            pose_offset[5] = q[2];
            pose_offset[6] = q[3];
            e.second->pose_offset(pose_offset);
            break;
        }
    }
}

void RVizAffordanceTemplatePanel::enable_config_panel(int state) {
    if (state == Qt::Checked) {
        ui_->groupBox->setEnabled(false);
        ui_->load_config_button->setEnabled(true);
    } else {
        ui_->groupBox->setEnabled(true);
        ui_->load_config_button->setEnabled(false);
    }
}

void RVizAffordanceTemplatePanel::connect_callback() {
    if (connected_) {
        disconnect();
    } else {
        connect();
    }
}

void RVizAffordanceTemplatePanel::disconnect() {
    bool success = false;
    try {
        ui_->server_output_status->clear();
        removeAffordanceTemplates();
        removeRecognitionObjects();
        socket_->close();
        connected_ = false;
        delete socket_;
        success = true;
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
    if (success) {
        ui_->server_connection_label->setText("disconnected");
        ui_->server_connection_label->setStyleSheet("QLabel {color: red;}");
        ui_->connect_button->setText("Connect");
    }
    controls_->setConnected(connected_);
}

void RVizAffordanceTemplatePanel::connect() {
    // return if already connected
    bool success = false;
    try {
        string addr = ui_->server_text->text().toStdString();
        socket_ = util::client_socket(context_, addr);
        controls_->setSocket(socket_);
        connected_ = true;
        sendPing();
        getAvailableInfo();
        getRunningItems();
        success = true;
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
    cout << "connect: " << success << endl;
    if (success) {
        ui_->server_connection_label->setText("connected");
        ui_->server_connection_label->setStyleSheet("QLabel {color: green;}");
        ui_->connect_button->setText("Disconnect");
    }
    controls_->setConnected(connected_);
}

void RVizAffordanceTemplatePanel::getAvailableInfo() {
    if (!connected_) {
        return;
    }

    // templates must be exported in plugin_description.xml of affordance_template package
    Request req;
    req.set_type(Request::QUERY);
    Response rep;
    send_request(req, rep);
    int yoffset = YOFFSET;

    // set up Affordance Template select menu
    for (auto& c: rep.affordance_template()) {
        cout << c.type() << endl;
        string image_path = util::resolvePackagePath(c.image_path());
        QMap<QString, QVariant> waypoint_map;
        for (auto& wp: c.waypoint_info()) {
            waypoint_map[QString::number(wp.id())] = QVariant(wp.num_waypoints());
        }
        AffordanceSharedPtr pitem(new Affordance(c.type(), image_path, waypoint_map));
        std::cout << image_path << std::endl;
        pitem->setPos(XOFFSET, yoffset);
        yoffset += PIXMAP_SIZE + YOFFSET;
        affordanceTemplateGraphicsScene_->addItem(pitem.get());
        addAffordance(pitem);
    }
    cout << "setting up addAffordanceDisplayItem callback" << endl;
    QObject::connect(affordanceTemplateGraphicsScene_, SIGNAL(selectionChanged()), this, SLOT(addAffordanceDisplayItem()));
    affordanceTemplateGraphicsScene_->update();

    // set up Recognition Object select menu
    for (auto& c: rep.recognition_object()) {
        cout << c.type() << endl;
        cout << c.launch_file() << endl;
        cout << c.package() << endl;
        cout << c.image_path() << endl;
        cout << c.topic() << endl;
        string image_path = util::resolvePackagePath(c.image_path());
        RecognitionObjectSharedPtr pitem(new RecognitionObject(c.type(), c.launch_file(), c.package(), image_path));
        cout << image_path << endl;
        pitem->setPos(XOFFSET, yoffset);
        yoffset += PIXMAP_SIZE + YOFFSET;
        recognitionObjectGraphicsScene_->addItem(pitem.get());
        addRecognitionObject(pitem);
    }
    cout << "setting up addObjectDisplayItem callback" << endl;
    QObject::connect(recognitionObjectGraphicsScene_, SIGNAL(selectionChanged()), this, SLOT(addObjectDisplayItem()));
    recognitionObjectGraphicsScene_->update();

    // load stuff for robot config sub panel
    ui_->robot_select->disconnect(SIGNAL(currentIndexChanged(int)));
    ui_->end_effector_select->disconnect(SIGNAL(currentIndexChanged(int)));
    ui_->robot_select->clear();
    ui_->end_effector_select->clear();

    for (auto& r: rep.robot()) {

        RobotConfigSharedPtr pitem(new RobotConfig(r.filename()));
        pitem->uid(r.filename());
        pitem->name(r.name());
        pitem->moveit_config_package(r.moveit_config_package());
        pitem->frame_id(r.frame_id());
        pitem->gripper_service(r.gripper_service());

        vector<float> root_offset(7);
        root_offset[0] = (float)(r.root_offset().position().x());
        root_offset[1] = (float)(r.root_offset().position().y());
        root_offset[2] = (float)(r.root_offset().position().z());
        root_offset[3] = (float)(r.root_offset().orientation().x());
        root_offset[4] = (float)(r.root_offset().orientation().y());
        root_offset[5] = (float)(r.root_offset().orientation().z());
        root_offset[6] = (float)(r.root_offset().orientation().w());
        pitem->root_offset(root_offset);

        

        for (auto& e: r.end_effectors().end_effector()) {
            EndEffectorConfigSharedPtr eitem(new EndEffectorConfig(e.name()));
            eitem->id(e.id());
            vector<float> pose_offset(7);
            pose_offset[0] = (float)(e.pose_offset().position().x());
            pose_offset[1] = (float)(e.pose_offset().position().y());
            pose_offset[2] = (float)(e.pose_offset().position().z());
            pose_offset[3] = (float)(e.pose_offset().orientation().x());
            pose_offset[4] = (float)(e.pose_offset().orientation().y());
            pose_offset[5] = (float)(e.pose_offset().orientation().z());
            pose_offset[6] = (float)(e.pose_offset().orientation().w());
            eitem->pose_offset(pose_offset);
            pitem->endeffectorMap[(*eitem).name()] = eitem;
        }
        for (auto& p: r.end_effector_pose_ids().pose_group()) {
            EndEffectorPoseIDConfigSharedPtr piditem(new EndEffectorPoseConfig(p.name()));
            piditem->id(p.id());
            piditem->group(p.group());
            pitem->endeffectorPoseMap[(*piditem).name()] = piditem;
        }

        addRobot(pitem);

        ui_->robot_select->addItem(QString(pitem->uid().c_str()));
    }

    setupRobotPanel(robotMap_.begin()->first);
    QObject::connect(ui_->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
    QObject::connect(ui_->end_effector_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeEndEffector(int)));

    // set Controls
    controls_->setRobotMap(robotMap_);
}

void RVizAffordanceTemplatePanel::setupRobotPanel(const string& key) {

    cout << "setupRobotPanel for " << key << endl;
    string name = (*robotMap_[key]).name();
    string pkg = (*robotMap_[key]).moveit_config_package();
    string frame_id = (*robotMap_[key]).frame_id();
    string gripper_service = (*robotMap_[key]).gripper_service();

    vector<float> root_offset = (*robotMap_[key]).root_offset();

    ui_->robot_name->setText(QString(name.c_str()));
    ui_->moveit_package->setText(QString(pkg.c_str()));
    ui_->frame_id->setText(QString(frame_id.c_str()));
    ui_->gripper_service->setText(QString(gripper_service.c_str()));

    ui_->robot_tx->setText(QString::number(root_offset[0]));
    ui_->robot_ty->setText(QString::number(root_offset[1]));
    ui_->robot_tz->setText(QString::number(root_offset[2]));

    cout << root_offset[3] << endl;
    cout << root_offset[4] << endl;
    cout << root_offset[5] << endl;
    cout << root_offset[6] << endl;

    vector<float> rpy = util::quaternionToRPY(root_offset[3],root_offset[4],root_offset[5],root_offset[6]);

    ui_->robot_rr->setText(QString::number(rpy[0]));
    ui_->robot_rp->setText(QString::number(rpy[1]));
    ui_->robot_ry->setText(QString::number(rpy[2]));

    ui_->end_effector_select->clear();

    cout << "setupRobotPanel() -- adding end_effectors" << endl;
    for (auto& e: (*robotMap_[key]).endeffectorMap) {
        ui_->end_effector_select->addItem(e.second->name().c_str());
    }

    setupEndEffectorConfigPanel((*robotMap_[key]).endeffectorMap.begin()->first);

}

void RVizAffordanceTemplatePanel::setupEndEffectorConfigPanel(const string& key) {

    cout << "setupEndEffectorConfigPanel() -- setting panel data: " << key << endl;
    string robot_key = ui_->robot_select->currentText().toUtf8().constData();

    for (auto& e: (*robotMap_[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            ui_->ee_name->setText(e.second->name().c_str());
            ui_->ee_id->setText(QString::number(e.second->id()));
            vector<float> pose_offset = e.second->pose_offset();
            ui_->ee_tx->setText(QString::number(pose_offset[0]));
            ui_->ee_ty->setText(QString::number(pose_offset[1]));
            ui_->ee_tz->setText(QString::number(pose_offset[2]));

            vector<float> rpy = util::quaternionToRPY(pose_offset[3],pose_offset[4],pose_offset[5],pose_offset[6]);
            ui_->ee_rr->setText(QString::number(rpy[0]));
            ui_->ee_rp->setText(QString::number(rpy[1]));
            ui_->ee_ry->setText(QString::number(rpy[2]));
            break;
        }
    }

}

void RVizAffordanceTemplatePanel::changeRobot(int id) {
    QString r = ui_->robot_select->itemText(id);
    setupRobotPanel(r.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::changeEndEffector(int id) {
    QString ee = ui_->end_effector_select->itemText(id);
    setupEndEffectorConfigPanel(ee.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::deleteButton() {
    if(ui_->server_output_status->currentItem()) {
        killAffordanceTemplate(ui_->server_output_status->currentItem());
    }
}

void RVizAffordanceTemplatePanel::removeAffordanceTemplates() {
    affordanceTemplateGraphicsScene_->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: affordanceTemplateGraphicsScene_->items()) {
        affordanceTemplateGraphicsScene_->removeItem(pitem);
    }
    affordanceMap_.clear();
    affordanceTemplateGraphicsScene_->update();
}

void RVizAffordanceTemplatePanel::sendAffordanceTemplateAdd(const string& class_name) {
    Request req;
    req.set_type(Request::ADD);
    Template* temp(req.add_affordance_template());
    temp->set_type(class_name);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::sendAffordanceTemplateKill(const string& class_name, int id) {
    Request req;
    req.set_type(Request::KILL);
    Template* temp(req.add_affordance_template());
    temp->set_type(class_name);
    temp->set_id(id);
    Response resp;
    send_request(req, resp,10000000);
}

void RVizAffordanceTemplatePanel::killAffordanceTemplate(QListWidgetItem* item) {
    vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(template_info[1]) >> id;
    sendAffordanceTemplateKill(template_info[0], id);
    getRunningItems();
}


void RVizAffordanceTemplatePanel::removeRecognitionObjects() {
    recognitionObjectGraphicsScene_->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: recognitionObjectGraphicsScene_->items()) {
        recognitionObjectGraphicsScene_->removeItem(pitem);
    }
    recognitionObjectMap_.clear();
    recognitionObjectGraphicsScene_->update();
}

void RVizAffordanceTemplatePanel::sendRecognitionObjectAdd(const string& object_name) {
    Request req;
    req.set_type(Request::START_RECOGNITION);
    RecogObject* temp(req.add_recognition_object());
    temp->set_type(object_name);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::sendRecognitionObjectKill(const string& object_name, int id) {
    Request req;
    req.set_type(Request::KILL);
    RecogObject* temp(req.add_recognition_object());
    temp->set_type(object_name);
    temp->set_id(id);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::killRecognitionObject(QListWidgetItem* item) {
    vector<string> object_info = util::split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(object_info[1]) >> id;
    sendRecognitionObjectKill(object_info[0], id);
    getRunningItems();
}


void RVizAffordanceTemplatePanel::sendShutdown() {
    Request req;
    req.set_type(Request::SHUTDOWN);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::sendPing() {
    Request req;
    req.set_type(Request::PING);
    Response resp;
    send_request(req, resp);
    cout << resp.success() << endl;
}

void RVizAffordanceTemplatePanel::getRunningItems() {
    cout << "Requesting running Items..." << endl;
    Request req;
    req.set_type(Request::RUNNING);
    Response resp;
    send_request(req, resp, 30000000);
    ui_->server_output_status->clear();
    int id = 0;
    ui_->control_template_box->clear();
    for (auto& c: resp.affordance_template()) {
        string name = c.type() + ":" + to_string(c.id());
        ui_->server_output_status->addItem(QString::fromStdString(name));
        ui_->server_output_status->item(id)->setForeground(Qt::blue);
        id++;
        cout << "Found running AT: " << name << endl;
        ui_->control_template_box->addItem(QString(name.c_str()));
    }
    for (auto& c: resp.recognition_object()) {
        string name = c.type() + ":" + to_string(c.id());
        ui_->server_output_status->addItem(QString::fromStdString(name));
        ui_->server_output_status->item(id)->setForeground(Qt::green);
        id++;
        cout << "Found running RO: " << name << endl;
    }
    ui_->server_output_status->sortItems();
}


void RVizAffordanceTemplatePanel::safeLoadConfig() {
    if(!ui_->robot_lock->isChecked()) {
        cout << "Can't load while RobotConfig is unlocked" << endl;
        return;
    }
    loadConfig();
}

void RVizAffordanceTemplatePanel::loadConfig() {

    cout << "RVizAffordanceTemplatePanel::loadConfig() -- connected_: " << connected_ << endl;

    if (!connected_) {
        cout << "reconnecting..." << endl;
        connect();
    }

    cout << "RVizAffordanceTemplatePanel::loadConfig() -- WARNING::taking parameters loaded from original config, not the GUI yet!!! " << endl;

    Request req;
    req.set_type(Request::LOAD_ROBOT);
    Robot *robot = req.mutable_robot();
    Pose *robot_offset = robot->mutable_root_offset();
    EndEffectorMap *ee_map = robot->mutable_end_effectors();
    EndEffectorPoseIDMap *ee_pose_map = robot->mutable_end_effector_pose_ids();
    string key = ui_->robot_select->currentText().toUtf8().constData();
    string name = (*robotMap_[key]).name();
    string pkg = (*robotMap_[key]).moveit_config_package();
    string gripper_service = (*robotMap_[key]).gripper_service();
    string frame_id = (*robotMap_[key]).frame_id();
    vector<float> root_offset = (*robotMap_[key]).root_offset();

    robot->set_filename(key);
    robot->set_name(name);
    robot->set_moveit_config_package(pkg);
    robot->set_gripper_service(gripper_service);
    robot->set_frame_id(frame_id);
    Position *rop = robot_offset->mutable_position();
    Orientation *roo = robot_offset->mutable_orientation();
    rop->set_x(root_offset[0]);
    rop->set_y(root_offset[1]);
    rop->set_z(root_offset[2]);
    roo->set_x(root_offset[3]);
    roo->set_y(root_offset[4]);
    roo->set_z(root_offset[5]);
    roo->set_w(root_offset[6]);

    // remove all rows from before
    while(ui_->end_effector_table->rowCount()>0) {
        ui_->end_effector_table->removeCellWidget(0,0);
        ui_->end_effector_table->removeCellWidget(0,1);
        ui_->end_effector_table->removeCellWidget(0,2);
        ui_->end_effector_table->removeCellWidget(0,3);
        ui_->end_effector_table->removeRow(0);
    }

    int r = 0;
    for (auto& e: (*robotMap_[key]).endeffectorMap) {
        EndEffector *ee = ee_map->add_end_effector();
        Pose *ee_offset = ee->mutable_pose_offset();
        ee->set_name(e.second->name());
        ee->set_id(e.second->id());
        vector<float> pose_offset = e.second->pose_offset();
        Position *pop = ee_offset->mutable_position();
        Orientation *poo = ee_offset->mutable_orientation();
        pop->set_x(pose_offset[0]);
        pop->set_y(pose_offset[1]);
        pop->set_z(pose_offset[2]);
        poo->set_x(pose_offset[3]);
        poo->set_y(pose_offset[4]);
        poo->set_z(pose_offset[5]);
        poo->set_w(pose_offset[6]);

        // add rows to end effector controls table
        QTableWidgetItem *i= new QTableWidgetItem(QString(e.second->name().c_str()));
        ui_->end_effector_table->insertRow(r);

        ui_->end_effector_table->setItem(r,0,new QTableWidgetItem(QString(e.second->name().c_str())));
        ui_->end_effector_table->setItem(r,1,new QTableWidgetItem(QString("-")));
        ui_->end_effector_table->setItem(r,2,new QTableWidgetItem(QString("-")));

        QTableWidgetItem *pItem = new QTableWidgetItem();
        pItem->setCheckState(Qt::Checked);
        ui_->end_effector_table->setItem(r,3,pItem);

        r++;

    }

    cout << "ADDED END EFFECTOR POSE MAP TO REQUEST MESSAGE" << endl;
    for (auto& e: (*robotMap_[key]).endeffectorPoseMap) {
        EndEffectorPoseID *pid = ee_pose_map->add_pose_group();
        pid->set_name(e.second->name());
        pid->set_group(e.second->group());
        pid->set_id(e.second->id());
    }

    ui_->end_effector_table->resizeColumnsToContents();
    ui_->end_effector_table->resizeRowsToContents();

    Response resp;
    send_request(req, resp, 20000000);

    robot_name_ = key;
    controls_->setRobotName(robot_name_);

    enable_config_panel(Qt::Checked);
}


void RVizAffordanceTemplatePanel::addAffordanceDisplayItem() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem()" << endl;
    QList<QGraphicsItem*> list = affordanceTemplateGraphicsScene_->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- " << class_name << endl;
        sendAffordanceTemplateAdd(class_name);
        cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- retrieving waypoint info" << endl;
        for (auto& c: list.at(i)->data(WAYPOINT_DATA).toMap().toStdMap()) {
            string robot_key = ui_->robot_select->currentText().toUtf8().constData();
            for (auto& e: (*robotMap_[robot_name_]).endeffectorMap) {
                for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
                    if (e.second->name() == ui_->end_effector_table->item(r,0)->text().toStdString() ) {
                        ui_->end_effector_table->setItem(r,2,new QTableWidgetItem(QString::number(c.second.toInt())));
                    }
                }
            }
        }
    }
    // update running templates
    getRunningItems();
}

void RVizAffordanceTemplatePanel::addObjectDisplayItem() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addObjectDisplayItem()" << endl;
    QList<QGraphicsItem*> list = recognitionObjectGraphicsScene_->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string object_name = list.at(i)->data(OBJECT_INDEX).toString().toStdString();
        cout << "RVizAffordanceTemplatePanel::addObjectDisplayItem() -- " << object_name << endl;
        sendRecognitionObjectAdd(object_name);
    }
    // update running templates
    getRunningItems();
}


void RVizAffordanceTemplatePanel::send_request(const Request& request, Response& response, long timeout) {
    if (!connected_) {
        return;
    }

    bool success = util::send_request(socket_, request, response, timeout);

    if (!success) {
        disconnect();
    }
}

bool RVizAffordanceTemplatePanel::addAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (!checkAffordance(obj)) {
        affordanceMap_[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (checkAffordance(obj)) {
        affordanceMap_.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkAffordance(const AffordanceSharedPtr& obj) {
    if (affordanceMap_.find((*obj).key()) == affordanceMap_.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (!checkRecognitionObject(obj)) {
        recognitionObjectMap_[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (checkRecognitionObject(obj)) {
        recognitionObjectMap_.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    if (recognitionObjectMap_.find((*obj).key()) == recognitionObjectMap_.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (!checkRobot(obj)) {
        robotMap_[(*obj).uid()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (checkRobot(obj)) {
        robotMap_.erase((*obj).uid());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRobot(const RobotConfigSharedPtr& obj) {
    if (robotMap_.find((*obj).uid()) == robotMap_.end()) {
        return false;
    }
    return true;
}


void RVizAffordanceTemplatePanel::configChanged()
{
    rviz::Panel::configChanged();
}

std::string RVizAffordanceTemplatePanel::getRobotFromDescription() {
    std::string robot = "";
    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to parse robot_description rosparam");
    } else {
        cout << "RVizAffordanceTemplatePanel::getRobotFromDescription() -- found robot: " << model.name_ << endl;
        robot = model.name_;
    }
    return robot;
}

#include <pluginlib/class_list_macros.h>
#if ROS_VERSION_MINIMUM(1,9,41)
    // starting with Groovy
    // PLUGINLIB_EXPORT_CLASS(class_type, base_class_type)
    PLUGINLIB_EXPORT_CLASS(rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#else
    // PLUGINLIB_DECLARE_CLASS(pkg, class_name, class_type, base_class_type)
    PLUGINLIB_DECLARE_CLASS(rviz_affordance_template_panel, RVizAffordanceTemplatePanel, rviz_affordance_template_panel::RVizAffordanceTemplatePanel, rviz::Panel)
#endif


