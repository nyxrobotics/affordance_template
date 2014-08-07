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
    _context(1),
    _connected(false),
    _ui(new Ui::RVizAffordanceTemplatePanel),
    _descriptionRobot(""),
    _controls(new Controls(_ui))
{
    // Setup the panel.
    _ui->setupUi(this);

    setupWidgets();
    connect();

    _descriptionRobot = getRobotFromDescription();

    if (_descriptionRobot != "") {
        std::string yamlRobotCandidate = _descriptionRobot + ".yaml";
        cout << "RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel() -- searching for Robot: " << yamlRobotCandidate << endl;
        map<string,RobotConfigSharedPtr>::const_iterator it = _robotMap.find(yamlRobotCandidate);
        if (it != _robotMap.end() ) {
            int idx= _ui->robot_select->findText(QString(yamlRobotCandidate.c_str()));
            _ui->robot_select->setCurrentIndex(idx);
            setupRobotPanel(yamlRobotCandidate);
            loadConfig();
        }
    }

}

RVizAffordanceTemplatePanel::~RVizAffordanceTemplatePanel()
{
    delete _ui;
}

void RVizAffordanceTemplatePanel::setupWidgets() {

    _affordanceTemplateGraphicsScene = new QGraphicsScene(this);
    _ui->affordanceTemplateGraphicsView->setScene(_affordanceTemplateGraphicsScene);

    _recognitionObjectGraphicsScene = new QGraphicsScene(this);
    _ui->recognitionObjectGraphicsView->setScene(_recognitionObjectGraphicsScene);

    QObject::connect(_ui->server_output_status, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(killAffordanceTemplate(QListWidgetItem*)));

    QObject::connect(_ui->connect_button, SIGNAL(clicked()), this, SLOT(connect_callback()));
    QObject::connect(_ui->load_config_button, SIGNAL(clicked()), this, SLOT(safeLoadConfig()));
    QObject::connect(_ui->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));

    QObject::connect(_ui->go_to_start_button, SIGNAL(clicked()), this, SLOT(go_to_start()));
    QObject::connect(_ui->go_to_end_button, SIGNAL(clicked()), this, SLOT(go_to_end()));
    QObject::connect(_ui->pause_button, SIGNAL(clicked()), this, SLOT(pause()));
    QObject::connect(_ui->play_backwards_button, SIGNAL(clicked()), this, SLOT(play_backward()));
    QObject::connect(_ui->play_button, SIGNAL(clicked()), this, SLOT(play_forward()));
    QObject::connect(_ui->step_backwards_button, SIGNAL(clicked()), this, SLOT(step_backward()));
    QObject::connect(_ui->step_forward_button, SIGNAL(clicked()), this, SLOT(step_forward()));

    QObject::connect(_ui->refresh_button, SIGNAL(clicked()), this, SLOT(getAvailableInfo()));

}

void RVizAffordanceTemplatePanel::connect_callback() {
    if (_connected) {
        disconnect();
    } else {
        connect();
    }
    _controls->setConnected(_connected);
}

void RVizAffordanceTemplatePanel::disconnect() {
    bool success = false;
    try {
        _ui->server_output_status->clear();
        removeAffordanceTemplates();
        removeRecognitionObjects();
        _socket->close();
        _connected = false;
        delete _socket;
        success = true;
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
    if (success) {
        _ui->server_connection_label->setText("disconnected");
        _ui->server_connection_label->setStyleSheet("QLabel {color: red;}");
        _ui->connect_button->setText("Connect");
    }
}

void RVizAffordanceTemplatePanel::connect() {
    // return if already connected
    bool success = false;
    try {
        string addr = _ui->server_text->text().toStdString();
        _socket = util::client_socket(_context, addr);
        _controls->setSocket(_socket);
        _connected = true;
        sendPing();
        getAvailableInfo();
        getRunningItems();
        success = true;
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
    }
    cout << "connect: " << success << endl;
    if (success) {
        _ui->server_connection_label->setText("connected");
        _ui->server_connection_label->setStyleSheet("QLabel {color: green;}");
        _ui->connect_button->setText("Disconnect");
    }
}

void RVizAffordanceTemplatePanel::getAvailableInfo() {
    if (!_connected) {
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
        _affordanceTemplateGraphicsScene->addItem(pitem.get());
        addAffordance(pitem);
    }
    cout << "setting up addAffordanceDisplayItem callback" << endl;
    QObject::connect(_affordanceTemplateGraphicsScene, SIGNAL(selectionChanged()), this, SLOT(addAffordanceDisplayItem()));
    _affordanceTemplateGraphicsScene->update();

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
        _recognitionObjectGraphicsScene->addItem(pitem.get());
        addRecognitionObject(pitem);
    }
    cout << "setting up addObjectDisplayItem callback" << endl;
    QObject::connect(_recognitionObjectGraphicsScene, SIGNAL(selectionChanged()), this, SLOT(addObjectDisplayItem()));
    _recognitionObjectGraphicsScene->update();

    // load stuff for robot config sub panel
    _ui->robot_select->disconnect(SIGNAL(currentIndexChanged(int)));
    _ui->end_effector_select->disconnect(SIGNAL(currentIndexChanged(int)));
    _ui->robot_select->clear();
    _ui->end_effector_select->clear();

    for (auto& r: rep.robot()) {

        RobotConfigSharedPtr pitem(new RobotConfig(r.filename()));
        pitem->uid(r.filename());
        pitem->name(r.name());
        pitem->moveit_config_package(r.moveit_config_package());
        pitem->frame_id(r.frame_id());

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

        _ui->robot_select->addItem(QString(pitem->uid().c_str()));
    }

    setupRobotPanel(_robotMap.begin()->first);
    QObject::connect(_ui->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
    QObject::connect(_ui->end_effector_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeEndEffector(int)));

    // set Controls
    _controls->setRobotMap(_robotMap);
}

void RVizAffordanceTemplatePanel::setupRobotPanel(const string& key) {

    cout << "setupRobotPanel for " << key << endl;
    string name = (*_robotMap[key]).name();
    string pkg = (*_robotMap[key]).moveit_config_package();
    string frame_id = (*_robotMap[key]).frame_id();
    vector<float> root_offset = (*_robotMap[key]).root_offset();

    _ui->robot_name->setText(QString(name.c_str()));
    _ui->moveit_package->setText(QString(pkg.c_str()));
    _ui->frame_id->setText(QString(frame_id.c_str()));

    _ui->robot_tx->setText(QString::number(root_offset[0]));
    _ui->robot_ty->setText(QString::number(root_offset[1]));
    _ui->robot_tz->setText(QString::number(root_offset[2]));

    cout << root_offset[3] << endl;
    cout << root_offset[4] << endl;
    cout << root_offset[5] << endl;
    cout << root_offset[6] << endl;

    vector<float> rpy = util::quaternionToRPY(root_offset[3],root_offset[4],root_offset[5],root_offset[6]);

    _ui->robot_rr->setText(QString::number(rpy[0]));
    _ui->robot_rp->setText(QString::number(rpy[1]));
    _ui->robot_ry->setText(QString::number(rpy[2]));

    _ui->end_effector_select->clear();

    cout << "setupRobotPanel() -- adding end_effectors" << endl;
    for (auto& e: (*_robotMap[key]).endeffectorMap) {
        _ui->end_effector_select->addItem(e.second->name().c_str());
    }

    setupEndEffectorConfigPanel((*_robotMap[key]).endeffectorMap.begin()->first);

}

void RVizAffordanceTemplatePanel::setupEndEffectorConfigPanel(const string& key) {

    cout << "setupEndEffectorConfigPanel() -- setting panel data: " << key << endl;
    string robot_key = _ui->robot_select->currentText().toUtf8().constData();

    for (auto& e: (*_robotMap[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            _ui->ee_name->setText(e.second->name().c_str());
            _ui->ee_id->setText(QString::number(e.second->id()));
            vector<float> pose_offset = e.second->pose_offset();
            _ui->ee_tx->setText(QString::number(pose_offset[0]));
            _ui->ee_ty->setText(QString::number(pose_offset[1]));
            _ui->ee_tz->setText(QString::number(pose_offset[2]));

            vector<float> rpy = util::quaternionToRPY(pose_offset[3],pose_offset[4],pose_offset[5],pose_offset[6]);
            _ui->ee_rr->setText(QString::number(rpy[0]));
            _ui->ee_rp->setText(QString::number(rpy[1]));
            _ui->ee_ry->setText(QString::number(rpy[2]));
            break;
        }
    }

}

void RVizAffordanceTemplatePanel::changeRobot(int id) {
    QString r = _ui->robot_select->itemText(id);
    setupRobotPanel(r.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::changeEndEffector(int id) {
    QString ee = _ui->end_effector_select->itemText(id);
    setupEndEffectorConfigPanel(ee.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::removeAffordanceTemplates() {
    _affordanceTemplateGraphicsScene->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: _affordanceTemplateGraphicsScene->items()) {
        _affordanceTemplateGraphicsScene->removeItem(pitem);
    }
    _affordanceMap.clear();
    _affordanceTemplateGraphicsScene->update();
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
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::killAffordanceTemplate(QListWidgetItem* item) {
    vector<string> template_info = util::split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(template_info[1]) >> id;
    sendAffordanceTemplateKill(template_info[0], id);
    getRunningItems();
}


void RVizAffordanceTemplatePanel::removeRecognitionObjects() {
    _recognitionObjectGraphicsScene->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: _recognitionObjectGraphicsScene->items()) {
        _recognitionObjectGraphicsScene->removeItem(pitem);
    }
    _recognitionObjectMap.clear();
    _recognitionObjectGraphicsScene->update();
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
    _ui->server_output_status->clear();
    int id = 0;
    _ui->control_template_box->clear();
    for (auto& c: resp.affordance_template()) {
        string name = c.type() + ":" + to_string(c.id());
        _ui->server_output_status->addItem(QString::fromStdString(name));
        _ui->server_output_status->item(id)->setForeground(Qt::blue);
        id++;
        cout << "Found running AT: " << name << endl;
        _ui->control_template_box->addItem(QString(name.c_str()));
    }
    for (auto& c: resp.recognition_object()) {
        string name = c.type() + ":" + to_string(c.id());
        _ui->server_output_status->addItem(QString::fromStdString(name));
        _ui->server_output_status->item(id)->setForeground(Qt::green);
        id++;
        cout << "Found running RO: " << name << endl;
    }
    _ui->server_output_status->sortItems();
}


void RVizAffordanceTemplatePanel::safeLoadConfig() {
    if(_ui->robot_lock->isChecked()) {
        cout << "Can't load while RobotConfig is locked" << endl;
        return;
    }
    loadConfig();
}

void RVizAffordanceTemplatePanel::loadConfig() {

    cout << "RVizAffordanceTemplatePanel::loadConfig() -- _connected: " << _connected << endl;

    if (!_connected) {
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
    string key = _ui->robot_select->currentText().toUtf8().constData();
    string name = (*_robotMap[key]).name();
    string pkg = (*_robotMap[key]).moveit_config_package();
    string frame_id = (*_robotMap[key]).frame_id();
    vector<float> root_offset = (*_robotMap[key]).root_offset();

    robot->set_filename(key);
    robot->set_name(name);
    robot->set_moveit_config_package(pkg);
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
    while(_ui->end_effector_table->rowCount()>0) {
        _ui->end_effector_table->removeCellWidget(0,0);
        _ui->end_effector_table->removeCellWidget(0,1);
        _ui->end_effector_table->removeCellWidget(0,2);
        _ui->end_effector_table->removeCellWidget(0,3);
        _ui->end_effector_table->removeRow(0);
    }

    int r = 0;
    for (auto& e: (*_robotMap[key]).endeffectorMap) {
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
        _ui->end_effector_table->insertRow(r);

        _ui->end_effector_table->setItem(r,0,new QTableWidgetItem(QString(e.second->name().c_str())));
        _ui->end_effector_table->setItem(r,1,new QTableWidgetItem(QString("-")));
        _ui->end_effector_table->setItem(r,2,new QTableWidgetItem(QString("-")));

        QTableWidgetItem *pItem = new QTableWidgetItem();
        pItem->setCheckState(Qt::Checked);
        _ui->end_effector_table->setItem(r,3,pItem);

        r++;

    }

    cout << "ADDED END EFFECTOR POSE MAP TO REQUEST MESSAGE" << endl;
    for (auto& e: (*_robotMap[key]).endeffectorPoseMap) {
        EndEffectorPoseID *pid = ee_pose_map->add_pose_group();
        pid->set_name(e.second->name());
        pid->set_group(e.second->group());
        pid->set_id(e.second->id());
    }

    _ui->end_effector_table->resizeColumnsToContents();
    _ui->end_effector_table->resizeRowsToContents();

    Response resp;
    send_request(req, resp, 20000000);

    _robot_name = key;
    _controls->setRobotName(_robot_name);
}


void RVizAffordanceTemplatePanel::addAffordanceDisplayItem() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem()" << endl;
    QList<QGraphicsItem*> list = _affordanceTemplateGraphicsScene->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- " << class_name << endl;
        sendAffordanceTemplateAdd(class_name);
        cout << "RVizAffordanceTemplatePanel::addAffordanceDisplayItem() -- retrieving waypoint info" << endl;
        for (auto& c: list.at(i)->data(WAYPOINT_DATA).toMap().toStdMap()) {
            string robot_key = _ui->robot_select->currentText().toUtf8().constData();
            for (auto& e: (*_robotMap[_robot_name]).endeffectorMap) {
                for (int r=0; r<_ui->end_effector_table->rowCount(); r++ ) {
                    if (e.second->name() == _ui->end_effector_table->item(r,0)->text().toStdString() ) {
                        _ui->end_effector_table->setItem(r,2,new QTableWidgetItem(QString::number(c.second.toInt())));
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
    QList<QGraphicsItem*> list = _recognitionObjectGraphicsScene->selectedItems();
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
    if (!_connected) {
        return;
    }

    bool success = util::send_request(_socket, request, response, timeout);

    if (!success) {
        disconnect();
    }
}

bool RVizAffordanceTemplatePanel::addAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (!checkAffordance(obj)) {
        _affordanceMap[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (checkAffordance(obj)) {
        _affordanceMap.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkAffordance(const AffordanceSharedPtr& obj) {
    if (_affordanceMap.find((*obj).key()) == _affordanceMap.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (!checkRecognitionObject(obj)) {
        _recognitionObjectMap[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    // check if template is in our map
    if (checkRecognitionObject(obj)) {
        _recognitionObjectMap.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRecognitionObject(const RecognitionObjectSharedPtr& obj) {
    if (_recognitionObjectMap.find((*obj).key()) == _recognitionObjectMap.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (!checkRobot(obj)) {
        _robotMap[(*obj).uid()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (checkRobot(obj)) {
        _robotMap.erase((*obj).uid());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRobot(const RobotConfigSharedPtr& obj) {
    if (_robotMap.find((*obj).uid()) == _robotMap.end()) {
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


