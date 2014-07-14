/* Qt Includes */
//#include <QObjectList>

// TODO: don't like these
#define PIXMAP_SIZE 100
#define XOFFSET 20
#define YOFFSET 20
#define CLASS_INDEX 0

/* Project Includes */
#include "rviz_affordance_template_panel.hpp"
//#include "rviz_control_panel.hpp"


using namespace rviz_affordance_template_panel;
using namespace std;

/** \brief Helper function to return a new socket.
 */
static zmq::socket_t* client_socket (zmq::context_t& context, const string& addr) {
    zmq::socket_t* client = new zmq::socket_t (context, ZMQ_REQ);
    string serv = "tcp://" + addr;
    client->connect(serv.c_str());

    // configure socket to not wait at close time
    int linger = 0;
    client->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    return client;
}

static vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/** \brief Split a string by a delimiter and return it in a vector.
 */
static vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}



RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel(QWidget *parent) :
    rviz::Panel(parent),
    context(1),
    connected(false),
    _ui(new Ui::RVizAffordanceTemplatePanel),
    descriptionRobot(""),
    force_load(true)
{
    // Setup the panel.
    _ui->setupUi(this);
    setupWidgets();
    connect();

    descriptionRobot = getRobotFromDescription();

    if (descriptionRobot != "") {
        std::string yamlRobotCandidate = descriptionRobot + ".yaml";
        cout << "RVizAffordanceTemplatePanel::RVizAffordanceTemplatePanel() -- searching for Robot: " << yamlRobotCandidate << endl;
        map<string,RobotConfigSharedPtr>::const_iterator it = robotMap.find(yamlRobotCandidate);
        if (it != robotMap.end() ) {
            setupRobotPanel(yamlRobotCandidate);
            loadConfig();
            int idx= _ui->robot_select->findText(QString(yamlRobotCandidate.c_str()));
            _ui->robot_select->setCurrentIndex(idx);
        }
    }

}

RVizAffordanceTemplatePanel::~RVizAffordanceTemplatePanel()
{
    delete _ui;
}

void RVizAffordanceTemplatePanel::setupWidgets() {

    runningList = new QListWidget();
    runningList->setMinimumHeight(200);
    QObject::connect(runningList, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(killTemplate(QListWidgetItem*)));

    graphicsScene = new QGraphicsScene(this);
    _ui->graphicsView->setScene(graphicsScene);

    QObject::connect(_ui->connect_button, SIGNAL(clicked()), this, SLOT(connect()));
    QObject::connect(_ui->load_config_button, SIGNAL(clicked()), this, SLOT(safeLoadConfig()));
    QObject::connect(_ui->robot_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeRobot(int)));
}

void RVizAffordanceTemplatePanel::connect() {
    // return if already connected
    bool success = false;
    if (connected) {
        try {
            runningList->clear();
            removeTemplates();
            socket->close();
            connected = false;
            delete socket;
            success = true;
        } catch (const zmq::error_t& ex) {
            cerr << ex.what() << endl;
        }
        cout << "disconnect: " << success << endl;
        if (success) {
            _ui->server_connection_label->setText("disconnected");
            _ui->server_connection_label->setStyleSheet("QLabel {color: red;}");
            _ui->connect_button->setText("Connect");
        }
    } else {
        try {
            string addr = _ui->server_text->text().toStdString();
            socket = client_socket(context, addr);
            connected = true;
            sendPing();
            getAvailableTemplates();
            getRunningTemplates();
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
}

void RVizAffordanceTemplatePanel::getAvailableTemplates() {
    if (connected) {
        // templates must be exported in plugin_description.xml of affordance_template package
        Request req;
        req.set_type(Request::QUERY);
        Response rep;
        send_request(req, rep);
        int yoffset = YOFFSET;

        string package_path = ros::package::getPath("affordance_template_library");
        for (auto& c: rep.affordance_template()) {

            cout << c.type() << endl;

            string image_path = resolvePackagePath(c.image_path());

            std::cout << image_path << std::endl;
            AffordanceSharedPtr pitem(new Affordance(c.type(), image_path));

            pitem->setPos(XOFFSET, yoffset);
            yoffset += PIXMAP_SIZE + YOFFSET;

            graphicsScene->addItem(pitem.get());
            addAffordance(pitem);
        }

        cout << "setting up addTemplate callback" << endl;
        QObject::connect(graphicsScene, SIGNAL(selectionChanged()), this, SLOT(addTemplate()));
        graphicsScene->update();

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
            addRobot(pitem);

            _ui->robot_select->addItem(QString(pitem->uid().c_str()));
        }

        setupRobotPanel(robotMap.begin()->first);
    }
}

void RVizAffordanceTemplatePanel::setupRobotPanel(const string& key) {

    cout << "setupRobotPanel for " << key << endl;
    string name = (*robotMap[key]).name();
    string pkg = (*robotMap[key]).moveit_config_package();
    string frame_id = (*robotMap[key]).frame_id();
    vector<float> root_offset = (*robotMap[key]).root_offset();

    _ui->robot_name->setText(QString(name.c_str()));
    _ui->moveit_package->setText(QString(pkg.c_str()));
    _ui->frame_id->setText(QString(frame_id.c_str()));

    _ui->robot_tx->setText(QString::number(root_offset[0]));
    _ui->robot_ty->setText(QString::number(root_offset[1]));
    _ui->robot_tz->setText(QString::number(root_offset[2]));

    _ui->robot_rr->setText(QString::number(0));
    _ui->robot_rp->setText(QString::number(0));
    _ui->robot_ry->setText(QString::number(0));

    _ui->end_effector_select->clear();

    cout << "setupRobotPanel() -- adding end_effectors" << endl;
    for (auto& e: (*robotMap[key]).endeffectorMap) {
        _ui->end_effector_select->addItem(e.second->name().c_str());
    }

    setupEndEffectorConfigPanel((*robotMap[key]).endeffectorMap.begin()->first);

    QObject::connect(_ui->end_effector_select, SIGNAL(currentIndexChanged(int)), this, SLOT(changeEndEffector(int)));

}

void RVizAffordanceTemplatePanel::setupEndEffectorConfigPanel(const string& key) {

    cout << "setupEndEffectorConfigPanel() -- setting panel data: " << key << endl;

    string robot_key = _ui->robot_select->currentText().toUtf8().constData();

//    EndEffectorConfig e = (*(*robotMap[robot_key]).endeffectorMap[key]);

    for (auto& e: (*robotMap[robot_key]).endeffectorMap) {
        if (e.second->name() == key) {
            _ui->ee_name->setText(e.second->name().c_str());
            _ui->ee_id->setText(QString::number(e.second->id()));
            vector<float> pose_offset = e.second->pose_offset();
            _ui->ee_tx->setText(QString::number(pose_offset[0]));
            _ui->ee_ty->setText(QString::number(pose_offset[1]));
            _ui->ee_tz->setText(QString::number(pose_offset[2]));
            _ui->ee_rr->setText(QString::number(pose_offset[3]));
            _ui->ee_rp->setText(QString::number(pose_offset[4]));
            _ui->ee_ry->setText(QString::number(pose_offset[5]));
            //_ui->ee_tw(pose_offset[6]);
            break;
        }
    }

}

void RVizAffordanceTemplatePanel::changeRobot(int id) {
    QString r = _ui->robot_select->itemText(id);
    setupRobotPanel(r.toUtf8().constData());
}

void RVizAffordanceTemplatePanel::changeEndEffector(int id) {
    cout << "change eeeee" << endl;
    QString ee = _ui->end_effector_select->itemText(id);
    setupEndEffectorConfigPanel(ee.toUtf8().constData());
}

string RVizAffordanceTemplatePanel::resolvePackagePath(const string& str) {
  string package_prefix_str ("package://");
  size_t found = str.find(package_prefix_str);
  if (found==string::npos) return str;
  string sub_str = str.substr(package_prefix_str.length(),str.length()-1);
  string delimiter = "/";
  string package_name = sub_str.substr(0, sub_str.find(delimiter));
  string package_path = ros::package::getPath(package_name);
  string file_path = sub_str.substr(package_name.length(),sub_str.length()-1);
  return package_path + file_path;
}

void RVizAffordanceTemplatePanel::removeTemplates() {
    graphicsScene->disconnect(SIGNAL(selectionChanged()));
    for (auto& pitem: graphicsScene->items()) {
        graphicsScene->removeItem(pitem);
    }
    affordanceMap.clear();
    graphicsScene->update();
}

void RVizAffordanceTemplatePanel::sendAdd(const string& class_name) {
    Request req;
    req.set_type(Request::ADD);
    Template* temp(req.add_affordance_template());
    temp->set_type(class_name);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::sendKill(const string& class_name, int id) {
    Request req;
    req.set_type(Request::KILL);
    Template* temp(req.add_affordance_template());
    temp->set_type(class_name);
    temp->set_id(id);
    Response resp;
    send_request(req, resp);
}

void RVizAffordanceTemplatePanel::killTemplate(QListWidgetItem* item) {
    vector<string> template_info = split(item->text().toUtf8().constData(), ':');
    int id;
    istringstream(template_info[1]) >> id;
    sendKill(template_info[0], id);
    getRunningTemplates();
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

void RVizAffordanceTemplatePanel::getRunningTemplates() {
    Request req;
    req.set_type(Request::RUNNING);
    Response resp;
    send_request(req, resp, 10000000);
    runningList->clear();
    for (auto& c: resp.affordance_template()) {
        string name = c.type() + ":" + to_string(c.id());
        runningList->addItem(QString::fromStdString(name));
    }
    runningList->sortItems();
}

void RVizAffordanceTemplatePanel::safeLoadConfig() {
    if(_ui->robot_lock->isChecked()) {
        cout << "Can't load while RobotConfig is locked" << endl;
        return;
    }
    loadConfig();
}

void RVizAffordanceTemplatePanel::loadConfig() {

    cout << "RVizAffordanceTemplatePanel::loadConfig() -- connected: " << connected << endl;
    if (!connected) {
        cout << "reconnecting..." << endl;
        connect();
    }

    Request req;
    req.set_type(Request::LOAD_ROBOT);
    Robot *robot = req.mutable_robot();
    Pose *robot_offset = robot->mutable_root_offset();
    EndEffectorMap *ee_map = robot->mutable_end_effectors();
    string key = _ui->robot_select->currentText().toUtf8().constData();
    string name = (*robotMap[key]).name();
    string pkg = (*robotMap[key]).moveit_config_package();
    string frame_id = (*robotMap[key]).frame_id();
    vector<float> root_offset = (*robotMap[key]).root_offset();

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

    for (auto& e: (*robotMap[key]).endeffectorMap) {
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
    }

    Response resp;
    send_request(req, resp, 20000000);

}


void RVizAffordanceTemplatePanel::addTemplate() {
    // Add an object template to the InteractiveMarkerServer for each selected item.
    cout << "RVizAffordanceTemplatePanel::addTemplate()" << endl;
    QList<QGraphicsItem*> list = graphicsScene->selectedItems();
    for (int i=0; i < list.size(); ++i) {
        // Get the object template class name from the first element in the QGraphicsItem's custom data
        // field. This field is set in the derived Affordance class when setting up the widgets.
        string class_name = list.at(i)->data(CLASS_INDEX).toString().toStdString();
        cout << "RVizAffordanceTemplatePanel::addTemplate() -- " << class_name << endl;
        sendAdd(class_name);
    }
    // update running templates
    getRunningTemplates();
}

void RVizAffordanceTemplatePanel::send_request(const Request& request, Response& response, long timeout) {
    cout << "RVizAffordanceTemplatePanel::send_request() 1" << endl;
    if (connected) {
        cout << "RVizAffordanceTemplatePanel::send_request() 2" << endl;
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

            cout << "RVizAffordanceTemplatePanel::send_request() 3" << endl;

            } else {
                cout << "RVizAffordanceTemplatePanel::send_request() discon" << endl;
                disconnect();
            }
        } catch (const zmq::error_t& ex) {
            cerr << ex.what() << endl;
        }
    }
    cout << "RVizAffordanceTemplatePanel::send_request() end" << endl;

}

bool RVizAffordanceTemplatePanel::addAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (!checkAffordance(obj)) {
        affordanceMap[(*obj).key()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeAffordance(const AffordanceSharedPtr& obj) {
    // check if template is in our map
    if (checkAffordance(obj)) {
        affordanceMap.erase((*obj).key());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkAffordance(const AffordanceSharedPtr& obj) {
    if (affordanceMap.find((*obj).key()) == affordanceMap.end()) {
        return false;
    }
    return true;
}

bool RVizAffordanceTemplatePanel::addRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (!checkRobot(obj)) {
        robotMap[(*obj).uid()] = obj;
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::removeRobot(const RobotConfigSharedPtr& obj) {
    // check if robot is in our map
    if (checkRobot(obj)) {
        robotMap.erase((*obj).uid());
        return true;
    }
    return false;
}

bool RVizAffordanceTemplatePanel::checkRobot(const RobotConfigSharedPtr& obj) {
    if (robotMap.find((*obj).uid()) == robotMap.end()) {
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


