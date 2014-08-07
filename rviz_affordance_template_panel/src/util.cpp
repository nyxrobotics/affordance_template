#include "util.hpp"

namespace util {

/** \brief Helper function to return a new socket.
 */
zmq::socket_t* client_socket (zmq::context_t& context, const string& addr) {
    zmq::socket_t* client = new zmq::socket_t (context, ZMQ_REQ);
    string serv = "tcp://" + addr;
    client->connect(serv.c_str());

    // configure socket to not wait at close time
    int linger = 0;
    client->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    return client;
}

vector<string> &split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/** \brief Split a string by a delimiter and return it in a vector.
 */
vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}

/** \brief KDL helper function convert quaternion to rpy angles
*/
vector<float> quaternionToRPY(float x, float y, float z, float w) {
    vector<float> rpy(3);
    double rr, rp, ry;
    KDL::Rotation::Quaternion(x,y,z,w).GetRPY(rr,rp,ry);
    rpy[0] = (float)rr;
    rpy[1] = (float)rp;
    rpy[2] = (float)ry;
    cout << "converted quaternion(" << x << ", " << y << ", " << z << ", " << w << ") to rpy(" << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << ")" << endl;
    return rpy;
}

/** \brief KDL helper function convert rpy angles to quaternion
*/
vector<float> RPYToQuaternion(float rr, float rp, float ry) {
    vector<float> q(4);
    double x,y,z,w;
    KDL::Rotation::RPY(rr,rp,ry).GetQuaternion(x,y,z,w);
    q[0] = (float)x;
    q[1] = (float)y;
    q[2] = (float)z;
    q[3] = (float)w;
    return q;
}

string resolvePackagePath(const string& str) {
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

bool send_request(zmq::socket_t* socket, const Request& request, Response& response, long timeout) {
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
            return false;
        }
    } catch (const zmq::error_t& ex) {
        cerr << ex.what() << endl;
        return false;
    }

    return true;
}

}