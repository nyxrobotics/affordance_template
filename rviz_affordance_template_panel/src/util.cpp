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
    // rpy[0] = 0;
    // rpy[1] = 0;
    // rpy[2] = 0;
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

}