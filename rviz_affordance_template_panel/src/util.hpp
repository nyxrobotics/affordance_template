#ifndef UTIL_HPP
#define UTIL_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <zmq.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <kdl/frames.hpp>

#include "AffordanceTemplateServerCmd.pb.h"

using namespace std;

namespace util
{

    zmq::socket_t* client_socket(zmq::context_t& context, const string& addr);
    vector<string> &split(const string &s, char delim, vector<string> &elems);
    vector<string> split(const string &s, char delim);
    vector<float> quaternionToRPY(float x, float y, float z, float w);
    vector<float> RPYToQuaternion(float rr, float rp, float ry);
    string resolvePackagePath(const string& str);
    bool send_request(zmq::socket_t* socket, const Request& request, Response& response, long timeout);

}

#endif // UTIL_HPP
