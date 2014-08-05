#ifndef UTIL_HPP
#define UTIL_HPP

#include <zmq.hpp>
#include <vector>
#include <iostream>
#include <sstream>
#include <kdl/frames.hpp>

using namespace std;

namespace util
{

    zmq::socket_t* client_socket (zmq::context_t& context, const string& addr);
    vector<string> &split(const string &s, char delim, vector<string> &elems);
    vector<string> split(const string &s, char delim);
    vector<float> quaternionToRPY(float x, float y, float z, float w);
    vector<float> RPYToQuaternion(float rr, float rp, float ry);

}

#endif // UTIL_HPP