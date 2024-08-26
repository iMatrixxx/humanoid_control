#ifndef MPCIOLCM_H
#define MPCIOLCM_H

#include <boost/array.hpp>
#include <string>
// #include "../common/cppTypes.h"
// #include "convexMPC_interface.h"

#include <lcm/lcm-cpp.hpp>
#include "RealDesLcm.hpp"
#include "RealFbLcm.hpp"
#include <mutex>
#include <thread>

//接受Fb，发送Des


class IOLcm
{
    public:
        IOLcm(std::string robot_name);
        ~IOLcm();
        void sendOutput(const std::vector<double> &action);
        // void recvInput();
        bool ifRecv(){ return _if_recv; }
        void get_status(std::vector<double>& status);

        lcm::LCM _lcm;
    private:
        RealFbLcm _lcm_Rec_Fb;
        RealDesLcm _lcm_Pub_Des;

        std::string _robot_name;

        bool _if_recv;
        bool _running;

        // lcm::LCM _lcm;
        std::thread _lcmThread;

        void initRecv(); // initialize subscribers

        void InputCallback(const lcm::ReceiveBuffer* rbuf, const std::string& chan,  const RealFbLcm* msg);
        
        void lcmHandler();
};   

#endif