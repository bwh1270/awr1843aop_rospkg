#include "awr1843aop/Parser.hpp"


namespace awr1843aop
{

    PLUGINLIB_EXPORT_CLASS(awr1843aop::ParameterParser, nodelet::Nodelet);

    ParameterParser::ParameterParser() {}

    void ParameterParser::onInit() {}

    void ParameterParser::ParamsParser(awr1843aop::sensorCLI &srv, ros::NodeHandle &nh) {

        //   ROS_ERROR("%s",srv.request.comm.c_str());
        //   ROS_ERROR("%s",srv.response.resp.c_str());
        std::vector <std::string> v;
        std::string s = srv.request.req.c_str(); 
        std::istringstream ss(s);
        std::string token;
        std::string req;
        int i = 0;
        while (std::getline(ss, token, ' ')) {
            v.push_back(token);
            if (i > 0) {
                if (!req.compare("profileCfg")) {
                    switch (i) {
                        case 2:
                            nh.setParam("/awr1843aop/istartFreq", std::stof(token)); break;
                        case 3:
                            nh.setParam("/awr1843aop/idleTime", std::stof(token)); break;
                        case 4:
                            nh.setParam("/awr1843aop/adcStartTime", std::stof(token)); break;
                        case 5:
                            nh.setParam("/awr1843aop/rampEndTime", std::stof(token)); break;
                        case 8:
                            nh.setParam("/awr1843aop/freqSlopeConst", std::stof(token)); break;
                        case 10:
                            nh.setParam("/awr1843aop/numAdcSamples", std::stoi(token)); break;
                        case 11:
                            nh.setParam("/awr1843aop/digOutSampleRate", std::stof(token)); break;
                        case 14:
                            nh.setParam("/awr1843aop/rxGain", std::stof(token)); break;
                    }
                } else if (!req.compare("frameCfg")) {
                    switch (i) {
                    case 1:
                        nh.setParam("/awr1843aop/chirpStartIdx", std::stoi(token)); break;
                    case 2:
                        nh.setParam("/awr1843aop/chirpEndIdx", std::stoi(token)); break;
                    case 3:
                        nh.setParam("/awr1843aop/numLoops", std::stoi(token)); break;
                    case 4:
                        nh.setParam("/awr1843aop/numFrames", std::stoi(token)); break;
                    case 5:
                        nh.setParam("/awr1843aop/framePeriodicity", std::stof(token)); break;
                    }
                }
            } else req = token;
            i++;
        }
    }

    void ParameterParser::CalParams(ros::NodeHandle &nh) {
        float c0 = 299792458;
        int chirpStartIdx;
        int chirpEndIdx;
        int numLoops;
        float framePeriodicity;
        float startFreq;
        float idleTime;
        float adcStartTime;
        float rampEndTime;
        float digOutSampleRate;
        float freqSlopeConst;
        float numAdcSamples;

        nh.getParam("/awr1843aop/startFreq", startFreq);
        nh.getParam("/awr1843aop/idleTime", idleTime);
        nh.getParam("/awr1843aop/adcStartTime", adcStartTime);
        nh.getParam("/awr1843aop/rampEndTime", rampEndTime);
        nh.getParam("/awr1843aop/digOutSampleRate", digOutSampleRate);
        nh.getParam("/awr1843aop/freqSlopeConst", freqSlopeConst);
        nh.getParam("/awr1843aop/numAdcSamples", numAdcSamples);

        nh.getParam("/awr1843aop/chirpStartIdx", chirpStartIdx);
        nh.getParam("/awr1843aop/chirpEndIdx", chirpEndIdx);
        nh.getParam("/awr1843aop/numLoops", numLoops);
        nh.getParam("/awr1843aop/framePeriodicity", framePeriodicity);

        int nTx = chirpEndIdx - chirpStartIdx + 1;
        int nd = numLoops;
        int nAdc = numAdcSamples;
        float tfr = framePeriodicity * 1e-3;
        float fs = digOutSampleRate * 1e3;
        float kf = freqSlopeConst * 1e12;
        float adc_duration = nAdc / fs;
        float BW = adc_duration * kf;
        float PRI = (idleTime + rampEndTime) * 1e-6;
        float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 

        float rRes = c0 / (2 * BW);
        float rMax = nAdc * rRes;
        float vMax = c0 / (2 * fc * PRI) / nTx;
        float vRes = vMax / nd;

        nh.setParam("/awr1843aop/num_TX", nTx);
        nh.setParam("/awr1843aop/f_s", fs);
        nh.setParam("/awr1843aop/f_c", fc);
        nh.setParam("/awr1843aop/BW", BW);
        nh.setParam("/awr1843aop/PRI", PRI);
        nh.setParam("/awr1843aop/t_fr", tfr);
        nh.setParam("/awr1843aop/max_range", rMax);
        nh.setParam("/awr1843aop/range_resolution", rRes);
        nh.setParam("/awr1843aop/max_doppler_vel", vMax);
        nh.setParam("/awr1843aop/doppler_vel_resolution", vRes);
    }

}
