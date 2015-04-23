#pragma once

#include "statisticSampler.h" // base class
#include "types.h"

namespace eq
{
    class StatisticsProxy
    {
    public:
        const UUID& getID() const { return _id; };
        uint32_t getSerial() const { return _serial; }
        uint32_t getCurrentFrame() const { return _frame; }
        Config* getConfig() const { return _config; }
        co::LocalNodePtr getLocalNode() const { return _localNode; }
        std::string getName() const { return _name; }
        uint32_t getTaskID() const { return _taskID; }

        UUID _id;
        uint32_t _serial;
        uint32_t _frame;
        Config* _config;
        co::LocalNodePtr _localNode;
        std::string _name;
        uint32_t _taskID;
    };


    /** Holds one proxied statistics event, used for profiling. */
    class ProxyStatistics : public StatisticSampler< StatisticsProxy >
    {
    public:
        ProxyStatistics( const Statistic::Type type, StatisticsProxy* proxy,
                        const uint32_t frameNumber );
        ~ProxyStatistics();
    };
}

