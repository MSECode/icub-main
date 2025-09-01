/*
 * SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RawValuesPublisherRemapper.h"


#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

namespace {
YARP_LOG_COMPONENT(RAWVALUESPUBLISHERREMAPPER, "yarp.device.rawvaluespublisherremapper")
}

// Private methods

bool RawValuesPublisherRemapper::open(yarp::os::Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString());

    m_verbose = (prop.check("verbose","if present, give detailed output"));
    if (m_verbose)
    {
        yCInfo(RAWVALUESPUBLISHERREMAPPER, "Running with verbose output\n");
    }

    if(!parseParams(prop))
    {
        yCError(RAWVALUESPUBLISHERREMAPPER) << "Error parsing configuration parameters";
        return false;
    }

    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "RawValuesPublisherRemapper device started";
    for (const auto& name : m_axesNames)
    {
        //TODO: debug print to be removed once the remapper will be fully implemented
        yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Axes Name: " << name;
    }
    return true;
}

bool RawValuesPublisherRemapper::close()
{
    return detachAll();
}

bool RawValuesPublisherRemapper::attachAll(const yarp::dev::PolyDriverList& drivers)
{
    if (drivers.size() < 1)
    {
        yCError(RAWVALUESPUBLISHERREMAPPER) << "attachAll: cannot attach to less than one device";
        return false;
    }
    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Attaching to " << drivers.size() << " devices";
    m_remappedControlBoards.resize(drivers.size());
    for (size_t i = 0; i < drivers.size(); i++)
    {
        yarp::dev::PolyDriver* poly = drivers[i]->poly;
        if (!poly)
        {
            yCError(RAWVALUESPUBLISHERREMAPPER) << "NullPointerException when getting the polyDriver at attachAll.";
            detachAll();
            return false;
        }

        yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Attaching to device " << drivers[i]->key.c_str();

        // View all the interfaces
        if (!poly->view(m_remappedControlBoards[i]))
        {
            yCError(RAWVALUESPUBLISHERREMAPPER) << "Failure in viewing raw values publisher interface";
            detachAll();
            return false;
        }
    }

    return true;
}

bool RawValuesPublisherRemapper::detachAll()
{
    m_remappedControlBoards.resize(0);
    return true;
}

bool RawValuesPublisherRemapper::getRawDataMap(std::map<std::string, std::vector<std::int32_t>>& map)
{
    return true;
}
bool RawValuesPublisherRemapper::getRawData(std::string key, std::vector<std::int32_t>& data)
{
    return true;
}
bool RawValuesPublisherRemapper::getKeys(std::vector<std::string>& keys)
{
    return true;
}

int RawValuesPublisherRemapper::getNumberOfKeys()
{
    return 0;
}

bool RawValuesPublisherRemapper::getMetadataMap(rawValuesKeyMetadataMap& metamap)
{
    return true;
}

bool RawValuesPublisherRemapper::getKeyMetadata(std::string key, rawValuesKeyMetadata& meta)
{
    return true;
}

bool RawValuesPublisherRemapper::getAxesNames(std::string key, std::vector<std::string>& axesNames)
{
    return true;
}
