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
    if(!parseParams(config))
    {
        return false;
    }

    yCDebug(RAWVALUESPUBLISHERREMAPPER) << "RawValuesPublisherRemapper device started";
    for (const auto& name : m_axesNames)
    {
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
