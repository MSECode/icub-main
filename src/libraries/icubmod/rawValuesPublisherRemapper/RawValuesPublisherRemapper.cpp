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
        iCub::debugLibrary::IRawValuesPublisher* rawValuesPublisher = nullptr;
        if (!poly->view(rawValuesPublisher))
        {
            yCError(RAWVALUESPUBLISHERREMAPPER) << "Failure in viewing raw values publisher interface";
            detachAll();
            return false;
        }
        else
        {
            yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Successfully viewed raw values publisher interface";
            std::vector<std::string> keys;
            rawValuesPublisher->getKeys(keys);
            for (const auto& key : keys)
            {
                yCDebug(RAWVALUESPUBLISHERREMAPPER) << "Key: " << key;
            }
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
    map.clear();
    bool allOk = true;
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     std::map<std::string, std::vector<std::int32_t>> temp_map;
    //     if (!m_remappedControlBoards[i]->getRawDataMap(temp_map))
    //     {
    //         yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get raw data map from control board " << i;
    //         allOk = false;
    //         continue;
    //     }
    //     map.insert(temp_map.begin(), temp_map.end());
    // }
    return allOk;
}

bool RawValuesPublisherRemapper::getRawData(std::string key, std::vector<std::int32_t>& data)
{
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     if (m_remappedControlBoards[i]->getRawData(key, data))
    //     {
    //         return true;
    //     }
    // }
    yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Key not found: " << key;
    return false;
}

bool RawValuesPublisherRemapper::getKeys(std::vector<std::string>& keys)
{
    keys.clear();
    bool allOk = true;
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     std::vector<std::string> temp_keys;
    //     if (!m_remappedControlBoards[i]->getKeys(temp_keys))
    //     {
    //         yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get keys from control board " << i;
    //         allOk = false;
    //         continue;
    //     }
    //     keys.insert(keys.end(), temp_keys.begin(), temp_keys.end());
    // }
    return allOk;
}

int RawValuesPublisherRemapper::getNumberOfKeys()
{
    int total_keys = 0;
    bool allOk = true;
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     int n = m_remappedControlBoards[i]->getNumberOfKeys();
    //     if (n < 0)
    //     {
    //         yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get number of keys from control board " << i;
    //         allOk = false;
    //         continue;
    //     }
    //     total_keys += n;
    // }
    return allOk ? total_keys : -1;
}

bool RawValuesPublisherRemapper::getMetadataMap(rawValuesKeyMetadataMap& metamap)
{
    bool allOk = true;
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     if (!m_remappedControlBoards[i]->getMetadataMap(metamap))
    //     {
    //         yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Failed to get metadata map from control board " << i;
    //         allOk = false;
    //         continue;
    //     }
    // }
    return allOk;
}

bool RawValuesPublisherRemapper::getKeyMetadata(std::string key, rawValuesKeyMetadata& meta)
{
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     if (m_remappedControlBoards[i]->getKeyMetadata(key, meta))
    //     {
    //         return true;
    //     }
    // }
    yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Metadata not found for key: " << key;
    return false;
}

bool RawValuesPublisherRemapper::getAxesNames(std::string key, std::vector<std::string>& axesNames)
{
    // for (size_t i = 0; i < m_remappedControlBoards.size(); i++)
    // {
    //     if (m_remappedControlBoards[i]->getAxesNames(key, axesNames))
    //     {
    //         return true;
    //     }
    // }
    yCWarning(RAWVALUESPUBLISHERREMAPPER) << "Axes names not found for key: " << key;
    return false;
}
