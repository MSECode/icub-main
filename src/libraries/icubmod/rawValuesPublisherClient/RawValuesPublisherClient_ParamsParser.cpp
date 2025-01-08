/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Wed Jan  8 10:37:54 2025


#include "RawValuesPublisherClient_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(RawValuesPublisherClientParamsCOMPONENT, "yarp.device.RawValuesPublisherClient")
}


RawValuesPublisherClient_ParamsParser::RawValuesPublisherClient_ParamsParser()
{
}


std::vector<std::string> RawValuesPublisherClient_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("remote");
    params.push_back("local");
    params.push_back("externalConnection");
    params.push_back("carrier");
    return params;
}


bool      RawValuesPublisherClient_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(RawValuesPublisherClientParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter remote
    {
        if (config.check("remote"))
        {
            m_remote = config.find("remote").asString();
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'remote' using value:" << m_remote;
        }
        else
        {
            yCError(RawValuesPublisherClientParamsCOMPONENT) << "Mandatory parameter 'remote' not found!";
            yCError(RawValuesPublisherClientParamsCOMPONENT) << "Description of the parameter: Prefix of the ports to which to connect, opened by RawValuesParameterSensorsServer device.";
            return false;
        }
        prop_check.unput("remote");
    }

    //Parser of parameter local
    {
        if (config.check("local"))
        {
            m_local = config.find("local").asString();
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'local' using value:" << m_local;
        }
        else
        {
            yCError(RawValuesPublisherClientParamsCOMPONENT) << "Mandatory parameter 'local' not found!";
            yCError(RawValuesPublisherClientParamsCOMPONENT) << "Description of the parameter: Port prefix of the ports opened by this device.";
            return false;
        }
        prop_check.unput("local");
    }

    //Parser of parameter externalConnection
    {
        if (config.check("externalConnection"))
        {
            m_externalConnection = config.find("externalConnection").asBool();
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'externalConnection' using value:" << m_externalConnection;
        }
        else
        {
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'externalConnection' using DEFAULT value:" << m_externalConnection;
        }
        prop_check.unput("externalConnection");
    }

    //Parser of parameter carrier
    {
        if (config.check("carrier"))
        {
            m_carrier = config.find("carrier").asString();
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'carrier' using value:" << m_carrier;
        }
        else
        {
            yCInfo(RawValuesPublisherClientParamsCOMPONENT) << "Parameter 'carrier' using DEFAULT value:" << m_carrier;
        }
        prop_check.unput("carrier");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(RawValuesPublisherClientParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(RawValuesPublisherClientParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      RawValuesPublisherClient_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: RawValuesPublisherClient\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'remote': Prefix of the ports to which to connect, opened by RawValuesParameterSensorsServer device.\n");
    doc = doc + std::string("'local': Port prefix of the ports opened by this device.\n");
    doc = doc + std::string("'externalConnection': If set to true, the connection to the rpc port of the RVP server is skipped and it is possible to connect to the data source externally after being opened\n");
    doc = doc + std::string("'carrier': The carier used for the connection with the server.\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device rawValuesPublisherClient --remote <mandatory_value> --local <mandatory_value> --externalConnection false --carrier tcp\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device rawValuesPublisherClient --remote <mandatory_value> --local <mandatory_value>\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}