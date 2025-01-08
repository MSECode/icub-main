/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Wed Jan  8 10:37:54 2025


#include "FakeRawValuesPublisher_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(FakeRawValuesPublisherParamsCOMPONENT, "yarp.device.FakeRawValuesPublisher")
}


FakeRawValuesPublisher_ParamsParser::FakeRawValuesPublisher_ParamsParser()
{
}


std::vector<std::string> FakeRawValuesPublisher_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("name");
    params.push_back("njomos");
    params.push_back("threshold");
    return params;
}


bool      FakeRawValuesPublisher_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(FakeRawValuesPublisherParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter name
    {
        if (config.check("name"))
        {
            m_name = config.find("name").asString();
            yCInfo(FakeRawValuesPublisherParamsCOMPONENT) << "Parameter 'name' using value:" << m_name;
        }
        else
        {
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Mandatory parameter 'name' not found!";
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Description of the parameter: Name of the device";
            return false;
        }
        prop_check.unput("name");
    }

    //Parser of parameter njomos
    {
        if (config.check("njomos"))
        {
            m_njomos = config.find("njomos").asInt64();
            yCInfo(FakeRawValuesPublisherParamsCOMPONENT) << "Parameter 'njomos' using value:" << m_njomos;
        }
        else
        {
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Mandatory parameter 'njomos' not found!";
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Description of the parameter: Number of joint or motors to be instantiated";
            return false;
        }
        prop_check.unput("njomos");
    }

    //Parser of parameter threshold
    {
        if (config.check("threshold"))
        {
            m_threshold = config.find("threshold").asInt64();
            yCInfo(FakeRawValuesPublisherParamsCOMPONENT) << "Parameter 'threshold' using value:" << m_threshold;
        }
        else
        {
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Mandatory parameter 'threshold' not found!";
            yCError(FakeRawValuesPublisherParamsCOMPONENT) << "Description of the parameter: Threshold used for defining the amplitude of the sawthooth curve that simulates periodic raw encoder data";
            return false;
        }
        prop_check.unput("threshold");
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
                yCError(FakeRawValuesPublisherParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(FakeRawValuesPublisherParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
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


std::string      FakeRawValuesPublisher_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: FakeRawValuesPublisher\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'name': Name of the device\n");
    doc = doc + std::string("'njomos': Number of joint or motors to be instantiated\n");
    doc = doc + std::string("'threshold': Threshold used for defining the amplitude of the sawthooth curve that simulates periodic raw encoder data\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device fakeRawValuesPublisher --name <mandatory_value> --njomos <mandatory_value> --threshold <mandatory_value>\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device fakeRawValuesPublisher --name <mandatory_value> --njomos <mandatory_value> --threshold <mandatory_value>\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
