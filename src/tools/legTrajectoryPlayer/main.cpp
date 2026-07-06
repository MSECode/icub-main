// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Copyright (C) 2026 Istituto Italiano di Tecnologia
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <hdf5.h>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr double radToDeg = 57.2957795130823208768;

const char* defaultPositionBase = "/robot_logger_device/walking/joints_state/positions/desired";
const char* defaultVelocityBase = "/robot_logger_device/walking/joints_state/velocities/measured";

struct H5Handle
{
    hid_t id{-1};
    herr_t (*closer)(hid_t){nullptr};
    H5Handle() = default;
    H5Handle(hid_t h, herr_t (*closeFn)(hid_t)) : id(h), closer(closeFn) {}
    H5Handle(const H5Handle&) = delete;
    H5Handle& operator=(const H5Handle&) = delete;
    ~H5Handle() { close(); }

    void close()
    {
        if (id >= 0) {
            closer(id);
            id = -1;
        }
    }

    operator hid_t() const { return id; }
};

size_t elementCount(const std::vector<hsize_t>& dims)
{
    size_t count = 1;
    for (size_t i = 0; i < dims.size(); ++i) {
        count *= static_cast<size_t>(dims[i]);
    }
    return count;
}

std::string readCharDataset(hid_t dataset)
{
    H5Handle space(H5Dget_space(dataset), H5Sclose);
    if (space.id < 0) {
        return {};
    }

    const int rank = H5Sget_simple_extent_ndims(space);
    std::vector<hsize_t> dims(static_cast<size_t>(rank));
    H5Sget_simple_extent_dims(space, dims.data(), nullptr);

    std::vector<unsigned short> utf16(elementCount(dims));
    if (H5Dread(dataset, H5T_NATIVE_USHORT, H5S_ALL, H5S_ALL, H5P_DEFAULT, utf16.data()) < 0) {
        return {};
    }

    std::string out;
    for (size_t i = 0; i < utf16.size(); ++i) {
        if (utf16[i] != 0) {
            out.push_back(utf16[i] < 128 ? static_cast<char>(utf16[i]) : '?');
        }
    }
    return out;
}

std::vector<std::string> readCellStrings(hid_t file, const std::string& path)
{
    std::vector<std::string> strings;
    H5Handle dataset(H5Dopen2(file, path.c_str(), H5P_DEFAULT), H5Dclose);
    if (dataset.id < 0) {
        yError() << "Unable to open string-cell dataset" << path;
        return strings;
    }

    H5Handle space(H5Dget_space(dataset), H5Sclose);
    const int rank = H5Sget_simple_extent_ndims(space);
    std::vector<hsize_t> dims(static_cast<size_t>(rank));
    H5Sget_simple_extent_dims(space, dims.data(), nullptr);

    std::vector<hobj_ref_t> refs(elementCount(dims));
    if (H5Dread(dataset, H5T_STD_REF_OBJ, H5S_ALL, H5S_ALL, H5P_DEFAULT, refs.data()) < 0) {
        yError() << "Unable to read references from" << path;
        return strings;
    }

    for (size_t i = 0; i < refs.size(); ++i) {
        H5Handle referenced(H5Rdereference2(dataset, H5P_DEFAULT, H5R_OBJECT, &refs[i]), H5Oclose);
        strings.push_back(referenced.id >= 0 ? readCharDataset(referenced) : std::string{});
    }
    return strings;
}

bool readDoubleDataset(hid_t file, const std::string& path, std::vector<double>& data, std::vector<hsize_t>& dims)
{
    H5Handle dataset(H5Dopen2(file, path.c_str(), H5P_DEFAULT), H5Dclose);
    if (dataset.id < 0) {
        yError() << "Unable to open double dataset" << path;
        return false;
    }

    H5Handle space(H5Dget_space(dataset), H5Sclose);
    const int rank = H5Sget_simple_extent_ndims(space);
    dims.resize(static_cast<size_t>(rank));
    H5Sget_simple_extent_dims(space, dims.data(), nullptr);

    data.resize(elementCount(dims));
    if (H5Dread(dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data()) < 0) {
        yError() << "Unable to read" << path;
        return false;
    }
    return true;
}

int dataIndex(const std::vector<hsize_t>& dims, int sample, int channel)
{
    return static_cast<int>((static_cast<size_t>(sample) * static_cast<size_t>(dims[1]) *
                            static_cast<size_t>(dims[2])) + static_cast<size_t>(channel));
}

std::string prefixedJointName(const std::string& side, int axis)
{
    static const char* localNames[] = {
        "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"
    };
    return side + "_" + localNames[axis];
}

struct Trajectory
{
    std::vector<double> time;
    std::vector<std::string> jointNames;
    std::vector<std::vector<double>> positionsDeg;
    std::vector<std::vector<double>> velocitiesDeg;

    bool empty() const { return time.empty() || jointNames.empty(); }

    double endTime() const
    {
        return time.empty() ? 0.0 : time.back();
    }

    int jointIndex(const std::string& name) const
    {
        for (size_t i = 0; i < jointNames.size(); ++i) {
            if (jointNames[i] == name) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    void interpolate(double t, std::vector<double>& pos, std::vector<double>& vel) const
    {
        pos.assign(jointNames.size(), 0.0);
        vel.assign(jointNames.size(), 0.0);
        if (empty()) {
            return;
        }
        if (t <= time.front()) {
            pos = positionsDeg.front();
            vel = velocitiesDeg.front();
            return;
        }
        if (t >= time.back()) {
            pos = positionsDeg.back();
            vel = velocitiesDeg.back();
            return;
        }

        std::vector<double>::const_iterator upper = std::upper_bound(time.begin(), time.end(), t);
        const size_t i1 = static_cast<size_t>(upper - time.begin());
        const size_t i0 = i1 - 1;
        const double denom = time[i1] - time[i0];
        const double alpha = denom > 0.0 ? (t - time[i0]) / denom : 0.0;
        for (size_t j = 0; j < jointNames.size(); ++j) {
            pos[j] = positionsDeg[i0][j] + alpha * (positionsDeg[i1][j] - positionsDeg[i0][j]);
            vel[j] = velocitiesDeg[i0][j] + alpha * (velocitiesDeg[i1][j] - velocitiesDeg[i0][j]);
        }
    }
};

bool loadTrajectory(const std::string& matPath,
                    const std::string& positionBase,
                    const std::string& velocityBase,
                    bool inputIsRadians,
                    Trajectory& trajectory)
{
    H5Handle file(H5Fopen(matPath.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (file.id < 0) {
        yError() << "Unable to open MAT/HDF5 file" << matPath;
        return false;
    }

    const std::vector<std::string> allNames = readCellStrings(file, positionBase + "/elements_names");
    const std::vector<std::string> velocityNames = readCellStrings(file, velocityBase + "/elements_names");
    if (allNames.empty() || velocityNames.empty()) {
        return false;
    }

    std::vector<double> posData;
    std::vector<hsize_t> posDims;
    std::vector<double> velData;
    std::vector<hsize_t> velDims;
    std::vector<double> stamps;
    std::vector<hsize_t> stampDims;
    if (!readDoubleDataset(file, positionBase + "/data", posData, posDims) ||
        !readDoubleDataset(file, velocityBase + "/data", velData, velDims) ||
        !readDoubleDataset(file, positionBase + "/timestamps", stamps, stampDims)) {
        return false;
    }

    if (posDims.size() != 3 || velDims.size() != 3 || posDims[0] == 0 || posDims[2] != allNames.size()) {
        yError() << "Unexpected trajectory dimensions in MAT file";
        return false;
    }

    std::map<std::string, int> posChannel;
    std::map<std::string, int> velChannel;
    for (size_t i = 0; i < allNames.size(); ++i) {
        posChannel[allNames[i]] = static_cast<int>(i);
    }
    for (size_t i = 0; i < velocityNames.size(); ++i) {
        velChannel[velocityNames[i]] = static_cast<int>(i);
    }

    std::vector<int> selectedPosChannels;
    std::vector<int> selectedVelChannels;
    trajectory.jointNames.clear();
    for (int sideIndex = 0; sideIndex < 2; ++sideIndex) {
        const std::string side = sideIndex == 0 ? "l" : "r";
        for (int axis = 0; axis < 6; ++axis) {
            const std::string name = prefixedJointName(side, axis);
            if (posChannel.count(name) == 0 || velChannel.count(name) == 0) {
                yError() << "Missing leg joint channel" << name;
                return false;
            }
            trajectory.jointNames.push_back(name);
            selectedPosChannels.push_back(posChannel[name]);
            selectedVelChannels.push_back(velChannel[name]);
        }
    }

    const size_t samples = static_cast<size_t>(posDims[0]);
    const size_t joints = trajectory.jointNames.size();
    trajectory.time.resize(samples);
    trajectory.positionsDeg.assign(samples, std::vector<double>(joints, 0.0));
    trajectory.velocitiesDeg.assign(samples, std::vector<double>(joints, 0.0));

    const double t0 = stamps.empty() ? 0.0 : stamps.front();
    for (size_t sample = 0; sample < samples; ++sample) {
        trajectory.time[sample] = sample < stamps.size() ? stamps[sample] - t0 : static_cast<double>(sample) * 0.01;
        for (size_t joint = 0; joint < joints; ++joint) {
            const double scale = inputIsRadians ? radToDeg : 1.0;
            trajectory.positionsDeg[sample][joint] =
                posData[dataIndex(posDims, static_cast<int>(sample), selectedPosChannels[joint])] * scale;
            trajectory.velocitiesDeg[sample][joint] =
                velData[dataIndex(velDims, static_cast<int>(sample), selectedVelChannels[joint])] * scale;
        }
    }

    yInfo() << "Loaded" << samples << "samples for" << joints << "leg joints from" << matPath;
    yInfo() << "Trajectory duration:" << trajectory.endTime() << "s";
    return true;
}

bool dumpCsv(const Trajectory& trajectory, const std::string& path)
{
    std::ofstream out(path.c_str());
    if (!out.is_open()) {
        yError() << "Unable to write CSV dump" << path;
        return false;
    }

    out << "time";
    for (size_t i = 0; i < trajectory.jointNames.size(); ++i) {
        out << "," << trajectory.jointNames[i] << "_pos_deg";
    }
    for (size_t i = 0; i < trajectory.jointNames.size(); ++i) {
        out << "," << trajectory.jointNames[i] << "_vel_deg_s";
    }
    out << "\n";

    for (size_t sample = 0; sample < trajectory.time.size(); ++sample) {
        out << trajectory.time[sample];
        for (size_t j = 0; j < trajectory.jointNames.size(); ++j) {
            out << "," << trajectory.positionsDeg[sample][j];
        }
        for (size_t j = 0; j < trajectory.jointNames.size(); ++j) {
            out << "," << trajectory.velocitiesDeg[sample][j];
        }
        out << "\n";
    }
    yInfo() << "Dumped extracted leg trajectory to" << path;
    return true;
}

std::vector<int> bottleToAxisList(const yarp::os::Bottle* bottle)
{
    std::vector<int> axes;
    if (bottle == nullptr) {
        return axes;
    }
    for (int i = 0; i < bottle->size(); ++i) {
        const int axis = bottle->get(i).asInt32();
        if (axis >= 0 && axis < 6 && std::find(axes.begin(), axes.end(), axis) == axes.end()) {
            axes.push_back(axis);
        }
    }
    return axes;
}

std::vector<int> defaultAxes()
{
    std::vector<int> axes;
    for (int i = 0; i < 6; ++i) {
        axes.push_back(i);
    }
    return axes;
}

std::vector<int> firstAxes(int count)
{
    std::vector<int> axes;
    for (int i = 0; i < std::max(0, std::min(6, count)); ++i) {
        axes.push_back(i);
    }
    return axes;
}

bool bottleHasString(const yarp::os::Bottle* bottle, const std::string& value)
{
    if (bottle == nullptr) {
        return false;
    }
    for (int i = 0; i < bottle->size(); ++i) {
        if (bottle->get(i).asString() == value) {
            return true;
        }
    }
    return false;
}

enum class PlaybackMode
{
    PositionDirect,
    Position,
    Velocity
};

PlaybackMode parseMode(const std::string& mode)
{
    if (mode == "position") {
        return PlaybackMode::Position;
    }
    if (mode == "velocity") {
        return PlaybackMode::Velocity;
    }
    return PlaybackMode::PositionDirect;
}

struct Board
{
    std::string sidePrefix;
    std::string partName;
    std::vector<int> axes;
    std::vector<int> trajectoryIndices;
    yarp::dev::PolyDriver driver;
    yarp::dev::IPositionDirect* positionDirect{nullptr};
    yarp::dev::IPositionControl* positionControl{nullptr};
    yarp::dev::IVelocityControl* velocityControl{nullptr};
    yarp::dev::IControlMode* controlMode{nullptr};
    yarp::dev::IControlLimits* limits{nullptr};
    std::vector<double> minLimits;
    std::vector<double> maxLimits;

    bool open(const std::string& robot, const std::string& moduleName, PlaybackMode mode)
    {
        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", "/" + robot + "/" + partName);
        options.put("local", "/" + moduleName + "/" + partName + "/client");
        options.put("carrier", "tcp");

        if (!driver.open(options)) {
            yError() << "Unable to open remote_controlboard for" << partName;
            return false;
        }

        bool ok = true;
        ok &= driver.view(positionControl);
        ok &= driver.view(controlMode);
        ok &= driver.view(limits);
        if (mode == PlaybackMode::PositionDirect) {
            ok &= driver.view(positionDirect);
        }
        if (mode == PlaybackMode::Velocity) {
            ok &= driver.view(velocityControl);
        }
        if (!ok || positionControl == nullptr || controlMode == nullptr) {
            yError() << "Missing required YARP control interfaces for" << partName;
            return false;
        }

        int axesInPart = 0;
        positionControl->getAxes(&axesInPart);
        if (axesInPart < 6) {
            yError() << partName << "has only" << axesInPart << "axes, expected at least 6";
            return false;
        }

        minLimits.assign(axes.size(), -std::numeric_limits<double>::infinity());
        maxLimits.assign(axes.size(), std::numeric_limits<double>::infinity());
        for (size_t i = 0; i < axes.size(); ++i) {
            if (limits != nullptr) {
                double min = 0.0;
                double max = 0.0;
                if (limits->getLimits(axes[i], &min, &max)) {
                    yDebug() << "Axis" << axes[i] << "limits for" << partName << ":" << min << "to" << max;
                    minLimits[i] = min;
                    maxLimits[i] = max;
                }
            }
        }
        return setMode(mode);
    }

    bool setMode(PlaybackMode mode)
    {
        int vocab = VOCAB_CM_POSITION_DIRECT;
        if (mode == PlaybackMode::Position) {
            vocab = VOCAB_CM_POSITION;
        } else if (mode == PlaybackMode::Velocity) {
            vocab = VOCAB_CM_VELOCITY;
        }
        bool ok = true;
        for (size_t i = 0; i < axes.size(); ++i) {
            ok &= controlMode->setControlMode(axes[i], vocab);
        }
        return ok;
    }

    void restorePositionMode()
    {
        if (controlMode == nullptr) {
            return;
        }
        for (size_t i = 0; i < axes.size(); ++i) {
            controlMode->setControlMode(axes[i], VOCAB_CM_POSITION);
        }
    }

    bool send(const std::vector<double>& pos, const std::vector<double>& vel, PlaybackMode mode)
    {
        std::vector<double> positionRefs(axes.size(), 0.0);
        std::vector<double> velocityRefs(axes.size(), 0.0);
        for (size_t i = 0; i < axes.size(); ++i) {
            const int trajectoryIndex = trajectoryIndices[i];
            positionRefs[i] = std::max(minLimits[i], std::min(maxLimits[i], pos[trajectoryIndex]));
            velocityRefs[i] = std::max(1.0, std::fabs(vel[trajectoryIndex]));
        }

        if (mode == PlaybackMode::PositionDirect) {
            return positionDirect->setPositions(static_cast<int>(axes.size()), axes.data(), positionRefs.data());
        }
        if (mode == PlaybackMode::Velocity) {
            std::vector<double> signedVelocityRefs(axes.size(), 0.0);
            for (size_t i = 0; i < axes.size(); ++i) {
                signedVelocityRefs[i] = vel[trajectoryIndices[i]];
            }
            return velocityControl->velocityMove(static_cast<int>(axes.size()), axes.data(), signedVelocityRefs.data());
        }

        bool ok = positionControl->setRefSpeeds(static_cast<int>(axes.size()), axes.data(), velocityRefs.data());
        ok &= positionControl->positionMove(static_cast<int>(axes.size()), axes.data(), positionRefs.data());
        return ok;
    }

    void stop(PlaybackMode mode)
    {
        if (mode == PlaybackMode::Velocity && velocityControl != nullptr) {
            for (size_t i = 0; i < axes.size(); ++i) {
                velocityControl->stop(axes[i]);
            }
        } else if (positionControl != nullptr) {
            for (size_t i = 0; i < axes.size(); ++i) {
                positionControl->stop(axes[i]);
            }
        }
    }

    void close()
    {
        restorePositionMode();
        driver.close();
    }
};

} // namespace

class LegTrajectoryPlayerModule : public yarp::os::RFModule
{
public:
    bool configure(yarp::os::ResourceFinder& rf) override
    {
        if (rf.check("help")) {
            printHelp();
            return false;
        }

        m_moduleName = rf.check("name", yarp::os::Value("legTrajectoryPlayer")).asString();
        m_robotName = rf.check("robot", yarp::os::Value("icub")).asString();
        m_matPath = rf.check("mat", yarp::os::Value("")).asString();
        m_period = rf.check("period", yarp::os::Value(0.01)).asFloat64();
        m_timeScale = rf.check("timeScale", yarp::os::Value(1.0)).asFloat64();
        m_duration = rf.check("duration", yarp::os::Value(-1.0)).asFloat64();
        m_dryRun = rf.check("dryRun");
        m_mode = parseMode(rf.check("controlMode", yarp::os::Value("positionDirect")).asString());

        const std::string positionBase = rf.check("positionBase", yarp::os::Value(defaultPositionBase)).asString();
        const std::string velocityBase = rf.check("velocityBase", yarp::os::Value(defaultVelocityBase)).asString();
        const std::string dataUnits = rf.check("dataUnits", yarp::os::Value("radians")).asString();

        if (m_matPath.empty()) {
            yError() << "Missing --mat <walking-data.mat>";
            printHelp();
            return false;
        }
        if (m_timeScale <= 0.0 || m_period <= 0.0) {
            yError() << "--timeScale and --period must be positive";
            return false;
        }

        if (!loadTrajectory(m_matPath, positionBase, velocityBase, dataUnits != "degrees", m_trajectory)) {
            return false;
        }

        if (rf.check("dump")) {
            if (!dumpCsv(m_trajectory, rf.find("dump").asString())) {
                return false;
            }
        }

        if (!configureBoards(rf)) {
            return false;
        }

        if (!m_dryRun && m_boards.empty()) {
            yError() << "No joints selected";
            return false;
        }

        m_rpcPort.open("/" + m_moduleName + "/rpc");
        attach(m_rpcPort);
        m_startTime = yarp::os::Time::now();
        yInfo() << "Starting trajectory playback with period" << m_period << "s";
        return true;
    }

    double getPeriod() override
    {
        return m_period;
    }

    bool updateModule() override
    {
        if (m_stopping) {
            return false;
        }

        const double elapsed = yarp::os::Time::now() - m_startTime;
        const double trajectoryTime = elapsed / m_timeScale;
        if ((m_duration > 0.0 && elapsed >= m_duration) || trajectoryTime >= m_trajectory.endTime()) {
            yInfo() << "Requested trajectory interval completed";
            stopBoards();
            return false;
        }

        std::vector<double> positions;
        std::vector<double> velocities;
        m_trajectory.interpolate(trajectoryTime, positions, velocities);

        if (m_dryRun) {
            if (m_iteration % 100 == 0) {
                yInfo() << "dryRun t=" << trajectoryTime << "s";
                for (auto &&pos : positions)
                {                   
                    yInfo() << "position:" << pos << "deg";   
                }
            }
        } else {
            for (size_t i = 0; i < m_boards.size(); ++i) {
                if (!m_boards[i].send(positions, velocities, m_mode)) {
                    yWarning() << "Failed to send command to" << m_boards[i].partName;
                }
            }
        }
        ++m_iteration;
        return true;
    }

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) override
    {
        const std::string cmd = command.get(0).asString();
        if (cmd == "help") {
            reply.addString("commands: help, stop, quit");
            return true;
        }
        if (cmd == "stop" || cmd == "quit") {
            stopBoards();
            m_stopping = true;
            reply.addString("stopping");
            return true;
        }
        reply.addString("unknown command");
        return true;
    }

    bool close() override
    {
        stopBoards();
        for (size_t i = 0; i < m_boards.size(); ++i) {
            m_boards[i].close();
        }
        m_rpcPort.close();
        return true;
    }

private:
    bool configureBoards(yarp::os::ResourceFinder& rf)
    {
        const yarp::os::Bottle* legs = rf.find("legs").asList();
        const bool useLeft = legs == nullptr || bottleHasString(legs, "left");
        const bool useRight = legs == nullptr || bottleHasString(legs, "right");

        std::vector<int> commonAxes = defaultAxes();
        if (rf.check("numJoints")) {
            commonAxes = firstAxes(rf.find("numJoints").asInt32());
        }
        if (rf.check("joints")) {
            commonAxes = bottleToAxisList(rf.find("joints").asList());
        }

        std::vector<int> leftAxes = commonAxes;
        std::vector<int> rightAxes = commonAxes;
        if (rf.check("leftJoints")) {
            leftAxes = bottleToAxisList(rf.find("leftJoints").asList());
        }
        if (rf.check("rightJoints")) {
            rightAxes = bottleToAxisList(rf.find("rightJoints").asList());
        }

        if (useLeft && !leftAxes.empty()) {
            addBoard("l", "left_leg", leftAxes);
        }
        if (useRight && !rightAxes.empty()) {
            addBoard("r", "right_leg", rightAxes);
        }

        if (m_dryRun) {
            yInfo() << "dryRun enabled: not opening remote_controlboard devices";
            return true;
        }

        for (size_t i = 0; i < m_boards.size(); ++i) {
            if (!m_boards[i].open(m_robotName, m_moduleName, m_mode)) {
                return false;
            }
            yInfo() << "Configured" << m_boards[i].partName << "axes" << m_boards[i].axes.size();
        }
        return true;
    }

    void addBoard(const std::string& side, const std::string& part, const std::vector<int>& axes)
    {
        m_boards.emplace_back();
        Board& board = m_boards.back();
        board.sidePrefix = side;
        board.partName = part;
        board.axes = axes;
        for (size_t i = 0; i < axes.size(); ++i) {
            const std::string name = prefixedJointName(side, axes[i]);
            const int index = m_trajectory.jointIndex(name);
            if (index >= 0) {
                board.trajectoryIndices.push_back(index);
            }
        }
        if (board.axes.size() == board.trajectoryIndices.size()) {
            return;
        } else {
            yError() << "Could not map all axes for" << part;
            m_boards.pop_back();
        }
    }

    void stopBoards()
    {
        if (m_dryRun) {
            return;
        }
        for (size_t i = 0; i < m_boards.size(); ++i) {
            m_boards[i].stop(m_mode);
        }
    }

    void printHelp() const
    {
        yInfo() << "legTrajectoryPlayer options:";
        yInfo() << "--mat <file.mat>                         MATLAB v7.3 walking logger file";
        yInfo() << "--robot <name>                           robot name, default icub";
        yInfo() << "--name <moduleName>                      default legTrajectoryPlayer";
        yInfo() << "--legs \"(left right)\"                    choose boards, default both";
        yInfo() << "--joints \"(0 1 2 3 4 5)\"                local leg axes for selected legs";
        yInfo() << "--leftJoints \"(...)\" --rightJoints \"(...)\" per-leg axis subsets";
        yInfo() << "--numJoints <n>                          shortcut for axes 0..n-1";
        yInfo() << "--duration <seconds>                     wall-clock playback duration";
        yInfo() << "--timeScale <factor>                     >1 slower, <1 faster";
        yInfo() << "--controlMode positionDirect|position|velocity";
        yInfo() << "--positionBase <hdf5-path>               default walking desired positions";
        yInfo() << "--velocityBase <hdf5-path>               default walking measured velocities";
        yInfo() << "--dataUnits radians|degrees              default radians, converted to YARP degrees";
        yInfo() << "--dump <file.csv>                        dump extracted leg position/velocity data";
        yInfo() << "--dryRun                                 load and sample without moving the robot";
    }

    std::string m_moduleName;
    std::string m_robotName;
    std::string m_matPath;
    Trajectory m_trajectory;
    std::deque<Board> m_boards;
    yarp::os::RpcServer m_rpcPort;
    PlaybackMode m_mode{PlaybackMode::PositionDirect};
    double m_period{0.01};
    double m_timeScale{1.0};
    double m_duration{-1.0};
    double m_startTime{0.0};
    bool m_dryRun{false};
    bool m_stopping{false};
    size_t m_iteration{0};
};

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP server is not available";
        return EXIT_FAILURE;
    }

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    LegTrajectoryPlayerModule module;
    return module.runModule(rf) ? EXIT_SUCCESS : EXIT_FAILURE;
}
