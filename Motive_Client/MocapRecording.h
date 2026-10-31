#pragma once

// Native, unfiltered NatNet recording. Only owned frame copies cross the callback
// boundary; JSON serialization, file I/O and control replies belong to the worker.
#include "NatNetCAPI.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <locale>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace mocap_recording
{
    namespace fs = std::filesystem;
    inline std::int64_t unixNanoseconds()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    inline void text(std::ostream& out, const std::string& value)
    {
        out << '"';
        for (unsigned char c : value)
        {
            if (c == '"' || c == '\\') out << '\\' << c;
            else if (c < 32) out << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                                  << int(c) << std::dec << std::setfill(' ');
            else out << c;
        }
        out << '"';
    }
    inline void number(std::ostream& out, double value)
    {
        if (std::isfinite(value)) out << value; else out << "null";
    }
    template<class Function> void array(std::ostream& out, int count, Function write)
    {
        out << '[';
        for (int i = 0; i < count; ++i) { if (i) out << ','; write(i); }
        out << ']';
    }
    inline void position(std::ostream& out, float x, float y, float z)
    {
        out << '['; number(out, x); out << ','; number(out, y); out << ','; number(out, z); out << ']';
    }
    template<class Pose> void pose(std::ostream& out, const Pose& p)
    {
        out << "\"position\":"; position(out, p.x, p.y, p.z);
        out << ",\"quaternion_xyzw\":[";
        number(out, p.qx); out << ','; number(out, p.qy); out << ',';
        number(out, p.qz); out << ','; number(out, p.qw); out << ']';
    }
    inline void rigidBody(std::ostream& out, const sRigidBodyData& p)
    {
        out << "{\"id\":" << p.ID << ','; pose(out, p);
        out << ",\"mean_error\":"; number(out, p.MeanError);
        out << ",\"params\":" << p.params << '}';
    }
    inline void marker(std::ostream& out, const sMarker& p)
    {
        out << "{\"id\":" << p.ID << ",\"position\":"; position(out, p.x, p.y, p.z);
        out << ",\"size\":"; number(out, p.size);
        out << ",\"residual\":"; number(out, p.residual);
        out << ",\"params\":" << p.params << '}';
    }
    template<class Device> void analog(std::ostream& out, const Device& d)
    {
        out << "{\"id\":" << d.ID << ",\"params\":" << d.params << ",\"channels\":";
        array(out, d.nChannels, [&](int c) {
            array(out, d.ChannelData[c].nFrames, [&](int f) { number(out, d.ChannelData[c].Values[f]); });
        });
        out << '}';
    }
    struct Frame
    {
        std::unique_ptr<sFrameOfMocapData> data = std::make_unique<sFrameOfMocapData>();
        std::int64_t receiveUnixNs = 0;
        std::int64_t receiveSteadyNs = 0;
        int picoID = -1;
        ~Frame() { NatNet_FreeFrame(data.get()); }
    };
    inline void writeFrame(std::ostream& out, const Frame& owned)
    {
        const auto& f = *owned.data;
        out << std::setprecision(17) << "{\"frame\":" << f.iFrame
            << ",\"receive_unix_ns\":" << owned.receiveUnixNs
            << ",\"receive_steady_ns\":" << owned.receiveSteadyNs
            << ",\"pico_id\":" << owned.picoID << ",\"timestamp\":";
        number(out, f.fTimestamp);
        out << ",\"camera_mid_exposure_ticks\":" << f.CameraMidExposureTimestamp
            << ",\"camera_data_received_ticks\":" << f.CameraDataReceivedTimestamp
            << ",\"transmit_ticks\":" << f.TransmitTimestamp
            << ",\"precision_timestamp_secs\":" << f.PrecisionTimestampSecs
            << ",\"precision_timestamp_fractional_secs\":" << f.PrecisionTimestampFractionalSecs
            << ",\"timecode\":" << f.Timecode << ",\"timecode_subframe\":" << f.TimecodeSubframe
            << ",\"params\":" << f.params << ",\"marker_sets\":";
        array(out, f.nMarkerSets, [&](int i) {
            const auto& m = f.MocapData[i]; out << "{\"name\":"; text(out, m.szName);
            out << ",\"positions\":";
            array(out, m.nMarkers, [&](int j) { position(out, m.Markers[j][0], m.Markers[j][1], m.Markers[j][2]); });
            out << '}';
        });
        out << ",\"other_markers\":";
        array(out, f.nOtherMarkers, [&](int i) { position(out, f.OtherMarkers[i][0], f.OtherMarkers[i][1], f.OtherMarkers[i][2]); });
        out << ",\"labeled_markers\":";
        array(out, f.nLabeledMarkers, [&](int i) { marker(out, f.LabeledMarkers[i]); });
        out << ",\"rigid_bodies\":";
        array(out, f.nRigidBodies, [&](int i) { rigidBody(out, f.RigidBodies[i]); });
        out << ",\"skeletons\":";
        array(out, f.nSkeletons, [&](int i) {
            const auto& s = f.Skeletons[i]; out << "{\"id\":" << s.skeletonID << ",\"rigid_bodies\":";
            array(out, s.nRigidBodies, [&](int j) { rigidBody(out, s.RigidBodyData[j]); }); out << '}';
        });
        out << ",\"assets\":";
        array(out, f.nAssets, [&](int i) {
            const auto& a = f.Assets[i]; out << "{\"id\":" << a.assetID << ",\"rigid_bodies\":";
            array(out, a.nRigidBodies, [&](int j) { rigidBody(out, a.RigidBodyData[j]); });
            out << ",\"markers\":";
            array(out, a.nMarkers, [&](int j) { marker(out, a.MarkerData[j]); }); out << '}';
        });
        out << ",\"force_plates\":";
        array(out, f.nForcePlates, [&](int i) { analog(out, f.ForcePlates[i]); });
        out << ",\"devices\":";
        array(out, f.nDevices, [&](int i) { analog(out, f.Devices[i]); });
        out << ",\"imu\":";
        array(out, f.nIMU, [&](int i) {
            const auto& p = f.IMU[i]; out << "{\"id\":" << p.ID << ','; pose(out, p);
            out << ",\"params\":" << p.params << '}';
        });
        out << ",\"gpio\":";
        array(out, f.nGPIO, [&](int i) {
            const auto& p = f.GPIO[i]; out << "{\"id\":" << p.ID << ",\"values\":";
            array(out, p.numberOfGPIOPorts, [&](int j) { out << p.GPIO[j]; }); out << '}';
        });
        out << "}\n";
    }
    inline void writePico(std::ostream& out, const Frame& owned)
    {
        const auto& f = *owned.data;
        const sRigidBodyData* p = nullptr;
        for (int i = 0; i < f.nRigidBodies; ++i)
            if (f.RigidBodies[i].ID == owned.picoID) { p = &f.RigidBodies[i]; break; }
        out << std::setprecision(17) << f.iFrame << ',' << owned.receiveUnixNs << ',';
        number(out, f.fTimestamp);
        out << ',' << owned.picoID << ',' << (p ? 1 : 0);
        if (p)
        {
            out << ',' << ((p->params & 1) ? 1 : 0) << ','
                << ((p->params & 1) && std::isfinite(p->MeanError) && p->MeanError >= 0 && p->MeanError <= 0.01f ? 1 : 0);
            for (float v : {p->MeanError, p->x, p->y, p->z, p->qx, p->qy, p->qz, p->qw})
            { out << ','; if (std::isfinite(v)) out << v; }
            out << ',' << p->params;
        }
        else out << ",0,0,,,,,,,,,"; // absent pose stays absent, never an invented identity pose
        out << '\n';
    }

    // Model names and calibrated marker layouts give the raw IDs meaning offline.
    inline std::string describeModels(const sDataDescriptions& descriptions)
    {
        std::ostringstream out; out.imbue(std::locale::classic()); out << std::setprecision(17);
        array(out, descriptions.nDataDescriptions, [&](int i) {
            const auto& d = descriptions.arrDataDescriptions[i];
            out << "{\"type\":" << d.type;
            auto named = [&](const char* name, int id) { out << ",\"name\":"; text(out, name); out << ",\"id\":" << id; };
            auto rb = [&](const sRigidBodyDescription& r) {
                named(r.szName, r.ID); out << ",\"parent_id\":" << r.parentID << ",\"offset\":";
                position(out, r.offsetx, r.offsety, r.offsetz);
                out << ",\"offset_quaternion_xyzw\":[" << r.offsetqx << ',' << r.offsetqy << ',' << r.offsetqz << ',' << r.offsetqw << ']';
                out << ",\"markers\":";
                array(out, r.nMarkers, [&](int j) {
                    out << "{\"name\":"; text(out, r.szMarkerNames && r.szMarkerNames[j] ? r.szMarkerNames[j] : "");
                    out << ",\"required_label\":" << (r.MarkerRequiredLabels ? r.MarkerRequiredLabels[j] : 0) << ",\"position\":";
                    if (r.MarkerPositions) position(out, r.MarkerPositions[j][0], r.MarkerPositions[j][1], r.MarkerPositions[j][2]); else out << "null";
                    out << '}';
                });
            };
            switch (d.type)
            {
                case Descriptor_RigidBody: rb(*d.Data.RigidBodyDescription); break;
                case Descriptor_MarkerSet: {
                    const auto& m = *d.Data.MarkerSetDescription; named(m.szName, -1); out << ",\"marker_names\":";
                    array(out, m.nMarkers, [&](int j) { text(out, m.szMarkerNames[j]); }); break;
                }
                case Descriptor_Skeleton: {
                    const auto& s = *d.Data.SkeletonDescription; named(s.szName, s.skeletonID); out << ",\"rigid_bodies\":";
                    array(out, s.nRigidBodies, [&](int j) { out << '{'; rb(s.RigidBodies[j]); out << '}'; }); break;
                }
                case Descriptor_Camera: {
                    const auto& c = *d.Data.CameraDescription; named(c.strName, -1); out << ','; pose(out, c); break;
                }
                case Descriptor_Asset: {
                    const auto& a = *d.Data.AssetDescription; named(a.szName, a.AssetID); out << ",\"asset_type\":" << a.AssetType << ",\"rigid_bodies\":";
                    array(out, a.nRigidBodies, [&](int j) { out << '{'; rb(a.RigidBodies[j]); out << '}'; });
                    out << ",\"markers\":";
                    array(out, a.nMarkers, [&](int j) {
                        const auto& m = a.Markers[j]; out << "{\"name\":"; text(out, m.szName);
                        out << ",\"id\":" << m.ID << ",\"position\":"; position(out, m.x, m.y, m.z);
                        out << ",\"size\":" << m.size << ",\"params\":" << m.params << '}';
                    }); break;
                }
                case Descriptor_IMU: {
                    const auto& p = *d.Data.IMUDescription; named(p.szName, p.ID);
                    out << ",\"sensor_fused\":" << (p.SensorFused ? "true" : "false") << ",\"rigid_body_id\":" << p.RigidBodyID; break;
                }
                case Descriptor_GPIO: {
                    const auto& p = *d.Data.GPIODescription; named(p.szName, p.ID); out << ",\"port_names\":";
                    array(out, p.numberOfGPIOPorts, [&](int j) { text(out, p.szGPIONames[j]); }); break;
                }
                case Descriptor_Anchor: {
                    const auto& a = *d.Data.AnchorDescription; named(a.szName, a.ActiveID); out << ",\"position\":"; position(out, a.x, a.y, a.z); break;
                }
                case Descriptor_Device: {
                    const auto& p = *d.Data.DeviceDescription; named(p.strName, p.ID);
                    out << ",\"serial\":"; text(out, p.strSerialNo);
                    out << ",\"device_type\":" << p.iDeviceType << ",\"channel_data_type\":" << p.iChannelDataType << ",\"channel_names\":";
                    array(out, p.nChannels, [&](int j) { text(out, p.szChannelNames[j]); }); break;
                }
                case Descriptor_ForcePlate: {
                    const auto& p = *d.Data.ForcePlateDescription; named(p.strSerialNo, p.ID);
                    out << ",\"width\":" << p.fWidth << ",\"length\":" << p.fLength << ",\"origin\":";
                    position(out, p.fOriginX, p.fOriginY, p.fOriginZ); out << ",\"calibration\":";
                    array(out, 12, [&](int j) { array(out, 12, [&](int k) { number(out, p.fCalMat[j][k]); }); });
                    out << ",\"corners\":"; array(out, 4, [&](int j) { position(out, p.fCorners[j][0], p.fCorners[j][1], p.fCorners[j][2]); });
                    out << ",\"plate_type\":" << p.iPlateType << ",\"channel_data_type\":" << p.iChannelDataType << ",\"channel_names\":";
                    array(out, p.nChannels, [&](int j) { text(out, p.szChannelNames[j]); }); break;
                }
            }
            out << '}';
        });
        return out.str();
    }

    class Recorder
    {
    public:
        ~Recorder() { shutdown(); }
        bool initialize(const fs::path& root, std::string metadata, unsigned short port = 5011, std::size_t queueLimit = 32,
                        std::function<std::string()> metadataProvider = {})
        {
            WSADATA startup{};
            if (WSAStartup(MAKEWORD(2, 2), &startup) != 0) return false;
            winsock_ = true;
            root_ = fs::absolute(root); metadata_ = std::move(metadata); queueLimit_ = queueLimit;
            metadataProvider_ = std::move(metadataProvider);
            socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (socket_ == INVALID_SOCKET) return false;
            sockaddr_in address{}; address.sin_family = AF_INET;
            address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(port);
            if (bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == SOCKET_ERROR)
            { closesocket(socket_); socket_ = INVALID_SOCKET; return false; }
            int size = sizeof(address); getsockname(socket_, reinterpret_cast<sockaddr*>(&address), &size);
            port_ = ntohs(address.sin_port);
            u_long nonblocking = 1; ioctlsocket(socket_, FIONBIO, &nonblocking);
            alive_ = true; worker_ = std::thread([this] { run(); }); return true;
        }
        unsigned short port() const { return port_; }
        // This is the only method called by NatNet. It never writes files or prints.
        void capture(sFrameOfMocapData& frame, int picoID, std::int64_t receiveUnixNs, std::int64_t receiveSteadyNs)
        {
            std::uint64_t generation;
            { std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return; generation = generation_; }
            try
            {
                auto owned = std::make_unique<Frame>();
                if (NatNet_CopyFrame(&frame, owned->data.get()) != ErrorCode_OK) { failCapture(generation, "NatNet_CopyFrame failed"); return; }
                owned->picoID = picoID; owned->receiveUnixNs = receiveUnixNs; owned->receiveSteadyNs = receiveSteadyNs;
                std::lock_guard<std::mutex> lock(mutex_);
                if (!accepting_ || generation != generation_) return;
                if (queue_.size() >= queueLimit_) { accepting_ = false; ++dropped_; error_ = "Recording queue overflow; take incomplete"; }
                else queue_.push_back(std::move(owned));
                condition_.notify_one();
            }
            catch (...) { failCapture(generation, "Unable to copy recording frame"); }
        }
        void shutdown()
        {
            alive_ = false; condition_.notify_one();
            if (worker_.joinable()) worker_.join();
            if (socket_ != INVALID_SOCKET) { closesocket(socket_); socket_ = INVALID_SOCKET; }
            if (winsock_) { WSACleanup(); winsock_ = false; }
        }
    private:
        SOCKET socket_ = INVALID_SOCKET;
        bool winsock_ = false;
        unsigned short port_ = 0;
        std::atomic<bool> alive_{false};
        std::thread worker_;
        std::mutex mutex_;
        std::condition_variable condition_;
        std::deque<std::unique_ptr<Frame>> queue_;
        bool accepting_ = false;
        std::uint64_t generation_ = 0, dropped_ = 0;
        std::size_t queueLimit_ = 32;
        std::string error_;
        fs::path root_, session_;
        std::string metadata_;
        std::function<std::string()> metadataProvider_;
        std::ofstream frames_, pico_;
        bool opened_ = false;
        std::uint64_t written_ = 0, missing_ = 0, discontinuities_ = 0;
        int previousFrame_ = -1;
        std::int64_t started_ = 0;
        std::map<std::string, std::uint64_t> lastCommand_;
        void failCapture(std::uint64_t generation, const char* error)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (generation != generation_ || !accepting_) return;
            accepting_ = false; ++dropped_; error_ = error; condition_.notify_one();
        }
        void start()
        {
            { std::lock_guard<std::mutex> lock(mutex_); if (accepting_) return; }
            finish();
            { std::lock_guard<std::mutex> lock(mutex_); error_.clear(); dropped_ = 0; }
            written_ = missing_ = discontinuities_ = 0; previousFrame_ = -1;
            try
            {
                if (metadataProvider_) metadata_ = metadataProvider_();
                started_ = unixNanoseconds();
                session_ = root_ / ("take_" + std::to_string(started_));
                if (!fs::create_directories(session_)) throw std::runtime_error("Take directory already exists");
                std::ofstream metadata(session_ / "metadata.json");
                metadata << "{\"schema\":\"mtc_natnet_recording_v1\",\"started_unix_ns\":" << started_
                         << ",\"position_units\":\"m\",\"coordinates\":\"native Motive stream; no Unity alignment or filtering\","
                         << "\"quaternion_order\":\"xyzw\",\"source\":" << metadata_ << "}\n";
                metadata.close();
                if (!metadata) throw std::runtime_error("Unable to write metadata");
                frames_.clear(); pico_.clear();
                frames_.open(session_ / "frames.jsonl"); pico_.open(session_ / "pico.csv");
                frames_.imbue(std::locale::classic()); pico_.imbue(std::locale::classic());
                if (!frames_ || !pico_) throw std::runtime_error("Unable to open recording files");
                pico_ << "frame,receive_unix_ns,motive_timestamp,pico_id,available,tracked,good_quality,mean_error_m,x_m,y_m,z_m,qx,qy,qz,qw,params\n";
                opened_ = true;
                std::lock_guard<std::mutex> lock(mutex_); ++generation_; accepting_ = true;
            }
            catch (const std::exception& e)
            {
                frames_.close(); pico_.close();
                std::lock_guard<std::mutex> lock(mutex_); error_ = e.what(); accepting_ = false;
            }
        }
        void write(const Frame& f)
        {
            writeFrame(frames_, f); writePico(pico_, f);
            if (!frames_ || !pico_)
            {
                std::lock_guard<std::mutex> lock(mutex_); accepting_ = false;
                ++dropped_; error_ = "Recording write failed; take incomplete"; return;
            }
            const int id = f.data->iFrame;
            if (previousFrame_ >= 0 && id > previousFrame_ + 1) missing_ += id - previousFrame_ - 1;
            if (previousFrame_ >= 0 && id <= previousFrame_) ++discontinuities_;
            previousFrame_ = id; ++written_;
        }
        void drain()
        {
            std::deque<std::unique_ptr<Frame>> batch;
            { std::lock_guard<std::mutex> lock(mutex_); batch.swap(queue_); }
            for (const auto& f : batch) write(*f);
        }
        void finish()
        {
            { std::lock_guard<std::mutex> lock(mutex_); accepting_ = false; }
            if (!opened_) return;
            drain(); frames_.close(); pico_.close(); opened_ = false;
            std::string error;
            std::uint64_t dropped;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if ((!frames_ || !pico_) && error_.empty()) error_ = "Unable to flush recording; take incomplete";
                error = error_; dropped = dropped_;
            }
            // No file operations while holding the callback's queue mutex.
            std::ofstream summary(session_ / "summary.json");
            summary << "{\"finalized\":true,\"started_unix_ns\":" << started_ << ",\"stopped_unix_ns\":" << unixNanoseconds()
                    << ",\"frames_written\":" << written_ << ",\"recording_drops\":" << dropped
                    << ",\"missing_source_frame_ids\":" << missing_ << ",\"source_discontinuities\":" << discontinuities_
                    << ",\"error\":"; text(summary, error); summary << "}\n"; summary.close();
            if (!summary) { std::lock_guard<std::mutex> lock(mutex_); error_ = "Unable to write recording summary; take incomplete"; }
        }
        void commands()
        {
            for (int count = 0; count < 16; ++count)
            {
                char buffer[512]; sockaddr_in sender{}; int size = sizeof(sender);
                int length = recvfrom(socket_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&sender), &size);
                if (length <= 0) return;
                std::istringstream in(std::string(buffer, length));
                std::string protocol, client, operation; std::uint64_t sequence;
                if (!(in >> protocol >> client >> sequence >> operation) || protocol != "MTCREC1" || client.size() > 64) continue;
                if (operation != "START" && operation != "STOP" && operation != "STATUS") continue;
                // Retries and out-of-order UDP commands must never create a second take.
                auto it = lastCommand_.find(client);
                if (operation != "STATUS" && (it == lastCommand_.end() || sequence > it->second))
                {
                    if (lastCommand_.size() >= 128 && it == lastCommand_.end()) continue;
                    lastCommand_[client] = sequence;
                    if (operation == "START") start(); else finish();
                }
                bool accepting; std::string error; std::uint64_t dropped;
                { std::lock_guard<std::mutex> lock(mutex_); accepting = accepting_; error = error_; dropped = dropped_; }
                if (!accepting && opened_) finish();
                { std::lock_guard<std::mutex> lock(mutex_); error = error_; dropped = dropped_; }
                std::ostringstream response;
                response << "MTCREC1 " << client << ' ' << sequence << ' ' << (!error.empty() ? "ERROR" : accepting ? "ON" : "OFF")
                         << ' ' << written_ << ' ' << dropped << ' ' << (session_.empty() ? "-" : session_.u8string());
                const auto bytes = response.str();
                sendto(socket_, bytes.data(), static_cast<int>(bytes.size()), 0, reinterpret_cast<sockaddr*>(&sender), size);
            }
        }
        void run()
        {
            auto flushTime = std::chrono::steady_clock::now();
            while (alive_)
            {
                commands(); drain();
                bool accepting;
                { std::lock_guard<std::mutex> lock(mutex_); accepting = accepting_; }
                if (opened_ && !accepting) finish();
                if (opened_ && std::chrono::steady_clock::now() - flushTime >= std::chrono::seconds(1))
                {
                    frames_.flush(); pico_.flush(); flushTime = std::chrono::steady_clock::now();
                    if (!frames_ || !pico_)
                    { std::lock_guard<std::mutex> lock(mutex_); accepting_ = false; error_ = "Recording flush failed; take incomplete"; }
                }
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait_for(lock, std::chrono::milliseconds(10), [&] { return !queue_.empty() || !alive_; });
            }
            finish();
        }
    };
}
