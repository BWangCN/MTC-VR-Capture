// Tracks retain persistent IDs through short occlusions, and Unity receives
// id,x,y,z for each active marker so visuals do not swap when ordering changes.
//
// Unity packet format:
//   frameNumber;pointCount;id,x,y,z;id,x,y,z;...
//
// Build with MSVC, not MinGW, because NatNetLib.lib is an MSVC library.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "MocapRecording.h"

namespace
{
    // =============================================================================
    // NatNet configuration
    // =============================================================================

    constexpr const char *kServerAddress = "192.168.0.209";
    constexpr std::uint16_t kCommandPort = 1510;
    constexpr std::uint16_t kDataPort = 1511;

    // =============================================================================
    // Unity UDP configuration
    // =============================================================================

    // Use 127.0.0.1 when Unity runs on this Windows computer.
    // Replace it with the Ubuntu laptop or PICO IP when sending over the network.
    // constexpr const char *kUnityAddress = "192.168.0.82";
    constexpr const char *kUnityAddress = "127.0.0.1";
    constexpr std::uint16_t kUnityPort = 5005;

#ifdef _WIN32
    SOCKET g_unitySocket = INVALID_SOCKET;
    sockaddr_in g_unityDestination{};
#endif

    // =============================================================================
    // Marker tracking configuration
    // =============================================================================

    // A newly visible marker must move this far from where its new track began.
    constexpr double kMovementThresholdMeters = 0.1; // 3 cm

    // Maximum distance a point may move between consecutive frames and still be
    // associated with the same track.
    constexpr double kMaximumAssociationDistanceMeters = 0.12;

    // Require a track to exist for this many matched frames before sending it.
    constexpr int kMinimumTrackAgeFrames = 3;

    constexpr float kMaximumPicoMeanErrorMeters = 0.01f;

    // =============================================================================
    // Global state
    // =============================================================================

    std::atomic<bool> g_running{true};
    std::atomic<std::uint64_t> g_framesReceived{0};
    std::atomic<std::uint64_t> g_missingFrameIDs{0};
    std::atomic<int> g_lastFrameNumber{-1};
    // Only the main thread writes to the console. A blocked terminal must never
    // hold up NatNet's receive callback and accumulate old tracking frames.
    std::atomic<std::size_t> g_rawPointCount{0};
    std::atomic<std::size_t> g_activePointCount{0};
    std::atomic<std::uint64_t> g_sendErrors{0};
    std::atomic<std::uint64_t> g_serverClockFrequency{0};
    std::atomic<double> g_cameraAgeMilliseconds{-1};
    std::atomic<double> g_transitMilliseconds{-1};
    std::atomic<double> g_softwareMilliseconds{-1};
    std::atomic<double> g_maximumCallbackMilliseconds{0};

    // =============================================================================
    // Data structures
    // =============================================================================

    struct Point3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct TrackedPoint
    {
        int id = -1;
        Point3 position{};
    };

    constexpr const char *kPicoRigidBodyName = "Pico";

    std::atomic<int> g_picoRigidBodyID{-1};
    mocap_recording::Recorder g_recorder;

    struct RigidBodyPose
    {
        bool available = false;
        bool tracked = false;

        bool goodQuality = false;

        Point3 position{};

        float qx = 0.0f;
        float qy = 0.0f;
        float qz = 0.0f;
        float qw = 1.0f;

        float meanError = -1.0f;
    };

    struct MarkerTrack
    {
        int id = -1;

        Point3 currentPosition{};
        Point3 previousPosition{};

        // The marker must move 3 cm from this position to become active.
        Point3 activationReferencePosition{};

        int ageFrames = 1;

        bool matchedThisFrame = false;
        bool active = false;
        int missedFrames = 0;
    };

    std::vector<MarkerTrack> g_tracks;
    int g_nextTrackID = 0;

    // =============================================================================
    // Signal handling
    // =============================================================================

    void handleSignal(int)
    {
        g_running = false;
    }

    // =============================================================================
    // Geometry
    // =============================================================================

    double squaredDistance(const Point3 &first, const Point3 &second)
    {
        const double dx =
            static_cast<double>(first.x) -
            static_cast<double>(second.x);

        const double dy =
            static_cast<double>(first.y) -
            static_cast<double>(second.y);

        const double dz =
            static_cast<double>(first.z) -
            static_cast<double>(second.z);

        return dx * dx + dy * dy + dz * dz;
    }

    double pointDistance(const Point3 &first, const Point3 &second)
    {
        return std::sqrt(
            squaredDistance(first, second));
    }

    // =============================================================================
    // Unity UDP
    // =============================================================================

    bool initializeUnityUdp()
    {
#ifdef _WIN32
        WSADATA wsaData{};

        const int startupResult =
            WSAStartup(MAKEWORD(2, 2), &wsaData);

        if (startupResult != 0)
        {
            std::cerr
                << "WSAStartup failed with error "
                << startupResult
                << ".\n";

            return false;
        }

        g_unitySocket = socket(
            AF_INET,
            SOCK_DGRAM,
            IPPROTO_UDP);

        if (g_unitySocket == INVALID_SOCKET)
        {
            std::cerr
                << "Unable to create Unity UDP socket. Error: "
                << WSAGetLastError()
                << '\n';

            WSACleanup();
            return false;
        }

        g_unityDestination = {};
        g_unityDestination.sin_family = AF_INET;
        g_unityDestination.sin_port = htons(kUnityPort);

        const int addressResult = inet_pton(
            AF_INET,
            kUnityAddress,
            &g_unityDestination.sin_addr);

        if (addressResult != 1)
        {
            std::cerr
                << "Invalid Unity IP address: "
                << kUnityAddress
                << '\n';

            closesocket(g_unitySocket);
            g_unitySocket = INVALID_SOCKET;
            WSACleanup();

            return false;
        }

        return true;
#else
        std::cerr
            << "Unity UDP output is implemented for Windows only.\n";

        return false;
#endif
    }

    void shutdownUnityUdp()
    {
#ifdef _WIN32
        if (g_unitySocket != INVALID_SOCKET)
        {
            closesocket(g_unitySocket);
            g_unitySocket = INVALID_SOCKET;
        }

        WSACleanup();
#endif
    }

    bool findPicoRigidBodyID(
        NatNetClient &client)
    {
        sDataDescriptions *descriptions =
            nullptr;

        const int result =
            client.GetDataDescriptionList(
                &descriptions);

        if (result != ErrorCode_OK ||
            descriptions == nullptr)
        {
            std::cerr
                << "Unable to retrieve Motive data descriptions.\n";

            return false;
        }

        bool found = false;

        for (int i = 0;
             i < descriptions->nDataDescriptions;
             ++i)
        {
            sDataDescription *description =
                &descriptions->arrDataDescriptions[i];

            if (description == nullptr)
            {
                continue;
            }

            if (description->type !=
                Descriptor_RigidBody)
            {
                continue;
            }

            sRigidBodyDescription *rigidBody =
                description->Data.RigidBodyDescription;

            if (rigidBody == nullptr ||
                rigidBody->szName == nullptr)
            {
                continue;
            }

            std::cout
                << "Motive rigid body: "
                << rigidBody->szName
                << " | ID: "
                << rigidBody->ID
                << '\n';

            if (std::string(rigidBody->szName) ==
                kPicoRigidBodyName)
            {
                g_picoRigidBodyID =
                    rigidBody->ID;

                found = true;
            }
        }

        // GetDataDescriptionList allocates caller-owned descriptions (SDK sample
        // clients pair each successful query with NatNet_FreeDescriptions).
        NatNet_FreeDescriptions(descriptions);

        if (!found)
        {
            std::cerr
                << "Could not find rigid body named \""
                << kPicoRigidBodyName
                << "\".\n";

            return false;
        }

        std::cout
            << "Using Pico rigid body ID: "
            << g_picoRigidBodyID
            << '\n';

        return true;
    }

    RigidBodyPose extractPicoRigidBodyPose(
        const sFrameOfMocapData &frame)
    {
        RigidBodyPose pose;

        const int picoID = g_picoRigidBodyID.load();
        if (picoID < 0)
        {
            return pose;
        }

        for (int i = 0;
             i < frame.nRigidBodies;
             ++i)
        {
            const sRigidBodyData &rigidBody =
                frame.RigidBodies[i];

            if (rigidBody.ID != picoID)
            {
                continue;
            }

            pose.available = true;

            const bool motiveTracked =
                (rigidBody.params & 0x01) != 0;

            pose.meanError =
                rigidBody.MeanError;

            const bool errorIsValid =
                std::isfinite(pose.meanError) &&
                pose.meanError >= 0.0f;

            pose.goodQuality =
                motiveTracked &&
                errorIsValid &&
                pose.meanError <=
                    kMaximumPicoMeanErrorMeters;

            // Unity should only use the pose when this is true.
            pose.tracked =
                pose.goodQuality;

            pose.position = Point3{
                rigidBody.x,
                rigidBody.y,
                rigidBody.z};

            pose.qx = rigidBody.qx;
            pose.qy = rigidBody.qy;
            pose.qz = rigidBody.qz;
            pose.qw = rigidBody.qw;

            return pose;
        }

        return pose;
    }

    void sendFrameToUnity(
        int frameNumber,
        const std::vector<TrackedPoint> &points,
        const RigidBodyPose &picoPose)
    {
#ifdef _WIN32
        if (g_unitySocket == INVALID_SOCKET)
        {
            return;
        }

        std::ostringstream message;

        message
            << frameNumber
            << ';'
            << points.size();

        message
            << std::fixed
            << std::setprecision(6);

        for (const TrackedPoint &point : points)
        {
            message
                << ';'
                << point.id
                << ','
                << point.position.x
                << ','
                << point.position.y
                << ','
                << point.position.z;
        }

        /*
         * Rigid-body section:
         *
         * ;PICO;available;tracked;x,y,z;qx,qy,qz,qw
         */
        /*
         * PICO;available;goodQuality;meanError;x,y,z;qx,qy,qz,qw
         */
        message
            << ";PICO;"
            << (picoPose.available ? 1 : 0)
            << ';'
            << (picoPose.goodQuality ? 1 : 0)
            << ';'
            << picoPose.meanError
            << ';'
            << picoPose.position.x
            << ','
            << picoPose.position.y
            << ','
            << picoPose.position.z
            << ';'
            << picoPose.qx
            << ','
            << picoPose.qy
            << ','
            << picoPose.qz
            << ','
            << picoPose.qw;

        const std::string payload =
            message.str();

        const int result = sendto(
            g_unitySocket,
            payload.c_str(),
            static_cast<int>(payload.size()),
            0,
            reinterpret_cast<const sockaddr *>(
                &g_unityDestination),
            static_cast<int>(
                sizeof(g_unityDestination)));

        if (result == SOCKET_ERROR)
        {
            g_sendErrors.fetch_add(1, std::memory_order_relaxed);
        }
#else
        (void)frameNumber;
        (void)points;
        (void)picoPose;
#endif
    }
    // =============================================================================
    // NatNet marker extraction
    // =============================================================================

    std::vector<Point3> extractMarkers(
        const sFrameOfMocapData &frame)
    {
        std::vector<Point3> points;

        // Prefer raw unlabeled points.
        if (frame.nOtherMarkers > 0)
        {
            points.reserve(
                static_cast<std::size_t>(
                    frame.nOtherMarkers));

            for (int i = 0;
                 i < frame.nOtherMarkers;
                 ++i)
            {
                const auto &marker =
                    frame.OtherMarkers[i];

                points.push_back(
                    Point3{
                        marker[0],
                        marker[1],
                        marker[2]});
            }

            return points;
        }

        // Fall back to labeled markers only when OtherMarkers is empty.
        points.reserve(
            static_cast<std::size_t>(
                frame.nLabeledMarkers));

        for (int i = 0;
             i < frame.nLabeledMarkers;
             ++i)
        {
            const sMarker &marker =
                frame.LabeledMarkers[i];

            points.push_back(
                Point3{
                    marker.x,
                    marker.y,
                    marker.z});
        }

        return points;
    }

    // =============================================================================
    // Track association and movement filtering
    // =============================================================================

    void beginTrackingFrame()
    {
        for (MarkerTrack &track : g_tracks)
        {
            track.matchedThisFrame = false;
        }
    }

    int findNearestTrackIndex(
        const Point3 &point)
    {
        const double maximumSquaredDistance =
            kMaximumAssociationDistanceMeters *
            kMaximumAssociationDistanceMeters;

        double bestSquaredDistance =
            std::numeric_limits<double>::infinity();

        int bestIndex = -1;

        for (std::size_t index = 0;
             index < g_tracks.size();
             ++index)
        {
            const MarkerTrack &track =
                g_tracks[index];

            if (track.matchedThisFrame)
            {
                continue;
            }

            const double candidateSquaredDistance =
                squaredDistance(
                    point,
                    track.currentPosition);

            if (candidateSquaredDistance >
                maximumSquaredDistance)
            {
                continue;
            }

            if (candidateSquaredDistance <
                bestSquaredDistance)
            {
                bestSquaredDistance =
                    candidateSquaredDistance;

                bestIndex =
                    static_cast<int>(index);
            }
        }

        return bestIndex;
    }

    void createTrack(
        const Point3 &point)
    {
        MarkerTrack track;

        track.id =
            g_nextTrackID++;

        track.currentPosition =
            point;

        track.previousPosition =
            point;

        track.activationReferencePosition =
            point;

        track.ageFrames = 1;
        track.matchedThisFrame = true;
        track.active = false;

        g_tracks.push_back(track);
    }

    void updateTrack(
        MarkerTrack &track,
        const Point3 &point)
    {
        track.previousPosition =
            track.currentPosition;

        track.currentPosition =
            point;

        track.matchedThisFrame = true;
        track.missedFrames = 0;
        ++track.ageFrames;

        // Once active, remain active until this marker disappears.
        if (track.active)
        {
            return;
        }

        const double displacementFromTrackStart =
            pointDistance(
                track.currentPosition,
                track.activationReferencePosition);

        if (displacementFromTrackStart >=
            kMovementThresholdMeters)
        {
            track.active = true;
        }
    }

    void removeDisappearedTracks()
    {
        constexpr int kMaximumMissedFrames = 20;

        for (MarkerTrack &track : g_tracks)
        {
            if (!track.matchedThisFrame)
            {
                ++track.missedFrames;
            }
        }

        g_tracks.erase(
            std::remove_if(
                g_tracks.begin(),
                g_tracks.end(),
                [kMaximumMissedFrames](const MarkerTrack &track)
                {
                    return track.missedFrames > kMaximumMissedFrames;
                }),
            g_tracks.end());
    }

    void updateTracks(
        const std::vector<Point3> &points)
    {
        beginTrackingFrame();

        for (const Point3 &point : points)
        {
            const int trackIndex =
                findNearestTrackIndex(point);

            if (trackIndex < 0)
            {
                createTrack(point);
                continue;
            }

            updateTrack(
                g_tracks[static_cast<std::size_t>(
                    trackIndex)],
                point);
        }

        removeDisappearedTracks();
    }

    std::vector<TrackedPoint> getActiveVisiblePoints()
    {
        std::vector<TrackedPoint> points;
        points.reserve(g_tracks.size());

        for (const MarkerTrack &track : g_tracks)
        {
            if (!track.active ||
                !track.matchedThisFrame ||
                track.ageFrames < kMinimumTrackAgeFrames)
            {
                continue;
            }

            points.push_back(TrackedPoint{
                track.id,
                track.currentPosition});
        }

        // Stable ordering is useful for logs and debugging, even though Unity
        // now keys visuals by persistent track ID.
        std::sort(
            points.begin(),
            points.end(),
            [](const TrackedPoint &a, const TrackedPoint &b)
            {
                return a.id < b.id;
            });

        return points;
    }

    // =============================================================================
    // NatNet callback
    // =============================================================================

    void NATNET_CALLCONV frameReceivedCallback(
        sFrameOfMocapData *frame,
        void *userData)
    {
        if (frame == nullptr)
        {
            return;
        }

        const auto callbackStart = std::chrono::steady_clock::now();
        const auto receiveUnixNs = mocap_recording::unixNanoseconds();
        const auto receiveSteadyNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            callbackStart.time_since_epoch()).count();
        g_recorder.capture(*frame, g_picoRigidBodyID.load(), receiveUnixNs, receiveSteadyNs);
        auto *client = static_cast<NatNetClient *>(userData);
        // NatNet's synchronized clock measures age BEFORE Unity receives a frame.
        // Zero timestamps (e.g. playback/unsupported camera systems) are unavailable.
        const double transitMs = client && frame->TransmitTimestamp
            ? client->SecondsSinceHostTimestamp(frame->TransmitTimestamp) * 1000.0 : -1.0;
        const double cameraMs = client && frame->CameraMidExposureTimestamp
            ? client->SecondsSinceHostTimestamp(frame->CameraMidExposureTimestamp) * 1000.0 : -1.0;
        g_transitMilliseconds.store(std::isfinite(transitMs) ? transitMs : -1.0, std::memory_order_relaxed);
        g_cameraAgeMilliseconds.store(std::isfinite(cameraMs) ? cameraMs : -1.0, std::memory_order_relaxed);
        const auto frequency = g_serverClockFrequency.load(std::memory_order_relaxed);
        const bool softwareAvailable = frequency && frame->CameraDataReceivedTimestamp &&
            frame->TransmitTimestamp >= frame->CameraDataReceivedTimestamp;
        g_softwareMilliseconds.store(softwareAvailable
            ? (frame->TransmitTimestamp - frame->CameraDataReceivedTimestamp) * 1000.0 / frequency : -1.0,
            std::memory_order_relaxed);

        g_framesReceived.fetch_add(
            1,
            std::memory_order_relaxed);

        const int previousFrame =
            g_lastFrameNumber.exchange(
                frame->iFrame,
                std::memory_order_relaxed);

        if (previousFrame >= 0 &&
            frame->iFrame >
                previousFrame + 1)
        {
            const auto missing =
                static_cast<std::uint64_t>(
                    frame->iFrame -
                    previousFrame -
                    1);

            g_missingFrameIDs.fetch_add(
                missing,
                std::memory_order_relaxed);
        }

        const std::vector<Point3> points =
            extractMarkers(*frame);

        updateTracks(points);

        const std::vector<TrackedPoint> activePoints =
            getActiveVisiblePoints();

        const RigidBodyPose picoPose =
            extractPicoRigidBodyPose(*frame);

        sendFrameToUnity(
            frame->iFrame,
            activePoints,
            picoPose);

        g_rawPointCount.store(points.size(), std::memory_order_relaxed);
        g_activePointCount.store(activePoints.size(), std::memory_order_relaxed);
        const double processingMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - callbackStart).count();
        double previousMaximum = g_maximumCallbackMilliseconds.load(std::memory_order_relaxed);
        while (processingMs > previousMaximum && !g_maximumCallbackMilliseconds.compare_exchange_weak(
            previousMaximum, processingMs, std::memory_order_relaxed)) {}
    }

    // =============================================================================
    // Motive frame-rate query
    // =============================================================================

    float queryMotiveFrameRate(
        NatNetClient &client)
    {
        void *response = nullptr;
        int responseSize = 0;

        const int result =
            client.SendMessageAndWait(
                "FrameRate",
                &response,
                &responseSize);

        if (result == ErrorCode_OK &&
            response != nullptr &&
            responseSize >=
                static_cast<int>(
                    sizeof(float)))
        {
            const float configuredFrameRate =
                *static_cast<float *>(
                    response);

            std::cout
                << "  Configured Motive frame rate: "
                << configuredFrameRate
                << " Hz\n";
            return configuredFrameRate;
        }
        else
        {
            std::cerr
                << "Unable to query Motive frame rate."
                << " Error code: "
                << result
                << '\n';
        }
        return 0.0f;
    }

    std::string recordingMetadata(NatNetClient& client, const sServerDescription& server, float frameRate)
    {
        sDataDescriptions* descriptions = nullptr;
        const bool available = client.GetDataDescriptionList(&descriptions) == ErrorCode_OK && descriptions;
        std::unique_ptr<sDataDescriptions, decltype(&NatNet_FreeDescriptions)> ownedDescriptions(descriptions, &NatNet_FreeDescriptions);
        int picoID = g_picoRigidBodyID.load();
        if (available)
        {
            picoID = -1;
            for (int i = 0; i < descriptions->nDataDescriptions; ++i)
            {
                const auto& d = descriptions->arrDataDescriptions[i];
                if (d.type == Descriptor_RigidBody && d.Data.RigidBodyDescription &&
                    std::string(d.Data.RigidBodyDescription->szName) == kPicoRigidBodyName)
                    picoID = d.Data.RigidBodyDescription->ID;
            }
        }
        g_picoRigidBodyID.store(picoID);
        std::ostringstream out; out.imbue(std::locale::classic());
        out << "{\"server_address\":"; mocap_recording::text(out, kServerAddress);
        out << ",\"server_application\":"; mocap_recording::text(out, server.szHostApp);
        out << ",\"motive_version\":";
        mocap_recording::array(out, 4, [&](int i) { out << int(server.HostAppVersion[i]); });
        out << ",\"natnet_version\":";
        mocap_recording::array(out, 4, [&](int i) { out << int(server.NatNetVersion[i]); });
        out << ",\"host_clock_frequency\":" << server.HighResClockFrequency
            << ",\"frame_rate\":" << frameRate << ",\"pico_name\":\"Pico\",\"pico_id\":" << g_picoRigidBodyID.load()
            << ",\"models_available\":" << (available ? "true" : "false")
            << ",\"models\":" << (available ? mocap_recording::describeModels(*descriptions) : "[]") << '}';
        return out.str();
    }

} // namespace

// =============================================================================
// Main
// =============================================================================

int main(
    int argc,
    char *argv[])
{
    if (argc < 2 || argc > 3)
    {
        std::cerr
            << "Usage:\n  "
            << argv[0]
            << " <client-local-ip> [recording-directory]\n\n"
            << "Example:\n  "
            << argv[0]
            << " 192.168.0.237\n";

        return EXIT_FAILURE;
    }

    const char *localAddress =
        argv[1];

    std::signal(
        SIGINT,
        handleSignal);

    std::signal(
        SIGTERM,
        handleSignal);

    NatNetClient client;

    client.SetFrameReceivedCallback(
        frameReceivedCallback,
        &client);

    sNatNetClientConnectParams
        connectParams;

    connectParams.connectionType =
        ConnectionType_Unicast;

    connectParams.serverAddress =
        kServerAddress;

    connectParams.localAddress =
        localAddress;

    connectParams.serverCommandPort =
        kCommandPort;

    connectParams.serverDataPort =
        kDataPort;

    std::cout
        << "Connecting to Motive...\n"
        << "  Server:       "
        << kServerAddress
        << '\n'
        << "  Client:       "
        << localAddress
        << '\n'
        << "  Command port: "
        << kCommandPort
        << '\n'
        << "  Data port:    "
        << kDataPort
        << '\n';

    const int connectResult =
        client.Connect(
            connectParams);

    if (connectResult !=
        ErrorCode_OK)
    {
        std::cerr
            << "NatNet connection failed."
            << " Error code: "
            << connectResult
            << '\n';

        return EXIT_FAILURE;
    }

    sServerDescription
        serverDescription{};

    const int descriptionResult =
        client.GetServerDescription(
            &serverDescription);

    if (descriptionResult !=
            ErrorCode_OK ||
        !serverDescription.HostPresent)
    {
        std::cerr
            << "Motive server description unavailable.\n";

        client.Disconnect();
        return EXIT_FAILURE;
    }

    std::cout
        << "\nConnected successfully."
        << "\n  Server application: "
        << serverDescription.szHostApp
        << "\n  Motive version: "
        << serverDescription.HostAppVersion[0]
        << '.'
        << serverDescription.HostAppVersion[1]
        << '.'
        << serverDescription.HostAppVersion[2]
        << '.'
        << serverDescription.HostAppVersion[3]
        << "\n  NatNet version: "
        << serverDescription.NatNetVersion[0]
        << '.'
        << serverDescription.NatNetVersion[1]
        << '.'
        << serverDescription.NatNetVersion[2]
        << '.'
        << serverDescription.NatNetVersion[3]
        << '\n';

    if (!findPicoRigidBodyID(client))
    {
        std::cerr
            << "Pico pose will not be sent until the "
            << "rigid body can be found.\n";
    }

    const float frameRate = queryMotiveFrameRate(client);

    g_serverClockFrequency.store(serverDescription.HighResClockFrequency, std::memory_order_relaxed);

    if (!initializeUnityUdp())
    {
        std::cerr
            << "Warning: Unity UDP output is unavailable.\n";
    }
    else
    {
        std::cout
            << "Sending active points to Unity at "
            << kUnityAddress
            << ':'
            << kUnityPort
            << '\n';
    }

    const auto recordingRoot = argc == 3 ? std::filesystem::path(argv[2]) :
        std::filesystem::absolute(argv[0]).parent_path() / "recordings";
    if (!g_recorder.initialize(recordingRoot, "{}", 5011, 32,
            [&] { return recordingMetadata(client, serverDescription, frameRate); }))
        std::cerr << "Recording control unavailable: cannot bind localhost UDP 5011.\n";
    else
        std::cout << "Recording control: localhost UDP 5011 | output: " << recordingRoot.u8string() << '\n';

    std::cout
        << "\nMovement-filter settings:"
        << "\n  Activation distance: "
        << kMovementThresholdMeters
        << " m"
        << "\n  Association distance: "
        << kMaximumAssociationDistanceMeters
        << " m"
        << "\n  Disappeared tracks: retained for 20 frames"
        << "\n  Diagnostics: one summary per second; no console I/O in the frame callback"
        << "\n  Timing values of -1 mean unavailable"
        << "\n\nReceiving frames."
        << "\nPress Ctrl+C to stop.\n\n";

    auto previousRateTime =
        std::chrono::steady_clock::now();

    while (g_running)
    {
        std::this_thread::sleep_for(
            std::chrono::seconds(1));

        const auto now =
            std::chrono::steady_clock::now();

        const double elapsedSeconds =
            std::chrono::duration<double>(
                now -
                previousRateTime)
                .count();

        const std::uint64_t receivedFrames =
            g_framesReceived.exchange(
                0,
                std::memory_order_relaxed);

        const std::uint64_t missingFrames =
            g_missingFrameIDs.exchange(
                0,
                std::memory_order_relaxed);

        const double receivedHz =
            static_cast<double>(
                receivedFrames) /
            elapsedSeconds;

        std::cout
            << "[NatNet] Received: "
            << std::fixed
            << std::setprecision(2)
            << receivedHz
            << " Hz"
            << " | Missing frame IDs: "
            << missingFrames
            << " | frame " << g_lastFrameNumber.load(std::memory_order_relaxed)
            << " | points raw/active " << g_rawPointCount.load(std::memory_order_relaxed)
            << '/' << g_activePointCount.load(std::memory_order_relaxed)
            << " | callback max " << g_maximumCallbackMilliseconds.exchange(0, std::memory_order_relaxed) << " ms"
            << " | Motive software " << g_softwareMilliseconds.load(std::memory_order_relaxed) << " ms"
            << " | transit/queue " << g_transitMilliseconds.load(std::memory_order_relaxed) << " ms"
            << " | camera age " << g_cameraAgeMilliseconds.load(std::memory_order_relaxed) << " ms"
            << " | send errors " << g_sendErrors.exchange(0, std::memory_order_relaxed)
            << std::endl; // Flush only on this summary thread, never in the callback.

        previousRateTime = now;
    }

    std::cout
        << "\nDisconnecting...\n";

    client.Disconnect();
    g_recorder.shutdown();
    shutdownUnityUdp();

    return EXIT_SUCCESS;
}
