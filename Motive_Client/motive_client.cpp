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

    // Print diagnostics every N NatNet frames.
    constexpr int kPrintEveryNFrames = 10;

    constexpr float kMaximumPicoMeanErrorMeters = 0.01f;

    // =============================================================================
    // Global state
    // =============================================================================

    std::atomic<bool> g_running{true};
    std::atomic<std::uint64_t> g_framesReceived{0};
    std::atomic<std::uint64_t> g_missingFrameIDs{0};
    std::atomic<int> g_lastFrameNumber{-1};

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

    int g_picoRigidBodyID = -1;

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

        // Do not free descriptions.
        // NatNetClient owns the returned memory.

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

        if (g_picoRigidBodyID < 0)
        {
            return pose;
        }

        for (int i = 0;
             i < frame.nRigidBodies;
             ++i)
        {
            const sRigidBodyData &rigidBody =
                frame.RigidBodies[i];

            if (rigidBody.ID != g_picoRigidBodyID)
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
            std::cerr
                << "Unity UDP send failed. Error: "
                << WSAGetLastError()
                << '\n';
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
    // Diagnostics
    // =============================================================================

    void printActiveMarkers(
        int frameNumber,
        std::size_t rawPointCount)
    {
        std::size_t activeVisibleCount = 0;

        for (const MarkerTrack &track : g_tracks)
        {
            if (track.active &&
                track.matchedThisFrame &&
                track.ageFrames >=
                    kMinimumTrackAgeFrames)
            {
                ++activeVisibleCount;
            }
        }

        std::cout
            << std::fixed
            << std::setprecision(4)
            << "\n========================================"
            << "\nFrame: "
            << frameNumber
            << "\nRaw marker points: "
            << rawPointCount
            << "\nCurrent visible tracks: "
            << g_tracks.size()
            << "\nActive visible points: "
            << activeVisibleCount
            << '\n';

        for (const MarkerTrack &track : g_tracks)
        {
            if (!track.active ||
                !track.matchedThisFrame ||
                track.ageFrames <
                    kMinimumTrackAgeFrames)
            {
                continue;
            }

            const double movementThisFrame =
                pointDistance(
                    track.currentPosition,
                    track.previousPosition);

            const double displacementFromTrackStart =
                pointDistance(
                    track.currentPosition,
                    track.activationReferencePosition);

            std::cout
                << "\nTrack ID: "
                << track.id
                << "\n  Position: ["
                << track.currentPosition.x
                << ", "
                << track.currentPosition.y
                << ", "
                << track.currentPosition.z
                << "]"
                << "\n  Movement this frame: "
                << movementThisFrame
                << " m"
                << "\n  Displacement from track start: "
                << displacementFromTrackStart
                << " m"
                << "\n  Track age: "
                << track.ageFrames
                << " frames"
                << '\n';
        }

        std::cout
            << "========================================\n";
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

        (void)userData;

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

        if (frame->iFrame %
                kPrintEveryNFrames ==
            0)
        {
            printActiveMarkers(
                frame->iFrame,
                points.size());
        }
    }

    // =============================================================================
    // Motive frame-rate query
    // =============================================================================

    void queryMotiveFrameRate(
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
        }
        else
        {
            std::cerr
                << "Unable to query Motive frame rate."
                << " Error code: "
                << result
                << '\n';
        }
    }

} // namespace

// =============================================================================
// Main
// =============================================================================

int main(
    int argc,
    char *argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage:\n  "
            << argv[0]
            << " <client-local-ip>\n\n"
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

    queryMotiveFrameRate(client);

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

    std::cout
        << "\nMovement-filter settings:"
        << "\n  Activation distance: "
        << kMovementThresholdMeters
        << " m"
        << "\n  Association distance: "
        << kMaximumAssociationDistanceMeters
        << " m"
        << "\n  Disappeared markers: removed immediately"
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
            << '\n';

        previousRateTime = now;
    }

    std::cout
        << "\nDisconnecting...\n";

    client.Disconnect();
    shutdownUnityUdp();

    return EXIT_SUCCESS;
}