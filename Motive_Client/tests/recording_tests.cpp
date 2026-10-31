// Loopback only. Never connects to Motive or starts/stops the user's sender.
#define main motiveClientMainForRecordingTests
#include "../motive_client.cpp"
#undef main
#include <stdexcept>

namespace
{
    void require(bool test, const char* message) { if (!test) throw std::runtime_error(message); }
    std::string command(SOCKET socket, unsigned short port, const std::string& request)
    {
        sockaddr_in destination{}; destination.sin_family = AF_INET;
        destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK); destination.sin_port = htons(port);
        require(sendto(socket, request.data(), int(request.size()), 0, reinterpret_cast<sockaddr*>(&destination), sizeof(destination)) != SOCKET_ERROR,
                "Failed to send control command");
        char reply[4096];
        const int size = recv(socket, reply, sizeof(reply), 0);
        require(size > 0, "No recording acknowledgement");
        return std::string(reply, size);
    }
    void state(const std::string& reply, const char* expected)
    {
        require(reply.find(std::string(" ") + expected + " ") != std::string::npos, "Unexpected recording state");
    }
}
int main(int argc, char** argv)
{
    try
    {
        require(argc == 2, "Pass a unique output root");
        const auto root = std::filesystem::absolute(argv[1]);
        WSADATA wsa{}; require(WSAStartup(MAKEWORD(2, 2), &wsa) == 0, "WSAStartup failed");
        SOCKET control = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        DWORD timeout = 3000; setsockopt(control, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<char*>(&timeout), sizeof(timeout));
        auto descriptions = std::make_unique<sDataDescriptions>();
        auto rb = std::make_unique<sRigidBodyDescription>();
        strcpy_s(rb->szName, "Pico"); rb->ID = 42; rb->offsetqw = 1;
        descriptions->nDataDescriptions = 1; descriptions->arrDataDescriptions[0].type = Descriptor_RigidBody;
        descriptions->arrDataDescriptions[0].Data.RigidBodyDescription = rb.get();
        const auto models = mocap_recording::describeModels(*descriptions);
        mocap_recording::Recorder recorder;
        require(recorder.initialize(root / "normal", "{\"frame_rate\":100,\"host_clock_frequency\":1000,\"models\":" + models + '}', 0), "Cannot bind test recorder");
        state(command(control, recorder.port(), "MTCREC1 test 1 STATUS"), "OFF");
        const auto started = command(control, recorder.port(), "MTCREC1 test 2 START"); state(started, "ON");
        require(command(control, recorder.port(), "MTCREC1 test 2 START") == started, "START retry created another take");

        auto frame = std::make_unique<sFrameOfMocapData>();
        auto other = std::make_unique<MarkerData[]>(2);
        other[0][0] = 1.25f; other[1][1] = 2.5f;
        frame->nOtherMarkers = 2; frame->OtherMarkers = other.get();
        frame->nMarkerSets = 1; strcpy_s(frame->MocapData[0].szName, "set \"A\"");
        frame->MocapData[0].nMarkers = 2; frame->MocapData[0].Markers = other.get();
        frame->nLabeledMarkers = 1; frame->LabeledMarkers[0].ID = 123;
        frame->LabeledMarkers[0].x = 9; frame->LabeledMarkers[0].residual = 0.002f;
        frame->LabeledMarkers[0].params = 0x41;
        frame->nRigidBodies = 1; frame->RigidBodies[0].ID = 42;
        frame->RigidBodies[0].x = 4.5f; frame->RigidBodies[0].qy = 0.5f;
        frame->RigidBodies[0].qw = 0.8660254f; frame->RigidBodies[0].params = 1;
        frame->RigidBodies[0].MeanError = 0.001f;
        frame->nSkeletons = 1; frame->Skeletons[0].skeletonID = 17;
        frame->Skeletons[0].nRigidBodies = 1; frame->Skeletons[0].RigidBodyData = frame->RigidBodies;
        frame->nAssets = 1; frame->Assets[0].assetID = 19;
        frame->Assets[0].nRigidBodies = 1; frame->Assets[0].RigidBodyData = frame->RigidBodies;
        frame->Assets[0].nMarkers = 1; frame->Assets[0].MarkerData = frame->LabeledMarkers;
        frame->nForcePlates = 1; frame->ForcePlates[0].ID = 20; frame->ForcePlates[0].nChannels = 1;
        frame->ForcePlates[0].ChannelData[0].nFrames = 2; frame->ForcePlates[0].ChannelData[0].Values[1] = 3.25f;
        frame->nDevices = 1; frame->Devices[0].ID = 21; frame->Devices[0].nChannels = 1;
        frame->Devices[0].params = 3; frame->Devices[0].ChannelData[0].nFrames = 1;
        frame->Devices[0].ChannelData[0].Values[0] = 7.5f;
        frame->nIMU = 1; frame->IMU[0].ID = 22; frame->IMU[0].qx = 0.25f; frame->IMU[0].params = 5;
        frame->nGPIO = 1; frame->GPIO[0].ID = 23; frame->GPIO[0].numberOfGPIOPorts = 2; frame->GPIO[0].GPIO[1] = 65535;
        frame->Timecode = 1234; frame->TimecodeSubframe = 3; frame->params = 7;
        frame->CameraMidExposureTimestamp = 100; frame->CameraDataReceivedTimestamp = 110; frame->TransmitTimestamp = 120;
        frame->PrecisionTimestampSecs = 123; frame->PrecisionTimestampFractionalSecs = 456;
        frame->iFrame = 10; frame->fTimestamp = 0.1;
        recorder.capture(*frame, 42, 1000000000, 900000000);
        // Mutation immediately after capture must never leak into the owned copy.
        other[0][0] = 99; frame->RigidBodies[0].x = 8; frame->RigidBodies[0].params = 0;
        frame->RigidBodies[0].MeanError = 0.1f;
        frame->iFrame = 12; frame->fTimestamp = 0.12;
        recorder.capture(*frame, 42, 1020000000, 920000000);
        frame->nRigidBodies = 0; frame->iFrame = 13;
        recorder.capture(*frame, 42, 1030000000, 930000000);
        state(command(control, recorder.port(), "MTCREC1 test 3 STOP"), "OFF");
        state(command(control, recorder.port(), "MTCREC1 test 3 STOP"), "OFF");
        state(command(control, recorder.port(), "MTCREC1 test 2 START"), "OFF"); // delayed stale datagram
        state(command(control, recorder.port(), "MTCREC1 test 4 STATUS"), "OFF");
        recorder.capture(*frame, 42, 0, 0); // OFF must discard
        state(command(control, recorder.port(), "MTCREC1 test 5 START"), "ON");
        recorder.capture(*frame, 42, 0, 0);
        recorder.shutdown(); // active take must drain and finalize on graceful exit

        mocap_recording::Recorder overflow;
        require(overflow.initialize(root / "overflow", "{}", 0, 0), "Cannot bind overflow fixture");
        state(command(control, overflow.port(), "MTCREC1 overflow 1 START"), "ON");
        overflow.capture(*frame, 42, 0, 0);
        state(command(control, overflow.port(), "MTCREC1 overflow 2 STATUS"), "ERROR");
        overflow.shutdown();
        const auto blocked = root / "file_instead_of_directory";
        std::ofstream(blocked) << "fixture";
        mocap_recording::Recorder failure;
        require(failure.initialize(blocked, "{}", 0), "Cannot bind file error fixture");
        state(command(control, failure.port(), "MTCREC1 failure 1 START"), "ERROR");
        failure.shutdown();

        // Exercise the real receive callback while the recorder writes on its
        // worker: a paced 100 Hz, 1000-marker stream must stay complete.
        require(g_recorder.initialize(root / "paced", "{}", 0), "Cannot bind paced recorder");
        state(command(control, g_recorder.port(), "MTCREC1 paced 1 START"), "ON");
        auto many = std::make_unique<MarkerData[]>(1000);
        auto paced = std::make_unique<sFrameOfMocapData>();
        paced->nOtherMarkers = 1000; paced->OtherMarkers = many.get();
        g_tracks.clear(); g_maximumCallbackMilliseconds.store(0);
        const auto begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i)
        {
            for (int j = 0; j < 1000; ++j) { many[j][0] = j * 0.4f + i * 0.002f; many[j][2] = 1; }
            paced->iFrame = i;
            frameReceivedCallback(paced.get(), nullptr);
            std::this_thread::sleep_until(begin + std::chrono::milliseconds((i + 1) * 10));
        }
        state(command(control, g_recorder.port(), "MTCREC1 paced 2 STOP"), "OFF");
        g_recorder.shutdown();
        std::cout << "Recording enabled, synthetic 100 Hz / 1000 markers, largest callback: "
                  << g_maximumCallbackMilliseconds.load() << " ms\n";
        closesocket(control); WSACleanup();
        std::cout << "PASS: loopback start/stop, retries, stale commands, frame ownership, quality/missing poses, queue overflow, output failure, shutdown drain.\n";
        return 0;
    }
    catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
