// Exercise the real callback and sender without connecting to a capture server.
#define main motiveClientMainForTests
#include "../motive_client.cpp"
#undef main
#include <memory>
#include <stdexcept>
#include <streambuf>

namespace
{
    struct ConsoleGuard : std::streambuf
    {
        std::size_t writes = 0;
        std::streamsize xsputn(const char*, std::streamsize count) override { ++writes; return count; }
        int overflow(int character) override { ++writes; return character; }
    };
    void require(bool condition, const char* message)
    {
        if (!condition) throw std::runtime_error(message);
    }
}

int main()
{
    ConsoleGuard console;
    auto* originalOut = std::cout.rdbuf(&console);
    auto* originalError = std::cerr.rdbuf(&console);
    double maximumMs = 0;
    try
    {
        for (int count : {10, 100, 1000})
        {
            g_tracks.clear();
            g_nextTrackID = 0;
            g_maximumCallbackMilliseconds.store(0);
            auto markers = std::make_unique<MarkerData[]>(count);
            // NatNet frames contain large fixed arrays; keep fixtures off the stack.
            auto frame = std::make_unique<sFrameOfMocapData>();
            frame->nOtherMarkers = count;
            frame->OtherMarkers = markers.get();
            for (int step = 0; step < 100; step++)
            {
                for (int i = 0; i < count; i++)
                {
                    markers[i][0] = (i % 40) * 0.4f + step * 0.002f;
                    markers[i][1] = (i / 40) * 0.4f;
                    markers[i][2] = 1.0f;
                }
                frame->iFrame = step;
                frameReceivedCallback(frame.get(), nullptr);
            }
            require(g_rawPointCount.load() == static_cast<std::size_t>(count), "Raw point count changed");
            require(g_activePointCount.load() == static_cast<std::size_t>(count), "Moving markers lost activation");
            require(g_tracks.size() == static_cast<std::size_t>(count), "Stable marker IDs were lost");
            maximumMs = std::max(maximumMs, g_maximumCallbackMilliseconds.load());
            require(g_cameraAgeMilliseconds.load() == -1, "Unavailable camera timing must not report zero latency");
            require(g_transitMilliseconds.load() == -1, "Unavailable transit timing must not report zero latency");
        }
        // Exercise error reporting too: even a failed UDP send must not print.
        g_unitySocket = static_cast<SOCKET>(123456789);
        auto errors = g_sendErrors.load();
        sendFrameToUnity(1, {}, RigidBodyPose{});
        g_unitySocket = INVALID_SOCKET;
        require(g_sendErrors.load() == errors + 1, "Failed UDP sends must be counted");
        auto timedFrame = std::make_unique<sFrameOfMocapData>();
        timedFrame->CameraDataReceivedTimestamp = 100;
        timedFrame->TransmitTimestamp = 125;
        g_serverClockFrequency.store(1000);
        frameReceivedCallback(timedFrame.get(), nullptr);
        require(g_softwareMilliseconds.load() == 25.0, "Motive software timing conversion is wrong");
        require(console.writes == 0, "Receive callback/sender performed console I/O");
        std::cout.rdbuf(originalOut);
        std::cerr.rdbuf(originalError);
        std::cout << "PASS: zero console writes with 10/100/1000 markers, stable track IDs and activation, "
                  << "UDP error counters, unavailable timing handling, software timing conversion. "
                  << "Largest callback time in this synthetic run: " << maximumMs << " ms\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        g_unitySocket = INVALID_SOCKET;
        std::cout.rdbuf(originalOut);
        std::cerr.rdbuf(originalError);
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
