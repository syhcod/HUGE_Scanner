// raw_still_get_img.cpp
// One-shot RAW Bayer still capture (2592x1944) from Raspberry Pi Camera v1.3 (OV5647)
// using libcamera C++ API, returning unsigned short** like get_img(exposure_us).
//
// Build:
//   sudo apt install -y libcamera-dev pkg-config
//   g++ -std=c++17 -O2 -Wall -Wextra raw_still_get_img.cpp -o rawcap \
//       $(pkg-config --cflags --libs libcamera) -pthread
//
// Run (test main enabled at bottom):
//   ./rawcap 30000
//
// Notes:
// - Returns Bayer mosaic (e.g., SGBRG10_CSI2P). This is the "2x2 pattern" mosaic, not an RGB image.
// - OV5647 is RAW10 (10-bit) -> returned values are 0..1023 stored in uint16.
// - Caller must free with free_img().

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <array>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// ---- Public API you asked for ----
static int g_w = 0;
static int g_h = 0;

int get_img_w() { return g_w; }
int get_img_h() { return g_h; }

void free_img(unsigned short **img) {
    if (!img) return;
    // rows share one contiguous allocation; row[0] points to base
    delete[] img[0];
    delete[] img;
}

// Unpack RAW10 CSI-2 packed row (4 pixels -> 5 bytes) into uint16 row.
static inline void unpack_raw10_csi2p_row(const uint8_t *rowPacked, int width, uint16_t *rowOut) {
    if (width % 4 != 0)
        throw std::runtime_error("RAW10 unpack requires width multiple of 4.");

    const int groups = width / 4;
    int x = 0;

    for (int g = 0; g < groups; ++g) {
        const uint8_t b0 = rowPacked[g * 5 + 0];
        const uint8_t b1 = rowPacked[g * 5 + 1];
        const uint8_t b2 = rowPacked[g * 5 + 2];
        const uint8_t b3 = rowPacked[g * 5 + 3];
        const uint8_t b4 = rowPacked[g * 5 + 4];

        rowOut[x++] = uint16_t(b0) | (uint16_t((b4 >> 0) & 0x03) << 8);
        rowOut[x++] = uint16_t(b1) | (uint16_t((b4 >> 2) & 0x03) << 8);
        rowOut[x++] = uint16_t(b2) | (uint16_t((b4 >> 4) & 0x03) << 8);
        rowOut[x++] = uint16_t(b3) | (uint16_t((b4 >> 6) & 0x03) << 8);
    }
}
class PersistentRawCapture {
public:
    PersistentRawCapture(int reqW = 2592, int reqH = 1944, int bufferCount = 6, int warmupFrames = 3)
        : reqW_(reqW), reqH_(reqH), bufferCount_(bufferCount), warmupFrames_(warmupFrames) {
        using namespace libcamera;

        if (cm_.start() != 0)
            throw std::runtime_error("CameraManager start() failed");
        if (cm_.cameras().empty())
            throw std::runtime_error("No cameras available");

        cam_ = cm_.cameras().front();
        if (cam_->acquire() != 0)
            throw std::runtime_error("Camera acquire() failed");

        cfg_ = cam_->generateConfiguration({ StreamRole::Raw });
        if (!cfg_)
            throw std::runtime_error("generateConfiguration(Raw) failed");

        libcamera::StreamConfiguration &rawCfg = cfg_->at(0);
        rawCfg.size.width  = reqW_;
        rawCfg.size.height = reqH_;
        rawCfg.pixelFormat = formats::SGBRG10_CSI2P;
        rawCfg.bufferCount = bufferCount_;

        if (cfg_->validate() == CameraConfiguration::Invalid)
            throw std::runtime_error("Invalid camera configuration");
        if (cam_->configure(cfg_.get()) != 0)
            throw std::runtime_error("Camera configure() failed");

        rawStream_ = rawCfg.stream();
        if (!rawStream_)
            throw std::runtime_error("raw stream null");

        // Read back what we actually got.
        W_ = rawCfg.size.width;
        H_ = rawCfg.size.height;
        strideBytes_ = rawCfg.stride;
        fmt_ = rawCfg.pixelFormat;

        std::cout << "RAW configured: "
                  << W_ << "x" << H_
                  << " stride=" << strideBytes_
                  << " fmt=" << fmt_.toString()
                  << " buffers=" << rawCfg.bufferCount
                  << "\n";

        if (W_ % 4 != 0)
            throw std::runtime_error("Width not multiple of 4 (RAW10 unpack assumes this).");

        const bool isRaw10 =
            (fmt_ == formats::SGBRG10_CSI2P) ||
            (fmt_ == formats::SRGGB10_CSI2P) ||
            (fmt_ == formats::SBGGR10_CSI2P) ||
            (fmt_ == formats::SGRBG10_CSI2P);
        if (!isRaw10)
            throw std::runtime_error("Unexpected pixel format (not RAW10 CSI2P).");

        // Allocate buffers once.
        alloc_ = std::make_unique<FrameBufferAllocator>(cam_);
        if (alloc_->allocate(rawStream_) < 0)
            throw std::runtime_error("FrameBuffer allocation failed");

        bufs_ = &alloc_->buffers(rawStream_);
        if (!bufs_ || bufs_->empty())
            throw std::runtime_error("No buffers from allocator");

        // Start camera once.
        cam_->requestCompleted.connect(this, &PersistentRawCapture::onComplete);
        if (cam_->start() != 0)
            throw std::runtime_error("Camera start failed");

        // Warm up: capture and discard a few frames so controls/pipe settle.
        for (int i = 0; i < warmupFrames_; ++i) {
            capture_into_internal(/*exposure_us=*/30000, /*gain=*/1.0f, /*discard=*/true);
        }

        // Update globals for your "function style"
        g_w = W_;
        g_h = H_;
    }

    ~PersistentRawCapture() {
        try {
            if (cam_) {
                cam_->requestCompleted.disconnect(this, &PersistentRawCapture::onComplete);
                cam_->stop();
                cam_->release();
                cam_.reset();
            }
            cm_.stop();
        } catch (...) {
            // destructor must not throw
        }
    }

    int width() const { return W_; }
    int height() const { return H_; }

    // Capture one RAW Bayer frame into a freshly allocated unsigned short**.
    // Returned values are RAW10 (0..1023) stored in uint16.
    unsigned short **capture_bayer_u16(int exposure_us, float gain = 1.0f) {
        // Allocate return image as unsigned short** with contiguous backing.
        unsigned short **rows = new unsigned short *[H_];
        unsigned short *base  = new unsigned short[size_t(W_) * size_t(H_)];
        for (int y = 0; y < H_; ++y) rows[y] = base + size_t(y) * size_t(W_);

        try {
            capture_into(rows, exposure_us, gain);
            return rows;
        } catch (...) {
            free_img(rows);
            throw;
        }
    }

private:
    // libcamera lifetime members
    libcamera::CameraManager cm_;
    std::shared_ptr<libcamera::Camera> cam_;
    std::unique_ptr<libcamera::CameraConfiguration> cfg_;
    libcamera::Stream *rawStream_ = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> alloc_;
    const std::vector<std::unique_ptr<libcamera::FrameBuffer>> *bufs_ = nullptr;

    // configured stream properties
    int reqW_ = 0, reqH_ = 0;
    int bufferCount_ = 0;
    int warmupFrames_ = 0;
    int W_ = 0, H_ = 0;
    int strideBytes_ = 0;
    libcamera::PixelFormat fmt_;

    // synchronization for one in-flight request
    std::mutex m_;
    std::condition_variable cv_;
    bool done_ = false;
    libcamera::Request::Status status_ = libcamera::Request::RequestCancelled;
    libcamera::FrameBuffer *completedFb_ = nullptr;

    // Choose a buffer index for the next capture. Single-flight means we can just alternate.
    size_t nextBuf_ = 0;

    void onComplete(libcamera::Request *req) {
        status_ = req->status();
        if (rawStream_) {
            auto it = req->buffers().find(rawStream_);
            if (it != req->buffers().end())
                completedFb_ = it->second;
        }
        {
            std::lock_guard<std::mutex> lk(m_);
            done_ = true;
        }
        cv_.notify_one();
    }

    void setControls(libcamera::Request *r, int exposure_us, float gain) {
        using namespace libcamera;
        r->controls().set(controls::AeEnable, false);
        r->controls().set(controls::ExposureTime, exposure_us);

        // Many pipelines clamp ExposureTime unless FrameDurationLimits is also set.
        std::array<int64_t, 2> frameDur = { (int64_t)exposure_us, (int64_t)exposure_us };
        r->controls().set(controls::FrameDurationLimits, frameDur);

        r->controls().set(controls::AnalogueGain, gain);
    }

    void capture_into(unsigned short **rows, int exposure_us, float gain) {
        // one request, one buffer, wait for completion, then mmap/unpack into rows
        capture_into_internal(exposure_us, gain, /*discard=*/false, rows);
    }

    void capture_into_internal(int exposure_us, float gain, bool discard, unsigned short **rowsOut = nullptr) {
        using namespace libcamera;

        if (!bufs_ || bufs_->empty())
            throw std::runtime_error("No buffers available");

        // Pick a buffer (single-flight: safe).
        FrameBuffer *fb = (*bufs_)[nextBuf_ % bufs_->size()].get();
        nextBuf_++;

        std::unique_ptr<Request> req = cam_->createRequest();
        if (!req)
            throw std::runtime_error("createRequest failed");

        if (req->addBuffer(rawStream_, fb) != 0)
            throw std::runtime_error("addBuffer failed");

        setControls(req.get(), exposure_us, gain);

        // Arm wait state
        {
            std::lock_guard<std::mutex> lk(m_);
            done_ = false;
            status_ = Request::RequestCancelled;
            completedFb_ = nullptr;
        }

        if (cam_->queueRequest(req.get()) != 0)
            throw std::runtime_error("queueRequest failed");

        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&]{ return done_; });
        }

        if (status_ != Request::RequestComplete)
            throw std::runtime_error("Request did not complete");
        if (!completedFb_)
            throw std::runtime_error("Request completed but framebuffer was not captured");

        if (discard)
            return;

        if (!rowsOut)
            throw std::runtime_error("rowsOut is null");

        // Map and unpack
        const auto &p0 = completedFb_->planes()[0];
        int fd = p0.fd.get();
        size_t mapLen = p0.length;

        void *map = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, 0);
        if (map == MAP_FAILED)
            throw std::runtime_error(std::string("mmap failed: ") + strerror(errno));

        const uint8_t *src8 = static_cast<const uint8_t *>(map);

        // RAW10 CSI2 packed (4 pixels -> 5 bytes)
        for (int y = 0; y < H_; ++y) {
            const uint8_t *rowPacked = src8 + size_t(y) * size_t(strideBytes_);
            uint16_t *rowOut = reinterpret_cast<uint16_t *>(rowsOut[y]);
            unpack_raw10_csi2p_row(rowPacked, W_, rowOut);
        }

        munmap(map, mapLen);
    }
};
// The function you asked for:
unsigned short **get_img(int microsec) {
    try {
        // Persistent camera instance: initialize once, reuse across calls.
        static PersistentRawCapture cap(/*reqW=*/2592, /*reqH=*/1944, /*buffers=*/6, /*warmupFrames=*/3);
        g_w = cap.width();
        g_h = cap.height();
        return cap.capture_bayer_u16(microsec, /*gain=*/1.0f);
    } catch (const std::exception &e) {
        std::cerr << "get_img ERROR: " << e.what() << "\n";
        return nullptr;
    }
}

#ifdef BUILD_TEST_MAIN
#include <cstdlib>
#include <fstream>

static std::string home_path(const char *leaf) {
    const char *home = std::getenv("HOME");
    if (!home || !*home) home = "/tmp";
    return std::string(home) + "/" + leaf;
}

int main(int argc, char **argv) {
    int us = (argc > 1) ? std::stoi(argv[1]) : 30000;

    unsigned short **img = get_img(us);
    if (!img) return 1;

    const int W = get_img_w();
    const int H = get_img_h();

    std::cout << "Captured " << W << "x" << H
              << " RAW Bayer (uint16 values, RAW10 valid) [persistent pipeline]\n";

    // Save to ~/test.raw (uint16 little-endian, row-major)
    const std::string outPath = home_path("test.raw");
    std::ofstream f(outPath, std::ios::binary);
    if (!f) {
        std::cerr << "Failed to open output file: " << outPath << "\n";
        free_img(img);
        return 1;
    }

    // We allocated rows with one contiguous block at img[0]
    f.write(reinterpret_cast<const char *>(img[0]),
            std::streamsize(size_t(W) * size_t(H) * sizeof(unsigned short)));

    f.close();
    std::cout << "Wrote: " << outPath << " ("
              << (size_t(W) * size_t(H) * 2) << " bytes)\n";
    std::cout << "Tip: For cleaner RAW, prefer longer exposure over high gain; adjust gain in get_img() if needed.\n";

    free_img(img);
    return 0;
}
#endif
