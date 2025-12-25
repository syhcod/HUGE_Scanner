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
class OneShotRawCapture {
public:
    unsigned short **capture_bayer_u16(int exposure_us) {
        using namespace libcamera;

        CameraManager cm;
        if (cm.start() != 0) throw std::runtime_error("CameraManager start() failed");
        if (cm.cameras().empty()) throw std::runtime_error("No cameras available");

        std::shared_ptr<Camera> cam = cm.cameras().front();
        if (cam->acquire() != 0) throw std::runtime_error("Camera acquire() failed");

        std::unique_ptr<CameraConfiguration> cfg =
            cam->generateConfiguration({ StreamRole::Raw });
        if (!cfg) throw std::runtime_error("generateConfiguration(Raw) failed");

        StreamConfiguration &rawCfg = cfg->at(0);

        // Request full-res still
        rawCfg.size.width  = 2592;
        rawCfg.size.height = 1944;

        // Prefer RAW10 packed Bayer. Pipeline may choose a different Bayer order;
        // but it should remain RAW10 CSI2P for OV5647.
        rawCfg.pixelFormat = formats::SGBRG10_CSI2P;
        rawCfg.bufferCount = 1;

        if (cfg->validate() == CameraConfiguration::Invalid)
            throw std::runtime_error("Invalid camera configuration");

        if (cam->configure(cfg.get()) != 0)
            throw std::runtime_error("Camera configure() failed");
        std::cout << "RAW configured: "
                  << rawCfg.size.width << "x" << rawCfg.size.height
                  << " stride=" << rawCfg.stride
                  << " fmt=" << rawCfg.pixelFormat.toString()
                  << "\n";
        Stream *rawStream = rawCfg.stream();
        if (!rawStream) throw std::runtime_error("raw stream null");

        const int W = rawCfg.size.width;
        const int H = rawCfg.size.height;
        const int strideBytes = rawCfg.stride;
        const PixelFormat fmt = rawCfg.pixelFormat;

        // Update globals for your "function style"
        g_w = W;
        g_h = H;

        if (W % 4 != 0)
            throw std::runtime_error("Width not multiple of 4 (RAW10 unpack assumes this).");

        const bool isRaw10 =
            (fmt == formats::SGBRG10_CSI2P) ||
            (fmt == formats::SRGGB10_CSI2P) ||
            (fmt == formats::SBGGR10_CSI2P) ||
            (fmt == formats::SGRBG10_CSI2P);

        if (!isRaw10)
            throw std::runtime_error("Unexpected pixel format (not RAW10 CSI2P).");

        // Allocate your return image as unsigned short** with contiguous backing.
        unsigned short **rows = new unsigned short *[H];
        unsigned short *base  = new unsigned short[size_t(W) * size_t(H)];
        for (int y = 0; y < H; ++y) rows[y] = base + size_t(y) * size_t(W);

        // Ensure all libcamera-owned objects die before cm.stop()
        {
            FrameBufferAllocator alloc(cam);
            if (alloc.allocate(rawStream) < 0)
                throw std::runtime_error("FrameBuffer allocation failed");

            const auto &bufs = alloc.buffers(rawStream);
            if (bufs.empty())
                throw std::runtime_error("No buffers from allocator");

            FrameBuffer *fb = bufs[0].get();

            std::unique_ptr<Request> req = cam->createRequest();
            if (!req) throw std::runtime_error("createRequest failed");

            if (req->addBuffer(rawStream, fb) != 0)
                throw std::runtime_error("addBuffer failed");

            // Manual exposure
            req->controls().set(controls::AeEnable, false);
            req->controls().set(controls::ExposureTime, exposure_us);

            // If you want fixed gain too, uncomment:
            // req->controls().set(controls::AnalogueGain, 1.0f);

            done_ = false;
            status_ = Request::RequestCancelled;

            cam->requestCompleted.connect(this, &OneShotRawCapture::onComplete);

            if (cam->start() != 0)
                throw std::runtime_error("Camera start failed");

            if (cam->queueRequest(req.get()) != 0) {
                cam->requestCompleted.disconnect(this, &OneShotRawCapture::onComplete);
                cam->stop();
                throw std::runtime_error("queueRequest failed");
            }

            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [&]{ return done_.load(); });
            }

            cam->requestCompleted.disconnect(this, &OneShotRawCapture::onComplete);
            cam->stop();

            if (status_ != Request::RequestComplete)
                throw std::runtime_error("Request did not complete");

            // Map and unpack
            const auto &p0 = fb->planes()[0];
            int fd = p0.fd.get();
            size_t mapLen = p0.length;

            void *map = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, 0);
            if (map == MAP_FAILED)
                throw std::runtime_error(std::string("mmap failed: ") + strerror(errno));

            const uint8_t *src8 = static_cast<const uint8_t *>(map);

            // CASE 1: RAW10 CSI2 packed (4 pixels -> 5 bytes)
            const bool isCSI2P =
                (fmt == formats::SGBRG10_CSI2P) ||
                (fmt == formats::SRGGB10_CSI2P) ||
                (fmt == formats::SBGGR10_CSI2P) ||
                (fmt == formats::SGRBG10_CSI2P);

            if (isCSI2P) {
                for (int y = 0; y < H; ++y) {
                    const uint8_t *rowPacked = src8 + size_t(y) * size_t(strideBytes);
                    uint16_t *rowOut = reinterpret_cast<uint16_t *>(rows[y]);
                    unpack_raw10_csi2p_row(rowPacked, W, rowOut);
                }
            } else {
                // CASE 2: Unpacked RAW10 in 16-bit words (common at full-res modes)
                // Each pixel is uint16 in memory; valid range is still 0..1023.
                for (int y = 0; y < H; ++y) {
                    const uint16_t *row16 = reinterpret_cast<const uint16_t *>(
                        src8 + size_t(y) * size_t(strideBytes)
                    );
                    uint16_t *rowOut = reinterpret_cast<uint16_t *>(rows[y]);

                    for (int x = 0; x < W; ++x) {
                        // Some pipelines store RAW10 either:
                        // - in the low 10 bits, or
                        // - left-shifted (e.g., bits 15..6).
                        // We'll detect by checking magnitude:
                        uint16_t v = row16[x];

                        // Heuristic: if values look like multiples of 64 and mostly > 1023, shift down.
                        if (v > 1023) v >>= 6;

                        rowOut[x] = v & 0x03FF;
                    }
                }
            }

            munmap(map, mapLen);
        }

        cam->release();
        cam.reset();
        cm.stop();

        return rows;
    }

private:
    void onComplete(libcamera::Request *req) {
        status_ = req->status();
        {
            std::lock_guard<std::mutex> lk(m_);
            done_.store(true);
        }
        cv_.notify_one();
    }

    std::mutex m_;
    std::condition_variable cv_;
    std::atomic<bool> done_{false};
    libcamera::Request::Status status_{libcamera::Request::RequestCancelled};
};

// The function you asked for:
unsigned short **get_img(int microsec) {
    try {
        OneShotRawCapture cap;
        return cap.capture_bayer_u16(microsec);
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
              << " RAW Bayer (uint16 values, RAW10 valid)\n";

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

    free_img(img);
    return 0;
}
#endif
