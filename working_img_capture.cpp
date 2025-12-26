#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <map>
#include <vector>
#include <cstring>
#include <mutex>
#include <condition_variable>

#include <errno.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>
#include <sys/ioctl.h>

#include <libcamera/libcamera.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace libcamera;

static std::shared_ptr<Camera> camera;

// stream + geometry for callback
static Stream *gStream = nullptr;
static int gW = 0, gH = 0, gStride = 0;

// one-shot sync + storage
static std::mutex gMtx;
static std::condition_variable gCv;
static bool gotImage = false;
static int discardLeft = 8;                 // discard first N frames
static std::vector<uint8_t> capturedBgr;    // size = gStride * gH (includes padding)

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    // Find OUR stream buffer (don’t assume buffers.begin()).
    auto it = request->buffers().find(gStream);
    if (it == request->buffers().end()) {
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
        return;
    }

    // Discard initial frames to let exposure/AWB settle.
    if (discardLeft > 0) {
        discardLeft--;
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
        return;
    }

    {   // already captured?
        std::lock_guard<std::mutex> lk(gMtx);
        if (gotImage)
            return;
    }

    FrameBuffer *buffer = it->second;
    if (buffer->planes().empty()) {
        std::cerr << "No planes\n";
        return;
    }

    const auto &p0 = buffer->planes()[0];
    int fd = p0.fd.get();
    size_t mapLen = p0.length;

    void *map = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
        std::cerr << "mmap failed: " << std::strerror(errno) << "\n";
        return;
    }

    // ---- DMA-BUF cache sync START (READ) ----
    dma_buf_sync s{};
    s.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &s) != 0) {
        std::cerr << "DMA_BUF_IOCTL_SYNC START failed: " << std::strerror(errno) << "\n";
        munmap(map, mapLen);
        return;
    }
    // ----------------------------------------

    const size_t frameBytes = (size_t)gStride * (size_t)gH;
    const uint8_t *src = static_cast<const uint8_t *>(map);

    {
        std::lock_guard<std::mutex> lk(gMtx);
        capturedBgr.assign(src, src + frameBytes); // FAST: just copy bytes
        gotImage = true;
    }

    // ---- DMA-BUF cache sync END (READ) ----
    s.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &s) != 0) {
        std::cerr << "DMA_BUF_IOCTL_SYNC END failed: " << std::strerror(errno) << "\n";
    }
    // --------------------------------------

    munmap(map, mapLen);

    gCv.notify_one();

    // DO NOT requeue after we captured one still.
}

int main()
{
    auto cm = std::make_unique<CameraManager>();
    cm->start();

    if (cm->cameras().empty()) {
        std::cout << "No cameras were identified on the system.\n";
        cm->stop();
        return EXIT_FAILURE;
    }

    camera = cm->cameras().front();
    if (camera->acquire() != 0) {
        std::cerr << "Failed to acquire camera\n";
        cm->stop();
        return EXIT_FAILURE;
    }

    auto config = camera->generateConfiguration({ StreamRole::StillCapture });
    StreamConfiguration &streamConfig = config->at(0);

    // You can request BGR888, but validate() may adjust.
    streamConfig.pixelFormat = formats::BGR888;
    streamConfig.size.width  = 2592;
    streamConfig.size.height = 1944;
    streamConfig.bufferCount = 4;

    auto status = config->validate();
    if (status == CameraConfiguration::Invalid) {
        std::cerr << "Camera Configuration is invalid\n";
        return EXIT_FAILURE;
    }

    if (camera->configure(config.get()) != 0) {
        std::cerr << "configure failed\n";
        return EXIT_FAILURE;
    }

    std::cout << "ACTUAL: " << streamConfig.toString() << "\n";

    gW = (int)streamConfig.size.width;
    gH = (int)streamConfig.size.height;
    gStride = (int)streamConfig.stride;

    Stream *stream = streamConfig.stream();
    gStream = stream;

    auto allocator = std::make_unique<FrameBufferAllocator>(camera);
    for (StreamConfiguration &cfg : *config) {
        if (allocator->allocate(cfg.stream()) < 0) {
            std::cerr << "Can't allocate buffers\n";
            return EXIT_FAILURE;
        }
    }

    const auto &buffers = allocator->buffers(stream);
    if (buffers.empty()) {
        std::cerr << "No buffers\n";
        return EXIT_FAILURE;
    }

    // Build one Request per buffer and queue them all (important!)
    std::vector<std::unique_ptr<Request>> requests;
    requests.reserve(buffers.size());
    for (size_t i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> req = camera->createRequest();
        if (!req) {
            std::cerr << "Can't create request\n";
            return EXIT_FAILURE;
        }
        if (req->addBuffer(stream, buffers[i].get()) < 0) {
            std::cerr << "Can't add buffer to request\n";
            return EXIT_FAILURE;
        }
        requests.push_back(std::move(req));
    }

    // Reset capture state
    {
        std::lock_guard<std::mutex> lk(gMtx);
        gotImage = false;
        capturedBgr.clear();
    }
    discardLeft = 8;

    camera->requestCompleted.connect(requestComplete);

    if (camera->start() != 0) {
        std::cerr << "camera start failed\n";
        return EXIT_FAILURE;
    }

    for (auto &req : requests)
        camera->queueRequest(req.get());

    // Wait for one good frame
    {
        std::unique_lock<std::mutex> lk(gMtx);
        gCv.wait(lk, [] { return gotImage; });
    }

    camera->requestCompleted.disconnect(requestComplete);
    camera->stop();

    // Convert captured BGR(stride) -> packed RGB and write PNG
    if (capturedBgr.size() < (size_t)gStride * (size_t)gH) {
        std::cerr << "Captured buffer too small\n";
        return EXIT_FAILURE;
    }

    std::vector<uint8_t> rgb((size_t)gW * (size_t)gH * 3);
    for (int y = 0; y < gH; ++y) {
        const uint8_t *srcRow = capturedBgr.data() + (size_t)y * (size_t)gStride;
        uint8_t *dstRow = rgb.data() + (size_t)y * (size_t)gW * 3;
        for (int x = 0; x < gW; ++x) {
            uint8_t B = srcRow[3 * x + 0];
            uint8_t G = srcRow[3 * x + 1];
            uint8_t R = srcRow[3 * x + 2];
            dstRow[3 * x + 0] = R;
            dstRow[3 * x + 1] = G;
            dstRow[3 * x + 2] = B;
        }
    }

    if (!stbi_write_png("/home/kde/test.png", gW, gH, 3, rgb.data(), gW * 3))
        std::cerr << "Failed to write PNG\n";
    else
        std::cout << "Saved PNG: /home/kde/test.png (" << gW << "x" << gH << ")\n";

    // cleanup
    allocator.reset();
    camera->release();
    camera.reset();
    cm->stop();
    return 0;
}
