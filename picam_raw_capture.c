// picam_raw_capture.c
// Capture absolute RAW Bayer (not debayered) from Raspberry Pi Camera Module v1.3
// using rpicam-raw, set exposure time with --shutter (microseconds), and return
// a 2D array (rows) of uint16 pixels via a char** API.
//
// Build:
//   gcc -O2 -Wall picam_raw_capture.c -o picam_raw_capture
// Run test:
//   sudo ./picam_raw_capture 30000
//
// Notes:
// - Requires rpicam-apps installed (rpicam-raw).
// - get_picture(microsec) returns char** where each row points to uint16_t pixels.
//   Cast like: uint16_t* row = (uint16_t*)img[y]; pixel = row[x];
// - Call free_picture(img) to free memory.

#define _GNU_SOURCE
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef CAP_W
#define CAP_W 640
#endif

#ifndef CAP_H
#define CAP_H 480
#endif

// Global metadata for the last captured frame (simple API, matches your signature)
static int g_width = 0;
static int g_height = 0;

// Forward decls
static int run_capture_command(int microsec, const char *out_path);
static uint16_t *load_and_unpack_raw(const char *path, int w, int h);
static uint16_t *unpack_raw10_packed(const uint8_t *src, size_t nbytes, int w, int h);
static uint16_t *load_raw16_unpacked_le(const uint8_t *src, size_t nbytes, int w, int h);

int get_picture_width(void)  { return g_width; }
int get_picture_height(void) { return g_height; }

void free_picture(char **rows) {
    if (!rows) return;
    // rows[0] points into a contiguous uint16 buffer we allocated
    free(rows[0]); // pixel buffer
    free(rows);    // row pointers
}

// Your requested API:
// Returns row pointers; each row is a pointer to uint16_t[width] but typed as char*.
char **get_picture(int microsec) {
    const int w = CAP_W;
    const int h = CAP_H;

    // Capture one raw frame into a temp file.
    // Use PID to avoid collisions.
    char out_path[256];
    snprintf(out_path, sizeof(out_path), "/tmp/picam_raw_%d_%%04d.raw", (int)getpid());

    if (run_capture_command(microsec, out_path) != 0) {
        fprintf(stderr, "Capture failed. Check camera connection and that rpicam-raw works.\n");
        return NULL;
    }

    uint16_t *pixels = load_and_unpack_raw(out_path, w, h);
    unlink(out_path);

    if (!pixels) return NULL;

    // Build row pointer table (char** as requested)
    char **rows = (char **)malloc((size_t)h * sizeof(char *));
    if (!rows) {
        perror("malloc rows");
        free(pixels);
        return NULL;
    }
    for (int y = 0; y < h; y++) {
        rows[y] = (char *)(pixels + (size_t)y * (size_t)w);
    }

    g_width = w;
    g_height = h;
    return rows;
}

static int run_capture_command(int microsec, const char *out_path) {
    // rpicam-raw captures raw frames (Bayer) without formatting the raw file.
    // We use --segment 1 so it writes exactly one frame (filename can be plain).
    //
    // Exposure time is set with --shutter <us>. If the framerate is too high to
    // allow the shutter time, the sensor will clamp exposure; lower framerate helps.
    //
    // Command rationale:
    // -t 200: run shortly
    // --frames 1: capture one frame (if supported); if not, segment 1 helps.
    // --segment 1: write one file per frame
    // --framerate 1: allows longer shutter times
    // --nopreview: avoid display
    //
    // Some builds may not support --frames; segment still works.
    // We'll attempt with --frames 1 first, and fall back if it fails.
    char cmd1[1024];
    snprintf(cmd1, sizeof(cmd1),
             "rpicam-raw -t 400 --frames 1 --segment 1 --nopreview "
             "--width %d --height %d --framerate 1 --shutter %d -o %s 1>/dev/null",
             CAP_W, CAP_H, microsec, out_path);

    int ret = system(cmd1);
    if (ret == 0) return 0;

    // Fallback (older versions)
    char cmd2[1024];
    snprintf(cmd2, sizeof(cmd2),
             "rpicam-raw -t 400 --segment 1 --nopreview "
             "--width %d --height %d --framerate 1 --shutter %d -o %s 1>/dev/null",
             CAP_W, CAP_H, microsec, out_path);

    ret = system(cmd2);
    if (ret == 0) return 0;

    // If on older OS, command might be libcamera-raw instead of rpicam-raw
    char cmd3[1024];
    snprintf(cmd3, sizeof(cmd3),
             "libcamera-raw -t 400 --segment 1 --nopreview "
             "--width %d --height %d --framerate 1 --shutter %d -o %s 1>/dev/null",
             CAP_W, CAP_H, microsec, out_path);

    ret = system(cmd3);
    return (ret == 0) ? 0 : -1;
}

static uint16_t *load_and_unpack_raw(const char *path, int w, int h) {
    struct stat st;
    if (stat(path, &st) != 0) {
        perror("stat raw file");
        return NULL;
    }
    size_t nbytes = (size_t)st.st_size;
    if (nbytes == 0) {
        fprintf(stderr, "Raw file is empty.\n");
        return NULL;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        perror("fopen raw");
        return NULL;
    }

    uint8_t *buf = (uint8_t *)malloc(nbytes);
    if (!buf) {
        perror("malloc raw buf");
        fclose(f);
        return NULL;
    }

    if (fread(buf, 1, nbytes, f) != nbytes) {
        perror("fread raw");
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);

    // Expected sizes for common raw Bayer outputs:
    // - RAW10 packed: w*h*10/8 bytes (must be integer; width typically multiple of 4)
    // - RAW16 unpacked: w*h*2 bytes
    const size_t expect_packed10 = ((size_t)w * (size_t)h * 10u) / 8u;
    const size_t expect_u16 = (size_t)w * (size_t)h * 2u;

    uint16_t *pixels = NULL;

    if (nbytes == expect_u16) {
        pixels = load_raw16_unpacked_le(buf, nbytes, w, h);
    } else if (nbytes == expect_packed10) {
        pixels = unpack_raw10_packed(buf, nbytes, w, h);
    } else {
        // Some modes include stride/padding. You can still support that,
        // but we need the stride value. For now, fail loudly with info.
        fprintf(stderr,
                "Unexpected raw file size: %zu bytes.\n"
                "Expected RAW10 packed: %zu bytes, or RAW16 unpacked: %zu bytes.\n"
                "Your camera mode may include stride/padding.\n"
                "Try changing CAP_W/CAP_H to a standard mode or share the rpicam-raw console output.\n",
                nbytes, expect_packed10, expect_u16);
        free(buf);
        return NULL;
    }

    free(buf);
    return pixels;
}

static uint16_t *load_raw16_unpacked_le(const uint8_t *src, size_t nbytes, int w, int h) {
    (void)nbytes;
    size_t npix = (size_t)w * (size_t)h;
    uint16_t *out = (uint16_t *)malloc(npix * sizeof(uint16_t));
    if (!out) { perror("malloc out"); return NULL; }

    // Little-endian 16-bit per pixel, typically with lower bits used (e.g., 10-bit data in 16 bits)
    for (size_t i = 0; i < npix; i++) {
        out[i] = (uint16_t)src[2*i] | ((uint16_t)src[2*i + 1] << 8);
    }
    return out;
}

// Unpack RAW10 packed format (4 pixels -> 5 bytes)
// Layout: B0..B3 = high 8 bits of 4 pixels; B4 contains low 2 bits for each pixel.
// This packing scheme is widely used for Pi camera RAW10 streams.
static uint16_t *unpack_raw10_packed(const uint8_t *src, size_t nbytes, int w, int h) {
    if (w % 4 != 0) {
        fprintf(stderr, "RAW10 packed unpacker requires width multiple of 4 (got %d)\n", w);
        return NULL;
    }

    size_t npix = (size_t)w * (size_t)h;
    uint16_t *out = (uint16_t *)malloc(npix * sizeof(uint16_t));
    if (!out) { perror("malloc out"); return NULL; }

    const size_t groups_per_row = (size_t)w / 4u;
    const size_t bytes_per_row = groups_per_row * 5u;
    const size_t expected = bytes_per_row * (size_t)h;

    if (nbytes != expected) {
        fprintf(stderr, "RAW10 size mismatch: got %zu, expected %zu\n", nbytes, expected);
        free(out);
        return NULL;
    }

    size_t out_idx = 0;
    for (int y = 0; y < h; y++) {
        const uint8_t *row = src + (size_t)y * bytes_per_row;
        for (size_t g = 0; g < groups_per_row; g++) {
            const uint8_t b0 = row[g*5 + 0];
            const uint8_t b1 = row[g*5 + 1];
            const uint8_t b2 = row[g*5 + 2];
            const uint8_t b3 = row[g*5 + 3];
            const uint8_t b4 = row[g*5 + 4];

            // Low 2 bits for each pixel are packed into b4:
            // p0 lowbits = b4[1:0], p1 = b4[3:2], p2 = b4[5:4], p3 = b4[7:6]
            out[out_idx++] = ((uint16_t)b0 << 2) | ((b4 >> 0) & 0x03);
            out[out_idx++] = ((uint16_t)b1 << 2) | ((b4 >> 2) & 0x03);
            out[out_idx++] = ((uint16_t)b2 << 2) | ((b4 >> 4) & 0x03);
            out[out_idx++] = ((uint16_t)b3 << 2) | ((b4 >> 6) & 0x03);
        }
    }
    return out;
}

#ifdef TEST_MAIN
int main(int argc, char **argv) {
    int us = 30000;
    if (argc >= 2) us = atoi(argv[1]);

    char **img = get_picture(us);
    if (!img) return 1;

    int w = get_picture_width();
    int h = get_picture_height();
    printf("Captured raw %dx%d, exposure=%d us\n", w, h, us);

    // Print a tiny 8x4 sample (as numbers) from top-left
    for (int y = 0; y < 4 && y < h; y++) {
        uint16_t *row = (uint16_t *)img[y];
        for (int x = 0; x < 8 && x < w; x++) {
            printf("%4u ", row[x]);
        }
        printf("\n");
    }

    free_picture(img);
    return 0;
}
#endif
