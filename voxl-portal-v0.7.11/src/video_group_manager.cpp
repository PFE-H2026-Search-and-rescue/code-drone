#include "video_group_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>
#include <turbojpeg.h>

#include <algorithm>
#include <vector>
#include <mutex>

#include "cmd_group_manager.h"
#include "manager_utils.h"
#include "page_manager.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <mongoose.h>
#include <map>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define THROW(action, message)                                                 \
    {                                                                          \
        printf("ERROR in line %d while %s:\n%s\n", __LINE__, action, message); \
    }

#define THROW_TJ(action) THROW(action, tjGetErrorStr2(tjInstance))

#define THROW_UNIX(action) THROW(action, strerror(errno))

#define min(x, y) ((x) > (y) ? (y) : (x))

#define DEFAULT_SUBSAMP TJSAMP_GRAY

#define TIMEOUT_THRES_NS 5000000000L 

#define DEBUG_RATE_NS 1000000000L

extern struct mg_mgr mgr_;

static uint64_t running_count = 0;
static bool compressed_image = true;
std::map<std::string, int> cams_map_single;
std::map<std::string, int> cams_map_multiview;
std::
int64_t h264_connection_id = 0;
int64_t snapshot_connection_id = 0;
int64_t always_listening_ws_id = -1;
int64_t established_h264_callback_id = 0;
static bool multiview;

//Delete here 
#define NUM_TIME_SAMPLES 20
#define NUM_QUALITY_STEPS 32

// Static jpg quality for now
#define JPG_QUALITY 70
int jpg_quality = JPG_QUALITY;


typedef struct VideoStats
{
    char name[128];
    float fps_sending;
    float fps_actual;
    uint8_t quality = JPG_QUALITY;
    char server_name[128] = "unknown";
    float exposure_ms;
    int16_t gain;
    int16_t format;
} __attribute__((packed)) VideoStats;

// Simple LL to keep track of running cameras
// also stores quality for that specific cam
typedef struct _cam_data
{
    int64_t ts = -1;
    VideoStats video_stats;
    int64_t times[NUM_TIME_SAMPLES] = {0};
    bool skipped[NUM_TIME_SAMPLES] = {0};
    int time_index = 0;
    uint32_t frame_num = 0;
    bool is_h264 = false;
    bool send_data = true;
} _cam_data;

static _cam_data cams[PIPE_CLIENT_MAX_CHANNELS];

static websocket_info ws_info;

std::vector<std::mutex> vid_stats_lock(PIPE_CLIENT_MAX_CHANNELS);
std::mutex websocket_lock;

void _remove_cam_from_list(int ch)
{
    memset(cams[ch].video_stats.name, 0, sizeof(cams[ch].video_stats.name));
    cams[ch].ts = -1;
    cams[ch].video_stats.quality = 0;
    cams[ch].frame_num = 0;
}

static int _get_cam_channel(const char *name)
{
    for (int i = 0; i < PIPE_CLIENT_MAX_CHANNELS; i++)
    {
        if (cams[i].ts != -1 && !strcmp(cams[i].video_stats.name, name)){
            //printf("get cam channel %d %s\n", i, name);
            return i;
        }
    }
    return -1;
}

static pthread_mutex_t yuv_buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
static unsigned char *yuv_buffer;
static unsigned int yuv_buffer_size = 0;

// Converts a nv12 image to YUV420
static void _cvt_nv_yuv(unsigned char *frame, int width, int height, int isNV12)
{
    int i, j;
    int y_size = width * height;

    pthread_mutex_lock(&yuv_buffer_mutex);

    if (yuv_buffer_size < y_size / 2) {
        if (yuv_buffer_size == 0) {
            yuv_buffer = (unsigned char *)malloc(y_size/2);
        } else {
            yuv_buffer = (unsigned char *)realloc(yuv_buffer, y_size/2);
        }
        yuv_buffer_size = y_size / 2;
    }

    if (!yuv_buffer) {
        printf("ERROR: Failed to malloc YUV buffer for resampling\n");
        pthread_mutex_unlock(&yuv_buffer_mutex);
        exit(-1);
    }

    memcpy(yuv_buffer, frame + y_size, (y_size / 2));

    unsigned char *u = frame + y_size;
    unsigned char *v = frame + (y_size * 5 / 4);

    for (int i = 0; i < y_size / 4; i++)
    {
        if (isNV12)
        { // nv12
            u[i] = yuv_buffer[(i * 2)];
            v[i] = yuv_buffer[(i * 2) + 1];
        }
        else
        { // nv21
            u[i] = yuv_buffer[(i * 2) + 1];
            v[i] = yuv_buffer[(i * 2)];
        }
    }

    pthread_mutex_unlock(&yuv_buffer_mutex);
}

static void _make_422_planar(unsigned char *frame, int width, int height)
{
    int i, j;
    int size = width * height * 2;

    pthread_mutex_lock(&yuv_buffer_mutex);

    if (yuv_buffer_size < size) {
        if (yuv_buffer_size == 0) {
            yuv_buffer = (unsigned char *)malloc(size);
        } else {
            yuv_buffer = (unsigned char *)realloc(yuv_buffer, size);
        }
        yuv_buffer_size = size;
    }

    if (!yuv_buffer) {
        printf("ERROR: Failed to malloc YUV buffer for resampling\n");
        pthread_mutex_unlock(&yuv_buffer_mutex);
        exit(-1);
    }

    memcpy(yuv_buffer, frame, size);

    unsigned char *u = frame + (size / 2);
    unsigned char *v = frame + (size * 3 / 4);

    for (int i = 0; i < size / 4; i++)
    {
        frame[i*2]     = yuv_buffer[i*4];
        frame[i*2 + 1] = yuv_buffer[i*4 + 2];
        u[i]           = yuv_buffer[i*4 + 1];
        v[i]           = yuv_buffer[i*4 + 3];
    }

    pthread_mutex_unlock(&yuv_buffer_mutex);
}

static int _raw16_to_raw8(camera_image_metadata_t meta, uint8_t* frame)
{
    int n = meta.width*meta.height;
    if(meta.size_bytes!=(n*2)){
        fprintf(stderr, "ERROR, received RAW16 camera frame with %d bytes, expected %d\n", meta.size_bytes, n*2);
        return -1;
    }

    // very dumb, just grab the most significant 8 bits
    uint16_t* frame16 = (uint16_t*)frame;
    for(int i=0; i<n; i++){
        frame[i] = frame16[i]>>8;
    }
    return 0;
}

static __attribute__((destructor)) void cleanupbuffer()
{
    if (yuv_buffer) {
        free(yuv_buffer);
        yuv_buffer = nullptr;
    } 
}

// Calculate the average framerate from the last NUM_TIME_SAMPLES frames
static void calc_avg_fr_hz(_cam_data &cam)
{
    int64_t max_full_time = -1;
    int64_t min_full_time = 0x7FFFFFFFFFFFFFFF;

    int64_t full_interval;

    uint8_t skip_count = 0;

    // find min and max times
    for (int i = 0; i < NUM_TIME_SAMPLES; i++)
    {
        if (cam.skipped[i]){
            skip_count++;
        }
        if (cam.times[i] < min_full_time){
            min_full_time = cam.times[i];
        }
        if (cam.times[i] > max_full_time)
        {
            max_full_time = cam.times[i];
        }
    }

    // calculate duration
    full_interval = max_full_time - min_full_time;

    // number of frames / number of seconds
    cam.video_stats.fps_actual =  (NUM_TIME_SAMPLES - 1) / (double)(full_interval / 1000000000.0);
    cam.video_stats.fps_sending = (NUM_TIME_SAMPLES - skip_count - 1) / (double)(full_interval / 1000000000.0);
}

#define FMT_INVALID 0
#define FMT_STANDARD 1
#define FMT_YUV 2
#define FMT_NV 33
#define FMT_STEREO_NV 4
static int tj_fmt_from_mpa(camera_image_metadata_t meta,
                           int *infmt, int *outfmt)
{
    switch (meta.format)
    {
    case IMAGE_FORMAT_RAW8:
    case IMAGE_FORMAT_STEREO_RAW8:
        *infmt = TJPF_GRAY;
        *outfmt = TJSAMP_GRAY;
        return FMT_STANDARD;

    case IMAGE_FORMAT_RGB:
        *infmt = TJPF_RGB;
        *outfmt = TJSAMP_422;
        return FMT_STANDARD;

    case IMAGE_FORMAT_YUV420:
        *outfmt = TJSAMP_420;
        return FMT_YUV;

    case IMAGE_FORMAT_YUV422:
        *outfmt = TJSAMP_422;
        return FMT_YUV;

    case IMAGE_FORMAT_NV21:
    case IMAGE_FORMAT_NV12:
        *outfmt = TJSAMP_420;
        return FMT_NV;

    case IMAGE_FORMAT_STEREO_NV12:
    case IMAGE_FORMAT_STEREO_NV21:
        *outfmt = TJSAMP_420;
        return FMT_STEREO_NV;

    case IMAGE_FORMAT_H264:
        return IMAGE_FORMAT_H264;
    /*
    case IMAGE_FORMAT_JPG :
    case IMAGE_FORMAT_FLOAT32 :
    case IMAGE_FORMAT_RAW16 :
    case IMAGE_FORMAT_H264 :
    case IMAGE_FORMAT_H265 :
    case IMAGE_FORMAT_JPG :*/
    default:
        return FMT_INVALID; // unsupported
    }
}

bool send_vid_stats()
{
    std::lock_guard<std::mutex> lg(websocket_lock);
    static int64_t last_call_t_ns = VCU_time_monotonic_ns();

    if (VCU_time_monotonic_ns() - last_call_t_ns < DEBUG_RATE_NS) return false;

    char *vid_stats_ptr = nullptr;

    if (!ws_info.connected) {
        return false;
    }

    int16_t cam_count = 0;
    for (int i = 0; i < PIPE_CLIENT_MAX_CHANNELS; i++)
    {
        if (cams[i].ts != -1)
            cam_count++;
    }

    if (cam_count <= 0)
        return false;

    vid_stats_ptr = (char *)malloc(sizeof(VideoStats) * cam_count + sizeof(int16_t));
    memcpy(vid_stats_ptr, &cam_count, sizeof(int16_t));
    int offset = sizeof(int16_t);
    for (int i = 0; i < PIPE_CLIENT_MAX_CHANNELS; i++)
    {
        if (cams[i].ts != -1)
        {
            vid_stats_lock[i].lock();
            memcpy(vid_stats_ptr + offset, &(cams[i].video_stats), sizeof(VideoStats));
            offset += sizeof(VideoStats);
            vid_stats_lock[i].unlock();
        } 
    }

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next)
    {
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id)))
            continue;

        if (c->send.len > offset/2)
        {
            //fprintf(stderr, "dropping vid stats packet\n");
            continue;
        }
        mg_ws_send(c, vid_stats_ptr, offset, WEBSOCKET_OP_BINARY);
    }

    free(vid_stats_ptr);
    vid_stats_ptr = nullptr;

    return true;
}

void H264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        h264_connection_id = c->id;
    }
}

static void SnapshotCameraCallback(int ch, camera_image_metadata_t meta, char *frame, void *context) {
    cams[ch].video_stats.quality = -1;
    cams[ch].skipped[cams[ch].time_index] = false;
    cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
    cams[ch].time_index %= NUM_TIME_SAMPLES;
    cams[ch].video_stats.format = meta.format;
    cams[ch].video_stats.gain = meta.gain;
    cams[ch].video_stats.format = meta.format;
    cams[ch].video_stats.exposure_ms = meta.exposure_ns/1000000.;

    vid_stats_lock[ch].lock(); 
    calc_avg_fr_hz(cams[ch]);
    vid_stats_lock[ch].unlock();

    static pipe_info_t cam_info;
    if (strcmp(cams[ch].video_stats.server_name, cam_info.server_name))
    {
        pipe_client_get_info(ch, &cam_info);
        memcpy(cams[ch].video_stats.server_name, &(cam_info.server_name), sizeof(cams[ch].video_stats.server_name));
    }

    int64_t curTime = VCU_time_monotonic_ns();

    unsigned long jpegSize = 0;
    unsigned char *jpegBuf = nullptr;
    int width = meta.width;
    int height = meta.height;
    
    jpegSize = meta.size_bytes;
    jpegBuf = (unsigned char*) frame;

    cams[ch].frame_num++;

    for (struct mg_connection *c = mgr_.conns; c != NULL; c = c->next) {
        if (strcmp(c->label, cams[ch].video_stats.name) != 0)
        {
            continue;
        }
        char header[256];
        char footer[256];
        int header_len;
        header_len = snprintf(header, sizeof(header),
                            "--foo\r\nContent-Type: image/jpeg\r\n"
                            "Content-Length: %lu\r\n\r\n",
                            (unsigned long)jpegSize);

        char *message = (char *)malloc(header_len + jpegSize);
            
        memcpy(message, header, header_len);
        memcpy(message + header_len, jpegBuf, jpegSize);
        
        mg_ws_send(c, message, header_len + jpegSize, WEBSOCKET_OP_BINARY);
        mg_ws_send(c, message, header_len + jpegSize, WEBSOCKET_OP_BINARY);
        
        free(message);
        message = nullptr;

        bool did_send = send_vid_stats();
        while (did_send == false) {
            did_send = send_vid_stats();
        }

        cams[ch].ts = curTime;
    }

    return;
}

static void H264CameraDataCallback(int ch, camera_image_metadata_t meta, char*frame, void *context) {
    
    cams[ch].video_stats.quality = -1;
    cams[ch].skipped[cams[ch].time_index] = false;
    cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
    cams[ch].time_index %= NUM_TIME_SAMPLES;
    cams[ch].video_stats.format = meta.format;
    cams[ch].video_stats.gain = meta.gain;
    cams[ch].video_stats.format = meta.format;
    cams[ch].video_stats.exposure_ms = meta.exposure_ns/1000000.;

    vid_stats_lock[ch].lock(); 
    calc_avg_fr_hz(cams[ch]);
    vid_stats_lock[ch].unlock();

    int64_t curTime = VCU_time_monotonic_ns();

    static pipe_info_t cam_info;
    if (strcmp(cams[ch].video_stats.server_name, cam_info.server_name))
    {
        pipe_client_get_info(ch, &cam_info);
        memcpy(cams[ch].video_stats.server_name, &(cam_info.server_name), sizeof(cams[ch].video_stats.server_name));
    }

    unsigned long jpegSize = 0;
    unsigned char *jpegBuf = nullptr;
    int width = meta.width;
    int height = meta.height;
    
    jpegSize = meta.size_bytes;
    jpegBuf = (unsigned char*) frame;

    cams[ch].frame_num++;

    /*
        It's the mg_ws_send function call which is triggering this segfault, but why?
        Use a for loop to go through all active connections rather than relying on the H264_Call mg_connection struct
    */
    
    char header[256];
    char footer[256];
    int header_len;
    
    header_len = snprintf(header, sizeof(header),
                        "--foo\r\nContent-Type: image/h264\r\n"
                        "Content-Length: %lu\r\n\r\n",
                        (unsigned long)jpegSize);

    char *message = (char *)malloc(header_len + jpegSize);

    
    memcpy(message, header, header_len);
    memcpy(message + header_len, jpegBuf, jpegSize);
    
    for (struct mg_connection *c = mgr_.conns; c != NULL; c = c->next)
    {
        if (c->id != h264_connection_id) {
            continue;
        }

        mg_ws_send(c, message, header_len + jpegSize, WEBSOCKET_OP_BINARY);

        free(message);
        message = nullptr;

        cams[ch].ts = curTime;
        //printf("sent h264 %d bytes\n",header_len + jpegSize);
    }

    if (running_count % 10 == 0){
        send_vid_stats();
    }

    running_count++;
    return;
}

static void CameraDataCallback(int ch, camera_image_metadata_t meta, char* frame, void *context)
{
    //printf("+CameraDataCallback, %d bytes\n",meta.size_bytes);

    if (meta.format == IMAGE_FORMAT_H264) {
        //do not drop if this is SPS or PPS packet because SPS is often sent together with the first I frame during start
        if (pipe_client_bytes_in_pipe(ch) && (frame[4] != 0x67) && (frame[4] != 0x68))
        {
            cams[ch].skipped[cams[ch].time_index] = true;
            cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
            cams[ch].time_index %= NUM_TIME_SAMPLES;
            cams[ch].video_stats.quality = jpg_quality;
            cams[ch].ts = meta.timestamp_ns;
            // fprintf(stderr, "dropping h264 frame\n");
            switch (frame[4]) {
                case 0x67 : //Header
                    //printf("| H264 (head)");
                    break;
                case 0x65 : // I Frame
                    //printf("| H264 (I)   ");
                    break;
                case 0x41 : // P Frame
                    //printf("\n| H264 (P)   ");
                    cams[ch].send_data = false;
                    break;
                default   : // TODO test to see what a b frame is
                    //printf("| H264       ");
                    break;
            }
            return;
        }

        switch (frame[4]) {
            case 0x67 : //Header
                //printf("| H264 (head)");
                cams[ch].send_data = true;
                break;
            case 0x65 : // I Frame
                //printf("| H264 (I)   ");
                cams[ch].send_data = true;
                break;
            case 0x41 : // P Frame
                //printf("\n| H264 (P)   ");
                break;
            default   : // TODO test to see what a b frame is
                //printf("| H264       ");
                break;
        }

        //cams[ch].send_data = true;
        
        cams[ch].video_stats.quality = jpg_quality;
        cams[ch].skipped[cams[ch].time_index] = false;
        cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
        cams[ch].time_index %= NUM_TIME_SAMPLES;
        cams[ch].video_stats.format = meta.format;
        cams[ch].video_stats.gain = meta.gain;
        cams[ch].video_stats.format = meta.format;
        cams[ch].video_stats.exposure_ms = meta.exposure_ns/1000000.;

        vid_stats_lock[ch].lock(); 
        calc_avg_fr_hz(cams[ch]);
        vid_stats_lock[ch].unlock();

        int64_t curTime = VCU_time_monotonic_ns();

        static pipe_info_t cam_info;
        if (strcmp(cams[ch].video_stats.server_name, cam_info.server_name))
        {
            pipe_client_get_info(ch, &cam_info);
            memcpy(cams[ch].video_stats.server_name, &(cam_info.server_name), sizeof(cams[ch].video_stats.server_name));
        }

        unsigned long jpegSize = 0;
        unsigned char *jpegBuf = nullptr;
        int width = meta.width;
        int height = meta.height;
        
        jpegSize = meta.size_bytes;
        jpegBuf = (unsigned char*) frame;

        cams[ch].frame_num++;

        for (struct mg_connection *c = mgr_.conns; c != NULL; c = c->next) {
            if (strcmp(c->label, "check_h264_stream") != 0) {
                continue;
            }

            mg_ws_send(c, "activate", 8, WEBSOCKET_OP_TEXT);
        }

        bool published = false;
        

        for (struct mg_connection *c = mgr_.conns; c != NULL; c = c->next) {
            if (strcmp(c->label, cams[ch].video_stats.name) != 0)
            {
                continue;
            }

            char header[256];
            char footer[256];
            int header_len;
            header_len = snprintf(header, sizeof(header),
                                "--foo\r\nContent-Type: video/h264\r\n"
                                "Content-Length: %lu\r\n\r\n",
                                (unsigned long)jpegSize);

            char *message = (char *)malloc((header_len + jpegSize) * 2);

            
            memcpy(message, header, header_len);
            memcpy(message + header_len, jpegBuf, jpegSize);
            if (cams[ch].send_data == true) {
               send((int64_t)c->fd, message, header_len + jpegSize, 0); 
               //printf("CameraDataCallback actually sent data\n");
            }
            
            free(message);
            message = nullptr;

            cams[ch].ts = curTime;

            //printf("CameraDataCallback sent h264 %d bytes to %s, %p\n",header_len + jpegSize, c->label, c);
            published = true;
        }

        jpegBuf = nullptr;

        if (running_count % 10 == 0){
            send_vid_stats();
        }
        
        running_count++;
        //pipe_client_flush(ch);

        if (!published){
            int channel_to_close = _get_cam_channel(cams[ch].video_stats.name);
            if (channel_to_close != -1) {
                int64_t current_time = VCU_time_monotonic_ns();
                //fprintf(stderr, "\n !!! Ceasing to publish %s at %ld\n", cams[ch].video_stats.name, current_time);
                _remove_cam_from_list(channel_to_close);
                pipe_client_close(channel_to_close); 
            }
        }
        
    }
    else {
        // We can't process the frames fast enough, the pipe is getting backed up, skip a frame
        if (pipe_client_bytes_in_pipe(ch))
        {
            cams[ch].skipped[cams[ch].time_index] = true;
            cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
            cams[ch].time_index %= NUM_TIME_SAMPLES;
            cams[ch].video_stats.quality = jpg_quality;
            cams[ch].ts = meta.timestamp_ns;
            //printf("flushing pipe ch %d %s\n", ch, cams[ch].video_stats.name);
            pipe_client_flush(ch);
            return;
        }
        //printf("received cam data ch %d %s\n", ch, cams[ch].video_stats.name);

        cams[ch].video_stats.quality = jpg_quality;
        cams[ch].skipped[cams[ch].time_index] = false;
        cams[ch].times[cams[ch].time_index++] = meta.timestamp_ns;
        cams[ch].time_index %= NUM_TIME_SAMPLES;
        cams[ch].video_stats.format = meta.format;
        cams[ch].video_stats.gain = meta.gain;
        cams[ch].video_stats.format = meta.format;
        cams[ch].video_stats.exposure_ms = meta.exposure_ns/1000000.;

        vid_stats_lock[ch].lock(); 
        calc_avg_fr_hz(cams[ch]);
        vid_stats_lock[ch].unlock();

        static pipe_info_t cam_info;
        if (strcmp(cams[ch].video_stats.server_name, cam_info.server_name))
        {
            pipe_client_get_info(ch, &cam_info);
            memcpy(cams[ch].video_stats.server_name, &(cam_info.server_name), sizeof(cams[ch].video_stats.server_name));
        }

        int64_t curTime = VCU_time_monotonic_ns();
        
        

        if (meta.format == IMAGE_FORMAT_H264) {
            
        }
        else if (ch != -1 && curTime - cams[ch].ts > TIMEOUT_THRES_NS)
        {
            fprintf(stderr, "\nCeasing to publish ch %d %s at %ld\n", ch, cams[ch].video_stats.name, curTime);
            _remove_cam_from_list(ch);
            pipe_client_close(ch);
            return;
        }
        
        /* BEGIN JPEG COMPRESS */
        tjhandle tjInstance = NULL;
        unsigned long jpegSize = 0;
        unsigned char *jpegBuf = nullptr; /* Dynamically allocate the JPEG buffer */
        int pixelFormat;
        int outSubsamp;
        int outQual = cams[ch].video_stats.quality;
        int flags = 0;
        // speed up flags
        flags |= TJFLAG_FASTUPSAMPLE;
        flags |= TJFLAG_FASTDCT;
        int width = meta.width;
        int height = meta.height * (meta.format == IMAGE_FORMAT_STEREO_RAW8 ? 2 : 1);

        // handle 16-bit images, just reduce to 8-bit
        if(meta.format == IMAGE_FORMAT_RAW16){
            if(_raw16_to_raw8(meta, (uint8_t*)frame)){
                return;
            }
            meta.format = IMAGE_FORMAT_RAW8;
            meta.size_bytes/=2;
        }

        switch (tj_fmt_from_mpa(meta, &pixelFormat, &outSubsamp))
        {
        case FMT_STANDARD:

            compressed_image = true;
            if ((tjInstance = tjInitCompress()) == NULL)
                THROW_TJ("initializing compressor");

            if (tjCompress2(tjInstance, (uint8_t *)frame, width, 0, height, pixelFormat,
                            &jpegBuf, &jpegSize, outSubsamp, outQual, flags) < 0)
                THROW_TJ("compressing image");

            tjDestroy(tjInstance);
            tjInstance = NULL;
            break;

        case FMT_STEREO_NV:
        {
            compressed_image = true;
            // special case: have two stacked nv frames
            // need to move all the color channels to the end of the image n1, v1, n2, v2
            // grab the nv channels of first image
            unsigned char *nv_mover = (unsigned char *)malloc(width * height / 2);
            memcpy(nv_mover, frame + (width * height), width * height / 2);

            // now that we have them, we can overwrite that section of the image with the next full black/white frame
            memcpy(frame + (width * height), frame + (width * height * 3 / 2), width * height);

            // throw frame 1 nv channels at the end of image 2
            memcpy(frame + (width * height * 2), nv_mover, width * height / 2);

            free(nv_mover);
            nv_mover = nullptr;

            _cvt_nv_yuv((unsigned char *)frame, width, height * 2, meta.format == IMAGE_FORMAT_STEREO_NV12);

            // good to go now with the double nv frame?
            if ((tjInstance = tjInitCompress()) == NULL)
                THROW_TJ("initializing compressor");

            if (tjCompressFromYUV(tjInstance, (uint8_t *)frame, width, 1, height * 2, outSubsamp,
                                &jpegBuf, &jpegSize, outQual, flags) < 0)
                THROW_TJ("compressing image");

            tjDestroy(tjInstance);
            tjInstance = NULL;
        }
        break;

        case FMT_NV:
            compressed_image = true;
            _cvt_nv_yuv((unsigned char *)frame, width, height, meta.format == IMAGE_FORMAT_NV12);

        case FMT_YUV:

            if ((tjInstance = tjInitCompress()) == NULL)
                THROW_TJ("initializing compressor");

            if (outSubsamp == TJSAMP_422) _make_422_planar((uint8_t *)frame, width, height);

            if (tjCompressFromYUV(tjInstance, (uint8_t *)frame, width, 1, height, outSubsamp,
                                &jpegBuf, &jpegSize, outQual, flags) < 0)
                THROW_TJ("compressing image");

            tjDestroy(tjInstance);
            compressed_image = true;
            tjInstance = NULL;

            break;

        case IMAGE_FORMAT_H264:
            jpegBuf = (unsigned char*) frame;
            jpegSize = meta.size_bytes;
            break;

        case FMT_INVALID:
        default:
            return;
        }

        if (jpegBuf == nullptr) {
            return;
        }

        cams[ch].frame_num++;
        int i = 0;
        for (struct mg_connection *c = mgr_.conns; c != NULL; c = c->next)
            {
            if (strcmp(c->label, cams[ch].video_stats.name) != 0)
            {
                continue;
            }
            // This can be the start of quality and framerate control
            // There is some funny business with memory management in
            // the mg_send function. Seems to be trying to minimize
            // memory usage which is definitely not an issue here.
            // The following code writes to the socket directly
            // and seems to be significantly more stable.
            char header[256];
            char footer[256];
            int header_len;
            // no footer needed due to initial multi-part def
            header_len = snprintf(header, sizeof(header),
                                "--foo\r\nContent-Type: image/jpeg\r\n"
                                "Content-Length: %lu\r\n\r\n",
                                (unsigned long)jpegSize);

            char *message = (char *)malloc(header_len + jpegSize);

            
            memcpy(message, header, header_len);
            memcpy(message + header_len, jpegBuf, jpegSize);

            //Identify where char* frame is sent
            int sent = send((int64_t)c->fd, message, header_len + jpegSize, 0);

            free(message);
            message = nullptr;

            cams[ch].ts = curTime;
        }
        if (compressed_image) {
            tjFree(jpegBuf);
        }
        jpegBuf = nullptr;

        if (running_count % 10 == 0){
            send_vid_stats();
        }

        running_count++;
    }

    return;
}

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: %s connected\n", cams[ch].video_stats.name);
    // Reset timeout timer since we've reconnected to the server
    cams[ch].ts = VCU_time_monotonic_ns();
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("Server providing: %s disconnected\n", cams[ch].video_stats.name);
}

void VideoGroupManagerCallback(struct mg_connection *c, int ev,
                               void *ev_data, void *fn_data)
{
    printf("VideoGroupManagerCallback\n");
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;

    // Chop off the /video/ of the uri to get the camera name
    const char *tmp = &(hm->uri.ptr[strlen(VideoGroupManagerClaimString) - 1]);

    // And remove all of the https garbage, we just want the pipe name
    int len = min(strchr(tmp, ' ') - tmp, strchr(tmp, '/') - tmp);

    char in_name[64];

    // Copy name in and terminate the string
    strncpy(in_name, tmp, len);
    in_name[len] = 0;

    int j = 0;
    for (int i = 0; in_name[i] != 0; i++)
    {
        if (valid_char(in_name[i]))
            in_name[j++] = in_name[i];
    }
    in_name[j] = 0;

    int ch = _get_cam_channel(in_name);

    const char* check_h264 = "h264";
    const char* check_snapshot = "snapshot";

    if (ch == -1)
    {
        printf("\nReceived Video request for: %s, %p\n", in_name, c);
        char pipe_name[256];
        ch = pipe_client_get_next_available_channel();

        if (ch == -1)
            return;

        strcpy(cams[ch].video_stats.name, in_name);
        strcpy(pipe_name, in_name);
        cams[ch].ts = VCU_time_monotonic_ns();

        pipe_client_set_connect_cb(ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(ch, _disconnect_cb, NULL);
        if (strstr(in_name, check_h264) != nullptr) {
            pipe_client_set_camera_helper_cb(ch, H264CameraDataCallback, NULL);
            printf("\nrequest is h264: %s\n", in_name);
        }
        else if (strstr(in_name, check_snapshot) != nullptr) {
            pipe_client_set_camera_helper_cb(ch, SnapshotCameraCallback, NULL);
        }
        else {
            pipe_client_set_camera_helper_cb(ch, CameraDataCallback, NULL);
            printf("\nrequest is regular callback: %s\n", in_name);
        }
        if (int ret = pipe_client_open(ch, pipe_name, PROCESS_NAME,
                                       CLIENT_FLAG_EN_CAMERA_HELPER, 0))
        {
            pipe_print_error(ret);
            fprintf(stderr, "\n\nFailed to open pipe: %s\n\n\n", in_name);
            _remove_cam_from_list(ch);
        }
        else{
            printf("opened ch %d %s\n", ch, in_name);
        }
    }

    printf("\n !! Received Video request for: %s, %p\n", in_name, c);

    strcpy(c->label, in_name);

    mg_printf(
        c, "%s",
        (char *)"HTTP/1.0 200 OK\r\n"
                "Cache-Control: no-cache\r\n"
                "Pragma: no-cache\r\nExpires: Thu, 01 Dec 1994 16:00:00 GMT\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=--foo\r\n\r\n");
}

void VideoStatsManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());
    }
    else
    {
        ws_info.connection_ids.push_back(c->id);
        //printf("recieved video stats callback\n");
        ws_info.connected = true;
    }
}

void SnapshotManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) 
{
    printf("SnapshotManagerCallback\n");
    //Check that http message was received
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        //printf("%s\n",hm->body);
        std::string url(hm->uri.ptr, hm->uri.len);
        printf("url: %s\n",url.c_str());

        char param_id_buffer[128];
        char param_val_buffer[128];
        float param_val;

        int param_id_length = mg_http_get_var(&hm->query, "param_id", param_id_buffer, sizeof(param_id_buffer));

        if (mg_http_get_var(&hm->query, "param_val", param_val_buffer, 20)) {
            sscanf(param_val_buffer, "%f", &param_val);
            param_val /= 100.0;
            printf("zoom val %f\n",param_val);

            std::string cmd = std::string("voxl-send-command hires_misp_color set_misp_zoom ") + std::to_string(param_val);

            FILE *fp = popen(cmd.c_str(), "r");

            if (fp == NULL)
            {
                printf("Failed to run command\n");
            }
            else{
                char response[512];
                char *current_pos = response;
                int len;

                while (fgets(current_pos, sizeof(response), fp))
                {
                    int len = strlen(current_pos);
                    current_pos[len - 1] = ' ';
                    current_pos = &current_pos[len];
                }

                *current_pos = 0;
                pclose(fp);
            }
        }


        std::string cam_name = url.substr(10);
        int channel_to_send_command = _get_cam_channel(cam_name.c_str());  
        pipe_client_send_control_cmd(channel_to_send_command, "snapshot\n");
        if (strcmp(cam_name.c_str(), "hires_snapshot") == 0) {
            snapshot_connection_id = c->id;
        }
    }                       
}

void SnapshotCheckCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        std::string url(hm->uri.ptr, hm->uri.len);
        std::string total(hm->query.ptr, hm->query.len);
        std::string cam_name = url.substr(7);
        // check string is /var/run/mpa/cam_name/info

        // This is the adb command that will be executed in the adb shell by our system 
        std::string adb_command = "cat /var/run/mpa/" + cam_name + "/info | grep -o 'snapshot'";

        // Open a pipe to execute the ADB shell command and capture the output
        FILE* pipe = popen(adb_command.c_str(), "r");
        if (!pipe) {
            std::cerr << "Failed to run command." << std::endl;
            return;
        }

        // Read the output of the command
        char buffer[128];
        memset(buffer, 0, sizeof(buffer));
        fgets(buffer, sizeof(buffer), pipe);

        // Close the pipe
        pclose(pipe);

        // Check if the output contains the word
        bool word_found = strlen(buffer) > 0;

        if (word_found == 1) {
            mg_ws_send(c, "FOUND", 5, WEBSOCKET_OP_TEXT);
        }else {
            mg_ws_send(c, "NOT FOUND", 9, WEBSOCKET_OP_TEXT);
        }
    }
    return;
}


// I don't think this is used anymore ??
void CloseH264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        int channel = _get_cam_channel("hires_small_h264");
        if (channel != -1) {
            int64_t current_time = VCU_time_monotonic_ns();
            fprintf(stderr, "\nCeasing to publish h264 %s at %ld\n", "hires_small_h264", current_time);
            _remove_cam_from_list(channel);
            pipe_client_close(channel); 
        }
    }
}

void CloseSnapshotCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        int channel = _get_cam_channel("hires_snapshot");
        if (channel != -1) {
            int64_t current_time = VCU_time_monotonic_ns();
            fprintf(stderr, "\nCeasing to publish snapshot %s at %ld\n", "hires_snapshot", current_time);
            _remove_cam_from_list(channel);
            pipe_client_close(channel); 
        }
    }
}

void VIOCloseCameraCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        std::string url(hm->uri.ptr, hm->uri.len);
        std::string total(hm->query.ptr, hm->query.len);
        std::string cam_name = url.substr(11);
        int channel_to_close = _get_cam_channel(cam_name.c_str());
        if (channel_to_close != -1) {
            int64_t current_time = VCU_time_monotonic_ns();
            fprintf(stderr, "\nCeasing to publish vio cam ch %d %s at %ld\n", channel_to_close, cam_name.c_str(), current_time);
            _remove_cam_from_list(channel_to_close);
            pipe_client_close(channel_to_close); 
        }
    }

}

void CheckCameraH264Callback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
       strcpy(c->label, "check_h264_stream");  
    }
    if (ev == MG_EV_WS_MSG) {
        int chan = _get_cam_channel((char*) ev_data);
        if (chan != -1) {
            cams[chan].is_h264 = false;
            int64_t cur_time = VCU_time_monotonic_ns();
            fprintf(stderr, "\nCeasing to publish h264  %s at %ld\n", (char*) ev_data, cur_time);
            _remove_cam_from_list(chan);
            pipe_client_close(chan);
        }
    }
}

void CloseDownLastSingleCamCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        std::string url(hm->uri.ptr, hm->uri.len);
        std::string total(hm->query.ptr, hm->query.len);
        std::string cam_name = url.substr(19);
        int channel_to_close = _get_cam_channel(cam_name.c_str());
        if (channel_to_close != -1) {
            int64_t current_time = VCU_time_monotonic_ns();
            fprintf(stderr, "\nCeasing to publish single cam ch %d %s at %ld\n", channel_to_close, cam_name.c_str(), current_time);
            _remove_cam_from_list(channel_to_close);
            pipe_client_close(channel_to_close); 
        }
    }
}


