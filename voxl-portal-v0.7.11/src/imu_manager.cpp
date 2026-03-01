#include "imu_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>

#include <algorithm>

#include "page_manager.h"
#include "manager_utils.h"
#include "pipe_channels.h"

#ifdef __ANDROID__ // nal
#define IMU_NAME "android_imu"
#elif !defined(BUILD_QRB5165) // voxl
#define IMU_NAME "imu0"
#elif defined(BUILD_QRB5165) // voxl2
#define IMU_NAME "imu_apps"
#endif
#define TDK_NAME "imu_iim"

#define TDK_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR TDK_NAME "/")
#define IMU_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR IMU_NAME "/")

extern struct mg_mgr mgr_;

static websocket_info ws_info;

#ifdef BUILD_QRB5165 // voxl2
static websocket_info fft_ws_info;
#define IMU_FFT_NAME "imu_apps_fft"
#define IMU_FFT_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR IMU_FFT_NAME "/")
#endif

static void _connect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("connected to imu server ch %d\n", ch);
}

static void _disconnect_cb(int ch, __attribute__((unused)) void *context)
{
    printf("disconnected from imu server ch %d\n", ch);
}

static void ImuDataCallback(int ch, char *data, int bytes, __attribute__((unused)) void *context)
{
    
    if (!ws_info.connected){
        return;
    }
    int n_packets;
    imu_data_t *data_array = pipe_validate_imu_data_t(data, bytes, &n_packets);
    if (data_array == NULL){
        return;
    }
    struct mg_connection *c;

    // printf("received imu data on ch %d\n", ch);

    for (size_t i = 0; i < ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != ws_info.connection_ids[i])
                continue;
            // check if we are backing up the socket
            // if so, just drop the packet for this connection, but mark sent as true to prevent closure
            if (c->send.len != 0 || c->recv.len != 0)
            {
                // fprintf(stderr, "dropping imu packet\n");
                continue;
            }
            mg_ws_send(c, (char *)&data_array[n_packets - 1], sizeof(imu_data_t), WEBSOCKET_OP_BINARY);
        }
    }
}

void ImuManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{
    printf("hellow from imu calllback\n");
    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (ws_info.connection_ids.empty())
        {
            ws_info.connected = false;
            pipe_client_close(IMU_PIPE_CH);
            pipe_client_close(IMU_TDK_PIPE_CH);
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;
        const char *imu_location = IMU_LOCATION;
        int pipe_ch = IMU_PIPE_CH;          

        struct mg_http_message *hm = (struct mg_http_message *)ev_data;
        if (mg_http_match_uri(hm, "/imutdk")){
            fprintf(stderr, "setting imu-location to tdk iim\n"); 
            imu_location = TDK_LOCATION;
            pipe_ch = IMU_TDK_PIPE_CH;

        }

        pipe_client_set_connect_cb(pipe_ch, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(pipe_ch, _disconnect_cb, NULL);
        pipe_client_set_simple_helper_cb(pipe_ch, ImuDataCallback, c);
        int ret = pipe_client_open(pipe_ch, imu_location, PROCESS_NAME,
                                   EN_PIPE_CLIENT_SIMPLE_HELPER, IMU_RECOMMENDED_READ_BUF_SIZE);
        if (ret != 0){
            fprintf(stderr, "failed to connect to pipe \n");
        }
    }

    ws_info.connection_ids.push_back(c->id);
}

#ifdef BUILD_QRB5165 // voxl2

#define IMU_FFT_MAGIC_NUMBER  (0x564F584C + 101)

#define MAX_FFT_BUF_LEN 512
#define MAX_FFT_FREQ_BINS ((MAX_FFT_BUF_LEN/2)+1)

#define IMU_FFT_RECOMMENDED_READ_BUF_SIZE   (sizeof(imu_fft_data_t) * 5)

typedef struct imu_fft_data_t{
    uint32_t magic_number; ///< Set to IMU_FFT_MAGIC_NUMBER
    int n_freq;
    float max_freq_hz; // min frequency should always be 0

    float accl_ms2[3][MAX_FFT_FREQ_BINS];     ///< XYZ acceleration in m/s^2
    float gyro_rad[3][MAX_FFT_FREQ_BINS];     ///< XYZ gyro rotation in rad/s

} __attribute__((packed)) imu_fft_data_t;


static void FftDataCallback(__attribute__((unused)) int ch, char *data, int bytes, __attribute__((unused)) void *context)
{
    if (!fft_ws_info.connected)
        return;

    struct mg_connection *c;

    for (size_t i = 0; i < fft_ws_info.connection_ids.size(); i++)
    {
        for (c = mgr_.conns; c != NULL; c = c->next)
        {
            if (c->id != fft_ws_info.connection_ids[i])
                continue;
            // check if we are backing up the socket
            // if so, just drop the packet for this connection, but mark sent as true to prevent closure
            if (c->send.len != 0 || c->recv.len != 0)
            {
                // fprintf(stderr, "dropping fft packet\n");
                continue;
            }

            mg_ws_send(c, data, sizeof(imu_fft_data_t), WEBSOCKET_OP_BINARY);
        }
    }
}


void FftManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data)
{

    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        fft_ws_info.connection_ids.erase(std::remove(fft_ws_info.connection_ids.begin(), fft_ws_info.connection_ids.end(), c->id), fft_ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (fft_ws_info.connection_ids.empty())
        {
            fft_ws_info.connected = false;
            pipe_client_close(FFT_PIPE_CH);
        }
        return;
    }

    if (!fft_ws_info.connected)
    {
        fft_ws_info.connected = true;

        pipe_client_set_connect_cb(FFT_PIPE_CH, _connect_cb, NULL);
        pipe_client_set_disconnect_cb(FFT_PIPE_CH, _disconnect_cb, NULL);
        pipe_client_set_simple_helper_cb(FFT_PIPE_CH, FftDataCallback, c);
        int ret = pipe_client_open(FFT_PIPE_CH, IMU_FFT_LOCATION, PROCESS_NAME,
                                   EN_PIPE_CLIENT_SIMPLE_HELPER, IMU_FFT_RECOMMENDED_READ_BUF_SIZE);

    }

    fft_ws_info.connection_ids.push_back(c->id);
}
#endif
