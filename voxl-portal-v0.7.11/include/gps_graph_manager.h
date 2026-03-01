#ifndef GPS_GRAPH_MANAGER_H
#define GPS_GRAPH_MANAGER_H

#include <stdint.h>
#include <fcntl.h> 
#include <sys/stat.h>
#include <sys/wait.h>
#include <fstream>

#include <algorithm>
#include <vector>

#define GpsGraphClaimString "/gps"

typedef struct satellite_info {
    uint8_t msg_index;
    uint8_t satellite_prn[20];
    uint8_t satellite_snr[20];
} __attribute__((packed)) satellite_info;

void GpsGraphManagerCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);

void send_satellite_info(uint8_t* satellite_prn, uint8_t* satellite_snr);

#endif