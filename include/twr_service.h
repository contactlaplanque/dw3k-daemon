#ifndef TWR_SERVICE_H
#define TWR_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    TWR_ROLE_INITIATOR = 0,
    TWR_ROLE_RESPONDER = 1
} twr_role_t;

typedef struct {
    uint16_t pan_id;
    uint16_t local_id;
    uint16_t peer_id;
    uint16_t antenna_delay;
} twr_service_config_t;

typedef struct {
    uint32_t attempts;
    uint32_t successes;
    size_t raw_count;
    size_t filtered_count;
    size_t rejected_count;
    double raw_mean;
    double raw_stddev;
    double raw_min;
    double raw_max;
    double raw_median;
    double filtered_mean;
    double filtered_stddev;
    double filtered_min;
    double filtered_max;
    double filtered_median;
} twr_measurement_stats_t;

void twr_service_set_config(const twr_service_config_t *cfg);
void twr_service_get_config(twr_service_config_t *cfg);
void twr_service_set_logging(bool enable);
void twr_service_prepare_frames(twr_role_t role);

int twr_service_run_initiator(uint32_t num_measurements, twr_measurement_stats_t *stats);
int twr_service_run_responder(uint32_t expected_polls);

#endif


