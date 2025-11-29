#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "deca_device_api.h"
#include "init.h"
#include "twr_service.h"

#define DEFAULT_SOCKET_PATH "/var/run/dw3k-daemon.sock"
#define DEFAULT_CONFIG_PATH "/etc/dw3k-daemon.conf"
#define DEFAULT_MONITOR_INTERVAL_MS 1000U
#define DEFAULT_PAN_ID 0xDECAu
#define DEFAULT_NODE_ID 0x1000u
#define DEFAULT_ANT_DELAY 16385u
#define MAX_REQUEST_SIZE 4096

typedef struct {
    pthread_mutex_t lock;
    pthread_mutex_t hw_mutex;
    bool chip_active;
    bool monitor_should_exit;
    bool monitor_running;
    bool hw_busy;
    bool last_health_ok;
    bool config_dirty;
    bool have_last_stats;
    bool foreground;
    uint32_t monitor_interval_ms;
    int server_fd;
    pthread_t monitor_thread;
    time_t last_measurement_time;
    char socket_path[PATH_MAX];
    char config_path[PATH_MAX];
    char last_health_msg[128];
    twr_service_config_t cfg;
    twr_measurement_stats_t last_stats;
} daemon_state_t;

static volatile sig_atomic_t g_should_terminate = 0;
static bool g_foreground_mode = false;

static void log_message(int priority, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    if (g_foreground_mode) {
        va_list copy;
        va_copy(copy, args);
        FILE *stream = (priority <= LOG_INFO) ? stdout : stderr;
        vfprintf(stream, fmt, copy);
        fputc('\n', stream);
        va_end(copy);
    }
    vsyslog(priority, fmt, args);
    va_end(args);
}

static void signal_handler(int signo) {
    (void)signo;
    g_should_terminate = 1;
}

static void daemon_state_init(daemon_state_t *state) {
    memset(state, 0, sizeof(*state));
    pthread_mutex_init(&state->lock, NULL);
    pthread_mutex_init(&state->hw_mutex, NULL);
    state->server_fd = -1;
    state->monitor_interval_ms = DEFAULT_MONITOR_INTERVAL_MS;
    state->cfg.pan_id = DEFAULT_PAN_ID;
    state->cfg.local_id = DEFAULT_NODE_ID;
    state->cfg.peer_id = 0;
    state->cfg.antenna_delay = DEFAULT_ANT_DELAY;
    state->config_dirty = true;
    snprintf(state->socket_path, sizeof(state->socket_path), "%s", DEFAULT_SOCKET_PATH);
    snprintf(state->config_path, sizeof(state->config_path), "%s", DEFAULT_CONFIG_PATH);
    snprintf(state->last_health_msg, sizeof(state->last_health_msg), "inactive");
    twr_service_set_config(&state->cfg);
    twr_service_set_logging(false);
}

static int mkdir_p(const char *path, mode_t mode) {
    if (!path || path[0] == '\0') {
        return -1;
    }
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof(tmp), "%s", path);
    size_t len = strlen(tmp);
    if (len == 0) return -1;
    if (tmp[len - 1] == '/') tmp[len - 1] = '\0';
    for (char *p = tmp + 1; *p; ++p) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(tmp, mode) != 0 && errno != EEXIST) {
                return -1;
            }
            *p = '/';
        }
    }
    if (mkdir(tmp, mode) != 0 && errno != EEXIST) {
        return -1;
    }
    return 0;
}

static int ensure_parent_directory(const char *path) {
    char dir_buf[PATH_MAX];
    snprintf(dir_buf, sizeof(dir_buf), "%s", path);
    char *slash = strrchr(dir_buf, '/');
    if (!slash) {
        return 0;
    }
    if (slash == dir_buf) {
        slash[1] = '\0';
    } else {
        *slash = '\0';
    }
    if (dir_buf[0] == '\0') {
        return 0;
    }
    return mkdir_p(dir_buf, 0775);
}

static int create_server_socket(const char *path) {
    if (ensure_parent_directory(path) != 0) {
        log_message(LOG_ERR, "Failed to ensure socket directory for %s: %s", path, strerror(errno));
        return -1;
    }

    int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        log_message(LOG_ERR, "socket() failed: %s", strerror(errno));
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (strlen(path) >= sizeof(addr.sun_path)) {
        log_message(LOG_ERR, "Socket path too long: %s", path);
        close(fd);
        return -1;
    }
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    unlink(path);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        log_message(LOG_ERR, "bind() failed: %s", strerror(errno));
        close(fd);
        return -1;
    }

    if (listen(fd, 4) != 0) {
        log_message(LOG_ERR, "listen() failed: %s", strerror(errno));
        close(fd);
        unlink(path);
        return -1;
    }

    return fd;
}

static int ensure_config_file(daemon_state_t *state) {
    if (access(state->config_path, F_OK) == 0) {
        return 0;
    }
    if (ensure_parent_directory(state->config_path) != 0) {
        log_message(LOG_ERR, "Failed to create config directory for %s: %s", state->config_path, strerror(errno));
        return -1;
    }
    FILE *fp = fopen(state->config_path, "w");
    if (!fp) {
        log_message(LOG_ERR, "Failed to create config file %s: %s", state->config_path, strerror(errno));
        return -1;
    }
    fprintf(fp,
            "# dw3k-daemon configuration\n"
            "# Values accept decimal or hex numbers (e.g. 0x1234)\n"
            "pan_id=0x%04X\n"
            "node_id=0x%04X\n"
            "antenna_delay=%u\n"
            "monitor_interval_ms=%u\n"
            "socket_path=%s\n",
            state->cfg.pan_id,
            state->cfg.local_id,
            state->cfg.antenna_delay,
            state->monitor_interval_ms,
            state->socket_path);
    fclose(fp);
    chmod(state->config_path, 0644);
    log_message(LOG_INFO, "Wrote default config to %s", state->config_path);
    return 0;
}

static const char *json_find_value(const char *json, const char *key) {
    char pattern[128];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *pos = strstr(json, pattern);
    if (!pos) return NULL;
    pos += strlen(pattern);
    while (*pos && isspace((unsigned char)*pos)) pos++;
    if (*pos != ':') return NULL;
    pos++;
    while (*pos && isspace((unsigned char)*pos)) pos++;
    return pos;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t out_len) {
    const char *pos = json_find_value(json, key);
    if (!pos || *pos != '"') return false;
    pos++;
    size_t i = 0;
    while (*pos && *pos != '"' && i + 1 < out_len) {
        out[i++] = *pos++;
    }
    if (*pos != '"') return false;
    out[i] = '\0';
    return true;
}

static bool json_get_uint(const char *json, const char *key, uint32_t *out_value) {
    const char *pos = json_find_value(json, key);
    if (!pos) return false;
    errno = 0;
    char *end = NULL;
    unsigned long value;
    if (*pos == '"') {
        pos++;
        value = strtoul(pos, &end, 0);
        if (!end || *end != '"') return false;
    } else {
        value = strtoul(pos, &end, 0);
    }
    if (errno != 0) {
        return false;
    }
    *out_value = (uint32_t)value;
    return true;
}

static void respond_ok(int fd, const char *data_fmt, ...) {
    char data[2048];
    if (data_fmt) {
        va_list args;
        va_start(args, data_fmt);
        vsnprintf(data, sizeof(data), data_fmt, args);
        va_end(args);
    } else {
        snprintf(data, sizeof(data), "{}");
    }
    char body[2304];
    snprintf(body, sizeof(body), "{\"status\":\"ok\",\"data\":%s}\n", data);
    (void)write(fd, body, strlen(body));
}

static void respond_error(int fd, const char *fmt, ...) {
    char msg[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    char body[768];
    snprintf(body, sizeof(body), "{\"status\":\"error\",\"error\":\"%s\"}\n", msg);
    (void)write(fd, body, strlen(body));
}

static int apply_config_locked(daemon_state_t *state) {
    twr_service_config_t cfg;
    bool dirty = false;
    pthread_mutex_lock(&state->lock);
    dirty = state->config_dirty;
    cfg = state->cfg;
    if (dirty) {
        state->config_dirty = false;
    }
    pthread_mutex_unlock(&state->lock);
    if (!dirty) {
        return 0;
    }
    twr_service_set_config(&cfg);
    return 0;
}

static void *monitor_thread_main(void *arg) {
    daemon_state_t *state = (daemon_state_t *)arg;
    while (true) {
        pthread_mutex_lock(&state->lock);
        bool exit_requested = state->monitor_should_exit;
        bool chip_active = state->chip_active;
        bool hw_busy = state->hw_busy;
        uint32_t interval_ms = state->monitor_interval_ms;
        pthread_mutex_unlock(&state->lock);

        if (exit_requested) {
            break;
        }

        if (chip_active && !hw_busy) {
            bool ok = false;
            uint32_t devid = 0;
            pthread_mutex_lock(&state->hw_mutex);
            devid = dwt_readdevid();
            ok = (devid == DWT_DEVICE_ID);
            pthread_mutex_unlock(&state->hw_mutex);

            pthread_mutex_lock(&state->lock);
            state->last_health_ok = ok;
            if (ok) {
                snprintf(state->last_health_msg, sizeof(state->last_health_msg), "device ready");
            } else {
                snprintf(state->last_health_msg, sizeof(state->last_health_msg), "unexpected dev id 0x%08X", devid);
            }
            pthread_mutex_unlock(&state->lock);
        }

        if (interval_ms == 0) {
            interval_ms = DEFAULT_MONITOR_INTERVAL_MS;
        }
        usleep(interval_ms * 1000U);
    }
    pthread_mutex_lock(&state->lock);
    state->monitor_running = false;
    pthread_mutex_unlock(&state->lock);
    return NULL;
}

static int start_monitor_thread(daemon_state_t *state) {
    pthread_mutex_lock(&state->lock);
    if (state->monitor_running) {
        pthread_mutex_unlock(&state->lock);
        return 0;
    }
    state->monitor_should_exit = false;
    state->monitor_running = true;
    pthread_mutex_unlock(&state->lock);

    if (pthread_create(&state->monitor_thread, NULL, monitor_thread_main, state) != 0) {
        pthread_mutex_lock(&state->lock);
        state->monitor_running = false;
        pthread_mutex_unlock(&state->lock);
        log_message(LOG_ERR, "Failed to start monitor thread");
        return -1;
    }
    return 0;
}

static void stop_monitor_thread(daemon_state_t *state) {
    pthread_mutex_lock(&state->lock);
    if (!state->monitor_running) {
        pthread_mutex_unlock(&state->lock);
        return;
    }
    state->monitor_should_exit = true;
    pthread_mutex_unlock(&state->lock);
    pthread_join(state->monitor_thread, NULL);
}

static int activate_chip(daemon_state_t *state) {
    pthread_mutex_lock(&state->lock);
    if (state->chip_active) {
        pthread_mutex_unlock(&state->lock);
        return 0;
    }
    pthread_mutex_unlock(&state->lock);

    if (chip_init() != 0) {
        log_message(LOG_ERR, "chip_init failed");
        return -1;
    }

    pthread_mutex_lock(&state->lock);
    state->chip_active = true;
    state->last_health_ok = true;
    snprintf(state->last_health_msg, sizeof(state->last_health_msg), "initialized");
    state->config_dirty = true;
    pthread_mutex_unlock(&state->lock);

    pthread_mutex_lock(&state->hw_mutex);
    apply_config_locked(state);
    pthread_mutex_unlock(&state->hw_mutex);

    return start_monitor_thread(state);
}

static int deactivate_chip(daemon_state_t *state) {
    pthread_mutex_lock(&state->lock);
    if (!state->chip_active) {
        pthread_mutex_unlock(&state->lock);
        return 0;
    }
    if (state->hw_busy) {
        pthread_mutex_unlock(&state->lock);
        return -1;
    }
    state->chip_active = false;
    pthread_mutex_unlock(&state->lock);

    stop_monitor_thread(state);

    pthread_mutex_lock(&state->hw_mutex);
    chip_shutdown();
    pthread_mutex_unlock(&state->hw_mutex);

    pthread_mutex_lock(&state->lock);
    snprintf(state->last_health_msg, sizeof(state->last_health_msg), "inactive");
    pthread_mutex_unlock(&state->lock);
    return 0;
}

static int perform_measurement(daemon_state_t *state, twr_role_t role, uint32_t count,
                               uint16_t peer_id, uint32_t expected_polls,
                               twr_measurement_stats_t *stats) {
    pthread_mutex_lock(&state->lock);
    if (!state->chip_active) {
        pthread_mutex_unlock(&state->lock);
        return -1;
    }
    if (state->hw_busy) {
        pthread_mutex_unlock(&state->lock);
        return -2;
    }
    state->hw_busy = true;
    state->cfg.peer_id = peer_id;
    state->config_dirty = true;
    pthread_mutex_unlock(&state->lock);

    pthread_mutex_lock(&state->hw_mutex);
    apply_config_locked(state);
    twr_service_prepare_frames(role);
    int rc;
    if (role == TWR_ROLE_RESPONDER) {
        rc = twr_service_run_responder(expected_polls);
    } else {
        rc = twr_service_run_initiator(count, stats);
    }
    pthread_mutex_unlock(&state->hw_mutex);

    pthread_mutex_lock(&state->lock);
    state->hw_busy = false;
    if (role == TWR_ROLE_INITIATOR && rc == 0 && stats) {
        state->last_stats = *stats;
        state->have_last_stats = true;
        state->last_measurement_time = time(NULL);
    }
    pthread_mutex_unlock(&state->lock);
    return rc;
}

static bool load_config_line(daemon_state_t *state, const char *line) {
    const char *eq = strchr(line, '=');
    if (!eq) return false;
    size_t key_len = (size_t)(eq - line);
    char key[64];
    if (key_len >= sizeof(key)) return false;
    strncpy(key, line, key_len);
    key[key_len] = '\0';
    char value_str[128];
    snprintf(value_str, sizeof(value_str), "%s", eq + 1);
    for (size_t i = 0; key[i]; ++i) key[i] = (char)tolower((unsigned char)key[i]);
    char *newline = strpbrk(value_str, "\r\n");
    if (newline) *newline = '\0';
    char *value_ptr = value_str;
    while (*value_ptr && isspace((unsigned char)*value_ptr)) value_ptr++;
    char *end_trim = value_ptr + strlen(value_ptr);
    while (end_trim > value_ptr && isspace((unsigned char)end_trim[-1])) {
        *--end_trim = '\0';
    }

    pthread_mutex_lock(&state->lock);
    if (strcmp(key, "pan_id") == 0) {
        state->cfg.pan_id = (uint16_t)strtoul(value_ptr, NULL, 0);
        state->config_dirty = true;
    } else if (strcmp(key, "node_id") == 0 || strcmp(key, "chip_id") == 0 ||
               strcmp(key, "initiator_id") == 0) {
        state->cfg.local_id = (uint16_t)strtoul(value_ptr, NULL, 0);
        state->config_dirty = true;
    } else if (strcmp(key, "peer_id") == 0 || strcmp(key, "responder_id") == 0) {
        state->cfg.peer_id = (uint16_t)strtoul(value_ptr, NULL, 0);
        state->config_dirty = true;
    } else if (strcmp(key, "antenna_delay") == 0) {
        state->cfg.antenna_delay = (uint16_t)strtoul(value_ptr, NULL, 0);
        state->config_dirty = true;
    } else if (strcmp(key, "monitor_interval_ms") == 0) {
        state->monitor_interval_ms = (uint32_t)strtoul(value_ptr, NULL, 0);
    } else if (strcmp(key, "socket_path") == 0) {
        snprintf(state->socket_path, sizeof(state->socket_path), "%s", value_ptr);
    } else {
        pthread_mutex_unlock(&state->lock);
        return false;
    }
    twr_service_config_t cfg = state->cfg;
    pthread_mutex_unlock(&state->lock);
    twr_service_set_config(&cfg);
    return true;
}

static void load_config_file(daemon_state_t *state, const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        log_message(LOG_INFO, "No config file at %s", path);
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        load_config_line(state, line);
    }
    fclose(fp);
}

static void send_status(int fd, daemon_state_t *state) {
    pthread_mutex_lock(&state->lock);
    bool chip = state->chip_active;
    bool busy = state->hw_busy;
    bool health = state->last_health_ok;
    char health_msg[128];
    snprintf(health_msg, sizeof(health_msg), "%s", state->last_health_msg);
    twr_service_config_t cfg = state->cfg;
    bool have_stats = state->have_last_stats;
    twr_measurement_stats_t stats = state->last_stats;
    time_t last_time = state->last_measurement_time;
    uint32_t monitor_interval = state->monitor_interval_ms;
    char socket_path[PATH_MAX];
    snprintf(socket_path, sizeof(socket_path), "%s", state->socket_path);
    pthread_mutex_unlock(&state->lock);

    if (have_stats) {
        respond_ok(fd,
            "{\"chip_active\":%s,\"hw_busy\":%s,\"monitor_ok\":%s,"
            "\"health_msg\":\"%s\",\"config\":{\"pan_id\":%u,\"node_id\":%u,"
            "\"antenna_delay\":%u,\"monitor_interval_ms\":%u,"
            "\"socket_path\":\"%s\"},"
            "\"last_peer_id\":%u,"
            "\"last_measurement\":{\"ts\":%lld,\"attempts\":%u,\"successes\":%u,"
            "\"raw_count\":%zu,\"filtered_count\":%zu,\"raw_mean\":%.3f,"
            "\"filtered_mean\":%.3f,\"filtered_stddev\":%.3f}}",
            chip ? "true" : "false",
            busy ? "true" : "false",
            health ? "true" : "false",
            health_msg,
            cfg.pan_id,
            cfg.local_id,
            cfg.antenna_delay,
            monitor_interval,
            socket_path,
            cfg.peer_id,
            (long long)last_time,
            stats.attempts,
            stats.successes,
            stats.raw_count,
            stats.filtered_count,
            stats.raw_mean,
            stats.filtered_mean,
            stats.filtered_stddev);
    } else {
        respond_ok(fd,
            "{\"chip_active\":%s,\"hw_busy\":%s,\"monitor_ok\":%s,"
            "\"health_msg\":\"%s\",\"config\":{\"pan_id\":%u,\"node_id\":%u,"
            "\"antenna_delay\":%u,\"monitor_interval_ms\":%u,"
            "\"socket_path\":\"%s\"},\"last_peer_id\":%u}",
            chip ? "true" : "false",
            busy ? "true" : "false",
            health ? "true" : "false",
            health_msg,
            cfg.pan_id,
            cfg.local_id,
            cfg.antenna_delay,
            monitor_interval,
            socket_path,
            cfg.peer_id);
    }
}

static void handle_twr_request(int fd, const char *req, daemon_state_t *state) {
    uint32_t peer_id = 0;
    if (!json_get_uint(req, "peer_id", &peer_id)) {
        respond_error(fd, "peer_id missing");
        return;
    }
    char role_buf[16];
    twr_role_t role = TWR_ROLE_INITIATOR;
    if (json_get_string(req, "role", role_buf, sizeof(role_buf))) {
        if (strcasecmp(role_buf, "responder") == 0) {
            role = TWR_ROLE_RESPONDER;
        } else if (strcasecmp(role_buf, "initiator") != 0) {
            respond_error(fd, "invalid role");
            return;
        }
    }

    uint32_t count = 0;
    if (role == TWR_ROLE_INITIATOR) {
        if (!json_get_uint(req, "count", &count)) {
            respond_error(fd, "count missing");
            return;
        }
        if (count == 0 || count > 1000) {
            respond_error(fd, "count must be 1-1000");
            return;
        }
    } else {
        json_get_uint(req, "count", &count);
        if (count > 1000) {
            respond_error(fd, "count must be <= 1000");
            return;
        }
    }

    uint32_t expected_polls = 0;
    if (!json_get_uint(req, "expected_polls", &expected_polls)) {
        if (role == TWR_ROLE_RESPONDER) {
            expected_polls = count;
        }
    }
    if (expected_polls > 1000) {
        respond_error(fd, "expected_polls must be 0-1000");
        return;
    }

    pthread_mutex_lock(&state->lock);
    uint16_t local_id = state->cfg.local_id;
    pthread_mutex_unlock(&state->lock);
    if (peer_id == 0 || peer_id > 0xFFFFu) {
        respond_error(fd, "peer_id must be 1-65535");
        return;
    }
    if (peer_id == local_id) {
        respond_error(fd, "peer_id must differ from node_id");
        return;
    }

    twr_measurement_stats_t stats;
    twr_measurement_stats_t *stats_ptr = (role == TWR_ROLE_INITIATOR) ? &stats : NULL;
    int rc = perform_measurement(state, role, count, (uint16_t)peer_id, expected_polls, stats_ptr);
    if (rc == -1) {
        respond_error(fd, "chip disabled");
        return;
    } else if (rc == -2) {
        respond_error(fd, "busy");
        return;
    } else if (rc != 0) {
        respond_error(fd, "measurement failed");
        return;
    }

    if (role == TWR_ROLE_INITIATOR && stats_ptr) {
        respond_ok(fd,
            "{\"attempts\":%u,\"successes\":%u,\"raw_count\":%zu,"
            "\"filtered_count\":%zu,\"rejected\":%zu,"
            "\"raw_mean\":%.3f,\"raw_median\":%.3f,\"raw_stddev\":%.3f,"
            "\"filtered_mean\":%.3f,\"filtered_median\":%.3f,\"filtered_stddev\":%.3f,"
            "\"filtered_min\":%.3f,\"filtered_max\":%.3f}",
            stats.attempts,
            stats.successes,
            stats.raw_count,
            stats.filtered_count,
            stats.rejected_count,
            stats.raw_mean,
            stats.raw_median,
            stats.raw_stddev,
            stats.filtered_mean,
            stats.filtered_median,
            stats.filtered_stddev,
            stats.filtered_min,
            stats.filtered_max);
    } else {
        respond_ok(fd, "{\"role\":\"responder\",\"expected_polls\":%u}", expected_polls);
    }
}

static void handle_client_request(int fd, const char *req, daemon_state_t *state) {
    char cmd[64];
    if (!json_get_string(req, "cmd", cmd, sizeof(cmd))) {
        respond_error(fd, "missing cmd");
        return;
    }

    if (strcmp(cmd, "chip_enable") == 0) {
        if (activate_chip(state) == 0) {
            respond_ok(fd, "{\"chip_active\":true}");
        } else {
            respond_error(fd, "chip init failed");
        }
    } else if (strcmp(cmd, "chip_disable") == 0) {
        if (deactivate_chip(state) == 0) {
            respond_ok(fd, "{\"chip_active\":false}");
        } else {
            respond_error(fd, "chip busy or shutdown failed");
        }
    } else if (strcmp(cmd, "get_status") == 0) {
        send_status(fd, state);
    } else if (strcmp(cmd, "chip_test") == 0) {
        pthread_mutex_lock(&state->lock);
        bool active = state->chip_active;
        pthread_mutex_unlock(&state->lock);
        if (!active) {
            respond_error(fd, "chip disabled");
            return;
        }
        uint32_t devid = 0;
        pthread_mutex_lock(&state->hw_mutex);
        devid = dwt_readdevid();
        pthread_mutex_unlock(&state->hw_mutex);
        respond_ok(fd, "{\"device_id\":\"0x%08X\"}", devid);
    } else if (strcmp(cmd, "set_node_id") == 0) {
        uint32_t node = 0;
        if (!json_get_uint(req, "node_id", &node)) {
            respond_error(fd, "missing node_id");
            return;
        }
        if (node == 0 || node > 0xFFFFu) {
            respond_error(fd, "node_id must be 1-65535");
            return;
        }
        twr_service_config_t cfg;
        pthread_mutex_lock(&state->lock);
        state->cfg.local_id = (uint16_t)node;
        state->config_dirty = true;
        cfg = state->cfg;
        pthread_mutex_unlock(&state->lock);
        twr_service_set_config(&cfg);
        respond_ok(fd, "{}");
    } else if (strcmp(cmd, "set_pan_id") == 0) {
        uint32_t pan = 0;
        if (!json_get_uint(req, "pan_id", &pan)) {
            respond_error(fd, "missing pan_id");
            return;
        }
        twr_service_config_t cfg;
        pthread_mutex_lock(&state->lock);
        state->cfg.pan_id = (uint16_t)pan;
        state->config_dirty = true;
        cfg = state->cfg;
        pthread_mutex_unlock(&state->lock);
        twr_service_set_config(&cfg);
        respond_ok(fd, "{}");
    } else if (strcmp(cmd, "set_antenna_delay") == 0) {
        uint32_t delay = 0;
        if (!json_get_uint(req, "antenna_delay", &delay)) {
            respond_error(fd, "missing antenna_delay");
            return;
        }
        twr_service_config_t cfg;
        pthread_mutex_lock(&state->lock);
        state->cfg.antenna_delay = (uint16_t)delay;
        state->config_dirty = true;
        cfg = state->cfg;
        pthread_mutex_unlock(&state->lock);
        twr_service_set_config(&cfg);
        respond_ok(fd, "{}");
    } else if (strcmp(cmd, "set_monitor_interval") == 0) {
        uint32_t interval = 0;
        if (!json_get_uint(req, "interval_ms", &interval) || interval == 0) {
            respond_error(fd, "invalid interval");
            return;
        }
        pthread_mutex_lock(&state->lock);
        state->monitor_interval_ms = interval;
        pthread_mutex_unlock(&state->lock);
        respond_ok(fd, "{}");
    } else if (strcmp(cmd, "twr_measure") == 0) {
        handle_twr_request(fd, req, state);
    } else if (strcmp(cmd, "get_config") == 0) {
        pthread_mutex_lock(&state->lock);
        twr_service_config_t cfg = state->cfg;
        uint32_t monitor_interval = state->monitor_interval_ms;
        char socket_path[PATH_MAX];
        char config_path[PATH_MAX];
        snprintf(socket_path, sizeof(socket_path), "%s", state->socket_path);
        snprintf(config_path, sizeof(config_path), "%s", state->config_path);
        pthread_mutex_unlock(&state->lock);
        respond_ok(fd,
                   "{\"pan_id\":%u,\"node_id\":%u,\"antenna_delay\":%u,"
                   "\"monitor_interval_ms\":%u,"
                   "\"socket_path\":\"%s\",\"config_path\":\"%s\"}",
                   cfg.pan_id, cfg.local_id, cfg.antenna_delay,
                   monitor_interval, socket_path, config_path);
    } else {
        respond_error(fd, "unknown cmd");
    }
}

static void event_loop(daemon_state_t *state) {
    char buffer[MAX_REQUEST_SIZE];
    while (!g_should_terminate) {
        int client_fd = accept4(state->server_fd, NULL, NULL, SOCK_CLOEXEC);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            log_message(LOG_ERR, "accept() failed: %s", strerror(errno));
            break;
        }
        ssize_t total = read(client_fd, buffer, sizeof(buffer) - 1);
        if (total <= 0) {
            close(client_fd);
            continue;
        }
        buffer[total] = '\0';
        handle_client_request(client_fd, buffer, state);
        close(client_fd);
    }
}

static int daemonize_process(void) {
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid > 0) exit(0);
    if (setsid() < 0) return -1;
    pid = fork();
    if (pid < 0) return -1;
    if (pid > 0) exit(0);
    umask(0);
    if (chdir("/") != 0) {
        return -1;
    }
    int null_fd = open("/dev/null", O_RDWR);
    if (null_fd >= 0) {
        dup2(null_fd, STDIN_FILENO);
        dup2(null_fd, STDOUT_FILENO);
        dup2(null_fd, STDERR_FILENO);
        if (null_fd > 2) close(null_fd);
    }
    return 0;
}

int main(int argc, char **argv) {
    bool foreground = false;
    daemon_state_t state;
    daemon_state_init(&state);

    int opt;
    while ((opt = getopt(argc, argv, "fc:s:")) != -1) {
        switch (opt) {
        case 'f':
            foreground = true;
            break;
        case 'c':
            snprintf(state.config_path, sizeof(state.config_path), "%s", optarg);
            break;
        case 's':
            snprintf(state.socket_path, sizeof(state.socket_path), "%s", optarg);
            break;
        default:
            fprintf(stderr, "Usage: %s [-f] [-c config] [-s socket]\n", argv[0]);
            return 1;
        }
    }

    state.foreground = foreground;
    g_foreground_mode = foreground;
    openlog("dw3k-daemon", LOG_PID | LOG_CONS, LOG_DAEMON);
    if (ensure_config_file(&state) != 0) {
        return 1;
    }
    load_config_file(&state, state.config_path);

    if (!foreground) {
        if (daemonize_process() != 0) {
            log_message(LOG_ERR, "Failed to daemonize");
            return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);

    state.server_fd = create_server_socket(state.socket_path);
    if (state.server_fd < 0) {
        return 1;
    }

    log_message(LOG_INFO, "dw3k-daemon listening on %s", state.socket_path);
    event_loop(&state);

    close(state.server_fd);
    unlink(state.socket_path);
    deactivate_chip(&state);
    closelog();
    return 0;
}


