#ifndef LOG_MSG
#define LOG_MSG

typedef nx_struct log_msg {
  nx_uint16_t occupancy;
  nx_uint16_t num_occupant;
  nx_uint16_t activity;
} log_msg_t;

enum {
  AM_LOG_MSG = 101;
};

#endif
