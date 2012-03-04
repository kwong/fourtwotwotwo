#ifndef STR_MSG
#define STR_MSG

typedef nx_struct sr_msg {
  nx_uint16_t type; // 0 for serial to mote, 1 for mote to mote
  nx_uint16_t light_a;
  nx_uint16_t humidity_a;
  nx_uint16_t temp_a;
  nx_uint16_t light_b;
  nx_uint16_t humidity_b;
  nx_uint16_t temp_b;
} sr_msg_t;

enum {
  AM_SR_MSG = 101,
};

#endif
