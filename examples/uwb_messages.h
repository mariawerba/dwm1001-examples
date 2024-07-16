//uwb_messages.h

#include "deca_device_api.h"


typedef enum
{
    BEACON,
    JOIN_REQ,
    JOIN_CONF,
    SET_LOC,
    TEST
} message_type_t;

typedef struct
{
    uint8 fctrl[2];
    uint8 seq_nb;
    uint8 panid[2];
    uint8 dest_addr[2];
    uint8 src_addr[2];
    uint8 msg_type;
} message_header_t;

typedef struct
{
    uint8 session_id;
    uint8 cluster_flag;
    uint8 superframe_num;
    uint8 seat_num;
    uint8 seat_map;
    uint8 rx_ts[4];
    uint8 tx_ts[4];
    uint8 data_type;
    uint8 data1[4];
    uint8 data2[4];
} beacon_payload_t;

typedef struct
{
    uint8 seat_num;
    /* data */
} join_req_payload_t;

typedef struct 
{
    uint8 seat_num;
} join_conf_payload_t;


typedef struct
{
    bool joined;
    uint8 fctrl[2]; //default, means something in IEEE
    uint8 panid[2]; //default
    uint8 src_addr[2];
    uint8 session_id;
    uint8 cluster_flag;
    uint8 superframe_num;
    uint8 seat_num;
    uint8 seat_map;
    uint8 init_addr[2];
} uwb_network_param_t;
