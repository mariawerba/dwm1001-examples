#include "deca_device_api.h"

enum msg_id { BEACON, JOIN_REQ, JOIN_CFM};

typedef struct
{
    /* data */
    uint8 frame_ctl;
    uint8 seq_num;
    uint8 pan_id; 
    uint8 dest_addr;
    uint8 src_addr;
}
dwt_msg_t;

//uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x12, 0x34}



//eventually make a get panid function in deca_device.c