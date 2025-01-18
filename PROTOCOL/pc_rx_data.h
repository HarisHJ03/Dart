#ifndef _pc_rx_data_H
#define _pc_rx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_RX_FIFO_BUFLEN 500

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR         = 4,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;

/**
 * @brief
 */
#pragma pack(1)
typedef struct
{
  union
  {
    uint8_t data_byte;
    struct
    {
      uint8_t visual_valid : 1;
      uint8_t reserved : 7;
    } info;
  } mode_Union;
  float aim_yaw;
  float aim_pitch;
} robot_rx_data;
#pragma pack()

extern uart_dma_rxdata_t pc_rx_obj;
extern unpack_data_t pc_unpack_obj;
extern robot_rx_data pc_recv_mesg;


void pc_rx_param_init(void);
void pc_data_handler(uint8_t *p_frame);

#endif
