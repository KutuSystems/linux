/*
 *  FOS.h -- Register definitions for FOS implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 10/03/14
 *
 */

#ifndef _FOS_H
#define _FOS_H

#define COTDR_DATA_SIZE 2
#define BOTDA_DATA_SIZE 4
#define ROTDR_DATA_SIZE 8

#define DMA_BUSY_FLAG      0x80
#define DDR_RESET_FLAG     0x40
#define UI_RESET_FLAG      0x20
#define DDR_CAL_FLAG       0x10
#define SPI_BUSY_FLAG      0x08
#define FIFO_EMPTY_FLAG    0x04
#define SWEEP_RUNNING_FLAG 0x02
#define TEST_RUNNING_FLAG  0x01

#define SPI_MAX_WAIT_COUNT 1000000
#define MAX_WAIT_COUNT     10000

/*
** System constants
*/
#define FPGA_RESET               0x01
#define START_TEST               0x02
#define STOP_TEST                0x04
#define SWEEP_START              0x08

#define MODE_BITS                0x07
#define BOTDA_MODE               0x00
#define BOTDA_MODE_ALT           0x01
#define COTDR_MODE_ALT           0x02
#define COTDR_MODE               0x03
#define COTDR_MODE_DUAL          0x04
#define ROTDR_MODE               0x05
#define TEST_DATA_ENABLE         0x08
#define DAC_UPDATE_ENABLE        0x10
#define CLK_200_MODE             0x20
#define DMA_ACTIVE               0x40
#define DMA_DEBUG                0x80
#define SWEEP_MODE               0x100
#define SWEEP_TEST_MODE          0x200
#define TEST_CLK_SELECT          0x400
#define SMA_CLK_ENABLE           0x800

#define REPEAT_LOOP              0x80000000
#define INTERRUPT_ACTIVE         0x40000000
#define REPEAT_COUNT             0x000003ff

#define RUN_BOTDA                (BOTDA_MODE|DAC_UPDATE_ENABLE|START_TEST)
#define RUN_BOTDA_TEST           (BOTDA_MODE|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE|START_TEST)
#define RUN_COTDR                COTDR_MODE|START_TEST
#define RUN_COTDR_TEST           (COTDR_MODE|TEST_DATA_ENABLE|START_TEST)
#define RUN_ROTDR                ROTDR_MODE|START_TEST
#define RUN_ROTDR_TEST           (ROTDR_MODE|TEST_DATA_ENABLE|START_TEST)
#define RUN_NO_TEST              0

#define FREQUENCY_COUNTER_ACTIVE (TEST_CLK_SELECT)
#define FREQUENCY_TEST_ACTIVE    (SWEEP_TEST_MODE|TEST_CLK_SELECT)
#define EOM_ACTIVE               (SWEEP_TEST_MODE)

#define DEFAULT_ROW_STRIDE       0x00100000

#define DMA_LENGTH	(64*1024*1024)

#define FOS_DEBUG_READ           1
#define FOS_DEBUG_WRITE          2
#define FOS_DEBUG_DMA_READ       3
#define FOS_DEBUG_DMA_WRITE      4


enum fos_user_cmds
{
   FOS_USER_RESET,
   FOS_USER_SET_CLK,
   FOS_USER_SET_PULSE,
   FOS_USER_SET_ADC_OFFSET,
   FOS_USER_SET_ROW_STRIDE,
   FOS_USER_RUN_TEST,
   FOS_USER_STATUS,
   FOS_USER_READ_DATA,
   FOS_USER_READ_FREQUENCY,
   FOS_USER_SPI_WRITE,
   FOS_USER_RUN_SWEEP,
   FOS_USER_READ_DAC_TABLE,
   FOS_USER_WRITE_DAC_TABLE
};

/*
 *  struct FOS_registers.
 *  This structure points to the first block where the registers are located
 */

struct FOS_cmd_struct {
   __u32                           config;
   __u32                           run_test;
   __u32                           botda_start_freq;
   __u32                           botda_end_freq;
   __u32                           adc_count;
   __u32                           botda_count;
   __u32                           row_stride;
} ;

struct FOS_continuous_cmd_struct {
   __u32                           config;
   __u32                           run_test;
   __u32                           start_column;
   __u32                           end_column;
   __u32                           adc_count;
   __u32                           row_stride;
} ;


struct FOS_pulse_struct {
   __u32                           pulse_width_AOM;
   __u32                           pulse_width_EOM;
   __u32                           pulse_width_gain;
   __u32                           pulse_rate;
   __u32                           pulse_delay;
} ;

struct FOS_sweep_cmd_struct {
   __u32                           run_test;
   __u32                           initial_dac_value;
   __u32                           final_dac_value;
   __u32                           dac_step_size;
   __u32                           *sweep_data;
} ;

struct FOS_spi_cmd_struct {
   __u32                           port_addr[16];
   __u32                           port_data[16];
   __u32                           num_spi_writes;
} ;

struct FOS_read_data_struct {
   __u32                         *data;
   __u32                         column;
   __u32                         row;
   __u32                         num_columns;
   __u32                         num_rows;
   __u32                         row_stride;
   __u32                         datatype;
   __u32                         dma_addr_offset;
   __u32                         dma_active;
} ;

struct FOS_debug_struct {
   __u32                           cmd;
   __u32                           reg;
   __u32                           data;
} ;


#define FOS_IOCTL_BASE	'W'

#define FOS_USER_RESET              _IOWR(FOS_IOCTL_BASE, 0x81, struct FOS_cmd_struct)
#define FOS_USER_SET_CLK            _IOWR(FOS_IOCTL_BASE, 0x82, struct FOS_cmd_struct)
#define FOS_USER_SET_PULSE          _IOWR(FOS_IOCTL_BASE, 0x83, struct FOS_cmd_struct)
#define FOS_USER_SET_ADC_OFFSET     _IOWR(FOS_IOCTL_BASE, 0x84, struct FOS_cmd_struct)
#define FOS_USER_SET_ROW_STRIDE     _IOWR(FOS_IOCTL_BASE, 0x85, struct FOS_cmd_struct)
#define FOS_USER_RUN_TEST           _IOWR(FOS_IOCTL_BASE, 0x86, struct FOS_cmd_struct)
#define FOS_USER_STATUS             _IOWR(FOS_IOCTL_BASE, 0x87, struct FOS_cmd_struct)
#define FOS_USER_READ_DATA          _IOWR(FOS_IOCTL_BASE, 0x88, struct FOS_cmd_struct)
#define FOS_USER_READ_FREQUENCY     _IOWR(FOS_IOCTL_BASE, 0x89, struct FOS_cmd_struct)
#define FOS_USER_SPI_WRITE          _IOWR(FOS_IOCTL_BASE, 0x8a, struct FOS_cmd_struct)
#define FOS_USER_CONFIG_SWEEP       _IOWR(FOS_IOCTL_BASE, 0x8b, struct FOS_cmd_struct)
#define FOS_USER_RUN_SWEEP          _IOWR(FOS_IOCTL_BASE, 0x8c, struct FOS_cmd_struct)
#define FOS_USER_READ_DAC_TABLE     _IOWR(FOS_IOCTL_BASE, 0x8d, struct FOS_cmd_struct)
#define FOS_USER_WRITE_DAC_TABLE    _IOWR(FOS_IOCTL_BASE, 0x8e, struct FOS_cmd_struct)
#define FOS_INTERRUPT_COUNT         _IOWR(FOS_IOCTL_BASE, 0x8f, struct FOS_cmd_struct)
#define FOS_DMA_BLOCK_COUNT         _IOWR(FOS_IOCTL_BASE, 0x90, struct FOS_cmd_struct)
#define FOS_CONTINUOUS_SCAN         _IOWR(FOS_IOCTL_BASE, 0x91, struct FOS_cmd_struct)
#define FOS_DMA_STATUS              _IOWR(FOS_IOCTL_BASE, 0x92, struct FOS_cmd_struct)
#define FOS_DMA_TEST                _IOWR(FOS_IOCTL_BASE, 0x93, struct FOS_cmd_struct)
#define FOS_REG_DEBUG               _IOWR(FOS_IOCTL_BASE, 0x94, struct FOS_cmd_struct)

#endif /* _FOS_H */
