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


#define TEST_RUNNING_FLAG        (1<<0)
#define SWEEP_RUNNING_FLAG       (1<<1)
#define FIFO_EMPTY_FLAG          (1<<2)
#define SPI_BUSY_FLAG            (1<<3)
#define DDR_CAL_FLAG             (1<<4)
#define UI_BUSY_FLAG             (1<<5)

#define STAT_ARBITER_INT_FLAG    (1<<6)
#define STAT_MIG2HOST_INT_FLAG   (1<<7)
#define STAT_HOST2MIG_INT_FLAG   (1<<8)
#define STAT_DEBUG_INT_FLAG      (1<<9)
#define STAT_EDFA_INT_FLAG       (1<<10)
#define STAT_RS485_INT_FLAG      (1<<11)
#define STAT_I2C_INT_FLAG        (1<<12)
#define STAT_GPIO_INT_FLAG       (1<<13)
#define STAT_SPI_INT_FLAG        (1<<14)
#define STAT_QSPI_INT_FLAG       (1<<15)

#define STAT_VERSION_MINOR_MASK  0x00ff0000
#define STAT_VERSION_MINOR_SHIFT 16

#define STAT_VERSION_MAJOR_MASK  0xff000000
#define STAT_VERSION_MAJOR_SHIFT 24

#define BIT_INT_ARBITER         (1<<0)
#define BIT_INT_MIG2HOST        (1<<1)
#define BIT_INT_HOST2MIG        (1<<2)
#define BIT_INT_DEBUG           (1<<3)
#define BIT_INT_EDFA            (1<<4)
#define BIT_INT_RS485           (1<<5)
#define BIT_INT_I2C             (1<<6)
#define BIT_INT_GPIO            (1<<7)
#define BIT_INT_SPI             (1<<8)
#define BIT_INT_QSPI            (1<<9)

#define BIT_CLR_ARBITER         (1<<16)
#define BIT_CLR_MIG2HOST        (1<<17)
#define BIT_CLR_HOST2MIG        (1<<18)
#define BIT_CLR_DEBUG           (1<<19)
#define BIT_CLR_EDFA            (1<<20)
#define BIT_CLR_RS485           (1<<21)
#define BIT_CLR_I2C             (1<<22)
#define BIT_CLR_GPIO            (1<<23)
#define BIT_CLR_SPI             (1<<24)
#define BIT_CLR_QSPI            (1<<25)


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
#define GLOBAL_INT_ENABLE        0x1000

#define CLK_400_MODE             0x0

#define REPEAT_LOOP              0x80000000
#define INTERRUPT_ACTIVE         0x40000000
#define REPEAT_COUNT             0x000003ff

#define RUN_BOTDA                (BOTDA_MODE|DAC_UPDATE_ENABLE)
#define RUN_BOTDA_TEST           (BOTDA_MODE|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE)
#define RUN_COTDR                (COTDR_MODE)
#define RUN_COTDR_TEST           (COTDR_MODE|TEST_DATA_ENABLE)
#define RUN_COTDR_TEST_200       (COTDR_MODE|TEST_DATA_ENABLE|CLK_200_MODE)
#define RUN_ALT_COTDR_TEST       (COTDR_MODE_ALT|TEST_DATA_ENABLE)
#define RUN_ALT_COTDR_TEST_200   (COTDR_MODE_ALT|TEST_DATA_ENABLE|CLK_200_MODE)
#define RUN_DUAL_COTDR_TEST      (COTDR_MODE_DUAL|TEST_DATA_ENABLE)
#define RUN_DUAL_COTDR_TEST_200  (COTDR_MODE_DUAL|TEST_DATA_ENABLE|CLK_200_MODE)
#define RUN_ROTDR                ROTDR_MODE
#define RUN_ROTDR_TEST           (ROTDR_MODE|TEST_DATA_ENABLE)
#define RUN_NO_TEST              0

#define FREQUENCY_COUNTER_ACTIVE (TEST_CLK_SELECT)
#define FREQUENCY_TEST_ACTIVE    (SWEEP_TEST_MODE|TEST_CLK_SELECT)
#define EOM_ACTIVE               (SWEEP_TEST_MODE)

#define DEFAULT_ROW_STRIDE       0x00100000

#define DMA_LENGTH	(4*1024*1024)

#define FOS_DEBUG_READ           1
#define FOS_DEBUG_WRITE          2
#define FOS_DEBUG_DMA_READ       3
#define FOS_DEBUG_DMA_WRITE      4

#define MAX_DATA_AREAS           8
#define FOS_LINKED_LIST_NEXT     0x12348765
#define FOS_LINKED_LIST_LAST     0xabcd4321


#define TRANSFER_MASK            0xffff0000
#define TRANSFER_CH              0x0001
#define TRANSFER_NUM_CH          0x0002
#define TRANSFER_RDWR            0x0004
#define TRANSFER_DATA_MODE       0x0008

#define TRANSFER_MAGIC           0xd45f0000
#define TRANSFER_CH0             0x0000
#define TRANSFER_CH1             0x0001
#define TRANSFER_SINGLE          0x0000
#define TRANSFER_DUAL            0x0002
#define TRANSFER_READ            0x0000
#define TRANSFER_WRITE           0x0004
#define TRANSFER_RAW             0x0000
#define TRANSFER_FILTERED        0x0008

#define TRANSFER_RD_SINGLE_CH0      (TRANSFER_MAGIC|TRANSFER_SINGLE|TRANSFER_CH0|TRANSFER_READ|TRANSFER_RAW)
#define TRANSFER_RD_SINGLE_CH1      (TRANSFER_MAGIC|TRANSFER_SINGLE|TRANSFER_CH1|TRANSFER_READ|TRANSFER_RAW)
#define TRANSFER_RD_DUAL_CH0        (TRANSFER_MAGIC|TRANSFER_DUAL|TRANSFER_CH0|TRANSFER_READ|TRANSFER_RAW)
#define TRANSFER_RD_DUAL_CH1        (TRANSFER_MAGIC|TRANSFER_DUAL|TRANSFER_CH1|TRANSFER_READ|TRANSFER_RAW)

enum fos_user_cmds
{
   FOS_USER_RESET,
   FOS_USER_SET_CLK,
   FOS_USER_SET_PULSE,
   FOS_USER_SET_ADC_OFFSET,
   FOS_USER_SET_ROW_STRIDE,
   FOS_USER_RUN_TEST,
   FOS_USER_STATUS,
   FOS_USER_MIG2HOST_DATA,
   FOS_USER_HOST2MIG_DATA,
   FOS_USER_READ_FREQUENCY,
   FOS_USER_SPI_WRITE,
   FOS_USER_RUN_SWEEP,
   FOS_USER_READ_DAC_TABLE,
   FOS_USER_WRITE_DAC_TABLE,
   FOS_INTERRUPT_COUNT,
   FOS_DMA_BLOCK_COUNT,
   FOS_CONTINUOUS_SCAN,
   FOS_REG_DEBUG,
   FOS_INTERRUPT_ENABLE,
   FOS_INTERRUPT_STATUS,
   FOS_DMA_REG_DEBUG,
   FOS_TRANSFER_TO_USER
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
   __u32                           num_bytes[16];
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

struct FOS_transfer_data_struct {
   __u32                         mig_base_address; // address of MIG memory
   __u32                         mig_stride;       // memory stride
   __u32                         num_columns;      // Number of bytes in column (multiple of 64)
   __u32                         num_rows;         // Number of rows to transfer
   __u32                         host_offset_addr; // offset from start of DMA buffer
} ;

struct FOS_transfer_user_struct {
   __u32                         *user_address;    // address of user memory
   __u32                         command;          // memory stride
   __u32                         mig_stride;       // memory stride
   __u32                         first_row;        // first row to transfer (starts at 0)
   __u32                         num_rows;         // Number of rows to transfer
   __u32                         first_column[MAX_DATA_AREAS];
   __u32                         num_columns[MAX_DATA_AREAS];
   __u32                         list_next[MAX_DATA_AREAS];
} ;

struct FOS_int_status_struct {
   __u32 int_active_mask;
   __u32 irq_count;
   __u32 arbiter_int_count;
   __u32 mig2host_int_count;
   __u32 host2mig_int_count;
   __u32 debug_int_count;
   __u32 edfa_int_count;
   __u32 rs485_int_count;
   __u32 i2c_int_count;
   __u32 gpio_int_count;
   __u32 spi_int_count;
   __u32 qspi_int_count;
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
#define FOS_USER_MIG2HOST_DATA      _IOWR(FOS_IOCTL_BASE, 0x88, struct FOS_transfer_data_struct)
#define FOS_USER_HOST2MIG_DATA      _IOWR(FOS_IOCTL_BASE, 0x89, struct FOS_transfer_data_struct)
#define FOS_USER_READ_FREQUENCY     _IOWR(FOS_IOCTL_BASE, 0x8a, struct FOS_cmd_struct)
#define FOS_USER_SPI_WRITE          _IOWR(FOS_IOCTL_BASE, 0x8b, struct FOS_cmd_struct)
#define FOS_USER_CONFIG_SWEEP       _IOWR(FOS_IOCTL_BASE, 0x8c, struct FOS_cmd_struct)
#define FOS_USER_RUN_SWEEP          _IOWR(FOS_IOCTL_BASE, 0x8d, struct FOS_cmd_struct)
#define FOS_USER_READ_DAC_TABLE     _IOWR(FOS_IOCTL_BASE, 0x8e, struct FOS_cmd_struct)
#define FOS_USER_WRITE_DAC_TABLE    _IOWR(FOS_IOCTL_BASE, 0x8f, struct FOS_cmd_struct)
#define FOS_INTERRUPT_COUNT         _IOWR(FOS_IOCTL_BASE, 0x90, struct FOS_cmd_struct)
#define FOS_DMA_BLOCK_COUNT         _IOWR(FOS_IOCTL_BASE, 0x91, struct FOS_cmd_struct)
#define FOS_CONTINUOUS_SCAN         _IOWR(FOS_IOCTL_BASE, 0x92, struct FOS_cmd_struct)
#define FOS_REG_DEBUG               _IOWR(FOS_IOCTL_BASE, 0x93, struct FOS_cmd_struct)
#define FOS_INTERRUPT_ENABLE        _IOWR(FOS_IOCTL_BASE, 0x94, struct FOS_cmd_struct)
#define FOS_INTERRUPT_STATUS        _IOWR(FOS_IOCTL_BASE, 0x95, struct FOS_cmd_struct)
#define FOS_DMA_REG_DEBUG           _IOWR(FOS_IOCTL_BASE, 0x96, struct FOS_cmd_struct)
#define FOS_TRANSFER_TO_USER        _IOWR(FOS_IOCTL_BASE, 0x97, struct FOS_cmd_struct)

#endif /* _FOS_H */
