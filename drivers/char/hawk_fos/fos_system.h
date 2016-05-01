/*
 *  FOS_system.h -- Register definitions for FOS implementation
 *
 *  Greg Smart
 *
 *  Version 0.1 10/03/14
 *
 */

/*--------------------------------------------------------------------
 *
 *  FOS register include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _FOS_SYSTEM_H
#define _FOS_SYSTEM_H

/* general register memory location */

#define FOS_BASE                 0x43C00000

#define R_FOS_FIFO_BASE          0x0800
#define R_FOS_SWEEP_RAM_BASE     0x1000
#define R_FOS_DAC_LOOKUP_BASE    0x2000
#define R_FREQUENCY_STATUS       0x3000

#define R_PULSE_RATE             0x0000
#define R_PULSE_WIDTH            0x0004
#define R_RUN_TEST               0x0008
#define R_CONFIG                 0x000C

#define R_MIG2HOST_READ_ADDR     0x0010   // read address on 64 byte boundaries
#define R_MIG2HOST_STRIDE        0x0014
#define R_MIG2HOST_COL_COUNT     0x0018
#define R_MIG2HOST_WRITE_ADDR    0x001C
#define R_MIG2HOST_ROW_COUNT     0x0020

#define R_MEM_STRIDE             0x0024   // Write number of 256byte blocks to read
#define R_SPI_PORT_SPEED         0x0028   // Set SPI port speed

#define R_BOTDA_START_FREQ       0x002C   // Write start frequency of BOTDA scan
#define R_BOTDA_END_FREQ         0x0030   // Write end frequency of BOTDA scan
#define R_ADC_COUNT              0x0034   // Write number of ADC samples to store * 16
#define R_BOTDA_COUNT            0x0038   // Write number of accumulations to perform
#define R_ADC_OFFSET             0x003C   // Write adc offset value to remove

#define R_SPI0                   0x0040
#define R_SPI1                   0x0044
#define R_SPI2                   0x0048
#define R_SPI3                   0x004C
#define R_SPI4                   0x0050
#define R_SPI5                   0x0054   // SPI 5 is a 24 bit SPI word
#define R_SPI6                   0x0058
#define R_SPI7                   0x005C   // This register doesn't exist

#define R_HOST2MIG_WRITE_ADDR    0x0060   // WRITE address on 64 byte boundaries
#define R_HOST2MIG_STRIDE        0x0064
#define R_HOST2MIG_COL_COUNT     0x0068
#define R_HOST2MIG_READ_ADDR     0x006C
#define R_HOST2MIG_ROW_COUNT     0x0070

#define R_INTERRUPT              0x0074   // size of DMA transfers

#define R_RUN_STATUS             0x0000   // read status

#define FOS_FIFO_BASE            (FOS_BASE + R_FOS_FIFO_BASE)
#define FOS_SWEEP_RAM_BASE       (FOS_BASE + R_FOS_SWEEP_RAM_BASE)
#define FOS_DAC_LOOKUP_BASE      (FOS_BASE + R_FOS_DAC_LOOKUP_BASE)

#define PULSE_RATE_ADDR          (FOS_BASE + R_PULSE_RATE)
#define PULSE_WIDTH_ADDR         (FOS_BASE + R_PULSE_WIDTH)
#define RUN_TEST_ADDR            (FOS_BASE + R_RUN_TEST)
#define CONFIG_ADDR              (FOS_BASE + R_CONFIG)
#
#define DATA_READ_ADDR           (FOS_BASE + R_DATA_READ_ADDR)
#define DATA_READ_STRIDE_ADDR    (FOS_BASE + R_DATA_READ_STRIDE)
#define DATA_READ_COL_COUNT_ADDR (FOS_BASE + R_DATA_READ_COL_COUNT)
#define DATA_READ_ROW_COUNT_ADDR (FOS_BASE + R_DATA_READ_ROW_COUNT)

#define DATA_READ_START_ADDR     (FOS_BASE + R_DATA_READ_START)  // read address on 64 byte boundaries
#define MEM_STRIDE_ADDR          (FOS_BASE + R_MEM_STRIDE)  // Write number of 256byte blocks to read
#define SPI_PORT_SPEED_ADDR      (FOS_BASE + R_SPI_PORT_SPEED) // Set SPI port speed

#define BOTDA_START_FREQ_ADDR    (FOS_BASE + R_BOTDA_START_FREQ)  // Write start frequency of BOTDA scan
#define BOTDA_END_FREQ_ADDR      (FOS_BASE + R_BOTDA_END_FREQ)  // Write end frequency of BOTDA scan

#define ADC_COUNT_ADDR           (FOS_BASE + R_ADC_COUNT)  // Write number of ADC samples to store * 16

#define BOTDA_COUNT_ADDR         (FOS_BASE + R_BOTDA_COUNT)  // Write number of accumulations to perform

#define ADC_OFFSET_ADDR          (FOS_BASE + R_ADC_OFFSET)  // Write adc offset value to remove

#define SPI_ADDR_BASE            (FOS_BASE + R_SPI0)
#define SPI0_ADDR                (FOS_BASE + R_SPI0)
#define SPI1_ADDR                (FOS_BASE + R_SPI1)
#define SPI2_ADDR                (FOS_BASE + R_SPI2)
#define SPI3_ADDR                (FOS_BASE + R_SPI3)
#define SPI4_ADDR                (FOS_BASE + R_SPI4)
#define SPI5_ADDR                (FOS_BASE + R_SPI5)  // SPI 5 is a 24 bit SPI word
#define SPI6_ADDR                (FOS_BASE + R_SPI6)
#define SPI7_ADDR                (FOS_BASE + R_SPI7)  // This register doesn't exist

#define COUNT_STATUS_ADDR        (FOS_BASE + R_COUNT_STATUS)   // SPI 5 is a 24 bit SPI word
#define FREQUENCY_STATUS_ADDR    (FOS_BASE + R_FREQUENCY_STATUS)
#define RUN_STATUS_ADDR          (FOS_BASE + R_RUN_STATUS)     // This register doesn't exist

/*
** Sweep constants
*/
#define DATA_SWEEP               0x100
#define FULL_SWEEP               0x00

#define SWEEP_TYPE               0x01
#define TEST_SWEEP_ENABLE        0x02
#define OUTPUT_SWEEP_CLK_ENABLE  0x04
#define RUN_SWEEP_ENABLE         0x08

/*
 * DMA register constants
 */
#define MM2S_DMACR	0x00
#define MM2S_DMASR	0x04
#define MM2S_SA	   0x18
#define MM2S_LENGTH	0x28

#define S2MM_DMACR	0x30
#define S2MM_DMASR	0x34
#define S2MM_DA	   0x48
#define S2MM_LENGTH	0x58

/*
 *  struct FOS_axi_lite_base.
 *  This structure provides general access for the 3 blocks within the system
 */

struct FOS_axi_lite_base {
   u32                           registers[512];
   u32                           ddr3_fifo[512];
   u32                           sweep_ram[1024];
   u32                           dac_lookup[1024];
} ;

/*
 *  struct FOS_registers.
 *  This structure points to the first block where the registers are located
 */

struct FOS_registers {
   u32                           pulse_rate_gen;      // 0x0
   u32                           pulse_width_gen;     // 0x4
   u32                           sweep_test;          // 0x8
   u32                           run_test;            // 0xC
   u32                           data_read_addr;      // 0x10
   u32                           data_read_stride;    // 0x14
   u32                           data_read_col_count; // 0x18
   u32                           data_read_row_count; // 0x1C
   u32                           data_read_start;     // 0x20
   u32                           mem_stride;          // 0x24
   u32                           spi_port_speed;      // 0x28
   u32                           botda_start_freq;    // 0x2C
   u32                           botda_end_freq;      // 0x30
   u32                           adc_count;           // 0x34
   u32                           botda_count;         // 0x38
   u32                           adc_offset;          // 0x3C
   u32                           spi0;                // 0x40
   u32                           spi1;                // 0x44
   u32                           spi2;                // 0x48
   u32                           spi3;                // 0x4C
   u32                           spi4;                // 0x50
   u32                           spi5;                // 0x54
   u32                           spi6;                // 0x58
   u32                           spi7;                // 0x5C
   u32                           data_write_addr;      // 0x60
   u32                           data_write_stride;    // 0x64
   u32                           data_write_col_count; // 0x68
   u32                           data_write_row_count; // 0x6C
   u32                           data_write_start;     // 0x70
   u32                           unused1;             // 0x74
   u32                           unused2;             // 0x78
   u32                           unused3;             // 0x7C
   u32                           count_status;        // 0x80
   u32                           frequency_status;    // 0x84
   u32                           run_status;          // 0x88
} ;

#define MAX_DEVICES     4

/**
 * struct fos_drvdata - Device Configuration driver structure
 *
 * @dev: Pointer to the device structure
 * @cdev: Instance of the cdev structure
 * @devt: Pointer to the dev_t structure
 * @class: Pointer to device class
 * @fclk_class: Pointer to fclk device class
 * @dma_done: The dma_done status bit for the DMA command completion
 * @error_status: The error status captured during the DMA transfer
 * @irq: Interrupt number
 * @clk: Peripheral clock for devcfg
 * @fclk: Array holding references to the FPGA clocks
 * @fclk_exported: Flag inidcating whether an FPGA clock is exported
 * @is_open: The status bit to indicate whether the device is opened
 * @sem: Instance for the mutex
 * @lock: Instance of spinlock
 * @base_address: The virtual device base address of the device registers
 * @is_partial_bitstream: Status bit to indicate partial/full bitstream
 */
struct fos_drvdata {
   struct device *dev;
   struct cdev cdev;
   dev_t devt;
   struct class *class;
   int irq;
   uint32_t irq_count;
   int dma_block_count;
   struct clk *clk;
   volatile bool dma_done;
   volatile int error_status;
   bool is_open;
   struct mutex mutex;
   spinlock_t lock;
   void __iomem *base;
   void __iomem *dma_base;
   uint32_t config_state;
   char *dma_addr;
   dma_addr_t dma_handle;
   struct FOS_read_data_struct repeat_read_cmd;

	struct list_head dev_list;
};

static inline void fos_write_reg(struct fos_drvdata *fos, unsigned int reg, uint32_t val)
{
	writel(val, fos->base + reg);
}

static inline uint32_t fos_read_reg(struct fos_drvdata *fos, unsigned int reg)
{
	return(readl(fos->base + reg));
}

static inline void fosdma_write_reg(struct fos_drvdata *fos, unsigned int reg, uint32_t val)
{
	writel(val, fos->dma_base + reg);
}

static inline uint32_t fosdma_read_reg(struct fos_drvdata *fos, unsigned int reg)
{
	return(readl(fos->dma_base + reg));
}

//
//
//
//
//  Prototypes
//
//
//
//
//
//
//
//
//
//
//

u32 FOS_Status(struct fos_drvdata *fos);

//
// FOS_Open()
//
// Open FOS system and mmap registers for user access.
//
// Returns pointer to virtual address.
//
u32 FOS_Open(u32 init_fpga);

//
// FOS_Close()
//
// Close FOS system and unmap memory.  Function is called with
// virtual address that was returned when opened.
//
u32 FOS_Close(int fd);

//
// FOS_Set_Pulse()
//
// Setup the pulse generator
//
int FOS_Set_Pulse(struct fos_drvdata *fos, void *user_ptr);

//
// FOS_Set_Adc_Offset()
//
// Set the ADC DC offset
//
u32 FOS_Set_Adc_Offset(int arg1);

//
// FOS_Set_Row_Stride()
//
// Set the memory row stride
//
u32 FOS_Set_Row_Stride(int arg1);

//
// FOS_Run Test()
//
// Initiate a COTDR, BOTBA or ROTDR test
//
// Function will return immediately after initiating test
// before test completion
//
int FOS_Run_Test(struct fos_drvdata *fos, void *user_ptr);

//
// FOS_Continuous_Scan()
//
// Initiate a continuous COTDR test
//
// Function will return immediately after initiating test
//
// Continuous scan uses 1024 scans looping around continually
// For default setup of 1Mbyte row_stride this means it will work
// with 1Gbyte of DDR3 memory
//
// Can support either 1 or 2 adc channels, at this point support 1 only
// until further testing.
//
// This command sets up a looping test.  Every 128 scans an interrupt is
// generated. The ISR calls FOS_Read_data which initiates a DMA transfer
// of the selected column block to system memory. Only one data block is
// currently supported.
//
// After the command is initiated, the write_block counter is incremented
// There are 8 memory blocks (8Mbytes each).  For a single channel transfer
// the maximum nuber of columns is (32768 - 32).  The column number must be
// on a boundary of 32 in increments of 32.
//
// Once started, the user only has to monitor the write_block counter to check
// when a new block is available.
//
int FOS_Continuous_Scan(struct fos_drvdata *fos, void *user_ptr);

//
// FOS_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
//static inline u32 FOS_Status(struct fos_drvdata *fos);

//
// FOS_Read_Data()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_Read_Data(struct fos_drvdata *fos, struct FOS_read_data_struct *cmd);

//
// FOS_Read_Frequency()
//
// Read frequency of index (-1 if error)
//
int FOS_Read_Frequency(struct fos_drvdata *fos, void *arg_ptr);

//
// FOS_SPI_Write()
//
// Write a command to SPI port
//
int FOS_SPI_Write(struct fos_drvdata *fos, void *user_ptr);

//
// FOS_Run_Sweep()
//
// Run a frequency sweep
// cmd determines number of dac values to sweep.
// This is normally 800,65536,or 262144
//
int FOS_Run_Sweep(struct fos_drvdata *fos, void *arg_ptr);

//
// FOS_Read_Dac_Table()
//
// This function reads the DAC table back from the
// FPGA.  It is useful for comparing table values to
// calculated values
//
int FOS_Read_DAC_Table(struct fos_drvdata *fos, u32 *read_data);

//
// FOS_Write_Dac_Table()
//
// This function writes the DAC table to the FPGA.
// Table values are 20 bit words.  Values > 2^20 are
// written as 0xFFFFF,
//
int FOS_Write_DAC_Table(struct fos_drvdata *fos, u32 *write_data);

#endif /* _FOS_SYSTEM_H */
