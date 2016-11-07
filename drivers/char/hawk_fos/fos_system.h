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

#define R_FOS_SPI_READ_BASE      0x0800
#define R_FOS_SWEEP_RAM_BASE     0x1000
#define R_FOS_DAC_LOOKUP_BASE    0x2000
#define R_FREQUENCY_STATUS       0x3000

// Control registers
#define R_PULSE_RATE             0x0000
#define R_PULSE_WIDTH            0x0004
#define R_CONFIG                 0x0008
#define R_INTERRUPT              0x000C
#define R_RUN_TEST               0x0010
#define R_MEM_STRIDE             0x0014   // size of each row in bytes
#define R_BOTDA_START_FREQ       0x0018   // Write start frequency of BOTDA scan
#define R_BOTDA_END_FREQ         0x001C   // Write end frequency of BOTDA scan
#define R_ADC_COUNT              0x0020   // Write number of ADC samples to store * 16
#define R_BOTDA_COUNT            0x0024   // Write number of accumulations to perform
#define R_ADC_OFFSET             0x0028   // Write adc offset value to remove
#define R_SPI_PORT_SPEED         0x002C   // Set SPI port speed
#define R_RSVD1                  0x0030
#define R_RSVD2                  0x0034
#define R_RSVD3                  0x0038
#define R_RSVD4                  0x003C

// SPI interface registers
#define R_SPI0                   0x0040
#define R_SPI1                   0x0044
#define R_SPI2                   0x0048
#define R_SPI3                   0x004C
#define R_SPI4                   0x0050
#define R_SPI5                   0x0054   // SPI 5 is a 24 bit SPI word
#define R_SPI6                   0x0058
#define R_SPI7                   0x005C   // This register doesn't exist

// MIG2HOST interface registers
#define R_MIG2HOST_READ_ADDR     0x0060   // read address on 64 byte boundaries
#define R_MIG2HOST_STRIDE        0x0064
#define R_MIG2HOST_COL_COUNT     0x0068
#define R_MIG2HOST_ROW_COUNT     0x006C
#define R_MIG2HOST_WRITE_ADDR    0x0070
#define R_MIG2HOST_WRITE_ADDR_HI 0x0074

// HOST2MIG interface registers
#define R_HOST2MIG_WRITE_ADDR    0x0080   // WRITE address on 64 byte boundaries
#define R_HOST2MIG_STRIDE        0x0084
#define R_HOST2MIG_COL_COUNT     0x0088
#define R_HOST2MIG_ROW_COUNT     0x008C
#define R_HOST2MIG_READ_ADDR     0x0090
#define R_HOST2MIG_READ_ADDR_HI  0x0094

#define DMA_TRANSFER_SIZE        0x100000

#define INTERRUPT_MASK           0x03ff   // There are 10 interrupt sources

#define R_RUN_STATUS             0x0000   // read status


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
   u32                           config;              // 0x8
   u32                           interrupt;           // 0xC
   u32                           run_test;            // 0x10
   u32                           mem_stride;          // 0x14
   u32                           botda_start_freq;    // 0x18
   u32                           botda_end_freq;      // 0x1C
   u32                           adc_count;           // 0x20
   u32                           botda_count;         // 0x24
   u32                           adc_offset;          // 0x28
   u32                           spi_port_speed;      // 0x2C
   u32                           rsvd1;               // 0x30
   u32                           rsvd2;               // 0x34
   u32                           rsvd3;               // 0x38
   u32                           rsvd4;               // 0x3C
   u32                           spi0;                // 0x40
   u32                           spi1;                // 0x44
   u32                           spi2;                // 0x48
   u32                           spi3;                // 0x4C
   u32                           spi4;                // 0x50
   u32                           spi5;                // 0x54
   u32                           spi6;                // 0x58
   u32                           spi7;                // 0x5C
   u32                           mig2host_read_addr;  // 0x60
   u32                           mig2host_stride;     // 0x64
   u32                           mig2host_col_count;  // 0x68
   u32                           mig2host_row_count;  // 0x6C
   u64                           mig2host_write_addr; // 0x70
   u32                           unused1;             // 0x78
   u32                           unused2;             // 0x7C
   u32                           host2mig_read_addr;  // 0x80
   u32                           host2mig_stride;     // 0x84
   u32                           host2mig_col_count;  // 0x88
   u32                           host2mig_row_count;  // 0x8C
   u64                           host2mig_write_addr; // 0x90
   u32                           unused3;             // 0x98
   u32                           unused4;             // 0x9C
};

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
   uint32_t int_active_mask;
   uint32_t irq_count;
   uint32_t arbiter_int_count;
   uint32_t mig2host_int_count;
   uint32_t host2mig_int_count;
   uint32_t debug_int_count;
   uint32_t edfa_int_count;
   uint32_t rs485_int_count;
   uint32_t i2c_int_count;
   uint32_t gpio_int_count;
   uint32_t spi_int_count;
   uint32_t qspi_int_count;
   int dma_block_count;
   int dev64_support;
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

static inline void fos_write_reg64(struct fos_drvdata *fos, unsigned int reg, uint64_t val)
{
	writeq(val, fos->base + reg);
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
// FOS_tfer_mig2host()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_tfer_mig2host(struct fos_drvdata *fos, struct FOS_transfer_data_struct *cmd);

//
// FOS_transfer_to_user()
//
// transfer data array to user space
//
int FOS_transfer_to_user(struct fos_drvdata *fos, struct FOS_transfer_user_struct *cmd);

//
// FOS_tfer_host2mig()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_tfer_host2mig(struct fos_drvdata *fos, struct FOS_transfer_data_struct *cmd);

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
