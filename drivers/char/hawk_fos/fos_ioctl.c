/* The industrial I/O core
 *
 * Copyright (c) 2015 Greg Smart
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Handling of device specific ioctls.
 *
 *
 * Things to look at here.
 * - create generic link into driver
 *
 */
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>

#include <linux/iio/iio.h>
#include "fos.h"
#include "fos_system.h"
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define DEBUG
//
// FOS_Status()
//
// read the FOS status registers
//
// status if system is running test, sweep or is idle
//
u32 FOS_Status(struct fos_drvdata *fos)
{
   u32 status;

   status = fos_read_reg(fos, R_RUN_STATUS);
   status &= 0x000000ff;

   return status;
}

//
// FOS_Set_Pulse()
//
// Setup the pulse generator
//
int FOS_Set_Pulse(struct fos_drvdata *fos, void *user_ptr)
{

   struct FOS_pulse_struct pulse;

   u32 rate_reg, width_reg;
   u32 pulse_rate, pulse_delay, pulse_width_AOM, pulse_width_EOM, pulse_width_gain;

   if (copy_from_user(&pulse, user_ptr, sizeof(pulse))) {
      printk(KERN_DEBUG "FOS_Set_Pulse: copy failed\n");

      return -EFAULT;
   }

   if (FOS_Status(fos) & SWEEP_RUNNING_FLAG)
      return -2;

   if (FOS_Status(fos) & TEST_RUNNING_FLAG)
      return -3;

   pulse_rate        = pulse.pulse_rate/10;
   pulse_delay       = pulse.pulse_delay/5;
   pulse_width_AOM   = pulse.pulse_width_AOM/5;
   pulse_width_EOM   = pulse.pulse_width_EOM/5;
   pulse_width_gain  = pulse.pulse_width_gain/5;

   rate_reg = (pulse_rate & 0x3f)|((pulse_delay & 0xff) << 8);
   width_reg = ((pulse_width_gain & 0x1ff) << 16)|((pulse_width_AOM & 0x3f) << 8)|(pulse_width_EOM & 0x3f);

   fos_write_reg(fos, R_PULSE_RATE, rate_reg);
   fos_write_reg(fos, R_PULSE_WIDTH, width_reg);

   return 0;
}

//
// FOS_Run Test()
//
// Initiate a COTDR, BOTBA or ROTDR test
//
// Function will return immediately after initiating test
// before test completion
//
int FOS_Run_Test(struct fos_drvdata *fos, void *user_ptr)
{
   struct FOS_cmd_struct cmd;

   u32               adc_count_value;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "FOS_Run_Test: copy failed\n");

      return -EFAULT;
   }

   if (FOS_Status(fos) & SPI_BUSY_FLAG)
      return -1;

   if (FOS_Status(fos) & SWEEP_RUNNING_FLAG)
      return -2;

   if (FOS_Status(fos) & TEST_RUNNING_FLAG)
      return -3;

   if ((cmd.config & MODE_BITS) == BOTDA_MODE) {
      // write out BOTDA format data
      adc_count_value = cmd.adc_count + 15;
      adc_count_value /= 16;
   }  else if ((cmd.config & MODE_BITS) == ROTDR_MODE) {
      // write out ROTDR format data
      adc_count_value = cmd.adc_count + 15;
      adc_count_value /= 16;
   }  else if ((cmd.config & MODE_BITS) == COTDR_MODE) {
      // write out COTDR format data
      adc_count_value = cmd.adc_count + 31;
      adc_count_value /= 32;
      adc_count_value *= 2;
   }

   fos_write_reg(fos, R_BOTDA_START_FREQ, cmd.botda_start_freq);
   fos_write_reg(fos, R_BOTDA_END_FREQ, cmd.botda_end_freq);

   fos_write_reg(fos, R_ADC_COUNT, adc_count_value);
   fos_write_reg(fos, R_BOTDA_COUNT, cmd.botda_count);
   fos_write_reg(fos, R_MEM_STRIDE, cmd.row_stride);

   cmd.config &= (MODE_BITS|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE);
   fos->config_state &= ~(MODE_BITS|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE);
   fos->config_state |= cmd.config;
   fos_write_reg(fos, R_CONFIG, fos->config_state);

   if (cmd.run_test != RUN_NO_TEST)
      fos_write_reg(fos, R_RUN_TEST, START_TEST);

   return 0;
}

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
int FOS_Continuous_Scan(struct fos_drvdata *fos, void *user_ptr)
{
   struct FOS_continuous_cmd_struct cmd;
   struct FOS_read_data_struct *read_cmd;

   u32   adc_count_value,dma_size,mode;
   u32   status;

   printk(KERN_DEBUG "ioctl: entered FOS_Continuous scan\n");

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "FOS_Continuous_Scan: copy failed\n");

      return -EFAULT;
   }

   // if stop test then shut off controller and exit
   if(cmd.run_test == 0) {
      fos_write_reg(fos, R_RUN_TEST, STOP_TEST);
      fos_write_reg(fos, R_BOTDA_END_FREQ, REPEAT_COUNT);
      return 0;
   }

   status = FOS_Status(fos);
   printk(KERN_DEBUG "ioctl: FOS_Continuous_scan, status = 0x%x\n",status);

   if (status & SPI_BUSY_FLAG)
      return -1;

   if (status & SWEEP_RUNNING_FLAG)
      return -2;

   if (status & TEST_RUNNING_FLAG)
      return -3;

   // this is set to -1 so the first interrupt increments to 0 when the DMA is started
   // The next interrupt will increment to 1 when the DMA has completed
   fos->dma_block_count = -1;

   mode = cmd.config & MODE_BITS;
   if ((mode != COTDR_MODE) && (mode != COTDR_MODE_ALT) && (mode != COTDR_MODE_DUAL))
      return -4;

   // User can configure these options
   cmd.config &= MODE_BITS|TEST_DATA_ENABLE|SWEEP_TEST_MODE|TEST_CLK_SELECT|SMA_CLK_ENABLE;

   // These options are mandatory
   fos->config_state = DMA_ACTIVE|CLK_200_MODE|cmd.config;

   // write out configuration
   fos_write_reg(fos, R_CONFIG, fos->config_state);

   // write out COTDR format data
   adc_count_value = cmd.adc_count + 31;
   adc_count_value /= 32;
   adc_count_value *= 2;

   // start at address block 0
   fos_write_reg(fos, R_BOTDA_START_FREQ, 0);

   // end at address block 1023
   fos_write_reg(fos, R_BOTDA_END_FREQ, REPEAT_LOOP|INTERRUPT_ACTIVE|REPEAT_COUNT);
   fos_write_reg(fos, R_ADC_COUNT, adc_count_value);

   // no accumulate and add
   fos_write_reg(fos, R_BOTDA_COUNT, 1);

   // use 1Mbyte blocks at the moment
   fos_write_reg(fos, R_MEM_STRIDE, cmd.row_stride);

   // setup read command
   read_cmd = &fos->repeat_read_cmd;
   read_cmd->data = 0;
   read_cmd->dma_active = 1;
   read_cmd->dma_addr_offset = 0;

   if (mode == COTDR_MODE_DUAL)
      read_cmd->datatype = COTDR_DATA_SIZE*2;
   else
      read_cmd->datatype = COTDR_DATA_SIZE;

   read_cmd->column = cmd.start_column;
   read_cmd->num_columns = (cmd.end_column - cmd.start_column);
   read_cmd->row = 0;
   read_cmd->num_rows = 32;
   read_cmd->row_stride = cmd.row_stride;

   // write number of 64-bit transfers to dma size register
   dma_size = (read_cmd->num_rows *read_cmd->num_columns* read_cmd->datatype)>>3;
   fos_write_reg(fos, R_DMA_SIZE, dma_size);
#ifdef DEBUG
  printk(KERN_DEBUG "DMA size = 0x%d = %d 64-bit words\n",dma_size,dma_size);
#endif

   if (cmd.run_test != RUN_NO_TEST) {
      fos_write_reg(fos, R_RUN_TEST, START_TEST);
#ifdef DEBUG
      printk(KERN_DEBUG "started continuous loop test\n");
#endif
   }

   return 0;
}


//
// FOS_Read_Data()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_Read_Data(struct fos_drvdata *fos, struct FOS_read_data_struct *cmd)
{
   u64 addr;
   u32 i;

   u32 size,data_size;
   u32 count;
   u32 column, row, num_columns, num_rows, row_stride, datatype;

   column      = cmd->column;
   row         = cmd->row;
   num_columns = cmd->num_columns;
   num_rows    = cmd->num_rows;
   row_stride  = cmd->row_stride;
   datatype    = cmd->datatype;

   data_size   = 0;
   count       = 0;

   fosdma_write_reg(fos, S2MM_DMACR, 4);

   if (row > 8200) {
#ifdef DEBUG
      printk(KERN_DEBUG "Too many rows = %d\n",row);
#endif
      return -1;
   }

   if ((row+num_rows) > 8200) {
#ifdef DEBUG
      printk(KERN_DEBUG "Too many rows + num_rows = %d\n",row+num_rows);
#endif
      return -1;
   }

   if (datatype == COTDR_DATA_SIZE) {
      data_size = COTDR_DATA_SIZE;
      if (column & 1)
         return -2;
      if (num_columns & 1)
         return -3;
   }
   if (datatype == BOTDA_DATA_SIZE)
      data_size = BOTDA_DATA_SIZE;
   if (datatype == ROTDR_DATA_SIZE)
      data_size = ROTDR_DATA_SIZE;

   if (data_size == 0) {
#ifdef DEBUG
      printk(KERN_DEBUG "data size not valid = %d\n",data_size);
#endif
      return -4;
   }

   // set DMA mode
   if (cmd->dma_active == 1)
      fos->config_state |= DMA_ACTIVE;
   else
      fos->config_state &= ~DMA_ACTIVE;

   fos_write_reg(fos, R_CONFIG, fos->config_state);

   addr = (u64)data_size * (u64)column + (u64)row_stride*(u64)row;
#ifdef DEBUG
   printk(KERN_DEBUG "addr = 0x%llx\n",addr);
#endif


   // size is size of transfer in 64 bit words
   size = (data_size * num_columns * num_rows) >> 3;
#ifdef DEBUG
   printk(KERN_DEBUG "size = %d, data_size = %d, num_columns = %d, addr = 0x%llx\n",size,data_size,num_columns,addr);
#endif

   //
   // loop for each row
   //
   num_columns >>= 5;

#ifdef DEBUG
   printk(KERN_DEBUG "Address for read is 0x%llx, size of request is %d blocks\n",addr,size);
   printk(KERN_DEBUG "stride = 0x%x, num_columns = %d, num_rows = %d, size = %d\n",row_stride,num_columns*32,num_rows,size);
#endif

   fos_write_reg(fos, R_DATA_READ_ADDR, (u32)(addr & 0x00000000ffffffff));
   fos_write_reg(fos, R_DATA_READ_STRIDE, row_stride);
   fos_write_reg(fos, R_DATA_READ_COL_COUNT, (num_columns*data_size/2));
   fos_write_reg(fos, R_DATA_READ_ROW_COUNT, num_rows);
//   fos_write_reg(fos, R_DATA_READ_START, size);

   // if DMA transfer then start controller
   if (cmd->dma_active == 1) {
      // start write to memory (max = 8M - 1), so use 4Mbyte blocks max
#ifdef DEBUG
      printk(KERN_DEBUG "S2MM_DA = 0x%x, S2MM_LENGTH = 0x%x = %d\n",(fos->dma_handle + cmd->dma_addr_offset),(size*8),(size*8));
#endif

      fosdma_write_reg(fos, S2MM_DMACR, 1);
      fosdma_write_reg(fos, S2MM_DA, fos->dma_handle + cmd->dma_addr_offset);
      fosdma_write_reg(fos, S2MM_LENGTH, (size*8));
      fos_write_reg(fos, R_DATA_READ_START, size);
   } else {
      fos_write_reg(fos, R_DATA_READ_START, size);

      //
      // wait until fifo has data
      //
      i = 0;
      while ((FOS_Status(fos) & FIFO_EMPTY_FLAG) && (i < MAX_WAIT_COUNT)) {
         i++;
         if ((i&31) == 31) {
#ifdef DEBUG
            printk(KERN_DEBUG "Failure to get data, RETRY read!!\n");
#endif
            fos_write_reg(fos, R_DATA_READ_START, size);
         }
      }
   }

   return 0;
}

//
// FOS_Read_Frequency()
//
// Read frequency of index (-1 if error)
//
int FOS_Read_Frequency(struct fos_drvdata *fos, void *arg_ptr)
{
   u32 i;
   u32 *tmp;

   tmp = kmalloc(sizeof(u32)*1024,GFP_KERNEL);

   for(i = 0; i<4096; i+=4)
      tmp[i] = readl(fos->base + R_FOS_SWEEP_RAM_BASE + i)&0x000FFFFF;

   if (copy_to_user((u32 *)arg_ptr, tmp, (sizeof(u32)*1024))) {
      kfree(tmp);
      return -EFAULT;
   }
   kfree(tmp);
   return 0;
}

//
// FOS_SPI_Write()
//
// Write a command to SPI port
// 3byte_mode only affects dac port (SPI 2)
//
int FOS_SPI_Write(struct fos_drvdata *fos, void *user_ptr)
{
   u32   i,j,addr;

   struct FOS_spi_cmd_struct  cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "FOS_SPI_Write: copy failed\n");

      return -EFAULT;
   }

   if (cmd.num_spi_writes == 0)
     return 0;

   if (cmd.num_spi_writes > 16)
      return -EFAULT;

   for (j = 0; j < cmd.num_spi_writes; j++) {
      //
      // Wait for SPI access to finish
      //
      i = 0;
      while ((FOS_Status(fos) & SPI_BUSY_FLAG) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif
      if (cmd.port_addr[j] > 6)
         return -EFAULT;

      addr = R_SPI0 + 4*cmd.port_addr[j];
      fos_write_reg(fos, addr, cmd.port_data[j]);

      // wait until SPI write completes
      i = 0;
      while ((FOS_Status(fos) & SPI_BUSY_FLAG) && (i < MAX_WAIT_COUNT))
         i++;

   }
   return 0;
}

//
// FOS_Run_Sweep()
//
// Run a frequency sweep
// cmd determines number of dac values to sweep.
// This is normally 800,65536,or 262144
//
int FOS_Run_Sweep(struct fos_drvdata *fos, void *user_ptr)
{
   u32 i,*tmp,num_sweeps;
   struct FOS_sweep_cmd_struct cmd;

   if (copy_from_user(&cmd, user_ptr, sizeof(cmd))) {
      printk(KERN_DEBUG "FOS_Run_Sweep: copy failed\n");

      return -EFAULT;
   }

   num_sweeps = (cmd.final_dac_value - cmd.initial_dac_value)/cmd.dac_step_size;

   if (FOS_Status(fos) & SPI_BUSY_FLAG)
      return -1;

   if (FOS_Status(fos) & SWEEP_RUNNING_FLAG)
      return -2;

   if (FOS_Status(fos) & TEST_RUNNING_FLAG)
      return -3;

   if (num_sweeps > 1024)
      return -EFAULT;

   tmp = kmalloc(sizeof(u32)*1024,GFP_KERNEL);

   if (num_sweeps == 1024)
      fos->config_state &= ~DATA_SWEEP;
   else
      fos->config_state |= DATA_SWEEP;

   // start sweep test
   fos_write_reg(fos, R_CONFIG, fos->config_state);
   fos_write_reg(fos, R_RUN_TEST, SWEEP_START);

   //
   // Wait until sweep is finished, allow 20 seconds
   //
   i = 0;
   while ((FOS_Status(fos) & SWEEP_RUNNING_FLAG) && (i < 1000)) {
      msleep_interruptible(20);
      i++;
   }
   //
   // read back frequency table
   //
   for (i=0; i<num_sweeps; i++) {
      tmp[i] = readl(fos->base + R_FOS_SWEEP_RAM_BASE + i*4);
   }

   if (copy_to_user(cmd.sweep_data, tmp, (sizeof(u32)*num_sweeps))) {
      kfree(tmp);
      return -EFAULT;
   }
   kfree(tmp);

   return 0;
}

//
// FOS_Read_Dac_Table()
//
// This function reads the DAC table back from the
// FPGA.  It is useful for comparing table values to
// calculated values
//
int FOS_Read_DAC_Table(struct fos_drvdata *fos, u32 *read_data)
{
   u32 i,*dac_table;

   dac_table = kmalloc(sizeof(u32)*1024,GFP_KERNEL);

   for(i = 0; i<1024; i++) {
      dac_table[i] = readl(fos->base + R_FOS_DAC_LOOKUP_BASE + i*4)&0x000FFFFF;
   }

   if (copy_to_user(read_data, dac_table, (sizeof(u32)*1024))) {
      kfree(dac_table);
      return -EFAULT;
   }
   kfree(dac_table);
   return 0;
}

//
// FOS_Write_Dac_Table()
//
// This function writes the DAC table to the FPGA.
// Table values are 20 bit words.  Values > 2^20 are
// written as 0xFFFFF,
//
int FOS_Write_DAC_Table(struct fos_drvdata *fos, u32 *write_data)
{
   u32 i, test, *dac_table;

   dac_table = kmalloc(sizeof(u32)*1024,GFP_KERNEL);

   if (copy_from_user(dac_table, write_data, (sizeof(u32)*1024))) {
      kfree(dac_table);
      return -EFAULT;
   }

   for(i = 0; i<1024; i++) {
      // 18 or 20 bit DAC
      fos_write_reg(fos, (R_FOS_DAC_LOOKUP_BASE + i*4), 0x00100000|(dac_table[i]&0x000FFFFF));

      test = readl(fos->base + R_FOS_DAC_LOOKUP_BASE + i*4);
      if (test != (0x00100000|(dac_table[i]&0x000FFFFF))) {
         kfree(dac_table);
         return -1;
      }
   }
   kfree(dac_table);
   return 0;
}
