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

   adc_count_value = cmd.adc_count;
   if (((cmd.config & MODE_BITS) == BOTDA_MODE)||((cmd.config & MODE_BITS) == BOTDA_MODE_ALT)) {
      // write out BOTDA format data
      adc_count_value = cmd.adc_count + 15;
      adc_count_value /= 16;
   }  else if ((cmd.config & MODE_BITS) == ROTDR_MODE) {
      // write out ROTDR format data
      adc_count_value = cmd.adc_count + 15;
      adc_count_value /= 16;
   }  else if (((cmd.config & MODE_BITS) == COTDR_MODE)||((cmd.config & MODE_BITS) == COTDR_MODE_ALT)||((cmd.config & MODE_BITS) == COTDR_MODE_DUAL)) {
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

   cmd.config &= (MODE_BITS|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE|CLK_200_MODE);
   fos->config_state &= ~(MODE_BITS|TEST_DATA_ENABLE|DAC_UPDATE_ENABLE|CLK_200_MODE);
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
   fos_write_reg(fos, R_MIG2HOST_COL_COUNT, dma_size);
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
// FOS_tfer_mig2host()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_tfer_mig2host(struct fos_drvdata *fos, struct FOS_transfer_data_struct *cmd)
{
   u32 transfer_size,last_addr;
   u32 num_columns, num_rows, row_stride;
   u64 host_addr;

   num_columns = cmd->num_columns & 0xffffffc0;
   if (num_columns != cmd->num_columns) {
      printk(KERN_DEBUG "columns not aligned to 64 bytes = %d\n",cmd->num_columns);
      return -1;
   }

   num_rows    = cmd->num_rows;

   transfer_size = num_rows * num_columns;
   last_addr = cmd->host_offset_addr + transfer_size;

   if (last_addr > DMA_LENGTH) {
      printk(KERN_DEBUG "Transfer range outside of allowable range, start = 0x%08x, end offset - 0x%08x\n",cmd->host_offset_addr,last_addr);
      return -2;
   }

   row_stride  = cmd->mig_stride;

   if (transfer_size == 0) {
      printk(KERN_DEBUG "transfer size not valid = %d\n",transfer_size);
      return -3;
   }

   host_addr = fos->dma_handle;
   host_addr += cmd->host_offset_addr;

#ifdef DEBUG
   printk(KERN_DEBUG "MIG ddress for mig2host transfer is 0x%08x, size of request is %d bytes\n",cmd->mig_base_address,transfer_size);
   printk(KERN_DEBUG "stride = 0x%x, num_columns = %d, num_rows = %d\n",row_stride,num_columns,num_rows);
   printk(KERN_DEBUG "Host address for mig2host transfer is 0x%llx\n",host_addr);
#endif

   // if DMA transfer then start controller
   fos_write_reg(fos, R_MIG2HOST_READ_ADDR, cmd->mig_base_address);
   fos_write_reg(fos, R_MIG2HOST_STRIDE, row_stride);
   fos_write_reg(fos, R_MIG2HOST_COL_COUNT, num_columns);
   fos_write_reg64(fos, R_MIG2HOST_WRITE_ADDR, host_addr);
//   fos_write_reg(fos, R_MIG2HOST_WRITE_ADDR, fos->dma_handle + cmd->host_offset_addr);
//   fos_write_reg(fos, R_MIG2HOST_WRITE_ADDR_HI, 0);
   fos_write_reg(fos, R_MIG2HOST_ROW_COUNT, num_rows);

   return 0;
}

//
// FOS_transfer_to_user()
//
// transfer data array to user space
//
int FOS_transfer_to_user(struct fos_drvdata *fos, struct FOS_transfer_data_struct *cmd)
{
   u32 *user_addr,addr_offset,row_stride;
   u32 index_min,index_max,index_last;

   index_min = 10000000;
   index_max = 0;

   // calculate number areas to transfer, and columns needed
   for(i=0; i<MAX_DATA_AREAS; i++) {

      // check if message is valid
      if ((cmd->list_next[i] != FOS_LINKED_LIST_NEXT) && (cmd->list_next[i] != FOS_LINKED_LIST_LAST))
         return -1;

      if (cmd->first_column[i] < index_min)
         index_min = cmd->first_column[i];

      if (cmd->last_column[i] > index_max)
         index_max = cmd->last_column[i];

      if (cmd->list_next[i] == FOS_LINKED_LIST_LAST) {
         index_last = i;
         i = MAX_DATA_AREAS;
      }
   }

   // align transfer to 32 column boundaries
   index_min &= 0xffffffe0;
   index_max += 0x1f;
   index_max &= 0xffffffe0;






   num_columns = cmd->num_columns & 0xffffffc0;
   if (num_columns != cmd->num_columns) {
      printk(KERN_DEBUG "columns not aligned to 64 bytes = %d\n",cmd->num_columns);
      return -1;
   }

   num_rows    = cmd->num_rows;

   transfer_size = num_rows * num_columns;
   last_addr = cmd->host_offset_addr + transfer_size;

   if (last_addr > DMA_LENGTH) {
      printk(KERN_DEBUG "Transfer range outside of allowable range, start = 0x%08x, end offset - 0x%08x\n",cmd->host_offset_addr,last_addr);
      return -2;
   }

   row_stride  = cmd->mig_stride;

   if (transfer_size == 0) {
      printk(KERN_DEBUG "transfer size not valid = %d\n",transfer_size);
      return -3;
   }

   host_addr = fos->dma_handle;
   host_addr += cmd->host_offset_addr;

#ifdef DEBUG
   printk(KERN_DEBUG "MIG ddress for mig2host transfer is 0x%08x, size of request is %d bytes\n",cmd->mig_base_address,transfer_size);
   printk(KERN_DEBUG "stride = 0x%x, num_columns = %d, num_rows = %d\n",row_stride,num_columns,num_rows);
   printk(KERN_DEBUG "Host address for mig2host transfer is 0x%llx\n",host_addr);
#endif

   // if DMA transfer then start controller
   fos_write_reg(fos, R_MIG2HOST_READ_ADDR, cmd->mig_base_address);
   fos_write_reg(fos, R_MIG2HOST_STRIDE, row_stride);
   fos_write_reg(fos, R_MIG2HOST_COL_COUNT, num_columns);
   fos_write_reg64(fos, R_MIG2HOST_WRITE_ADDR, host_addr);
//   fos_write_reg(fos, R_MIG2HOST_WRITE_ADDR, fos->dma_handle + cmd->host_offset_addr);
//   fos_write_reg(fos, R_MIG2HOST_WRITE_ADDR_HI, 0);
   fos_write_reg(fos, R_MIG2HOST_ROW_COUNT, num_rows);

   return 0;
}

//
// FOS_tfer_host2mig()
//
// Read a 2-dimensional block of data from test
// data is available immediately
// -1 is returned if error, number of bytes is returned if ok.
//
int FOS_tfer_host2mig(struct fos_drvdata *fos, struct FOS_transfer_data_struct *cmd)
{
   u32 transfer_size,last_addr;
   u32 num_columns, num_rows, row_stride;
   u64 host_addr;

   num_columns = cmd->num_columns & 0xffffffc0;
   if (num_columns != cmd->num_columns) {
      printk(KERN_DEBUG "columns not aligned to 64 bytes = %d\n",cmd->num_columns);
      return -1;
   }

   num_rows    = cmd->num_rows;

   transfer_size = num_rows * num_columns;
   last_addr = cmd->host_offset_addr + transfer_size;

   if (last_addr > DMA_LENGTH) {
      printk(KERN_DEBUG "Transfer range outside of allowable range, start = 0x%08x, end offset - 0x%08x\n",cmd->host_offset_addr,last_addr);
      return -2;
   }

   row_stride  = cmd->mig_stride;

   if (transfer_size == 0) {
      printk(KERN_DEBUG "transfer size not valid = %d\n",transfer_size);
      return -3;
   }

   host_addr = fos->dma_handle;
   host_addr += cmd->host_offset_addr;
#ifdef DEBUG
   printk(KERN_DEBUG "MIG Address for host2mig transfer is 0x%08x, size of request is %d bytes\n",cmd->mig_base_address,transfer_size);
   printk(KERN_DEBUG "stride = 0x%x, num_columns = %d, num_rows = %d\n",row_stride,num_columns,num_rows);
   printk(KERN_DEBUG "Host address for host2mig transfer is 0x%llx\n",host_addr);
#endif

   // if DMA transfer then start controller
   fos_write_reg(fos, R_HOST2MIG_WRITE_ADDR, cmd->mig_base_address);
   fos_write_reg(fos, R_HOST2MIG_STRIDE, row_stride);
   fos_write_reg(fos, R_HOST2MIG_COL_COUNT, num_columns);
//   fos_write_reg64(fos, R_HOST2MIG_READ_ADDR, host_addr);
   fos_write_reg(fos, R_HOST2MIG_READ_ADDR, (u32)(host_addr & 0xffffffff));
   fos_write_reg(fos, R_HOST2MIG_READ_ADDR_HI, (u32)(host_addr >> 32));
   fos_write_reg(fos, R_HOST2MIG_ROW_COUNT, num_rows);

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
   u32   i,j,addr,data,num_bytes;

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
      num_bytes = cmd.num_bytes[j] << 24;
      data = cmd.port_data[j] & 0xffffff;
      data |= num_bytes;
      fos_write_reg(fos, addr, data);
      printk(KERN_DEBUG "Wrote register 0x%x with val = 0x%x\n",addr,data);
      // wait until SPI write completes
      i = 0;
      while ((FOS_Status(fos) & SPI_BUSY_FLAG) && (i < MAX_WAIT_COUNT))
         i++;
#ifdef DEBUG
      printk(KERN_DEBUG "Looped through SPI wait %d times\n",i);
#endif

      // readback data

      data = fos_read_reg(fos,R_FOS_SPI_READ_BASE);
      cmd.port_data[j] = data;
      printk(KERN_DEBUG "Read back 0x%x from SPI port\n",data);

   }

   if (copy_to_user(user_ptr, &cmd, sizeof(cmd))) {
      printk(KERN_DEBUG "FOS_SPI_Write: copy_to_user failed\n");
      return -EFAULT;
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
