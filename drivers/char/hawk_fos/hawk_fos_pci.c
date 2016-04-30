/*
 * Wrapper Driver used to control a two-channel Xilinx DMA Engine
 */
#include <linux/dmaengine.h>

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <xen/page.h>

#include <linux/slab.h>
#include <linux/pci.h>
//#include <linux/platform_device.h>

#include "fos.h"
#include "fos_system.h"

#define DRIVER_NAME "hawk_fos"
#define MODULE_NAME "hawk_fos"
#define FOS_DEVICES 1

static const char fosname[] = "hawk_fos";

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(0x10ee, 0x7042), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

LIST_HEAD( fos_full_dev_list );

static struct fos_drvdata *get_elem_from_list_by_inode(struct inode *i)
{
   struct list_head *pos;
   struct fos_drvdata *fos = NULL;

   list_for_each( pos, &fos_full_dev_list ) {
      struct fos_drvdata *tmp;
      tmp = list_entry( pos, struct fos_drvdata, dev_list );
      if (tmp->devt == i->i_rdev)
      {
         fos = tmp;
         break;
      }
   }
   return fos;
}

static int fos_open(struct inode *i, struct file *filp)
{
   struct fos_drvdata *fos;

   fos = container_of(i->i_cdev, struct fos_drvdata, cdev);

   fos->irq_count = 0;

   printk(KERN_DEBUG "<%s> file: open()\n", MODULE_NAME);
   filp->private_data = fos;
   return 0;
}

static int fos_release(struct inode *i, struct file *f)
{
   struct fos_drvdata *fos;

   fos = container_of(i->i_cdev, struct fos_drvdata, cdev);

   fos_write_reg(fos, R_RUN_TEST, STOP_TEST);
   fos_write_reg(fos, R_BOTDA_END_FREQ, 0);

   printk(KERN_DEBUG "<%s> file: close()\n", MODULE_NAME);
   return 0;
}

static int fos_mmap(struct file *filp, struct vm_area_struct *vma)
{
   struct fos_drvdata *fos = filp->private_data;
   int result;
   unsigned long requested_size;

   requested_size = vma->vm_end - vma->vm_start;

   printk(KERN_DEBUG "<%s> file: mmap()\n", MODULE_NAME);
   printk(KERN_DEBUG
         "<%s> file: memory size reserved: %d, mmap size requested: %lu\n",
         MODULE_NAME, DMA_LENGTH, requested_size);

   if (requested_size > DMA_LENGTH) {
      printk(KERN_DEBUG "<%s> Error: %d reserved != %lu requested)\n",
            MODULE_NAME, DMA_LENGTH, requested_size);

      return -EAGAIN;
   }

   vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

   result = remap_pfn_range(vma, vma->vm_start,fos->dma_handle >> PAGE_SHIFT, requested_size, vma->vm_page_prot);


   if (result) {
      printk(KERN_DEBUG "<%s> Error: in calling remap_pfn_range: returned %d\n",
            MODULE_NAME, result);

      return -EAGAIN;
   }

   printk(KERN_DEBUG "<%s> : mmap complete \n",MODULE_NAME);
   return 0;
}

static ssize_t fos_read(struct file *f, char __user * buf, size_t
      len, loff_t * off)
{
   /* printk(KERN_INFO "<%s> file: read()\n", MODULE_NAME); */
   struct fos_drvdata *fos;
   int      count;
   char     *ptr;

   if (len > 65536)
   {
      return 0;
   }
   fos = get_elem_from_list_by_inode(f->f_inode);

   count = len;
   ptr   = buf;

   while (count > 1024) {
      memcpy(ptr, fos->base + R_FOS_FIFO_BASE, 1024);
      count -= 1024;
      ptr+= 1024;
   }
   memcpy(ptr, fos->base + R_FOS_FIFO_BASE, count);

   return len;
}

static long fos_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct fos_drvdata *fos = filp->private_data;
   //   int __user *ip = (int __user *)arg;
   void  *arg_ptr = (void *)arg;
   long  ret = 0;
   unsigned int s2mm_status;
   struct FOS_read_data_struct read_cmd;
   struct FOS_debug_struct debug_cmd;

   //printk(KERN_DEBUG "<%s> ioctl: entered fos_ioctl\n", MODULE_NAME);

   switch (cmd) {
      case FOS_USER_RESET:
         if (arg & FPGA_RESET) {
            fos_write_reg(fos, R_RUN_TEST, FPGA_RESET);
            fos->config_state = FPGA_RESET;
         } else {
            fos_write_reg(fos, R_RUN_TEST, 0);
            fos->config_state = 0;
         }
         return 0;

      case FOS_USER_SET_CLK:
         if ((arg != 200)&&(arg != 400))
            return -1;

         if (arg == 200)
            fos->config_state |= CLK_200_MODE;
         else
            fos->config_state &= ~CLK_200_MODE;

         fos_write_reg(fos, R_CONFIG, fos->config_state);

         return ret;

      case FOS_USER_SET_PULSE:
         ret = FOS_Set_Pulse(fos, arg_ptr);
         return ret;

      case FOS_USER_SET_ADC_OFFSET:
         fos_write_reg(fos, R_ADC_OFFSET, arg);
         return 0;

      case FOS_USER_SET_ROW_STRIDE:
         fos_write_reg(fos, R_MEM_STRIDE, arg);
         return 0;

      case FOS_USER_RUN_TEST:
         ret = FOS_Run_Test(fos, arg_ptr);
         return ret;

      case FOS_USER_STATUS:
         ret = FOS_Status(fos);
         if (copy_to_user(arg_ptr, &ret, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case FOS_USER_READ_DATA:

         if (copy_from_user(&read_cmd, arg_ptr, sizeof(read_cmd))) {
            printk(KERN_DEBUG "FOS_USER_READ_DATA: copy failed\n");

            return -EFAULT;
         }
         ret = FOS_Read_Data(fos, &read_cmd);
         return ret;

      case FOS_USER_READ_FREQUENCY:
         ret = FOS_Read_Frequency(fos, arg_ptr);
         return ret;

      case FOS_USER_SPI_WRITE:
         ret = FOS_SPI_Write(fos, arg_ptr);
         return ret;

      case FOS_USER_CONFIG_SWEEP:
         if ((arg == FREQUENCY_COUNTER_ACTIVE)||(arg == FREQUENCY_TEST_ACTIVE)||(arg == EOM_ACTIVE)) {
            fos->config_state &= ~(SWEEP_TEST_MODE|TEST_CLK_SELECT);
            fos->config_state |= arg;
            fos_write_reg(fos, R_CONFIG, fos->config_state);
            return 0;
         } else {
            return -1;
         }

      case FOS_USER_RUN_SWEEP:
         ret = FOS_Run_Sweep(fos, arg_ptr);
         return ret;

      case FOS_USER_READ_DAC_TABLE:
         ret =  FOS_Read_DAC_Table(fos, (u32 *)arg_ptr);
         return ret;

      case FOS_USER_WRITE_DAC_TABLE:
         ret = FOS_Write_DAC_Table(fos, (u32 *)arg_ptr);
         return ret;

      case FOS_INTERRUPT_COUNT:
         if (copy_to_user(arg_ptr, &fos->irq_count, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case FOS_DMA_BLOCK_COUNT:
         if (copy_to_user(arg_ptr, &fos->dma_block_count, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case FOS_CONTINUOUS_SCAN:
         ret = FOS_Continuous_Scan(fos, arg_ptr);
         return ret;

      case FOS_DMA_STATUS:
         s2mm_status = fosdma_read_reg(fos,S2MM_DMASR);
         if (copy_to_user(arg_ptr, &s2mm_status, sizeof(u32))) {
            return -EFAULT;
         }
         return 0;

      case FOS_DMA_TEST:
         if (arg >= 0x800000)
            return -EFAULT;

         // set dma into loopback mode
         fos_write_reg(fos, R_CONFIG, (fos->config_state|DMA_DEBUG));

         // start read from memory
         fosdma_write_reg(fos, MM2S_DMACR, 1);
         fosdma_write_reg(fos, MM2S_SA, fos->dma_handle);
         fosdma_write_reg(fos, MM2S_LENGTH, arg);

         // start write to memory
         fosdma_write_reg(fos, S2MM_DMACR, 1);
         fosdma_write_reg(fos, S2MM_DA, (fos->dma_handle + (DMA_LENGTH/2)));
         fosdma_write_reg(fos, S2MM_LENGTH, arg);

         printk(KERN_DEBUG "<%s> : started dma \n",MODULE_NAME);

         s2mm_status = fosdma_read_reg(fos,S2MM_DMASR);
         while(!(s2mm_status & 1<<12) && !(s2mm_status & 1<<1) && !(s2mm_status & 1<<0)){
            s2mm_status = fosdma_read_reg(fos,S2MM_DMASR);
         }

         printk(KERN_DEBUG "<%s> :  dma status = 0x%x\n",MODULE_NAME,s2mm_status);

         // set configuration back to original state
         fos_write_reg(fos, R_CONFIG, fos->config_state);

         return 0;

      case FOS_REG_DEBUG:

         if (copy_from_user(&debug_cmd, arg_ptr, sizeof(debug_cmd))) {
            printk(KERN_DEBUG "FOS_REG_DEBUG: copy failed\n");

            return -EFAULT;
         }

         if (debug_cmd.cmd == FOS_DEBUG_WRITE){
            fos_write_reg(fos, debug_cmd.reg, debug_cmd.data);
            return 0;
         }

         if (debug_cmd.cmd == FOS_DEBUG_DMA_WRITE) {
            fosdma_write_reg(fos, debug_cmd.reg, debug_cmd.data);
            return 0 ;
         }

         if (debug_cmd.cmd == FOS_DEBUG_READ)
            debug_cmd.data = fos_read_reg(fos, debug_cmd.reg);

         if (debug_cmd.cmd == FOS_DEBUG_DMA_READ)
            debug_cmd.data = fosdma_read_reg(fos, debug_cmd.reg);

         if (copy_to_user(arg_ptr, &debug_cmd, sizeof(debug_cmd))) {
            return -EFAULT;
         }
         return 0;

      default:
         break;
   }

   return ret;
}

/**
 * fos_isr() - The main interrupt handler.
 * @irq:	The interrupt number.
 * @data:	Pointer to the driver data structure.
 * returns: IRQ_HANDLED after the interrupt is handled.
 **/
irqreturn_t fos_isr(int irq, void *data)
{
   struct fos_drvdata *fos = data;
   u32   dma_block,current_row;
   struct FOS_read_data_struct *read_cmd;

   spin_lock(&fos->lock);

   // clear interrupt
   fos_write_reg(fos, R_BOTDA_END_FREQ, REPEAT_LOOP|INTERRUPT_ACTIVE|REPEAT_COUNT);

   fos->irq_count++;

   read_cmd = &fos->repeat_read_cmd;
   // Start DMA read
//   fos->dma_block_count++;
//   dma_block = fos->dma_block_count & 0x7;

   current_row = fos_read_reg(fos,R_FREQUENCY_STATUS);

   dma_block = current_row >> 7;

   fos->dma_block_count = dma_block;

   printk(KERN_DEBUG "<%s> :  interrupt : current_row = %d, current_block = %d\n",MODULE_NAME,current_row,dma_block);

   // set rows to read
   read_cmd->row = 128 * dma_block;

   // set address offset in command
   read_cmd->dma_addr_offset = dma_block * (DMA_LENGTH>>3);
   if (FOS_Read_Data(fos, read_cmd)) {
      // failed so shut off interrupt
      fos_write_reg(fos, R_BOTDA_END_FREQ, REPEAT_COUNT);
   }

   spin_unlock(&fos->lock);

   return IRQ_HANDLED;
}

static const struct file_operations fos_fops = {
   .owner            = THIS_MODULE,
   .read             = fos_read,
   .mmap             = fos_mmap,
   .unlocked_ioctl   = fos_ioctl,
   .open             = fos_open,
   .release          = fos_release,
};


//
// comment out platform driver support
//

//static const struct of_device_id fos_of_match_table[] = {
//   { .compatible = "kutu,axi4-fos-controller-1.00-a", (void *)&fos_fops },
//   { },
//};
//MODULE_DEVICE_TABLE(of, fos_of_match_table);

static int fos_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
   struct fos_drvdata *fos;
   dev_t devt;
   int ret;
   struct device *dev;

	/* enable the PCI device */
   ret =	pci_enable_device(pdev);

   dev = &pdev->dev;

	if (ret) {
		dev_err(dev,"pci_enable_device() failed. Aborting.\n");
		return ret;
	} else {
      dev_err(dev,"pci_enable_device() succeeded.\n");
   }

   // allocate private memory
	fos = devm_kzalloc(&pdev->dev, sizeof(*fos), GFP_KERNEL);
	if (!fos)
		return -ENOMEM;

   dev_info(&pdev->dev, "Hawk FOS trying to get PCI resources\n");

   // set private memory pointer
   pci_set_drvdata(pdev, fos);

   // Get info about BAR0 and sanity check
   ret = pci_resource_flags(pdev, 0);
	if ((ret & IORESOURCE_MEM) != IORESOURCE_MEM) {
		dev_err(dev, "Incorrect BAR0 configuration. return value = %d\n",ret);
		return -ENODEV;
	} else {
		dev_err(dev, "BAR0 configuration is correct.\n");
	}

   // request PCI regions
   ret = pci_request_regions(pdev, "hawk_fos");
	if (ret) {
		dev_err(dev, "Can't request BAR regions, return value = %d\n",ret);
      return -ENODEV;
	} else {
      dev_err(dev, "pci_request_regions() succeeded.\n");
   }

   ret = pci_resource_start(pdev, 0);
   dev_err(dev, "BAR0 is located at physical address 0x%x\n",ret);

   // base register is BAR0
   fos->base = pci_iomap(pdev,0,pci_resource_len(pdev, 0));

   fos->config_state = 0;

   dev_info(&pdev->dev, "Hawk FOS finished call to PCI get resources\n");

   // Setup IRQ
   dev_info(&pdev->dev, "Hawk FOS trying to allocate legacy irq\n");

   fos->irq = pdev->irq;
   ret = devm_request_irq(&pdev->dev, pdev->irq, fos_isr, 0, fosname, fos);
   if (ret) {
      dev_err(&pdev->dev, "Failed to register IRQ");
      return ret;
   }

   dev_info(&pdev->dev, "Hawk FOS successfully setup irq\n");

   spin_lock_init(&fos->lock);
   mutex_init(&fos->mutex);

   fos->is_open = 0;
   fos->dma_done = 0;
   fos->error_status = 0;

   //
   // reset FOS device
   //

   ret = alloc_chrdev_region(&devt, 0, FOS_DEVICES, DRIVER_NAME);
   if (ret < 0)
      goto failed5;
   dev_info(&pdev->dev, "Successfully allocated chrdev region\n");

   fos->devt = devt;

   cdev_init(&fos->cdev, &fos_fops);
   fos->cdev.owner = THIS_MODULE;
   ret = cdev_add(&fos->cdev, devt, 1);
   if (ret) {
      dev_err(&pdev->dev, "cdev_add() failed\n");
      goto failed6;
   }

   fos->class = class_create(THIS_MODULE, DRIVER_NAME);
   if (IS_ERR(fos->class)) {
      dev_err(&pdev->dev, "failed to create class\n");
      goto failed6;
   }

   dev = device_create(fos->class, &pdev->dev, devt, fos, DRIVER_NAME);
   if (IS_ERR(dev)) {
      dev_err(&pdev->dev, "unable to create device\n");
      goto failed7;
   }
   dev_info(&pdev->dev, "Successfully created device\n");

   //
   // allocate mmap area
   //
/*
   fos->dma_addr = dma_alloc_coherent(NULL, DMA_LENGTH, &fos->dma_handle, GFP_KERNEL);

   dev_info(&pdev->dev, "dma_addr = 0x%x, dma_handle = 0x%x\n",(u32)fos->dma_addr,(u32)fos->dma_handle);
   dev_info(&pdev->dev, "fos base = 0x%x, dma base = 0x%x\n",(u32)fos->base,(u32)fos->dma_base);

   if (!fos->dma_addr) {
      printk(KERN_ERR "<%s> Error: allocating dma memory failed\n", MODULE_NAME);

      ret = -ENOMEM;
      goto failed8;
   }
   dev_info(&pdev->dev, "Successfully allocated dma memory\n");

   dev_info(&pdev->dev, "Hawk FOS finished loading driver\n");
*/
   return 0;

//failed8:
//   device_destroy(fos->class, fos->devt);
failed7:
   class_destroy(fos->class);
failed6:
   /* Unregister char driver */
   unregister_chrdev_region(devt, FOS_DEVICES);
failed5:

   return ret;
}

static void fos_remove(struct pci_dev *pdev)
{
   struct fos_drvdata *fos;

   fos = pci_get_drvdata(pdev);

   unregister_chrdev_region(fos->devt, FOS_DEVICES);

   //	sysfs_remove_group(&pdev->dev.kobj, &fos_attr_group);

   device_destroy(fos->class, fos->devt);
   class_destroy(fos->class);
   cdev_del(&fos->cdev);
   clk_unprepare(fos->clk);

   /* free mmap area */
   if (fos->dma_addr) {
      dma_free_coherent(NULL, DMA_LENGTH, fos->dma_addr, fos->dma_handle);
   }
}


//
// comment out platform driver support
//

//static struct platform_driver fos_driver = {
//   .probe = fos_probe,
//   .remove = fos_remove,
//   .driver = {
//      .name = "fos",
//      .of_match_table = fos_of_match_table,
//   },
//};
//module_platform_driver(fos_driver);


static struct pci_driver pci_driver = {
	.name = fosname,
	.id_table = ids,
	.probe = fos_probe,
	.remove = fos_remove,
};

static int __init pci_hawk_fos_init(void)
{
	return pci_register_driver(&pci_driver);
}

static void __exit pci_hawk_fos_exit(void)
{
	pci_unregister_driver(&pci_driver);
}

module_init(pci_hawk_fos_init);
module_exit(pci_hawk_fos_exit);

MODULE_AUTHOR("Greg Smart <Greg.Smart@kutu.com.au>");
MODULE_DESCRIPTION("Hawk FOS Linux PCIE driver");
MODULE_LICENSE("GPL v2");
