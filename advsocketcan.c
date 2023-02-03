//******************************************************************************
//
// Copyright (c) 2022 Advantech Industrial Automation Group.
//
// Advantech serial CAN card socketCAN driver
// 
// This program is free software; you can redistribute it and/or modify it 
// under the terms of the GNU General Public License as published by the Free 
// Software Foundation; either version 2 of the License, or (at your option) 
// any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT 
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for 
// more details.
// 
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc., 59 
// Temple Place - Suite 330, Boston, MA  02111-1307, USA.
// 
//
//
//******************************************************************************


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/string.h>
#include "sja1000.h"


MODULE_AUTHOR(" Wang.guigui ");
MODULE_DESCRIPTION("SocketCAN driver for Advantech CAN cards");
MODULE_LICENSE("GPL");
static char *serial_version = "3.0.0.4";
static char *serial_revdate = "2022/06/30";

#define CAN_Clock (16000000 / 2) //16000000 : crystal frequency

#define Fpga_Bar2  2    //Bar2
#define Bar0       0    //Bar0

#define sja1000_CDR             (CDR_CLKOUT_MASK |CDR_CBP )
#define sja1000_OCR             (OCR_TX0_PULLDOWN | OCR_TX0_PULLUP)
#define ADVANTECH_VANDORID	0x13FE

static unsigned int cardnum = 1;
static unsigned int portsernum = 0;
static void advcan_pci_remove_one(struct pci_dev *pdev);
static int advcan_pci_init_one(struct pci_dev *pdev,const struct pci_device_id *ent);

static struct pci_device_id advcan_board_table[] = {
   {ADVANTECH_VANDORID, 0x1680, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0x3680, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0x2052, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0x1681, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc002, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc101, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc102, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc104, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc201, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc202, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc204, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc301, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc302, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0xc304, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},
   {ADVANTECH_VANDORID, 0x00c5, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},//device id 0x00c5 is add for the device support DMA
   {ADVANTECH_VANDORID, 0x00D7, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},//device id 0x00d7 is add for B-version hardware
   {ADVANTECH_VANDORID, 0x00F7, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PCI_ANY_ID},//device id 0x00f7 is add for B-version hardware legacy irq + No TXFIFO
   {0,}			
};

static struct pci_driver can_socket_pci_driver = {
   .name = "advcan_pci_socket",
   .id_table = advcan_board_table,
   .probe = advcan_pci_init_one,
   .remove = advcan_pci_remove_one,
};


static u8 advcan_read_io(const struct sja1000_priv *priv, int port)
{
   return inb((unsigned long)priv->reg_base + port);
}

static void advcan_write_io(const struct sja1000_priv *priv,int port, u8 val)
{
   outb(val,(unsigned long) (priv->reg_base + port));
}

static u8 advcan_read_mem(const struct sja1000_priv *priv, int port)//port means reg offset,not port
{
   return ioread8(priv->reg_base + (port<<2));
}

static void advcan_write_mem(const struct sja1000_priv *priv,int port, u8 val)//port means reg offset,not port
{
   iowrite8(val, priv->reg_base + (port<<2));
}


static inline int check_CAN_chip(const struct sja1000_priv *priv)
{
   unsigned char flg;

   priv->write_reg(priv, REG_MOD, 1);//enter reset mode
   priv->write_reg(priv, REG_CDR, CDR_PELICAN);

   mdelay(2);

   flg = priv->read_reg(priv, REG_CDR);//check enter reset mode

   if (flg == CDR_PELICAN) return 1;

   return 0;
}


static int advcan_pci_init_one(struct pci_dev *pdev,const struct pci_device_id *ent)
{	
   unsigned int portNum;
   u64 address;
   unsigned int bar,barFlag,offset,len;
   int err;
   struct advcan_pci_card *devExt;
   struct sja1000_priv *priv;
   struct net_device *dev;
   int dma_spted = 0;
   int txFIFOSpted = 0;
   unsigned int lenBar2 = 0;
   void __iomem *Bar2VirtualAddr = NULL;
   u32 fpga_ver = 0;

   portNum = 0;
   bar = 0;
   barFlag = 0;
   offset = 0x100;

   DEBUG_INFO("******** advcan_pci_init_one\n");

   if (pci_enable_device(pdev) < 0) {
      dev_err(&pdev->dev, "initialize device failed \n");
      return -ENODEV;
   }

   devExt = kzalloc(sizeof(struct advcan_pci_card), GFP_KERNEL);
   if (devExt == NULL) {
      dev_err(&pdev->dev, "allocate memory failed\n");
      pci_disable_device(pdev);
      return -ENOMEM;
   }

   pci_set_drvdata(pdev, devExt);

   if (pdev->device == 0xc001
      || pdev->device == 0xc002
      || pdev->device == 0xc004
      || pdev->device == 0xc101
      || pdev->device == 0xc102
      || pdev->device == 0xc104) {
         portNum = pdev->device & 0xf;
         len = 0x100;
   }
   else if (pdev->device == 0xc201
      || pdev->device == 0xc202
      || pdev->device == 0xc204
      || pdev->device == 0xc301
      || pdev->device == 0xc302
      || pdev->device == 0xc304 
      || pdev->device == 0x00c5
      || pdev->device == 0x00D7
      || pdev->device == 0x00F7) {
         if (pdev->device == 0x00c5 || pdev->device == 0x00D7 || pdev->device == 0x00F7) {
            portNum = 2;
         }
         else {
            portNum = pdev->device & 0xf;
         }

         offset = 0x400;
         len = 0x400;
   }
   else {
      if (pdev->device == 0x1680
         || pdev->device == 0x3680
         || pdev->device == 0x2052) {
            portNum = 2;
            bar = 2;
            barFlag = 1;
            offset = 0x0;
      }
      else if (pdev->device == 0x1681) {
         portNum = 1;
         bar = 2;
         barFlag = 1;
         offset = 0x0;
      }
      len = 128;
   }

   devExt->pci_dev		= pdev;
   devExt->cardnum		= cardnum;
   devExt->portNum		= portNum;

   DEBUG_INFO("#####deviceID:0x%0x\n",pdev->device);

   // Get FPGA version
   lenBar2 = pci_resource_len(pdev, Fpga_Bar2);
   if (lenBar2 > 0) {
      u64 fpga_base = pci_resource_start(pdev, Fpga_Bar2);
      DEBUG_INFO("------Bar2_fpga_base:0x%0llx,len:%d\n",fpga_base,lenBar2);

      if( request_mem_region(fpga_base , lenBar2, "myadvcan") == NULL ) {
         DEBUG_INFO ("request_mem_region error for BAR2\n");   
         goto error_out;
      }

      Bar2VirtualAddr = ioremap(fpga_base, lenBar2);

      if (Bar2VirtualAddr == NULL) {
         DEBUG_INFO ("memory map error for BAR2\n");
         goto error_out;
      }

      fpga_ver = ioread32(Bar2VirtualAddr);
      printk("FPGA Version:0x%0x\n",fpga_ver);
      if (fpga_ver >= 0x00010000) {//version >=  v0.1.0.0,support dma
         dma_spted = 1;
      }
      if (fpga_ver >= 0x00020000) {
         txFIFOSpted = 1;
      }

      iounmap(Bar2VirtualAddr);
      release_mem_region(fpga_base,lenBar2);
   }

   if (dma_spted) {//MSI must enable only once for each card,otherwise it will cause a kernal crash
#ifdef CONFIG_PCI_MSI
      pci_enable_msi(pdev);
      pci_set_master(pdev);//for Rx DMA Or MSI interrupt,must enable bus master
#endif
   }

   for (int i = 0; i < devExt->portNum; i++) {
      address = pci_resource_start(pdev, bar)+ offset * i ;//device ID 0xc302  bar0
      devExt->Base[i] = address;
      devExt->addlen[i] = len;

      dev = adv_alloc_sja1000dev(sizeof(struct advcan_pci_card));
      if (dev == NULL) {
         goto error_out;
      }

      devExt->net_dev[i] = dev;
      priv = netdev_priv(dev);//point to struct sja1000_priv that alloc before
      priv->priv = devExt;
      priv->irq_flags = IRQF_SHARED;

      priv->serialNo = portsernum;

      printk("---------init one port;;;;priv->serialNo:%d\n",priv->serialNo);

      //and dma support,FIFO support
      priv->dma_spted = dma_spted;
      priv->tx_fifo_spted = txFIFOSpted;
      //end
      dev->irq = pdev->irq;

      if(pdev->device == 0xc201
         ||  pdev->device == 0xc202
         ||  pdev->device == 0xc204
         ||  pdev->device == 0xc301
         ||  pdev->device == 0xc302
         ||  pdev->device == 0xc304 
         ||  pdev->device == 0x00c5
         ||  pdev->device == 0x00D7
         ||  pdev->device == 0x00F7) {//Memory
            if(request_mem_region(devExt->Base[i] , devExt->addlen[i], "advcan") == NULL ) {
               DEBUG_INFO ("memory map error\n");   
               goto error_out;
            }
            priv->read_reg  = advcan_read_mem;
            priv->write_reg = advcan_write_mem;
            devExt->Base_mem[i] = ioremap(devExt->Base[i], devExt->addlen[i]);

            if (devExt->Base_mem[i] == NULL) {
               DEBUG_INFO ("ioremap error\n");
               goto error_out;
            }
            priv->reg_base = devExt->Base_mem[i];//priv->reg_base is virtual address mapped by ioremap
      }
      else {//IO
         if ( request_region(address, len, "advcan") == NULL ) {
            DEBUG_INFO ("IO map error\n"); 
            goto error_out;
         }
         priv->read_reg  = advcan_read_io;
         priv->write_reg = advcan_write_io;

         priv->reg_base = (void *) (address);
      }
      if (barFlag) {
         bar++ ;
      }

      if (check_CAN_chip(priv)) {
         printk("Check channel OK!\n");

         priv->can.clock.freq = CAN_Clock;
         priv->ocr = sja1000_OCR;
         priv->cdr = sja1000_CDR;
         SET_NETDEV_DEV(dev, &pdev->dev);

         /* Register SJA1000 device */
         err = adv_register_sja1000dev(dev);
         if (err) {
            dev_err(&pdev->dev, "Registering device failed (err=%d)\n", err);
            adv_free_sja1000dev(dev);
            goto error_out;
         }
      } 
      else {
         adv_free_sja1000dev(dev);
      }
      portsernum++;
   }

   cardnum++;

   return 0;

error_out:
   DEBUG_INFO("init card error\n");
   advcan_pci_remove_one(pdev);
   return -EIO;;
}


static void advcan_pci_remove_one(struct pci_dev *pdev)
{
   struct net_device *dev;
   struct advcan_pci_card *devExt = pci_get_drvdata(pdev);
   struct sja1000_priv *priv = NULL;

   for (int i = 0; i < devExt->portNum; i++) {
      dev = devExt->net_dev[i];
      if (!dev) {
         continue;
      }
      priv = netdev_priv(dev);
      if (!priv) {
         continue;
      }
      if (priv->dma_spted) {
#ifdef	CONFIG_PCI_MSI
         pci_disable_msi(pdev);
#endif
      }

      dev_info(&pdev->dev, "Removing %s.\n", dev->name);

      adv_unregister_sja1000dev(dev);
      adv_free_sja1000dev(dev);

      if ( pdev->device == 0xc201
         || pdev->device == 0xc202
         || pdev->device == 0xc204
         || pdev->device == 0xc301
         || pdev->device == 0xc302
         || pdev->device == 0xc304 
         || pdev->device == 0x00c5
         || pdev->device == 0x00D7
         || pdev->device == 0x00F7) {	 
            iounmap(devExt->Base_mem[i]);
            release_mem_region(devExt->Base[i], devExt->addlen[i]);
      }
      else {
         release_region(devExt->Base[i], devExt->addlen[i]);		
      }
   }

   kfree(devExt);
   pci_disable_device(pdev);
   pci_set_drvdata(pdev, NULL);
}


static int __init advcan_socket_pci_init(void)
{
   printk("\n");

   printk("=============================================================");
   printk("====\n");
   printk("Advantech SocketCAN Drivers. V%s [%s]\n", serial_version, serial_revdate);
   printk(" ----------------init----------------\n");
   printk("=============================================================");
   printk("====\n");

   return pci_register_driver(&can_socket_pci_driver);
}

static void __exit advcan_socket_pci_exit(void)
{
   DEBUG_INFO(" -----------------exit----------------\n");
   pci_unregister_driver(&can_socket_pci_driver);
}

module_init(advcan_socket_pci_init);
module_exit(advcan_socket_pci_exit);

