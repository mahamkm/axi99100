/*
 *  linux/drivers/serial/99100.c
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This code is modified to support ASIX 99100 series serial devices
 */

#include <linux/version.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif

#if defined(CONFIG_SERIAL_99xx_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/console.h>
#include <linux/sysrq.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
#include <linux/mca.h>
#endif

#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/8250_pci.h>
#include <linux/interrupt.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/netlink.h>
#include <net/sock.h>

#include "ax99100_spi.h"
#include "ax99100_sp.h"
#include "ioctl.h"



static char version_spi[] =

KERN_INFO "ASIX AX99100 PCIe Bridge to SPI:v" DRV_VERSION

//	" " __TIME__ " " __DATE__ "\n"

	"    http://www.asix.com.tw\n";


#if 0
#define DEBUG(fmt...)	printk(KERN_ERR fmt)
#else
#define DEBUG(fmt...)	;
#endif
/* ================================================================ */
int spi_suspend_count;
struct cdev 	spi;

static struct class* spi_class = NULL;

struct sock *nl_sk = NULL;
extern struct net init_net;
struct sk_buff *skb;

struct spi_99100 {
	unsigned int		dev_major;
	unsigned int		dev_minor;
	
	unsigned long		iobase0;			// bar0
	unsigned char __iomem	*membase[2];			// 0: bar1 1:bar5
	resource_size_t		mapbase[2];			// for ioremap
	

	unsigned int		irq;

	
	char 	*		tx_dma_v;			//Virtual Address of DMA Buffer for TX
	dma_addr_t 		tx_dma_p;			//Physical Address of DMA Buffer for TX
	char 	*		rx_dma_v;			//Virtual Address of DMA Buffer for RX
	dma_addr_t 		rx_dma_p;			//Physical Address of DMA Buffer for RX
  
	int			tool_pid;
	
	
};
/* IOCTL*/
PSPI_REG	reg;
PMMAP_SPI_REG	reg_m;
PSPI_DMA 	dma;
static struct spi_99100 spi99100;
/* ================================================================ */

/* memmap read reg */
static _INLINE_ u32 ax99100_dread_mem_reg(int offset, int bar)
{
       return readl(spi99100.membase[bar] + offset);	
}

/* memmap write reg */
static _INLINE_ void ax99100_dwrite_mem_reg(int offset, int value, int bar)
{
	writel(value, spi99100.membase[bar] + offset);	
}

/* iomap read reg */
static _INLINE_ u8 ax99100_dread_io_reg(unsigned char offset)
{
       return inb(spi99100.iobase0 + offset);	
}

/* iomap write reg */
static _INLINE_ void ax99100_dwrite_io_reg(unsigned char offset,unsigned char value)
{
       outb(value, spi99100.iobase0 + offset);
}

/******************************************************
 * 
 * File Operation
 * 
 * ****************************************************/

static long spi99100_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	
	unsigned long	length;	
	
	switch (cmd) {	
	case IOCTL_IO_SET_REGISTER:
	{
		if(copy_from_user(reg, (PSPI_REG)arg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_io_reg(reg->Offset, reg->Value);
		break;
	}
	case IOCTL_IO_READ_REGISTER:
	{			
		if(copy_from_user(reg, (PSPI_REG)arg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		reg->Value = ax99100_dread_io_reg(reg->Offset);
		
		if(copy_to_user((PSPI_REG)arg, reg, sizeof(SPI_REG)))
			return -ENOIOCTLCMD;
		
		
		break;
	}
	case IOCTL_MEM_SET_REGISTER:
	{
		if(copy_from_user(reg_m, (PMMAP_SPI_REG)arg, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_mem_reg(reg_m->Offset, reg_m->Value, reg_m->Bar);
		break;
	}
	case IOCTL_MEM_READ_REGISTER:
	{
		if(copy_from_user(reg_m, (PMMAP_SPI_REG)arg, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
	  
		reg_m->Value = ax99100_dread_mem_reg(reg_m->Offset, reg_m->Bar);
		
		if(copy_to_user((PMMAP_SPI_REG)arg, reg_m, sizeof(MMAP_SPI_REG)))
			return -ENOIOCTLCMD;
		
		break;
	}
	case IOCTL_SET_TX_DMA_REG:
	{
		if(copy_from_user(&length, (unsigned long *)arg, sizeof(unsigned long)))
			return -ENOIOCTLCMD;
		
		ax99100_dwrite_mem_reg(REG_TDMASAR0, spi99100.tx_dma_p, BAR1);
		ax99100_dwrite_mem_reg(REG_TDMASAR1, 0x0, BAR1);
		ax99100_dwrite_mem_reg(REG_TDMALR, length, BAR1);
		ax99100_dwrite_mem_reg(REG_TDMASTAR, START_DMA, BAR1);
		
		break;
	}
	case IOCTL_SET_RX_DMA_REG:
	{
		if(copy_from_user(&length, (unsigned long *)arg, sizeof(unsigned long)))
			return -ENOIOCTLCMD;	  
		
		ax99100_dwrite_mem_reg(REG_RDMASAR0, spi99100.rx_dma_p, BAR1);
		ax99100_dwrite_mem_reg(REG_RDMASAR1, 0x0, BAR1);
		ax99100_dwrite_mem_reg(REG_RDMALR, length, BAR1);
		ax99100_dwrite_mem_reg(REG_RDMASTAR, START_DMA, BAR1);
		
		break;
	}
	case IOCTL_TX_DMA_WRITE:
	{
		if(copy_from_user(dma, (PSPI_DMA)arg, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		memcpy_toio(spi99100.tx_dma_v, dma->Buffer, dma->Length);
		
		break;
	}
	case IOCTL_RX_DMA_READ:
	{
		if(copy_from_user(dma, (PSPI_DMA)arg, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		memcpy_fromio(dma->Buffer, spi99100.rx_dma_v, dma->Length);
		
		if(copy_to_user((PSPI_DMA)arg, dma, sizeof(SPI_DMA)))
			return -ENOIOCTLCMD;
		
		break;
	}

	default:
		return -ENOIOCTLCMD;	
	}
	return 0;
}

static int spi99100_open (struct inode *inop, struct file *filp)
{
	reg = kmalloc(sizeof(SPI_REG), GFP_KERNEL);
	memset(reg, '\0', sizeof(SPI_REG));
	if (!reg)
		return -1;
	
	reg_m = kmalloc(sizeof(MMAP_SPI_REG), GFP_KERNEL);
	memset(reg_m, '\0', sizeof(MMAP_SPI_REG));
	if (!reg_m)
		return -1;
	
	dma = kmalloc(sizeof(SPI_DMA), GFP_KERNEL);
	memset(dma, '\0', sizeof(SPI_DMA));
	if (!dma)
		return -1;
	
	return 0;
}

static int spi99100_release (struct inode *inop, struct file *filp)
{
	kfree(reg);
	kfree(reg_m);
	kfree(dma);
	
	return 0;
}

static struct file_operations bridge_fops = {
	.owner		=	THIS_MODULE,	
	.unlocked_ioctl	=	spi99100_ioctl,	
	.open		=	spi99100_open,
	.release	=	spi99100_release,
};
/********************************************************************
 * 
 * NETLINK
 * 
 ********************************************************************/
void netlink_get(struct sk_buff *__skb) {
 	
	struct nlmsghdr *nlh = NULL;
	char str[100];	

	if (skb)
		kfree_skb(skb);
	
	skb = skb_get(__skb);
        if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
                memcpy(str, NLMSG_DATA(nlh), sizeof(str));
                printk("%s: received netlink message payload:%s\n",
		__FUNCTION__, (char*)NLMSG_DATA(nlh));
		spi99100.tool_pid = nlh->nlmsg_pid;
	}
}

void netlink_sendmsg(int pid)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	
	char msg[30] = "Interrupt Complete!";

	if (!nl_sk)
		return;

	skb = nlmsg_new(MAX_PAYLOAD_SIZE, GFP_KERNEL);

	if (!skb)
		printk(KERN_ERR "nlmsg_new error");

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_PAYLOAD_SIZE, 0);

	memcpy(NLMSG_DATA(nlh), msg, sizeof(msg));	

	netlink_unicast(nl_sk, skb, pid, MSG_DONTWAIT);	
}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	
struct netlink_kernel_cfg netlink_kerncfg = {		
		        .input = netlink_get,
		    };
#endif
/********************************************************************
 * 
 * PCIE FUNCTION
 * 
 ********************************************************************/

//PCI driver remove function. Rlease the resources used by the port
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static void spi99100_remove_one(struct pci_dev *dev)
#else
static void __devexit spi99100_remove_one(struct pci_dev *dev)
#endif
{
	dev_t	device = MKDEV(spi99100.dev_major, spi99100.dev_minor);

	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);	
	
	/* DMA free */
	pci_free_consistent(dev,DMA_BUFFER_SZ,spi99100.tx_dma_v,spi99100.tx_dma_p);
	pci_free_consistent(dev,DMA_BUFFER_SZ,spi99100.rx_dma_v,spi99100.rx_dma_p);
	
	if  (dev->subsystem_device != PCI_SUBVEN_ID_AX99100_SPI) {
		dev_err(&dev->dev, "Not AX99100 SPI device when remove!\n");
		return;
	} 
	
	/* Remove Char Device & Class */
	device_destroy(spi_class, device);
	cdev_del(&spi);
	class_destroy(spi_class);	
	unregister_chrdev_region(device,1);
	
	/* Remove netlink setting */
	if (skb)
		kfree_skb(skb);	
	if (nl_sk)
		sock_release(nl_sk->sk_socket);	

	free_irq(spi99100.irq, &spi99100);

	pci_disable_device(dev);
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

void init_local_data(struct pci_dev *dev)
{
	unsigned long	base, len;

	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);	
		
	/* memory map  */
	/* bar1 */
	len =  pci_resource_len(dev, FL_BASE1);
	base = pci_resource_start(dev, FL_BASE1);
	spi99100.mapbase[0] = base;
	spi99100.membase[0] = ioremap(base,len);
	/* bar5 */
	len =  pci_resource_len(dev, FL_BASE5);
	base = pci_resource_start(dev, FL_BASE5);
	spi99100.mapbase[1] = base;
	spi99100.membase[1] = ioremap(base,len);
	
	
	DEBUG("bar1 membase=0x%x mapbase=0x%x\n",
		(unsigned int)spi99100.membase[0],(unsigned int)spi99100.mapbase[0]);
	DEBUG("bar5 membase=0x%x mapbase=0x%x\n",
		(unsigned int)spi99100.membase[1],(unsigned int)spi99100.mapbase[1]);
	
	/* io map */
	base = pci_resource_start(dev,FL_BASE0);
	spi99100.iobase0 = base;
	
	DEBUG("bar0 iobase=0x%x\n",(unsigned int)spi99100.iobase0);	
	
	
	/* DMA for TX */
	spi99100.tx_dma_v =
		(char *)pci_alloc_consistent(dev,DMA_BUFFER_SZ,&spi99100.tx_dma_p);
	memset(spi99100.tx_dma_v,0,DMA_BUFFER_SZ);
	
	DEBUG("tx_dma_v=0x%x tx_dma_p=0x%x\n",(unsigned int)spi99100.tx_dma_v,
		(unsigned int)spi99100.tx_dma_p);
	
	/* DMA for RX */
	spi99100.rx_dma_v =
		(char *)pci_alloc_consistent(dev,DMA_BUFFER_SZ,&spi99100.rx_dma_p);
	memset(spi99100.rx_dma_v,0,DMA_BUFFER_SZ);
	
	DEBUG("rx_dma_v=0x%x rx_dma_p=0x%x\n",(unsigned int)spi99100.rx_dma_v,
		(unsigned int)spi99100.rx_dma_p);
	

	
	
	DEBUG("In %s---------------------------------------END\n",__FUNCTION__);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static irqreturn_t spi99100_interrupt(int irq, void *dev_id)
#else
static irqreturn_t spi99100_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
	int handled = 0;
	unsigned long isr_status = 0;
	unsigned long sdcr = 0;	
	
	/* Read SDCR */	
	sdcr = ax99100_dread_io_reg(REG_SDCR);	
	if (!(sdcr & INTERRUPT_ENABLE_MASK))
		return IRQ_RETVAL(0);
	
	/* Read ISR */	
	isr_status = ax99100_dread_io_reg(REG_SPIMISR);	
	if (!(isr_status & INTERRUPT_MASK))
		return IRQ_RETVAL(0);
	/* Clear ISR */
	ax99100_dwrite_io_reg(REG_SPIMISR, isr_status);

	if (isr_status & SPIMISR_STC) {
		DEBUG("SPI Transceiver Complete\n");
		netlink_sendmsg(spi99100.tool_pid);
		handled = 1;
	}
	if (isr_status & SPIMISR_STERR) {
		DEBUG("SPI Transceiver Error Indication\n");
		handled = 1;
	}

	return IRQ_RETVAL(handled);
}

/* helper function to reset device connect to spi */
void spi_reset(void)
{
	ax99100_dwrite_mem_reg(REG_SWRST, SW_RESET, BAR1);	
}

static int register_char_device (struct spi_99100 *spi_device)
{	 
	dev_t		dev = 0;
	int		alloc_ret = 0,cdev_ret = 0;
	struct device 	*device = NULL;
	
	
	memset(&spi,0,sizeof(struct cdev));
	
	alloc_ret = alloc_chrdev_region(&dev, 0, 1, DEV_NAME);
	if (alloc_ret) {
		DEBUG("alloc_chrdev_region Failed.\n");		
		goto disable;
	}	
	
	spi_device->dev_major = MAJOR(dev);
	spi_device->dev_minor = MINOR(dev);
	
	spi_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(spi_class)) {    /* $ls /sys/class */
		 DEBUG("class_create Failed %ld.\n",PTR_ERR(spi_class));	
		 goto disable;
	}
	
	device = device_create(spi_class, NULL, dev, NULL, NODE_NAME);
	DEBUG("maj: %d,min: %d\n",spi_device->dev_major,spi_device->dev_minor);
	if (IS_ERR(device)) { 
		DEBUG("device_create Failed %ld.\n",PTR_ERR(device));	
		goto disable;
	}	
	
	cdev_init(&spi, &bridge_fops);
	spi.owner 	= THIS_MODULE;
	spi.ops	= &bridge_fops;
	cdev_ret = cdev_add(&spi, dev, 1);	
	if (cdev_ret) {
		DEBUG("cdev_add Failed.\n");
		goto disable;
	}		
	
	return 1;
	
disable:
	if (cdev_ret != 0)
		cdev_del(&spi);
	if (device != NULL)
		device_destroy(spi_class, dev);	  
	if (spi_class != NULL)
		class_destroy(spi_class);
	if (alloc_ret != 0)
		unregister_chrdev_region(dev, 1);
	return -1;
};
void assign_driver(struct pci_dev*);//PCI drivers probe function
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
static int spi99100_probe(struct pci_dev *dev, const struct pci_device_id *ent)
#else
static int __devinit spi99100_probe(struct pci_dev *dev, const struct pci_device_id *ent)
#endif
{
	int retval,ret;	
	
	printk(version_spi);

	memset(&spi99100,0,sizeof(struct spi_99100));
	
	retval = pci_enable_device(dev);
	
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);
	
	if (retval) {
		dev_err(&dev->dev, "Device enable FAILED\n");
                return retval;
	}	

	/* To verify whether it is a local bus communication hardware */
	if ((dev->class >> 16) != PCI_CLASS_OTHERS){
		DEBUG("Not a spi communication hardware\n");
		retval = -ENODEV;
		goto disable;
	}
	
	/* Initial Netlink sock */	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)	
	nl_sk = netlink_kernel_create(&init_net,NETLINK_TEST, &netlink_kerncfg);
#else
	nl_sk = netlink_kernel_create(&init_net,NETLINK_TEST, 0, netlink_get, NULL, THIS_MODULE);
	
#endif	
	ret = register_char_device(&spi99100);
	if (ret < 0) {
	/* Register this block driver with the kernel */
		  DEBUG("In %s char_device_register FAILED\n",__FUNCTION__);
		  return -1;
	}
	
	pci_set_master(dev);	
	
	init_local_data(dev);	

	spi_reset();
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	if ((retval = request_irq(dev->irq, spi99100_interrupt,
				  SA_SHIRQ,"ax99100_spi", &spi99100))) 
		goto disable;
#else
	if ((retval = request_irq(dev->irq, spi99100_interrupt,
				IRQF_SHARED,"ax99100_spi", &spi99100))) 
		goto disable;
#endif

	spi99100.irq = dev->irq;

	DEBUG("In %s---0-----------------------------------END\n",__FUNCTION__);
	return 0;	
	 
disable:
	pci_disable_device(dev);
	DEBUG("In %s---1-----------------------------------END\n",__FUNCTION__);
	return retval;
}


static int spi99100_suspend(struct pci_dev *dev, pm_message_t state)
{
	u16 data;

	spi_suspend_count++;

	/* Enable PME and D3 */
	if (dev->pm_cap) {
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
		pci_write_config_word(dev, dev->pm_cap + PCI_PM_CTRL, data | PCI_PM_CTRL_PME_ENABLE | PCI_D3hot);
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
	}

	pci_disable_device(dev);
	pci_save_state(dev);
	pci_enable_wake(dev, PCI_D3hot, 1);
	pci_set_power_state(dev, PCI_D3hot);

	return 0;
};

static int spi99100_resume(struct pci_dev *dev)
{
	u16 data;
	
	pci_set_power_state(dev, PCI_D0);
	pci_restore_state(dev);
	pci_enable_wake(dev, PCI_D0, 0);

	if (pci_enable_device(dev) < 0) {
		printk(KERN_ERR"pci_enable_device failed, ""disabling device\n");
		return -EIO;
	}
	pci_set_master(dev);

	spi_suspend_count--;

	/* Disable PME */
	if (dev->pm_cap) {
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
		pci_write_config_word(dev, dev->pm_cap + PCI_PM_CTRL, data & (~PCI_PM_CTRL_PME_ENABLE));
		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &data);
	}

	return 0;
};

static struct pci_device_id spi99100_pci_tbl[] = {
	{0x125B, 0x9100, PCI_SUBDEV_ID_AX99100, PCI_SUBVEN_ID_AX99100_SPI, 0, 0, 0},

	{0, },
};
static struct pci_driver starex_spi_driver = {
 	.name		= "AX99100_SPI",
	.probe		= spi99100_probe,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
	.remove		= spi99100_remove_one,
#else
	.remove		= __devexit_p(spi99100_remove_one),
#endif
	.id_table	= spi99100_pci_tbl,
	.suspend	= spi99100_suspend,
	.resume		= spi99100_resume,
};
/* Drivers entry function. register with the pci core */
int spi99100_init(void)
{	
	int ret;
	
	
	
	DEBUG("In %s---------------------------------------START\n",__FUNCTION__);	
	
	memset(&spi99100,0,sizeof(struct spi_99100));	
	
	ret = pci_register_driver(&starex_spi_driver);
	if (ret < 0){
		DEBUG("In %s pci_register_driver FAILED\n",__FUNCTION__);
	}
	
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);
	
	return ret;	

}

/* Drivers exit function. Unregister with the PCI core as well as serial core */
void spi99100_exit(void)
{	
	DEBUG("In %s ---------------------------------------START\n",__FUNCTION__);
		
	pci_unregister_driver(&starex_spi_driver);
	
	DEBUG("In %s ---------------------------------------END\n",__FUNCTION__);	
}
