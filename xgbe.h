/********************************************************************************

 **** Copyright (C), 2020, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : xgbe.h
 * Author        : func
 * Date          : 2020-02-07
 * Version       : 1.0
 * Function List :
 * Description   : xgbe
 * Record        :
 * 1.Date        : 2020-02-07
 *   Author      : func
 *   Modification: Created file

 *******************************************************************************/ 
#ifndef _E1000_H_
#define _E1000_H_
 
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#define DBUG 1

#ifdef DEBUG
#define DEBUG_MSG(...) printk(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

#define USE_IXGBE_METHOD
 
#define BAR_0		0
#define BAR_1		1
#define BAR_5		5
 
#define XGBE_ETHERNET_DEVICE(device_id) {\
     PCI_DEVICE(PCI_VENDOR_ID_XILINX, device_id)}

#define E1000_MAX_INTR			10

/*
 * Count for polling __E1000_RESET condition every 10-20msec.
 */
#define E1000_CHECK_RESET_COUNT	50

/* TX/RX descriptor defines */
#define E1000_DEFAULT_TXD		256
#define E1000_MAX_TXD			256
#define E1000_MIN_TXD			48
#define E1000_MAX_82544_TXD		4096

#define E1000_DEFAULT_RXD		256
#define E1000_MAX_RXD			256
#define E1000_MIN_RXD			48
#define E1000_MAX_82544_RXD		4096

#define E1000_MIN_ITR_USECS		10 /* 100000 irq/sec */
#define E1000_MAX_ITR_USECS		10000 /* 100    irq/sec */

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE	1600

/* Supported Rx Buffer Sizes */
#define E1000_RXBUFFER_128		128    /* Used for packet split */
#define E1000_RXBUFFER_256		256    /* Used for packet split */
#define E1000_RXBUFFER_512		512
#define E1000_RXBUFFER_1024		1024
#define E1000_RXBUFFER_2048		2048
#define E1000_RXBUFFER_4096		4096
#define E1000_RXBUFFER_8192		8192
#define E1000_RXBUFFER_16384		16384

/* SmartSpeed delimiters */
#define E1000_SMARTSPEED_DOWNSHIFT	3
#define E1000_SMARTSPEED_MAX		15

/* Packet Buffer allocations */
#define E1000_PBA_BYTES_SHIFT		0xA
#define E1000_TX_HEAD_ADDR_SHIFT	7
#define E1000_PBA_TX_MASK		0xFFFF0000

/* Flow Control Watermarks */
#define E1000_FC_HIGH_DIFF	0x1638 /* High: 5688 bytes below Rx FIFO size */
#define E1000_FC_LOW_DIFF	0x1640 /* Low:  5696 bytes below Rx FIFO size */

#define E1000_FC_PAUSE_TIME	0xFFFF /* pause for the max or until send xon */

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16
/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16 /* Must be power of 2 */

#define AUTO_ALL_MODES		0
#define E1000_EEPROM_82544_APM	0x0004
#define E1000_EEPROM_APME	0x0400

#ifndef E1000_MASTER_SLAVE
/* Switch to override PHY master/slave setting */
#define E1000_MASTER_SLAVE	e1000_ms_hw_default
#endif

#define E1000_MNG_VLAN_NONE	(-1)


enum e1000_state_t {
	__E1000_TESTING,
	__E1000_RESETTING,
	__E1000_DOWN,
	__E1000_DISABLED
};

#define E1000_DESC_UNUSED(R)						\
({									\
	unsigned int clean = smp_load_acquire(&(R)->next_to_clean);	\
	unsigned int use = READ_ONCE((R)->next_to_use);			\
	(clean > use ? 0 : (R)->count) + clean - use - 1;		\
})

#define E1000_RX_DESC_EXT(R, i)						\
	(&(((union e1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, e1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, e1000_context_desc)


/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct e1000_tx_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	unsigned long time_stamp;
	u16 length;
	u16 next_to_watch;
	bool mapped_as_page;
	unsigned short segs;
	unsigned int bytecount;
};

struct e1000_rx_buffer {
	union {
		struct page *page; /* jumbo: alloc_page */
		u8 *data; /* else, netdev_alloc_frag */
	} rxbuf;
	dma_addr_t dma;
};

struct e1000_tx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct e1000_tx_buffer *buffer_info;

	u16 tdh;
	u16 tdt;
	bool last_tx_tso;
};

struct e1000_rx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct e1000_rx_buffer *buffer_info;
	struct sk_buff *rx_skb_top;

	/* cpu for rx queue */
	int cpu;

	u16 rdh;
	u16 rdt;
};

/* board specific private data structure */

struct e1000_adapter {
	unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];
	u32 bd_number;
	u32 rx_buffer_len;
	u32 wol;
	u32 smartspeed;
	u32 en_mng_pt;
	u16 link_speed;
	u16 link_duplex;
	spinlock_t stats_lock;
	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;
	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
	u16 tx_itr;
	u16 rx_itr;

	u8 fc_autoneg;

	/* TX */
	struct e1000_tx_ring *tx_ring;      /* One per active queue */
	unsigned int restart_queue;
	u32 txd_cmd;
	u32 tx_int_delay;
	u32 tx_abs_int_delay;
	u32 gotcl;
	u64 gotcl_old;
	u64 tpt_old;
	u64 colc_old;
	u32 tx_timeout_count;
	u32 tx_fifo_head;
	u32 tx_head_addr;
	u32 tx_fifo_size;
	u8  tx_timeout_factor;
	atomic_t tx_fifo_stall;
	bool pcix_82544;
	bool detect_tx_hung;
	bool dump_buffers;

	/* RX */
	bool (*clean_rx)(struct e1000_adapter *adapter,
			 struct e1000_rx_ring *rx_ring,
			 int *work_done, int work_to_do);
	void (*alloc_rx_buf)(struct e1000_adapter *adapter,
			     struct e1000_rx_ring *rx_ring,
			     int cleaned_count);
	struct e1000_rx_ring *rx_ring;      /* One per active queue */
	struct napi_struct napi;

	int num_tx_queues;
	int num_rx_queues;

	u64 hw_csum_err;
	u64 hw_csum_good;
	u32 alloc_rx_buff_failed;
	u32 rx_int_delay;
	u32 rx_abs_int_delay;
	bool rx_csum;
	u32 gorcl;
	u64 gorcl_old;

	/* OS defined structs */
	struct net_device *netdev;
	struct pci_dev *pdev;

	/* structs defined in e1000_hw.h */
	struct e1000_hw hw;
	struct e1000_hw_stats stats;
	struct e1000_phy_info phy_info;
	struct e1000_phy_stats phy_stats;

	u32 test_icr;
	struct e1000_tx_ring test_tx_ring;
	struct e1000_rx_ring test_rx_ring;

	int msg_enable;

	/* to not mess up cache alignment, always add to the bottom */
	bool tso_force;
	bool smart_power_down;	/* phy smart power down */
	bool quad_port_a;
	unsigned long flags;
	u32 eeprom_wol;

	/* for ioport free */
	int bars;
	int need_ioport;

	bool discarding;

	struct work_struct reset_task;
	struct delayed_work watchdog_task;
	struct delayed_work fifo_stall_task;
	struct delayed_work phy_info_task;
};


 
#endif /* _E1000_H_ */

