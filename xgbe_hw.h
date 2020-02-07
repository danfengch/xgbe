/********************************************************************************

 **** Copyright (C), 2020, xx xx xx xx info&tech Co., Ltd.                ****

 ********************************************************************************
 * File Name     : xgbe_hw.h
 * Author        : func
 * Date          : 2020-02-07
 * Version       : 1.0
 * Function List :
 * Description   : xgbe_hw
 * Record        :
 * 1.Date        : 2020-02-07
 *   Author      : func
 *   Modification: Created file

 *******************************************************************************/

#ifndef _E1000_HW_H_
#define _E1000_HW_H_
 
#include "e1000_osdep.h"


#define SPEED_10    10
#define SPEED_100   100
#define SPEED_1000  1000
#define HALF_DUPLEX 1
#define FULL_DUPLEX 2

/* The sizes (in bytes) of a ethernet packet */
#define ENET_HEADER_SIZE             14
#define MINIMUM_ETHERNET_FRAME_SIZE  64	/* With FCS */
#define ETHERNET_FCS_SIZE            4
#define MINIMUM_ETHERNET_PACKET_SIZE \
    (MINIMUM_ETHERNET_FRAME_SIZE - ETHERNET_FCS_SIZE)
#define CRC_LENGTH                   ETHERNET_FCS_SIZE

 
/* PCI bus types */
typedef enum {
	e1000_bus_type_unknown = 0,
	e1000_bus_type_pci,
	e1000_bus_type_pcix,
	e1000_bus_type_pcie,
	e1000_bus_type_reserved
} e1000_bus_type;


/* Structure containing variables used by the shared code (e1000_hw.c) */
struct e1000_hw {
	u8 __iomem *hw_addr;
	u8 __iomem *flash_address;
	void __iomem *ce4100_gbe_mdio_base_virt;
	e1000_media_type media_type;
	void *back;
	struct e1000_shadow_ram *eeprom_shadow_ram;
	u32 flash_bank_size;
	u32 flash_base_addr;
	e1000_fc_type fc;
	e1000_bus_speed bus_speed;
	e1000_bus_width bus_width;
	e1000_bus_type bus_type;
	struct e1000_eeprom_info eeprom;
	e1000_ms_type master_slave;
	e1000_ms_type original_master_slave;
	e1000_ffe_config ffe_config_state;
	u32 asf_firmware_present;
	u32 eeprom_semaphore_present;
	unsigned long io_base;
	u32 phy_id;
	u32 phy_revision;
	u32 phy_addr;
	u32 original_fc;
	u32 txcw;
	u32 autoneg_failed;
	u32 max_frame_size;
	u32 min_frame_size;
	u32 mc_filter_type;
	u32 num_mc_addrs;
	u32 collision_delta;
	u32 tx_packet_delta;
	u32 ledctl_default;
	u32 ledctl_mode1;
	u32 ledctl_mode2;
	bool tx_pkt_filtering;
	struct e1000_host_mng_dhcp_cookie mng_cookie;
	u16 phy_spd_default;
	u16 autoneg_advertised;
	u16 pci_cmd_word;
	u16 fc_high_water;
	u16 fc_low_water;
	u16 fc_pause_time;
	u16 current_ifs_val;
	u16 ifs_min_val;
	u16 ifs_max_val;
	u16 ifs_step_size;
	u16 ifs_ratio;
	u16 device_id;
	u16 vendor_id;
	u16 subsystem_id;
	u16 subsystem_vendor_id;
	u8 revision_id;
	u8 autoneg;
	u8 mdix;
	u8 forced_speed_duplex;
	u8 wait_autoneg_complete;
	u8 dma_fairness;
	u8 mac_addr[NODE_ADDRESS_SIZE];
	u8 perm_mac_addr[NODE_ADDRESS_SIZE];
	bool disable_polarity_correction;
	bool speed_downgraded;
	e1000_smart_speed smart_speed;
	e1000_dsp_config dsp_config_state;
	bool get_link_status;
	bool serdes_has_link;
	bool tbi_compatibility_en;
	bool tbi_compatibility_on;
	bool laa_is_present;
	bool phy_reset_disable;
	bool initialize_hw_bits_disable;
	bool fc_send_xon;
	bool fc_strict_ieee;
	bool report_tx_early;
	bool adaptive_ifs;
	bool ifs_params_forced;
	bool in_ifs_mode;
	bool mng_reg_access_disabled;
	bool leave_av_bit_off;
	bool bad_tx_carr_stats_fd;
	bool has_smbus;
};
 
 
#endif /* _E1000_HW_H_ */

