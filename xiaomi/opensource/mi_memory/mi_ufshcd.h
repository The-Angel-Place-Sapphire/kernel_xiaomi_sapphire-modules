/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * mi_ufshcd.h - glue between Xiaomi mi-memory and the GKI 5.15 UFS host driver
 *
 * The sky-t-oss mi-memory sources include the Xiaomi forked UFS core
 * ("../../scsi/mi_ufs/mi-ufshcd.h").  On sapphire (khaje, android13-5.15 GKI)
 * the stock mi_memory.ko is built against the GKI UFS core instead: it calls
 * the vmlinux exports ufshcd_read_desc_param()/ufshcd_query_descriptor_retry()
 * and reads struct ufs_hba with the GKI layout (dev @0x40, priv @0x100,
 * dev_cmd.lock @0xa40, ...).  The only Xiaomi additions it relies on live in
 * the Xiaomi-modified ufs_qcom.ko: get_ufs_data() and the ufs_uic_stats block
 * inserted in struct ufs_qcom_host right after dev_req_params (@0x6c).
 */
#ifndef __MI_UFSHCD_H__
#define __MI_UFSHCD_H__

#include "ufshcd.h"	/* $(srctree)/drivers/scsi/ufs */
#include "ufs.h"
#include "ufs-qcom.h"

/* the Xiaomi UFS fork names of the GKI exports */
#define mi_ufshcd_read_desc_param		ufshcd_read_desc_param
#define mi_ufshcd_query_descriptor_retry	ufshcd_query_descriptor_retry

#ifndef QUERY_DESC_HEALTH_DEF_SIZE
#define QUERY_DESC_HEALTH_DEF_SIZE		0x25
#endif

/* Host UIC error code PHY adapter layer */
#ifndef UFS_QCOM_XIAOMI_STATS	/* defined by our ufs-qcom.h */
enum ufshcd_ec_pa {
	UFS_EC_PA_LANE_0,
	UFS_EC_PA_LANE_1,
	UFS_EC_PA_LANE_2,
	UFS_EC_PA_LANE_3,
	UFS_EC_PA_LINE_RESET,
	UFS_EC_PA_MAX,
};

/* Host UIC error code data link layer */
enum ufshcd_ec_dl {
	UFS_EC_DL_NAC_RECEIVED,
	UFS_EC_DL_TCx_REPLAY_TIMER_EXPIRED,
	UFS_EC_DL_AFCx_REQUEST_TIMER_EXPIRED,
	UFS_EC_DL_FCx_PROTECT_TIMER_EXPIRED,
	UFS_EC_DL_CRC_ERROR,
	UFS_EC_DL_RX_BUFFER_OVERFLOW,
	UFS_EC_DL_MAX_FRAME_LENGTH_EXCEEDED,
	UFS_EC_DL_WRONG_SEQUENCE_NUMBER,
	UFS_EC_DL_AFC_FRAME_SYNTAX_ERROR,
	UFS_EC_DL_NAC_FRAME_SYNTAX_ERROR,
	UFS_EC_DL_EOF_SYNTAX_ERROR,
	UFS_EC_DL_FRAME_SYNTAX_ERROR,
	UFS_EC_DL_BAD_CTRL_SYMBOL_TYPE,
	UFS_EC_DL_PA_INIT_ERROR,
	UFS_EC_DL_PA_ERROR_IND_RECEIVED,
	UFS_EC_DL_MAX,
};

struct ufs_uic_stats {
	u32 pa_err_cnt_total;
	u32 pa_err_cnt[UFS_EC_PA_MAX];
	u32 dl_err_cnt_total;
	u32 dl_err_cnt[UFS_EC_DL_MAX];
	u32 dme_err_cnt;
};

/*
 * Head of struct ufs_qcom_host as built in the Xiaomi ufs_qcom.ko
 * (sky-t-oss drivers/scsi/ufs/ufs-qcom.h): only the members up to ufs_stats
 * are accessed here.
 */
#endif

struct mi_ufs_qcom_host {
	u32 caps;
	struct phy *generic_phy;
	struct ufs_hba *hba;
	struct ufs_qcom_bus_vote bus_vote;
	struct ufs_pa_layer_attr dev_req_params;
	struct ufs_uic_stats ufs_stats;
};

/*
 * customer debug interface (exported by the Xiaomi ufs_qcom.ko)
 */
#ifndef UFS_QCOM_XIAOMI_STATS
struct UFS_ERR_STATE_DEBUG {
	u64 err_occurred; /*if happend err*/
	char err_reason[10][32]; /*err reason*/
};

struct UFS_DATA {
	struct UFS_ERR_STATE_DEBUG ufs_err_state;
};

struct UFS_DATA *get_ufs_data(void);
#endif


/* ufshcd_get_variant() as carried by the sapphire mi_ufshcd.h copy */
static inline void *mi_ufshcd_get_variant(struct ufs_hba *hba)
{
	BUG_ON(!hba);
	return hba->priv;
}

#endif /* __MI_UFSHCD_H__ */
