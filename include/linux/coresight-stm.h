#ifndef __LINUX_CORESIGHT_STM_H_
#define __LINUX_CORESIGHT_STM_H_

#include <uapi/linux/coresight-stm.h>

#define stm_log_inv(entity_id, ch_id, data, size)		\
	stm_trace(STM_OPTION_NONE, ch_id, entity_id, data, size)

#define stm_log_inv_ts(entity_id, ch_id, data, size)		\
	stm_trace(STM_OPTION_TIMESTAMPED, ch_id, entity_id,	\
		  data, size)

#define stm_log_gtd(entity_id, ch_id, data, size)		\
	stm_trace(STM_OPTION_GUARANTEED, ch_id, entity_id,	\
		  data, size)

#define stm_log_gtd_ts(entity_id, ch_id, data, size)		\
	stm_trace(STM_OPTION_GUARANTEED | STM_OPTION_TIMESTAMPED,	\
		  ch_id, entity_id, data, size)

#define stm_log(entity_id, ch_id, data, size)				\
	stm_log_inv_ts(entity_id, ch_id, data, size)

#ifdef CONFIG_CORESIGHT_STM
extern int stm_trace(u32 options, int channel_id,
		     u8 entity_id, const void *data, u32 size);
#else
static inline int stm_trace(u32 options, int channel_id,
			    u8 entity_id, const void *data, u32 size)
{
	return 0;
}
#endif

#endif
