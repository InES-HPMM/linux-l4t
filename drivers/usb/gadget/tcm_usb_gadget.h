#ifndef __TARGET_USB_GADGET_H__
#define __TARGET_USB_GADGET_H__

/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 */

#include <linux/kref.h>
/* #include <linux/usb/uas.h> */
#include <linux/usb/composite.h>
#include <linux/usb/uas.h>
#include <linux/usb/storage.h>
#include <linux/spinlock.h>
#include <scsi/scsi.h>
#include <target/target_core_base.h>
#include <target/target_core_fabric.h>

#define USBG_NAMELEN 32

#define fuas_to_gadget(f)	(f->function.config->cdev->gadget)
#define UASP_SS_EP_COMP_LOG_STREAMS 4
#define UASP_SS_EP_COMP_NUM_STREAMS (1 << UASP_SS_EP_COMP_LOG_STREAMS)

/* The following UASP_STREAM_* flags are used to set the flags variable
 * in the uas_stream structure
 */
/* This tells if the stream resource is currently being used */
#define UASP_STREAM_BUSY   1

/* When all the pre-allocated stream resources are busy and a new stream
 * resource is created, this flag is used so that the resource can be
 * freed once the command is completed
 */
#define UASP_STREAM_RES_ALLOCATED	2
/* The following three flags are used to indicate what is the latst
 * endpoint on which the data transfer is happening for a command so
 * that when an error occurs, we can dequeue the request on this endpoint
 * which inturn deletes the command from the SCSI layer in the usb req
 * callback
 */
#define UASP_STREAM_EP_IN_ENQUEUED	4
#define UASP_STREAM_EP_OUT_ENQUEUED	8
#define UASP_STREAM_EP_STATUS_ENQUEUED  16

/* Used to clear the above UASP_STREAM_EP_*_QUEUED Flags of the
 * flags variable in the uas_stream structure. If more flags are
 * used later then this MASK definition has to be changed accordingly
 */
#define UASP_STREAM_EP_QUEUE_CLEAR_MASK       0x3

/* The following is the maximum value for a valid stream ID
 */
#define VALID_STREAM_ID_MAX	0xfffd

enum {
	USB_G_STR_CONFIG = USB_GADGET_FIRST_AVAIL_IDX,
	USB_G_STR_INT_UAS,
	USB_G_STR_INT_BBB,
};

#define USB_G_ALT_INT_BBB       0
#define USB_G_ALT_INT_UAS       1

struct usbg_nacl {
	/* Binary World Wide unique Port Name for SAS Initiator port */
	u64 iport_wwpn;
	/* ASCII formatted WWPN for Sas Initiator port */
	char iport_name[USBG_NAMELEN];
	/* Returned by usbg_make_nodeacl() */
	struct se_node_acl se_node_acl;
};

struct tcm_usbg_nexus {
	struct se_session *tvn_se_sess;
};

struct usbg_tpg {
	struct mutex tpg_mutex;
	/* SAS port target portal group tag for TCM */
	u16 tport_tpgt;
	/* Pointer back to usbg_tport */
	struct usbg_tport *tport;
	struct workqueue_struct *workqueue;
	/* Returned by usbg_make_tpg() */
	struct se_portal_group se_tpg;
	u32 gadget_connect;
	struct tcm_usbg_nexus *tpg_nexus;
	atomic_t tpg_port_count;
};

struct usbg_tport {
	/* SCSI protocol the tport is providing */
	u8 tport_proto_id;
	/* Binary World Wide unique Port Name for SAS Target port */
	u64 tport_wwpn;
	/* ASCII formatted WWPN for SAS Target port */
	char tport_name[USBG_NAMELEN];
	/* Returned by usbg_make_tport() */
	struct se_wwn tport_wwn;
};

enum uas_state {
	UASP_SEND_DATA,
	UASP_RECEIVE_DATA,
	UASP_SEND_STATUS,
	UASP_QUEUE_COMMAND,
};

#define USBG_MAX_CMD    64
struct usbg_cmd {
	/* common */
	u8 cmd_buf[USBG_MAX_CMD];
	u32 data_len;
	struct work_struct work;
	int unpacked_lun;
	struct se_cmd se_cmd;
	void *data_buf; /* used if no sg support available */
	struct f_uas *fu;
	struct completion write_complete;
	struct kref ref;

	/* UAS only */
	u16 tag;
	u16 prio_attr;
	u8 tm_function;
	u16 tm_tag;
	struct sense_iu sense_iu;
	struct response_ui response_iu;
	enum uas_state state;
	struct uas_stream *stream;
	bool no_scsi_contact;
	int cmd_status;
	/* BOT only */
	__le32 bot_tag;
	unsigned int csw_code;
	unsigned is_read:1;
	/* If this flag is set then the command processing
	 * should not proceed. For ex if the device receives
	 * a command with a tag same as the tag of a previously
	 * received command, then the driver sets this flag for
	 * the previous command so that the previous command
	 * processing will not proceed further.
	 */
	bool cancel_command;
};

struct uas_stream {
	struct usb_request	*req_in;
	struct usb_request	*req_out;
	struct usb_request	*req_status;
	/* Storing the current command pointer that is
	 * using this stream
	 */
	struct usbg_cmd         *cmd;
	u8     flags;
};

struct usbg_cdb {
	struct usb_request	*req;
	void			*buf;
};

struct bot_status {
	struct usb_request	*req;
	struct bulk_cs_wrap	csw;
};

struct f_uas {
	struct usbg_tpg		*tpg;
	struct usb_function	function;
	u16			iface;

	u32			flags;
#define USBG_ENABLED		(1 << 0)
#define USBG_IS_UAS		(1 << 1)
#define USBG_USE_STREAMS	(1 << 2)
#define USBG_IS_BOT		(1 << 3)
#define USBG_BOT_CMD_PEND	(1 << 4)

	struct usbg_cdb		cmd;
	struct usb_ep		*ep_in;
	struct usb_ep		*ep_out;

	/* UAS */
	struct usb_ep		*ep_status;
	struct usb_ep		*ep_cmd;
	struct uas_stream	stream[UASP_SS_EP_COMP_NUM_STREAMS];

	/* BOT */
	struct bot_status	bot_status;
	struct usb_request	*bot_req_in;
	struct usb_request	*bot_req_out;
	spinlock_t lock;
};

extern struct usbg_tpg *the_only_tpg_I_currently_have;

#endif
