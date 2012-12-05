
/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <unistd.h>
#include <errno.h>

#include "vold.h"
#include "cmd_dispatch.h"
#include "ums.h"
#include "volmgr.h"

struct cmd_dispatch {
    char *cmd;
    int (* dispatch) (char *);
};

static void dispatch_cmd(char *cmd);
static int do_send_ums_status(char *cmd);
static int do_set_ums_enable(char *cmd);
static int do_mount_volume(char *cmd);
static int do_eject_media(char *cmd);
static int do_format_media(char *cmd);

static struct cmd_dispatch dispatch_table[] = {
#ifdef ADD_ISO
    { VOLD_CMD_ENABLE_UMS_SD,      do_set_ums_enable },
    { VOLD_CMD_ENABLE_UMS_CD,      do_set_ums_enable },
#else
    { VOLD_CMD_ENABLE_UMS,      do_set_ums_enable },
#endif /* ADD_ISO */
    { VOLD_CMD_DISABLE_UMS,     do_set_ums_enable },
    { VOLD_CMD_SEND_UMS_STATUS, do_send_ums_status },
    { VOLD_CMD_MOUNT_VOLUME,    do_mount_volume },
    { VOLD_CMD_EJECT_MEDIA,     do_eject_media },
    { VOLD_CMD_FORMAT_MEDIA,    do_format_media },
    { NULL, NULL }
};

int process_framework_command(int socket)
{
    int rc;
    char buffer[101];

    if ((rc = read(socket, buffer, sizeof(buffer) -1)) < 0) {
        LOGE("Unable to read framework command (%s)", strerror(errno));
        return -errno;
    } else if (!rc)
        return -ECONNRESET;

    int start = 0;
    int i;

    buffer[rc] = 0;

    for (i = 0; i < rc; i++) {
        if (buffer[i] == 0) {
            dispatch_cmd(buffer + start);
            start = i + 1;
        }
    }
    return 0;
}

static void dispatch_cmd(char *cmd)
{
    struct cmd_dispatch *c;

    LOG_VOL("dispatch_cmd(%s):", cmd);

    for (c = dispatch_table; c->cmd != NULL; c++) {
        if (!strncmp(c->cmd, cmd, strlen(c->cmd))) {
            c->dispatch(cmd);
            return;
        }
    }

    LOGE("No cmd handlers defined for '%s'", cmd);
}

static int do_send_ums_status(char *cmd)
{
    return ums_send_status();
}


#ifdef ADD_ISO
extern int loopback_start(struct devmapping *dm);
extern int loopback_stop(struct devmapping *dm);
#include <stdlib.h>
#endif
static int do_set_ums_enable(char *cmd)
{
#ifdef ADD_ISO
	static struct devmapping *loop = NULL;
	static struct devmapping loopback;
	static char devpath[4096];
	// TODO: lun_syspath を変更する必要がある。
	static char *lun_syspath = "/sys/devices/platform/s3c-usbgadget/gadget/lun0/file";
	LOG_VOL("do_set_ums_enable:%s:%s\n", cmd, lun_syspath);

	if (!strcmp(cmd, VOLD_CMD_ENABLE_UMS_SD)) {
		loop = NULL;
		LOG_VOL("do_set_ums_enable:%s\n", VOLD_CMD_ENABLE_UMS_SD);
        	return volmgr_enable_ums(true);
	} else if (!strncmp(cmd, VOLD_CMD_ENABLE_UMS_CD, strlen(VOLD_CMD_ENABLE_UMS_CD))) {
		/* ループバックデバイスパスの取得 */
		LOG_VOL("do_set_ums_enable:%s\n", VOLD_CMD_ENABLE_UMS_CD);
		int rc = 0;
		memset(devpath, 0, 4096);
		memcpy(devpath, &cmd[strlen(VOLD_CMD_ENABLE_UMS_CD)], 
				strlen(&cmd[strlen(VOLD_CMD_ENABLE_UMS_CD)]));
		LOG_VOL("do_set_ums_enable devpath=%s\n", devpath);

		/* ループバックデバイスの登録 */
		loopback.src_type = dmsrc_loopback;
		loopback.type_data.loop.loop_src = devpath;
		loopback.type_data.loop.loop_dev = NULL;
		rc = loopback_start(&loopback);
		if (rc) {
			LOGE("loopback_start err=%d\n", rc);
		}
		/* ループバックデバイスパスをカーネルに追加する */
		/* カーネルにはloopback dev登録であることを知らせナイトね */
		rc = ums_enable(loopback.type_data.loop.loop_dev, lun_syspath, false);
		if (rc) {
			LOGE("ums_enable err=%d\n", rc);
		}
		loop = &loopback;
		return 0;
	} else {
		int ret = 0;
		LOG_VOL("do_set_ums_enable:%s\n", "disable_ums\n");
		if (loop) {
			int rc;
			rc = ums_disable(lun_syspath);
			if (rc) {
				LOGE("ums_enable err=%d\n", rc);
			}
			/* ループバックデバイスの解除 */
			rc = loopback_stop(loop);
			if (rc) {
				LOGE("loopback_stop err=%d\n", rc);
			} else {
				free(loopback.type_data.loop.loop_dev);
				loopback.type_data.loop.loop_dev = NULL;
			}
		} else {
			ret = volmgr_enable_ums(false);
		}
		loop = NULL;
		return ret;
	}
#else
    if (!strcmp(cmd, VOLD_CMD_ENABLE_UMS))
        return volmgr_enable_ums(true);

    return volmgr_enable_ums(false);
#endif /* ADD_ISO */
}

static int do_mount_volume(char *cmd)
{
    return volmgr_start_volume_by_mountpoint(&cmd[strlen("mount_volume:")]);
}

static int do_format_media(char *cmd)
{
    return volmgr_format_volume(&cmd[strlen("format_media:")]);
}

static int do_eject_media(char *cmd)
{
    return volmgr_stop_volume_by_mountpoint(&cmd[strlen("eject_media:")]);
}
