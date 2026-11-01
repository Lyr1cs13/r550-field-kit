/*
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2026.
 * Non-connectable SLE advertiser for the RSSI scan sample.
 */
#include <string.h>
#include "common_def.h"
#include "securec.h"
#include "soc_osal.h"
#include "sle_common.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "app_init.h"

#define SLE_RSSI_SCAN_ADV_LOG          "[sle rssi scan adv]"
#define SLE_RSSI_SCAN_ADV_HANDLE       1
#define SLE_RSSI_SCAN_ADV_INTERVAL     0x0320U /* 100 ms: interval unit is 125 us */
#define SLE_RSSI_SCAN_ADV_DATA_MAX_LEN 32U
#define SLE_RSSI_SCAN_DEVICE_NAME      "R550_BC01"
#define SLE_RSSI_SCAN_ADV_HEADER_LEN   2U

enum {
    SLE_RSSI_SCAN_AD_DISCOVERY_LEVEL = 0x01,
    SLE_RSSI_SCAN_AD_ACCESS_MODE = 0x02,
    SLE_RSSI_SCAN_AD_COMPLETE_LOCAL_NAME = 0x0B,
};

static uint16_t sle_rssi_scan_build_adv_data(uint8_t *data, uint16_t capacity)
{
    const uint8_t device_name[] = SLE_RSSI_SCAN_DEVICE_NAME;
    const uint8_t name_len = sizeof(device_name) - 1U;
    uint16_t index = 0;

    if ((data == NULL) || (capacity < (uint16_t)(8U + name_len))) {
        return 0;
    }

    /* Keep the two standard SLE fields used by the SDK samples. */
    data[index++] = 2;
    data[index++] = SLE_RSSI_SCAN_AD_DISCOVERY_LEVEL;
    data[index++] = SLE_ANNOUNCE_LEVEL_NORMAL;
    data[index++] = 2;
    data[index++] = SLE_RSSI_SCAN_AD_ACCESS_MODE;
    data[index++] = 0;

    /* A short name in the primary announcement lets passive scanners identify us. */
    data[index++] = name_len + 1U;
    data[index++] = SLE_RSSI_SCAN_AD_COMPLETE_LOCAL_NAME;
    if (memcpy_s(&data[index], capacity - index, device_name, name_len) != EOK) {
        return 0;
    }
    return (uint16_t)(index + name_len);
}

static errcode_t sle_rssi_scan_configure_and_start_advertising(void)
{
    sle_announce_param_t param = {0};
    sle_announce_data_t announce_data = {0};
    uint8_t data[SLE_RSSI_SCAN_ADV_DATA_MAX_LEN] = {0};
    uint8_t empty_seek_rsp_data = 0;
    const uint8_t local_addr[SLE_ADDR_LEN] = {0x52, 0x35, 0x35, 0x30, 0x00, 0x01};
    errcode_t ret;

    param.announce_handle = SLE_RSSI_SCAN_ADV_HANDLE;
    param.announce_mode = SLE_ANNOUNCE_MODE_NONCONN_NONSCAN;
    param.announce_gt_role = SLE_ANNOUNCE_ROLE_T_NO_NEGO;
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;
    param.announce_interval_min = SLE_RSSI_SCAN_ADV_INTERVAL;
    param.announce_interval_max = SLE_RSSI_SCAN_ADV_INTERVAL;
    param.announce_channel_map = 0x07;
    param.announce_tx_power = 0; /* Fixed 0 dBm setting; actual EIRP is hardware dependent. */
    param.own_addr.type = 0;
    if (memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN) != EOK) {
        return ERRCODE_SLE_FAIL;
    }

    ret = sle_set_announce_param(SLE_RSSI_SCAN_ADV_HANDLE, &param);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s set announce parameters failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    announce_data.announce_data_len = sle_rssi_scan_build_adv_data(data, sizeof(data));
    if (announce_data.announce_data_len == 0) {
        osal_printk("%s build announce data failed\r\n", SLE_RSSI_SCAN_ADV_LOG);
        return ERRCODE_SLE_FAIL;
    }
    announce_data.announce_data = data;
    announce_data.seek_rsp_data_len = 0;
    announce_data.seek_rsp_data = &empty_seek_rsp_data;
    ret = sle_set_announce_data(SLE_RSSI_SCAN_ADV_HANDLE, &announce_data);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s set announce data failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    ret = sle_start_announce(SLE_RSSI_SCAN_ADV_HANDLE);
    osal_printk("%s non-connectable announcement start, name=%s, status=0x%x\r\n",
                SLE_RSSI_SCAN_ADV_LOG, SLE_RSSI_SCAN_DEVICE_NAME, ret);
    return ret;
}

static void sle_rssi_scan_announce_enable_cb(uint32_t announce_id, errcode_t status)
{
    osal_printk("%s announcement enabled, id=%u, status=0x%x\r\n",
                SLE_RSSI_SCAN_ADV_LOG, (unsigned int)announce_id, status);
}

static void sle_rssi_scan_sle_enable_cb(errcode_t status)
{
    osal_printk("%s SLE enabled, status=0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, status);
    if (status == ERRCODE_SLE_SUCCESS) {
        (void)sle_rssi_scan_configure_and_start_advertising();
    }
}

errcode_t sle_rssi_scan_advertiser_init(void)
{
    sle_announce_seek_callbacks_t callbacks = {0};
    errcode_t ret;

    callbacks.sle_enable_cb = sle_rssi_scan_sle_enable_cb;
    callbacks.announce_enable_cb = sle_rssi_scan_announce_enable_cb;
    ret = sle_announce_seek_register_callbacks(&callbacks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s register callbacks failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
        return ret;
    }

    ret = enable_sle();
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("%s enable SLE failed: 0x%x\r\n", SLE_RSSI_SCAN_ADV_LOG, ret);
    }
    return ret;
}

static void *fixed_beacon_task(const char *arg) { (void)arg;osal_msleep(1200);sle_rssi_scan_advertiser_init();return NULL; }
static void fixed_beacon_start(void) {
 osal_kthread_lock();osal_task *t=osal_kthread_create((osal_kthread_handler)fixed_beacon_task,0,"BeaconTx",0x1400);
 if(t!=NULL) { osal_kthread_set_priority(t,26); }
 osal_kthread_unlock();
}
app_run(fixed_beacon_start);
