/* Decoded SLE announcement reports, never connect or stop after discovery. */
#include <stdint.h>
#include <string.h>
#include "soc_osal.h"
#include "app_init.h"
#include "at.h"
#include "tcxo.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
typedef struct { uint64_t us; uint32_t seq; int8_t rssi; } sample_t;
static sample_t samples[128];
static uint32_t head,tail,sequence,dropped,invalid;
static const uint8_t beacon_addr[6]={0x52,0x35,0x35,0x30,0x00,0x01};
static const char name[]="R550_BC01";
static int matches(const uint8_t *p,uint8_t n)
{
    if (p==NULL) return 0;
    for (unsigned i=0;i<n;) {
        unsigned len=p[i],next=i+len+1;
        if (!len || next>n) return 0;
        if (len==sizeof(name) && p[i+1]==0x0b && !memcmp(p+i+2,name,sizeof(name)-1)) return 1;
        i=next;
    }
    return 0;
}
static void scan_result(sle_seek_result_info_t *r)
{
    if (!r || memcmp(r->addr.addr,beacon_addr,6) || !matches(r->data,r->data_length)) return;
    osal_kthread_lock();
    if (r->rssi==0x7f) {invalid++;osal_kthread_unlock();return;}
    sample_t s={uapi_tcxo_get_us(),++sequence,(int8_t)r->rssi};
    if (head-tail>=128) dropped++;
    else samples[head++%128]=s;
    osal_kthread_unlock();
}
static void enabled(errcode_t status)
{
    if (status!=ERRCODE_SLE_SUCCESS) return;
    sle_seek_param_t p={0};
    p.filter_duplicates=0;p.seek_filter_policy=SLE_SEEK_FILTER_ALLOW_ALL;
    p.seek_phys=SLE_SEEK_PHY_1M;p.seek_type[0]=SLE_SEEK_PASSIVE;
    p.seek_window[0]=100;p.seek_interval[0]=100;
    if (sle_set_seek_param(&p)==ERRCODE_SLE_SUCCESS) (void)sle_start_seek();
}
static at_ret_t hello(void)
{
    uapi_at_print("+BCHELLO:1,RX,1,0,100\r\n");return AT_RET_OK;
}
static const at_cmd_entry_t commands[]={{.name="BCHELLO",.cmd_id=0x7100,.cmd=hello}};
static void *beacon_receiver_task(const char *arg)
{
    (void)arg;osal_msleep(1200);
    if (uapi_at_cmd_table_register(commands,1,sizeof(uint32_t))!=0) return NULL;
    sle_announce_seek_callbacks_t c={0};c.sle_enable_cb=enabled;c.seek_result_cb=scan_result;
    if (sle_announce_seek_register_callbacks(&c)!=ERRCODE_SLE_SUCCESS) return NULL;
    (void)enable_sle();uint64_t last=0;
    for (;;) {
        sample_t s;int have=0;
        osal_kthread_lock();
        if (tail!=head) {s=samples[tail++%128];have=1;}
        uint32_t lost=dropped,bad=invalid,count=sequence;
        osal_kthread_unlock();
        if (have) uapi_at_print("+BCRSSI:%llu,%u,1,%d,%u\r\n",s.us,s.seq,(int)s.rssi,lost);
        uint64_t now=uapi_tcxo_get_us();
        if (now-last>=1000000ULL) {
            uapi_at_print("+BCSTATUS:%llu,%u,%u,%u\r\n",now,count,lost,bad);last=now;
        }
        osal_msleep(have?1:10);
    }
    return NULL;
}
static void beacon_receiver_start(void)
{
    osal_kthread_lock();
    osal_task *t=osal_kthread_create((osal_kthread_handler)beacon_receiver_task,0,"BeaconRx",0x1800);
    if(t!=NULL) osal_kthread_set_priority(t,26);
    osal_kthread_unlock();
}
app_run(beacon_receiver_start);
