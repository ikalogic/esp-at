#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_at.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "custom_ble.h"
#include "blecent.h"

#define PROFILE_NUM         3
#define CBLE_TAG            "CUSTOM-BLE"
#define PROFILE_A_APP_ID    0
#define PROFILE_B_APP_ID    1
#define PROFILE_C_APP_ID    2

static const char *tag = "NimBLE_BLE_CENT";
static int blecent_gap_event(struct ble_gap_event *event, void *arg);
static char tmp_str[200];
//Local functions

//Callbacks
static uint8_t cble_reset(uint8_t para_num);
static uint8_t cble_scan(uint8_t para_num);
static uint8_t cble_conn(uint8_t para_num);
static uint8_t cble_write(uint8_t para_num);


static esp_at_cmd_struct at_nble_cmds[] = {
    {"+CBLERESET", NULL, NULL, cble_reset, NULL},
    {"+CBLESCAN", NULL, NULL, cble_scan, NULL},
    {"+CBLECONN", NULL, NULL, cble_conn, NULL},
    {"+CBLEWRITE", NULL, NULL, cble_write, NULL},
};

void ble_store_config_init(void);



//chr Write callback
static int
blecent_write_cb(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr,
                     void *arg)
{
    printf("blecent_write_cb, status = %d\r\n",error->status);
    sprintf(tmp_str,"+CBLEWRITE: %d\r\n",error->status);
    esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));
    return 0;
}

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    printf("blecent_on_disc_complete\r\n");
    if (status != 0) {
        /* Service discovery failed.  Terminate the connection. */
        printf( "Error: Service discovery failed; status=%d "
                    "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */
    printf( "Service discovery complete; status=%d "
                "conn_handle=%d\n", status, peer->conn_handle);
    
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;
    printf("blecent_gap_event=%x\r\n",event->type);
    
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }
        if (fields.name)
        {
            printf("ADV name: %s\r\n",fields.name);
        }
        
        sprintf(tmp_str,"+CBLESCAN:%.*s,%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\r\n",fields.name_len, fields.name,
        (u32_t)event->disc.addr.val[0],(u32_t)event->disc.addr.val[1],(u32_t)event->disc.addr.val[2],
        (u32_t)event->disc.addr.val[3],(u32_t)event->disc.addr.val[4],(u32_t)event->disc.addr.val[5]);
        esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));

        printf("ADV Addr: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\r\n",
        (u32_t)event->disc.addr.val[0],(u32_t)event->disc.addr.val[1],(u32_t)event->disc.addr.val[2],
        (u32_t)event->disc.addr.val[3],(u32_t)event->disc.addr.val[4],(u32_t)event->disc.addr.val[5]);
        
        
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            /* Connection successfully established. */
            printf("Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            printf("\n");

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                printf( "Failed to add peer; rc=%d\n", rc);
                sprintf(tmp_str,"+CBLECONNECT: Failed to add peer\r\n");
                esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               blecent_on_disc_complete, NULL);
            if (rc != 0) {
                sprintf(tmp_str,"+CBLECONNECT: Failed to discover\r\n");
                esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));
                printf( "Failed to discover services; rc=%d\n", rc);
                return 0;
            }

            sprintf(tmp_str,"+CBLECONNECT:%d\r\n",event->connect.conn_handle);
            esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));
        } else {
            /* Connection attempt failed; resume scanning. */
            printf( "Error: Connection failed; status=%d\n",
                        event->connect.status);
            sprintf(tmp_str,"+CBLECONNECT: Failed with status%d\r\n",event->connect.status);
            esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));            
            //blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        printf("disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        printf("\n");

        sprintf(tmp_str,"+CBLEDISCONNECT:%d\r\n",event->connect.conn_handle);
        esp_at_port_write_data((uint8_t *)tmp_str,strlen(tmp_str));

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        //blecent_scan();
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        printf("discovery complete; reason=%d\n",
                    event->disc_complete.reason);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        printf("encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
        printf("received %s; conn_handle=%d attr_handle=%d "
                    "attr_len=%d\n",
                    event->notify_rx.indication ?
                    "indication" :
                    "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));

        /* Attribute data is contained in event->notify_rx.attr_data. */
        return 0;

    case BLE_GAP_EVENT_MTU:
        printf("mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static void
blecent_on_reset(int reason)
{
    printf( "Resetting state; reason=%d\n", reason);
}

static void
blecent_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

}

static void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    //TODO
}



void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}


// Untested function!!
static uint8_t cble_reset(uint8_t para_num)
{
    int rc;
    nimble_port_stop();
    vTaskDelay(100);

    rc = peer_init(CONFIG_BT_NIMBLE_MAX_CONNECTIONS, 64, 64, 64);
    assert(rc == 0);

    nimble_port_freertos_init(blecent_host_task);
    return ESP_AT_RESULT_CODE_OK;
}

/**
 * @brief Ex: AT+CBLESCAN=20 for 20 seconds discovery
 * 
 * @param para_num 
 * @return uint8_t 
 */
static uint8_t cble_scan(uint8_t para_num)
{
    int32_t cnt = 0;
    int32_t scan_period_s;
    
    if (para_num == 0)
    {
        scan_period_s = 5;
    }
    else
    {
        if (esp_at_get_para_as_digit (cnt++,&scan_period_s) != ESP_AT_PARA_PARSE_RESULT_OK) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    

    if (scan_period_s == 0)
    {
        ble_gap_disc_cancel();
        printf("Scan Aborted by AT\r\n");
    }
    else
    {
        printf("Scan requested by AT\r\n");
    }
    
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        printf( "error determining address type; rc=%d\n", rc);
        return ESP_AT_RESULT_CODE_FAIL;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, scan_period_s*1000, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        printf( "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
    return ESP_AT_RESULT_CODE_OK;
}

/**
 * @brief Ex: AT+CBLECONN="AA:BB:CC:DD",1 //1 is for random address type
 * 
 * @param para_num 
 * @return uint8_t 
 */
static uint8_t cble_conn(uint8_t para_num)
{
    
    ble_addr_t a;
    int32_t a_type;
    int32_t cnt = 0;
    uint8_t* c_mac = NULL;
    char *token;
    int rc;

    if(esp_at_get_para_as_str (cnt++,&c_mac) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (cnt < para_num) {
        if (esp_at_get_para_as_digit (cnt++,&a_type) != ESP_AT_PARA_PARSE_RESULT_OK) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }

    a.type = a_type;
    token = strtok((char*)c_mac, ":");
    cnt = 0;
    while((token != NULL) && (cnt < 6)) 
    {
      a.val[cnt++] = strtol(token,NULL,16);
      token = strtok(NULL, ":");
    }

    printf("Peer address type : %d, value=",a.type);
    print_bytes(a.val,6);
    printf("\r\n");

    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &a /*&disc->addr*/, 30000, NULL,
                         blecent_gap_event, NULL);

    if (rc != 0) 
    {
        printf( "Error: Failed to connect to device; addr_type=%d "
        "addr=%s\n",a.type, addr_str(a.val));
        return ESP_AT_RESULT_CODE_ERROR;
    }

    return ESP_AT_RESULT_CODE_OK;
}

/**
 * @brief Ex: AT+CBLEWRITE=1234,"AA:0F:5F:2E:A1"
 * 
 * @param para_num 
 * @return uint8_t 
 */
static uint8_t cble_write(uint8_t para_num)
{
    const struct peer_chr *chr;
    uint8_t value[32];
    int32_t conn_handle;
    int32_t cnt = 0;
    uint8_t* data_str;
    char *token;
    int rc;

    if(esp_at_get_para_as_digit(cnt++,&conn_handle) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
    }

    if (cnt < para_num) {
        if (esp_at_get_para_as_str(cnt++,&data_str) != ESP_AT_PARA_PARSE_RESULT_OK) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }

    token = strtok((char*)data_str, ":");
    cnt = 0;
    while((token != NULL)) 
    {
      value[cnt++] = strtol(token,NULL,16);
      token = strtok(NULL, ":");
    }

    const struct peer *peer = peer_find((uint16_t)conn_handle);

    chr = peer_chr_find_uuid(peer,BLE_UUID16_DECLARE(0xa001),
                             BLE_UUID16_DECLARE(0xa002));
    
    rc = ble_gattc_write_flat(  peer->conn_handle, 
                                chr->chr.val_handle,
                                value, cnt, 
                                blecent_write_cb, NULL);
    if (rc)
    {
        printf("Error while writing: %d",rc);
        return ESP_AT_RESULT_CODE_ERROR;
    }
    return ESP_AT_RESULT_CODE_OK;
}



void custom_ble_reg(void)
{
    esp_at_custom_cmd_array_regist (at_nble_cmds, sizeof(at_nble_cmds)/sizeof(at_nble_cmds[0]));
    //cble_scan(0);
}

void custom_ble_ll_init(void)
{
    int rc;
    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Configure the host. */
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(CONFIG_BT_NIMBLE_MAX_CONNECTIONS, 64, 64, 64);
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(blecent_host_task);
}


