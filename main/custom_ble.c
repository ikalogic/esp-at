

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
static uint8_t peer_addr[6];
static char tmp_str[200];
//Local functions

//Callbacks
static uint8_t cble_scan(uint8_t para_num);
static uint8_t cble_conn(uint8_t para_num);


static esp_at_cmd_struct at_nble_cmds[] = {
    {"+CBLESCAN", NULL, NULL, cble_scan, NULL},
    {"+CBLECONN", NULL, NULL, cble_conn, NULL},
};

void ble_store_config_init(void);

/**
 * Application callback.  Called when the attempt to subscribe to notifications
 * for the ANS Unread Alert Status characteristic has completed.
 */
static int
blecent_on_subscribe(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr,
                     void *arg)
{
    printf("blecent_on_subscribe\r\n");
    printf("Subscribe complete; status=%d conn_handle=%d "
                "attr_handle=%d\n",
                error->status, conn_handle, attr->handle);

    return 0;
}

//chr Write callback
static int
blecent_write_cb(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr,
                     void *arg)
{
    printf("blecent_write_cb, status = %d\r\n",error->status);
    return 0;
}
/**
 * Application callback.  Called when the write to the ANS Alert Notification
 * Control Point characteristic has completed.
 */
static int
blecent_on_write(uint16_t conn_handle,                  //delete this function?
                 const struct ble_gatt_error *error,
                 struct ble_gatt_attr *attr,
                 void *arg)
{
    printf("blecent_on_write\r\n");
    MODLOG_DFLT(INFO,
                "Write complete; status=%d conn_handle=%d attr_handle=%d\n",
                error->status, conn_handle, attr->handle);

    /* Subscribe to notifications for the Unread Alert Status characteristic.
     * A central enables notifications by writing two bytes (1, 0) to the
     * characteristic's client-characteristic-configuration-descriptor (CCCD).
     */
    const struct peer_dsc *dsc;
    uint8_t value[2];
    int rc;
    const struct peer *peer = peer_find(conn_handle);

    dsc = peer_dsc_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLECENT_SVC_ALERT_UUID),
                             BLE_UUID16_DECLARE(BLECENT_CHR_UNR_ALERT_STAT_UUID),
                             BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if (dsc == NULL) {
        printf( "Error: Peer lacks a CCCD for the Unread Alert "
                    "Status characteristic\n");
        goto err;
    }

    value[0] = 1;
    value[1] = 0;
    rc = ble_gattc_write_flat(conn_handle, dsc->dsc.handle,
                              value, sizeof value, blecent_on_subscribe, NULL);
    if (rc != 0) {
        printf( "Error: Failed to subscribe to characteristic; "
                    "rc=%d\n", rc);
        goto err;
    }

    return 0;
err:
    /* Terminate the connection. */
    return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Application callback.  Called when the read of the ANS Supported New Alert
 * Category characteristic has completed.
 */
static int
blecent_on_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    printf("Read complete; status=%d conn_handle=%d", error->status,
                conn_handle);
    if (error->status == 0) {
        printf(" attr_handle=%d value=", attr->handle);
        print_mbuf(attr->om);
    }
    printf("\n");

    /* Write two bytes (99, 100) to the alert-notification-control-point
     * characteristic.
     */
    const struct peer_chr *chr;
    uint8_t value[2];
    int rc;
    const struct peer *peer = peer_find(conn_handle);

    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLECENT_SVC_ALERT_UUID),
                             BLE_UUID16_DECLARE(BLECENT_CHR_ALERT_NOT_CTRL_PT));
    if (chr == NULL) {
        printf( "Error: Peer doesn't support the Alert "
                    "Notification Control Point characteristic\n");
        goto err;
    }

    value[0] = 99;
    value[1] = 100;
    rc = ble_gattc_write_flat(conn_handle, chr->chr.val_handle,
                              value, sizeof value, blecent_on_write, NULL);
    if (rc != 0) {
        printf( "Error: Failed to write characteristic; rc=%d\n",
                    rc);
        goto err;
    }

    return 0;
err:
    /* Terminate the connection. */
    return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Performs three GATT operations against the specified peer:
 * 1. Reads the ANS Supported New Alert Category characteristic.
 * 2. After read is completed, writes the ANS Alert Notification Control Point characteristic.
 * 3. After write is completed, subscribes to notifications for the ANS Unread Alert Status
 *    characteristic.
 *
 * If the peer does not support a required service, characteristic, or
 * descriptor, then the peer lied when it claimed support for the alert
 * notification service!  When this happens, or if a GATT procedure fails,
 * this function immediately terminates the connection.
 */
static void
blecent_read_write_subscribe(const struct peer *peer)
{
    const struct peer_chr *chr;
    int rc;

    /* Read the supported-new-alert-category characteristic. */
    chr = peer_chr_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLECENT_SVC_ALERT_UUID),
                             BLE_UUID16_DECLARE(BLECENT_CHR_SUP_NEW_ALERT_CAT_UUID));
    if (chr == NULL) {
        printf( "Error: Peer doesn't support the Supported New "
                    "Alert Category characteristic\n");
        goto err;
    }

    rc = ble_gattc_read(peer->conn_handle, chr->chr.val_handle,
                        blecent_on_read, NULL);
    if (rc != 0) {
        printf( "Error: Failed to read characteristic; rc=%d\n",
                    rc);
        goto err;
    }

    return;
err:
    /* Terminate the connection. */
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    const struct peer_dsc *dsc;
    uint8_t value[2];
    uint16_t chr_handle;
    int rc;
    //const struct peer *peer = peer_find(conn_handle);

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
    
    const struct peer_svc *svc;
    const struct peer_chr *chr;
    SLIST_FOREACH(svc, &peer->svcs, next) {
        printf("Found service %x\r\n",svc->svc.uuid.u16.value);

        SLIST_FOREACH(chr, &svc->chrs, next) {
            printf("------chr: %x\r\n",chr->chr.uuid.u16.value);
        }
    }

    chr = peer_chr_find_uuid(peer,BLE_UUID16_DECLARE(0xa001),
                             BLE_UUID16_DECLARE(0xa002));
    
    value[0] = 0x11;
    value[1] = 0;
    rc = ble_gattc_write_flat(  peer->conn_handle, 
                                chr->chr.val_handle,
                                value, 2, 
                                blecent_write_cb, NULL);
    if (rc)
    {
        printf("Error while writing: %d",rc);
    }
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void
blecent_scan(void)
{
    printf("blecent_scan\r\n");
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        printf( "error determining address type; rc=%d\n", rc);
        return;
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

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        printf( "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
static int
blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
            disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return rc;
    }

    //ec:b5:6d:78:9a:61
    if (strncmp((char*)fields.name,"SPD",3) == 0)
    {
        printf("Found our device!\r\n");
        return 1;
    }

    return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;

    /* Don't do anything if we don't care about this advertiser. */
    if (!blecent_should_connect(disc)) {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        printf("Failed to cancel scan; rc=%d\n", rc);
        return;
    }


    return; //JFT
    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */

    //Test connecting using a fixed address
    ble_addr_t a;
    a.type = 1; //Random
    
    memcpy((void*)a.val,(void*)"\x61\x9a\x78\x6d\xb5\xec",6);
    //memcpy((void*)a.val,(void*)disc->addr.val,6);
    printf("Peer address type : %d, value=",a.type);
    print_bytes(a.val,6);
    printf("\r\n");

    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &a /*&disc->addr*/, 30000, NULL,
                         blecent_gap_event, NULL);
    //rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &disc->addr, 30000, NULL,
    //                    blecent_gap_event, NULL);

    if (rc != 0) {
        printf( "Error: Failed to connect to device; addr_type=%d "
                    "addr=%s\n",
                    disc->addr.type, addr_str(disc->addr.val));
        return;
    }
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
        
        
        /* An advertisment report was received during GAP discovery. */
        //print_adv_fields(&fields);

        /* Try to connect to the advertiser if it looks interesting. */
        //blecent_connect_if_interesting(&event->disc);
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
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               blecent_on_disc_complete, NULL);
            if (rc != 0) {
                printf( "Failed to discover services; rc=%d\n", rc);
                return 0;
            }
        } else {
            /* Connection attempt failed; resume scanning. */
            printf( "Error: Connection failed; status=%d\n",
                        event->connect.status);
            blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        printf("disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        printf("\n");

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        blecent_scan();
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


    // ble_addr_t a;
    // a.type = 1; //Random
    // memcpy((void*)a.val,(void*)"\x61\x9a\x78\x6d\xb5\xec",6);
    // //memcpy((void*)a.val,(void*)disc->addr.val,6);
    // printf("Peer address type : %d, value=",a.type);
    // print_bytes(a.val,6);
    // printf("\r\n");

    // rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &a /*&disc->addr*/, 30000, NULL,
    //                      blecent_gap_event, NULL);
    /* Begin scanning for a peripheral to connect to. */
    //blecent_scan();
    //cble_scan(0);
}

void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}


/**
 * @brief Ex: AT+CBLESCAN=20 for 20 seconds discovery
 * 
 * @param para_num 
 * @return uint8_t 
 */
static uint8_t cble_scan(uint8_t para_num)
{
    esp_err_t ret;
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

    if (cnt < para_num) {
        if(esp_at_get_para_as_str (cnt++,&c_mac) != ESP_AT_PARA_PARSE_RESULT_OK) {
            return ESP_AT_RESULT_CODE_ERROR;
        }
    }
    if (esp_at_get_para_as_digit (cnt++,&a_type) != ESP_AT_PARA_PARSE_RESULT_OK) {
        return ESP_AT_RESULT_CODE_ERROR;
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
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(blecent_host_task);
}


