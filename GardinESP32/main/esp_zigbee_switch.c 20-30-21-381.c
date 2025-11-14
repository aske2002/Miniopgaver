/*
 * Zigbee HA Window Covering (Curtain) Example
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_zigbee_gateway.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "ESP_ZB_CURTAIN";

/* === Motor control pins === */
#define IN1 GPIO_NUM_2
#define IN2 GPIO_NUM_3
#define IN3 GPIO_NUM_4
#define IN4 GPIO_NUM_6

#define LIMIT_OPEN GPIO_NUM_7
#define LIMIT_CLOSE GPIO_NUM_8

#define HA_ESP_CURTAIN_ENDPOINT 10

typedef enum
{
    MOTOR_IDLE,
    SWITCH_SHOULD_BE_ON,
    SWITCH_SHOULD_BE_OFF
} motor_state_t;

static motor_state_t switch_state = MOTOR_IDLE;
static volatile bool motor_should_stop = false;

static uint8_t lift_pct = 0xFF; // 0xFF = unknown per spec, set to 0 or 100 when you know
static uint8_t operational_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_ONLINE;

// Step delay in milliseconds (adjust for speed)
#define STEP_DELAY_MS 20

// Full-step 4-step sequence
static const int step_sequence[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}};

static void write_wc_attrs(void)
{
    esp_zb_zcl_set_attribute_val(HA_ESP_CURTAIN_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
                                 &lift_pct, false);

    esp_zb_zcl_set_attribute_val(HA_ESP_CURTAIN_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID,
                                 &operational_status, false);
}

static void report_wc_attrs(void)
{
    write_wc_attrs();

    esp_zb_zcl_report_attr_cmd_t req = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0x0000, // trust center/coordinator
            .dst_endpoint = 1,               // coordinator’s endpoint is often 1
            .src_endpoint = HA_ESP_CURTAIN_ENDPOINT,
        },
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .attributeID = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
    };

    esp_zb_zcl_report_attr_cmd_req(&req);
    req.attributeID = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_STATUS_ID;
    esp_zb_zcl_report_attr_cmd_req(&req);
}

static void set_operational_status_opening(void) { operational_status |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL; }
static void set_operational_status_closing(void) { operational_status |= ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL; }
static void set_operational_status_stopped(void) { operational_status &= ~ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL; }

static void on_limit_open(void)
{
    lift_pct = 100;
    set_operational_status_stopped();
    report_wc_attrs();
}
static void on_limit_close(void)
{
    lift_pct = 0;
    set_operational_status_stopped();
    report_wc_attrs();
}
static void on_started_turning_on(void)
{
    set_operational_status_opening();
    report_wc_attrs();
}
static void on_started_turning_off(void)
{
    set_operational_status_closing();
    report_wc_attrs();
}

/* === Motor control === */
static void set_switch_state(bool isOn)
{
    gpio_set_level(IN1, isOn);
}

static void switch_turn_on(void)
{
    set_switch_state(true);
}

static void switch_turn_off(void)
{
    set_switch_state(false);
}

static void switch_emergency(void)
{
    set_switch_state(false);
    switch_state = SWITCH_SHOULD_BE_OFF;
}

static void switch_task(void *pvParameters)
{
    while (1)
    {
        if (switch_state == SWITCH_SHOULD_BE_ON)
        {
            switch_turn_on();
        }
        else if (switch_state == SWITCH_SHOULD_BE_OFF)
        {
            switch_turn_off();
        }
        else
        {
            // Idle state, small delay to yield CPU
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* === Zigbee command handler === */
static esp_err_t curtain_cluster_cmd_handler(esp_zb_zcl_window_covering_movement_message_t message)
{
    switch (message.command)
    {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        switch_state = SWITCH_SHOULD_BE_ON;
        on_started_turning_on();
        break;

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        switch_state = SWITCH_SHOULD_BE_OFF;
        on_started_turning_off();
        break;

    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        ESP_LOGI(TAG, "Zigbee command: STOP");
        switch_emergency();
        break;
    default:
        ESP_LOGW(TAG, "Unknown WindowCovering command: %d", message.command);
        break;
    }
    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), direction(0x%x), attribute(0x%x)",
                 variable->status, message->info.cluster, variable->direction, variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

static void esp_app_zb_attribute_handler(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute)
{
    ESP_LOGI(TAG, "Attribute handler: cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
             cluster_id, attribute->id, attribute->data.type,
             attribute->data.value ? *(uint8_t *)attribute->data.value : 0);
    /* Handle attribute value here if needed */
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    ESP_LOGI(TAG, "Read attribute response: from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->info.src_address.u.short_addr, message->info.src_endpoint,
             message->info.dst_endpoint, message->info.cluster);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable)
    {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)",
                 variable->status, message->info.cluster,
                 variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        if (variable->status == ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            esp_app_zb_attribute_handler(message->info.cluster, &variable->attribute);
        }

        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster);
    esp_app_zb_attribute_handler(message->cluster, &message->attribute);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Zigbee action callback received: id(0x%x)", callback_id);
    switch (callback_id)
    {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        esp_zb_zcl_window_covering_movement_message_t *move_msg = (esp_zb_zcl_window_covering_movement_message_t *)message;
        ret = curtain_cluster_cmd_handler(*move_msg);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}
/* === Zigbee signals === */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *sig;

    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Device started (%s)",
                     esp_zb_bdb_is_factory_new() ? "factory-new" : "reboot");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Starting network steering...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Startup failed: %s", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Joined Zigbee network");
        }
        else
        {
            ESP_LOGI(TAG, "Steering failed");
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s, status: %s",
                 esp_zb_zdo_signal_to_string(sig_type),
                 esp_err_to_name(err_status));
        break;
    }
}

/* === Cluster creation === */
static esp_zb_cluster_list_t *curtain_clusters_create(esp_zb_window_covering_cfg_t *covering_cfg)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    /* Basic cluster */
    esp_zb_attribute_list_t *basic_cluster =
        esp_zb_basic_cluster_create(&covering_cfg->basic_cfg);

    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER);

    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Identify cluster */
    esp_zb_cluster_list_add_identify_cluster(cluster_list,
                                             esp_zb_identify_cluster_create(&covering_cfg->identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Groups cluster */
    esp_zb_cluster_list_add_groups_cluster(cluster_list,
                                           esp_zb_groups_cluster_create(&covering_cfg->groups_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Scenes cluster */
    esp_zb_cluster_list_add_scenes_cluster(cluster_list,
                                           esp_zb_scenes_cluster_create(&covering_cfg->scenes_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* Window covering cluster */
    esp_zb_attribute_list_t *wc_cluster =
        esp_zb_window_covering_cluster_create(&covering_cfg->window_cfg);
    esp_zb_cluster_list_add_window_covering_cluster(cluster_list, wc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    return cluster_list;
}

/* === Zigbee main task === */
static esp_zb_ep_list_t *curtain_ep_create(uint8_t endpoint_id, esp_zb_window_covering_cfg_t *covering_cfg)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t endpoint_cfg = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(ep_list, curtain_clusters_create(covering_cfg), endpoint_cfg);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_cfg);

    /* Create full device config using your macro */
    esp_zb_window_covering_cfg_t wc_device_cfg = ESP_ZB_DEFAULT_WINDOW_COVERING_CONFIG();
    wc_device_cfg.window_cfg.covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_ROLLERSHADE_2_MOTOR;
    /* Create and register endpoint */
    esp_zb_ep_list_t *ep_list = curtain_ep_create(HA_ESP_CURTAIN_ENDPOINT, &wc_device_cfg);
    esp_zb_device_register(ep_list);

    lift_pct = (gpio_get_level(LIMIT_OPEN) == 0) ? 100 : (gpio_get_level(LIMIT_CLOSE) == 0) ? 0 : 0xFF;

    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

static void setup_gpio(void)
{
    gpio_reset_pin(IN1);

    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LIMIT_OPEN) | (1ULL << LIMIT_CLOSE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);
}

/* === App main === */
void app_main(void)
{
    // GPIO init
    setup_gpio();

    ESP_ERROR_CHECK(nvs_flash_init());

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    ESP_LOGI(TAG, "Starting Zigbee Curtain Device...");
    xTaskCreate(switch_task, "switch_task", 4096, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
