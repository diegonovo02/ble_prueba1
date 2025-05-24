/*
 * nRF52840-Dongle · PERIFÉRICO BLE  (NCS 2.9)
 * Servicio NUS con ping periódico; potencia TX fija en Kconfig.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/hci.h>

LOG_MODULE_REGISTER(dongle_main, LOG_LEVEL_INF);

// ---------- Configuración rápida ----------
#define APP_PHY_MODE     BT_GAP_LE_PHY_CODED     // 1 M | 2 M | CODED
// Si no es CODED, comentar la siguiente linea.
#define APP_CODED_OPT    BT_CONN_LE_PHY_OPT_CODED_S2  // S8 o S2

// Configuración del "PING" periódico 
#define PING_LEN          244    // Tamaño del paquete en bytes
#define PING_INTERVAL_MS  50   // Intervalo entre pings en ms

// Definición de LED vía Devicetree
#define LED_DONGLE_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_DONGLE_NODE, gpios);

static struct bt_conn *current_conn;        // Conexión activa
static struct k_work_delayable ping_work;   // Trabajo diferido para pings

// ---------- Función para enviar pings periódicos ----------
static void send_ping(struct k_work *w)
{
    static uint32_t seq = 0;

    if (current_conn) {
        static uint8_t payload[PING_LEN];

        // Formatea "PING N" al principio del buffer
        int len = snprintk((char *)payload, PING_LEN, "PING %lu", ++seq);

        // Rellena el resto con 'P' hasta PING_LEN si sobra espacio
        for (int i = len; i < PING_LEN; i++) {
            payload[i] = 'P';
        }

        bt_nus_send(current_conn, payload, PING_LEN);
    }
    // Reprograma el siguiente envío
    k_work_reschedule(&ping_work, K_MSEC(PING_INTERVAL_MS));
}

// ---------- Función auxiliar: leer el PHY que está usando el controlador ----------
static int hci_read_phy(struct bt_conn *c, uint8_t *tx_phy, uint8_t *rx_phy)
{
    uint16_t handle;
    if (bt_hci_get_conn_handle(c, &handle))
        return -EIO;

    struct net_buf *buf = bt_hci_cmd_create(BT_HCI_OP_LE_READ_PHY,
                                            sizeof(struct bt_hci_cp_le_read_phy));
    struct bt_hci_cp_le_read_phy *cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);

    struct net_buf *rsp;
    int err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_PHY, buf, &rsp);
    if (!err) {
        const struct bt_hci_rp_le_read_phy *rp = (void *)rsp->data;
        *tx_phy = rp->tx_phy;
        *rx_phy = rp->rx_phy;
        net_buf_unref(rsp);
    }
    return err;
}

// ---------- Callback NUS (recibe datos del central) ----------
static void nus_rx_cb(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    LOG_INF("RX: %.*s", len, data); // Imprime datos recibidos
}
static struct bt_nus_cb nus_cbs = { .received = nus_rx_cb };

// ---------- Callbacks de conexión/desconexión ----------
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("Connect err %u", err); return; }

    current_conn = bt_conn_ref(conn);   // Refleja conexión
    gpio_pin_set_dt(&led, 1);           // Enciende LED

    struct bt_conn_le_phy_param custom_phy;

    if (APP_PHY_MODE == BT_GAP_LE_PHY_CODED) {
        custom_phy.options = APP_CODED_OPT;
        custom_phy.pref_tx_phy = BT_GAP_LE_PHY_CODED;
        custom_phy.pref_rx_phy = BT_GAP_LE_PHY_CODED;

        if (APP_CODED_OPT == BT_CONN_LE_PHY_OPT_CODED_S2) {
            LOG_INF("Solicitando PHY CODED S=2");
        } else if (APP_CODED_OPT == BT_CONN_LE_PHY_OPT_CODED_S8) {
            LOG_INF("Solicitando PHY CODED S=8");
        } else {
            LOG_WRN("APP_CODED_OPT desconocido, usando S=8 por defecto");
            custom_phy.options = BT_CONN_LE_PHY_OPT_CODED_S8;
        }

    } else if (APP_PHY_MODE == BT_GAP_LE_PHY_2M) {
        custom_phy.options = BT_CONN_LE_PHY_OPT_NONE;
        custom_phy.pref_tx_phy = BT_GAP_LE_PHY_2M;
        custom_phy.pref_rx_phy = BT_GAP_LE_PHY_2M;
        LOG_INF("Solicitando PHY 2M");

    } else if (APP_PHY_MODE == BT_GAP_LE_PHY_1M) {
        custom_phy.options = BT_CONN_LE_PHY_OPT_NONE;
        custom_phy.pref_tx_phy = BT_GAP_LE_PHY_1M;
        custom_phy.pref_rx_phy = BT_GAP_LE_PHY_1M;
        LOG_INF("Solicitando PHY 1M");

    } else {
        LOG_WRN("APP_PHY_MODE desconocido, usando PHY 1M por defecto");
        custom_phy.options = BT_CONN_LE_PHY_OPT_NONE;
        custom_phy.pref_tx_phy = BT_GAP_LE_PHY_1M;
        custom_phy.pref_rx_phy = BT_GAP_LE_PHY_1M;
    }

    bt_conn_le_phy_update(conn, &custom_phy);


    LOG_INF("Central connected → starting pings");
    k_work_schedule(&ping_work, K_MSEC(1500)); // Esperar 1,5s antes de enviar el primer ping
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (0x%02X)", reason);
    gpio_pin_set_dt(&led, 0);   // Apaga LED

    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; } // Libera conexión
    k_work_cancel_delayable(&ping_work);    // Detiene pings
}

static struct bt_conn_cb conn_cbs = {
    .connected    = connected,
    .disconnected = disconnected,
};

// ---------- Datos de advertising con UUID NUS ----------
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
        BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0x6E,0x40,0x01,0x00,0xB5,0xA3,0xF3,0x93,
        0xE0,0xA9,0x00,0xE5,0x0E,0xC9,0xA2,0x9E) // UUID NUS
};

// ---------- MAIN ----------
void main(void)
{
    if (!device_is_ready(led.port)) return; // Verifica que el GPIO del LED esté disponible
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); // Cofigurar LED como salida y apagarlo

    if (bt_enable(NULL)) { LOG_ERR("bt_enable() failed"); return; } // Inicializa Bluetooth
    bt_conn_cb_register(&conn_cbs); // Registra callbacks de conexión

    if (bt_nus_init(&nus_cbs)) { LOG_ERR("NUS init failed"); return; } // Inicializa servicio NUS

    k_work_init_delayable(&ping_work, send_ping); // Inicializa el envio de "PING N"

    int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0); // Comenzamos adversiting BLE
    LOG_INF("Advertising %s (%d)", err ? "FAIL" : "started", err);
}
