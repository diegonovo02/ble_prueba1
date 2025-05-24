/*
 * nRF5340-DK · CENTRAL BLE  (NCS 2.9)
 * Escanea nombre "Dongle", conecta, solicita PHY, descubre NUS,
 * mide RSSI y bitrate.  Potencia fija vía prj.conf.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/byteorder.h>


LOG_MODULE_REGISTER(central_uart, LOG_LEVEL_INF);

// ---------- Configuración rápida ----------
#define TARGET_NAME "Dongle"                    // Nombre del dispositivo BLE al que se conectará
#define APP_PHY_MODE     BT_GAP_LE_PHY_CODED     // 1 M | 2 M | CODED
// Si no es CODED, comentar la siguiente linea.
#define APP_CODED_OPT  BT_CONN_LE_PHY_OPT_CODED_S2  // S8 o S2

// ------------------------------------------

// Variables para métricas de bitrate 
static uint32_t first_t_ms, bytes_total;

// Estado global de conexiones y cliente NUS
static struct bt_conn        *default_conn;
static struct bt_nus_client   nus;
static struct bt_conn        *pending_conn;

// ---------- Función auxiliar: leer RSSI ----------
static int hci_read_rssi(struct bt_conn *c, int8_t *rssi)
{
    uint16_t handle;
    if (bt_hci_get_conn_handle(c, &handle)) 
        return -EIO; // Error si no obtiene el handle

    // Crear comando HCI para leer RSSI
    struct net_buf *buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(struct bt_hci_cp_read_rssi));
    struct bt_hci_cp_read_rssi *cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle); // Convertimos a little-endian

    struct net_buf *rsp;    // Buffer de respuesta
    int err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp); // Envío comando sincrónicamente
    if (!err) {
        *rssi = ((struct bt_hci_rp_read_rssi *)rsp->data)->rssi; // Extrae RSSI en dBm
        net_buf_unref(rsp); // Liberamos buffer de respuesta
    }
    return err;
}

//---------- Función auxiliar: leer el PHY que está usando el controlador ----------
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

// ---------- Callbacks del cliente NUS ----------
static uint8_t nus_rx_cb(struct bt_nus_client *cl, const uint8_t *data, uint16_t len)
{
    static uint32_t last_seq = 0;
    uint32_t seq = 0;

    if (!first_t_ms) 
        first_t_ms = k_uptime_get_32(); // Tiempo del primer paquete
    bytes_total += len;

    // Parsea "PING N" y se detectan pérdidas de paquetes
    if (len >= 6 && sscanf((const char *)data, "PING %lu", &seq) == 1) {
        if (last_seq != 0 && seq != last_seq + 1) {
            LOG_WRN("Pérdida: esperado %lu pero llegó %lu", last_seq + 1, seq);
        }
        last_seq = seq;
    } else {
        LOG_WRN("No se pudo interpretar secuencia: %.*s", len, data);
    }

    int8_t rssi;
    if (!hci_read_rssi(default_conn, &rssi)) // Lee RSSI
        LOG_INF("Paquete %lu - %uB - RSSI=%d dBm", seq, len, rssi);

    return BT_GATT_ITER_CONTINUE;   // Continúa iternando notificaciones
}

static const struct bt_nus_client_cb nus_cbs = { .received = nus_rx_cb }; // Se asigna callback al inicizalizar cliente NUS

// ---------- Descubrimiento y suscripción GATT (NUS) ----------
static void dm_discovered(struct bt_gatt_dm *dm, void *ctx)
{
    struct bt_nus_client *cl = ctx;
    bt_nus_handles_assign(dm, cl);  // Asigna handles de servicios/características
    bt_gatt_dm_data_release(dm);    // Liberar datos del DM (GATT Disovery Manager)

    // Habilita notificaciones de RX (del servidor al cliente)
    if (bt_nus_subscribe_receive(cl)) {
        LOG_ERR("NUS rx_notif_enable failed");
    } else {
        LOG_INF("NUS listo: suscrito y recibiendo datos");
    }
}

static void dm_not_found(struct bt_conn *c, void *ctx)
{
    LOG_WRN("NUS no encontrado");
}
static void dm_error(struct bt_conn *c, int err, void *ctx)
{
    LOG_ERR("gatt_dm error %d", err);
}
// Callbacks para éxito, servicio no encontrado y error
static const struct bt_gatt_dm_cb dm_cb = {
    .completed         = dm_discovered,
    .service_not_found = dm_not_found,
    .error_found       = dm_error,
};

// ---------- Callbacks de conexión/desconexión ----------
static void connected(struct bt_conn *c, uint8_t err)
{
    if (err) { LOG_ERR("Conn err %u", err); return; }

    default_conn = bt_conn_ref(c);  // Guarda referencia a la conexión activa
    first_t_ms = bytes_total = 0;   // Reiniciar variables a 0

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

    bt_conn_le_phy_update(c, &custom_phy);

    bt_nus_client_init(&nus, &(struct bt_nus_client_init_param){ .cb = nus_cbs });  // Inicia el cliente NUS

    bt_gatt_dm_start(c, BT_UUID_NUS_SERVICE, &dm_cb, &nus);  // Inicia descubrimiento dinámico de servicio NUS
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{   
    LOG_INF("Disconnected (0x%02X)", reason);   // Log avisando de la desconexión

    if (first_t_ms && bytes_total) {
        uint32_t elapsed_ms = k_uptime_get_32() - first_t_ms;   // Tiempo transcurrido desde la primera recepción
        uint32_t kbps = (bytes_total * 8 * 1000U) / elapsed_ms; // Calcula el bitrate medio en kbps
        LOG_INF("Bitrate medio = %u kbps", kbps);
    }
    if (default_conn) { bt_conn_unref(default_conn); default_conn = NULL; } // Liberamos la referencia a la conexión

    bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);  // Reinicia escaneo
}

static void le_phy_updated(struct bt_conn *conn,
                           struct bt_conn_le_phy_info *param)
{
    const char *tx_phy_str = "UNKNOWN";
    const char *rx_phy_str = "UNKNOWN";

    switch (param->tx_phy) {
        case BT_GAP_LE_PHY_1M:    tx_phy_str = "1M";    break;
        case BT_GAP_LE_PHY_2M:    tx_phy_str = "2M";    break;
        case BT_GAP_LE_PHY_CODED: tx_phy_str = "CODED"; break;
    }

    switch (param->rx_phy) {
        case BT_GAP_LE_PHY_1M:    rx_phy_str = "1M";    break;
        case BT_GAP_LE_PHY_2M:    rx_phy_str = "2M";    break;
        case BT_GAP_LE_PHY_CODED: rx_phy_str = "CODED"; break;
    }

    LOG_INF("PHY actualizado: TX = %s, RX = %s", tx_phy_str, rx_phy_str);

    // Verificación extra leyendo el controller
    uint8_t tx_phy_raw, rx_phy_raw;
    if (!hci_read_phy(conn, &tx_phy_raw, &rx_phy_raw))
        LOG_INF("LE Read PHY → TX=%u, RX=%u", tx_phy_raw, rx_phy_raw);
}


BT_CONN_CB_DEFINE(conn_cbs) = { // Callbacks para eventos de conexión BLE
    .connected    = connected,
    .disconnected = disconnected,
    .le_phy_updated   = le_phy_updated,
};

// ---------- Filtrar nombre en paquetes de advertising ----------
static bool adv_name_match(struct net_buf_simple *ad)
{
    while (ad->len) {
        uint8_t len  = net_buf_simple_pull_u8(ad);
        uint8_t type = net_buf_simple_pull_u8(ad);

        // Tipo de dato: Nombre completo o abreviado
        if ((type == BT_DATA_NAME_COMPLETE ||
             type == BT_DATA_NAME_SHORTENED) &&
            len - 1 == strlen(TARGET_NAME) &&
            !memcmp(ad->data, TARGET_NAME, len - 1))
            return true;

        net_buf_simple_pull(ad, len - 1);
    }
    return false;
}

// ---------- Callbacks de escaneo ----------
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    // Filtra solo ADV_IND, ADV_SCAN_IND o SCAN_RSP
    if (type != BT_GAP_ADV_TYPE_ADV_IND      &&     // Advertising estándar conectable y detectable
        type != BT_GAP_ADV_TYPE_ADV_SCAN_IND &&     // Advertising que permite escaneo (respuesta a scan request)
        type != BT_GAP_ADV_TYPE_SCAN_RSP)           // Respuesta a un scan request (contiene datos adicionales)
        return;

    if (!adv_name_match(ad))
        return; // Si el nombre no coincide, ignora paquete

    LOG_INF("¡%s encontrado! Conectando…", TARGET_NAME);

    // Copia la dirección Bluetooth LE del dispositivo encontrado para usarla en la conexión
    bt_addr_le_t peer;
    bt_addr_le_copy(&peer, addr);

    bt_le_scan_stop();  // Detiene el escaneo activo antes de iniciar la conexión para ahorrar recursos

    // Crea una conexión BLE con el dispositivo cuya dirección está en 'peer'
    int err = bt_conn_le_create(&peer, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &pending_conn);
    if (err) {
        LOG_ERR("bt_conn_le_create() → %d", err);
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    }
}

// ---------- MAIN ----------
int main(void)
{
    if (bt_enable(NULL)) { // Inicializamos el stack Bluetooth
        LOG_ERR("bt_enable() failed");
        return 0;
    }

    // Configurar parametros de escaneo BLE (En BLE, tanto intervalo como ventana se expresan en unidades de 0.625 ms.)
    struct bt_le_scan_param scan_param = {
        .type     = BT_LE_SCAN_TYPE_ACTIVE, // Escaneo activo (solicita respuestas scan request para obtener más datos)
        .options  = BT_LE_SCAN_OPT_NONE,    // Sin opciones especiales
        .interval = 0x0060, // Tiempo entre ventanas de escaneo (0x60 = 96 --> 96 x 0.625ms = 60ms)
        .window   = 0x0030, // Duración de cada ventana de escaneo  (0x30 = 45 --> 48 x 0.625ms = 30ms)
    };


    bt_le_scan_start(&scan_param, device_found); // Inicia el escaneo BLE 

    LOG_INF("Central listo, escaneando…");
    return 0;
}
