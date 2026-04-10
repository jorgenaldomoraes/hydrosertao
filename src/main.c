/**
 * =====================================================================
 * HydroSertão - Sistema Inteligente de Irrigação IoT
 * =====================================================================
 * Plataforma : BitDogLab (RP2040)
 * Autor      : Jorgenaldo Silva Moraes
 * Versão     : 1.0.0
 *
 * Servidor HTTP embutido — acesse http://IP_DA_PLACA no navegador
 * =====================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "ssd1306.h"

/* ─── WS2812 ─────────────────────────────────────────────────────── */
#define WS2812_PIN      7
#define NUM_LEDS        25

/* ─── Wi-Fi ──────────────────────────────────────────────────────── */
#define WIFI_SSID       "Tayna_5GHz"
#define WIFI_PASSWORD   "Silva@123"

/* ─── GPIOs ──────────────────────────────────────────────────────── */
#define LED_R_PIN       13
#define LED_G_PIN       11
#define LED_B_PIN       12
#define BUZZER_PIN      21
#define BTN_A_PIN       5
#define BTN_B_PIN       6
#define JOYSTICK_Y_ADC  1
#define TEMP_ADC        4

/* ─── I2C ────────────────────────────────────────────────────────── */
#define I2C_PORT        i2c1
#define I2C_SDA_PIN     14
#define I2C_SCL_PIN     15
#define I2C_FREQ        400000

/* ─── UART ───────────────────────────────────────────────────────── */
#define UART_PORT       uart0
#define UART_BAUD       115200
#define UART_TX_PIN     0
#define UART_RX_PIN     1

/* ─── Limiares ───────────────────────────────────────────────────── */
#define UMIDADE_MINIMA  30.0f
#define UMIDADE_MAXIMA  70.0f
#define INTERVALO_MS    1000

/* ─── Estado ─────────────────────────────────────────────────────── */
typedef struct {
    float    umidade;
    float    temperatura;
    bool     irrigando;
    bool     emergencia;
    bool     modo_manual;
    uint32_t tempo_irrigando;
} SistemaState;

volatile SistemaState sistema = {0};
volatile bool flag_modo_mudou = false;
volatile bool flag_emergencia = false;

/* ─── WS2812 via PIO ─────────────────────────────────────────────── */
static PIO  ws_pio    = pio0;
static uint ws_sm     = 0;
static uint ws_offset = 0;

static const uint16_t ws2812_program_instructions[] = {
    0x6221, 0x1123, 0xe001, 0x1100, 0xe000, 0x0000,
};
static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length       = 6,
    .origin       = -1,
};

static inline void ws2812_put_pixel(uint32_t grb) {
    pio_sm_put_blocking(ws_pio, ws_sm, grb << 8u);
}
static inline uint32_t urgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

static const uint8_t padrao_gota[25]   = {0,0,1,0,0, 0,1,1,1,0, 1,1,1,1,1, 1,1,1,1,1, 0,1,1,1,0};
static const uint8_t padrao_sol[25]    = {1,0,1,0,1, 0,1,1,1,0, 1,1,1,1,1, 0,1,1,1,0, 1,0,1,0,1};
static const uint8_t padrao_x[25]      = {1,0,0,0,1, 0,1,0,1,0, 0,0,1,0,0, 0,1,0,1,0, 1,0,0,0,1};
static const uint8_t padrao_manual[25] = {0,1,1,1,0, 0,0,0,1,0, 0,0,1,0,0, 0,0,0,0,0, 0,0,1,0,0};

void matriz_mostrar(const uint8_t *p, uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++)
        ws2812_put_pixel(p[i] ? urgb(r, g, b) : urgb(0, 0, 0));
    sleep_us(300);
}
void matriz_apagar(void) {
    for (int i = 0; i < NUM_LEDS; i++) ws2812_put_pixel(urgb(0, 0, 0));
    sleep_us(300);
}

/* ─── Buffer HTTP ────────────────────────────────────────────────── */
static char http_response[3000];

/* ─── Protótipos ─────────────────────────────────────────────────── */
void hardware_init(void);
void ws2812_init(void);
void display_init(void);
void wifi_init(void);
void start_http_server(void);
float ler_umidade(void);
float ler_temperatura(void);
void controlar_irrigacao(bool ligar);
void set_led(bool r, bool g, bool b);
void buzzer_beep(uint32_t freq_hz, uint32_t duracao_ms);
void atualizar_display(void);
void atualizar_matriz(void);
void log_serial(const char *msg);
void callback_emergencia(uint gpio, uint32_t events);
void callback_modo(void);

/* ─── Servidor HTTP embutido ─────────────────────────────────────── */
static void criar_pagina_html(void) {
    const char *cor_irrig = sistema.irrigando  ? "#2196F3" : "#9e9e9e";
    const char *st_irrig  = sistema.irrigando  ? "LIGADA"  : "DESLIGADA";
    const char *cor_emerg = sistema.emergencia ? "#f44336" : "#4CAF50";
    const char *st_emerg  = sistema.emergencia ? "EMERGENCIA" : "NORMAL";
    const char *st_modo   = sistema.modo_manual ? "MANUAL" : "AUTOMATICO";
    const char *cor_modo  = sistema.modo_manual ? "#FF9800" : "#4CAF50";
    int upct = (int)sistema.umidade;
    const char *cor_umid  = upct >= 30 ? "#4CAF50" : "#f44336";

    snprintf(http_response, sizeof(http_response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=UTF-8\r\n"
        "Refresh: 3\r\n\r\n"
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>HydroSertao</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;background:#f0f4f8;padding:16px;margin:0}"
        "h1{text-align:center;color:#1a5276;margin-bottom:4px}"
        ".sub{text-align:center;color:#888;font-size:12px;margin-bottom:16px}"
        ".grid{display:flex;flex-wrap:wrap;gap:12px;justify-content:center}"
        ".card{background:#fff;border-radius:14px;padding:18px;min-width:160px;"
        "text-align:center;box-shadow:0 3px 8px rgba(0,0,0,.1)}"
        ".lbl{font-size:11px;color:#999;text-transform:uppercase;margin-bottom:6px}"
        ".val{font-size:34px;font-weight:bold;color:#1a5276}"
        ".unit{font-size:15px;color:#888}"
        ".badge{padding:7px 14px;border-radius:20px;color:#fff;"
        "font-weight:bold;font-size:13px;display:inline-block;margin-top:6px}"
        ".bar-bg{background:#e0e0e0;border-radius:6px;height:12px;margin-top:8px;overflow:hidden}"
        ".bar{height:100%%;border-radius:6px}"
        ".foot{text-align:center;color:#bbb;font-size:11px;margin-top:20px}"
        "a.btn{display:inline-block;margin:4px;padding:8px 16px;border-radius:8px;"
        "background:#1a5276;color:#fff;text-decoration:none;font-size:13px}"
        "a.btn.red{background:#c0392b}"
        "</style></head><body>"
        "<h1>&#127754;&#127797; HydroSertao</h1>"
        "<p class='sub'>Atualiza a cada 3s</p>"
        "<div class='grid'>"
        "<div class='card'><div class='lbl'>Umidade</div>"
        "<div class='val'>%.1f<span class='unit'>%%</span></div>"
        "<div class='bar-bg'><div class='bar' style='width:%d%%;background:%s'></div></div></div>"
        "<div class='card'><div class='lbl'>Temperatura</div>"
        "<div class='val'>%.1f<span class='unit'>C</span></div></div>"
        "<div class='card'><div class='lbl'>Status</div>"
        "<div class='badge' style='background:%s'>%s</div><br>"
        "<div class='badge' style='background:%s'>%s</div><br>"
        "<div class='badge' style='background:%s'>%s</div></div>"
        "<div class='card'><div class='lbl'>Tempo Irrigando</div>"
        "<div class='val'>%lu<span class='unit'>s</span></div></div>"
        "</div>"
        "<div style='text-align:center;margin-top:16px'>"
        "<a class='btn' href='/irrigar/on'>Ligar Irrigacao</a>"
        "<a class='btn red' href='/irrigar/off'>Desligar Irrigacao</a>"
        "</div>"
        "<p class='foot'>HydroSertao v1.0 | Jorgenaldo Silva Moraes | EmbarcaTech 2025</p>"
        "</body></html>\r\n",
        sistema.umidade, upct, cor_umid,
        sistema.temperatura,
        cor_irrig, st_irrig,
        cor_emerg, st_emerg,
        cor_modo,  st_modo,
        sistema.tempo_irrigando
    );
}

static err_t http_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) { tcp_close(tpcb); return ERR_OK; }
    char *req = (char *)p->payload;
    if (strstr(req, "GET /irrigar/on"))  controlar_irrigacao(true);
    if (strstr(req, "GET /irrigar/off")) controlar_irrigacao(false);
    criar_pagina_html();
    tcp_write(tpcb, http_response, strlen(http_response), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    pbuf_free(p);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);
    return ERR_OK;
}

void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) return;
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    log_serial("Servidor HTTP na porta 80.");
}

/* ═══════════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════════ */
int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    log_serial("=== HydroSertao v1.0.0 ===");

    hardware_init();
    ws2812_init();
    display_init();
    wifi_init();
    start_http_server();

    log_serial("Sistema pronto!");
    buzzer_beep(1000, 200);
    set_led(false, true, false);
    matriz_mostrar(padrao_sol, 20, 20, 0);

    uint32_t ultimo_leitura = 0;
    uint32_t contador       = 0;

    while (true) {
        uint32_t agora = to_ms_since_boot(get_absolute_time());

        if (flag_modo_mudou) {
            flag_modo_mudou = false;
            sistema.modo_manual = !sistema.modo_manual;
            buzzer_beep(sistema.modo_manual ? 600 : 1000, 80);
            log_serial(sistema.modo_manual ? "Modo: MANUAL" : "Modo: AUTO");
        }

        if (flag_emergencia) {
            flag_emergencia = false;
            sistema.emergencia = true;
            sistema.irrigando  = false;
            set_led(true, false, false);
            log_serial("EMERGENCIA ativada!");
        }

        if ((agora - ultimo_leitura) >= INTERVALO_MS) {
            ultimo_leitura = agora;

            sistema.umidade     = ler_umidade();
            sistema.temperatura = ler_temperatura();

            if (!sistema.emergencia && !sistema.modo_manual) {
                if (sistema.umidade < UMIDADE_MINIMA && !sistema.irrigando)
                    controlar_irrigacao(true);
                else if (sistema.umidade >= UMIDADE_MAXIMA && sistema.irrigando)
                    controlar_irrigacao(false);
            }

            if (sistema.irrigando) sistema.tempo_irrigando++;

            atualizar_display();
            atualizar_matriz();

            if (contador % 5 == 0) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                    "Umid:%.1f%% Temp:%.1fC Irrig:%s Modo:%s",
                    sistema.umidade, sistema.temperatura,
                    sistema.irrigando ? "SIM" : "NAO",
                    sistema.modo_manual ? "MAN" : "AUTO");
                log_serial(buf);
            }
            contador++;
        }

        if (sistema.emergencia) {
            set_led(true, false, false);
            atualizar_display();
            if (!gpio_get(BTN_A_PIN)) {
                sleep_ms(3000);
                if (!gpio_get(BTN_A_PIN)) {
                    sistema.emergencia = false;
                    log_serial("Emergencia resetada.");
                    set_led(false, true, false);
                }
            }
        }

        cyw43_arch_poll();
        sleep_ms(10);
    }
    return 0;
}

/* ═══════════════════════════════════════════════════════════════════
 * HARDWARE INIT
 * ═══════════════════════════════════════════════════════════════════ */
void hardware_init(void) {
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_set_temp_sensor_enabled(true);

    gpio_init(LED_R_PIN); gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_G_PIN); gpio_set_dir(LED_G_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN); gpio_set_dir(LED_B_PIN, GPIO_OUT);
    set_led(false, false, false);

    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_PIN), false);

    gpio_init(BTN_A_PIN); gpio_set_dir(BTN_A_PIN, GPIO_IN); gpio_pull_up(BTN_A_PIN);
    gpio_init(BTN_B_PIN); gpio_set_dir(BTN_B_PIN, GPIO_IN); gpio_pull_up(BTN_B_PIN);

    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &callback_emergencia);
    gpio_set_irq_enabled(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true);
    irq_add_shared_handler(IO_IRQ_BANK0, callback_modo, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uart_init(UART_PORT, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    log_serial("Hardware inicializado.");
}

/* ═══════════════════════════════════════════════════════════════════
 * WS2812 INIT
 * ═══════════════════════════════════════════════════════════════════ */
void ws2812_init(void) {
    ws_offset = pio_add_program(ws_pio, &ws2812_program);
    ws_sm     = pio_claim_unused_sm(ws_pio, true);
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_out_pins(&c, WS2812_PIN, 1);
    sm_config_set_set_pins(&c, WS2812_PIN, 1);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (800000.0f * 10.0f));
    pio_gpio_init(ws_pio, WS2812_PIN);
    pio_sm_set_consecutive_pindirs(ws_pio, ws_sm, WS2812_PIN, 1, true);
    pio_sm_init(ws_pio, ws_sm, ws_offset, &c);
    pio_sm_set_enabled(ws_pio, ws_sm, true);
    matriz_apagar();
    log_serial("Matriz WS2812 iniciada.");
}

/* ═══════════════════════════════════════════════════════════════════
 * DISPLAY INIT
 * ═══════════════════════════════════════════════════════════════════ */
void display_init(void) {
    ssd1306_init();
    ssd1306_clear();
    ssd1306_draw_string(16, 28, "HydroSertao");
    ssd1306_show();
    sleep_ms(1500);
    log_serial("Display OLED iniciado.");
}

/* ═══════════════════════════════════════════════════════════════════
 * WI-FI INIT — mostra IP no display
 * ═══════════════════════════════════════════════════════════════════ */
void wifi_init(void) {
    if (cyw43_arch_init()) { log_serial("ERRO: Wi-Fi!"); return; }
    cyw43_arch_enable_sta_mode();
    log_serial("Conectando Wi-Fi...");

    ssd1306_clear();
    ssd1306_draw_string(0, 28, "Conectando WiFi...");
    ssd1306_show();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        log_serial("Wi-Fi FALHOU.");
        ssd1306_clear();
        ssd1306_draw_string(0, 28, "WiFi FALHOU!");
        ssd1306_show();
        return;
    }

    uint8_t *ip = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[20];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);

    char msg[48];
    snprintf(msg, sizeof(msg), "Wi-Fi OK! IP: %s", ip_str);
    log_serial(msg);

    ssd1306_clear();
    ssd1306_draw_string(0,  0, "WiFi conectado!");
    ssd1306_draw_string(0, 12, "Acesse:");
    ssd1306_draw_string(0, 24, ip_str);
    ssd1306_draw_string(0, 36, "no navegador");
    ssd1306_show();
    sleep_ms(4000);
}

/* ═══════════════════════════════════════════════════════════════════
 * SENSORES
 * ═══════════════════════════════════════════════════════════════════ */
float ler_umidade(void) {
    adc_select_input(JOYSTICK_Y_ADC);
    uint32_t soma = 0;
    for (int i = 0; i < 8; i++) { soma += adc_read(); sleep_us(100); }
    float u = ((float)(soma / 8) / 4095.0f) * 100.0f;
    if (u < 0.0f) u = 0.0f;
    if (u > 100.0f) u = 100.0f;
    return u;
}

float ler_temperatura(void) {
    adc_select_input(TEMP_ADC);
    uint16_t raw = adc_read();
    return 27.0f - (raw * 3.3f / 4095.0f - 0.706f) / 0.001721f;
}

/* ═══════════════════════════════════════════════════════════════════
 * ATUADORES
 * ═══════════════════════════════════════════════════════════════════ */
void controlar_irrigacao(bool ligar) {
    sistema.irrigando = ligar;
    set_led(false, !ligar, ligar);
    if (ligar) { buzzer_beep(800, 150); log_serial("Irrigacao: LIGADA"); }
    else       { log_serial("Irrigacao: DESLIGADA"); }
}

void set_led(bool r, bool g, bool b) {
    gpio_put(LED_R_PIN, r);
    gpio_put(LED_G_PIN, g);
    gpio_put(LED_B_PIN, b);
}

void buzzer_beep(uint32_t freq_hz, uint32_t duracao_ms) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint chan  = pwm_gpio_to_channel(BUZZER_PIN);
    uint32_t div = 125000000 / (freq_hz * 4096);
    if (div < 1) div = 1;
    pwm_set_clkdiv(slice, (float)div);
    pwm_set_wrap(slice, 4095);
    pwm_set_chan_level(slice, chan, 2048);
    pwm_set_enabled(slice, true);
    sleep_ms(duracao_ms);
    pwm_set_enabled(slice, false);
    gpio_put(BUZZER_PIN, 0);
}

/* ═══════════════════════════════════════════════════════════════════
 * DISPLAY — layout do arquivo que funcionou bem
 * ═══════════════════════════════════════════════════════════════════ */
void atualizar_display(void) {
    ssd1306_clear();
    char buf[22];

    ssd1306_draw_string(16, 0, "** HydroSertao **");
    for (int x = 0; x < 128; x++) ssd1306_draw_pixel(x, 9, true);

    snprintf(buf, sizeof(buf), "Umid : %5.1f %%", sistema.umidade);
    ssd1306_draw_string(0, 12, buf);

    snprintf(buf, sizeof(buf), "Temp : %5.1f C", sistema.temperatura);
    ssd1306_draw_string(0, 22, buf);

    if (sistema.emergencia)
        ssd1306_draw_string(0, 32, "Irrig: !EMERG!");
    else {
        snprintf(buf, sizeof(buf), "Irrig: %s", sistema.irrigando ? "LIGADA " : "DESLIG.");
        ssd1306_draw_string(0, 32, buf);
    }

    snprintf(buf, sizeof(buf), "Modo : %s", sistema.modo_manual ? "MANUAL  " : "AUTO    ");
    ssd1306_draw_string(0, 42, buf);

    snprintf(buf, sizeof(buf), "TIrr : %lu s", sistema.tempo_irrigando);
    ssd1306_draw_string(0, 52, buf);

    ssd1306_show();
}

/* ═══════════════════════════════════════════════════════════════════
 * MATRIZ
 * ═══════════════════════════════════════════════════════════════════ */
void atualizar_matriz(void) {
    if (sistema.emergencia) {
        static bool pisc = false;
        pisc = !pisc;
        if (pisc) matriz_mostrar(padrao_x, 30, 0, 0);
        else      matriz_apagar();
    } else if (sistema.modo_manual) {
        matriz_mostrar(padrao_manual, 20, 20, 0);
    } else if (sistema.irrigando) {
        matriz_mostrar(padrao_gota, 0, 0, 30);
    } else {
        uint8_t b = (uint8_t)(sistema.umidade / 100.0f * 25.0f);
        if (b < 5) b = 5;
        matriz_mostrar(padrao_sol, b, b, 0);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * LOG SERIAL
 * ═══════════════════════════════════════════════════════════════════ */
void log_serial(const char *msg) {
    uart_puts(UART_PORT, "[HydroSertao] ");
    uart_puts(UART_PORT, msg);
    uart_puts(UART_PORT, "\r\n");
}

/* ═══════════════════════════════════════════════════════════════════
 * CALLBACKS — só flags, zero bloqueio
 * ═══════════════════════════════════════════════════════════════════ */
void callback_emergencia(uint gpio, uint32_t events) {
    if (gpio == BTN_A_PIN && (events & GPIO_IRQ_EDGE_FALL))
        flag_emergencia = true;
}

void callback_modo(void) {
    uint32_t status = gpio_get_irq_event_mask(BTN_B_PIN);
    if (status & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(BTN_B_PIN, GPIO_IRQ_EDGE_FALL);
        flag_modo_mudou = true;
    }
}