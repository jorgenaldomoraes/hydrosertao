/**
 * =====================================================================
 * HydroSertão - Sistema Inteligente de Irrigação IoT
 * =====================================================================
 * Plataforma : BitDogLab (RP2040)
 * Autor      : Jorgenaldo Silva Moraes
 * Versão     : 1.1.0
 *
 * Servidor HTTP embutido na porta 80
 * Acesse: http://IP_DA_PLACA no navegador da mesma rede Wi-Fi
 *
 * Buzzer de emergência: alterna ligado/desligado a cada 500 ms
 * sem uso de sleep_ms(), mantendo Wi-Fi e display ativos.
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
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "ssd1306.h"

/* ─── Wi-Fi ──────────────────────────────────────────────────────── */
#define WIFI_SSID           "NOME_DA_REDE"
#define WIFI_PASSWORD       "SENHA_AQUI"

/* ─── GPIOs ──────────────────────────────────────────────────────── */
#define LED_R_PIN           13
#define LED_G_PIN           11
#define LED_B_PIN           12
#define BUZZER_PIN          21
#define BTN_A_PIN           5
#define BTN_B_PIN           6
#define JOYSTICK_Y_ADC      1
#define TEMP_ADC            4

/* ─── I2C ────────────────────────────────────────────────────────── */
#define I2C_PORT            i2c1
#define I2C_SDA_PIN         14
#define I2C_SCL_PIN         15
#define I2C_FREQ            400000

/* ─── UART ───────────────────────────────────────────────────────── */
#define UART_PORT           uart0
#define UART_BAUD           115200
#define UART_TX_PIN         0
#define UART_RX_PIN         1

/* ─── Limiares e temporização ────────────────────────────────────── */
#define UMIDADE_MINIMA      30.0f   /* % — abaixo: liga irrigacao     */
#define UMIDADE_MAXIMA      70.0f   /* % — acima : desliga irrigacao  */
#define INTERVALO_MS        1000    /* periodo de leitura dos sensores */
#define BUZZER_EMERG_MS     500     /* periodo do alarme de emergencia */
#define RESET_EMERG_MS      2000    /* tempo de pressao para reset     */

/* ─── Estado do sistema ──────────────────────────────────────────── */
typedef struct {
    float    umidade;         /* 0-100 %                        */
    float    temperatura;     /* graus Celsius                  */
    bool     irrigando;       /* irrigacao ativa                */
    bool     emergencia;      /* modo de emergencia ativo       */
    bool     modo_manual;     /* true = manual, false = auto    */
    uint32_t tempo_irrigando; /* segundos totais irrigando      */
} SistemaState;

volatile SistemaState sistema         = {0};
volatile bool         flag_modo_mudou = false;
volatile bool         flag_emergencia = false;

/* ─── Controle do buzzer de emergencia (nao bloqueante) ─────────── */
static bool     buzzer_emerg_on = false;
static uint32_t ultimo_bip      = 0;

/* ─── Prototipos ─────────────────────────────────────────────────── */
void  hardware_init(void);
void  display_init(void);
void  wifi_init(void);
void  start_http_server(void);
float ler_umidade(void);
float ler_temperatura(void);
void  controlar_irrigacao(bool ligar);
void  set_led(bool r, bool g, bool b);
void  buzzer_ligar(uint32_t freq_hz);
void  buzzer_desligar(void);
void  buzzer_beep(uint32_t freq_hz, uint32_t duracao_ms);
void  atualizar_display(void);
void  log_serial(const char *msg);
void  callback_emergencia(uint gpio, uint32_t events);
void  callback_modo(void);

/* ═══════════════════════════════════════════════════════════════════
 * SERVIDOR HTTP EMBUTIDO
 * ═══════════════════════════════════════════════════════════════════ */
static char http_response[3000];

static void montar_html(void) {
    const char *cor_irrig = sistema.irrigando   ? "#2196F3" : "#9e9e9e";
    const char *st_irrig  = sistema.irrigando   ? "LIGADA"  : "DESLIGADA";
    const char *cor_emerg = sistema.emergencia  ? "#f44336" : "#4CAF50";
    const char *st_emerg  = sistema.emergencia  ? "EMERGENCIA" : "NORMAL";
    const char *st_modo   = sistema.modo_manual ? "MANUAL"  : "AUTO";
    const char *cor_modo  = sistema.modo_manual ? "#FF9800" : "#4CAF50";
    int upct = (int)sistema.umidade;
    const char *cor_umid  = upct >= 30 ? "#4CAF50" : "#f44336";

    snprintf(http_response, sizeof(http_response),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=UTF-8\r\n"
        "Connection: close\r\nRefresh: 3\r\n\r\n"
        "<!DOCTYPE html><html><head>"
        "<meta charset=UTF-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>HydroSertao</title><style>"
        "body{font-family:Arial,sans-serif;background:#f0f4f8;padding:12px;margin:0}"
        ".g{display:flex;flex-wrap:wrap;gap:10px;justify-content:center;margin:10px 0}"
        ".c{background:#fff;border-radius:12px;padding:14px;min-width:140px;text-align:center;"
        "box-shadow:0 2px 6px rgba(0,0,0,.1)}"
        ".l{font-size:10px;color:#999;text-transform:uppercase;margin-bottom:4px}"
        ".v{font-size:30px;font-weight:bold;color:#1a5276}"
        ".u{font-size:13px;color:#888}"
        ".b{padding:6px 12px;border-radius:16px;color:#fff;font-weight:bold;"
        "font-size:12px;display:inline-block;margin-top:5px}"
        ".bg{background:#e0e0e0;border-radius:5px;height:10px;margin-top:6px;overflow:hidden}"
        ".br{height:100%%;border-radius:5px}"
        "a.btn{display:inline-block;margin:4px;padding:7px 14px;border-radius:8px;"
        "background:#1a5276;color:#fff;text-decoration:none;font-size:12px}"
        "a.r{background:#c0392b}"
        ".f{text-align:center;color:#bbb;font-size:10px;margin-top:12px}"
        "</style></head><body>"
        "<div style='text-align:center;padding:6px 0'>"
        "<svg width='200' height='42' viewBox='0 0 200 42'>"
        "<path d='M0 32Q10 20 20 32Q30 44 40 32Q50 20 60 32'"
        " stroke='#2196F3' stroke-width='3' fill='none'/>"
        "<line x1='74' y1='40' x2='74' y2='12' stroke='#2e7d32' stroke-width='4'/>"
        "<line x1='63' y1='25' x2='74' y2='25' stroke='#2e7d32' stroke-width='3'/>"
        "<line x1='63' y1='16' x2='63' y2='25' stroke='#2e7d32' stroke-width='3'/>"
        "<line x1='85' y1='21' x2='74' y2='21' stroke='#2e7d32' stroke-width='3'/>"
        "<line x1='85' y1='12' x2='85' y2='21' stroke='#2e7d32' stroke-width='3'/>"
        "<text x='94' y='32' font-family='Arial' font-size='24'"
        " font-weight='bold' fill='#1a5276'>Hydro</text>"
        "<text x='148' y='32' font-family='Arial' font-size='24'"
        " font-weight='bold' fill='#2e7d32'>Sertao</text>"
        "</svg></div>"
        "<p style='text-align:center;color:#888;font-size:11px;margin:0 0 10px'>"
        "Dashboard IoT | Atualiza a cada 3s</p>"
        "<div class='g'>"
        "<div class='c'><div class='l'>Umidade</div>"
        "<div class='v'>%.1f<span class='u'>%%</span></div>"
        "<div class='bg'><div class='br' style='width:%d%%;background:%s'></div></div></div>"
        "<div class='c'><div class='l'>Temperatura</div>"
        "<div class='v'>%.1f<span class='u'>C</span></div></div>"
        "<div class='c'><div class='l'>Status</div>"
        "<div class='b' style='background:%s'>%s</div><br>"
        "<div class='b' style='background:%s'>%s</div><br>"
        "<div class='b' style='background:%s'>%s</div></div>"
        "<div class='c'><div class='l'>T.Irrigando</div>"
        "<div class='v'>%lu<span class='u'>s</span></div></div>"
        "</div>"
        "<div style='text-align:center'>"
        "<a class='btn' href='/irrigar/on'>Ligar Irrigacao</a>"
        "<a class='btn r' href='/irrigar/off'>Desligar Irrigacao</a>"
        "</div>"
        "<p class='f'>HydroSertao v1.1 | Jorgenaldo Silva Moraes | EmbarcaTech 2025</p>"
        "</body></html>",
        sistema.umidade, upct, cor_umid,
        sistema.temperatura,
        cor_irrig, st_irrig,
        cor_emerg, st_emerg,
        cor_modo,  st_modo,
        sistema.tempo_irrigando
    );
}

static err_t http_sent_cb(void *arg, struct tcp_pcb *tpcb, uint16_t len) {
    tcp_close(tpcb);
    return ERR_OK;
}

static err_t http_callback(void *arg, struct tcp_pcb *tpcb,
                            struct pbuf *p, err_t err) {
    if (p == NULL) { tcp_close(tpcb); return ERR_OK; }
    char *req = (char *)p->payload;
    if (strstr(req, "GET /irrigar/on"))  controlar_irrigacao(true);
    if (strstr(req, "GET /irrigar/off")) controlar_irrigacao(false);
    pbuf_free(p);
    montar_html();
    uint16_t len = (uint16_t)strlen(http_response);
    tcp_sent(tpcb, http_sent_cb);
    err_t we = tcp_write(tpcb, http_response, len, TCP_WRITE_FLAG_COPY);
    if (we == ERR_OK) {
        tcp_output(tpcb);
    } else {
        log_serial("ERRO: tcp_write HTTP.");
        tcp_close(tpcb);
    }
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_callback);
    return ERR_OK;
}

void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) { log_serial("ERRO: PCB HTTP."); return; }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        log_serial("ERRO: Bind porta 80."); return;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    log_serial("Servidor HTTP OK na porta 80.");
}

/* ═══════════════════════════════════════════════════════════════════
 * MAIN
 * ═══════════════════════════════════════════════════════════════════ */
int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    log_serial("=== HydroSertao v1.1.0 ===");

    hardware_init();
    display_init();
    wifi_init();
    start_http_server();

    log_serial("Sistema pronto!");
    buzzer_beep(1000, 200);
    set_led(false, true, false);

    uint32_t ultimo_leitura = 0;
    uint32_t contador       = 0;

    while (true) {
        uint32_t agora = to_ms_since_boot(get_absolute_time());

        /* ── Botao B: troca de modo (flag vinda da IRQ) ── */
        if (flag_modo_mudou) {
            flag_modo_mudou     = false;
            sistema.modo_manual = !sistema.modo_manual;
            buzzer_beep(sistema.modo_manual ? 600 : 1000, 80);
            log_serial(sistema.modo_manual ? "Modo: MANUAL" : "Modo: AUTO");
        }

        /* ── Botao A: ativa emergencia (flag vinda da IRQ) ── */
        if (flag_emergencia) {
            flag_emergencia    = false;
            sistema.emergencia = true;
            sistema.irrigando  = false;
            controlar_irrigacao(false);
            set_led(true, false, false);
            log_serial("EMERGENCIA ativada!");
        }

        /* ── Buzzer intermitente de emergencia (nao bloqueante) ──────
         * A cada BUZZER_EMERG_MS (500 ms) alterna entre buzzer_ligar()
         * e buzzer_desligar() usando apenas timestamp — sem sleep_ms().
         * Isso garante que cyw43_arch_poll() nunca seja bloqueado
         * durante o alarme, mantendo Wi-Fi e display funcionando.
         * Os bips normais (buzzer_beep) continuam disponiveis fora
         * do estado de emergencia para confirmacoes de acao.
         * ─────────────────────────────────────────────────────────── */
        if (sistema.emergencia) {
            if ((agora - ultimo_bip) >= BUZZER_EMERG_MS) {
                ultimo_bip = agora;
                if (buzzer_emerg_on) {
                    buzzer_desligar();
                    buzzer_emerg_on = false;
                } else {
                    buzzer_ligar(500);
                    buzzer_emerg_on = true;
                }
            }
        } else if (buzzer_emerg_on) {
            buzzer_desligar();
            buzzer_emerg_on = false;
        }

        /* ── Leitura periodica dos sensores (1 s) ── */
        if ((agora - ultimo_leitura) >= INTERVALO_MS) {
            ultimo_leitura = agora;

            sistema.umidade     = ler_umidade();
            sistema.temperatura = ler_temperatura();

            /* Controle automatico — inativo em emergencia ou modo manual */
            if (!sistema.emergencia && !sistema.modo_manual) {
                if (sistema.umidade < UMIDADE_MINIMA && !sistema.irrigando) {
                    controlar_irrigacao(true);
                    log_serial("AUTO: Irrigacao LIGADA");
                } else if (sistema.umidade >= UMIDADE_MAXIMA && sistema.irrigando) {
                    controlar_irrigacao(false);
                    log_serial("AUTO: Irrigacao DESLIGADA");
                }
            }

            if (sistema.irrigando) sistema.tempo_irrigando++;

            atualizar_display();

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

        /* ── Reset de emergencia: manter Botao A pressionado 2 s ── */
        if (sistema.emergencia && !gpio_get(BTN_A_PIN)) {
            uint32_t t0 = to_ms_since_boot(get_absolute_time());
            while (!gpio_get(BTN_A_PIN)) {
                cyw43_arch_poll();
                sleep_ms(10);
                if ((to_ms_since_boot(get_absolute_time()) - t0) >= RESET_EMERG_MS) {
                    sistema.emergencia = false;
                    buzzer_desligar();
                    buzzer_emerg_on = false;
                    set_led(false, true, false);
                    log_serial("Emergencia resetada.");
                    break;
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

    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL,
        true, &callback_emergencia);
    gpio_set_irq_enabled(BTN_B_PIN, GPIO_IRQ_EDGE_FALL, true);
    irq_add_shared_handler(IO_IRQ_BANK0, callback_modo,
        PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

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
 * DISPLAY INIT
 * ═══════════════════════════════════════════════════════════════════ */
void display_init(void) {
    ssd1306_init();
    ssd1306_clear();

    int onda[] = {
        8,7,6,5,5,6,7,8,9,10,10,9, 8,7,6,5,5,6,7,8,9,10,10,9,
        8,7,6,5,5,6,7,8,9,10,10,9, 8,7,6,5,5,6,7,8,9,10,10,9,
        8,7,6,5,5,6,7,8,9,10,10,9, 8,7,6,5,5,6,7,8,9,10,10,9,
        8,7,6,5,5,6,7,8,9,10,10,9, 8,7,6,5,5,6,7,8,9,10,10,9,
        8,7,6,5,5,6,7,8,9,10,10,9, 8,7,6,5,5,6,7,8,9,10,10,9,
        8,7,6,5,5,6
    };
    for (int x = 0; x < 126; x++) {
        ssd1306_draw_pixel(x, onda[x],     true);
        ssd1306_draw_pixel(x, onda[x] + 1, true);
    }

    for (int y = 2; y < 13; y++)     ssd1306_draw_pixel(122, y, true);
    for (int x = 118; x <= 122; x++) ssd1306_draw_pixel(x, 6, true);
    for (int y = 4;   y <= 6;   y++) ssd1306_draw_pixel(118, y, true);
    for (int x = 122; x <= 126; x++) ssd1306_draw_pixel(x, 8, true);
    for (int y = 6;   y <= 8;   y++) ssd1306_draw_pixel(126, y, true);

    ssd1306_draw_string(12, 16, "** HydroSertao **");
    ssd1306_draw_string(6,  28, "Irrigacao Inteligente");
    ssd1306_draw_string(16, 40, "EmbarcaTech 2025");
    ssd1306_draw_string(10, 52, "Jorgenaldo Moraes");

    ssd1306_show();
    sleep_ms(3000);
    log_serial("Display OLED iniciado.");
}

/* ═══════════════════════════════════════════════════════════════════
 * WI-FI INIT
 * ═══════════════════════════════════════════════════════════════════ */
void wifi_init(void) {
    if (cyw43_arch_init()) { log_serial("ERRO: Wi-Fi!"); return; }
    cyw43_arch_enable_sta_mode();
    log_serial("Conectando Wi-Fi...");

    ssd1306_clear();
    ssd1306_draw_string(0, 24, "Conectando WiFi...");
    ssd1306_show();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK, 15000)) {
        log_serial("Wi-Fi FALHOU.");
        ssd1306_clear();
        ssd1306_draw_string(0, 28, "WiFi FALHOU!");
        ssd1306_show();
        return;
    }

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[20];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             ip[0], ip[1], ip[2], ip[3]);

    char msg[48];
    snprintf(msg, sizeof(msg), "Wi-Fi OK! IP: %s", ip_str);
    log_serial(msg);

    ssd1306_clear();
    ssd1306_draw_string(0,  0, "WiFi conectado!");
    ssd1306_draw_string(0, 12, "Acesse no navegador:");
    ssd1306_draw_string(0, 26, ip_str);
    ssd1306_draw_string(0, 40, "porta 80");
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
    if (u < 0.0f)   u = 0.0f;
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
    if (ligar) {
        buzzer_beep(800, 150);
        log_serial("Irrigacao: LIGADA");
    } else {
        log_serial("Irrigacao: DESLIGADA");
    }
}

void set_led(bool r, bool g, bool b) {
    gpio_put(LED_R_PIN, r);
    gpio_put(LED_G_PIN, g);
    gpio_put(LED_B_PIN, b);
}

/* Liga o buzzer via PWM de forma continua (sem bloqueio) */
void buzzer_ligar(uint32_t freq_hz) {
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint chan  = pwm_gpio_to_channel(BUZZER_PIN);
    uint32_t div = 125000000 / (freq_hz * 4096);
    if (div < 1) div = 1;
    pwm_set_clkdiv(slice, (float)div);
    pwm_set_wrap(slice, 4095);
    pwm_set_chan_level(slice, chan, 2048);
    pwm_set_enabled(slice, true);
}

/* Desliga o buzzer imediatamente */
void buzzer_desligar(void) {
    pwm_set_enabled(pwm_gpio_to_slice_num(BUZZER_PIN), false);
    gpio_put(BUZZER_PIN, 0);
}

/* Bip bloqueante — confirmacoes pontuais fora da emergencia */
void buzzer_beep(uint32_t freq_hz, uint32_t duracao_ms) {
    buzzer_ligar(freq_hz);
    sleep_ms(duracao_ms);
    buzzer_desligar();
}

/* ═══════════════════════════════════════════════════════════════════
 * DISPLAY OLED — atualizacao periodica
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
        ssd1306_draw_string(0, 32, "!!! EMERGENCIA !!!");
    else {
        snprintf(buf, sizeof(buf), "Irrig: %s",
                 sistema.irrigando ? "LIGADA " : "DESLIG.");
        ssd1306_draw_string(0, 32, buf);
    }

    snprintf(buf, sizeof(buf), "Modo : %s",
             sistema.modo_manual ? "MANUAL  " : "AUTO    ");
    ssd1306_draw_string(0, 42, buf);

    snprintf(buf, sizeof(buf), "TIrr : %lu s", sistema.tempo_irrigando);
    ssd1306_draw_string(0, 52, buf);

    ssd1306_show();
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
 * CALLBACKS — apenas sinalizam flags; zero processamento em IRQ
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