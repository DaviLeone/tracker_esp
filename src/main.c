#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdint.h>

/* ---------------- CONFIGURACOES ---------------- */

#define LED_PIN             2

#define MPU6050_ADDR        0x69  /* AD0 conectado em 3.3V */
#define MPU6050_REG_PWR     0x6B
#define MPU6050_REG_ACCEL   0x3B
#define MPU6050_REG_GYRO    0x43

#define ACCEL_THRESHOLD     10.0f  /* m/s² */
#define AVG_THRESHOLD      5.0f    /* m/s² */

#define ACCEL_ALERT_TIME   10      /* segundos */
#define AVG_ALERT_TIME      5      /* segundos */

#define SAMPLE_INTERVAL_MS 100     /* amostragem a cada 100 ms */

/* ---------------- DISPOSITIVOS ---------------- */

static const struct device *gpio_dev =
    DEVICE_DT_GET(DT_NODELABEL(gpio0));

static const struct device *rtc_dev =
    DEVICE_DT_GET(DT_NODELABEL(ds3231_rtc));

static const struct device *i2c_dev =
    DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* ---------------- VARIAVEIS ---------------- */

static int last_sec = -1;

static float accel_sum = 0.0f;
static uint32_t accel_samples = 0;

static int64_t accel_alert_until = 0;
static int64_t avg_alert_until = 0;

static int64_t last_avg_window = -1;

/* ---------------- MPU6050 ---------------- */

/* Escreve em um registrador do MPU6050 */
static int mpu_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_write(i2c_dev, data, sizeof(data), MPU6050_ADDR);
}

/* Leitura de registradores */
static int mpu_read_registers(uint8_t reg,
                              uint8_t *data,
                              uint8_t length)
{
    return i2c_write_read(i2c_dev,
                         MPU6050_ADDR,
                         &reg,
                         1,
                         data,
                         length);
}

/* Inicializacao do sensor */
static int mpu_init(void)
{
    int ret;

    /* Retira o MPU6050 do modo sleep */
    ret = mpu_write(MPU6050_REG_PWR, 0x00);

    if (ret < 0) {
        printk("Erro ao despertar MPU6050: %d\n", ret);
        return ret;
    }

    /* Acelerometro: escala +-2g */
    ret = mpu_write(0x1C, 0x00);

    if (ret < 0) {
        printk("Erro configurando acelerometro\n");
        return ret;
    }

    /* Giroscopio: escala +-250 graus/s */
    ret = mpu_write(0x1B, 0x00);

    if (ret < 0) {
        printk("Erro configurando giroscopio\n");
        return ret;
    }

    printk("MPU6050 inicializado!\n");

    return 0;
}

/* Le aceleracao e giroscopio */
static int mpu_read_data(float *ax, float *ay, float *az,
                         float *gx, float *gy, float *gz)
{
    uint8_t data[14];

    int ret = mpu_read_registers(MPU6050_REG_ACCEL,
                                  data,
                                  sizeof(data));

    if (ret < 0) {
        return ret;
    }

    /* Valores brutos de 16 bits */
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);

    int16_t raw_gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t raw_gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);

    /*
     * Acelerometro em +-2g:
     * 16384 LSB = 1g
     */
    *ax = (raw_ax / 16384.0f) * 9.80665f;
    *ay = (raw_ay / 16384.0f) * 9.80665f;
    *az = (raw_az / 16384.0f) * 9.80665f;

    /*
     * Giroscopio em +-250 graus/s:
     * 131 LSB = 1 grau/s
     */
    *gx = raw_gx / 131.0f;
    *gy = raw_gy / 131.0f;
    *gz = raw_gz / 131.0f;

    return 0;
}

/* ---------------- LED ---------------- */

static void update_led(struct rtc_time *tm)
{
    int64_t now = k_uptime_get();

    /*
     * Alertas tem prioridade sobre a logica
     * de piscar do RTC.
     */

    if (now < accel_alert_until) {
        gpio_pin_set(gpio_dev, LED_PIN, 1);
        return;
    }

    if (now < avg_alert_until) {
        gpio_pin_set(gpio_dev, LED_PIN, 1);
        return;
    }

    /*
     * Funcionalidade original:
     * LED pisca durante os primeiros 10 segundos
     * de cada minuto.
     */
    if (tm->tm_sec < 10) {
        gpio_pin_toggle(gpio_dev, LED_PIN);
    } else {
        gpio_pin_set(gpio_dev, LED_PIN, 0);
    }
}

/* ---------------- MAIN ---------------- */

int main(void)
{
    struct rtc_time tm;

    float ax, ay, az;
    float gx, gy, gz;

    float accel_magnitude;

    if (!device_is_ready(gpio_dev) ||
        !device_is_ready(rtc_dev) ||
        !device_is_ready(i2c_dev)) {

        printk("Erro: GPIO, RTC ou I2C nao esta pronto.\n");
        return 0;
    }

    gpio_pin_configure(gpio_dev,
                       LED_PIN,
                       GPIO_OUTPUT_INACTIVE);

    /* Inicializacao do MPU6050 */
    if (mpu_init() < 0) {
        printk("Falha na inicializacao do MPU6050.\n");
        return 0;
    }

    /* ---------------- RTC ---------------- */

    if (rtc_get_time(rtc_dev, &tm) < 0) {

        struct rtc_time init_tm = {
            .tm_year = 2026 - 1900,
            .tm_mon  = 11 - 1,
            .tm_mday = 13,
            .tm_hour = 13,
            .tm_min  = 40,
            .tm_sec  = 0,
        };

        rtc_set_time(rtc_dev, &init_tm);

        printk("Horario inicial gravado no DS3231.\n");
    }

    printk("Sistema iniciado!\n");

    printk("Monitorando acelerometro e giroscopio...\n");

    /* ---------------- LOOP ---------------- */

    while (1) {

        /* Leitura do RTC */
        if (rtc_get_time(rtc_dev, &tm) == 0) {

            /* Imprime horario a cada segundo */
            if (tm.tm_sec != last_sec) {

                last_sec = tm.tm_sec;

                printk("\nRTC: %02d:%02d:%02d\n",
                       tm.tm_hour,
                       tm.tm_min,
                       tm.tm_sec);
            }

            /* Leitura do MPU6050 */
            if (mpu_read_data(&ax, &ay, &az,
                              &gx, &gy, &gz) == 0) {

                /*
                 * Magnitude da aceleracao:
                 * sqrt(ax² + ay² + az²)
                 */
                accel_magnitude =
                    sqrtf(ax * ax +
                          ay * ay +
                          az * az);

                /* Monitor serial */
                printk("ACC X: %.2f m/s2 | "
                       "Y: %.2f | Z: %.2f | "
                       "Magnitude: %.2f m/s2\n",
                       ax, ay, az,
                       accel_magnitude);

                printk("GYRO X: %.2f deg/s | "
                       "Y: %.2f | Z: %.2f\n",
                       gx, gy, gz);

                /* ---------------- ALERTA 10s ---------------- */

                if (accel_magnitude > ACCEL_THRESHOLD) {

                    accel_alert_until =
                        k_uptime_get() +
                        (ACCEL_ALERT_TIME * 1000);

                    printk("!!! ALERTA: aceleracao "
                           "maior que 10 m/s2 !!!\n");
                }

                /* ---------------- MEDIA ---------------- */

                accel_sum += accel_magnitude;
                accel_samples++;

                /*
                 * Verifica a cada 30 segundos
                 * usando o RTC.
                 */
                if ((tm.tm_sec % 30 == 0) &&
                    (tm.tm_sec != last_avg_window)) {

                    last_avg_window = tm.tm_sec;

                    if (accel_samples > 0) {

                        float accel_avg =
                            accel_sum / accel_samples;

                        printk("\n===== MEDIA 30s =====\n");

                        printk("Media da aceleracao: %.2f m/s2\n",
                               accel_avg);

                        printk("Quantidade de amostras: %u\n",
                               accel_samples);

                        /* Reinicia acumuladores */
                        accel_sum = 0.0f;
                        accel_samples = 0;

                        /* ---------------- ALERTA 5s ---------------- */

                        if (accel_avg > AVG_THRESHOLD) {

                            avg_alert_until =
                                k_uptime_get() +
                                (AVG_ALERT_TIME * 1000);

                            printk("!!! ALERTA: media maior "
                                   "que 5 m/s2 !!!\n");

                            printk("LED ligado por 5 segundos.\n");
                        }
                    }
                }
            } else {
                printk("Erro na leitura do MPU6050!\n");
            }

            /* Atualiza LED */
            update_led(&tm);
        }

        k_msleep(SAMPLE_INTERVAL_MS);
    }

    return 0;
}