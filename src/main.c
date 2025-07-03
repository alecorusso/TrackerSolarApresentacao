#include "rtc_e_sol.h"

// Código feito por:
// Philip William -- Código solarimétrico e do RTC
// Anita Cunha, Yara Rodrigues e Alexander Oliveira -- Código dos servomotores

// PASSOS LÓGICOS PARA O TRACKER FUNCIONAR
// 1. Inicializar os hardwares relevantes
// 2. Puxar o valor do RTC a partir do rtc.c
// 3. Calcular o ângulo horário solar e ângulo zenital a partir do solarimetria.c
// 4. Girar o motor no ângulo adequado a partir do servo.c
// 5. Esperar 5 minutos para repetir o prodecimento do passo 2-4
// 6. Quando o ângulo zenital passar do horizonte (ou do ângulo de ataque da placa),
// antecipar a posição do dia seguinte e testar em intervalos de 30 minutos até que o sol nasça novamente.

// Valores de entrada
const data_t calibracao = {0, 0, 15, 28, 6, 2025};  // 28/06/2025 15:00:00
const float latitude_local_graus = -23.55;  // São Paulo (negativo para Sul)
const float longitude_local_graus = -46.63;  // São Paulo (negativo para Oeste)
const float longitude_meridiano_padrao_graus = -45.0;  // Para UTC-3 (BRT)
struct rtc_time tm;

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <pwm_z401.h>
#include "rtc_e_sol.h"
//#define CONFIG_SET_RTC_TIME  // comente o define para não configurar o RTC

#define SERVO_PERIODO_TPM_MODULO 7500
#define SERVO_GPIO_PORTA GPIOE
#define SERVO_GPIO_PINO  20

//ADC
#define ADC_RESOLUTION      12
#define ADC_GAIN            ADC_GAIN_1
#define ADC_REFERENCE       ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID      0  //Canal do ADC, veja a pinagem e nomes em \.platformio\packages\framework-zephyr\_pio\modules\hal\nxp\dts\nxp\kinetis\MKL25Z128VLK4-pinctrl.h
#define ADC_VREF_MV         3300

//GPIO
#define LED_NODE DT_ALIAS(led1)

#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);


//PWM
#define TPM_MODULE 1000         // Define a frequência do PWM fpwm = (TPM_CLK / (TPM_MODULE * PS))
uint16_t duty  = 950;  

//RTC


#define DS1307_NODE DT_ALIAS(ds1307)

#if !DT_NODE_HAS_STATUS(DS1307_NODE, okay)
#error "DS1307 RTC device not found in device tree"
#endif

const struct device *rtc = DEVICE_DT_GET(DS1307_NODE);

void definir_servo_angulo(float angulo){
    uint16_t valor_cnv = 0;
    uint16_t cnv_min = 135;
    uint16_t cnv_max = 890;

    if (angulo < 0.0f) angulo = 0.0f;
    if (angulo > 180.0f) angulo = 180.0f;

    valor_cnv = (uint16_t)(cnv_min + (angulo / 180.0f)*(cnv_max - cnv_min));
    
    pwm_tpm_CnV(TPM1, 0, valor_cnv);
}

int main(void){
    // 1.
    pwm_tpm_Init(TPM1, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM1, 0, TPM_PWM_H, GPIOE, 20);
    printk("Sistema do tracker solar inicializado.\n");

    uint16_t tick = 135;
    pwm_tpm_CnV(TPM1,0,tick);
    k_msleep(5000);
    for(tick = 135; tick < 890; tick += 5){
        pwm_tpm_CnV(TPM1,0,tick);
        k_msleep(20);
    }
    for(tick = 890; tick > 135; tick -= 5){
        pwm_tpm_CnV(TPM1,0,tick);
        k_msleep(20);
    }

    while(1){
        if (rtc_get_time(rtc, &tm) == 0) {
            printk("Hora: %04d-%02d-%02d %02d:%02d:%02d\n",
                   tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                   tm.tm_hour, tm.tm_min, tm.tm_sec);
        } else {
            printk("Erro ao ler a hora do RTC\n");
        }

        // 3.
        float omega_atual_graus = angulo_horario_solar();
        float zenital_atual_graus = angulo_zenital();
        if(omega_atual_graus < -900.0f || zenital_atual_graus < -900.0f){
            printk("Erro ao calcular o ângulo omega e zenital.\n");
            k_msleep(60000);
            continue;
        }
        printk("Calculado: Omega = %d graus, Zenital = %d graus.\n", (int)(omega_atual_graus*100), (int)(zenital_atual_graus*100));

        // 4.
        float angulo_servo_alvo = 90.0f;
        if(zenital_atual_graus < 90.0f){
            angulo_servo_alvo = omega_atual_graus + 90.0f;
            if(angulo_servo_alvo < 0.0f) angulo_servo_alvo = 0;
            if(angulo_servo_alvo > 180.0f) angulo_servo_alvo = 180.0f;

            definir_servo_angulo(angulo_servo_alvo);
            printk("Servo: %d graus (Sol acima do horizonte)\n", (int)angulo_servo_alvo);
            k_msleep(1000);
        }
    }
    return 0;
}

