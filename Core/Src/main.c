/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include "Nextion.h"

#include "time.h"

#include "stdbool.h"

#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


char input3_str[50];
int isg = 0;

uint32_t flash_adres = 0x0801F800;

uint32_t zaman = 10;

uint8_t rx_data[1];

uint32_t start_time = 0;

bool timing_active = false;

bool is_waiting = false;

uint8_t rx_indx = 0;

uint8_t rx_data[1];

uint8_t rx_buffer[100];

uint8_t transfer_cplt;

Nextion nextion;

NexComp t0;

NexComp t1;

NexComp t4;

NexComp t5;

NexComp t6;

NexComp t7;


NexComp t9;

NexComp t10;

NexComp t11;

NexComp t12;


NexComp t3;

NexComp t2;

NexComp t21;

NexComp t22;

NexComp t23;

NexComp t24;

NexComp t25;

NexComp t26;

NexComp t27;

NexComp t28;

NexComp t31;

NexComp t32;

NexComp t33;

NexComp t34;

NexComp t35;

NexComp t36;


#define PHASE1_SIGNAL 1

#define PHASE2_SIGNAL 1

#define PHASE3_SIGNAL 1

#define RST_SEQUENCE 1

#define TSR_SEQUENCE 2

#define NO_SEQUENCE 0

#define seqFault 5

#define SYSTEM_FREQUENCY 100

#define PERIOD_MS (1000 / SYSTEM_FREQUENCY)

#define MAX_MEASUREMENTS 100

#define MEASURE_TIME_MS 5000

#define SMOOTHING_FACTOR 32

#define FLASH_USER_START_ADDR  0x0801F800  // Flash'ın son sayfasının başlangıç adresi (STM32F103 için)


#define FLASH_USER_START_ADDR_UGN  0x0801F800  // UGNxDR için başlangıç adresi
#define FLASH_USER_START_ADDR_ZAMAN (FLASH_USER_START_ADDR_UGN + sizeof(FlashData))  // Zaman için başlangıç adresi



volatile bool manue1_active = false;
volatile uint32_t manue1_start_time = 0;

volatile uint8_t buzzer_active = 0; // Buzzer durumu için flag
volatile uint32_t buzzer_counter = 0; // Zaman sayacı

unsigned long t1_start, t1_end, t2_start, t2_end;

volatile uint32_t timestamp_phase1 = 0;

volatile uint32_t timestamp_phase2 = 0;

volatile uint32_t timestamp_phase3 = 0;

volatile uint32_t timestamp_phase1_zero_cross = 0;

volatile uint32_t timestamp_phase2_zero_cross = 0;

volatile uint32_t timestamp_phase3_zero_cross = 0;

volatile uint8_t phase_sequence = 0;

volatile uint32_t pulse_count_pa8 = 0;

volatile uint32_t pulse_count_pa9 = 0;

volatile uint32_t pulse_count_pa10 = 0;

volatile uint32_t time_elapsed = 0;

volatile uint32_t average_frequency_pa8 = 0;

volatile uint32_t average_frequency_pa9 = 0;

volatile uint32_t average_frequency_pa10 = 0;

volatile uint32_t input1 = 0;

volatile uint32_t smoothed_result = 0;

volatile uint32_t input3 = 0;

volatile uint32_t pABangle = 0;

volatile uint32_t pACangle = 0;

volatile uint32_t calc_value = 0;

volatile uint32_t t1_s = 0;

volatile uint32_t t1_e = 0;

volatile uint32_t t2_s = 0;

volatile uint32_t t2_e = 0;


volatile uint8_t timerFlag = 0;
volatile bool reset_countdown = false; // Sıfırlama bayrağı

bool is_counting = false;  // Geri sayım durumunu takip eden bayrak
bool is_counting2 = true;
int ebenin = 0;
static int stopped_time = 0;             // Durdurulan zamanı kaydetmek için
static int drmsym = 0;                   // Geri sayım durumunu tutan değişken
uint8_t input1_status = 0;
uint8_t input2_status = 0;
uint8_t input3_status = 0;
uint8_t input4_status = 0;
uint8_t input5_status = 0;

uint8_t UGN1DR ;
uint8_t UGN2DR ;
uint8_t UGN3DR ;
uint8_t UGN4DR ;
uint8_t UGN5DR=1 ;

typedef enum {
    SEQ_IDLE,
    SEQ_VISIBLE,
    SEQ_WAIT,
    SEQ_INVISIBLE
} SeqState;

typedef struct {
    const char *prefix;
    int startIndex;
    int endIndex;
    uint32_t delayMs;
    uint32_t lastTick;
    int currentIndex;
    SeqState state;
} SequentialDisplay;
uint8_t rx_data[1];

uint8_t rx_data_buffer[1]; // Renamed to avoid confusion
uint8_t previous_pin_state = GPIO_PIN_RESET;
uint8_t previous_pin_state3 = GPIO_PIN_RESET; // UGN_3_Pin için önceki durum
uint8_t previous_pin_state_switch = GPIO_PIN_RESET; // SWITCH pini için önceki durum



typedef struct {
    uint32_t zamann;       // 4 bytes
    uint8_t UGN1DR;       // 1 byte
    uint8_t UGN2DR;       // 1 byte
    uint8_t UGN3DR;       // 1 byte
    uint8_t UGN4DR;       // 1 byte
    uint8_t UGN5DR;       // 1 byte
    uint8_t padding[3];   // 3 bytes padding for alignment (optional)
} FlashData;

#define FLASH_USER_START_ADDR  0x0801F800  // Starting address for FlashData
FlashData flashData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SequentialDisplay seqDisplay; // Sıralı Görünürlük Yapılandırması

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Flash belleğe veri yazma fonksiyonu


void check_and_control_pins() {
    // Only proceed if ebenin_adindaki_degisken is 1
    if (ebenin == 1) {
        // Read the states of the pins
        GPIO_PinState ugn_1_state = HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin);
        GPIO_PinState ugn_3_state = HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin);
        GPIO_PinState ugn_4_state = HAL_GPIO_ReadPin(GPIOB, UGN_4_Pin);
        GPIO_PinState emergency_state = HAL_GPIO_ReadPin(GPIOB, EMERGENY_Pin);

        // Check if any of the pins is set
        if (ugn_1_state == GPIO_PIN_RESET ||
            ugn_3_state == GPIO_PIN_RESET ||
            ugn_4_state == GPIO_PIN_RESET ||
            emergency_state == GPIO_PIN_RESET) {

            // Call the control function if any pin is set
            control_pins_based_on_input5();  // Ä°kinci fonksiyonu Ã§aÄŸÄ±r
            //Buzzer_Control();
            is_waiting = false;  // Bekleme durumu iptal
            geri_sayim_dur(); // Geri sayımı durdur();
        }
    }
}

void Delay_ms(uint32_t ms)
{
  for (uint32_t i = 0; i < ms * 1000; i++)  // 1 ms için yaklaşık 1000 döngü
  {
    __asm("nop");  // Boş işlem (No Operation)
  }
}

void Buzzer_Control(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);  // Buzzeri aç
  Delay_ms(300);  // 100 ms bekle
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // Buzzeri kapat
}

void Pic_Control_kazan(void)
{
    // Nextion ekranına va0.val=1 komutunu gönder
    char cmd[20];
    sprintf(cmd, "va1.val=1\xFF\xFF\xFF");
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
    Buzzer_Control();
}

void Pic_Control_kapak(void)
{
    char cmd[20];
    sprintf(cmd, "va2.val=1\xFF\xFF\xFF");
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
    Buzzer_Control();
}

void Pic_Control_kol(void)
{

    char cmd[20];
    sprintf(cmd, "va3.val=1\xFF\xFF\xFF");
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
    Buzzer_Control();



}

void Check_UGN_1_Pin(void) {
    // Mevcut pin durumunu oku
    uint8_t current_pin_state = HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin); // GPIOA uygun GPIO portu

    // Eğer pin durumu bir önceki durumla farklıysa işlem yap
    if (current_pin_state != previous_pin_state) {
        if (current_pin_state == GPIO_PIN_SET) {
        	Pic_Control_kazan(); // Pin SET durumundaysa bu fonksiyonu çağır
        } else {
            char cmd[20];
            sprintf(cmd, "va1.val=0\xFF\xFF\xFF");
            HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
         	// Nextion_SetVisibility("p1", 1);   // p2'yi görünür yap
        }

        // Önceki durumu güncelle
        previous_pin_state = current_pin_state;
    }
}

void Check_UGN_3_Pin(void) {
    // Mevcut pin durumunu oku
    uint8_t current_pin_state = HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin); // GPIOC uygun GPIO portu

    // Eğer pin durumu bir önceki durumla farklıysa işlem yap
    if (current_pin_state != previous_pin_state3) {
        if (current_pin_state == GPIO_PIN_SET) {
            Pic_Control_kapak(); // Pin SET durumundaysa bu fonksiyonu çağır
        } else {
            char cmd[20];
            sprintf(cmd, "va2.val=0\xFF\xFF\xFF");
            HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
        }

        // Önceki durumu güncelle
        previous_pin_state3 = current_pin_state;
    }
}

void Check_SWITCH_Pin(void) {
    // Mevcut pin durumunu oku
    uint8_t current_pin_state = HAL_GPIO_ReadPin(GPIOB, EMERGENY_Pin); // GPIOB uygun GPIO portu

    // Eğer pin durumu bir önceki durumla farklıysa işlem yap
    if (current_pin_state != previous_pin_state_switch) {
        if (current_pin_state == GPIO_PIN_SET) {
            Pic_Control_kol(); // Pin SET durumundaysa bu fonksiyonu çağır
        } else {
            char cmd[20];
            sprintf(cmd, "va3.val=0\xFF\xFF\xFF");
            HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
        }

        // Önceki durumu güncelle
        previous_pin_state_switch = current_pin_state;
    }
}

void FlashData_Write(const FlashData* data) {
    HAL_FLASH_Unlock();  // Flash yazma kilidini aç

    // Flash silme yapılandırması
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = 1;

    // Flash sayfasını sil
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();
        Error_Handler();  // Hata yönetimi fonksiyonunu çağır
        return;
    }

    // Flash belleğe veriyi yaz
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t* dataPtr = (uint32_t*)data;
    for (size_t i = 0; i < sizeof(FlashData)/4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, dataPtr[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            Error_Handler();  // Hata yönetimi fonksiyonunu çağır
            return;
        }
        address += 4;  // Adresi kelime boyutunda artır
    }

    HAL_FLASH_Lock();  // Flash kilidini kapat

    // Seri monitöre başarı mesajı gönderme
    char msg[50];
    sprintf(msg, "Flash Yazma Başarılı: UGN1DR=%d\n", data->UGN1DR);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void FlashData_Read(FlashData* data) {
    uint32_t* dataPtr = (uint32_t*)FLASH_USER_START_ADDR;
    memcpy(data, dataPtr, sizeof(FlashData));

    // Veri doğrulama
    if (data->zamann > 180) {  // Örneğin, 180 saniyeden büyük bir değer geçersiz kabul ediliyor
        data->zamann = 10;  // Varsayılan değere geri dön
        FlashData_Write(data);  // Flash belleğe geri yaz
    }

    // Seri monitöre okunan veriyi gönderme
    char msg[50];
    sprintf(msg, "Flash Okuma: Zaman=%lu, UGN1DR=%d\n", data->zamann, data->UGN1DR);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void check_manue1_timer(void) {
    // MANUE1 aktifse
    if (manue1_active) {
        // Geçen süreyi hesapla (milisaniye)
        uint32_t elapsed_time = HAL_GetTick() - manue1_start_time;

        // Kalan süreyi hesapla (saniye cinsinden)
        uint32_t remaining_seconds = 0;
        if (elapsed_time < 5000) {
            remaining_seconds = (5000 - elapsed_time) / 1000;
            if ((5000 - elapsed_time) % 1000 > 0) {
                remaining_seconds++; // Yuvarla
            }
        }

        // Kalan süreyi t0'a yazdır
        char countdown_str[10];
        sprintf(countdown_str, "%lu", remaining_seconds);
        NextionSetText(&nextion, &t0, countdown_str);

        // 5 saniye geçtiyse durdur
        if (elapsed_time >= 5000) {  // 5000ms = 5 saniye
            manue1_active = false;
            ebenin = 0;  // Manuel moddan çık

            // Röleleri kapat
            control_pins_based_on_input4();

            // Buzzer uyarısı
            Buzzer_Control();

            // Ekran kontrolü
            Nextion_SetVisibility("b9", 1);   // Start butonunu göster
            Nextion_SetVisibility("b7", 0);   // Stop butonunu gizle

            // Durum mesajı
            NextionSetText(&nextion, &t9, "MANUEL DURDU");

            // Bekleme durumunu iptal et
            is_waiting = false;

            // Geri sayımı durdur
            geri_sayim_dur();

            // Normal zaman değerini geri yaz
            char zaman_str[10];
            sprintf(zaman_str, "%lu", zaman);
            NextionSetText(&nextion, &t0, zaman_str);
        }
    }
}

void UpdateSequentialDisplay(void) {
    char buffer[20];
    switch (seqDisplay.state) {
        case SEQ_VISIBLE:
            snprintf(buffer, sizeof(buffer), "%s%d", seqDisplay.prefix, seqDisplay.currentIndex);
            Nextion_SetVisibility(buffer, 1);
            seqDisplay.lastTick = HAL_GetTick();
            seqDisplay.state = SEQ_WAIT;
            break;
        case SEQ_WAIT:
            if (HAL_GetTick() - seqDisplay.lastTick >= seqDisplay.delayMs) {
                seqDisplay.state = SEQ_INVISIBLE;
            }
            break;
        case SEQ_INVISIBLE:
            snprintf(buffer, sizeof(buffer), "%s%d", seqDisplay.prefix, seqDisplay.currentIndex);
            Nextion_SetVisibility(buffer, 0);

            if (seqDisplay.currentIndex < seqDisplay.endIndex) {
                seqDisplay.currentIndex++;
                seqDisplay.state = SEQ_VISIBLE;
            } else {
                seqDisplay.state = SEQ_IDLE;
            }
            break;

        case SEQ_IDLE:
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2)
    {
        if (rx_data[0] != 13)
        {
            if (rx_indx < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_indx++] = rx_data[0];
            }
        }
        else
        {
            rx_buffer[rx_indx] = '\0';
            rx_indx = 0;
            transfer_cplt = 1;

            if (strcmp((char*)rx_buffer, "LED ON") == 0)
            {
                NextionSetText(&nextion, &t2, "ON");
            }
            else if (strcmp((char*)rx_buffer, "LED OFF") == 0)
            {
                NextionSetText(&nextion, &t2, "OFF");
            }

            else if (strcmp((char*)rx_buffer, "DAT0000") == 0)
            {
                if (zaman < 180) {
                    zaman++;
                    reset_countdown = true;
                    Buzzer_Control();
                } else {
                    NextionSetText(&nextion, &t0, "Max");
                    return;
                }

                char zaman_str[10];
                sprintf(zaman_str, "%lu", zaman);
                NexComp t21;
                NextionSetText(&nextion, &t0, zaman_str);
        	  	zaman_flash_kaydet(zaman);

            }


            else if (strcmp((char*)rx_buffer, "DAT0001") == 0) {
                if (zaman + 5 <= 180) {
                    zaman += 5;
                    reset_countdown = true;
                    Buzzer_Control();

                } else {
                    zaman = 180;
                    NextionSetText(&nextion, &t0, "Max");
                    return;
                }

                char zaman_str[10];
                sprintf(zaman_str, "%lu", zaman);
                NexComp t21;
                NextionSetText(&nextion, &t0, zaman_str);
        	  	zaman_flash_kaydet(zaman);

            }



            else if (strcmp((char*)rx_buffer, "DAT0002") == 0) {
                if (zaman + 10 <= 180) {
                    zaman += 10;
                    reset_countdown = true;
                    Buzzer_Control();


                } else {
                    zaman = 180;
                    NextionSetText(&nextion, &t0, "Max");
                    return;
                }

                // Zamanı string'e çevirip t21 textbox'ına yaz
                char zaman_str[10];
                sprintf(zaman_str, "%lu", zaman);  // Zamanı string formatına çeviriyoruz
                NexComp t21; // t21 bileşeni tanımlıyoruz
                NextionSetText(&nextion, &t0, zaman_str);  // t21 textbox'ına zaman değerini yaz
        	  	zaman_flash_kaydet(zaman);

            }



            else if (strcmp((char*)rx_buffer, "DAT0005") == 0) {
                if (zaman > 1) {  // Zaman 1'den büyükse azalt
                    zaman--;
                    char zaman_str[10];
                    sprintf(zaman_str, "%lu", zaman);
                    NextionSetText(&nextion, &t0, zaman_str);
            	  	zaman_flash_kaydet(zaman);
                    reset_countdown = true;
                    Buzzer_Control();



                } else {
                    NextionSetText(&nextion, &t0, "Min");  // Minimum değer uyarısı
                }
            }


            else if (strcmp((char*)rx_buffer, "DAT0004") == 0) {
                if (zaman > 5) {  // Zaman 5'ten büyükse azalt
                    zaman -= 5;
                    char zaman_str[10];
                    sprintf(zaman_str, "%lu", zaman);
                    NextionSetText(&nextion, &t0, zaman_str);
            	  	zaman_flash_kaydet(zaman);
                    reset_countdown = true;
                    Buzzer_Control();



                } else {
                    NextionSetText(&nextion, &t0, "Min");  // Minimum değer uyarısı
                }
            }


            else if (strcmp((char*)rx_buffer, "DAT0003") == 0) {
                if (zaman > 10) {  // Zaman 10'dan büyükse azalt
                    zaman -= 10;  // Zamanı 10 azalt
                    char zaman_str[10];
                    sprintf(zaman_str, "%lu", zaman);  // Zamanı string formatına çeviriyoruz
                    NextionSetText(&nextion, &t0, zaman_str);  // Nextion ekranında göster
            	  	zaman_flash_kaydet(zaman);
                    reset_countdown = true;
                    Buzzer_Control();


                } else {
                    NextionSetText(&nextion, &t0, "Min");  // Minimum değer uyarısı
                }
            }



            else if (strcmp((char*)rx_buffer, "ANAEKR") == 0)
            {

            	Nextion_GoToPage("page1");            // veya Nextion_SendCommand("page 1\xFF\xFF\xFF");
            	Delay_ms(150);
                            Buzzer_Control();  // "ANAEKR" mesajı geldiğinde Buzzer_Control fonksiyonunu çağır
                            mainpage();

            }


            if (strcmp((char*)rx_buffer, "MANUE1") == 0)
            {
                // 1) Tüm pinleri kontrol et
                bool all_pins_ok =
                    ((UGN1DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin) == GPIO_PIN_SET)) &&
                    ((UGN2DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_2_Pin) == GPIO_PIN_SET)) &&
                    ((UGN3DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin) == GPIO_PIN_SET)) &&
                    ((UGN4DR == 0) || (HAL_GPIO_ReadPin(GPIOB, UGN_4_Pin) == GPIO_PIN_SET)) &&
                    ((UGN5DR == 0) || (HAL_GPIO_ReadPin(GPIOB, EMERGENY_Pin) == GPIO_PIN_SET));



                // 3) Eğer tüm pinler uygunsa (all_pins_ok == true), 5 saniye işlemi başlat
                if (all_pins_ok)
                {
                    manue1_active = true;
                    manue1_start_time = HAL_GetTick();  // Başlangıç zamanını kaydet
                    control_pins_based_on_input3();   // Röleleri aktif et
                    Buzzer_Control();
                    ebenin = 1;
                    Nextion_SetVisibility("b7", 1);   // Stop butonunu göster
                    Nextion_SetVisibility("b9", 0);   // Start butonunu gizle
                    //Nextion_SetVisibility("p30", 1);
                    //Nextion_SetVisibility("p13", 1);
                    //Nextion_SetVisibility("p22", 1);

                    // Durum mesajı
                    NextionSetText(&nextion, &t9, "MANUEL AKTIF");
                }
            }


                       else if (strcmp((char*)rx_buffer, "DATTEST") == 0)
                       {

                           reset_countdown = true;
                           ZamanDegeriniYazdir();
                           Buzzer_Control();

                  	   }






            if (strcmp((char*)rx_buffer, "0START") == 0)
            {

                // 1) Tüm pinleri kontrol et
                bool all_pins_ok =
                    ((UGN1DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin) == GPIO_PIN_SET)) &&
                    ((UGN2DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_2_Pin) == GPIO_PIN_SET)) &&

                    ((UGN3DR == 0) || (HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin) == GPIO_PIN_SET)) &&
                    ((UGN4DR == 0) || (HAL_GPIO_ReadPin(GPIOB, UGN_4_Pin) == GPIO_PIN_SET)) &&

					((UGN5DR == 0) || (HAL_GPIO_ReadPin(GPIOB, EMERGENY_Pin) == GPIO_PIN_SET));


                if (all_pins_ok)
                {
                    is_counting = true;               // Global geri sayımı başlat
                    control_pins_based_on_input3();   // İlk fonksiyonu çağır
                    Buzzer_Control();
                    //Buzzer_Beep();
                    ebenin=1;// Sesli uyarı
                    Nextion_SetVisibility("b9", 0);   // p1'i görünür yap

                    Nextion_SetVisibility("b7", 1);   // p1'i görünür yap
            	    Delay_ms(90);  // 100 ms bekle

                    //Nextion_SetVisibility("p30", 1);   // p1'i görünür yap
                    //Nextion_SetVisibility("p13", 1);   // p1'i görünür yap
                    //Nextion_SetVisibility("p22", 1);   // p1'i görünür yap


                }
            }

            // EÄŸer "00STOP" mesajÄ± alÄ±nÄ±rsa
            else if (strcmp((char*)rx_buffer, "00STOP") == 0) {
                control_pins_based_on_input4();  // Ä°kinci fonksiyonu Ã§aÄŸÄ±r
                Buzzer_Control();
                is_waiting = false;  // Bekleme durumu iptal
                NextionSetText(&nextion, &t9, "OFF");
                geri_sayim_dur(); // Geri sayımı durdur
                Nextion_SetVisibility("b9", 1);   // p1'i görünür yap

            }


            else if (strcmp((char*)rx_buffer, "SETGET") == 0) {
            	HandleSetGetMessage();

            }

            else if (strcmp((char*)rx_buffer, "UGN1-0") == 0) {
            	UGN1DR=0;
        	    flashData.UGN1DR = 0;
                FlashData_Write(&flashData);  // Flash'e kaydet

				NextionSetText(&nextion, &t31, "Pasif");


            }

            else if (strcmp((char*)rx_buffer, "UGN1-1") == 0) {
            	UGN1DR=1;
        	    flashData.UGN1DR = 1;
                FlashData_Write(&flashData);  // Flash'e kaydet

				NextionSetText(&nextion, &t31, "Aktif");


            }

            else if (strcmp((char*)rx_buffer, "UGN2-0") == 0) {
            	UGN2DR=0;
				NextionSetText(&nextion, &t32, "Pasif");

        	    flashData.UGN2DR = 0;
                FlashData_Write(&flashData);  // Flash'e kaydet


            }

            else if (strcmp((char*)rx_buffer, "UGN2-1") == 0) {
            	UGN2DR=1;
				NextionSetText(&nextion, &t32, "Aktif");
        	    flashData.UGN2DR = 1;
                FlashData_Write(&flashData);  // Flash'e kaydet

            }

            else if (strcmp((char*)rx_buffer, "UGN3-0") == 0) {
            	UGN3DR=0;
				NextionSetText(&nextion, &t33, "Pasif");
        	    flashData.UGN3DR = 0;
                FlashData_Write(&flashData);  // Flash'e kaydet

            }

            else if (strcmp((char*)rx_buffer, "UGN3-1") == 0) {
            	UGN3DR=1;
				NextionSetText(&nextion, &t33, "Aktif");
        	    flashData.UGN3DR = 1;
                FlashData_Write(&flashData);

            }

            else if (strcmp((char*)rx_buffer, "UGN4-0") == 0) {
            	UGN4DR=0;
				NextionSetText(&nextion, &t34, "Pasif");
        	    flashData.UGN4DR = 0;
                FlashData_Write(&flashData);

            }
            else if (strcmp((char*)rx_buffer, "UGN4-1") == 0) {
            	UGN4DR=1;
				NextionSetText(&nextion, &t34, "Aktif");
        	    flashData.UGN4DR = 1;
                FlashData_Write(&flashData);

            }
            else if (strcmp((char*)rx_buffer, "UGN5-0") == 0) {
            	UGN5DR=0;
				NextionSetText(&nextion, &t35, "Pasif");
        	    flashData.UGN5DR = 0;
                FlashData_Write(&flashData);

            }
            else if (strcmp((char*)rx_buffer, "UGN5-1") == 0) {
            	UGN5DR=1;
				NextionSetText(&nextion, &t35, "Aktif");
        	    flashData.UGN5DR = 1;
                FlashData_Write(&flashData);  // Flash'e kaydet

            }

            memset(rx_buffer, 0, sizeof(rx_buffer));
        }

        HAL_UART_Receive_IT(&huart2, rx_data, 1);
    }
}


void Nextion_SendCommand(char *cmd)
{
    // cmd zaten sonuna \xFF\xFF\xFF konulmuş bir string ise direkt gönderilir.
    // veya sonradan eklenmek istenirse birleştirilebilir.
    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), 100);
}



void HandleSetGetMessage(void) {
    // UGN1DR değerini t31'e yazdır
    if (UGN1DR == 1) {
        NextionSetText(&nextion, &t31, "Aktif");
    } else {
        NextionSetText(&nextion, &t31, "Pasif");
    }

    // UGN2DR değerini t32'ye yazdır
    if (UGN2DR == 1) {
        NextionSetText(&nextion, &t32, "Aktif");
    } else {
        NextionSetText(&nextion, &t32, "Pasif");
    }

    // UGN3DR değerini t33'e yazdır
    if (UGN3DR == 1) {
        NextionSetText(&nextion, &t33, "Aktif");
    } else {
        NextionSetText(&nextion, &t33, "Pasif");
    }

    // UGN4DR değerini t34'e yazdır
    if (UGN4DR == 1) {
        NextionSetText(&nextion, &t34, "Aktif");
    } else {
        NextionSetText(&nextion, &t34, "Pasif");
    }

    // UGN5DR değerini t35'e yazdır
    if (UGN5DR == 1) {
        NextionSetText(&nextion, &t35, "Aktif");
    } else {
        NextionSetText(&nextion, &t35, "Pasif");
    }
}


void check_wait_time(void)
						 {
    					  if (is_waiting && (HAL_GetTick() - start_time >= zaman * 1000)) {  // Zaman deÄŸiÅŸkeni kadar sÃ¼reyi bekle
    						  control_pins_based_on_input4();  // Bekleme sÃ¼resi bitince fonksiyonu Ã§aÄŸÄ±r
    						  is_waiting = false;  // Bekleme durumunu bitir
    					  }
						 }

void ZamanDegeriniYaz(void)
						  {
    					   char zaman_str[10];
    					   sprintf(zaman_str, "%lu", zaman);  // ZamanÄ± string formatÄ±na Ã§evir
    					   NextionSetText(&nextion, &t21, zaman_str);  // t21 textbox'Ä±na zaman deÄŸerini yaz
						  }

void StoppedTimeDegeriniYaz(void) {
    char zaman_str[10];

    if (stopped_time == 0) {
        sprintf(zaman_str, "%lu", zaman);  // zaman değişkenini string formatına çevir
        NextionSetText(&nextion, &t0, zaman_str);  // t0 textbox'ına zaman değerini yaz
    } else {
        sprintf(zaman_str, "%d", stopped_time);  // stopped_time'ı string formatına çevir
        NextionSetText(&nextion, &t0, zaman_str);  // t0 textbox'ına stopped_time değerini yaz
    }
}

void zaman_flash_kaydet(uint32_t zaman) {
    HAL_FLASH_Unlock();  // Flash belleği yazılabilir duruma getir

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // Flash bellek sayfasını silmek için yapılandırma
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = 1;

    // Sayfayı sil
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock();  // Flash belleği tekrar kilitle
        return;  // Silme işlemi başarısız olduysa çık
    }

    // Zaman değerini Flash belleğe yaz
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR, zaman) != HAL_OK) {
        HAL_FLASH_Lock();  // Flash belleği tekrar kilitle
        return;  // Yazma işlemi başarısız olduysa çık
    }

    HAL_FLASH_Lock();  // Flash belleği tekrar kilitle
}

uint32_t zaman_flash_oku(void) {
    uint32_t saved_time = *(uint32_t*)FLASH_USER_START_ADDR;  // Flash adresinden oku

    // Geçerli bir değer kontrolü (örnek: 0xFFFFFFFF geçersiz kabul edilir)
    if (saved_time == 0xFFFFFFFF) {
        return 10;  // Geçerli bir değer yoksa varsayılan bir değer döndür (örneğin 10)
    }
    return saved_time;
}

void geri_sayim_dur(void) {
    is_counting = false; // Geri sayımı durdur
}

void geri_sayim_zaman_ile(void) {
    static uint32_t last_tick = 0;           // Son zaman kontrolü
    static int remaining_time = 0;           // Kalan süre
    static bool local_is_counting = false;   // Yerel geri sayım bayrağı
    static int saved_time = -1;              // Kaydedilen süre (-1 = kayıt yok)

    if (!is_counting) {  // Global geri sayım başlatılmamışsa
        if (local_is_counting) {             // Eğer geri sayım durdurulmuşsa
            saved_time = remaining_time;     // Kalan süreyi kaydet
            local_is_counting = false;       // Yerel bayrağı sıfırla

            // Ekrana kaydedilen süreyi yazdır
            char buffer[10];
            sprintf(buffer, "%d", saved_time);
            NextionSetText(&nextion, &t0, buffer);
        }
        return;
    }

    // Sıfırlama talebi var mı?
    if (reset_countdown) {
        remaining_time = zaman - 1;        // Kalan süreyi yeniden başlat
        saved_time = -1;                   // Kaydedilen süreyi sıfırla
        last_tick = HAL_GetTick();         // Başlangıç zamanını güncelle
        local_is_counting = true;          // Yerel geri sayımı başlat
        reset_countdown = false;           // Sıfırlama bayrağını temizle
    }

    if (!local_is_counting) {  // Yerel geri sayım ilk kez başlatılıyorsa
        if (saved_time >= 0) {
            // Önceden kaydedilmiş süre varsa oradan devam et
            remaining_time = saved_time;
        } else {
            // Kaydedilmiş süre yoksa baştan başla
            remaining_time = zaman - 1;
        }

        // Başlangıç değerini hemen ekrana yazdır
        char buffer[10];
        sprintf(buffer, "%d", remaining_time);
        NextionSetText(&nextion, &t0, buffer);

        local_is_counting = true;      // Yerel geri sayımı başlat
        last_tick = HAL_GetTick();     // Başlangıç zamanını güncelle
        saved_time = -1;               // Kaydedilen süreyi temizle (artık kullanımda)
    }

    // 1 saniye geçtiyse
    if (local_is_counting && (HAL_GetTick() - last_tick >= 1000)) {
        char buffer[10];
        sprintf(buffer, "%d", remaining_time);          // Kalan süreyi stringe çevir
        NextionSetText(&nextion, &t0, buffer);          // Ekrana yazdır

        remaining_time--;             // Geri sayımı azalt
        last_tick = HAL_GetTick();    // Zamanı güncelle

        if (remaining_time < 0) {     // Geri sayım bitti mi?
            local_is_counting = false; // Yerel geri sayımı durdur
            is_counting = false;      // Global geri sayımı durdur
            saved_time = -1;          // Kaydedilen süreyi temizle
            control_pins_based_on_input4();  // Zaman dolduğunda pin kontrolü yap

            // t0 alanına tekrar zaman değişkenini yaz
            sprintf(buffer, "%lu", zaman);           // Zaman değerini stringe çevir
            NextionSetText(&nextion, &t0, buffer);    // t0 alanına yazdır
        }
    }
}

void ZamanDegeriniYazdir(void) {
    char buffer[10];  // Zaman değerini string'e çevirmek için buffer

    // Zaman değerini string formatına çevir
    sprintf(buffer, "%d", zaman);

    // Nextion ekranındaki t0 textbox'ına yazdır
    NextionSetText(&nextion, &t0, buffer);
}

void guvenlik(void)
				   {
				    if (HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin) == GPIO_PIN_SET &&  // UGN_1 is active
					    HAL_GPIO_ReadPin(GPIOA, UGN_2_Pin) == GPIO_PIN_SET &&  // UGN_2 is active
					    HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin) == GPIO_PIN_SET &&  // UGN_3 is active
					    HAL_GPIO_ReadPin(GPIOB, UGN_4_Pin) == GPIO_PIN_SET &&  // UGN_4 is active
					    HAL_GPIO_ReadPin(GPIOB, SWITCH_Pin) == GPIO_PIN_RESET)   // SWITCH is active
				   {
					   isg = 0;
				   }
				   else
				   {
					isg = 0;
				   }
				    char isg_str[10];
				    sprintf(isg_str, "%d", isg);
				    NextionSetText(&nextion, &t2, isg_str);
				   }

void guvenlik2(void)
					{
					 if (HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin) == GPIO_PIN_SET) {
						 NextionSetText(&nextion, &t2, "SENSOR SET");
					}
					 else
					 	 {
						 NextionSetText(&nextion, &t2, "SENSOR RESET");
					}

					 if (HAL_GPIO_ReadPin(GPIOA, UGN_2_Pin) == GPIO_PIN_SET) {
						 NextionSetText(&nextion, &t3, "SENSOR SET");
					}
					 else
					 	 {
						 NextionSetText(&nextion, &t3, "SENSOR RESET");

					 	 }

					 if (HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin) == GPIO_PIN_SET) {
						 NextionSetText(&nextion, &t4, "SENSOR SET");
					}
					 else
					 	 {
						 NextionSetText(&nextion, &t4, "SENSOR RESET");

					 	 }

					 if (HAL_GPIO_ReadPin(GPIOA, UGN_4_Pin) == GPIO_PIN_SET) {
						 NextionSetText(&nextion, &t5, "SENSOR SET");
					}
					 else
					 	 {
						 NextionSetText(&nextion, &t5, "SENSOR RESET");

					 	 }

					 if (HAL_GPIO_ReadPin(GPIOA, SWITCH_Pin) == GPIO_PIN_SET) {
						 NextionSetText(&nextion, &t6, "SWITCH SET");
					}
					 else
					 	 {
						 NextionSetText(&nextion, &t6, "SWITCH RESET");

					 	 }

					 if (HAL_GPIO_ReadPin(GPIOA, SWITCH_Pin) == GPIO_PIN_SET) {
					 	NextionSetText(&nextion, &t7, "SWITCH SET");
					 	}
					  else
					 	{
					 	NextionSetText(&nextion, &t7, "SWITCH RESET");

					 	}
}

int digitalRead(int phase) {
    						if (phase == 1) {
    																return PHASE1_SIGNAL;
    										} else if (phase == 2) {
    																return PHASE2_SIGNAL;
    										} else if (phase == 3) {
    																return PHASE3_SIGNAL;
    										}
    				 						 return 0; // VarsayÄ±lan olarak sinyal yok
											}

unsigned long micros()
					  {
						struct timeval tv;
						gettimeofday(&tv, NULL);
						return (tv.tv_sec * 1000000) + tv.tv_usec;
					   }

void Measure()
			   {
    			while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
    			t1_start = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // ZamanÄ± kaydet
    			while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));
    			t1_end = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);   // ZamanÄ± kaydet
    			while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
    			t2_start = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // ZamanÄ± kaydet
    			while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10));
    			t2_end = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);   // ZamanÄ± kaydet
			   }

int calculatioins(int g) {
						  unsigned int k = 0;

						  g = g + 1;

						  float pf = (float)g / 1000000;

						  pf = pf * 50 * 360;  // 50 Hz frekans, 360 derece

						  k = (unsigned int)pf;

						  calc_value = k;

						  return k;
						}

uint8_t all_channels_zero(void)
								{
								 return (timestamp_phase1 == 0 && timestamp_phase2 == 0 && timestamp_phase3 == 0);
								}

int calculate_phase_angle(uint32_t time_difference)
													{
													 return ((float)time_difference / PERIOD_MS) * 360.0f;  // 360 dereceyi tam periyotla iliÅŸkilendiriyoruz
													}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
													{
														if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // Phase 1 (PA8)
														{
															timestamp_phase1_zero_cross = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
														}
														else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // Phase 2 (PA9)
														{
															timestamp_phase2_zero_cross = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
														}
														else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // Phase 3 (PA10)
														{
															timestamp_phase3_zero_cross = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
														}

														if (timestamp_phase1_zero_cross != 0 && timestamp_phase2_zero_cross != 0 && timestamp_phase3_zero_cross != 0)
														{
														if (timestamp_phase1_zero_cross < timestamp_phase2_zero_cross && timestamp_phase2_zero_cross < timestamp_phase3_zero_cross)
														{
															phase_sequence = 1;  // RST sÄ±rasÄ±
														}
														else if (timestamp_phase3_zero_cross < timestamp_phase2_zero_cross && timestamp_phase2_zero_cross < timestamp_phase1_zero_cross)
														{
																phase_sequence = 2;  // TSR sÄ±rasÄ±
														}
														else
														{
																phase_sequence = 0;  // GeÃ§ersiz faz sÄ±rasÄ±
														}

	        timestamp_phase1_zero_cross = 0;
	        timestamp_phase2_zero_cross = 0;
	        timestamp_phase3_zero_cross = 0;
														}
													}

void measure_phase_sequence(void)
								{
								 uint8_t input3_measurements[MAX_MEASUREMENTS] = {0};  // Ã–lÃ§Ã¼mleri saklamak iÃ§in dizi
								 uint8_t rst_count = 0, tsr_count = 0;
								 uint8_t i;
						   for (i = 0; i < MAX_MEASUREMENTS; i++)
						   {
							if (phase_sequence == 1)
							{
								input3 = 1;
								NextionSetText(&nextion, &t36, "R-S-T");
								input3_measurements[i] = 1;
								rst_count++;
							}
							else if (phase_sequence == 2)
							{
								input3 = 2;
								NextionSetText(&nextion, &t36, "T-S-R");
								input3_measurements[i] = 2;
								tsr_count++;
							}
							HAL_Delay(10);  // Her Ã¶lÃ§Ã¼m arasÄ±nda 10ms bekle
						   }
						   if (rst_count > tsr_count)
						   {
							   printf("Final Phase sequence: RST (Most frequent)\n");
							   input3 = 1;
						   }
						   else
						   {
							   printf("Final Phase sequence: TSR (Most frequent)\n");
							   input3 = 2;
						   }
								}

 void Buzzer_Beep(void) {

    HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_SET); // Buzzer'ı aç

    HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_RESET); // Buzzer'ı aç


}

void control_pins_based_on_input3(void)
					{
				     if (input3 == 2)
				     {
				    	 HAL_GPIO_WritePin(GPIOB, RELAY1_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY3_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY4_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY2_Pin, GPIO_PIN_RESET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY5_Pin, GPIO_PIN_RESET);
				     }
				     else if (input3 == 1)
				     {
				    	 HAL_GPIO_WritePin(GPIOB, RELAY5_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY2_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY3_Pin, GPIO_PIN_SET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY1_Pin, GPIO_PIN_RESET);
				    	 HAL_GPIO_WritePin(GPIOB, RELAY4_Pin, GPIO_PIN_RESET);
				     }
					}

void control_pins_based_on_input4(void)
									  {

        						       HAL_GPIO_WritePin(GPIOB, RELAY1_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY2_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY3_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY4_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY5_Pin, GPIO_PIN_RESET);
        							    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);  // Buzzeri aç
        							    Delay_ms(300);  // 100 ms bekle
        							    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // Buzzeri kapat
        							    Delay_ms(300);  // 100 ms bekle

        							    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);  // Buzzeri aç
        							    Delay_ms(300);  // 100 ms bekle
        							    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // Buzzeri kapat
        							    Delay_ms(300);  // 100 ms bekle


									  }

void control_pins_based_on_input5(void)
									  {

        						       HAL_GPIO_WritePin(GPIOB, RELAY1_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY2_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY3_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY4_Pin, GPIO_PIN_RESET);
        						       HAL_GPIO_WritePin(GPIOB, RELAY5_Pin, GPIO_PIN_RESET);

									  }

void Nextion_SetVisibility(const char* objectName, uint8_t visible)
{
    char cmd[50];
    sprintf(cmd, "vis %s,%d\xFF\xFF\xFF", objectName, visible);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 100);
}

void Nextion_GoToPage(const char *pageName) {
    char command[50];
    snprintf(command, sizeof(command), "page %s", pageName); // Komut oluştur
    HAL_UART_Transmit(&huart2, (uint8_t *)command, strlen(command), HAL_MAX_DELAY); // Komutu gönder
    uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF}; // Nextion komut bitiş dizisi
    HAL_UART_Transmit(&huart2, end_cmd, 3, HAL_MAX_DELAY); // Komut bitişini gönder
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2)
    {
        GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

        if (pinState == GPIO_PIN_SET && is_counting2 == false)
        {
        	Nextion_GoToPage("page1");            // veya Nextion_SendCommand("page 1\xFF\xFF\xFF");
            HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_RESET); // Buzzer'ı aç
            mainpage();
    	  	StoppedTimeDegeriniYaz();
    	  	is_counting2= true;
        }
        else if(pinState == GPIO_PIN_RESET)
        {
        	//Delay_ms(100);
            is_counting = false; // Geri sayımı durdur
            is_counting2 = false;
        	Nextion_GoToPage("page3");
        	control_pins_based_on_input4();// veya Nextion_SendCommand("page 1\xFF\xFF\xFF");
            HAL_GPIO_WritePin(GPIOA, BUZZER_Pin, GPIO_PIN_SET); // Buzzer'ı aç

        }
    }
}

void mainpage(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, UGN_1_Pin) == GPIO_PIN_SET)
    {
        Nextion_SetVisibility("p13", 1);
    }
    else
    {
        Nextion_SetVisibility("p1", 1);
    }

    if (HAL_GPIO_ReadPin(GPIOA, UGN_3_Pin) == GPIO_PIN_SET)
    {
        Nextion_SetVisibility("p22", 1);
    }
    else
    {
        Nextion_SetVisibility("p15", 1);
    }

    if (HAL_GPIO_ReadPin(GPIOB, EMERGENY_Pin) == GPIO_PIN_RESET)
    {
        Nextion_SetVisibility("p24", 1);
    }
    else
    {
        Nextion_SetVisibility("p30", 1);
    }

    //StoppedTimeDegeriniYaz();  // StoppedTimeDegeriniYaz fonksiyonunu çağır
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); // PA8 sinyali yakalama baÅŸlat
   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2); // PA9 sinyali yakalama baÅŸlat
   HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3); // PA10 sinyali yakalama baÅŸlat

    NextionInit(&nextion, &huart2);
    NextionAddComp(&nextion, &t0, "t0", 2, 8, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t1, "t1", 2, 2, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t4, "t4", 2,6, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t2, "t2", 2, 3, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t3, "t3", 2,5, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t5, "t5", 2,7, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t6, "t6", 2,8, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t7, "t7", 2,9, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t9, "t9", 1,13, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t10, "t10", 1,14, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t11, "t11", 1,15, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t12, "t12", 1,18, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t21, "t21", 1,1, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t22, "t22", 1,12, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t23, "t23", 1,16, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t24, "t24", 1,17, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t25, "t25", 1,18, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t26, "t26", 1,19, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t27, "t27", 1,20, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t28, "t28", 1,21, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t31, "t31", 2,14, NULL, NULL);  // t2 bileÅŸenini tanÄ±mla
    NextionAddComp(&nextion, &t32, "t32", 2,15, NULL, NULL);
    NextionAddComp(&nextion, &t33, "t33", 2,16, NULL, NULL);
    NextionAddComp(&nextion, &t34, "t34", 2,17, NULL, NULL);
    NextionAddComp(&nextion, &t35, "t35", 2,18, NULL, NULL);
    NextionAddComp(&nextion, &t36, "t36", 2,19, NULL, NULL);


    HAL_UART_Receive_IT(&huart2, rx_data, 1);  // UART kesmesini baÅŸlat
    HAL_Delay(2500);
    zaman = zaman_flash_oku();
  	measure_phase_sequence();
  	check_wait_time();  // Zaman kontrolÃ¼nÃ¼ yap
  	guvenlik2();
  	ZamanDegeriniYazdir();

    FlashData_Read(&flashData);

    if (flashData.zamann == 0xFFFFFFFF || flashData.zamann == 0) {
        flashData.zamann = 10;  // Geçerli bir değer yoksa varsayılan bir değer ata
        FlashData_Write(&flashData);
    }

    UGN1DR = flashData.UGN1DR;
    UGN2DR = flashData.UGN2DR;
    UGN3DR = flashData.UGN3DR;
    UGN4DR = flashData.UGN4DR;
    UGN5DR = flashData.UGN5DR;

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);  // Buzzeri aç
    Delay_ms(300);  // 100 ms bekle
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // Buzzeri kapat
    Delay_ms(300);  // 100 ms bekle
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);  // Buzzeri aç
    Delay_ms(300);  // 100 ms bekle
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  // Buzzeri kapat
    Delay_ms(700);  // 100 ms bekle

    Nextion_SetVisibility("p15", 1);   // p1'i görünür yap
    Delay_ms(300);  // 100 ms bekle

    Nextion_SetVisibility("p1", 1);   // p1'i görünür yap
    Delay_ms(300);  // 100 ms bekle


    Nextion_SetVisibility("p24", 1);   // p1'i görünür yap
    //Nextion_SetVisibility("p15", 1);   // p1'i görünür yap


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    HAL_Delay(50);
	    geri_sayim_zaman_ile();
	  	check_wait_time();  // Zaman kontrolÃ¼nÃ¼ yap
        //ShowSequentially("p", 1, 8, 200);
	    //check_wait_time();  // Zaman kontrolünü yap
	    check_manue1_timer();
	  	Check_UGN_1_Pin();
	  	Check_UGN_3_Pin();
	  	Check_SWITCH_Pin();
	  	check_and_control_pins();




       // Nextion_SetVisibility("b9", 0);   // p1'i görünür yap


     /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY1_Pin|RELAY2_Pin|RELAY3_Pin|RELAY4_Pin
                          |RELAY5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UGN_1_Pin UGN_2_Pin UGN_3_Pin */
  GPIO_InitStruct.Pin = UGN_1_Pin|UGN_2_Pin|UGN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : UGN_4_Pin EMERGENY_Pin PB10 PB11 */
  GPIO_InitStruct.Pin = UGN_4_Pin|EMERGENY_Pin|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SWITCH_Pin */
  GPIO_InitStruct.Pin = SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY1_Pin RELAY2_Pin RELAY3_Pin RELAY4_Pin
                           RELAY5_Pin */
  GPIO_InitStruct.Pin = RELAY1_Pin|RELAY2_Pin|RELAY3_Pin|RELAY4_Pin
                          |RELAY5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
