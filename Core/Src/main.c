/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Home - Nucleo-F411RE
  ******************************************************************************
  *
  * Horloge : HSE 8 MHz -> PLL (M=4, N=100, P=2) -> SYSCLK 100 MHz
  *           APB1 = 50 MHz (timers APB1 a 100 MHz), APB2 = 100 MHz
  *
  * Brochage :
  *   PA5  LD2            sortie, bascule a chaque tick de TIM2
  *   PC13 B1             EXTI13, front descendant (pull-up sur la carte)
  *   PC10 PIR            EXTI10, front montant, pull-down
  *   PA9  bouton fenetre EXTI9, front descendant, pull-up
  *   PC12 LED alarme     sortie push-pull (une seule LED dans le kit)
  *   PC7  buzzer         TIM3_CH2, PWM 2 kHz
  *   PB6  servo          TIM4_CH1, PWM 50 Hz
  *   PA0  MQ-2           ADC1 canal 0
  *   PA1  pluie          ADC1 canal 1
  *   PB8/PB9 TMP102      I2C1 (SCL/SDA), 100 kHz, adresse 0x48
  *   PA2/PA3 UART        USART2 vers le ST-LINK, 115200 8N1
  *
  * Principe : les interruptions ne font que lever des drapeaux, tout le
  * traitement se fait dans la boucle principale. TIM2 cadence les mesures
  * toutes les 2 s.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "tmp102.h"     // driver du capteur de temperature
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MODE_HOME = 0,      // maison occupee
	MODE_AWAY = 1       // maison vide : intrusion surveillee, fenetre fermee
} SystemMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define anti_rebond_ms  			200U    // on ignore les fronts pendant 200 ms apres un appui
#define PERIODE_CLIGNOTEMENT_MS     250U    // LED d'alarme : 250 ms allumee / 250 ms eteinte

/* Seuils gaz (valeurs ADC 0..4095), fixes a partir de nos mesures :
   air ~1100-1600, vapeurs de gel ~3200 */
#define GAZ_SEUIL_HAUT        		2500U   // declenchement de l'alarme
#define GAZ_SEUIL_BAS         		2000U   // fin d'alarme (hysteresis)
#define GAZ_NB_CONFIRMATION   		2U      // mesures consecutives pour changer d'etat
#define GAZ_SEUIL_DEFAUT      		200U    // en dessous : capteur debranche ou faux contact

/* Largeurs d'impulsion du servo (en us), trouvees en caracterisant la fenetre */
#define SERVO_CLOSED_US   			600U
#define SERVO_OPEN_US     			1750U

/* Seuils pluie : sec ~150, mouille = 4095 */
#define PLUIE_SEUIL_HAUT        	2000U
#define PLUIE_SEUIL_BAS         	1000U
#define PLUIE_NB_CONFIRMATION   	2U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* volatile = variable modifiee dans une interruption et lue dans la boucle */

/* Mode HOME / AWAY (bouton B1) */
volatile SystemMode_t g_mode = MODE_HOME;   // HOME au demarrage
volatile uint32_t g_nb_appuis = 0;          // compteur de test (anti-rebond)
static uint32_t g_dernier_appui_ms = 0;     // heure du dernier appui accepte
volatile uint8_t g_mode_event = 0;          // drapeau : le mode vient de changer

/* Intrusion (PIR) */
volatile uint32_t g_nb_pir = 0;             // compteur de test
volatile uint8_t g_pir_event = 0;           // drapeau : mouvement detecte
uint8_t g_alarme_intrusion = 0;

/* LED d'alarme */
static uint32_t g_dernier_clignotement_ms = 0;

/* Acquisition periodique */
volatile uint8_t g_acq_event = 0;           // drapeau leve par TIM2 toutes les 2 s

/* Gaz */
uint16_t g_gaz_brut = 0;                    // derniere mesure ADC
uint8_t g_alarme_gaz = 0;
static uint8_t g_gaz_compteur = 0;          // mesures consecutives qui confirment un changement

/* Fenetre */
static uint8_t g_fenetre_ouverte = 0;       // position reelle du servo
volatile uint8_t g_fenetre_event = 0;       // drapeau : appui sur le bouton fenetre
static uint32_t g_dernier_appui_fenetre_ms = 0;
uint8_t g_fenetre_demande = 0;              // ce que veut l'utilisateur (0 fermer, 1 ouvrir)

/* Pluie */
uint16_t g_pluie_brut = 0;
uint8_t g_pluie = 0;
static uint8_t g_pluie_compteur = 0;

/* Temperature */
uint8_t g_tmp102_ok = 0;                    // capteur detecte au demarrage
int32_t g_temp_centi = 0;                   // en centiemes de degre (2506 = 25,06 C)
uint8_t g_temp_valide = 0;                  // au moins une mesure lue
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void uart_print(const char *msg);
static void maj_buzzer(void);
static void fenetre_commander(uint8_t ouvrir);
static uint16_t adc_lire(uint32_t canal);
static void temp_texte(char *buf, size_t taille, int32_t centi);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
	  MX_TIM4_Init();
	  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* base de temps : interruption toutes les 2 s (PSC 9999, ARR 19999) */
  HAL_TIM_Base_Start_IT(&htim2);
  uart_print("\r\n=== Smart Home : demarrage en mode HOME ===\r\n");

  /* servo : on fixe la position fermee avant de lancer le PWM,
     pour que la premiere impulsion soit deja la bonne */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, SERVO_CLOSED_US);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  /* TMP102 : verifie qu'il repond puis le passe en mode one-shot */
  g_tmp102_ok = (tmp102_init() == HAL_OK);
  uart_print(g_tmp102_ok ? "[TMP102] Capteur detecte et configure (one-shot)\r\n"
                         : "[TMP102] ERREUR : capteur absent ou non configure\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* ---- Bouton MODE : HOME <-> AWAY ---- */
	  if (g_mode_event)
	  {
	      g_mode_event = 0;
	      if (g_mode == MODE_AWAY)
	      {
	          uart_print("[MODE] AWAY - surveillance activee\r\n");
	      }
	      else
	      {
	          uart_print("[MODE] HOME\r\n");
	          /* l'occupant est rentre : on desarme l'intrusion.
	             l'alarme gaz, elle, n'est pas touchee */
	          if (g_alarme_intrusion)
	          {
	              g_alarme_intrusion = 0;
	              uart_print("[ALARME] Intrusion arretee\r\n");
	          }
	      }
	  }

	  /* ---- PIR : un mouvement n'est une intrusion qu'en AWAY ---- */
	  if (g_pir_event)
	  {
	      g_pir_event = 0;
	      if (g_mode == MODE_AWAY)
	      {
	          if (!g_alarme_intrusion)      // on ne declenche qu'une fois
	          {
	              g_alarme_intrusion = 1;
	              uart_print("[ALARME] INTRUSION DETECTEE !\r\n");
	          }
	      }
	      else
	      {
	          uart_print("[PIR] Mouvement detecte (mode HOME, normal)\r\n");
	      }
	  }

	  /* ---- Bouton fenetre : change seulement la demande de l'utilisateur.
	          La position reelle est decidee plus bas (pluie et AWAY prioritaires) ---- */
	  if (g_fenetre_event)
	  {
	      g_fenetre_event = 0;
	      g_fenetre_demande = !g_fenetre_demande;
	      uart_print(g_fenetre_demande ? "[FENETRE] Demande d'ouverture\r\n"
	                                   : "[FENETRE] Demande de fermeture\r\n");
	  }

	  /* ---- Toutes les 2 s : mesures, decisions, supervision ---- */
	  if (g_acq_event)
	  {
	      g_acq_event = 0;

	      /* a) Mesures. Un seul ADC : on lit les deux canaux l'un apres l'autre */
	      g_gaz_brut   = adc_lire(ADC_CHANNEL_0);     // PA0 : MQ-2
	      g_pluie_brut = adc_lire(ADC_CHANNEL_1);     // PA1 : capteur de pluie

	      /* Temperature : la conversion du TMP102 dure 15 ms max (datasheet 5.5),
	         on attend 20 ms. Bloquant mais court (1 % du temps) et hors interruption */
	      if (g_tmp102_ok)
	      {
	          int16_t brut;
	          tmp102_declencher_mesure();
	          HAL_Delay(20);
	          if (tmp102_lire_brut(&brut) == HAL_OK)
	          {
	              g_temp_centi  = tmp102_convertir_centi(brut);
	              g_temp_valide = 1;
	          }
	          else
	          {
	              uart_print("[TMP102] Erreur de lecture I2C\r\n");
	          }
	      }

	      /* b) Gaz : hysteresis (2500 / 2000) + 2 mesures de confirmation.
	         Ne depend pas du mode : la securite gaz est toujours active */
	      if (g_gaz_brut < GAZ_SEUIL_DEFAUT)
	      {
	          /* valeur impossible pour un capteur qui marche : on le signale
	             sans couper une alarme eventuellement en cours */
	          g_gaz_compteur = 0;
	          uart_print("[GAZ] DEFAUT capteur (valeur anormalement basse)\r\n");
	      }
	      else if (!g_alarme_gaz)
	      {
	          if (g_gaz_brut > GAZ_SEUIL_HAUT)
	          {
	              g_gaz_compteur++;
	              if (g_gaz_compteur >= GAZ_NB_CONFIRMATION)
	              {
	                  g_alarme_gaz = 1;
	                  g_gaz_compteur = 0;
	                  uart_print("[ALARME] GAS ALERT !\r\n");
	              }
	          }
	          else
	          {
	              g_gaz_compteur = 0;               // serie interrompue
	          }
	      }
	      else
	      {
	          if (g_gaz_brut < GAZ_SEUIL_BAS)
	          {
	              g_gaz_compteur++;
	              if (g_gaz_compteur >= GAZ_NB_CONFIRMATION)
	              {
	                  g_alarme_gaz = 0;
	                  g_gaz_compteur = 0;
	                  uart_print("[GAZ] Fin d'alerte : air revenu a la normale\r\n");
	              }
	          }
	          else
	          {
	              g_gaz_compteur = 0;
	          }
	      }

	      /* b') Pluie : meme principe que le gaz */
	      if (!g_pluie)
	      {
	          if (g_pluie_brut > PLUIE_SEUIL_HAUT)
	          {
	              g_pluie_compteur++;
	              if (g_pluie_compteur >= PLUIE_NB_CONFIRMATION)
	              {
	                  g_pluie = 1;
	                  g_pluie_compteur = 0;
	                  uart_print("[PLUIE] Pluie detectee\r\n");
	              }
	          }
	          else
	          {
	              g_pluie_compteur = 0;
	          }
	      }
	      else
	      {
	          if (g_pluie_brut < PLUIE_SEUIL_BAS)
	          {
	              g_pluie_compteur++;
	              if (g_pluie_compteur >= PLUIE_NB_CONFIRMATION)
	              {
	                  g_pluie = 0;
	                  g_pluie_compteur = 0;
	                  uart_print("[PLUIE] Fin de pluie\r\n");
	              }
	          }
	          else
	          {
	              g_pluie_compteur = 0;
	          }
	      }

	      /* c) Supervision : ligne d'etat envoyee apres les decisions,
	         pour afficher l'etat a jour */
	      char temp[12] = "N/A";
	      if (g_temp_valide)
	          temp_texte(temp, sizeof(temp), g_temp_centi);

	      char msg[160];
	      snprintf(msg, sizeof(msg),
	               "[ETAT] mode=%s | temp=%s C | gaz=%u | alarme_gaz=%s | intrusion=%s | pluie=%u (%s) | fenetre=%s\r\n",
	               (g_mode == MODE_HOME) ? "HOME" : "AWAY",
	               temp,
	               (unsigned int)g_gaz_brut,
	               g_alarme_gaz ? "ON" : "OFF",
	               g_alarme_intrusion ? "ON" : "OFF",
	               (unsigned int)g_pluie_brut,
	               g_pluie ? "OUI" : "NON",
	               g_fenetre_ouverte ? "OPEN" : "CLOSED");
	      uart_print(msg);
	  }

	  /* ---- Sorties, mises a jour a chaque tour ---- */

	  /* LED d'alarme : clignote si au moins une alarme est active.
	     On compare l'heure au lieu d'attendre, pour ne pas bloquer la boucle */
	  if (g_alarme_intrusion || g_alarme_gaz)
	  {
	      uint32_t maintenant = HAL_GetTick();
	      if ((maintenant - g_dernier_clignotement_ms) >= PERIODE_CLIGNOTEMENT_MS)
	      {
	          g_dernier_clignotement_ms = maintenant;
	          HAL_GPIO_TogglePin(LED_INTRUSION_GPIO_Port, LED_INTRUSION_Pin);
	      }
	  }
	  else
	  {
	      /* eteinte d'office, sinon elle peut rester allumee a la fin de l'alarme */
	      HAL_GPIO_WritePin(LED_INTRUSION_GPIO_Port, LED_INTRUSION_Pin, GPIO_PIN_RESET);
	  }

	  maj_buzzer();

	  /* Fenetre ouverte seulement si l'utilisateur le demande, qu'il ne pleut pas
	     et qu'on est en HOME. Recalcule a chaque tour, le servo ne bouge
	     que si le resultat change */
	  uint8_t fenetre_voulue = g_fenetre_demande && !g_pluie && (g_mode == MODE_HOME);
	  fenetre_commander(fenetre_voulue);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Envoie un texte sur l'UART (port COM virtuel du ST-LINK).
   Bloquant (~87 us par caractere a 115200) : a n'appeler que depuis la boucle */
static void uart_print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 100);
}

/* Appelee par le HAL a chaque debordement d'un timer.
   Seul TIM2 est en interruption : toutes les 2 s on demande une acquisition */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);   // temoin de la base de temps
        g_acq_event = 1;
    }
}

/* Appelee pour toutes les lignes EXTI. GPIO_Pin indique quelle broche a declenche.
   On reste court : un drapeau, rien d'autre */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)
    {
        /* anti-rebond : on regarde l'heure au lieu d'attendre
           (pas de HAL_Delay dans une interruption) */
        uint32_t maintenant = HAL_GetTick();

        if ((maintenant - g_dernier_appui_ms) >= anti_rebond_ms)
        {
            g_dernier_appui_ms = maintenant;
            g_nb_appuis++;
            g_mode = (g_mode == MODE_HOME) ? MODE_AWAY : MODE_HOME;
            g_mode_event = 1;
        }
    }
    else if (GPIO_Pin == PIR_Pin)
    {
        /* sortie electronique, pas de rebond */
        g_nb_pir++;
        g_pir_event = 1;
    }
    else if (GPIO_Pin == BTN_FENETRE_Pin)
    {
        uint32_t maintenant = HAL_GetTick();
        if ((maintenant - g_dernier_appui_fenetre_ms) >= anti_rebond_ms)
        {
            g_dernier_appui_fenetre_ms = maintenant;
            g_fenetre_event = 1;
        }
    }
}

/* Buzzer commun aux deux alarmes.
   On compare ce qu'on veut a ce qui est, et on n'agit qu'au changement :
   le HAL refuse un Start sur un canal deja demarre */
static void maj_buzzer(void)
{
    static uint8_t buzzer_actif = 0;     // garde sa valeur d'un appel a l'autre
    uint8_t buzzer_voulu = (g_alarme_intrusion || g_alarme_gaz);

    if (buzzer_voulu && !buzzer_actif)
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);    // PWM 2 kHz sur PC7
        buzzer_actif = 1;
    }
    else if (!buzzer_voulu && buzzer_actif)
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
        buzzer_actif = 0;
    }
}

/* Place la fenetre dans la position demandee.
   Le PWM du servo tourne en permanence, on change juste la largeur
   d'impulsion (CCR1). Prise en compte a la periode suivante (preload) */
static void fenetre_commander(uint8_t ouvrir)
{
    if (ouvrir == g_fenetre_ouverte)
        return;                          // deja dans la bonne position

    g_fenetre_ouverte = ouvrir;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ouvrir ? SERVO_OPEN_US : SERVO_CLOSED_US);
    uart_print(ouvrir ? "[FENETRE] WINDOW OPEN\r\n" : "[FENETRE] WINDOW CLOSED\r\n");
}

/* Lit un canal de l'ADC1 (conversion unique, polling).
   On choisit d'abord le canal (le multiplexeur ne relie qu'une entree a la fois),
   puis start / attente de fin (~20 us) / lecture / stop.
   Renvoie 0 si la conversion n'aboutit pas, ce qui sera vu comme un defaut */
static uint16_t adc_lire(uint32_t canal)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = canal;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;   // sortie des capteurs a forte impedance
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    uint16_t valeur = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        valeur = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return valeur;
}

/* Ecrit une temperature en centiemes sous la forme "25.06" ou "-3.50".
   On reste en entiers car printf de nano.specs n'affiche pas les float */
static void temp_texte(char *buf, size_t taille, int32_t centi)
{
    int32_t absolu = (centi < 0) ? -centi : centi;
    snprintf(buf, taille, "%s%ld.%02ld", (centi < 0) ? "-" : "",
             (long)(absolu / 100), (long)(absolu % 100));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
