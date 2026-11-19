/*
 * tmp102.h
 *
 *  Created on: Oct 16, 2026
 *      Author: christophe
 *
 *  Driver minimal du capteur de temperature TMP102 (Texas Instruments)
 *  sur le bus I2C1 du STM32.
 *
 *  Utilisation :
 *    1. tmp102_init()               une fois au demarrage
 *    2. tmp102_declencher_mesure()  lance une conversion (10 a 15 ms)
 *    3. tmp102_lire_brut(&brut)     recupere le resultat, au moins 15 ms apres
 *    4. tmp102_convertir_centi()    transforme la valeur brute en centiemes de C
 *
 *  References : datasheet TMP102, SBOS397I
 */

#ifndef TMP102_H
#define TMP102_H

#include "main.h"     // types HAL (HAL_StatusTypeDef, uint8_t, int16_t...)

/* Adresse I2C sur 7 bits : 1001000 = 0x48 quand ADD0 est a la masse (Table 6-4).
   Le HAL veut l'adresse decalee d'un bit, il ajoute lui-meme le bit R/W :
   0x90 en ecriture, 0x91 en lecture */
#define TMP102_ADDR           (0x48 << 1)

/* Numeros des registres internes, envoyes en premier octet (Table 6-7) */
#define TMP102_REG_TEMP       0x00    // temperature, lecture seule
#define TMP102_REG_CONFIG     0x01    // configuration, lecture / ecriture

/* Octets ecrits dans le registre de configuration (Tables 6-10 et 6-11) */
#define TMP102_CFG1_SHUTDOWN  0x61    // 0110 0001 : SD = 1, le capteur dort entre deux mesures
#define TMP102_CFG1_ONESHOT   0xE1    // 1110 0001 : OS = 1 et SD = 1, une seule mesure puis veille
#define TMP102_CFG2           0xA0    // 1010 0000 : valeur par defaut, non modifiee

/* Renvoient HAL_OK si l'echange I2C s'est bien passe (ACK recu a chaque octet) */
HAL_StatusTypeDef tmp102_init(void);
HAL_StatusTypeDef tmp102_declencher_mesure(void);
HAL_StatusTypeDef tmp102_lire_brut(int16_t *brut);    // resultat range dans *brut

/* 1 unite brute = 0,0625 C ; renvoie des centiemes de degre (400 -> 2500 = 25,00 C) */
int32_t           tmp102_convertir_centi(int16_t brut);

#endif
