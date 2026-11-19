/*
 * tmp102.c
 *
 *  Created on: Oct 16, 2026
 *      Author: christophe
 *
 *  Le STM32 est maitre du bus, le TMP102 est esclave a l'adresse 0x48.
 *  Regle du capteur (datasheet 6.3.5) : dans une ecriture, le premier octet
 *  apres l'adresse est le numero du registre vise. Une lecture renvoie
 *  le dernier registre designe.
 */

#include "tmp102.h"
#include "i2c.h"      // donne acces a hi2c1, cree par CubeMX

#define TMP102_TIMEOUT_MS   100U    // au-dela, on abandonne l'echange


/* Passe le capteur en mode one-shot : il ne mesure plus en continu
   et attend qu'on lui demande une mesure.
   Trame : START | 0x90 | 0x01 | 0x61 | 0xA0 | STOP
   Si le capteur est absent, il ne repond pas a l'adresse (NACK)
   et la fonction renvoie une erreur */
HAL_StatusTypeDef tmp102_init(void)
{
    uint8_t trame[3] = { TMP102_REG_CONFIG, TMP102_CFG1_SHUTDOWN, TMP102_CFG2 };
    return HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, trame, 3, TMP102_TIMEOUT_MS);
}


/* Lance une mesure unique (bit OS a 1). La conversion dure 10 ms
   typique et 15 ms max (datasheet 5.5), il faut attendre avant de lire.
   Trame : START | 0x90 | 0x01 | 0xE1 | 0xA0 | STOP */
HAL_StatusTypeDef tmp102_declencher_mesure(void)
{
    uint8_t trame[3] = { TMP102_REG_CONFIG, TMP102_CFG1_ONESHOT, TMP102_CFG2 };
    return HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, trame, 3, TMP102_TIMEOUT_MS);
}


/* Lit la temperature brute (12 bits signes).
   Il faut d'abord replacer le pointeur sur le registre 0x00, car la commande
   de mesure l'a laisse sur le registre de configuration.
   Trame 1 : START | 0x90 | 0x00 | STOP
   Trame 2 : START | 0x91 | octet 1 | ACK | octet 2 | NACK | STOP */
HAL_StatusTypeDef tmp102_lire_brut(int16_t *brut)
{
    uint8_t pointeur = TMP102_REG_TEMP;
    uint8_t donnees[2];

    if (HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, &pointeur, 1, TMP102_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, donnees, 2, TMP102_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    /* Les 12 bits sont cales a gauche, les 4 derniers bits valent 0 (Tables 6-8, 6-9).
       ex : 0x19 0x00 -> 0x1900 -> 0x190 = 400 */
    uint16_t valeur = ((donnees[0] << 8) | donnees[1]) >> 4;

    /* Complement a deux sur 12 bits : si le bit 11 vaut 1 la temperature
       est negative, on retire 2^12 */
    if (valeur & 0x0800)
        *brut = (int16_t)valeur - 4096;
    else
        *brut = (int16_t)valeur;

    return HAL_OK;
}



/* 1 unite = 0,0625 C = 6,25 centiemes, d'ou x 625 / 100.
   On multiplie avant de diviser pour garder la precision, en 32 bits
   pour eviter le depassement. ex : 400 -> 2500 (25,00 C) */
int32_t tmp102_convertir_centi(int16_t brut)
{
    return ((int32_t)brut * 625) / 100;
}
