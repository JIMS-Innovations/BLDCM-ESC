/**
 * @file commutation.h
 * @author Jesutofunmi Kupoluyi
 * @brief  MOSFET bridge driver header file
 * @version 0.1
 * @date 2024-01-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __COMMUTATION_H__
#define __COMMUTATION_H__

/* Include main definition header */
#include "main.h"

/* Function protypes */
void commutate();
void inv_commutate();
void stop();
void set_speed(uint8_t);

void AH_BL();
void AH_CL();
void BH_CL();
void BH_AL();
void CH_AL();
void CH_BL();

void BEMF_A_FALLING();
void BEMF_A_RISING();
void BEMF_B_FALLING();
void BEMF_B_RISING();
void BEMF_C_FALLING();
void BEMF_C_RISING();


#endif/*__COMMUTATION_H__*/