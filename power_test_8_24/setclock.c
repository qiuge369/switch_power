/*
 * setclock.c
 *
 *  Created on: 2020年8月28日
 *      Author: pc
 */
#include "setclock.h"
uint16_t SetVCore (uint8_t level);
static uint16_t SetVCoreUp (uint8_t level);
/**
 *  该函数用于把 MCLK,SMCLK倍频到 12MHZ ，ACLK不变化是32.768KHz
 *
 *
 **/
void SetClock_MCLK12MHZ_SMCLK24MHZ_ACLK32_768K(void)
{
//    volatile unsigned int i;

    SetVCore(PMMCOREV_1);                     // Set VCore = 1.6V for 12MHz clock

    UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_6;                      // Select DCO range 24MHz operation
    UCSCTL2 = FLLD_1 + 732;                   // Set DCO Multiplier for 12MHz
                                              // (N + 1) * FLLRef = Fdco
                                              // (374 + 1) * 32768 = 12MHz
                                              // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                  // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 6xx
    // UG for optimization.
    // 32 x 32 x 12 MHz / 32,768 Hz = 375000 = MCLK cycles for DCO to settle
    __delay_cycles(375000);

    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                              // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
}
uint16_t SetVCore (uint8_t level)
{
  uint16_t actlevel;
  uint16_t status = 0;
  level &= PMMCOREV_3;                       // Set Mask for Max. level
  actlevel = (PMMCTL0 & PMMCOREV_3);         // Get actual VCore

  while (((level != actlevel) && (status == 0)) || (level < actlevel))      /*// step by step increase or decrease*/
  {
    if (level > actlevel)
      status = SetVCoreUp(++actlevel);
  }
  return status;
}

static uint16_t SetVCoreUp (uint8_t level)
  {
    uint16_t PMMRIE_backup,SVSMHCTL_backup;

    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;

    // Disable dedicated Interrupts to prevent that needed flags will be cleared
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVSMHDLYIE | SVSMLDLYIE | SVMLVLRIE | SVMHVLRIE | SVMHVLRPE);
    SVSMHCTL_backup = SVSMHCTL;
    PMMIFG &= ~(SVMHIFG | SVSMHDLYIFG);
    // Set SVM highside to new level and check if a VCore increase is possible
    SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);
    // Wait until SVM highside is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0);
    // Check if a VCore increase is possible
    if ((PMMIFG & SVMHIFG) == SVMHIFG){       //-> Vcc is to low for a Vcore increase
      // recover the previous settings
      PMMIFG &= ~SVSMHDLYIFG;
      SVSMHCTL = SVSMHCTL_backup;
      // Wait until SVM highside is settled
      while ((PMMIFG & SVSMHDLYIFG) == 0);
      // Clear all Flags
      PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
      // backup PMM-Interrupt-Register
      PMMRIE = PMMRIE_backup;

      // Lock PMM registers for write access
      PMMCTL0_H = 0x00;
      return PMM_STATUS_ERROR;            // return: voltage not set
    }
    // Set also SVS highside to new level       //-> Vcc is high enough for a Vcore increase
    SVSMHCTL |= (SVSHRVL0 * level);
    // Set SVM low side to new level
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level);
    // Wait until SVM low side is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG | SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait until new level reached
    if (PMMIFG & SVMLIFG)
      while ((PMMIFG & SVMLVLRIFG) == 0);
    // Set also SVS/SVM low side to new level
    PMMIFG &= ~SVSMLDLYIFG;
    SVSMLCTL |= SVSLE | (SVSLRVL0 * level);
    // wait for lowside delay flags
    while ((PMMIFG & SVSMLDLYIFG) == 0);

    // Disable SVS/SVM Low
    // Disable full-performance mode to save energy
    SVSMLCTL &= ~(_HAL_PMM_SVSLE + _HAL_PMM_SVMLE + _HAL_PMM_SVSFP + _HAL_PMM_SVMFP);
    SVSMHCTL &= ~(_HAL_PMM_SVSFP + _HAL_PMM_SVMFP);

    // Clear all Flags
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
    // backup PMM-Interrupt-Register
    PMMRIE = PMMRIE_backup;

    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
    return PMM_STATUS_OK;                               // return: OK
  }
