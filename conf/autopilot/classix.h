#ifndef CONFIG_CLASSIX_H
#define CONFIG_CLASSIX_H

/* Master oscillator freq.       */
#define FOSC (12000000) 
/* PLL multiplier                */
#define PLL_MUL (5)         
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL) 
/* Peripheral bus speed divider  */
#define PBSD 2    
/* Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD) 

#ifdef FBW
#define LED_1_BANK 1
#define LED_1_PIN 24

#define LED_2_BANK 1
#define LED_2_PIN 31


/* PPM : rc rx on P0.16 : FBW_RC1 connector */
//#define PPM_PINSEL PINSEL1
//#define PPM_PINSEL_VAL 0x03
//#define PPM_PINSEL_BIT 0


/* PPM : rc rx on P0.6 : FBW_RC0 connector */
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12

/* SERVOS : 4017 on FBW_RC0 connector */

/* MAT0.1 on P0.5 */
#define SERVO_CLOCK_PIN 5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10

/* reset on P1.25 */
#define SERVO_RESET_PIN 25
#define SERVO_RESET_PINSEL PINSEL2
#define SERVO_RESET_PINSEL_VAL 0
#define SERVO_RESET_PINSEL_BIT 3

/* ADCs : supply on AD0.6 ( P0.4 ) */
#define USE_AD0_6
#define BRICOLAGE_ADC


#endif /* FBW */

#ifdef AP
#define LED_1_BANK 1
#define LED_1_PIN 18

#define LED_2_BANK 1
#define LED_2_PIN 19
#endif /* AP */

#endif /* CONFIG_CLASSIX_H */
