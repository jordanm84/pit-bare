#include <stdint.h>
#include <stdbool.h>
#include "MK64F12.h"

#define PIN6_MASK            (1u << 6)
#define PIN21_MASK           (1u << 21)
#define PIN22_MASK           (1u << 22)
#define PIN26_MASK           (1u << 26)

static bool flashing = true;

void leds_init(void);
void blue_toggle(void);
void red_toggle(void);
void green_toggle(void);
void sw2_init(void);
void PIT_init(void);
void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PIT2_IRQHandler(void);
void PORTC_IRQHandler(void);

int main(void)
{
    leds_init();
    sw2_init();
    PIT_init();
    while (true) {
    }
    /* Should not reach this point */
    return 0;
}

void leds_init(void) {
    /* Enable the clock to PORTs B and E */
    SIM_SCGC5 |= (SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK);

    /* Select the GPIO function (Alternative 1) for pin 21 of PORT B */
    PORTB_PCR21 &= ~PORT_PCR_MUX_MASK;
    PORTB_PCR21 |= (1u << PORT_PCR_MUX_SHIFT);
    /* Select the GPIO function (Alternative 1) for pin 22 of PORT B */
    PORTB_PCR22 &= ~PORT_PCR_MUX_MASK;
    PORTB_PCR22 |= (1u << PORT_PCR_MUX_SHIFT);
    /* Select the GPIO function (Alternative 1) for pin 26 of PORT E */
    PORTE_PCR26 &= ~PORT_PCR_MUX_MASK;
    PORTE_PCR26 |= (1u << PORT_PCR_MUX_SHIFT);

    /* Set the data direction for pin 21 of PORT B to output */
    GPIOB_PDDR |= PIN21_MASK;
    /* Set the data direction for pin 22 of PORT B to output */
    GPIOB_PDDR |= PIN22_MASK;
    /* Set the data direction for pin 26 of PORT E to output */
    GPIOE_PDDR |= PIN26_MASK;

    /* LEDs off initially */
    GPIOB_PDOR |= PIN21_MASK;
    GPIOB_PDOR |= PIN22_MASK;
    GPIOE_PDOR |= PIN26_MASK;
}

void blue_toggle(void) {
    if (flashing) {
        GPIOB_PDOR ^= PIN21_MASK;
    }
}

void red_toggle(void) {
    if (flashing) {
        GPIOB_PDOR ^= PIN22_MASK; 
    }
}

void green_toggle(void) {
    if(flashing) {
        GPIOE_PDOR ^= PIN26_MASK; 
    }
}


void sw2_init(void) {
    /* Enable the clock to PORT C */
    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

    /* Select the GPIO function (Alternative 1) for pin 6 of PORT C */
    PORTC_PCR6 &= ~PORT_PCR_MUX_MASK;
    PORTC_PCR6 |= (1u << PORT_PCR_MUX_SHIFT);
    /* Select interrupt trigger on rising edge - button release */
    PORTC_PCR6 &= ~PORT_PCR_IRQC_MASK;
    PORTC_PCR6 |= (9u << PORT_PCR_IRQC_SHIFT);
    /* Set the data direction for pin 6 of PORT C to input */
    GPIOC_PDDR &= ~PIN6_MASK;
    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(PORTC_IRQn);
}

void PIT_init(void) {
/* Open the clock gate to the PIT */
    SIM_SCGC6 |= (1u << 23);
    /* Enable the clock for the PIT timers. Continue to run in debug mode */
    PIT_MCR_REG(PIT) = 0u;

    /* Period p = 0.5 s, bus clock f = 60 MHz, v = pf - 1 */ 
    PIT_LDVAL_REG(PIT, 0) = 29999999;
    /* Enable interrupt on timeout */
    PIT_TCTRL_REG(PIT, 0) |= PIT_TCTRL_TIE_MASK;
    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(PIT0_IRQn);

    /* Period p = 0.2 s, bus clock f = 60 MHz, v = pf - 1 */ 
    PIT_LDVAL_REG(PIT, 1) = 11999999;
    /* Enable interrupt on timeout */
    PIT_TCTRL_REG(PIT, 1) |= PIT_TCTRL_TIE_MASK;
    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(PIT1_IRQn);

    /* Period p = 0.33... s, bus clock f = 60 MHz, v = pf - 1 */ 
    PIT_LDVAL_REG(PIT, 2) = 19999999;
    /* Enable interrupt on timeout */
    PIT_TCTRL_REG(PIT, 2) |= PIT_TCTRL_TIE_MASK;
    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(PIT2_IRQn);

    /* Start the timer running */
    PIT_TCTRL_REG(PIT, 0) |= PIT_TCTRL_TEN_MASK;
    /* Start the timer running */
    PIT_TCTRL_REG(PIT, 1) |= PIT_TCTRL_TEN_MASK;
    /* Start the timer running */
    PIT_TCTRL_REG(PIT, 2) |= PIT_TCTRL_TEN_MASK;
}

void PIT0_IRQHandler(void) {
    /* Call the main handler function */
    blue_toggle();
    /* Clear the timer interrupt flag to allow further timer interrupts */
    PIT_TFLG_REG(PIT,0) |= PIT_TFLG_TIF_MASK;
}


void PIT1_IRQHandler(void) {
    /* Call the main handler function */
    red_toggle();
    /* Clear the timer interrupt flag to allow further timer interrupts */
    PIT_TFLG_REG(PIT, 1) |= PIT_TFLG_TIF_MASK;
}

void PIT2_IRQHandler(void) {
    /* Call the main handler function */
    green_toggle();
    /* Clear the timer interrupt flag to allow further timer interrupts */
    PIT_TFLG_REG(PIT, 2) |= PIT_TFLG_TIF_MASK;
}

void PORTC_IRQHandler(void) {
    if (PORTC_ISFR & PIN6_MASK) {
        /* Toggle the flashing state */
        flashing = !flashing;
        /* Clear the interrupt flag */
        PORTC_ISFR |= PIN6_MASK;
    }
}
