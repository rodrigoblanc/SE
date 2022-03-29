#include "MKL46Z4.h"
#include "lcd.h"

// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc


int hit = 0; int miss = 0; int system_status = 0; 
int bt1_status = 0; int bt2_status = 0;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

// RIGHT_SWITCH (SW1) = PTC3
void bt1_ini()
{
  SIM->COPC = 0;             
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;   

  PORTC->PCR[3] |= PORT_PCR_MUX(1); 
  PORTC->PCR[3] |= PORT_PCR_PE_MASK;
  PORTC->PCR[3] |= PORT_PCR_PS_MASK;

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); 
  NVIC_SetPriority(31, 0);  
  NVIC_EnableIRQ(31);    
}

// LEFT_SWITCH (SW2) = PTC12
void bt2_ini()
{
  SIM->COPC = 0;             
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;   

  PORTC->PCR[12] |= PORT_PCR_MUX(1); 
  PORTC->PCR[12] |= PORT_PCR_PE_MASK;
  PORTC->PCR[12] |= PORT_PCR_PS_MASK;

  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); 
  NVIC_SetPriority(31, 0);  
  NVIC_EnableIRQ(31);    
}

void PORTDIntHandler(void) {
  int pressed_switch = PORTC->ISFR;
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ

    // SW1
  if(pressed_switch == (0x8)) {
    ++bt1_status;
  }

  // SW2
  if(pressed_switch == (0x1000)) {
    ++bt2_status;
  }

  system_status = bt1_status + bt2_status;
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

// LED_GREEN = PTD5
void led_green_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

void led_green_on(void)
{
  GPIOD->PCOR |= (1 << 5);
}

void led_green_off(void) {
  GPIOD->PSOR |= (1 << 5);
}


// LED_RED = PTE29
void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR |= (1 << 29);
}

void led_red_on(void)
{
  GPIOE->PCOR |= (1 << 29);
}

void led_red_off(void) {
  GPIOE->PSOR |= (1 << 29);
}

// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

// Hit condition: (else, it is a miss)
// - Left switch matches red light
// - Right switch matches green light

int main(void)
{

  irclk_ini(); // Enable internal ref clk to use by LCD

  lcd_ini();
  lcd_display_dec(666);
  leds_ini();
  bt1_ini();
  bt2_ini();
  
  lcd_display_time(00,00);
  // 'Random' sequence :-)
  volatile unsigned int sequence = 0x32B14D98,
    index = 0;

  while (index < 32) {
    if (sequence & (1 << index)) { //odd
      //
      // Switch on green led
      // [...]
      //
      led_green_on();
      led_red_off();
      while (!system_status);
      if (bt1_status) {
        hit++;
      } else {
        miss++;
      }
      bt1_status=0;
      bt2_status=0;
    } else { //even
      //
      // Switch on red led
      // [...]
      //
      led_red_on();
      led_green_off();
      while (!system_status);
      if (bt2_status) {
        hit++;
      } else {
        miss++;
      }
      bt1_status=0;
      bt2_status=0;
    }
    
    system_status = 0;
    lcd_display_time(hit,miss);
    index++;
    // [...]
  }

  // Stop game and show blinking final result in LCD: hits:misses
  // [...]
  //

  while (1) {
    LCD->AR |= LCD_AR_BLINK(1);
    LCD->AR |= LCD_AR_BRATE(0xAA);
    lcd_display_time(hit, miss);
  }

  return 0;
}
