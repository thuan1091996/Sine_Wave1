/*******************************************************
This program was created by the
CodeWizardAVR V3.12 Advanced
Automatic Program Generator
© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : Sine_Wave1
Version : v1.0
Date    : 10/6/2018
Author  : Tran Minh Thuan
Supported by: Nguyen Duc Quyen
Company : Viet Mold Machine
Comments: 


Chip type               : ATmega8
Program type            : Application
AVR Core Clock frequency: 8.000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/
#include <mega8.h>
#include <stdio.h>
#include <alcd.h>                   // Alphanumeric LCD functions

#define F_CPU 8000000
#define PB0 PORTB.0
#define PB2 PORTB.2
unsigned int Freq_1=0;                //Default frequency is 400Hz   
unsigned int Tick_freq1=0;            //Variable to create signal 1
unsigned int signal_time1=0,Freq_1_duty=0;
unsigned int Freq_2_duty=70;          //Default duty cycle is 50%
unsigned int T_delay=0;
unsigned long Freq_load,Freq_2;
unsigned char Chuoi[16];
// Timer2 overflow interrupt service routine
interrupt [TIM2_OVF] void timer2_ovf_isr(void)
{
  Tick_freq1++;
  TCNT2=195;        //255-(176) =80 10us, but calib to 195 for accuracy 
}
interrupt [TIM1_OVF] void timer1_ovf_isr(void)
{
  PB2^=0x01; 
}
void Timer1_Init(void);
void Timer2_Init(void);
void GPIOB_Init(void);
void Signal_freq1(void);
void ADC_Init(void);
unsigned int ADC_Read(unsigned int channel);
void Update_signals(void);
void Interrupt_Enable(void);
void Display(void);
void Cal_Mean(void);
void main(void)
{
    GPIOB_Init();
    Timer1_Init();
    Timer2_Init();  
    ADC_Init(); 
12312312    lcd_init(16);
    Interrupt_Enable();
    #asm("sei") // Global enable interrupts
    while (1)
      {   
        Update_signals();
        Signal_freq1(); 
        Display();
      }
}
/*  Interrupt enable
 *  Enable overflow interrupt for timer1,2
 */
void Interrupt_Enable(void){
    TIMSK|=(1<<TOIE2)|(1<<TOIE1) | (0<<TOIE0);
}
void GPIOB_Init(void){
    DDRB|=0xFF;         //PB1 output 
    DDRD|=0xFF;
    DDRC=0; 
}
/*  Timer/Counter 1 initialization
 *  ICR1 contains top value (control frequency, but timer1_freq = 2*signal2 frequency)
 *  OCA1 create signal 2 
 *  Clock source: System Clock 
 *  Mode 14: Fast PWM go from 0 to value in ICR1   
 *  Prescaler:1
 *  Change the function by 
 *          +Change: Freq_load (ISR1), 2*signal frequency 
            +TCCR1A WGM: Operation mode
            +Freq_2:     Duty cycle of signal frequency
 */
void Timer1_Init(void){
    Freq_load=800;                    //10Khz, F_timer2 = 2*F_signal2  (Timer1 OVR -> 50Khz)
    Freq_2=Freq_load*Freq_2_duty/100;          //Calculate new compare value (Channel A)
    TCCR1A =(1<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0);        
    TCCR1A|=(1<<WGM11)|(0<<WGM10); TCCR1B|=(1<<WGM13) | (1<<WGM12); //Mode 14
    TCCR1B|=(0<<CS12) | (0<<CS11) | (1<<CS10);                      //Prescaler 1  
    OCR1AH=Freq_2>> 8;   OCR1AL=Freq_2& 0xff;   //Load new compare value (control Duty A)
    ICR1H=(Freq_load)>> 8;      ICR1L=(Freq_load)& 0xff;                                      ICR1H=(Freq_load)>> 8;      ICR1L=(Freq_load)& 0xff;
}
/*  Timer/Counter 2 initialization
 *  Mode: Normal top=0xFF
 *  Interrupt every 10us, change frequency by changing value of TCNT2 in Init function and Timer2 ISR
 */
void Timer2_Init(void){    
    ASSR=0<<AS2;                                          //Use clock from system clock
    OCR2=0x00;
    TCCR2=(0<<COM21) | (0<<COM20)| (0<<CS22) | (0<<CS21) | (1<<CS20);  //No prescaler
    TCNT2=195;        //255-(176) =80 10us, but calib to 195 for accuracy 
}
/*  Signal_freq1
 *  Change freq1 by change Freq_1
 */
void Signal_freq1(void){

    if(Tick_freq1>=signal_time1){
        PB0^=0x01;
        Tick_freq1=0; 
    }
}
/* ADC Inititilization
 * ADC 10 bit
 * Prescaler 128
 * Vref is VCC
*/
void ADC_Init(void){
    ADMUX = (1<<REFS0); //V_ref is VCC, left adjust
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);    // prescaler = 128
}
/* Read value from ADC channel
 * Input: Input channel (0-7)
 * Output: 0-1024 corresponding to the value
*/
unsigned int ADC_Read(unsigned int channel)
{
  unsigned int ADCval;
  channel &= 0b00000111;  // AND operation with 7
  ADMUX = (ADMUX & 0xF8)|channel; // clears the bottom 3 bits for choosing channel
  // start single convertion by writing ’1' to ADSC
  ADCSRA |= (1<<ADSC);
  // wait for conversion to complete, ADSC becomes ’0' again
  while(ADCSRA & (1<<ADSC));  
  ADCval = ADCL;
  ADCval = (ADCH << 8) + ADCval;    // ADCH is read so ADC can be updated again
  if(ADCval>=1000) ADCval=1000;
  return (ADCval);
}
void Update_signals(void){
    Cal_Mean();
    signal_time1=100000/(2*Freq_1);
    Freq_2=8*Freq_2_duty;                      //Calculate new compare value (Channel A)
    if(Freq_2>=800) Freq_2=799;
    OCR1AH=Freq_2>> 8;   OCR1AL=Freq_2& 0xff;   //Load new compare value (control Duty A)
}
void Display(void){
        sprintf(Chuoi,"Tan so 1:%04d",signal_time1);
        lcd_gotoxy(0,0);
        lcd_puts(Chuoi); 
        sprintf(Chuoi,"Duty 2:%d",Freq_2_duty);
        lcd_gotoxy(0,1);
        lcd_puts(Chuoi);
}
void Cal_Mean(void){
    unsigned count=0;  
    Freq_1=0;
    Freq_2_duty=0;
    for(count=0;count<50;count++){
        Freq_1+=ADC_Read(0)/2; 
        Freq_2_duty+=ADC_Read(1)/10;
    }
    Freq_1/=50;
    Freq_2_duty/=50;
}