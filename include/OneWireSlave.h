#ifndef OneWireSlave_h
#define OneWireSlave_h


#include <avr/io.h>
#include <avr/interrupt.h>

// OW_PORT Pin 7  - PB2

//OW Pin
#define OW_PORT PORTB //1 Wire Port
#define OW_PIN PINB //1 Wire Pin as number
#define OW_PORTN (1<<PINB2)  //Pin as bit in registers
#define OW_PINN (1<<PINB2)
#define OW_DDR DDRB  //pin direction register
#define SET_LOW OW_DDR|=OW_PINN;OW_PORT&=~OW_PORTN;  //set 1-Wire line to low
#define RESET_LOW {OW_DDR&=~OW_PINN;}  //set 1-Wire pin as input
//Pin interrupt 
#define EN_OWINT {GIMSK|=(1<<INT0);GIFR|=(1<<INTF0);}  //enable interrupt 
#define DIS_OWINT  GIMSK&=~(1<<INT0);  //disable interrupt
#define SET_RISING MCUCR=(1<<ISC01)|(1<<ISC00);  //set interrupt at rising edge
#define SET_FALLING MCUCR=(1<<ISC01); //set interrupt at falling edge
#define CHK_INT_EN (GIMSK&(1<<INT0))==(1<<INT0) //test if interrupt enabled
#define PIN_INT ISR(INT0_vect)  // the interrupt service routine
//Timer Interrupt
#define EN_TIMER {TIMSK |= (1<<TOIE0); TIFR|=(1<<TOV0);} //enable timer interrupt
#define DIS_TIMER TIMSK  &= ~(1<<TOIE0); // disable timer interrupt
#define TCNT_REG TCNT0  //register of timer-counter
#define TIMER_INT ISR(TIM0_OVF_vect) //the timer interrupt service routine
//States / Modes
#define OWT_MIN_RESET 51
#define OWT_RESET_PRESENCE 4
#define OWT_PRESENCE 20 
#define OWT_READLINE 3 //for fast master, 4 for slow master and long lines
#define OWT_LOWTIME 3 //for fast master, 4 for slow master and long lines 
#define OWM_SLEEP 0  //Waiting for next reset pulse
#define OWM_RESET 1  //Reset pulse received 
#define OWM_PRESENCE 2  //sending presence pulse
#define OWM_READ_COMMAND 3 //read 8 bit of command
#define OWM_SEARCH_ROM 4  //SEARCH_ROM algorithms
#define OWM_MATCH_ROM 5  //test number
#define OWM_READ_SCRATCHPAD 6
#define OWM_WRITE_SCRATCHPAD 7
#define OWM_CHK_RESET 8  //waiting of rising edge from reset pulse
#define OWM_READ_PIO 9
#define OWM_WRITE_PIO 10
#define OWM_READ_MEMORY 11
#define OWM_WRITE_MEMORY 12
#define OWM_READ 13 //  read from master
#define OWM_WRITE 14  //  write to master

//Write a bit after next falling edge from master
//its for sending a zero as soon as possible
#define OWW_NO_WRITE 2
#define OWW_WRITE_1 1
#define OWW_WRITE_0 0

class OneWireSlave {
  private:
  void beginReceiveBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)());
  void beginWriteBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)());
  public:
  void begin(void (*onCommand)(uint8_t), uint8_t* owId);
  uint8_t crc8(const uint8_t* data, uint8_t numBytes);
  void read(uint8_t* buffer, uint8_t numBytes, void (*complete)());
  void write(uint8_t* buffer, uint8_t numBytes, void (*complete)());
  static void reset();
};

#endif

