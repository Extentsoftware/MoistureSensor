#include "OneWireSlave.h"

class OneWireSlave {

  uint8_t* owid;

  volatile uint8_t bitp;  //pointer to current Byte
  volatile uint8_t ByteP; //pointer to current Bit
  volatile uint8_t mode; //state
  volatile uint8_t wmode; //if 0 next bit that send the device is  0
  volatile uint8_t actbit; //current
  volatile uint8_t srcount; //counter for search rom
  volatile uint8_t scrc; //CRC calculation
  volatile uint8_t cbuf; //Input buffer for a command
  void(*onCommand_)(uint8_t);
  uint8_t* rxBuffer_;
  uint8_t* txBuffer_;
  uint8_t rxBufferLength_;
  uint8_t txBufferLength_;
  void(*rxCallback_)();
  void(*txCallback_)();

  PIN_INT {
    uint8_t lwmode = wmode; //let this variables in registers
    uint8_t lmode = mode;
    if ((lwmode == OWW_WRITE_0)) {
      SET_LOW;  //if necessary set 0-Bit
      lwmode = OWW_NO_WRITE;
    }
    DIS_OWINT; //disable interrupt, only in OWM_SLEEP mode it is active
    switch (lmode) {
      case OWM_SLEEP:
        TCNT_REG = ~(OWT_MIN_RESET);
        EN_OWINT; //other edges ?
        break;
      //start of reading with falling edge from master, reading closed in timer isr
      case OWM_MATCH_ROM:  //falling edge wait for receive
      case OWM_READ:
      case OWM_READ_COMMAND:
        TCNT_REG = ~(OWT_READLINE); //wait a time for reading
        break;
      case OWM_SEARCH_ROM:   //Search algorithm waiting for receive or send
        if (srcount < 2) { //this means bit or complement is writing,
          TCNT_REG = ~(OWT_LOWTIME);
        } else
          TCNT_REG = ~(OWT_READLINE); //init for read answer of master
        break;
      case OWM_WRITE:
        TCNT_REG = ~(OWT_LOWTIME);
        break;
      case OWM_CHK_RESET:  //rising edge of reset pulse
        SET_FALLING
        TCNT_REG = ~(OWT_RESET_PRESENCE); //waiting for sending presence pulse
        lmode = OWM_RESET;
        break;
    }
    EN_TIMER;
    mode = lmode;
    wmode = lwmode;

  }
  void OneWireSlave::reset(){
    mode = OWM_SLEEP;
  }

  ////////////////////////////////
  // Timer interrupt
  ////////////////////////////////
  TIMER_INT {
    //Ask input line sate
    uint8_t p = ((OW_PIN & OW_PINN) == OW_PINN);
    //Interrupt still active ?
    if (CHK_INT_EN) {
      //maybe reset pulse
      if (p == 0) {
        mode = OWM_CHK_RESET; //wait for rising edge
        SET_RISING;
      }
      DIS_TIMER;
    } else {
      switch (mode) {
        case OWM_RESET:  //Reset pulse and time after is finished, now go in presence state
          mode = OWM_PRESENCE;
          SET_LOW;
          TCNT_REG = ~(OWT_PRESENCE);
          DIS_OWINT;  //No Pin interrupt necessary only wait for presence is done
          break;
        case OWM_SEARCH_ROM:
          RESET_LOW;  //Set low also if nothing send (branch takes time and memory)
          srcount++;  //next search rom mode
          switch (srcount) {
            case 1: wmode = !actbit; //preparation sending complement
              break;
            case 3:
              if (p != (actbit == 1)) { //check master bit
                mode = OWM_SLEEP; //not the same go sleep
              } else {
                bitp = (bitp << 1); //prepare next bit
                if (bitp == 0) {
                  bitp = 1;
                  ByteP++;
                  if (ByteP >= 8) {
                    mode = OWM_SLEEP; //all bits processed
                    break;
                  }
                }
                srcount = 0;
                actbit = (owid[ByteP] & bitp) == bitp;
                wmode = actbit;
              }
              break;
          }
          break;
        case OWM_PRESENCE:
          RESET_LOW;  //Presence is done now wait for a command
          mode = OWM_READ_COMMAND;
          cbuf = 0; bitp = 1; //Command buffer have to set zero, only set bits will write in
          break;
        case OWM_READ_COMMAND:
          if (p) {  //Set bit if line high
            cbuf |= bitp;
          }
          bitp = (bitp << 1);
          if (!bitp) { //8-Bits read
            bitp = 1;
            switch (cbuf) {
              case 0x55: ByteP = 0; mode = OWM_MATCH_ROM; break;
              case 0xF0:  //initialize search rom
                mode = OWM_SEARCH_ROM;
                srcount = 0;
                ByteP = 0;
                actbit = (owid[ByteP] & bitp) == bitp; //set actual bit
                wmode = actbit; //prepare for writing when next falling edge
                break;
              default:
                mode = OWM_SLEEP; //all other commands do nothing
                onCommand_(cbuf);
            }
          }
          break;
        case OWM_MATCH_ROM:
          if (p == ((owid[ByteP]&bitp) == bitp)) { //Compare with ID Buffer
            bitp = (bitp << 1);
            if (!bitp) {
              ByteP++;
              bitp = 1;
              if (ByteP >= 8) {
                mode = OWM_READ_COMMAND; //same? get next command
                cbuf = 0;
                break;
              }
            }
          } else {
            mode = OWM_SLEEP;
          }
          break;
        case OWM_READ:
          if (p) {
            rxBuffer_[ByteP] |= bitp;
          }
          bitp = (bitp << 1);
          if (!bitp) {
            ByteP++;
            bitp = 1;
            if (ByteP == rxBufferLength_) {
              rxCallback_();
  //            mode = OWM_SLEEP;
              break;
            } else rxBuffer_[ByteP] = 0;
          }
          break;
        case OWM_WRITE: //  write to host
          RESET_LOW;
          bitp = (bitp << 1);
          if (!bitp) {
            ByteP++;
            bitp = 1;
            //if (ByteP == 2) txCallback_();
            if (ByteP >= txBufferLength_) {
              txCallback_();
  //            mode = OWM_SLEEP;
              break;
            };
          }
          actbit = (bitp & txBuffer_[ByteP]) == bitp;
          wmode = actbit; //prepare for send firs bit
          break;
      }
    }
    if (mode == OWM_SLEEP) {
      DIS_TIMER;
    }
    if (mode != OWM_PRESENCE)  {
      TCNT_REG = ~(OWT_MIN_RESET - OWT_READLINE); //OWT_READLINE around OWT_LOWTIME
      EN_OWINT;
    }
  }


  void noOpCallback_() {}

  uint8_t OneWireSlave::crc8(const uint8_t* data, uint8_t numBytes)
  {
    uint8_t crc = 0;
    while (numBytes--) {
      uint8_t inByte = *data++;
      for (short i = 8; i; i--) {
        uint8_t mix = (crc ^ inByte) & 0x01;
        crc >>= 1;
        if (mix) crc ^= 0x8C;
        inByte >>= 1;
      }
    }
    return crc;
  }

  void OneWireSlave::begin(void (*onCommand)(uint8_t), uint8_t* owId) {
    owid = owId;
    owid[7] = crc8(owid,7);
    onCommand_ = onCommand;
    mode = OWM_SLEEP;
    wmode = OWW_NO_WRITE;
    OW_DDR &= ~OW_PINN;
    SET_FALLING
    CLKPR=(1<<CLKPCE);
    CLKPR=0; /*8Mhz*/ 
    TIMSK=0;
    GIMSK=(1<<INT0);  /*set direct GIMSK register*/
    TCCR0B=(1<<CS00)|(1<<CS01); /*8mhz /64 couse 8 bit Timer interrupt every 8us*/
    DIS_TIMER;
    sei();
  }

  void OneWireSlave::read(uint8_t* buffer, uint8_t numBytes, void (*complete)()) { //  read from master
    cli();
    OneWireSlave::beginReceiveBytes_(buffer, numBytes, complete);
    sei();
  }

  void OneWireSlave::beginReceiveBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)()) {
    rxBuffer_ = buffer;
    rxBufferLength_ = numBytes;
    rxCallback_ = complete;
    mode = OWM_READ;
    ByteP = 0;
    rxBuffer_[0] = 0;
  }

  void OneWireSlave::write(uint8_t* buffer, uint8_t numBytes, void (*complete)()) { //write to master
    //cli();
    OneWireSlave::beginWriteBytes_(buffer, numBytes, complete == 0 ? noOpCallback_ : complete);
    //sei();
  }

  void OneWireSlave::beginWriteBytes_(uint8_t* buffer, uint8_t numBytes, void(*complete)()) {
    mode = OWM_WRITE;
    txBuffer_ = buffer;
    txBufferLength_ = numBytes;
    txCallback_ = complete;
    ByteP = 0;
    bitp = 1;
    actbit = (bitp & txBuffer_[0]) == bitp;
    wmode = actbit; //prepare for send firs bit
  }

}