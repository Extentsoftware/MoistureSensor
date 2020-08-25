#define FAM 0xE2

#include "eeprom.h"

//  I2C constatants
#define SDA 0 //  SDA-port  (pin 5)
#define SCL 1 //  SCL-port  (pin 6)
#define SRF_ADDRESS 0x70    // Address of the SRF02
#define CMD         0x00    // Command byte, values of 0 being sent with write have to be masked as a byte to stop them being misinterpreted as NULL this is a bug with arduino 1.0
#define RANGEBYTE   0x02    // Byte for start of ranging data

//  Valid Onewire commands
#define CMD_ReadPio 0xF5
#define CMD_WritePio 0x5A
#define CMD_Readbuffer 0xBE
#define CMD_Writebuffer 0x4E
#define CMD_Copybuffer 0x48
#define CMD_RecallMemory 0xB8

//  data & buffers;
uint8_t latch;
uint8_t data[20];
uint8_t piodata[2];

#define PIOA 3  //  (pin 2)
#define PIOB 4  //  (pin 3)

class PIO {  //  pio
  public:
  void read() {
    piodata[0] = ~get();
    ow.write(&piodata[0], 1, &ow.reset);
  };
  void write() { 
    ow.read(&piodata[0], 2, &gotled); 
  };
  static uint8_t get() {
    uint8_t c = 0;
    if (latch & 0x01) c |= 0x02;
    if (digitalRead(PIOA)) c |= 0x01;
    if (latch & 0x02) c |= 0x08;
    if (digitalRead(PIOB)) c |= 0x04;
    uint8_t x = (~c) << 4;
    return x + c;
  }
  static void set(uint8_t val) {
    latch = val;
    port(PIOA, latch & 1);
    port(PIOB, latch & 2);
      eeprom_write_byte((uint8_t*)10, (unsigned char)latch);
  }
  static void gotled() {
    set(piodata[1]);
    ow.reset();
  };
  static void port(int led, bool stat) {
    if (stat) {
      digitalWrite(led, HIGH);
      pinMode(led, INPUT);
    } else {
      pinMode(led, OUTPUT);
      digitalWrite(led, LOW);
    }
  }
  PIO() {
      set(eeprom_read_byte( (uint8_t*)10));
  }
} pio;

class TANK {  //  tank
private:
  int lastRange;
  float radius = 32.5;
  float length = 150.0;
  float height = 137.0;
  int offset = 19;
  int volume;
  int range;
  int level;
  void Int2Buf(uint8_t * buf, int val) {
    buf[0] = (uint8_t)(val & 255);
    buf[1] = (uint8_t)(val >> 8);
  }
  int toInt(uint8_t buf[]) {
    return (buf[0] + (buf[1] << 8));
  }
  float toFloat(uint8_t buf[]) {
    return (buf[0] + (buf[1] << 8));
  };
  float area(float height, float radius) {
    float v = acos(1 - height / radius) * 2;  // i radianer
    float retval = radius * radius / 2 * (v - sin(v));
    return (retval);
  }
public:
  void data2buffer(uint8_t * buffer) {
    Int2Buf(&buffer[0], (int)(offset * 10));
    Int2Buf(&buffer[2], (int)(radius * 10));
    Int2Buf(&buffer[4], (int)(height * 10));
    Int2Buf(&buffer[6], (int)(length * 10));
    Int2Buf(&buffer[8], (int)(volume * 10));
    Int2Buf(&buffer[10], (int)(level * 10));
    Int2Buf(&buffer[12], (int)(range * 10));
    buffer[14] = ow.crc8(&buffer[0], 14);
  };
  void buffer2data(uint8_t * buffer){
    offset = (int) toInt(&buffer[0]) / 10;
    radius = (float) toFloat(&buffer[2])/10;
    height = (float) toFloat(&buffer[4])/10;
    length = (float) toFloat(&buffer[6])/10;
    eeprom.put(&buffer[0],8);
  };
  void calc(int newRange){
    if ((newRange < lastRange * 1.2) && (newRange > lastRange * .8)) { 
      range = (newRange); 
    }
    lastRange = newRange;
    level = height + offset - range;
    float a =
      area(min(level, radius), radius) +
      max(min(level - radius, height - 2 * radius), 0) * 2 * radius +
      area(radius, radius) - area(min((height - level), radius), radius);
    volume = round(a * length / 1000);
  }
  TANK() {
    uint8_t temp[10];
    if (eeprom.get(&temp[0], 8)) { buffer2data(&temp[0]); }
  };
} tank;

class BUFFER {  //  buffer
  private:
  static void sendData() {
    tank.data2buffer(&data[0]);
    ow.write(&data[0], 15, &ow.reset);
  }
  static void gotData(){
    //  first byte is address byte - not used
    tank.buffer2data(&data[1]);
    ow.reset();
  }
  public:
  void read() {
    //  read and discard one byte!
    ow.read(&data[0], 1, &sendData);
  };
  void write() {
    ow.read(&data[0], 15, &gotData);
  };
  void copy() {};
} buffer;


void onCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_ReadPio: pio.read(); break;
    case CMD_WritePio: pio.write(); break;
    case CMD_Readbuffer: buffer.read(); break;
    case CMD_Writebuffer: buffer.write();  break;
    case CMD_Copybuffer: buffer.copy(); break;
    case CMD_RecallMemory: ow.reset(); break;
  }
};

void setup(uint8_t id[8]) {
  ow.begin(&onCommand, id);
}

void loop() {
  getRange();
  _delay_ms(2000);
}

