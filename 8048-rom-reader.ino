
#define RESET_PORT PORTD
#define RESET_DDR DDRD
#define RESET_MASK ((1 << 7) | (1 << 6))

#define DB_PORT PORTC
#define DB_PIN PINC
#define DB_DDR DDRC

#define P2_PORT PORTF
#define P2_DDR DDRF
#define P2_MASK 0x07

#define MAX_ADDR 0x7FF

uint8_t read_rom_addr(uint16_t addr) {
  RESET_PORT &= ~RESET_MASK; // Make sure RESET low.
  DB_DDR = 0xFF; // Make sure output.

  DB_PORT = addr & 0xFF;
  P2_PORT = (addr >> 8) & 0xFF;
  delayMicroseconds(10);

  RESET_PORT |= RESET_MASK;
  DB_DDR = 0x00;
  delayMicroseconds(10);

  uint8_t data = DB_PIN;

  RESET_PORT &= ~RESET_MASK;
  DB_DDR = 0xFF;

  return data;
}

void setup() {
  RESET_DDR |= RESET_MASK;
  RESET_PORT &= ~RESET_MASK;

  DB_DDR = 0xFF;
  P2_DDR = 0xFF;

  Serial.begin(115200);
  while (!Serial) {
  }
}

void loop() {
  static uint16_t addr = 0;

  if ((addr & 0x00F) == 0) {
    Serial.println();
    if (addr < 0x100) {
      if (addr < 0x10) {
        Serial.print('0');
      }
      Serial.print('0');
    }
    Serial.print(addr, HEX);
    Serial.print(": ");
  }
  
  uint8_t data = read_rom_addr(addr++);
  if (data < 0x10) {
    Serial.print('0');
  }
  Serial.print(data, HEX);

  delay(100);

  if (addr > MAX_ADDR) {
    addr = 0;
  }
}
