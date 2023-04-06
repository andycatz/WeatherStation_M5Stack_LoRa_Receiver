/**
 * LoRa code for M5Stack by Andy Page
 * This code implements the RECEIVER ONLY.
 * 27th February 2021 (Happy birthday Dad!)
 * 
 */

#define LORA_DEFAULT_SS_PIN  5
#define LORA_DEFAULT_RESET_PIN 26
#define LORA_DEFAULT_DIO0_PIN 36   //Verified by scope on pin of M5Stack module

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80
#define PA_OUTPUT_RFO_PIN         0
#define PA_OUTPUT_PA_BOOST_PIN    1

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

SPISettings _spiSettings;
int _ss = LORA_DEFAULT_SS_PIN;
int _reset = LORA_DEFAULT_RESET_PIN;
int _dio0 = LORA_DEFAULT_DIO0_PIN;
int _packetIndex;
int _implicitHeaderMode;
volatile uint8_t rxFlag=0; //Gets set to 1 if the interrupt pin is set
long _frequency=0;
uint8_t lastDevice[8]; //Device ID for last device a message was received from e.g. UV0

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isr(){
  portENTER_CRITICAL_ISR(&mux);
  rxFlag=1; //Set flag to say some bytes were received by the module (it has interrupted)
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRaBegin(long frequency){
    Serial.print("LoRa Module starting at frequency ");
    Serial.println(frequency);
    _frequency=frequency;
    pinMode(_dio0, INPUT);
    attachInterrupt(_dio0, isr, RISING); //Set interrupt on DIO0 pin state change
    
    _spiSettings = SPISettings(8E6, MSBFIRST, SPI_MODE0);
    _packetIndex = 0;
    _implicitHeaderMode = 0; 
      // setup pins
    pinMode(_ss, OUTPUT);
    // set SS high
    digitalWrite(_ss, HIGH);
  
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(5); //Hold low for 5ms (minimum is 100µs)
    pinMode(_reset, INPUT); //Set pin to be input - which makes it high impedance as the datasheet for the SX1276/77/78/79 requests
    delay(15); //Wait 15ms before doing anything (datasheet minimum 10ms)
  
    // start SPI
    SPI.begin();
  
    // check version
    uint8_t version = LoRaReadRegister(REG_VERSION);
    if (version != 0x12) {
      Serial.println("LoRa Module not detected!");
      while(1){
        //Stop
      }
    }
    else{
      Serial.println("LoRa module detected OK");
    }
    LoRaSleep();
    setFrequency(frequency);
    // reset FIFO address and payload length
    LoRaWriteRegister(REG_FIFO_ADDR_PTR, 0);
    LoRaWriteRegister(REG_FIFO_RX_BASE_ADDR,0);
    LoRaWriteRegister(REG_PAYLOAD_LENGTH, 0);
     // set LNA boost
    LoRaWriteRegister(REG_LNA, LoRaReadRegister(REG_LNA) | 0x03);

    // set auto AGC
    LoRaWriteRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    LoRaSetTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

    LoRaWriteRegister(REG_DIO_MAPPING_1, 0x00); //Setup DIO0 to interrupt on receive

    //Set sync word to match transmitters
    LoRaSetSyncWord(0x55); 

    //Enable CRC checking
    LoRaEnableCrc();

    // clear IRQ's
    LoRaWriteRegister(REG_IRQ_FLAGS, 0xFF);

    // put in standby mode
    LoRaStandby();

    Serial.println("LoRa Module Ready");
}

/**
 * Sets the frequency of the LoRa module
 * freq - frequency in Hertz
 */
void setFrequency(long freq){
  uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  LoRaWriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  LoRaWriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  LoRaWriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));  
}

uint8_t LoRaReadRegister(uint8_t address) {
  return LoRaSingleTransfer(address & 0x7f, 0x00);
}

void LoRaWriteRegister(uint8_t address, uint8_t value) {
  LoRaSingleTransfer(address | 0x80, value);
}

uint8_t LoRaSingleTransfer(uint8_t address, uint8_t value) {
  uint8_t response;

  digitalWrite(_ss, LOW);

  SPI.beginTransaction(_spiSettings);
  SPI.transfer(address);
  response = SPI.transfer(value);
  SPI.endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}

void LoRaStandby() {
  LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRaSleep() {
  LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaContinuousReceive(){
  Serial.println("LoRa Module set to continuous receive mode");
  LoRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

uint8_t LoRaAvailable(){
  return (LoRaReadRegister(REG_RX_NB_BYTES));
}

void LoRaSetFIFOPointer(uint8_t p){
  LoRaWriteRegister(REG_FIFO_ADDR_PTR,p);
}

void LoRaSetTxPower(int level, int outputPin) {
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    LoRaWriteRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    LoRaWriteRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRaSetSyncWord(int sw) {
  LoRaWriteRegister(REG_SYNC_WORD, sw);
}

void LoRaDisableCrc() {
  LoRaWriteRegister(REG_MODEM_CONFIG_2, LoRaReadRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaEnableCrc() {
  LoRaWriteRegister(REG_MODEM_CONFIG_2, LoRaReadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

uint8_t LoRaReadIRQFlags(){
  return (LoRaReadRegister(REG_IRQ_FLAGS));
}

void LoRaClearIRQFlags(){
  LoRaWriteRegister(REG_IRQ_FLAGS,0xFF);
}

void LoRaDumpRegisters() {
  for (int i = 0; i < 128; i++) {
    Serial.print("0x");
    Serial.print(i, HEX);
    Serial.print(": 0x");
    Serial.println(LoRaReadRegister(i), HEX);
  }
}

uint8_t LoRaReadOpMode(){
  return LoRaReadRegister(REG_OP_MODE);
}

float LoRaReadPacketRSSI(){
  float rssiTemp =  -157 + float(LoRaReadRegister(REG_PKT_RSSI_VALUE));
  //float snrTemp = LoRaReadPacketSNR();
  return rssiTemp;
}

float LoRaReadPacketSNR(){
  uint8_t regValue = LoRaReadRegister(REG_PKT_SNR_VALUE);
  int16_t reg16Value = regValue;
  if(regValue>127){
    //Negative value
    reg16Value = regValue-256;  //2's compliment conversion
  }
  return (reg16Value * 0.25);  
}

/**
 * Read specified number of bytes from LoRa receiver
 */
void LoRaReadMessage(uint8_t *message, uint8_t length){
  uint8_t rssi = LoRaReadPacketRSSI();
  float snr = LoRaReadPacketSNR();
  LoRaWriteRegister(REG_FIFO_ADDR_PTR, LoRaGetRXMessageAddress()); //Set pointer to where the latest message starts
  for(uint8_t i=0;i<length;i++){
    message[i] = LoRaReadRegister(REG_FIFO);  
  }

}

void LoRaResetFIFOPointer(){
  LoRaWriteRegister(REG_FIFO_ADDR_PTR,0);
}

uint8_t LoRaGetRXMessageAddress(){
  return LoRaReadRegister(REG_FIFO_RX_CURRENT_ADDR);
}
