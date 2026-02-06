#include <Arduino.h>

// =====================
// Ports
// =====================
static HardwareSerial& IN  = Serial1; // from WROOM
static HardwareSerial& OUT = Serial;  // USB -> PC (BINARY ONLY)

// =====================
// Baud / Pins
// =====================
static const uint32_t IN_BAUD  = 921600;

// WROOM TX(17) -> S3 RX(18)
static const int IN_RX_PIN = 18;
static const int IN_TX_PIN = 17; // optional, not used

// =====================
// Protocol
// =====================
static const uint8_t MAGIC = 0xB2;

// =====================
// Buffers
// =====================
static const size_t ENC_MAX = 256;   // max encoded frame (no delimiter)
static const size_t DEC_MAX = 256;

static uint8_t encBuf[ENC_MAX];
static size_t  encLen = 0;

// =====================
// CRC16-CCITT (same as WROOM)
// =====================
static uint16_t crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b=0;b<8;b++){
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else            crc = (crc << 1);
    }
  }
  return crc;
}

// =====================
// COBS decode
// =====================
static int cobs_decode(const uint8_t* input, size_t length, uint8_t* output, size_t out_max){
  size_t read_index = 0;
  size_t write_index = 0;

  while(read_index < length){
    uint8_t code = input[read_index++];
    if(code == 0) return -1;

    for(uint8_t i = 1; i < code; i++){
      if(read_index >= length) return -1;
      if(write_index >= out_max) return -1;
      output[write_index++] = input[read_index++];
    }
    if(code < 0xFF && read_index < length){
      if(write_index >= out_max) return -1;
      output[write_index++] = 0x00;
    }
  }
  return (int)write_index;
}

// =====================
// COBS encode
// =====================
static size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output, size_t out_max){
  if(out_max == 0) return 0;
  size_t read_index=0, write_index=1, code_index=0;
  uint8_t code=1;

  while(read_index < length){
    if(write_index >= out_max) return 0;

    if(input[read_index] == 0){
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    } else {
      output[write_index++] = input[read_index++];
      code++;
      if(code == 0xFF){
        output[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
  }

  if(code_index >= out_max) return 0;
  output[code_index] = code;
  return write_index;
}

// =====================
// Bridge out: VERIFIED frames only (binary only)
// =====================
static void out_verified(const uint8_t* decoded, size_t decoded_len){
  uint8_t outEnc[ENC_MAX + 16];
  size_t outEncLen = cobs_encode(decoded, decoded_len, outEnc, sizeof(outEnc));
  if(outEncLen == 0) return;

  OUT.write(outEnc, outEncLen);
  OUT.write((uint8_t)0x00);
}

// =====================
// Decode+verify one frame then re-frame to OUT
// =====================
static void decode_verify_and_forward(const uint8_t* enc, size_t enc_len){
  uint8_t dec[DEC_MAX];

  int dec_len = cobs_decode(enc, enc_len, dec, sizeof(dec));
  if(dec_len < 0) return;
  if(dec_len < 6) return;

  if(dec[0] != MAGIC) return;

  uint8_t type = dec[1];
  uint8_t seq  = dec[2];
  uint8_t len  = dec[3];

  size_t need = 4u + (size_t)len + 2u;
  if((size_t)dec_len != need) return;

  uint16_t got  = (uint16_t)dec[4+len] | ((uint16_t)dec[4+len+1] << 8);
  uint16_t calc = crc16_ccitt(&dec[1], (size_t)(3 + len)); // type,seq,len,payload
  if(got != calc) return;

  (void)type; (void)seq;
  out_verified(dec, (size_t)dec_len);
}

// =====================
// Stream parser (0x00 delimiter)
// =====================
static void pump_uart(){
  while(IN.available()){
    int v = IN.read();
    if(v < 0) break;
    uint8_t b = (uint8_t)v;

    if(b == 0x00){
      if(encLen > 0){
        decode_verify_and_forward(encBuf, encLen);
        encLen = 0;
      }
      continue;
    }

    if(encLen < ENC_MAX){
      encBuf[encLen++] = b;
    } else {
      // overflow -> discard until next delimiter
      encLen = 0;
    }
  }
}

void setup(){
  // USB serial: no prints. Just start it.
  OUT.begin(115200);
  delay(100);

  IN.begin(IN_BAUD, SERIAL_8N1, IN_RX_PIN, IN_TX_PIN);
}

void loop(){
  pump_uart();
}
