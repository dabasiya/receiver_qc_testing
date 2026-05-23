#include <Wire.h>
#include <math.h>
#include <driver/i2s.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;

const char *ssid = "Realme";
const char *password = "11110000v";

long int packetcounter = 0;

IPAddress laptopIP(10,214,91,154);

#include <BLE2902.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
//#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RESET_UUID "beb5483e-36e1-4688-b7f5-ea07361b26b8"
#define FREQ_UUID "beb5483e-36e1-4688-b7f5-ea07361b26fe"
#define AUDIO_BUFFER "beb5483e-36e1-4688-b7f5-ea07361b26ab"

// ── WM8960 I2C address ───────────────────────────────
#define WM8960_ADDR 0x1A

#define AMPLITUDE  (((float)vrms*sqrt(2) / (1.27f * 1000.0f)) * pow(2,31))

// ── I2C pins for XIAO ESP32S3 ────────────────────────
#define I2C_SDA 2
#define I2C_SCL 4

#define I2S_BCLK   5
#define I2S_LRCLK  18
#define I2S_DOUT   19
#define I2S_DIN   15

#define SAMPLE_RATE 48000

unsigned int localPort = 7450;

uint32_t vrms = 180;
uint32_t frequencyuint32 = 1000;


int32_t buffer[512];


// reset variable 

bool reset = false;

float gtime = 0.0f;


float freq_timer[] = {
  0.0f,  // 125 HZ
  1.0f,  // 0 HZ gap
  1.2f,  // 250 HZ
  2.2f,  // 0 HZ gap
  2.4f,  // 500 HZ
  3.4f,  // 0 HZ gap
  3.6f,  // 750 HZ
  4.6f,  // 0 HZ gap
  4.8f,  // 1000 HZ
  5.8f,  // 0 HZ gap
  6.0f,  // 1500 HZ
  7.0f,  // 0 HZ gap
  7.2f,  // 2000 HZ
  8.2f,  // 0 HZ gap
  8.4f,  // 3000 HZ
  9.4f,  // 0 HZ gap
  9.6f,  // 4000 HZ
  10.6f, // 0 HZ gap
  10.8f, // 6000 HZ
  11.8f, // 0 HZ gap
  12.0f, // 8000 HZ
  13.0f  // 0 HZ gap
};

float freqs[] = {
  125.0f, 0.0f,
  250.0f, 0.0f,
  500.0f, 0.0f,
  750.0f, 0.0f,
  1000.0f, 0.0f,
  1500.0f, 0.0f,
  2000.0f, 0.0f,
  3000.0f, 0.0f,
  4000.0f, 0.0f,
  6000.0f, 0.0f,
  8000.0f, 0.0f
};

class BoolCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pchar) {
    String sval = pchar->getValue();
    reset = sval[0];
    Serial.println(sval[0]);
    Serial.println(sval);
  }

  void onRead(BLECharacteristic* pChar) {
    // when client reads back, return current stored value
    pChar->setValue((uint8_t*)&reset, sizeof(reset));
    Serial.println("Client read the characteristic");
  }
};

void printstrtohex(std::string s) {
  for (size_t i = 0; i < s.size(); i++) {
    Serial.printf("%02X ", (uint8_t)s[i]);
  }

  Serial.println();
}



// ─────────────────────────────────────────────────────
// Tone generator
// ─────────────────────────────────────────────────────
void playTone() {
  static float phase = 0.0f;

  int index = 0;
  float frequency = 0.0f;
  float offset = 0.0f;
  for(unsigned int i = 1; i < 22; i++) {
    if(gtime >= freq_timer[i-1] && gtime <= freq_timer[i]) {
      index = i-1;
      frequency = freqs[index];
      offset = gtime - freq_timer[i-1];
      break;
    }
  }

  const float inc = 2.0f * M_PI * frequencyuint32 / (float)SAMPLE_RATE;

  const int N = 512;
  int32_t buf[512];  // mono only
  for (int i = 0; i < N; i++) {
      int32_t s = (int32_t)(AMPLITUDE * sinf(phase));
      buf[i] = s;
      phase += inc;
      if (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;
  }
  size_t written = 0;
  i2s_write(I2S_NUM_0, buf, sizeof(buf), &written, portMAX_DELAY);
  //i2s_read(I2S_NUM_0, buf, sizeof(buf), &written, ])
  if (written != sizeof(buf))
    Serial.printf("i2s_write short: %u / %u\n", written, sizeof(buf));
}



// ─────────────────────────────────────────────────────
bool wm8960_write(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(WM8960_ADDR);
  Wire.write((reg << 1) | ((val >> 8) & 0x01));
  Wire.write(val & 0xFF);
  return (Wire.endTransmission() == 0);
}


void wm8960_init() {
  // Reset
  wm8960_write(0x0F, 0x000);
  delay(10);

  // ── Power: VREF + VMID(50k) + AINL + MICBIAS ──────
  // R25: VMIDSEL=01, VREF=1, AINL=1, AINR=1, ADCL=1, ADCR=1, MICB=1, MCLK=0
  wm8960_write(0x19, 0x0FE);
  delay(300);

  // ── Input PGA: enable LMIC ─────────────────────────
  // R47: LMIC=1, RMIC=0, LOMIX=1, ROMIX=1
  wm8960_write(0x2F, 0x028);

  wm8960_write(0x07, 0x00E);

  // ── LINPUT1 → inverting PGA input (LMN1=1, default) ─
  // R32: LMN1=1, LMIC2B=1 (connect PGA out → boost mixer)
  // LMICBOOST = 01 (+13dB)
  wm8960_write(0x20, 0x118);
  //            ^ bit8=LMN1, bit4=LMICBOOST[1]=0, 
  //              bit3=LMIC2B, bits[5:4]=01 for +13dB
  // Binary: 1_0001_1000 = 0x118

  // ── PGA volume: 0dB, unmute ────────────────────────
  // R0: IPVU=1, LINMUTE=0, LINVOL=010111 (0dB)
  wm8960_write(0x00, 0x13F);

  // ── Boost mixer → Left Output Mixer ───────────────
  // R45: LB2LO=1, LB2LOVOL=111 (0dB)
  wm8960_write(0x2D, 0x000);

  // ── Power up outputs: LOUT1 + ROUT1 ───────────────
  // R26: LOUT1=1, ROUT1=, spkl=1 for enable volume control
  wm8960_write(0x1A, 0x170);
  wm8960_write(0x0A, 0x1FF); // set dac volume


  // spk left 
  wm8960_write(0x28, 0x17F);


  // unmute dac
  wm8960_write(0x05, 0x000);

  //wm8960_write(0x08, 0x1C0);


  
  // set speaker left volume
 // wm8960_write(0x28, 0x17F);

  // set speaker boost
  //wm8960_write(0x33, 0x0A8);

  // left dac to left output mixer
  wm8960_write(0x22, 0x100);

  // ── HP volume: 0dB, OUT1VU=1 ──────────────────────
  // LOUT1VOL = 1111111 = +6dB, or 1111001 = 0dB
  wm8960_write(0x02, 0x179);  // left, with update bit
  wm8960_write(0x03, 0x179);  // right


  Serial.println("WM8960 init done");
}




// callback for ble server so when device is disconnected then ble is start to advertise
class ServerCDCallback : public BLEServerCallbacks {

  void onConnect(BLEServer* server) {
    Serial.println("Connected");
  }

  void onDisconnect(BLEServer* server) {
    Serial.println("Disconnected");

    BLEDevice::startAdvertising();
  }
};

class Float32Callback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    String sval = pChar->getValue();
    std::string raw = std::string(sval.c_str());
    Serial.printf("%s\n",raw);

    float val = 0;
    memcpy(&val, raw.data(), sizeof(float));
    vrms = val;
  }

  void onRead(BLECharacteristic* pChar) {
    
    Serial.println("Client read the characteristic");
  }
};

class Uint32Callback : public BLECharacteristicCallbacks {

  uint32_t* uptr = nullptr;

public :
  Uint32Callback(uint32_t* ptr) {
    uptr = ptr;
  }

  void onWrite(BLECharacteristic* pChar) {
    String sval = pChar->getValue();
    std::string raw = std::string(sval.c_str());

    uint32_t val = 0;
    memcpy(&val, raw.data(), sizeof(uint32_t));
    printstrtohex(raw);
    Serial.printf("%d\n", val);
    *uptr = val;
  }

  void onRead(BLECharacteristic* pChar) {
    // when client reads back, return current stored value
    pChar->setValue((uint8_t*)uptr, sizeof(uint32_t));
    Serial.println("Client read the characteristic");
  }
};


// ─────────────────────────────────────────────────────
// I2S init  (ESP32-S3 as master, WM8960 as slave)
// ESP32-S3 generates BCLK, LRCLK, MCLK.
// MCLK output on GPIO 0  → wire to WM8960 MCLK test pad
// ─────────────────────────────────────────────────────
void i2s_init() {
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 8,
    .dma_buf_len          = 128,
    .use_apll             = false,
    .tx_desc_auto_clear   = true,
    .fixed_mclk           = SAMPLE_RATE * 256   // 256 * Fs MCLK to WM8960
  };

  i2s_pin_config_t pins = {        
    .bck_io_num   = I2S_BCLK,
    .ws_io_num    = I2S_LRCLK,
    .data_out_num = I2S_DOUT,
    .data_in_num  = I2S_DIN
  };

  esp_err_t e;
  e = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  Serial.printf("i2s_driver_install: %s\n", esp_err_to_name(e));

  e = i2s_set_pin(I2S_NUM_0, &pins);
  Serial.printf("i2s_set_pin: %s\n", esp_err_to_name(e));

  e = i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  Serial.printf("i2s_set_clk: %s\n", esp_err_to_name(e));

  Serial.println("I2S init done");
}

BLECharacteristic *pcharbuffer = nullptr;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  wm8960_init();
  i2s_init();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500); Serial.print(F("."));
  }
  udp.begin(localPort);

  Serial.printf("UDP server : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);

  Serial.printf("work\n");
  BLEDevice::init("XIAO_ESP32S3");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE 
                                       );

  BLECharacteristic *pCharacteristic2 = pService->createCharacteristic(
                                         FREQ_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE 
                                       );

  pcharbuffer = pService->createCharacteristic(
    AUDIO_BUFFER, 
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pcharbuffer->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new Uint32Callback(&vrms));
  pCharacteristic2->setCallbacks(new Uint32Callback(&frequencyuint32));
  pServer->setCallbacks(new ServerCDCallback());
  pService->start();

  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  Serial.println("Playing 1 kHz tone...");

  BLEDevice::startAdvertising();
}

void loop() {

  unsigned long smillis = millis();
  if(reset) {
    reset = false;
    gtime = 0.0f;
  }

  playTone();


  
  // put your main code here, to run repeatedly:
  //int32_t buffer[256];
  size_t bytes_read;

  i2s_read(I2S_NUM_0, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);


   udp.beginPacket(laptopIP, localPort);

    udp.write(
        (uint8_t*)&packetcounter,
        sizeof(long int)
    );
    udp.write(
        (uint8_t*)buffer,
        bytes_read/2
    );

    udp.endPacket();

    packetcounter++;

    udp.beginPacket(laptopIP, localPort);

    udp.write(
        (uint8_t*)&packetcounter,
        sizeof(long int)
    );
    udp.write(
        (uint8_t*)&buffer[256],
        bytes_read/2
    );

    udp.endPacket();

    packetcounter++;

  float sum = 0;

  for(uint32_t i = 0; i<256; i++) {
    float amp = (buffer[i] * 1.27f) / pow(2,31);
    sum += pow(amp, 2);
  }

  double ans = (1.0f / 256.0f) * sum;
  float avg = sqrt(ans);
  float mv = pow(10, 43.0f/20.0f);
  mv = avg/mv;


  //Serial.println(vrms);
  //Serial.printf("raw : %f\n", avg);
  //Serial.printf("Voltage : %f\n", mv);
  //Serial.printf("converted : %f\n", mv/0.0126f);
  float pa = mv / 0.0126f;
  float spl = 20 * log10f(pa / (20.0f / 1000000.0f));
  //Serial.printf("Pascal : %f\n", pa);
  //Serial.printf("SPL : %f\n", spl);
  //Serial.printf("vrms : %d\n", vrms);
  //Serial.printf("freq : %d\n", frequencyuint32);
  

  unsigned long emillis = millis();
  gtime += ((float)emillis - (float)smillis)/1000.0f;
}
