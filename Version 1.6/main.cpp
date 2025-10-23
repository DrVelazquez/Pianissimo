#include <FastLED.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// Configuración hardware
#define DATA_PIN             5
#define NUM_NOTES            12
#define NOTE_COLUMNS         4
#define NUM_NOTE_COLUMNS     44
#define LEDS_PER_NOTE        8
#define BLANK_PER_NOTE       3
#define REGION_SIZE        (LEDS_PER_NOTE + BLANK_PER_NOTE)
#define NUM_LEDS           (NUM_NOTE_COLUMNS * REGION_SIZE)

CRGB leds[NUM_LEDS];

// Estado on/off y buffer de lluvia
bool active[NUM_NOTE_COLUMNS] = {false};
bool bufferRain[LEDS_PER_NOTE][NUM_NOTE_COLUMNS] = {false};
struct { unsigned long lastStep = 0; } rainState;

// Parámetros ajustables
uint16_t bufferIntervalMs = 100;
uint8_t  ledBrightness    = 50;
CRGB     colorWhiteKey    = CRGB::Blue;
CRGB     colorBlackKey    = CRGB::Purple;

// Bluetooth/MIDI
static BLEUUID midiServiceUUID("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
static BLEUUID midiCharUUID  ("7772E5DB-3868-4112-A1A9-F2669D106BF3");
BLEServer*       pServer               = nullptr;
BLECharacteristic* pMidiCharacteristic = nullptr;

bool showReadyFlag = false;
unsigned long readyStartTime = 0;
bool btBlinkOn = false;
unsigned long btBlinkLast = 0;
bool isConnected = false;

// Índice físico del LED
inline uint16_t physicalIndex(uint8_t key, uint8_t row) {
  uint16_t base = key * REGION_SIZE;
  return (key & 1)
    ? base + (LEDS_PER_NOTE - 1 - row)
    : base + row;
}

// Dibujar icono BT
void drawBT(bool on) {
  static const char* BT_bitmap[8] = {
    ".....##.....",
    ".....#.#....",
    "...#.#.#....",
    "....###.....",
    "....###.....",
    "...#.#.#....",
    ".....#.#....",
    ".....##....."
  };
  FastLED.clear();
  if (on) {
    for (int r = 0; r < 8; ++r) {
      for (int c = 0; c < 12; ++c) {
        if (BT_bitmap[r][c] == '#') {
          leds[physicalIndex(c, r)] = CRGB::Blue;
        }
      }
    }
  }
  FastLED.show();
}

// Mostrar READY en verde
void showReady() {
  static const char* READY_bitmap[8] = {
    "......#...#.",
    ".###..#..#.",
    "#...#.#.#...",
    "#...#.##....",
    "#...#.#.#...",
    "#...#.#..#..",
    ".###..#...#.",
    "............"
  };
  FastLED.clear();
  for (int r = 0; r < 8; ++r)
    for (int c = 0; c < 12; ++c)
      if (READY_bitmap[r][c]=='#')
        leds[physicalIndex(c,r)] = CRGB::Green;
  FastLED.show();
}

// Procesa un mensaje SysEx: F0 7D <cmd> <payload...> F7
void processSysEx(const std::vector<uint8_t>& msg) {
  if (msg.size() < 5) return;
  if (msg[0]!=0xF0 || msg[1]!=0x7D || msg.back()!=0xF7) return;
  uint8_t cmd = msg[2];
  const uint8_t* pl = msg.data()+3;
  size_t len = msg.size()-4;
  switch(cmd) {
    case 0x01:
      if (len>=1) {
        ledBrightness = pl[0];
        FastLED.setBrightness(ledBrightness);
      }
      break;
    case 0x02:
      if (len>=3) {
        colorWhiteKey = CRGB(pl[0], pl[1], pl[2]);
      }
      break;
    case 0x03:
      if (len>=3) {
        colorBlackKey = CRGB(pl[0], pl[1], pl[2]);
      }
      break;
    case 0x04:
      if (len>=2) {
        bufferIntervalMs = (uint16_t(pl[0])<<7) | uint16_t(pl[1]);
      }
      break;
  }
}

// Apaga todas las notas activas
void allNotesOff() {
  memset(active, 0, sizeof(active));
  memset(bufferRain, 0, sizeof(bufferRain));
  FastLED.clear();
  FastLED.show();
}

// ----------- FIX: parser BLE-MIDI robusto -----------
class MidiCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    std::string d = chr->getValue();
    if (d.empty()) return;
    std::vector<uint8_t> sysExBuf;
    size_t j = 0, n = d.size();

    // Procesar SysEx
    while (j < n) {
      uint8_t b = d[j++];
      if (b == 0xF0) {
        sysExBuf.clear();
        sysExBuf.push_back(b);
      } else if (!sysExBuf.empty()) {
        sysExBuf.push_back(b);
        if (b == 0xF7) {
          processSysEx(sysExBuf);
          sysExBuf.clear();
        }
      }
    }

    // Reiniciar índice para mensajes normales
    j = 0;
    uint8_t runningStatus = 0;

    while (j < n) {
      uint8_t b = d[j];

      // Saltar timestamps BLE-MIDI (bit7=1 y no status válido)
      if ((b & 0x80) && (b & 0xF0) != 0x80 && (b & 0xF0) != 0x90 && (b & 0xF0) != 0xB0) {
        j++;
        continue;
      }

      // Status nuevo
      if (b & 0x80) {
        runningStatus = b;
        j++;
        continue;
      }

      // Si no hay runningStatus, descartar
      if (runningStatus == 0) {
        j++;
        continue;
      }

      // NOTE ON / OFF
      if ((runningStatus & 0xF0) == 0x90 || (runningStatus & 0xF0) == 0x80) {
        if (j + 1 >= n) break;
        uint8_t note = d[j];
        uint8_t vel  = d[j + 1];
        j += 2;

        uint8_t degree = note % 12;
        uint8_t octave = note / 12;
        uint8_t key = degree + (octave % NOTE_COLUMNS) * NUM_NOTES;
        if (key < NUM_NOTE_COLUMNS) {
          if ((runningStatus & 0xF0) == 0x90 && vel > 0)
            active[key] = true;
          else
            active[key] = false;
        }
        continue;
      }

      // CONTROL CHANGE (All Notes Off)
      if ((runningStatus & 0xF0) == 0xB0) {
        if (j + 1 >= n) break;
        uint8_t ctrl = d[j];
        uint8_t val  = d[j + 1];
        j += 2;
        if (ctrl == 123 && val == 0)
          allNotesOff();
        continue;
      }

      j++;
    }
  }
};
// ----------------------------------------------------

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    isConnected = true;
    showReadyFlag = true;
    readyStartTime = millis();
  }
  void onDisconnect(BLEServer* s) override {
    isConnected = false;
    s->getAdvertising()->start();
  }
};

void setupBLEMidiServer() {
  BLEDevice::init("Pianissimo");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  auto svc = pServer->createService(midiServiceUUID);
  pMidiCharacteristic = svc->createCharacteristic(
    midiCharUUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pMidiCharacteristic->addDescriptor(new BLE2902());
  pMidiCharacteristic->setCallbacks(new MidiCallbacks());
  svc->start();
  auto adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(midiServiceUUID);
  adv->setScanResponse(true);
  adv->start();
  delay(500);
  uint8_t dummy[] = {0x80,0x80};
  pMidiCharacteristic->setValue(dummy,sizeof(dummy));
  pMidiCharacteristic->notify();
}

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(ledBrightness);
  FastLED.clear(); 
  FastLED.show();
  memset(bufferRain, 0, sizeof(bufferRain));
  memset(active, 0, sizeof(active));
  setupBLEMidiServer();
}

void loop() {
  unsigned long now = millis();

  if (showReadyFlag) {
    showReady();
    if (now - readyStartTime > 4000) {
      showReadyFlag = false;
      FastLED.clear(); FastLED.show();
      memset(bufferRain, 0, sizeof(bufferRain));
    }
    return;
  }

  if (!isConnected) {
    if (now - btBlinkLast > 600) {
      btBlinkLast = now;
      btBlinkOn = !btBlinkOn;
      drawBT(btBlinkOn);
    }
    return;
  }

  if (now - rainState.lastStep >= bufferIntervalMs) {
    rainState.lastStep = now;
    for (int r = LEDS_PER_NOTE-1; r>0; --r)
      for (int k=0; k<NUM_NOTE_COLUMNS; ++k)
        bufferRain[r][k] = bufferRain[r-1][k];
    for (int k=0; k<NUM_NOTE_COLUMNS; ++k)
      bufferRain[0][k] = active[k];

    FastLED.clear();
    for (int r=0; r<LEDS_PER_NOTE; ++r) {
      for (int k=0; k<NUM_NOTE_COLUMNS; ++k) if(bufferRain[r][k]) {
        bool white = ((k%12)==0 || (k%12)==2 || (k%12)==4 ||
                      (k%12)==5 || (k%12)==7 || (k%12)==9 || (k%12)==11);
        leds[ physicalIndex(k,r) ] = white ? colorWhiteKey : colorBlackKey;
      }
    }
    FastLED.show();
  }
}
