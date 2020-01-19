#include <axp20x.h>

#include <BluetoothSerial.h>

#include <SPIFFS.h>
//#include <U8x8lib.h>
//#include <U8g2lib.h>
#include <SPI.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <MicroNMEA.h>
#include <Ticker.h>

#include <SX1278FSK.h>
#include <Sonde.h>
#include <Display.h>
#include <Scanner.h>
#include <aprs.h>
#include "version.h"
#include "geteph.h"
#include "rs92gps.h"

int e;

enum MainState { ST_DECODER, ST_SPECTRUM, ST_WIFISCAN, ST_UPDATE, ST_TOUCHCALIB };
static MainState mainState = ST_SPECTRUM;

AXP20X_Class axp;
#define PMU_IRQ             35

#define LOCALUDPPORT 9002
boolean connected = false;
BluetoothSerial SerialBT; 
boolean forceReloadScreenConfig = false;

enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG };

// "doublepress" is now also used to eliminate key glitch on TTGO T-Beam startup (SENSOR_VN/GPIO39)
struct Button {
  uint8_t pin;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long keydownTime;
  int8_t doublepress;
  bool isTouched;
};
Button button1 = {0, 0, KP_NONE, 0, -1, false};
Button button2 = {0, 0, KP_NONE, 0, -1, false};


static int lastDisplay = 1;
static int currentDisplay = 1;

// timestamp when spectrum display was activated
static unsigned long specTimer;

// Read line from file, independent of line termination (LF or CR LF)
String readLine(Stream &stream) {
  String s = stream.readStringUntil('\n');
  int len = s.length();
  if (len == 0) return s;
  if (s.charAt(len - 1) == '\r') s.remove(len - 1);
  return s;
}

// Replaces placeholder with LED state value
String processor(const String& var) {
  Serial.println(var);
  if (var == "VERSION_NAME") {
    return String(version_name);
  }
  if (var == "VERSION_ID") {
    return String(version_id);
  }
  if (var == "AUTODETECT_INFO") {
    char tmpstr[128];
    const char *fpstr;
    int i = 0;
    while (fingerprintValue[i] != sonde.fingerprint && fingerprintValue[i] != -1) i++;
    if (fingerprintValue[i] == -1) {
      fpstr = "Unknown board";
    } else {
      fpstr = fingerprintText[i];
    }
    snprintf(tmpstr, 128, "Fingerprint %d (%s)", sonde.fingerprint, fpstr);
    return String(tmpstr);
  }
  return String();
}

const String sondeTypeSelect(int activeType) {
  String sts = "";
  for (int i = 0; i < NSondeTypes; i++) {
    sts += "<option value=\"";
    sts += sondeTypeStr[i];
    sts += "\"";
    if (activeType == i) {
      sts += " selected";
    }
    sts += ">";
    sts += sondeTypeStr[i];
    sts += "</option>";
  }
  return sts;
}

///////////////////////// Functions for Reading / Writing QRG list from/to qrg.txt

void setupChannelList() {
  File file = SPIFFS.open("/qrg.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/qrg.txt' for reading");
    return;
  }
  int i = 0;
  char launchsite[17] = "                ";
  sonde.clearSonde();
  Serial.println("Reading channel config:");
  while (file.available()) {
    String line = readLine(file);   //file.readStringUntil('\n');
    String sitename;
    if (!file.available()) break;
    if (line[0] == '#') continue;
    char *space = strchr(line.c_str(), ' ');
    if (!space) continue;
    *space = 0;
    float freq = atof(line.c_str());
    SondeType type;
    if (space[1] == '4') {
      type = STYPE_RS41;
    } else if (space[1] == 'R') {
      type = STYPE_RS92;
    }
    else if (space[1] == '9') {
      type = STYPE_DFM09;
    }
    else if (space[1] == '6') {
      type = STYPE_DFM06;
    }
    else if (space[1] == 'M') {
      type = STYPE_M10;
    }
    else continue;
    int active = space[3] == '+' ? 1 : 0;
    if (space[4] == ' ') {
      memset(launchsite, ' ', 16);
      int str_len = strlen(space + 5);
      strncpy(launchsite, space + 5, str_len > 16 ? 16 : str_len);
      if (sonde.config.debug == 1) {
        Serial.printf("Add %f - sondetype: %d (on/off: %d) - site #%d - name: %s\n ", freq, type, active, i, launchsite);
      }
    }
    sonde.addSonde(freq, type, active, launchsite);
    i++;
  }
  file.close();
}

///////////////////// Config form

void setupConfigData() {
  File file = SPIFFS.open("/config.txt", "r");
  if (!file) {
    Serial.println("There was an error opening the file '/config.txt' for reading");
    return;
  }
  while (file.available()) {
    String line = readLine(file);  //file.readStringUntil('\n');
    sonde.setConfig(line.c_str());
  }
}

struct st_configitems {
  const char *name;
  const char *label;
  int type;  // 0: numeric; i>0 string of length i; -1: separator; -2: type selector
  void *data;
};

struct st_configitems config_list[] = {
  /* General config settings */
  {"", "Software configuration", -5, NULL},
  {"wifi", "Wifi mode (0/1/2/3)", 0, &sonde.config.wifi},
  {"debug", "Debug mode (0/1)", 0, &sonde.config.debug},
  {"maxsonde", "Maxsonde", 0, &sonde.config.maxsonde},
  {"display", "Display screens (scan,default,...)", -6, sonde.config.display},
  /* Spectrum display settings */
  {"spectrum", "Show spectrum (-1=no, 0=forever, >0=seconds)", 0, &sonde.config.spectrum},
  {"startfreq", "Startfreq (MHz)", 0, &sonde.config.startfreq},
  {"channelbw", "Bandwidth (kHz)", 0, &sonde.config.channelbw},
  {"marker", "Spectrum MHz marker", 0, &sonde.config.marker},
  {"noisefloor", "Sepctrum noisefloor", 0, &sonde.config.noisefloor},
  /* decoder settings */
  {"", "Receiver configuration", -5, NULL},
  {"showafc", "Show AFC value", 0, &sonde.config.showafc},
  {"freqofs", "RX frequency offset (Hz)", 0, &sonde.config.freqofs},
  {"rs41.agcbw", "RS41 AGC bandwidth", 0, &sonde.config.rs41.agcbw},
  {"rs41.rxbw", "RS41 RX bandwidth", 0, &sonde.config.rs41.rxbw},
  {"rs92.rxbw", "RS92 RX (and AGC) bandwidth", 0, &sonde.config.rs92.rxbw},
  {"rs92.alt2d", "RS92 2D fix default altitude", 0, &sonde.config.rs92.alt2d},
  {"dfm.agcbw", "DFM6/9 AGC bandwidth", 0, &sonde.config.dfm.agcbw},
  {"dfm.rxbw", "DFM6/9 RX bandwidth", 0, &sonde.config.dfm.rxbw},
  {"", "Data feed configuration", -5, NULL},
  /* APRS settings */
  {"call", "Call", 8, sonde.config.call},
  {"passcode", "Passcode", 8, sonde.config.passcode},
  /* KISS tnc settings */
  {"kisstnc.active", "KISS TNC (port 14590) (needs reboot)", 0, &sonde.config.kisstnc.active},
  {"kisstnc.idformat", "KISS TNC ID Format", -2, &sonde.config.kisstnc.idformat},
  /* AXUDP settings */
  {"axudp.active", "AXUDP active", -3, &sonde.config.udpfeed.active},
  {"axudp.host", "AXUDP Host", 63, sonde.config.udpfeed.host},
  {"axudp.port", "AXUDP Port", 0, &sonde.config.udpfeed.port},
  {"axudp.idformat", "DFM ID Format", -2, &sonde.config.udpfeed.idformat},
  {"axudp.highrate", "Rate limit", 0, &sonde.config.udpfeed.highrate},
  /* APRS TCP settings, current not used */
  {"tcp.active", "APRS TCP active", -3, &sonde.config.tcpfeed.active},
  {"tcp.host", "ARPS TCP Host", 63, sonde.config.tcpfeed.host},
  {"tcp.port", "APRS TCP Port", 0, &sonde.config.tcpfeed.port},
  {"tcp.idformat", "DFM ID Format", -2, &sonde.config.tcpfeed.idformat},
  {"tcp.highrate", "Rate limit", 0, &sonde.config.tcpfeed.highrate},
  /* Hardware dependeing settings */
  {"", "Hardware configuration (requires reboot)", -5, NULL},
  {"disptype", "Display type (0=OLED/SSD1306, 1=TFT/ILI9225, 2=OLED/SH1106)", 0, &sonde.config.disptype},
  {"oled_sda", "OLED SDA/TFT SDA", 0, &sonde.config.oled_sda},
  {"oled_scl", "OLED SCL/TFT CLK", 0, &sonde.config.oled_scl},
  {"oled_rst", "OLED RST/TFT RST (needs reboot)", 0, &sonde.config.oled_rst},
  {"tft_rs", "TFT RS", 0, &sonde.config.tft_rs},
  {"tft_cs", "TFT CS", 0, &sonde.config.tft_cs},
  {"tft_orient", "TFT orientation (0/1/2/3), OLED flip: 3", 0, &sonde.config.tft_orient},
  {"button_pin", "Button input port", -4, &sonde.config.button_pin},
  {"button2_pin", "Button 2 input port", -4, &sonde.config.button2_pin},
  {"touch_thresh", "Touch button threshold<br>(0 for calib mode)", 0, &sonde.config.touch_thresh},
  {"power_pout", "Power control port", 0, &sonde.config.power_pout},
  {"led_pout", "LED output port", 0, &sonde.config.led_pout},
  {"gps_rxd", "GPS RXD pin (-1 to disable)", 0, &sonde.config.gps_rxd},
  {"gps_txd", "GPS TXD pin (not really needed)", 0, &sonde.config.gps_txd},
  {"mdnsname", "mDNS name", 14, &sonde.config.mdnsname},
};
const static int N_CONFIG = (sizeof(config_list) / sizeof(struct st_configitems));


// It is not safe to call millis() in ISR!!!
// millis() does a division int64_t by 1000 for which gcc creates a library call
// on a 32bit system, and the called function has no IRAM_ATTR
// so doing it manually...
// Code adapted for 64 bits from https://www.hackersdelight.org/divcMore.pdf
int64_t IRAM_ATTR divs10(int64_t n) {
  int64_t q, r;
  n = n + (n >> 63 & 9);
  q = (n >> 1) + (n >> 2);
  q = q + (q >> 4);
  q = q + (q >> 8);
  q = q + (q >> 16);
  q = q + (q >> 32);
  q = q >> 3;
  r = n - q * 10;
  return q + ((r + 6) >> 4);
  // return q + (r > 9);
}

int64_t IRAM_ATTR divs1000(int64_t n) {
  return divs10(divs10(divs10(n)));
}

unsigned long IRAM_ATTR my_millis()
{
  return divs1000(esp_timer_get_time());
}

void checkTouchStatus();
void touchISR();
void touchISR2();

// ISR won't work for SPI transfer, so forget about the following approach
///// Also initialized timers for sx1278 handling with interruts
///// fastest mode currentily is 4800 bit/s, i.e. 600 bytes/sec
///// 64 byte FIFO will last for at most about 106 ms.
///// lets use a timer every 20ms to handle sx1278 FIFO input, that should be fine.
// Instead create a tast...

Ticker ticker;
Ticker ledFlasher;

#define IS_TOUCH(x) (((x)!=255)&&((x)!=-1)&&((x)&128))
void initTouch() {
  // also used for LED
  ticker.attach_ms(300, checkTouchStatus);

  if ( !(IS_TOUCH(sonde.config.button_pin) || IS_TOUCH(sonde.config.button2_pin)) ) return; // no touch buttons configured
  /*
   *  ** no. readTouch is not safe to use in ISR!
      so now using Ticker
    hw_timer_t *timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, checkTouchStatus, true);
    timerAlarmWrite(timer, 300000, true);
    timerAlarmEnable(timer);
  */
  if ( IS_TOUCH(sonde.config.button_pin) ) {
    touchAttachInterrupt(sonde.config.button_pin & 0x7f, touchISR, sonde.config.touch_thresh);
    Serial.printf("Initializing touch 1 on pin %d\n", sonde.config.button_pin & 0x7f);
  }
  if ( IS_TOUCH(sonde.config.button2_pin) ) {
    touchAttachInterrupt(sonde.config.button2_pin & 0x7f, touchISR2, sonde.config.touch_thresh);
    Serial.printf("Initializing touch 2 on pin %d\n", sonde.config.button2_pin & 0x7f);
  }
}

char buffer[85];
MicroNMEA nmea(buffer, sizeof(buffer));

int lastCourse = 0;
void unkHandler(const MicroNMEA& nmea) {
  if (strcmp(nmea.getMessageID(), "VTG") == 0) {
    const char *s = nmea.getSentence();
    while (*s && *s != ',') s++;
    if (*s == ',') s++; else return;
    if (*s == ',') return; /// no new course data
    lastCourse = nmea.parseFloat(s, 0, NULL);
    Serial.printf("Course update: %d\n", lastCourse);
  }
}
void gpsTask(void *parameter) {
  nmea.setUnknownSentenceHandler(unkHandler);

  while (1) {
    while (Serial2.available()) {
      char c = Serial2.read();
      //Serial.print(c);
      if (nmea.process(c)) {
        //Serial.println(nmea.getSentence());
        long lat = nmea.getLatitude();
        long lon = nmea.getLongitude();
        long alt = -1;
        bool b = nmea.getAltitude(alt);
        bool valid = nmea.isValid();
        int course = nmea.getCourse() / 1000;
        uint8_t hdop = nmea.getHDOP();
        //Serial.printf("\nDecode: valid: %d  N %ld  E %ld  alt %ld (%d)  course:%d dop:%d", valid ? 1 : 0, lat, lon, alt, b, c, hdop);
      }
    }
    delay(50);
  }
}

void initGPS() {
  if (sonde.config.gps_rxd < 0) return; // GPS disabled
  Serial2.begin(9600, SERIAL_8N1, sonde.config.gps_rxd, sonde.config.gps_txd);

  xTaskCreate( gpsTask, "gpsTask",
               5000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
}


void sx1278Task(void *parameter) {
  /* new strategy:
      background tasks handles all interactions with sx1278.
      implementation is decoder specific.
      This task is a simple infinit loop that
       (a) initially and after frequency or mode change calls <decoder>.setup()
       (b) then repeatedly calls <decoder>.receive() which should
           (1) update data in the Sonde structure (additional updates may be done later in main loop/waitRXcomplete)
           (2) set output flag receiveResult (success/error/timeout and keybord events)

  */
  while (1) {
    if (rxtask.activate >= 128) {
      // activating sx1278 background task...
      Serial.printf("rx task: activate=%d  mainstate=%d\n", rxtask.activate, rxtask.mainState);
      rxtask.mainState = ST_DECODER;
      rxtask.currentSonde = rxtask.activate & 0x7F;
      Serial.println("rx task: calling sonde.setup()");
      sonde.setup();
    } else if (rxtask.activate != -1) {
      Serial.printf("rx task: activate=%d  mainstate=%d\n", rxtask.activate, rxtask.mainState);
      rxtask.mainState = rxtask.activate;
    }
    rxtask.activate = -1;
    /* only if mainState is ST_DECODER */
    if (rxtask.mainState != ST_DECODER) {
      delay(100);
      continue;
    }
    sonde.receive();
    delay(20);
  }
}


void IRAM_ATTR touchISR() {
  if (!button1.isTouched) {
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) button1.doublepress = 1;
    else button1.doublepress = 0;
    button1.keydownTime = now;
    button1.isTouched = true;
  }
}

void IRAM_ATTR touchISR2() {
  if (!button2.isTouched) {
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) button2.doublepress = 1;
    else button2.doublepress = 0;
    button2.keydownTime = now;
    button2.isTouched = true;
  }
}

// touchRead in ISR is also a bad idea. Now moved to Ticker task
void checkTouchButton(Button & button) {
  if (button.isTouched) {
    int tmp = touchRead(button.pin & 0x7f);
    Serial.printf("touch read %d: value is %d\n", button.pin & 0x7f, tmp);
    if (tmp > sonde.config.touch_thresh + 5) {
      button.isTouched = false;
      unsigned long elapsed = my_millis() - button.keydownTime;
      if (elapsed > 1500) {
        if (elapsed < 4000) {
          button.pressed = KP_MID;
        }
        else {
          button.pressed = KP_LONG;
        }
      } else if (button.doublepress) {
        button.pressed = KP_DOUBLE;
      } else {
        button.pressed = KP_SHORT;
      }
    }
  }
}

void ledOffCallback() {
  digitalWrite(sonde.config.led_pout, LOW);
}
void flashLed(int ms) {
  if (sonde.config.led_pout >= 0) {
    digitalWrite(sonde.config.led_pout, HIGH);
    ledFlasher.once_ms(ms, ledOffCallback);
  }
}

int doTouch = 0;
void checkTouchStatus() {
  checkTouchButton(button1);
  checkTouchButton(button2);
}

unsigned long bdd1, bdd2;
void IRAM_ATTR buttonISR() {
  if (digitalRead(button1.pin) == 0) { // Button down
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) {
      // Double press
      if (now - button1.keydownTime > 100)
        button1.doublepress = 1;
      bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button1.doublepress = 0;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  } else { //Button up
    unsigned long now = my_millis();
    if (button1.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button1.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button1.pressed = KP_MID;
      }
      else {
        button1.pressed = KP_LONG;
      }
    } else {
      if (button1.doublepress) button1.pressed = KP_DOUBLE;
      else button1.pressed = KP_SHORT;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  }
}

void IRAM_ATTR button2ISR() {
  if (digitalRead(button2.pin) == 0) { // Button down
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) {
      // Double press
      if (now - button2.keydownTime > 100)
        button2.doublepress = 1;
      //bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button2.doublepress = 0;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  } else { //Button up
    unsigned long now = my_millis();
    if (button2.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button2.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button2.pressed = KP_MID;
      }
      else {
        button2.pressed = KP_LONG;
      }
    } else {
      if (button2.doublepress) button2.pressed = KP_DOUBLE;
      else button2.pressed = KP_SHORT;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  }
}

int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
  int x = digitalRead(button1.pin);
  //Serial.printf("Debug: bdd1=%ld, bdd2=%ld\b", bdd1, bdd2);
  //Serial.printf("button1 press (dbl:%d) (now:%d): %d at %ld (%d)\n", button1.doublepress, x, p, button1.keydownTime, button1.numberKeyPresses);
  return p;
}

int getKey2Press() {
  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;
  //Serial.printf("button2 press: %d at %ld (%d)\n", p, button2.keydownTime, button2.numberKeyPresses);
  return p;
}
int hasKeyPress() {
  return button1.pressed || button2.pressed;
}
int getKeyPressEvent() {
  int p = getKeyPress();
  if (p == KP_NONE) {
    p = getKey2Press();
    if (p == KP_NONE)
      return EVT_NONE;
    Serial.printf("Key 2 was pressed [%d]\n", p + 4);
    return p + 4;
  }
  Serial.printf("Key 1 was pressed [%d]\n", p);
  return p;  /* map KP_x to EVT_KEY1_x / EVT_KEY2_x*/
}

#define SSD1306_ADDRESS 0x3c
bool ssd1306_found = false;
bool axp192_found = false;

int scanI2Cdevice(void)
{
  byte err, addr;
  int nDevices = 0;
  for (addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device found at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.print(addr, HEX);
      Serial.println(" !");
      nDevices++;

      if (addr == SSD1306_ADDRESS) {
        ssd1306_found = true;
        Serial.println("ssd1306 display found");
      }
      if (addr == AXP192_SLAVE_ADDRESS) {
        axp192_found = true;
        Serial.println("axp192 PMU found");
      }
    } else if (err == 4) {
      Serial.print("Unknow error at address 0x");
      if (addr < 16)
        Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  return nDevices;
}

extern int initlevels[40];
bool pmu_irq = false;


void setup()
{
  char buf[12];
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  for (int i = 0; i < 39; i++) {
    int v = gpio_get_level((gpio_num_t)i);
    Serial.printf("%d:%d ", i, v);
  }
  Serial.println("");
#if 0
  delay(2000);
  // temporary test
  volatile uint32_t *ioport = portOutputRegister(digitalPinToPort(4));
  uint32_t portmask = digitalPinToBitMask(4);
  int t = millis();
  for (int i = 0; i < 10000000; i++) {
    digitalWrite(4, LOW);
    digitalWrite(4, HIGH);
  }
  int res = millis() - t;
  Serial.printf("Duration w/ digitalWriteo: %d\n", res);

  t = millis();
  for (int i = 0; i < 10000000; i++) {
    *ioport |=  portmask;
    *ioport &= ~portmask;
  }
  res = millis() - t;
  Serial.printf("Duration w/ fast io: %d\n", res);
#endif
  for (int i = 0; i < 39; i++) {
    Serial.printf("%d:%d ", i, initlevels[i]);
  }
  Serial.println(" (before setup)");
  sonde.defaultConfig();  // including autoconfiguration
  aprs_gencrctab();

  Serial.println("Initializing SPIFFS");
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  Serial.println("Reading initial configuration");
  setupConfigData();    // configuration must be read first due to OLED ports!!!

  if ((sonde.fingerprint & 16) == 16) { // NOT TTGO v1 (fingerprint 64) or Heltec v1/v2 board (fingerprint 4)
    // FOr T-Beam 1.0
    for (int i = 0; i < 10; i++) { // try multiple times
      Wire.begin(21, 22);
      // Make sure the whole thing powers up!?!?!?!?!?
      U8X8 *u8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(0, 22, 21);
      u8x8->initDisplay();
      delay(500);

      scanI2Cdevice();
      if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
      } else {
        Serial.println("AXP192 Begin FAIL");
      }
      axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
      axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
      axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
      axp.setDCDC1Voltage(3300);
      axp.adc1Enable(AXP202_VBUS_VOL_ADC1, 1);
      axp.adc1Enable(AXP202_VBUS_CUR_ADC1, 1);
      pinMode(PMU_IRQ, INPUT_PULLUP);
      attachInterrupt(PMU_IRQ, [] {
        pmu_irq = true;
      }, FALLING);
      axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
      axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
      axp.clearIRQ();
      int ndevices = scanI2Cdevice();
      if (sonde.fingerprint != 17 || ndevices > 0) break; // only retry for fingerprint 17 (startup problems of new t-beam with oled)
      delay(500);
    }
  }
  if (sonde.config.power_pout >= 0) { // for a heltec v2, pull GPIO21 low for display power
    pinMode(sonde.config.power_pout & 127, OUTPUT);
    digitalWrite(sonde.config.power_pout & 127, sonde.config.power_pout & 128 ? 1 : 0);
  }

  if (sonde.config.led_pout >= 0) {
    pinMode(sonde.config.led_pout, OUTPUT);
    flashLed(1000); // testing
  }

  button1.pin = sonde.config.button_pin;
  button2.pin = sonde.config.button2_pin;
  if (button1.pin != 0xff) {
    if ( (button1.pin & 0x80) == 0 && button1.pin < 34 ) {
      Serial.println("Button 1 configured as input with pullup");
      pinMode(button1.pin, INPUT_PULLUP);
    } else
      pinMode(button1.pin, INPUT);  // configure as input if not disabled
  }
  if (button2.pin != 0xff) {
    if ( (button2.pin & 0x80) == 0 && button2.pin < 34 ) {
      Serial.println("Button 2 configured as input with pullup");
      pinMode(button2.pin, INPUT_PULLUP);
    } else
      pinMode(button2.pin, INPUT);  // configure as input if not disabled
  }
  // Handle button press
  if ( (button1.pin & 0x80) == 0) {
    attachInterrupt( button1.pin, buttonISR, CHANGE);
    Serial.printf("button1.pin is %d, attaching interrupt\n", button1.pin);
  }
  // Handle button press
  if ( (button2.pin & 0x80) == 0) {
    attachInterrupt( button2.pin, button2ISR, CHANGE);
    Serial.printf("button2.pin is %d, attaching interrupt\n", button2.pin);
  }
  initTouch();

  disp.init();
  delay(100);
  Serial.println("Showing welcome display");
  disp.rdis->welcome();
  delay(3000);
  Serial.println("Clearing display");
  sonde.clearDisplay();

  disp.initFromFile();
  Serial.printf("disp.initFromFile... layouts is %p", disp.layouts);


  // == show initial values from config.txt ========================= //
  if (sonde.config.debug == 1) {
    disp.rdis->setFont(FONT_SMALL);
    disp.rdis->drawString(0, 0, "Config:");

    delay(500);
    itoa(sonde.config.oled_sda, buf, 10);
    disp.rdis->drawString(0, 1, " SDA:");
    disp.rdis->drawString(6, 1, buf);

    delay(500);
    itoa(sonde.config.oled_scl, buf, 10);
    disp.rdis->drawString(0, 2, " SCL:");
    disp.rdis->drawString(6, 2, buf);

    delay(500);
    itoa(sonde.config.oled_rst, buf, 10);
    disp.rdis->drawString(0, 3, " RST:");
    disp.rdis->drawString(6, 3, buf);

    delay(1000);
    itoa(sonde.config.led_pout, buf, 10);
    disp.rdis->drawString(0, 4, " LED:");
    disp.rdis->drawString(6, 4, buf);

    delay(500);
    itoa(sonde.config.spectrum, buf, 10);
    disp.rdis->drawString(0, 5, " SPEC:");
    disp.rdis->drawString(6, 5, buf);

    delay(500);
    itoa(sonde.config.maxsonde, buf, 10);
    disp.rdis->drawString(0, 6, " MAX:");
    disp.rdis->drawString(6, 6, buf);

    delay(5000);
    sonde.clearDisplay();
  }
  // == show initial values from config.txt ========================= //

#if 0
  // == check the radio chip by setting default frequency =========== //
  if (rs41.setFrequency(402700000) == 0) {
    Serial.println(F("Setting freq: SUCCESS "));
  } else {
    Serial.println(F("Setting freq: ERROR "));
  }
  float f = sx1278.getFrequency();
  Serial.print("Frequency set to ");
  Serial.println(f);
  // == check the radio chip by setting default frequency =========== //
#endif

  //sx1278.setLNAGain(-48);
  sx1278.setLNAGain(0);

  int gain = sx1278.getLNAGain();
  Serial.print("RX LNA Gain is ");
  Serial.println(gain);

  // Print a success message
  Serial.println(F("SX1278 configuration finished"));

  Serial.println("Setup finished");
  Serial.println();

  //   xTaskCreate(mainloop, "MainServer", 10240, NULL, 10, NULL);

  // == setup default channel list if qrg.txt read fails =========== //
  setupChannelList();
#if 0
  sonde.clearSonde();
  sonde.addSonde(402.700, STYPE_RS41);
  sonde.addSonde(405.700, STYPE_RS41);
  sonde.addSonde(405.900, STYPE_RS41);
  sonde.addSonde(403.450, STYPE_DFM09);
  Serial.println("No channel config file, using defaults!");
  Serial.println();
#endif
  /// not here, done by sonde.setup(): rs41.setup();
  // == setup default channel list if qrg.txt read fails =========== //

  xTaskCreate( sx1278Task, "sx1278Task",
               10000, /* stack size */
               NULL, /* paramter */
               1, /* priority */
               NULL);  /* task handle*/
  sonde.setup();
  SerialBT.begin("ESP32LoRa"); //Bluetooth device name
  initGPS();
  getKeyPress();    // clear key buffer
}

void enterMode(int mode) {
  Serial.printf("enterMode(%d)\n", mode);
  // Backround RX task should only be active in mode ST_DECODER for now
  // (future changes might use RX background task for spectrum display as well)
  if (mode != ST_DECODER) {
    rxtask.activate = mode;
    while (rxtask.activate == mode) {
      delay(10);  // until cleared by RXtask -- rx task is deactivated
    }
  }
  mainState = (MainState)mode;
  if (mainState == ST_SPECTRUM) {
    Serial.println("Entering ST_SPECTRUM mode");
    // sonde.clearDisplay();
    disp.rdis->setFont(FONT_SMALL);
    specTimer = millis();
    //scanner.init();
  } else if (mainState == ST_WIFISCAN) {
    sonde.clearDisplay();
  }
  if (mode == ST_DECODER) {
    // trigger activation of background task
    // currentSonde should be set before enterMode()
    rxtask.activate = ACT_SONDE(sonde.currentSonde);
    // -- Dont update display while scanning
    // sonde.clearDisplay();
    // sonde.updateDisplay();
    // -- Will be updated when tracking
    disp.rdis->setFont(FONT_SMALL);
    disp.rdis->drawString(0, 0, sonde.si()->launchsite);
  }
}

static char text[40];
static const char *action2text(uint8_t action) {
  if (action == ACT_DISPLAY_DEFAULT) return "Default Display";
  if (action == ACT_DISPLAY_SPECTRUM) return "Spectrum Display";
  if (action == ACT_DISPLAY_WIFI) return "Wifi Scan Display";
  if (action == ACT_NEXTSONDE) return "Go to next sonde";
  if (action == ACT_PREVSONDE) return "presonde (not implemented)";
  if (action == ACT_NONE) return "none";
  if (action >= 128) {
    snprintf(text, 40, "Sonde=%d", action & 127);
  } else {
    snprintf(text, 40, "Display=%d", action);
  }
  return text;
}

// encode checksum for UKHAS standard
char *hexchar = "0123456789ABCDEF";
void setcheck(char *text, int len) {
	// "Message=$$RS_%s,%d,%06d,%0.5f,%0.5f,%d,0,0,0,rdzsonde*FFFF\r\n"
	// "0123456789^                                          ^65432211"
	uint16_t i, j, x = 0xffff;
	for (i = 10; i < len - 7; i++) {
		//Serial.print(text[i]);
		x ^= text[i] << 8;
		for (j = 0; j < 8; j++) {
			if (x & 0x8000)
				x = (x << 1) ^ 0x1021;
			else
				x = (x << 1);
		}
	}
	// len includes checksum characters !
	for (i = 0; i < 4; i++)
		text[len -3 -i] = hexchar[(x >> (4*i)) & 0xf]; 
}

void loopDecoder() {
  // sonde knows the current type and frequency, and delegates to the right decoder
  uint16_t res = sonde.waitRXcomplete();
  int action;
  Serial.printf("waitRX result is %x\n", (int)res);
  action = (int)(res >> 8);
  // TODO: update displayed sonde?

  if (action != ACT_NONE) {
    Serial.printf("Loop: triggering action %s (%d)\n", action2text(action), action);
    action = sonde.updateState(action);
    Serial.printf("Loop: action is %d, sonde index is %d\n", action, sonde.currentSonde);
    if (action != 255) {
      if (action == ACT_DISPLAY_SPECTRUM) {
        enterMode(ST_SPECTRUM);
        return;
      }
      else if (action == ACT_DISPLAY_WIFI) {
        // enterMode(ST_WIFISCAN);
        return;
      }
      else if (action == ACT_NEXTSONDE) {
        enterMode(ST_SPECTRUM); // update spectrum between scans
        return;
      }
    }
    Serial.printf("current main is %d, current rxtask is %d\n", sonde.currentSonde, rxtask.currentSonde);
  }

// Bluetooth to TNC could be added
#if 0
  if (!tncclient.connected()) {
    Serial.println("TNC client not connected");
    tncclient = tncserver.available();
    if (tncclient.connected()) {
      Serial.println("new TCP KISS connection");
    }
  }
  if (tncclient.available()) {
    Serial.print("TCP KISS socket: recevied ");
    while (tncclient.available()) {
      Serial.print(tncclient.read());  // Check if we receive anything from from APRSdroid
    }
    Serial.println("");
  }
  // wifi (axudp) or bluetooth (bttnc) active => send packet
  if ((res & 0xff) == 0 && (connected || tncclient.connected() )) {
    //Send a packet with position information
    // first check if ID and position lat+lonis ok
    SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
    if (s->validID && ((s->validPos & 0x03) == 0x03)) {
      const char *str = aprs_senddata(s, sonde.config.call, sonde.config.udpfeed.symbol);
      if (connected)  {
        char raw[201];
        int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
        Serial.println("Sending position via UDP");
        Serial.print("Sending: "); Serial.println(raw);
        udp.beginPacket(sonde.config.udpfeed.host, sonde.config.udpfeed.port);
        udp.write((const uint8_t *)raw, rawlen);
        udp.endPacket();
      }
      if (tncclient.connected()) {
        Serial.println("Sending position via TCP");
        char raw[201];
        int rawlen = aprsstr_mon2kiss(str, raw, APRS_MAXLEN);
        Serial.print("sending: "); Serial.println(raw);
        tncclient.write(raw, rawlen);
      }
    }
    //TODO
  }
#endif

  static int rawlen = 0;
  static char raw[101];
  static int timesince = 99;
  if ( !(res & 0xff) ) {
    SondeInfo *s = &sonde.sondeList[rxtask.receiveSonde];
    Serial.printf("Got position for Bluetooth (%d, %d)\n", s->validID && s->validPos, timesince);
    if (s->validID && s->validPos) {
      uint8_t H,m,sec;
      H = ((s->hhmmss) >>16) &31;
      m = ((s->hhmmss) >> 8) &63;
      sec = (s->hhmmss) &63;
      rawlen = snprintf(raw, 100, "Message=$$RS_%s,%d,%02d:%02d:%02d,%0.5f,%0.5f,%d,0,0,0,rdzsonde*FFFF\r\n",
			s->id, s->frame, H,m,sec, s->lat, s->lon, (int)s->alt/*speed,temp,humidity,comment*/);
      setcheck(raw, rawlen);
    }
  }
  timesince++; // roughly count seconds, to rate limit uploads
  if (rawlen && timesince > 9) {
 	SerialBT.write((uint8_t *)raw, rawlen);
	Serial.println(raw);

	rawlen = 0;
	timesince = 0;
  }

  // TODO: Accept commands from Bluetooth
  if (SerialBT.available()) {
    while (SerialBT.available())
      Serial.write(SerialBT.read());
    Serial.println("<from Bluetooth>");
  }
 
  Serial.println("updateDisplay started");
  if (forceReloadScreenConfig) {
    disp.initFromFile();
    sonde.clearDisplay();
    forceReloadScreenConfig = false;
  }
  sonde.updateDisplay();
  Serial.println("updateDisplay done");
}

void setCurrentDisplay(int value) {
  currentDisplay = sonde.config.display[value];
}

void loopSpectrum() {
  int marker = 0;
  char buf[10];

#if 0
  switch (getKeyPress()) {
    case KP_SHORT: /* move selection of peak, TODO */
      sonde.nextConfig(); // TODO: Should be set specific frequency
      enterMode(ST_DECODER);
      return;
    case KP_MID: /* restart, TODO */ break;
    case KP_LONG:
      Serial.println("loopSpectrum: KP_LONG");
      enterMode(ST_WIFISCAN);
      return;
    case KP_DOUBLE:
      setCurrentDisplay(0);
      enterMode(ST_DECODER);
      return;
    default: break;
  }
#endif

  scanner.scan();
  sonde.clearDisplay();
  scanner.plotResult();
#if 0
  if (sonde.config.marker != 0) {
    itoa((sonde.config.startfreq), buf, 10);
    disp.rdis->drawString(0, 1, buf);
    disp.rdis->drawString(7, 1, "MHz");
    itoa((sonde.config.startfreq + 6), buf, 10);
    disp.rdis->drawString(13, 1, buf);
  }

  if (sonde.config.spectrum > 0) {
    int remaining = sonde.config.spectrum - (millis() - specTimer) / 1000;
    itoa(remaining, buf, 10);
    Serial.printf("config.spectrum:%d  specTimer:%ld millis:%ld remaining:%d\n", sonde.config.spectrum, specTimer, millis(), remaining);
    if (sonde.config.marker != 0) {
      marker = 1;
    }
    disp.rdis->drawString(0, 1 + marker, buf);
    disp.rdis->drawString(2, 1 + marker, "Sec.");
    if (remaining <= 0) {
      setCurrentDisplay(0);
      enterMode(ST_DECODER);
    }
  }
#else
    enterMode(ST_DECODER);
#endif
}

void startSpectrumDisplay() {
  sonde.clearDisplay();
  disp.rdis->setFont(FONT_SMALL);
  disp.rdis->drawString(0, 0, "Spectrum Scan...");
  delay(500);
  enterMode(ST_SPECTRUM);
}


enum t_wifi_state { WIFI_DISABLED };
static t_wifi_state wifi_state = WIFI_DISABLED;

void initialMode() {
  if (sonde.config.touch_thresh == 0) {
    enterMode(ST_TOUCHCALIB);
    return;
  }
  if (sonde.config.spectrum != -1) {    // enable Spectrum in config.txt: spectrum=number_of_seconds
    startSpectrumDisplay();
  } else {
    setCurrentDisplay(0);
    enterMode(ST_DECODER);
  }
}

void loopTouchCalib() {
  uint8_t dispw, disph, dispxs, dispys;
  disp.rdis->clear();
  disp.rdis->getDispSize(&disph, &dispw, &dispxs, &dispys);
  char num[10];

  while (1) {
    int t1 = touchRead(button1.pin & 0x7f);
    int t2 = touchRead(button2.pin & 0x7f);
    disp.rdis->setFont(FONT_LARGE);
    disp.rdis->drawString(0, 0, "Touch calib.");
    disp.rdis->drawString(0, 3 * dispys, "Touch1: ");
    snprintf(num, 10, "%d  ", t1);
    disp.rdis->drawString(8 * dispxs, 3 * dispys, num);
    disp.rdis->drawString(0, 6 * dispys, "Touch2: ");
    snprintf(num, 10, "%d  ", t2);
    disp.rdis->drawString(8 * dispxs, 6 * dispys, num);
    delay(300);
  }
}


void loop() {
  // Serial.printf("\nRunning main loop in state %d. free heap: %d;\n", mainState, ESP.getFreeHeap());
  // Serial.printf("currentDisp:%d lastDisp:%d\n", currentDisplay, lastDisplay);
  switch (mainState) {
    case ST_DECODER: loopDecoder(); break;
    case ST_WIFISCAN:
    case ST_UPDATE:
    case ST_SPECTRUM: loopSpectrum(); break;
    case ST_TOUCHCALIB: loopTouchCalib(); break;
  }
#if 0
  int rssi = sx1278.getRSSI();
  Serial.print("  RSSI: ");
  Serial.print(rssi);

  int gain = sx1278.getLNAGain();
  Serial.print(" LNA Gain: "),
               Serial.println(gain);
#endif
  if (currentDisplay != lastDisplay && (mainState == ST_DECODER)) {
    disp.setLayout(currentDisplay);
    sonde.clearDisplay();
    sonde.updateDisplay();
    lastDisplay = currentDisplay;
  }
  // Serial.printf("Unused stack: %d\n", uxTaskGetStackHighWaterMark(0));
}
