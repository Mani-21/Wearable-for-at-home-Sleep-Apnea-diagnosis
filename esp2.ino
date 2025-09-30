/*
  sleep apnea project
  esp2 hub

  version 14 
  ref:
  https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
  https://lastminuteengineers.com/esp32-mdns-tutorial/
  https://www.aranacorp.com/en/communication-between-two-esp32s-via-udp/ 
  https://github.com/DFRobot/DFRobot_GDL/wiki/English-WIKI
  https://randomnerdtutorials.com/guide-to-sd-card-module-with-arduino/
  https://github.com/adafruit/RTClib/blob/master/src/RTC_DS3231.cpp
  https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
  https://github.com/sparkfun/MAX30105_Particle_Sensor_Breakout/tree/master/Libraries/Arduino/src
  https://docs.arduino.cc/programming/?_gl=1*1pz98k7*_up*MQ..*_ga*NTg3NzU3NzA0LjE3NTYzMDY3NTI.*_ga_NEXN8H46L5*czE3NTYzMDY3NTIkbzEkZzAkdDE3NTYzMDY3NTIkajYwJGwwJGg5NDMwNjIxMTI.
  https://www.instructables.com/Pulse-Oximeter-With-Much-Improved-Precision/
  https://www.dsprelated.com/freebooks/sasp/Quadratic_Interpolation_Spectral_Peaks.html
*/


// libs
#include <WiFi.h>
#include <WiFiUdp.h>    // ports
#include <WebServer.h>  // mode webpage
#include <Wire.h>       // i2c
#include <RTClib.h>     // rtc
#include <SPI.h>
#include <SD.h>
#include "DFRobot_GDL.h" // screen
#include "MAX30105.h" 
#include <arduinoFFT.h>
#include <math.h>
#include <ESPmDNS.h>    // mdns

// wifi
#define WIFI_SSID  "VM2C2F68"        // wifi username  // "VM2C2F68"  // "Mani"
#define WIFI_PASS  "dtUttDw2tqnu"   // wifi password  // "dtUttDw2tqnu" //"manikanta"
const char* MDNS_HOST = "sleep-hub"; // mdns name so browser can find http page

// ports
const uint16_t PORT_TELEMETRY = 5005; // senders -> hub
const uint16_t PORT_ACK       = 5006; // hub -> senders
const uint16_t PORT_SYNC      = 5007; // hub -> all broadcast
const uint16_t TCP_PORT       = 8000; // matlab tcp

// pins - screen + sd + i2c
#define SD_CS   13
#define TFT_DC  25
#define TFT_CS  14
#define TFT_RST 26
#define I2C_SDA 21 //rtc
#define I2C_SCL 22 //rtc

// timing
#define MAX_STALE_MS   3000   // data considered stale after this many ms
#define SYNC_PERIOD_MS 1000   // how often we broadcast sync
#define WIFI_RETRY_MS  20000  // how often we retry wifi if it disconnectes

// modes
struct ModeConfig { uint8_t mode; uint16_t hub_tick_ms; };
const ModeConfig MODE1 = {1, 500};   // 2 hz
const ModeConfig MODE2 = {2, 1000};  // 1 hz
volatile uint8_t currentMode = 1;    // starts in mode 1
ModeConfig cfg = MODE1;              // current hub 

// packets
// header structure copied from esp1
// header is common for all data packets
struct PacketHeader {
  uint8_t  dev_id;     // 1 esp1, 3 esp3
  uint8_t  mode;       // 1 or 2
  uint16_t payload_len;
  uint32_t seq;
  uint32_t t_ms;
};

// esp1/imu data packet structure
struct SensorPayload {
  float ax, ay, az;
  float gx, gy, gz;
  float env, ratio, rms;
  uint8_t zc;
  uint8_t posture;
  uint8_t motion;
  uint8_t effort;
  uint8_t why;
  float   conf;
};

// esp3/mic+temp data packet structure
struct MicTempPayload {
  int32_t avgAmplitude;
  float   maxMagnitude;
  float   dominantFreq;
  uint8_t snoring;
  float   tempC;
  float   env, ratio, rms;
  uint8_t zc;
  uint8_t effort;
  uint8_t why;
  float   conf;
  uint8_t apnea;
};

// ack reply packet
// (dev_id, seqAck=seq) to allow node side retry logic

//  Node logic (esp1 and 3)
//  send -> wait ACK_WAIT_MS -> if no ack, resend (<MAX_RESEND) -> drop.
struct AckPacket { uint8_t dev_id; uint32_t seqAck; };

// sync broadcast packet
struct SyncPacket {
  uint8_t  msgType;   // 1 sync, 2 mode set
  uint8_t  targetDev; // 0 all
  uint32_t epoch_ms;  // hub millis at send time 
  uint8_t  mode;      // current mode
};

// global
WiFiUDP udpRx, udpAck, udpSync;     // udp sockets
WebServer server(80);               // http port is 80
WiFiServer tcpServer(TCP_PORT);     //  tcp stream for matlab
WiFiClient tcpClient;               // connected client

// screen driver (st7789 240x240)
//https://github.com/DFRobot/DFRobot_GDL/wiki/English-WIKI
DFRobot_ST7789_240x240_HW_SPI screen(TFT_DC, TFT_CS, TFT_RST);
// rtc 
RTC_DS3231 rtc;

// timers for periodic tasks
uint32_t lastSync=0, lastLog=0, lastUi=0;
// last timestamps for incoming nodes
uint32_t lastESP1_ms=0, lastESP3_ms=0;
// simple apnea counter from esp3 
uint32_t apneaCount=0;
// wifi retry 
uint32_t lastWiFiAttempt=0;

// last received data 
SensorPayload  imuData{};
MicTempPayload micData{};
bool sdOk=false, ppgOk=false;
bool mdnsStarted=false;

// time in ms since boot
static inline uint32_t now_ms(){ return millis(); }

// convert why code to text
static inline const char* whyStr(uint8_t w){
  switch(w){ case 1: return "A"; case 2: return "B"; case 3: return "A+B"; case 4: return "Q"; default: return "-"; }
}

// ppg calc : hr/spo2
// Algorithm summaryy
// sampling Fs = 100 Hz into buffer N=256 (2.56sec)
// AC, 1-pole HPF y[n] = alpha * ( y[n-1] + x[n] - x[n-1] ), alpha=HPF_ALPHA
// then 1-pole LPF z[n] = z[n-1] + beta * ( y[n] - z[n-1] ), beta=LPF_BETA
// spo2 ratio-of-ratios = (AC_red/DC_red) / (AC_ir/DC_ir)
// HR: FFT on |IR| AC, peak in [0.6,3.0] Hz,
// parabolic interpolation -> peak to sub-bin resolution.
// Fs_eff: actual sample rate computed 
class ppgcalc {
public:
  // sampling and window
  static constexpr uint16_t SAMPLE_RATE_HZ    = 100;
  static constexpr uint16_t WIN_SIZE          = 256;   // 2.56sec
  static constexpr uint32_t HR_COMPUTE_MS     = 500;   
  static constexpr bool     USE_FS_EFF        = true;  // estimate actual fs

  // finger detection dc thresholds
  static constexpr uint32_t MIN_FINGER_IR     = 20000;
  static constexpr uint32_t MIN_FINGER_RED    = 20000;

  // filters for ac signal
  static constexpr float    HPF_ALPHA         = 0.984f; 
  static constexpr float    LPF_BETA          = 0.20f;  // large-> faster

  // simple spo2 mapping from red/ir ratio
  static constexpr float    SPO2_A            = 110.0f;
  static constexpr float    SPO2_B            = -25.0f;
  //plot only from [70,100]
  static constexpr float    SPO2_MIN_CLAMP    = 70.0f;
  static constexpr float    SPO2_MAX_CLAMP    = 100.0f;

  // hr search band and thersholds
  ////plot only from [36,180]
  static constexpr float    HR_MIN_BPM        = 40.0f;   //30,36
  static constexpr float    HR_MAX_BPM        = 180.0f;  //200
  static constexpr float    HR_SEARCH_LO_HZ   = 0.60f; //0.6*60=36bpm
  static constexpr float    HR_SEARCH_HI_HZ   = 3.00f; //3*600=180bpm
  static constexpr float    INTERP_DELTA_CLAMP= 0.5f;  // sub-bin shift 
  static constexpr float    IR_AC_NORM_MIN    = 0.00015f; // AC/DC min, if not signal not strong, ignore
  static constexpr float    FS_MIN_CLAMP      = 60.0f;
  static constexpr float    FS_MAX_CLAMP      = 140.0f;

  // max30102 hardware settings
  static constexpr uint8_t  LED_BRIGHTNESS    = 64;
  static constexpr uint8_t  SAMPLE_AVG        = 1;
  static constexpr uint8_t  LED_MODE          = 2;     // red + ir
  static constexpr uint16_t PULSE_WIDTH       = 411;
  static constexpr uint16_t ADC_RANGE         = 16384;
  static constexpr uint8_t  IR_PULSE_AMPL     = 0x24;
  static constexpr uint8_t  RED_PULSE_AMPL    = 0x24;

  bool begin(){
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mx.begin(Wire)) return false;

    // configure led and timing
    mx.setup(LED_BRIGHTNESS, SAMPLE_AVG, LED_MODE, SAMPLE_RATE_HZ, PULSE_WIDTH, ADC_RANGE);
    mx.setPulseAmplitudeIR(IR_PULSE_AMPL);
    mx.setPulseAmplitudeRed(RED_PULSE_AMPL);

    // clear buffers
    for (int i=0;i<WIN_SIZE;i++){ irRaw[i]=redRaw[i]=0; irFilt[i]=redFilt[i]=0; }

    // start timers
    lastSample = lastHRcomp = fs_t0_ms = millis();
    fs_samples = 0;

    enabled = true;
    return true;
  }

  void update(){
    if (!enabled) return;
    const uint32_t now = millis();
    const uint16_t DT_MS = (uint16_t)(1000UL / SAMPLE_RATE_HZ); // exact 10 ms step

    // sample at 100 hz
    if (now - lastSample >= DT_MS){ lastSample = now; pushSample(); }

    // compute hr/spo2 
    if (now - lastHRcomp >= HR_COMPUTE_MS){
      lastHRcomp = now;

      // spo2 from ratio of ac/dc
      computeRatioAndSpO2(lastRatio, lastSpO2, irDC, redDC, irRMS, redRMS);

      // hr from ir only using fft 
      float bpm_raw;
      const bool ok = computeHR_IR(irDC, irRMS, bpm_raw);
      hrRaw = ok ? bpm_raw : -1.0f;
      if (ok) smoothHR_push(bpm_raw);
      else {
        // keep history for smoothing so hr stays stable
        hrHist[2]=hrHist[1]; hrHist[1]=hrHist[0];
        hrHist[0] = (hrSmooth > 0) ? hrSmooth : -1.0f;
        hrSmooth  = -1.0f;
      }
    }
  }

  //numbers for screen/csv
  bool  enabled=false;
  float lastRatio=NAN, lastSpO2=NAN;
  float irDC=0, redDC=0, irRMS=0, redRMS=0;
  float hrSmooth=-1, hrRaw=-1;

private:
  MAX30105 mx;

  //  buffers (raw and filtered)
  float irRaw[WIN_SIZE], redRaw[WIN_SIZE];
  float irFilt[WIN_SIZE], redFilt[WIN_SIZE];
  uint16_t head=0;

  // filter states
  float ir_bp1=0, ir_bp2=0, ir_lp=0;
  float rd_bp1=0, rd_bp2=0, rd_lp=0;

  //  timers
  uint32_t lastSample=0, lastHRcomp=0;

  // effective sample rate estimation
  uint32_t fs_t0_ms=0, fs_samples=0;

  // hr smoothing history 
  float hrHist[3] = {-1.0f, -1.0f, -1.0f};

  static inline float hann(int i, int N){ return 0.5f*(1.0f - cosf(2.0f*PI*i/(N-1))); }
  static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

  void pushSample(){
    // read raw
    float ir  = mx.getIR();  if (ir  < 0) ir  = 0;
    float red = mx.getRed(); if (red < 0) red = 0;

    // store raw
    irRaw[head]=ir; redRaw[head]=red;

    // high-pass, low-pass to get ac
    // HPF y[n] = alpha * ( y[n-1] + x[n] - x[n-1] )
    // LPF z[n] = z[n-1] + beta * ( y[n] - z[n-1] )
    float ir_x  = ir  - irRaw[(head + WIN_SIZE - 1) % WIN_SIZE];
    float ir_hp = HPF_ALPHA * (ir_bp1 + ir_x - ir_bp2);
    ir_bp2 = ir_bp1; ir_bp1 = ir_x;
    ir_lp += LPF_BETA * (ir_hp - ir_lp);
    irFilt[head] = ir_lp;

    float rd_x  = red - redRaw[(head + WIN_SIZE - 1) % WIN_SIZE];
    float rd_hp = HPF_ALPHA * (rd_bp1 + rd_x - rd_bp2);
    rd_bp2 = rd_bp1; rd_bp1 = rd_x;
    rd_lp += LPF_BETA * (rd_hp - rd_lp);
    redFilt[head] = rd_lp;

    head = (head + 1) % WIN_SIZE;

    // update actual fs every 10 s
    fs_samples++;
    uint32_t dt = millis() - fs_t0_ms;
    if (dt > 10000) { fs_t0_ms = millis(); fs_samples = 0; }
  }

  void computeRatioAndSpO2(float& ratio, float& spo2,
                           float& irDCout, float& redDCout,
                           float& irRMSout, float& redRMSout){
    // dc (means) and ac rms over the window
    double sumIR=0, sumRED=0, sumIR2=0, sumRED2=0;
    for (int i=0;i<WIN_SIZE;i++){
      sumIR += irRaw[i]; sumRED += redRaw[i];
      sumIR2 += (double)irFilt[i]*irFilt[i];
      sumRED2 += (double)redFilt[i]*redFilt[i];
    }
    irDCout  = (float)(sumIR  / WIN_SIZE);
    redDCout = (float)(sumRED / WIN_SIZE);
    irRMSout = sqrtf((float)(sumIR2 / WIN_SIZE));
    redRMSout= sqrtf((float)(sumRED2 / WIN_SIZE));

    // threshold on finger present using dc
    bool finger = (irDCout >= MIN_FINGER_IR && redDCout >= MIN_FINGER_RED);
    if (!finger){ ratio=NAN; spo2=NAN; return; }

    // ratio of ratios
    // R = (AC_red/DC_red) / (AC_ir/DC_ir)
    ratio = (redRMSout / redDCout) / (irRMSout / irDCout);

    // SpO2 = A + B * R
    float s = SPO2_A + SPO2_B * ratio;
    if (s < SPO2_MIN_CLAMP) s = SPO2_MIN_CLAMP;
    if (s > SPO2_MAX_CLAMP) s = SPO2_MAX_CLAMP;
    spo2 = s;
  }

  bool hr_fft_ir(float Fs_eff, float& bpm_fft){
    bpm_fft = -1.0f;

    // windowed fft on filtered ir
    float rv[WIN_SIZE], iv[WIN_SIZE];
    for (int i=0;i<WIN_SIZE;i++){ int idx=(head+i)%WIN_SIZE; rv[i]=irFilt[idx]*hann(i,WIN_SIZE); iv[i]=0.0f; }
    ArduinoFFT<float> fft(rv, iv, WIN_SIZE, Fs_eff);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    // search only heart band
    // Frequency bin index k maps to f = k * Fs_eff / N
    int kMin = (int)floorf(HR_SEARCH_LO_HZ * WIN_SIZE / Fs_eff);
    int kMax = (int)ceilf (HR_SEARCH_HI_HZ * WIN_SIZE / Fs_eff);
    if (kMin < 1) kMin = 1;
    if (kMax > (WIN_SIZE/2 - 1)) kMax = WIN_SIZE/2 - 1;
    if (kMax <= kMin) return false;
    // pick best bin in the band
    int pk=kMin; float pkVal=0.0f;
    for (int k=kMin;k<=kMax;k++){ if (rv[k] > pkVal){ pkVal=rv[k]; pk=k; } }

    // parabolic interp around peak to inc eff resolution
    //   delta = 0.5*(a - c)/(a - 2b + c),  a=|X[k-1]|, b=|X[k]|, c=|X[k+1]|
    // https://www.dsprelated.com/freebooks/sasp/Quadratic_Interpolation_Spectral_Peaks.html
    float a=(pk>kMin)?rv[pk-1]:0.0f, b=rv[pk], c=(pk<kMax)?rv[pk+1]:0.0f;
    float denom=(a - 2.0f*b + c);
    float delta=(fabsf(denom)>1e-6f)? 0.5f*(a - c)/denom : 0.0f;
    if (delta> INTERP_DELTA_CLAMP) delta= INTERP_DELTA_CLAMP;
    if (delta<-INTERP_DELTA_CLAMP) delta=-INTERP_DELTA_CLAMP;

    // convert to bpm and clamp to range
    float bpm = (pk + delta) * Fs_eff * 60.0f / (float)WIN_SIZE;
    if (bpm < HR_MIN_BPM || bpm > HR_MAX_BPM) return false;

    bpm_fft = bpm; return true;
  }

  bool computeHR_IR(float irDC_in, float irRMS_in, float& bpm_out){
    bpm_out = -1.0f;

    // basic input gates
    if (irDC_in <= 0) return false;
    float acnorm = irRMS_in / irDC_in;
    if (acnorm < IR_AC_NORM_MIN) return false;

    // estimate effective fs 
    float Fs_eff = (float)SAMPLE_RATE_HZ;
    if (USE_FS_EFF){
      uint32_t dt_ms = millis() - fs_t0_ms;
      if (dt_ms > 0) Fs_eff = 1000.0f * (float)fs_samples / (float)dt_ms;
    }
    Fs_eff = clampf(Fs_eff, FS_MIN_CLAMP, FS_MAX_CLAMP);

    //if the FFT-based estimate is invalid, return false.
    float bpm_fft;
    if (hr_fft_ir(Fs_eff, bpm_fft)) { bpm_out=bpm_fft; return true; }
    return false;
  }

  void smoothHR_push(float bpm_raw){
    // simple iir using last two values for stability
    // hrSmooth = 0.50*new + 0.25*prev1 + 0.25*prev2
    float p1 = (hrHist[0] > 0) ? hrHist[0] : 0.0f;
    float p2 = (hrHist[1] > 0) ? hrHist[1] : 0.0f;
    hrSmooth  = 0.50f*bpm_raw + 0.25f*p1 + 0.25f*p2;
    hrHist[2] = hrHist[1];
    hrHist[1] = hrHist[0];
    hrHist[0] = hrSmooth;
  }
};

ppgcalc ppg;
float hr=-1, spo2=-1; 

// ui layout on the screen
const int X_LABEL = 10;
const int X_VALUE = 10;

const int URL_Y         = 8;     // small font line
const int MODE_LABEL_Y  = 32;    // big font from here
const int MODE_VALUE_Y  = 50;

const int WIFI_LABEL_Y  = 74;
const int WIFI_VALUE_Y  = 92;

const int HR_LABEL_Y    = 116;
const int HR_VALUE_Y    = 134;

const int APNEA_LABEL_Y = 158;
const int APNEA_VALUE_Y = 176;

const int SPO2_LABEL_Y  = 200;
const int SPO2_VALUE_Y  = 218;

// last shown state to avoid unnecessary flickers on screen
uint8_t  lastShownMode  = 0xFF;
String   lastShownURL   = "";
bool     lastWiFiOK     = false;
float    lastHRshown    = -999;
float    lastSpO2shown  = -999;

// clear one value line
inline void clearValueLine(int y){ screen.fillRect(X_VALUE, y - 2, 230, 22, COLOR_RGB565_BLACK); }

// mdns(start once after wifi connects)
bool startMDNSIfNeeded() {
  if (mdnsStarted) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!MDNS.begin(MDNS_HOST)) { mdnsStarted = false; return false; }
  MDNS.addService("http", "tcp", 80);         //  web ui
  MDNS.addService("sleep", "tcp", TCP_PORT);  //  tcp stream
  mdnsStarted = true;
  return true;
}
void restartMDNS() {
  if (mdnsStarted) { MDNS.end(); mdnsStarted = false; }
  startMDNSIfNeeded();
}

// wifi 
void beginWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); //enough for close range
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWiFiAttempt = now_ms();
}
void ensureWiFi() {
  // if connected, keep mdns up and return
  if (WiFi.status() == WL_CONNECTED) { startMDNSIfNeeded(); return; }

  // not connected, reconnect attempts
  if (now_ms() - lastWiFiAttempt < WIFI_RETRY_MS) return;

  // try reconnect
  WiFi.disconnect(true, true);
  delay(200);
  beginWiFi();

  // short wait to see if connected
  uint32_t t0 = now_ms();
  while (WiFi.status() != WL_CONNECTED && now_ms()-t0 < 7000) delay(100);
  if (WiFi.status() == WL_CONNECTED) { restartMDNS(); }
}

// screen ui drawing (static parts once)

void drawStaticUI() {
  screen.fillScreen(COLOR_RGB565_BLACK);
  screen.setTextWrap(false);

  // labels
  screen.setTextSize(1);
  screen.setTextColor(COLOR_RGB565_WHITE);
  screen.setCursor(X_LABEL, URL_Y);  screen.print("Url:");

  // 
  lastShownURL = "";  // first update to write the IP

  screen.setTextSize(2);
  screen.setTextColor(COLOR_RGB565_WHITE);

  screen.setCursor(X_LABEL, MODE_LABEL_Y);  screen.print("Mode:");  clearValueLine(MODE_VALUE_Y);
  screen.setCursor(X_LABEL, WIFI_LABEL_Y);  screen.print("Wifi:");  clearValueLine(WIFI_VALUE_Y);
  screen.setCursor(X_LABEL, HR_LABEL_Y);    screen.print("HR:");    clearValueLine(HR_VALUE_Y);
  screen.setCursor(X_LABEL, APNEA_LABEL_Y); screen.print("Apnea Count:"); clearValueLine(APNEA_VALUE_Y);
  screen.setCursor(X_LABEL, SPO2_LABEL_Y);  screen.print("spo2 (%):"); clearValueLine(SPO2_VALUE_Y);

  // first redo for these fields
  lastShownMode = 0xFF;
  lastWiFiOK    = !(WiFi.status() == WL_CONNECTED);  // <-- key change
  lastHRshown   = -999;
  lastSpO2shown = -999;
}


// screen ui dynamic updates (only change when value changes)
void updateUI() {
  // update url if ip changed
  String urlNow = String("http://") + WiFi.localIP().toString() + "/";
  if (urlNow != lastShownURL) {
    screen.setTextSize(1);
    screen.fillRect(X_LABEL + 40, URL_Y - 1, 190, 10, COLOR_RGB565_BLACK);
    screen.setCursor(X_LABEL + 40, URL_Y);
    screen.setTextColor(COLOR_RGB565_CYAN);
    screen.print(urlNow);
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastShownURL = urlNow;
  }

  screen.setTextSize(2);

  // mode line
  if (lastShownMode != currentMode) {
    clearValueLine(MODE_VALUE_Y);
    screen.setCursor(X_VALUE, MODE_VALUE_Y);
    screen.setTextColor(currentMode==1 ? COLOR_RGB565_GREEN : COLOR_RGB565_ORANGE);
    screen.print(currentMode==1 ? "mode-1 (2hz)" : "mode-2 (1hz)");
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastShownMode = currentMode;
  }

  // wifi status
  bool wifiOK = (WiFi.status() == WL_CONNECTED);
  if (wifiOK != lastWiFiOK) {
    clearValueLine(WIFI_VALUE_Y);
    screen.setCursor(X_VALUE, WIFI_VALUE_Y);
    screen.setTextColor(wifiOK ? COLOR_RGB565_GREEN : COLOR_RGB565_RED);
    screen.print(wifiOK ? "connected" : "disconnected");
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastWiFiOK = wifiOK;
  }

  // hr value
  if (fabs(hr - lastHRshown) > 0.1f) {
    clearValueLine(HR_VALUE_Y);
    screen.setCursor(X_VALUE, HR_VALUE_Y);
    if (hr > 0) {
      screen.setTextColor(COLOR_RGB565_GREEN);
      screen.print(hr, 1);
    } else {
      screen.setTextColor(COLOR_RGB565_DGRAY);
      screen.print("--");
    }
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastHRshown = hr;
  }

  // spo2 value
  if (fabs(spo2 - lastSpO2shown) > 0.1f) {
    clearValueLine(SPO2_VALUE_Y);
    screen.setCursor(X_VALUE, SPO2_VALUE_Y);
    if (spo2 > 0) {
      screen.setTextColor(COLOR_RGB565_CYAN);
      screen.print(spo2, 1);
    } else {
      screen.setTextColor(COLOR_RGB565_DGRAY);
      screen.print("--");
    }
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastSpO2shown = spo2;
  }

  // apnea count (update when it changes)
  static uint32_t lastShownApnea = 0xFFFFFFFF;
  if (apneaCount != lastShownApnea){
    clearValueLine(APNEA_VALUE_Y);
    screen.setCursor(X_VALUE, APNEA_VALUE_Y);
    screen.setTextColor(COLOR_RGB565_YELLOW);
    screen.print(apneaCount);
    screen.setTextColor(COLOR_RGB565_WHITE);
    lastShownApnea = apneaCount;
  }
}

//  web ui for mode switch
//   GET "/"     -> HTML mode control page
//   GET "/cur"  -> current mode {"mode": <1|2>}
//   GET "/mode?m=1|2" -> switches mode
const char* HTML_INDEX =
"<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Sleep Apnea Monitoring</title>"
"<style>"
"body{font-family:sans-serif;background:#111;color:#eee;margin:20px}"
"button{padding:10px 20px;margin:5px;border:none;border-radius:8px;cursor:pointer}"
".m1{background:#2d7;color:#fff}.m2{background:#a60;color:#fff}"
".card{background:#1b1b1b;padding:14px;border-radius:10px;margin-top:12px;line-height:1.4}"
"small{color:#bbb}"
"</style></head><body>"
"<h1>Mode Control</h1>"
"<p>Current mode: <span id='mode'>--</span></p>"
"<button class='m1' onclick='setMode(1)'>mode-1 (2&nbsp;Hz)</button>"
"<button class='m2' onclick='setMode(2)'>mode-2 (1&nbsp;Hz)</button>"
"<div class='card'>"
"<b>What the modes do</b><br>"
"<b>Mode-1 (2&nbsp;Hz)</b>: Higher logging rate (updates every 0.5&nbsp;s) and Microphone turned on. "
" Screen UI, Live plots and Data logging rows are produced faster, uses slightly more power.<br>"
"<b>Mode-2 (1&nbsp;Hz)</b>: Minimal logging (updates every 1&nbsp;s) with Micropgone turned off. "
" Slower screen updates, no snoring detection.<br>"
"</div>"
"<p id='msg'></p>"
"<script>"
"async function setMode(m){const r=await fetch('/mode?m='+m);document.getElementById('msg').textContent=await r.text();}"
"async function poll(){try{const r=await fetch('/cur');const j=await r.json();document.getElementById('mode').textContent=j.mode;}catch(e){}setTimeout(poll,1000);}poll();"
"</script></body></html>";


void handle_root() { server.send(200, "text/html", HTML_INDEX); }
void handle_cur()  { server.send(200, "application/json", String("{\"mode\":")+String(currentMode)+"}"); }
void handle_mode() {
  // read m=1 or m=2 and update mode
  if (!server.hasArg("m")) { server.send(400, "text/plain", "missing m"); return; }
  int m = server.arg("m").toInt();
  if (m!=1 && m!=2) { server.send(400, "text/plain", "m must be 1 or 2"); return; }
  currentMode = (uint8_t)m;
  cfg = (currentMode==1)?MODE1:MODE2;

  // broadcast mode set to all (esp1 and esp3)
  SyncPacket sp{}; sp.msgType = 2; sp.targetDev = 0; sp.epoch_ms = millis(); sp.mode = currentMode;
  udpSync.beginPacket(IPAddress(255,255,255,255), PORT_SYNC); //broadccast addr:255,255,255,255
  udpSync.write((uint8_t*)&sp, sizeof(sp));
  udpSync.endPacket();

  server.send(200, "text/plain", String("switched to mode-") + (int)currentMode);
}

// rx and ack 
// for every valid data packet:
//   1) copy data
//   2) reset lastESP*_ms 
//   3) send AckPacket back to source IP:PORT_ACK with seqAck=hdr.seq.
// nodes use this to stop resending.
void handleUdpIn(uint8_t* buf, int got, const IPAddress& from){
  // need at least header
  if (got < (int)sizeof(PacketHeader)) return;

  // copy header 
  PacketHeader hdr; memcpy(&hdr, buf, sizeof(hdr));
  uint8_t* payload = buf + sizeof(PacketHeader);

  // route by device id and payload size
  if (hdr.dev_id == 1 && hdr.payload_len >= sizeof(SensorPayload)) {
    memcpy(&imuData, payload, sizeof(SensorPayload));
    lastESP1_ms = now_ms();
  } else if (hdr.dev_id == 3 && hdr.payload_len >= sizeof(MicTempPayload)) {
    bool prev = micData.apnea;
    memcpy(&micData, payload, sizeof(MicTempPayload));
    lastESP3_ms = now_ms();
    if (micData.apnea && !prev) apneaCount++; // count rising edge pulses
  }

  // send ack back to sender ip:port
  AckPacket ack{ hdr.dev_id, hdr.seq };
  udpAck.beginPacket(from, PORT_ACK);
  udpAck.write((uint8_t*)&ack, sizeof(ack));
  udpAck.endPacket();
}


// csv logging
// CSV 
//   Date, Time, Mode,
//   IMU: ax..gz, Posture, Motion, Env, Ratio, RMS, ZC, Effort, Why, Conf,
//   Mic: TempC, EnvT, RatioT, RMST, ZCT, EffortT, WhyT, ConfT, Snore, DFreq, Mag, Amp, Apnea,
//   PPG: IRdc, REDdc, SpO2, HR, ApneaCount
// -1 if no data
String twoDigits(int v){ return (v<10) ? "0"+String(v) : String(v); }

void ensureCsvHeader() {
  // create header once if file is new
  if (!sdOk) return;
  File f = SD.open("/esp_log.csv", FILE_READ);
  bool need = true;
  if (f) { if (f.size()>0) need=false; f.close(); }
  if (need) {
    File w = SD.open("/esp_log.csv", FILE_WRITE);
    if (w) {
      w.println("Date,Time,Mode,"
                "ax,ay,az,gx,gy,gz,Posture,Motion,Env,Ratio,RMS,ZC,Effort,Why,Conf,"
                "TempC,EnvT,RatioT,RMST,ZCT,EffortT,WhyT,ConfT,Snore,DFreq,Mag,Amp,Apnea,"
                "IRdc,REDdc,SpO2,HR,ApneaCount");
      w.close();
    }
  }
}

String makeRow() {
  // read rtc 
  DateTime now = rtc.now();
  String dateStr = String(now.day()) + "-" + String(now.month()) + "-" + String(now.year());
  String timeStr = twoDigits(now.hour()) + ":" + twoDigits(now.minute()) + ":" + twoDigits(now.second());

  // check if latest samples are fresh 
  bool imuValid = (lastESP1_ms && (now_ms()-lastESP1_ms)<=MAX_STALE_MS);
  bool micValid = (lastESP3_ms && (now_ms()-lastESP3_ms)<=MAX_STALE_MS);

  // start row
  String row = dateStr + "," + timeStr + "," + String((int)currentMode) + ",";

  // imu block
  if (imuValid)
    row += String(imuData.ax,2)+","+String(imuData.ay,2)+","+String(imuData.az,2)+","+
           String(imuData.gx,2)+","+String(imuData.gy,2)+","+String(imuData.gz,2)+","+
           String(imuData.posture)+","+String(imuData.motion)+","+
           String(imuData.env,3)+","+String(imuData.ratio,3)+","+String(imuData.rms,3)+","+String((int)imuData.zc)+","+
           String((int)imuData.effort)+","+String(whyStr(imuData.why))+","+String(imuData.conf,2)+",";
  else
    row += "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-,-1,";

  // mic + temp block
  if (micValid)
    row += String(micData.tempC,1)+","+String(micData.env,3)+","+String(micData.ratio,3)+","+String(micData.rms,3)+","+String((int)micData.zc)+","+
           String((int)micData.effort)+","+String(whyStr(micData.why))+","+String(micData.conf,2)+","+
           String((int)micData.snoring)+","+String(micData.dominantFreq,1)+","+String(micData.maxMagnitude,1)+","+String(micData.avgAmplitude)+","+
           String((int)micData.apnea)+",";
  else
    row += "-1,-1,-1,-1,-1,-1,-,-1,-1,-1,-1,-1,-1,";

  // ppg block
  row += String(ppg.irDC,1)+","+String(ppg.redDC,1)+","+ String(spo2,1)+","+String(hr,1)+","+String(apneaCount);
  return row;
}

void appendCsv(const String& row) {
  // append a line to csv if sd ok
  if (!sdOk) return;
  File f = SD.open("/esp_log.csv", FILE_APPEND);
  if (f) { f.println(row); f.close(); }
}

// setup
void setup() {
  Serial.begin(115200);
  delay(150);

  // screen start and clear
  screen.begin();
  screen.fillScreen(COLOR_RGB565_BLACK);

  // i2c + rtc
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!rtc.begin()) Serial.println("rtc failed");
  // if rtc lost power, set to compile time once
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // sd card
  if (SD.begin(SD_CS)) { sdOk = true; ensureCsvHeader(); }

  // ppg sensor
  ppgOk = ppg.begin();

  // wifi connect and mdns
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWiFiAttempt = now_ms();
  {
    uint32_t t0 = now_ms();
    while (WiFi.status() != WL_CONNECTED && now_ms()-t0 < 20000) delay(100);
    if (WiFi.status() == WL_CONNECTED) { startMDNSIfNeeded(); }
  }

  // udp + web server
  //three HTTP routes
  udpRx.begin(PORT_TELEMETRY);
  server.on("/", handle_root); //registers the root page (HTML UI) used to view/change mode from a browser.
  server.on("/cur", handle_cur);// returns the current mode as JSON ({"mode":1/2})
  server.on("/mode", handle_mode);//registers the mode-switch endpoint, /mode?m=1|2 updates currentMode and broadcasts a SYNC/MODE_SET so nodes follow
  server.begin();

  // tcp stream (optional)
  tcpServer.begin();

  // draw first ui
  drawStaticUI();
  updateUI();

  // start timers
  lastSync = lastLog = lastUi = now_ms();
}

void loop() {
  // wifi and mdns 
  ensureWiFi();

  // http requests
  server.handleClient();

  // udp receive and ack
  {
    int sz = udpRx.parsePacket();
    if (sz > 0) {
      IPAddress from = udpRx.remoteIP();
      static uint8_t buf[512];
      if (sz > (int)sizeof(buf)) sz = sizeof(buf);
      int got = udpRx.read(buf, sz);
      if (got > 0) handleUdpIn(buf, got, from);
    }
  }

  // broadcast sync every second 
  if (now_ms() - lastSync >= SYNC_PERIOD_MS) {
    lastSync = now_ms();
    SyncPacket sp{}; sp.msgType = 1; sp.targetDev = 0; sp.epoch_ms = millis(); sp.mode = currentMode;
    udpSync.beginPacket(IPAddress(255,255,255,255), PORT_SYNC); //broadcast addr:(255,255,255,255)
    udpSync.write((uint8_t*)&sp, sizeof(sp));
    udpSync.endPacket();
  }

  // run ppg processing
  ppg.update();

  //  hr and spo2 for ui and csv
  //  smoothed HR (if valid) and last spo2(or -1 if invalid).
  static uint32_t lastExpose=0;
  if (now_ms() - lastExpose >= cfg.hub_tick_ms){
    lastExpose = now_ms();
    hr   = (ppg.hrSmooth > 0) ? ppg.hrSmooth : -1;
    spo2 = isnan(ppg.lastSpO2) ? -1 : ppg.lastSpO2;
  }

  // refresh screen every 500 ms
  if (now_ms()-lastUi >= 500) { lastUi = now_ms(); updateUI(); }

  // csv and tcp 
  if (now_ms()-lastLog >= cfg.hub_tick_ms) {
    lastLog = now_ms();
    String row = makeRow();
    appendCsv(row);

    // tcp client, if connected, send the row
    if (!tcpClient || !tcpClient.connected()) tcpClient = tcpServer.available();
    if (tcpClient && tcpClient.connected())   tcpClient.println(row);
  }
}
