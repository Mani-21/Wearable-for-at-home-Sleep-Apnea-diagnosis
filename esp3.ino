/*
  sleep apnea project
  esp3 mic and thermistor node
  version 14

    using arduino 1.8.19
  ref:
  https://lastminuteengineers.com/esp32-mdns-tutorial/
  https://www.aranacorp.com/en/communication-between-two-esp32s-via-udp/
  https://docs.arduino.cc/language-reference/en/functions/wifi/udp/
  https://docs.arduino.cc/language-reference/?_gl=1*19wvmhw*_up*MQ..*_ga*MTM4MTEzMDgwMy4xNzU2MzA2NDY5*_ga_NEXN8H46L5*czE3NTYzMDY0NjckbzEkZzEkdDE3NTYzMDY1MzAkajYwJGwwJGg2NjgyMzY2NTU.#functions
  https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/i2s.html
  https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
  https://docs.sunfounder.com/projects/esp32-starter-kit/en/latest/micropython/basic_projects/py_thermistor.html
  
*/

// forward declare packet types to avoid compilation error
struct PacketHeader;
struct MicTempPayload;
struct AckPacket;
struct SyncPacket;

// libs
#include <Arduino.h>     
#include <WiFi.h>        // wifi connect
#include <WiFiUdp.h>     // udp 
#include <ESPmDNS.h>     // mdns 
#include <driver/i2s.h>  // i2s driver for mic
#include <arduinoFFT.h>  // fft for snore band features
#include <math.h>        // math 
#include <stddef.h>     

// wifi
#define WIFI_SSID  "VM2C2F68"        // wifi username  // "VM2C2F68"  // "Mani"
#define WIFI_PASS  "dtUttDw2tqnu"   // wifi password  // "dtUttDw2tqnu" //"manikanta"
static const char* DEV_HOSTNAME = "esp3";      // this device
static const char* HUB_MDNS     = "sleep-hub"; // hub mdns name to find

// udp ports used by the project
static const uint16_t PORT_TELEMETRY = 5005; // esp3 -> hub data
static const uint16_t PORT_ACK       = 5006; // hub -> esp3 ack
static const uint16_t PORT_SYNC      = 5007; // hub -> all sync/mode

// this device identity
static const uint8_t  DEV_ID         = 3;    // 3 means mic+therm node

// mode and timing
// mode controls send rate and whether mic is on
struct ModeConfig { uint8_t id; uint16_t send_interval_ms; bool mic_on; };
static const ModeConfig MODE1 = { 1, 500,  true  }; // 2 hz send, mic on
static const ModeConfig MODE2 = { 2, 1000, false }; // 1 hz send, mic off
static const uint8_t DEFAULT_MODE = 1;              // start in mode 1

// ack and resend policy (simple)
static const uint32_t ACK_WAIT_MS        = 150; // wait before retry
static const uint8_t  MAX_RESEND         = 2;   // max resend count
// small random delay to avoid both esp1 and esp3 colliding on esp2
static const uint16_t SEND_JITTER_MS_MAX = 40;  // jitter range in ms

// thermistor config
// ntc divider: ntc -> vcc, fixed r_ref -> gnd
static const int  THERM_ADC_PIN     = 35;      // adc1 ch7 
static const int  ADC_BITS          = 12;      // esp32 12-bit adc
static const int  ADC_MAX_VAL       = (1<<ADC_BITS)-1;
static const float R_REF            = 10000.0f; // 10k to gnd
static const float R0               = 10000.0f; // ntc 10k at 25c
static const float T0_K             = 298.15f;  // 25 c in kelvin
static const float BETA             = 3950.0f;  // beta constant of ntc
#define REPORT_ABSOLUTE_TEMP 1                 // report absolute deg c
static const float TEMP_ABS_LP_TAU_S = 1.5f;   // lp smoothing on abs temp
#define USE_MEDIAN3 1                          // median of 3 adc reads

// time since boot in ms
static inline uint32_t now_ms(){ return millis(); }

//same packet copied from esp1
struct PacketHeader {
  uint8_t  dev_id;       // which node sent this
  uint8_t  mode;         // sender mode at time of send
  uint16_t payload_len;  // number of bytes after header
  uint32_t seq;          // increasing sequence
  uint32_t t_ms;         // sender millis at send time
};


struct MicTempPayload {
  // mic features from 8 khz / 1024 fft window
  int32_t avgAmplitude;  // mean absolute amplitude (scaled)
  float   maxMagnitude;  // strongest fft magnitude in band
  float   dominantFreq;  // freq of that strongest bin (hz)
  uint8_t snoring;       // 1 if in snore band and above thresholds

  // thermistor breathing features (low-pass band)
  float   tempC;         // absolute deg c (smoothed) 
  float   env, ratio, rms; // envelope and simple oscillation stats
  uint8_t zc;            // zero-crossing count in small window
  uint8_t effort;        // 1 when breathing effort detected
  uint8_t why;           
  float   conf;          

  // apnea indicator
  uint8_t apnea;         
};

//  ack from hub 
struct AckPacket { uint8_t dev_id; uint32_t seqAck; };

// sync from hub
struct SyncPacket { uint8_t msgType; uint8_t targetDev; uint32_t epoch_ms; uint8_t mode; };

// therm breath detector
// parameters for sampling, filters and decision logic
struct ThermParams {
  uint16_t sample_hz       = 25;     // read adc 25 times per second
  uint16_t analysis_hz     = 2;      // update detector 2 times per second
  bool  use_hpf            = true;   // high-pass to remove slow drift
  float temp_lp_hz         = 0.8f;   // low-pass cutoff for ac band
  float temp_hp_hz         = 0.02f;  // high-pass cutoff for ac band
  float env_tau_short_s    = 0.8f;   // fast envelope time constant
  float env_tau_long_s     = 25.0f;  // slow envelope time constant
  float abs_env_floor_c    = 0.050f; // absolute envelope floor in deg c
  float ratio_on           = 1.25f;  // fast/slow envelope ratio to turn on
  float ratio_off          = 1.10f;  // ratio to turn off (hysteresis)
  float dc_tau_s           = 0.8f;   // detrend window for oscillation
  int   zc_min             = 2;      // need at least this many zero crosses
  float zc_amp_floor_c     = 0.020f; // ignore tiny oscillations
  float rms_floor_c        = 0.030f; // minimum rms to count as oscillation
  float quiet_env_mult     = 1.2f;   // very quiet envelope, default off
  //same as imu
  int   eff_on_confirm_s   = 1;      // seconds of ok before turning on
  int   noeff_confirm_s    = 1;      // seconds of not ok before turning off
};

class ThermBreath {
public:
  // last computed values exposed for packet fill
  float lastTemp=0, lastTempAbs=0, lastEnv=0, lastRatio=0, lastRMS=0;
  int   lastZC=0; const char* lastWhy="-"; float lastConf=0;
  bool  lastEffort=false; bool apneaOut=false;

  // set up detector and adc pin
  bool begin(const ThermParams& tp, int adcPin){
    P = tp; pin = adcPin;
    // convert rates to millisecond 
    sample_period_ms   = (uint32_t)roundf(1000.0f / P.sample_hz);
    analysis_period_ms = (uint32_t)roundf(1000.0f / P.analysis_hz);

    //  filter alphas from time constants
    // same as imu
    lp_alpha    = alpha_from_tau(1.0f / (2.0f * PI_F * P.temp_lp_hz), P.sample_hz);
    if (P.use_hpf) hp_alpha = alpha_from_tau(1.0f / (2.0f * PI_F * P.temp_hp_hz), P.sample_hz);
    env_alpha_short = alpha_from_tau(P.env_tau_short_s, P.sample_hz);
    env_alpha_long  = alpha_from_tau(P.env_tau_long_s , P.sample_hz);
    dc_alpha        = alpha_from_tau(P.dc_tau_s,        P.analysis_hz);
    abs_alpha       = alpha_from_tau(TEMP_ABS_LP_TAU_S, P.sample_hz);

    //  timing and states
    lastSample = lastAnalysis = now_ms();
    seedAbs = true; seedEnv = true;
    return true;
  }

  // run detector in two loops, fast sampling and slower analysis
  void update(){
    uint32_t tnow = now_ms();

    // sample loop at 25 hz
    while ((uint32_t)(tnow - lastSample) >= sample_period_ms){
      lastSample += sample_period_ms;
      
     //random spikes in raw values, so
      // read adc (median-of-3 to reduce single read spikes)
#if USE_MEDIAN3
      uint16_t r0 = analogRead(pin);
      uint16_t r1 = analogRead(pin);
      uint16_t r2 = analogRead(pin);
      uint16_t raw = median3(r0, r1, r2);
#else
      uint16_t raw = analogRead(pin);
#endif
      // convert divider voltage to ntc resistance then to deg c
      float T = adcToTempC(raw);

      // absolute temperature low-pass smoothing
      if (seedAbs){ abs_lp_y = T; seedAbs=false; }
      else        { abs_lp_y += abs_alpha * (T - abs_lp_y); }
      lastTempAbs = abs_lp_y;

      // ac bandpass, hpf then lp, same as imu
      float preLP = T;
      if (P.use_hpf){ hp_lp_y += hp_alpha * (T - hp_lp_y); preLP = T - hp_lp_y; }
      lp_y += lp_alpha * (preLP - lp_y);
      lastTemp = lp_y;

      // envelope for fast and slow, same logic as imu
      float absT = fabsf(lastTemp);
      if (seedEnv){ envShort=absT; envLong=absT; seedEnv=false; }
      envShort += env_alpha_short * (absT - envShort);
      envLong  += env_alpha_long  * (absT - envLong);
    }

    // analysis loop at 2 hz
    if ((uint32_t)(tnow - lastAnalysis) >= analysis_period_ms){
      lastAnalysis += analysis_period_ms;

      // fast/slow envelope ratio threshold
      float base  = fmaxf(envLong, 0.8f * P.abs_env_floor_c);
      float ratio = (base > 1e-6f) ? (envShort / base) : 1.0f;
      bool envelopeOK = (envShort >= P.abs_env_floor_c) && (ratio >= P.ratio_on);

      // detrend the ac to get oscillation around zero, same as imu
      dcShort += dc_alpha * (lastTemp - dcShort);
      float d = lastTemp - dcShort;

      // buffer for zc and rms
      detBuf[detIdx]=d; detIdx=(detIdx+1)%OSC_BUF; if (detCount<OSC_BUF) detCount++;
      int   zc  = countZC(detBuf, detCount, P.zc_amp_floor_c);
      float rms = computeRMS(detBuf, detCount);
      bool oscillationOK = (zc >= P.zc_min) && (rms >= P.rms_floor_c);

      // quiet state reduces false positives
      bool quiet = (envShort < P.quiet_env_mult * P.abs_env_floor_c);

      // state machine with on/off confirms
      if ((envelopeOK || oscillationOK) && !quiet){ effOnCnt++; noEffCnt=0; }
      else { noEffCnt++; effOnCnt=0; }

      bool prevEffort = effortState;
      if (!effortState && effOnCnt >= (P.eff_on_confirm_s * P.analysis_hz)) { effortState=true;  noEffCnt=0; }
      if ( effortState && noEffCnt >= (P.noeff_confirm_s  * P.analysis_hz)) { effortState=false; effOnCnt=0; }

      // apnea pulse, after 10 s of no effort, hold 1.5 s
      if (prevEffort != effortState){
        if (!effortState){ effortOffStart = tnow; apneaSignaled=false; }
        else             { effortOffStart = 0;   apneaSignaled=false; }
      }
      if (!effortState && !apneaSignaled && effortOffStart && (tnow - effortOffStart >= 10000)){
        apneaSignaled  = true;
        apneaHoldUntil = tnow + 1500;
      }
      apneaOut = (apneaHoldUntil && (tnow < apneaHoldUntil));
      if (apneaHoldUntil && tnow >= apneaHoldUntil) apneaHoldUntil = 0;

      // reason text
      const char* why = "-";
      if      (quiet)                          why="Q";
      else if (envelopeOK && oscillationOK)    why="A+B";
      else if (envelopeOK)                     why="A";
      else if (oscillationOK)                  why="B";

      // confidence as is from imu, refer esp-1
      float envMag  = clamp01f( (envShort - P.abs_env_floor_c) / (2.0f * P.abs_env_floor_c) );
      float envRat  = clamp01f( (ratio    - P.ratio_off       ) / (P.ratio_on - P.ratio_off) );
      float envConf = envMag * envRat;
      float oscMag  = clamp01f( (rms      - P.rms_floor_c     ) / (2.0f * P.rms_floor_c) );
      float oscConf = (zc >= P.zc_min ? oscMag : 0.0f);
      float conf    = fmaxf(envConf, oscConf);

      // publish for data packet
      lastEnv=envShort; lastRatio=ratio; lastZC=zc; lastRMS=rms;
      lastWhy=why; lastConf=conf; lastEffort=effortState;
    }
  }

  //convert why string to code
  static uint8_t whyCode(const char* w){
    if (strcmp(w,"A")==0) return 1;
    if (strcmp(w,"B")==0) return 2;
    if (strcmp(w,"A+B")==0) return 3;
    if (strcmp(w,"Q")==0) return 4;
    return 0;
  }

private:
  ThermParams P; int pin=THERM_ADC_PIN;

  // timing for sample and analysis loops
  uint32_t lastSample=0, lastAnalysis=0;
  uint32_t sample_period_ms=40, analysis_period_ms=500;

  // bandpass 
  float lp_y=0, lp_alpha=0;
  float hp_lp_y=0, hp_alpha=0;

  // absolute temp smoothing
  float abs_lp_y=0, abs_alpha=0; bool seedAbs=false;

  // envelope tracking
  float envShort=0, envLong=0; float env_alpha_short=0, env_alpha_long=0; bool seedEnv=true;

  // buffer for osc 
  static const int OSC_BUF=16;
  float detBuf[OSC_BUF]; int detIdx=0, detCount=0;
  float dcShort=0, dc_alpha=0;

  // effort state counters and flags
  int effOnCnt=0, noEffCnt=0; bool effortState=false;

  // apnea pulse timing
  uint32_t effortOffStart=0; bool apneaSignaled=false; uint32_t apneaHoldUntil=0;

  // math 
  static constexpr float PI_F = 3.14159265358979323846f;
  static inline float alpha_from_tau(float tau_s, float fs){ return 1.0f - expf(-1.0f/(tau_s*fs)); }
  static inline float clamp01f(float x){ return (x<0)?0: (x>1)?1:x; }

  // convert adc reading to deg c using beta model
  static float adcToTempC(uint16_t raw){
    float v = (float)raw / (float)ADC_MAX_VAL;          // 0..1 fraction
    v = fminf(fmaxf(v, 1e-9f), 1.0f - 1e-9f);           // avoid div by zero
    float rntc = R_REF * (1.0f - v) / v;                // divider math
    float invT = (1.0f/T0_K) + (1.0f/BETA) * logf(rntc/R0); 
    return (1.0f/invT) - 273.15f;                       // kelvin to c
  }
  // median-of-3 to reject one odd sample
  static uint16_t median3(uint16_t a, uint16_t b, uint16_t c){
    if (a>b) { uint16_t t=a; a=b; b=t; }
    if (b>c) { uint16_t t=b; b=c; c=t; }
    if (a>b) { uint16_t t=a; a=b; b=t; }
    return b;
  }
  // zero crossing counter with amplitude floor
  static int countZC(const float* b, int n, float floor){
    int z=0; for (int i=1;i<n;i++){ float a=b[i], c=b[i-1];
      if ((a>floor && c<-floor) || (a<-floor && c>floor)) z++;
    } return z;
  }
  // rms over small buffer
  static float computeRMS(const float* b, int n){
    if (n<=0) return 0;
    float s=0; for(int i=0;i<n;i++){ s+=b[i]*b[i]; }
    return sqrtf(s/n);
  }
};

// mic and fft
// esp32 i2s pins for the mic module and fft settings
static const int  I2S_WS_PIN        = 17;   // lrcl / ws
static const int  I2S_BCLK_PIN      = 4;    // bclk
static const int  I2S_DATA_PIN      = 16;   // dout to esp32
static const int  MIC_SAMPLE_RATE   = 8000; // 8 khz audio
static const int  FFT_SIZE          = 1024; // window size
// search band for features and snore decision
static const int  FFT_MIN_FREQ      = 500;   // hz
static const int  FFT_MAX_FREQ      = 1500;  // hz
static const int  SNORE_FREQ_MIN    = 600;   // hz
static const int  SNORE_FREQ_MAX    = 1400;  // hz
// thresholds to call it snoring
static const float   THRESHOLD_MAG  = 15000.0f; // fft magnitude
static const int32_t THRESHOLD_AMP  = 10000;    // time-domain amp

// mic state and fft buffers
static bool micEnabled=false;
double vReal[FFT_SIZE], vImag[FFT_SIZE];
ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, MIC_SAMPLE_RATE);

// start the i2s mic in rx mode
void micBegin(){
  if (micEnabled) return;
  i2s_config_t cfg;
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX); // read-only
  cfg.sample_rate = MIC_SAMPLE_RATE;                      // 8 khz
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;        // 32-bit from mic
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;         // single channel
  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;            // default irq
  cfg.dma_buf_count = 4;                                  // small ring
  cfg.dma_buf_len = FFT_SIZE;                             // 1024 samples
  cfg.use_apll = false;                                   // internal clock
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins;
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num  = I2S_WS_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = I2S_DATA_PIN;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return;
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
  micEnabled = true; // flag on
}

// stop the i2s mic and free driver
void micEnd(){
  if (!micEnabled) return;
  i2s_driver_uninstall(I2S_NUM_0);
  micEnabled=false;
}

// run one 1024-sample capture and compute features
void computeMicFeatures(int32_t& avgAmp, float& maxMag, float& domFreq, uint8_t& snore){
  avgAmp=0; maxMag=0; domFreq=0; snore=0;   // defaults
  if (!micEnabled) return;                  // nothing if mic off

  // read one dma buffer worth of samples 
  int32_t samples[FFT_SIZE];
  size_t bytesRead=0;
  i2s_read(I2S_NUM_0, samples, sizeof(samples), &bytesRead, portMAX_DELAY);
  const int N = (int)(bytesRead / sizeof(int32_t));
  if (N <= 0) return;

  // scale down from 32 to int16 range and fill fft buffers
  int64_t sumAbs = 0;
  for (int i=0;i<N;i++){
    int32_t s = samples[i] >> 14;   //16
    vReal[i] = (double)s; vImag[i]=0.0;
    sumAbs += (s >= 0) ? s : -s;
  }
  // zero pad if less than 1024 samples
  for (int i=N;i<FFT_SIZE;i++){ vReal[i]=0.0; vImag[i]=0.0; }
  // mean absolute amplitude
  avgAmp = (N>0) ? (int32_t)(sumAbs / N) : 0;

  // window, fft, magnitude
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // search only within [500,1500] hz band
  const int kMin = (int)((FFT_MIN_FREQ * (float)FFT_SIZE) / MIC_SAMPLE_RATE);
  const int kMax = (int)((FFT_MAX_FREQ * (float)FFT_SIZE) / MIC_SAMPLE_RATE);
  int maxIndex = kMin; double localMax = 0.0;
  for (int k=kMin; k<=kMax; k++){
    if (vReal[k] > localMax){ localMax = vReal[k]; maxIndex = k; }
  }
  maxMag = (float)localMax;
  domFreq = (float)maxIndex * ((float)MIC_SAMPLE_RATE / (float)FFT_SIZE);

  // simple snore flag using band, magnitude and time-domain amp
  if (maxMag > THRESHOLD_MAG &&
      domFreq >= SNORE_FREQ_MIN && domFreq <= SNORE_FREQ_MAX &&
      avgAmp > THRESHOLD_AMP) snore = 1;
  else snore = 0;
}


// comunication
WiFiUDP udpTx, udpAck, udpSync;           // tx socket, ack rx, sync rx
IPAddress hubIP;                          
ModeConfig modeCfg = (DEFAULT_MODE==1)?MODE1:MODE2; // current mode cfg
uint8_t currentMode = DEFAULT_MODE;       // start mode id
uint32_t seq=0, nextSendDue=0;            // seq number and next send tick

// keep last sent packet in case we need to resend on missing ack
uint8_t  lastBuf[sizeof(PacketHeader)+sizeof(MicTempPayload)];
size_t   lastLen=0;
uint32_t lastTxSeq=0, lastTxTime=0;

// connect to wifi and start ack/sync 
void connectWiFi() {
  WiFi.mode(WIFI_STA);               // station mode
  WiFi.setSleep(false);              
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // strongest tx power
  WiFi.disconnect(true, true);       // clean state
  delay(200);
  WiFi.setHostname(DEV_HOSTNAME);    // set  hostname
  WiFi.begin(WIFI_SSID, WIFI_PASS);  // start connect
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  udpAck.begin(PORT_ACK);            // listen for hub ack
  udpSync.begin(PORT_SYNC);          // listen for hub sync
}

// try to reconnect if wifi drops
void maybeReconnectWiFi() {
  static uint32_t lastTry = 0;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastTry < 5000) return; // rate limit
  connectWiFi();
  lastTry = millis();
}

//  hub ip via mdns name "sleep-hub.local"
bool resolveHubMDNS() {
  if (!MDNS.begin(DEV_HOSTNAME)) return false; // start responder (needed to query)
  delay(300);
  IPAddress ip = MDNS.queryHost(HUB_MDNS);
  if ((uint32_t)ip == 0) return false;
  hubIP = ip; return true;
}

ThermBreath therm;  // thermistor breathing detector
ThermParams TP;     // default params

// apply a new mode (update time, toggle mic, schedule next send)
void applyMode(uint8_t mode) {
  currentMode = mode;
  modeCfg = (mode==2)?MODE2:MODE1;
  if (modeCfg.mic_on && !micEnabled) micBegin();
  if (!modeCfg.mic_on && micEnabled) micEnd();
  nextSendDue = now_ms() + modeCfg.send_interval_ms
              + (SEND_JITTER_MS_MAX ? random(0, SEND_JITTER_MS_MAX) : 0);
}

void sendPacket(const MicTempPayload& pl){
  PacketHeader hdr; hdr.dev_id = DEV_ID; hdr.mode = currentMode;
  hdr.payload_len = (uint16_t)sizeof(pl); hdr.seq = seq++; hdr.t_ms = now_ms();

  // copy header and payload into lastBuf so we can resend if needed
  memcpy(lastBuf, &hdr, sizeof(hdr));
  memcpy(lastBuf+sizeof(hdr), &pl, sizeof(pl));

  // send to hub ip 
  udpTx.beginPacket(hubIP, PORT_TELEMETRY);
  udpTx.write(lastBuf, sizeof(hdr)+sizeof(pl));
  udpTx.endPacket();

  // remember send info for retry
  lastLen = sizeof(hdr)+sizeof(pl);
  lastTxSeq = hdr.seq;
  lastTxTime = now_ms();
}

// check ack socket and resend once or twice if needed
void maybeResend(){
  if (!lastLen) return; // nothing pending

  // drain any ack packets and see if ours is acknowledged
  int sz = udpAck.parsePacket();
  while (sz >= (int)sizeof(AckPacket)){
    AckPacket ack; udpAck.read((uint8_t*)&ack, sizeof(ack));
    if (ack.seqAck == lastTxSeq){ lastLen=0; return; } // done
    sz = udpAck.parsePacket(); 
  }

  // not acked yet,wait a bit before retry
  if (now_ms() - lastTxTime < ACK_WAIT_MS) return;

  // do resends
  static uint8_t retries=0;
  if (retries < MAX_RESEND){
    udpTx.beginPacket(hubIP, PORT_TELEMETRY);
    udpTx.write(lastBuf, lastLen);
    udpTx.endPacket();
    retries++;
    lastTxTime = now_ms();
  } else {
    // give up on this packet and clear state
    retries=0; lastLen=0;
  }
}

// setup and loop
void setup(){
  Serial.begin(115200);
  delay(200);
  Serial.println("\nesp3 mic + therm");

  // seed prng using efuse mac (for jitter randomness)
  randomSeed((uint32_t)ESP.getEfuseMac());

  // connect wifi then try mdns to find hub ip
  connectWiFi();
  MDNS.end();          // ensure clean mdns state
  resolveHubMDNS();    // ok if it fails now, retry in loop

  // open udp port for data sends
  udpTx.begin(0);

  // adc setup for thermistor pin
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(THERM_ADC_PIN, ADC_11db);

  // start thermistor detector
  therm.begin(TP, THERM_ADC_PIN);

  // set initial mode and schedule first send
  applyMode(DEFAULT_MODE);
}

void loop(){
  // keep wifi up if it drops
  maybeReconnectWiFi();

  // if hub ip unknown, try mdns again every 3 s
  static uint32_t lastMDNS=0;
  if ((uint32_t)hubIP == 0 && now_ms() - lastMDNS > 3000){
    resolveHubMDNS();
    lastMDNS = now_ms();
  }


int sz = udpSync.parsePacket();
if (sz >= (int)sizeof(SyncPacket)){
  IPAddress from = udpSync.remoteIP();          
  SyncPacket sp; udpSync.read((uint8_t*)&sp, sizeof(sp));
  if (sp.targetDev==0 || sp.targetDev==DEV_ID){
    hubIP = from;                                
    if (sp.msgType==2) applyMode(sp.mode);       // handle mode
  }
}


  // run thermistor detector continuously
  therm.update();

  // time to send a packet and we know hub ip
  if ((int32_t)(now_ms() - nextSendDue) >= 0 && (uint32_t)hubIP != 0){
    MicTempPayload pl{}; // fill a fresh payload

    // collect mic features only if mode says mic on and driver enabled
    if (modeCfg.mic_on && micEnabled){
      computeMicFeatures(pl.avgAmplitude, pl.maxMagnitude, pl.dominantFreq, pl.snoring);
    } else {
      pl.avgAmplitude=0; pl.maxMagnitude=0; pl.dominantFreq=0; pl.snoring=0;
    }

    // choose absolute temp or ac temp per flag
#if REPORT_ABSOLUTE_TEMP
    pl.tempC = therm.lastTempAbs;
#else
    pl.tempC = therm.lastTemp;
#endif
    // copy detector stats
    pl.env   = therm.lastEnv;
    pl.ratio = therm.lastRatio;
    pl.rms   = therm.lastRMS;
    pl.zc    = (uint8_t)therm.lastZC;
    pl.effort= therm.lastEffort ? 1 : 0;
    pl.why   = ThermBreath::whyCode(therm.lastWhy);
    pl.conf  = therm.lastConf;
    pl.apnea = therm.apneaOut ? 1 : 0;

    // send and schedule next with a little jitter
    sendPacket(pl);
    nextSendDue = now_ms() + modeCfg.send_interval_ms
                + (SEND_JITTER_MS_MAX ? random(0, SEND_JITTER_MS_MAX) : 0);
  }

  // see if our last packet got acked,retry if not
  maybeResend();
}
