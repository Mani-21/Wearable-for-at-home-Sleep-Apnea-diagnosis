/*
  sleep apnea project
  esp1 imu node
  version 14 

  using arduino 1.8.19
  ref:
  https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
  https://lastminuteengineers.com/esp32-mdns-tutorial/
  https://www.aranacorp.com/en/communication-between-two-esp32s-via-udp/
  https://docs.arduino.cc/language-reference/en/functions/wifi/udp/
  https://docs.arduino.cc/language-reference/?_gl=1*19wvmhw*_up*MQ..*_ga*MTM4MTEzMDgwMy4xNzU2MzA2NDY5*_ga_NEXN8H46L5*czE3NTYzMDY0NjckbzEkZzEkdDE3NTYzMDY1MzAkajYwJGwwJGg2NjgyMzY2NTU.#functions
  
*/


// forward declaration to ignore compilation error
struct SensorPayload;

//all changeable parameters on top to try different settings and thresholds

// wifi
#define WIFI_SSID  "VM2C2F68"        //  wifi username  // "VM2C2F68"  // "Mani"
#define WIFI_PASS  "dtUttDw2tqnu"   //  wifi password  // "dtUttDw2tqnu" //"manikanta"
static const char* DEV_HOSTNAME = "esp1";      // this device mdns name

// identity and ports
static const uint8_t  DEV_ID         = 1;      // esp1 id
static const uint16_t PORT_TELEMETRY = 5005;   // esp1 -> hub
static const uint16_t PORT_ACK       = 5006;   // hub  -> esp1
static const uint16_t PORT_SYNC      = 5007;   // hub  -> all

// send timw and mode
//  send_interval_ms controls packet time.
// MODE1 = 500 ms -> 2 Hz; MODE2 = 1000 ms -> 1 Hz.
struct ModeConfig { uint8_t id; uint16_t send_interval_ms; };
static const ModeConfig MODE1 = { 1, 500  };   // 2 hz
static const ModeConfig MODE2 = { 2, 1000 };   // 1 hz
static const uint8_t DEFAULT_MODE = 1;         // start in mode 1

// ack and retry
// after sending, wait ACK_WAIT_MS for AckPacket with matching seq
// If not received, retry up to MAX_RESEND times
// Small SEND_JITTER_MS_MAX intentional random delay is added to de-phase multiple packets, so easy for esp2 to rx
static const uint32_t ACK_WAIT_MS        = 150;  // wait this long for ack
static const uint8_t  MAX_RESEND         = 2;    // retry this many times if no ack
static const uint16_t SEND_JITTER_MS_MAX = 40;   // random delay to reduce chance of collision with esp3

// i2c pins and imu ranges
static const int I2C_SDA_PIN = 21; // connect to imu sda
static const int I2C_SCL_PIN = 22; // connect to imu scl
//using smallest ranges
// Smallest ranges give best resolution for small breathing motions.
static const int IMU_ACC_G_RANGE    = 2;    // 2g range
static const int IMU_GYRO_DPS_RANGE = 250;  // 250 dps range
//5hz-> 300/min, enough for motion of interest
// Low pass to suppress high-frequency noise; approx. breathing is alpha0.1–1 hz (6 to 60 breaths/min).
static const int IMU_FILTER_HZ      = 5;    // lowest bandwidth for breathing




// libs
#include <WiFi.h>     //wifi
#include <WiFiUdp.h>  //ports 5005,6,7
#include <ESPmDNS.h>  //mdns
#include <Wire.h>     //i2c
#include <Adafruit_MPU6050.h> //imu
#include <Adafruit_Sensor.h>
#include <math.h>


//time in ms since boot
static inline uint32_t now_ms(){ return millis(); }

// imu effort parameters
// Summary of the processing (per posture-selected axis):
//refer section 2.4 for more details
//   raw -> HPF -> LPF(f_lp)  ==> AC
//   |AC| -> short/long envelopes (t_short, t_long) -> ratio = envShort/envLong
//   ac -> (t_dc) filtered -> signal centered at 0-> ZC + RMS -> oscillation count
//   quiet check using gyro RMS and accel deviation
//   state machine with ratio_on, ratio_off and confirmed over N seconds
struct EffortParams {
  uint16_t sample_hz      = 25;   // raw sample rate
  uint16_t analysis_hz    = 2;    // logging + detector update rate

  bool  use_hpf           = true; 
  float effort_lp_hz      = 1.2f; // low pass cutoff
  float effort_hp_hz      = 0.05f;// high pass cutoff
  // ac = [0.05, 1.2] hz comfortably covers breathing 0.1–1 hz

  float env_tau_short_s   = 0.6f; // fast envelope
  float env_tau_long_s    = 18.0f;// slow envelope
  // ratio envShort/envLong; breathing effort lift short > long

  float env_abs_floor     = 0.008f; // min envelope
  float ratio_on          = 1.35f;  // turn on threshold
  float ratio_off         = 1.12f;  // turn off threshold
  // to avoid flickering/ for stable effort signal, ratio >= 1.35 to turn on, drop below 1.12 to turn off.

  float dc_tau_s          = 0.8f; // detrend time constant to get a signal centered at 0
  int   zc_min            = 2;    // min zero crossings
  float zc_amp_floor      = 0.004f; // min amplitude for zc //0.01
  float rms_floor         = 0.006f; // min rms for motion   //0.01
 
  // if the motion is quiet,remove false effort on.
  float quiet_gyro_rms    = 0.05f; // quiet gyro  //0.1
  float quiet_acc_dev     = 0.20f; // quiet accel dev //0.1
  float quiet_env_mult    = 1.3f;  // quiet envelope factor //1.5
  

  float motion_gyro_rms_thr = 0.3f; // motion gyro thershold //0.2
  float motion_acc_dev_thr  = 1.20f;// motion accel dev thershold //1

  //observation: slightly high->effort always off, slightly low->always on
  //very sensitive to time

  int   eff_on_confirm_s  = 1; // on confirm time //0.5
  int   noeff_confirm_s   = 1; // off confirm time
 
};


// packet types 

//header structure
//header is common for all the data packets

struct PacketHeader {
  uint8_t  dev_id;       // who sent this, 1 or 3 ?
  uint8_t  mode;         // 1 or 2
  uint16_t payload_len;  // bytes after header
  uint32_t seq;          // sequence number
  uint32_t t_ms;         // local time in ms
};

//data packet structure
// packet holds both raw IMU samples and derived features.
//  6 floats raw (ax..gz)
//  3 floats features (env,ratio,rms) 
//  1 uint8_t zc 
//  4 uint8_t (posture,motion,effort,why) 
//  1 float conf 
struct SensorPayload {
  // raw imu
  float ax, ay, az;
  float gx, gy, gz;
  // effort features
  float env, ratio, rms;
  uint8_t zc;
  uint8_t posture;   // 1 prone, 2 right, 3 left, 4 supine, 5 stand, 0 unknown
  uint8_t motion;    // 0 or 1
  uint8_t effort;    // 0 or 1
  uint8_t why;       // 0 -, 1 a, 2 b, 3 a+b, 4 q
  float   conf;      // 0 to 1
};
//ack
struct AckPacket { uint8_t dev_id; uint32_t seqAck; };
//sync
struct SyncPacket { uint8_t msgType; uint8_t targetDev; uint32_t epoch_ms; uint8_t mode; }; // 1 sync, 2 mode set

// imu effort 
class IMUEffort {
public:
  enum Posture { P_UNKNOWN, P_SUPINE, P_PRONE, P_LEFT, P_RIGHT, P_STAND };

  //outputs updated at 2hz rate
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  float env=0, ratio=0, rms=0; int zc=0;
  bool  effort=false, motion=false;
  Posture stablePosture=P_UNKNOWN;
  const char* why="-"; float conf=0;

  bool begin(const EffortParams& p, int sda, int scl){
    P = p;
    Wire.begin(sda, scl);
    if (!mpu.begin()) return false;

    // all inbuilt sensor range options
    switch (IMU_ACC_G_RANGE){
      case 2:  mpu.setAccelerometerRange(MPU6050_RANGE_2_G); break;
      case 4:  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); break;
      case 8:  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); break;
      default: mpu.setAccelerometerRange(MPU6050_RANGE_16_G); break;
    }
    switch (IMU_GYRO_DPS_RANGE){
      case 250:  mpu.setGyroRange(MPU6050_RANGE_250_DEG);  break;
      case 500:  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  break;
      case 1000: mpu.setGyroRange(MPU6050_RANGE_1000_DEG); break;
      default:   mpu.setGyroRange(MPU6050_RANGE_2000_DEG); break;
    }
    // set filter bandwidth
    switch (IMU_FILTER_HZ){
      case 260: mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); break;
      case 184: mpu.setFilterBandwidth(MPU6050_BAND_184_HZ); break;
      case 94:  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);  break;
      case 44:  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);  break;
      case 21:  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  break;
      case 10:  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);  break;
      default:  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);   break;
    }

    // timing setup
    // sample_period_ms = round(1000 / sample_hz)
    // analysis_period_ms = round(1000 / analysis_hz)
    sample_period_ms   = (uint32_t)roundf(1000.0f / P.sample_hz);
    analysis_period_ms = (uint32_t)roundf(1000.0f / P.analysis_hz);

    // filter alphas
    //refer 2.4 section
    // 1st-order IIR : y[n] += alpha * (x[n] - y[n])
    // alpha from cutoff/time-constant at given sample rate:
    //   alpha = 1 - exp(-1 / (t * f_s))      
    //   or for cutoff f_c (Hz), t = 1/(2pi f_c) ⇒ alpha = 1 - exp(-2pi f_c / f_s)
    // alpha_from_tau -> alpha = 1 - exp(-1/(t * fs))
    lp_alpha        = alpha_from_tau(1.0f/(2.0f*PI_F*P.effort_lp_hz), P.sample_hz);
    if (P.use_hpf)  hp_alpha = alpha_from_tau(1.0f/(2.0f*PI_F*P.effort_hp_hz), P.sample_hz);
    env_alpha_short = alpha_from_tau(P.env_tau_short_s, P.analysis_hz);
    env_alpha_long  = alpha_from_tau(P.env_tau_long_s , P.analysis_hz);
    dc_alpha        = alpha_from_tau(P.dc_tau_s       , P.analysis_hz);

    // motion window length = 5s
    // motion_win = 5 * sample_hz (125 samples @25 Hz)
    motion_win = 5 * P.sample_hz; //
    if (motion_win > (int)MAX_MOTION_BUF) motion_win = MAX_MOTION_BUF;

    // start timers
    lastSample   = lastAnalysis = now_ms();
    return true;
  }

  void update(){
    uint32_t tnow = now_ms();

    // sample loop at sample rate
    // runs every sample_period_ms, takes latest IMU sample and updates
    // motion features + posture.
    while ((uint32_t)(tnow - lastSample) >= sample_period_ms){
      lastSample += sample_period_ms;

      sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);

      // copy raw values (units- m/s^2 for accel; rad/s for gyro 
      ax=a.acceleration.x; ay=a.acceleration.y; az=a.acceleration.z;
      gx=g.gyro.x;         gy=g.gyro.y;         gz=g.gyro.z;

      // compute motion 
      // gmag = sqrt(gx^2+gy^2+gz^2) 
      // gyroRMS over last motion_win samples
      // accDev is deviation from gravity (9.81 m/s^2)
      float gmag = sqrtf(gx*gx + gy*gy + gz*gz); //resultant g vector mag
      pushMotion(gmag, ax, ay, az);

      // posture 
      //posture doesn’t flicker when readings are near threshold, need the same posture for 0.5sec
      //25/2 = 12 samples = 480 ms
      // Posture decision rule
      //  if |axis| > 0.70 g and sign matches orientation
      //  consecutive samples (P.sample_hz/2) to confirm stablePosture
      Posture pcand = classifyPosture(ax, ay, az);
      if (pcand == candPosture) { if (postureSameCnt < 10000) postureSameCnt++; }
      else { candPosture = pcand; postureSameCnt=1; } //reset if posture change
      if (candPosture != P_UNKNOWN && postureSameCnt >= (P.sample_hz/2)) stablePosture = candPosture; //refuse unknown
    }

    // analysis loop at analysis rate
    // each analysis_period_ms (500 ms @ 2 hz) to update envelopes,oscillation features, and state machine.
    if ((uint32_t)(tnow - lastAnalysis) >= analysis_period_ms){
      lastAnalysis += analysis_period_ms;

      // choose axis based on posture
      //refer the fixed axis alignment in figure 33.
      // SUPINE/PRONE: chest motion stronger on X (ax)
      // LEFT/RIGHT : choose Z (az)
      // STAND/UNKNOWN: not required to compute
      float effSig = 0;
      if (stablePosture==P_SUPINE || stablePosture==P_PRONE) effSig = ax;
      else if (stablePosture==P_LEFT || stablePosture==P_RIGHT) effSig = az;

      // band pass
      // hpf + LPF to 1.2 Hz.
      // HPF: hp_lp_y += alpha_hp*(x - hp_lp_y); preLP = x - hp_lp_y.
      // LPF: lp_y += alpha_lp*(preLP - lp_y).
      float preLP = effSig;
      if (P.use_hpf){ hp_lp_y += hp_alpha * (effSig - hp_lp_y); preLP = effSig - hp_lp_y; }
      lp_y += lp_alpha * (preLP - lp_y);
      float band = lp_y;

      // envelope tracking
      // envShort = EMA(|ac|, t_short), envLong = EMA(|ac|, t_long)
      // ratio = envShort / envLong, if ratio rises above ratio_on and
      // absolute level passes atleast env_abs_floor,then good effort
      float aa = fabsf(band);
      if (seedEnv){ envShort = aa; envLong = aa; seedEnv=false; }
      envShort += env_alpha_short * (aa - envShort);
      envLong  += env_alpha_long  * (aa - envLong);
      ratio    = (envLong>1e-6f) ? (envShort/envLong) : 0.0f;
      bool envelopeOK = (envShort >= P.env_abs_floor) && (ratio >= P.ratio_on);

      // detrend then fill buffer
      // dcShort = EMA(ac, t_dc), d = ac - dcShort( 0centered)
      // OSC_BUF=16 for ZC and RMS.
      //band=ac
      dcShort += dc_alpha * (band - dcShort);
      float d  = band - dcShort;
      detBuf[detIdx] = d;
      detIdx = (detIdx + 1) % OSC_BUF;
      if (detCount < OSC_BUF) detCount++;

      // zc and rms
      // zero crossings counted with amplitude floor to ignore tiny noise:
      // increments when sign flips with |value| > zc_amp_floor.
      // RMS over same buffer, sqrt(mean(d^2)).
      zc  = countZC(detBuf, detCount, P.zc_amp_floor);
      rms = computeRMS(detBuf, detCount);
      bool oscillationOK = (zc >= P.zc_min) && (rms >= P.rms_floor);

      // quiet check
      // to suppress false positives when almost still
      //   gyroRMS < quiet_gyro_rms AND
      //   accDev  < quiet_acc_dev  AND
      //   envShort < quiet_env_mult * env_abs_floor
      bool quiet = (gyroRMS < P.quiet_gyro_rms) && (accDev < P.quiet_acc_dev) &&
                   (envShort < (P.quiet_env_mult * P.env_abs_floor));

      // state updates
      // on if either envelopeOK OR oscillationOK and not quiet,
      // confirmation windows (eff_on_confirm_s, noeff_confirm_s).
      if ((envelopeOK || oscillationOK) && !quiet) { effOnCnt++; noEffCnt=0; }
      else { noEffCnt++; effOnCnt=0; }

      if (!effort && effOnCnt >= P.eff_on_confirm_s * P.analysis_hz) { effort=true;  noEffCnt=0; }
      if ( effort && noEffCnt >= P.noeff_confirm_s  * P.analysis_hz) { effort=false; effOnCnt=0; }

      // effort reason 
      // why: "A"=envelope gate; "B"=oscillation gate; "A+B"=both; "Q"=quiet ; "-"=neither.
      if      (quiet)                       why="Q";
      else if (envelopeOK && oscillationOK) why="A+B";
      else if (envelopeOK)                  why="A";
      else if (oscillationOK)               why="B";
      else                                  why="-";

      // confidence to know how much effort is present and for easy post processing
      //   envMag = ( (envShort - floor) / (2*min thershold) )
      //   envRat = ( (ratio - ratio_off) / (ratio_on - ratio_off) )
      //   envConf = envMag * envRat
      //   oscMag  =( (rms - rms threshold) / (2*rms thershold) )
      //   oscConf = (zc>=zc_min ? oscMag : 0)
      //   conf = max(envConf, oscConf)
      //clamp all values to (0,1)
      float envMag  = clamp01f( (envShort - P.env_abs_floor) / (2.0f * P.env_abs_floor) );
      float envRat  = clamp01f( (ratio    - P.ratio_off     ) / (P.ratio_on - P.ratio_off) );
      float envConf = envMag * envRat;
      float oscMag  = clamp01f( (rms      - P.rms_floor     ) / (2.0f * P.rms_floor) );
      float oscConf = (zc >= P.zc_min ? oscMag : 0.0f);
      conf = fmaxf(envConf, oscConf);

      // motion flag 
      // motion indicates excessive movemen
      motion = (gyroRMS > P.motion_gyro_rms_thr) || (accDev > P.motion_acc_dev_thr);

      // share envelope value
      env = envShort;
    }
  }

  static uint8_t whyCode(const char* w){
    if (strcmp(w,"A")==0)   return 1;
    if (strcmp(w,"B")==0)   return 2;
    if (strcmp(w,"A+B")==0) return 3;
    if (strcmp(w,"Q")==0)   return 4;
    return 0;
  }

private:
  EffortParams P;
  Adafruit_MPU6050 mpu;
  //constants
  static constexpr float PI_F = 3.14159265358979323846f;
  static constexpr float G    = 9.81f;

  // timers
  uint32_t lastSample=0, lastAnalysis=0;
  uint32_t sample_period_ms=40, analysis_period_ms=500;

  // posture cond
  Posture candPosture=P_UNKNOWN; int postureSameCnt=0;
  float T_GRAV_HIGH = 0.70f * G; //atleast 70% of g in that direction required to confirm

  // motion buffers
  // window over last 125 samples:
  //   gSqBuf holds (gyro magnitude)^2, sumGSq keeps sum.
  //   gyroRMS = sqrt( sumGSq / N )
  static const uint16_t MAX_MOTION_BUF = 256;
  float gSqBuf[MAX_MOTION_BUF]; uint16_t gIdx=0, gCount=0; float sumGSq=0;
  float accMean = G; const float ACC_EMA_ALPHA = 0.01f;
  float gyroRMS=0, accDev=0;
  int   motion_win=125;

  // filters
  float lp_y=0, lp_alpha=0;
  float hp_lp_y=0, hp_alpha=0;

  // envelopes
  float envShort=0, envLong=0; float env_alpha_short=0, env_alpha_long=0; bool seedEnv=true;

  // oscillation buffer
  static const int   OSC_BUF=16;
  float detBuf[OSC_BUF]; int detIdx=0, detCount=0;
  float dcShort=0, dc_alpha=0;

  // counters
  int effOnCnt=0, noEffCnt=0;

  // alpha = 1 - exp(-1/(t * f_s))
  static inline float alpha_from_tau(float tau_s, float fs){ return 1.0f - expf(-1.0f/(tau_s*fs)); }
  static inline float clamp01f(float x){ return (x<0)?0: (x>1)?1:x; }

  void pushMotion(float gmag, float ax, float ay, float az){
    float g2 = gmag*gmag;
    if (gCount < motion_win){
      gSqBuf[gIdx] = g2; sumGSq += g2; gIdx=(gIdx+1)%motion_win; gCount++;
    } else {
      sumGSq += g2 - gSqBuf[gIdx];
      gSqBuf[gIdx] = g2; gIdx=(gIdx+1)%motion_win;
    }
    gyroRMS = sqrtf(sumGSq / (float)(gCount?gCount:1));
    float anorm = sqrtf(ax*ax + ay*ay + az*az);
    accMean += ACC_EMA_ALPHA * (anorm - accMean);
    accDev = fabsf(anorm - accMean);
  }

  static int countZC(const float* buf, int n, float floor){
    int z=0; for (int i=1;i<n;i++){
      float a=buf[i], b=buf[i-1];
      if ((a>floor && b<-floor) || (a<-floor && b>floor)) z++;
    } return z;
  }
  static float computeRMS(const float* buf, int n){
    if (n<=0) return 0;
    float s=0; for(int i=0;i<n;i++){ s+=buf[i]*buf[i]; }
    return sqrtf(s/n);
  }

  // Posture classification thresholds using gravity alignment:
  //   SUPINE :  az = +g, |az| > 0.70 g
  //   PRONE  :  az = -g, |az| > 0.70 g
  //   RIGHT  :  ax = +g, |ax| > 0.70 g
  //   LEFT   :  ax = -g, |ax| > 0.70 g
  //   STAND  :  |ay| > 0.70 g
  Posture classifyPosture(float ax, float ay, float az){
    if (fabsf(az)>T_GRAV_HIGH && az>0) return P_SUPINE;
    if (fabsf(az)>T_GRAV_HIGH && az<0) return P_PRONE;
    if (fabsf(ax)>T_GRAV_HIGH && ax>0) return P_RIGHT;
    if (fabsf(ax)>T_GRAV_HIGH && ax<0) return P_LEFT;
    if (fabsf(ay)>T_GRAV_HIGH)         return P_STAND;
    return P_UNKNOWN;
  }
};


// communication
WiFiUDP udpTx, udpAck, udpSync;
IPAddress hubIP;                          // from sync / ack
ModeConfig modeCfg = (DEFAULT_MODE==1)?MODE1:MODE2;
uint8_t currentMode = DEFAULT_MODE;

uint32_t seq=0, nextSendDue=0;

uint8_t lastBuf[sizeof(PacketHeader)+sizeof(SensorPayload)];
size_t  lastLen=0;
uint32_t lastTxSeq=0, lastTxTime=0;

IMUEffort imu;
EffortParams P;

// send one packet
// build header then payload into buffer and send via UDP.
// keep a copy in lastBuf for resends.
void sendPacket(const struct SensorPayload& pl){
  PacketHeader hdr{ DEV_ID, currentMode, (uint16_t)sizeof(pl), seq++, now_ms() };

  memcpy(lastBuf, &hdr, sizeof(hdr));
  memcpy(lastBuf+sizeof(hdr), &pl, sizeof(pl));

  udpTx.beginPacket(hubIP, PORT_TELEMETRY);
  udpTx.write(lastBuf, sizeof(hdr)+sizeof(pl));
  udpTx.endPacket();

  lastLen = sizeof(hdr)+sizeof(pl);
  lastTxSeq = hdr.seq;
  lastTxTime = now_ms();
}

// receive ack and resend if needed
// Algorithm:
//   If seqAck == lastTxSeq, clear
//   Else, after ACK_WAIT_MS, resend up to MAX_RESEND times.
//   If still no ack, drop the packet and move on.
void maybeResend(){
  if (!lastLen) return;

  static uint8_t retries = 0;

  int sz = udpAck.parsePacket();
  while (sz >= (int)sizeof(AckPacket)){
    IPAddress from = udpAck.remoteIP();
    AckPacket ack; udpAck.read((uint8_t*)&ack, sizeof(ack));
    hubIP = from; 
    if (ack.seqAck == lastTxSeq){ lastLen=0; retries=0; return; }
    sz = udpAck.parsePacket();
  }

  if (now_ms() - lastTxTime < ACK_WAIT_MS) return;

  if (retries < MAX_RESEND){
    udpTx.beginPacket(hubIP, PORT_TELEMETRY);
    udpTx.write(lastBuf, lastLen);
    udpTx.endPacket();
    retries++;
    lastTxTime = now_ms();
  } else {
    retries=0; lastLen=0; // give up on this packet
  }
}


// one time setup
void setup(){
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEV_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status()!=WL_CONNECTED){ delay(200); Serial.print("."); }
  //Serial.print("\nip: "); Serial.println(WiFi.localIP());

  if (!MDNS.begin(DEV_HOSTNAME)) //Serial.println("mdns failed");

  udpTx.begin(0);
  udpAck.begin(PORT_ACK);
  udpSync.begin(PORT_SYNC);

  if (!imu.begin(P, I2C_SDA_PIN, I2C_SCL_PIN)){
    Serial.println("mpu failed"); while(1) delay(1000);
  }

  nextSendDue = now_ms() + modeCfg.send_interval_ms;
}


// main loop
void loop(){
  // process SYNC from hub:
  //   msgType==1 time sync
  //   msgType==2 mode sync
  int sz = udpSync.parsePacket();
  if (sz >= (int)sizeof(SyncPacket)){
    IPAddress from = udpSync.remoteIP();
    SyncPacket sp; udpSync.read((uint8_t*)&sp, sizeof(sp));
    if (sp.targetDev==0 || sp.targetDev==DEV_ID){
      hubIP = from;
      if (sp.msgType==1 || sp.msgType==2){
        currentMode = sp.mode;
        modeCfg = (currentMode==1)?MODE1:MODE2;
      }
    }
  }

  // run imu processing
  imu.update();

  // send 
  if ((int32_t)(now_ms() - nextSendDue) >= 0 && (uint32_t)hubIP != 0){
    SensorPayload pl{};
    // raw copies for logging/plotting:
    pl.ax=imu.ax; pl.ay=imu.ay; pl.az=imu.az;
    pl.gx=imu.gx; pl.gy=imu.gy; pl.gz=imu.gz;
    // features:
    pl.env=imu.env; pl.ratio=imu.ratio; pl.rms=imu.rms;
    pl.zc=(uint8_t)imu.zc;

    // posture encoding to code used by csv
    uint8_t posture=0;
    if (imu.stablePosture==IMUEffort::P_PRONE)  posture=1;
    else if (imu.stablePosture==IMUEffort::P_RIGHT) posture=2;
    else if (imu.stablePosture==IMUEffort::P_LEFT)  posture=3;
    else if (imu.stablePosture==IMUEffort::P_SUPINE)posture=4;
    else if (imu.stablePosture==IMUEffort::P_STAND) posture=5;
    pl.posture = posture;

    pl.motion = imu.motion ? 1 : 0;
    pl.effort = imu.effort ? 1 : 0;
    pl.why    = IMUEffort::whyCode(imu.why);
    pl.conf   = imu.conf;

    sendPacket(pl);
    // next send time = now + interval +  random jitter (0, SEND_JITTER_MS_MAX)
    nextSendDue = now_ms() + modeCfg.send_interval_ms + (SEND_JITTER_MS_MAX ? random(0, SEND_JITTER_MS_MAX) : 0);
  }

  // check acks and resends
  maybeResend();
}
