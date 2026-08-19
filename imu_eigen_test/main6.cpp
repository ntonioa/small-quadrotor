#include <Arduino.h>
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif

#include <WiFi.h>
#include <WiFiUdp.h>
#include "common/mavlink.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <SPI.h>
#include "ICM_20948.h"
#include <math.h>
#include <cstring>

#include "predict.h"
#include "update.h"
using namespace sym; // per le funzioni EKF generate (Predict/Update)

// ====== PIN SPI ICM-20948 ======
#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

ICM_20948_SPI imu;

// ====== WiFi AP / UDP / MAVLink ======
static const char *WIFI_SSID = "DRONE_AP";
static const char *WIFI_PASS = "12345678";
static const uint16_t QGC_PORT = 14550;
WiFiUDP udp;
IPAddress qgc_bcast(192, 168, 4, 255);
IPAddress qgc_ip = qgc_bcast;

uint8_t sysid = 1;
uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

uint32_t last_hb_ms = 0;
uint32_t last_att_ms = 0;
uint32_t last_raw_rpy_ms = 0; // rate limiter per RPY non filtrati

// Integratore yaw "grezzo" (solo gyro Z, quindi driftante)
static float g_yaw_raw = 0.0f;

// ====== Tipi Eigen ======
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec3 = Eigen::Vector3f;
using Vec4 = Eigen::Vector4f;
using Mat77 = Eigen::Matrix<float, 7, 7>;
using Mat33 = Eigen::Matrix3f;

// ====== EKF ======
Vec7 x;   // [qw,qx,qy,qz, bx,by,bz] (bias gyro)
Mat77 P;  // covarianza
Mat77 Qm; // rumore processo
Mat33 Rm; // rumore misura (acc)

unsigned long last_time_us;  // micros per precisione
static constexpr uint32_t LOOP_PERIOD_US = 4000; // 250Hz = 4000us

// ====== Calibrazione grezza ======
struct
{
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
} calibration;

// ======================= MOTORI: DUTY @ 78 kHz  =======================
#include <algorithm>

#define PWM_FREQUENCY 30000// Hz 78000
#define PWM_RESOLUTION 10   // bit


static const int MOTOR_PINS[4] = {12, 13, 14, 15};
static const int MOTOR_CHANNELS[4] = {0, 1, 2, 3};

// clamp compatibile anche con toolchain vecchi
template <typename T>
static inline T my_clamp(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi
                                                                            : v; }

static inline void motors_init()
{
  for (int i = 0; i < 4; ++i)
  {
    ledcSetup(MOTOR_CHANNELS[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
    ledcWrite(MOTOR_CHANNELS[i], 0);
  }
}

// u01 ∈ [0,1] → ticks [0 .. 2^RES-1]
static inline int duty_from_unit(float u01)
{
  u01 = my_clamp(u01, 0.0f, 1.0f);
  const int ticks_max = (1 << PWM_RESOLUTION) - 1;
  int ticks = int(lroundf(u01 * ticks_max));
  if (ticks < 0)
    return 0;
  if (ticks > ticks_max)
    return ticks_max;
  return ticks;
}

static inline void write_motor_duty01(int motorIndex, float u01)
{
  if (motorIndex < 0 || motorIndex > 3)
    return;
  ledcWrite(MOTOR_CHANNELS[motorIndex], duty_from_unit(u01));
}

// Usa questa con un vettore di comandi 0..1 già normalizzati
template <typename Vec4Like>
static inline void motors_write_from_forces(const Vec4Like &u)
{
  for (int i = 0; i < 4; ++i)
  {
    float v = float(u(i));
    v = my_clamp(v, 0.0f, 1.0f);
    write_motor_duty01(i, v);
  }
}

static inline void motors_stop_all()
{
  for (int i = 0; i < 4; ++i)
    ledcWrite(MOTOR_CHANNELS[i], 0);
}
// ==============================================================================

#if __cplusplus < 201703L
namespace std
{
  template <class T>
  inline T clamp(const T &v, const T &lo, const T &hi) { return v < lo ? lo : (v > hi ? hi : v); }
}
#endif

// ====== Shim Quaternion compatibile coi generated ======
namespace sym
{
  template <typename Scalar>
  struct Quaternion
  {
    Eigen::Matrix<Scalar, 4, 1> storage;
    Quaternion() : storage(Eigen::Matrix<Scalar, 4, 1>::Zero()) {}
    Quaternion(Scalar w, Scalar x, Scalar y, Scalar z) { storage << w, x, y, z; }
    static Quaternion FromStorage(const Eigen::Matrix<Scalar, 4, 1> &v)
    {
      Quaternion q;
      q.storage = v;
      return q;
    }
    const Eigen::Matrix<Scalar, 4, 1> &Data() const { return storage; }
  };
}

// ====== Controller generated ======
#include "qd_from_rpy.h"
#include "geo_attitude_only.h"
#include "thrust_from_throttle.h"
#include "make_wrench.h"
#include "attitude_controller_step.h"

// ====== Parametri fisici / guadagni ======
static constexpr float MASS = 0.073f; // 73 g reali
static constexpr float GRAV = 9.81f;

// Parametri regolabili a runtime (possono essere aggiornati via MAVLink)
static float g_hover_thr = 0.50f;        // stick di hover ~50%
static float g_thr_max_factor = 1.2f;    // full stick = 1.2×(m*g) totale (headroom sopra hover)

static Eigen::Matrix3f J_body()
{
  Eigen::Matrix3f J;
  J << 7.1031997e-5, 1.32488e-7, -1.82229e-7,
      1.32488e-7, 6.8947577e-5, 6.66764e-7,
      -1.82229e-7, 6.66764e-7, 1.323e-4;
  return J;
}

// Guadagni regolabili a runtime (roll, pitch, yaw)
// Riduci drasticamente i guadagni per il test
// Guadagni di volo standard
static float g_kR_roll = 8.0f,  g_kR_pitch = 8.0f,  g_kR_yaw = 0.00f;
static float g_kW_roll = 1.5f,  g_kW_pitch = 1.5f,  g_kW_yaw = 0.00f;

// ====== Comandi da QGC ======
struct PilotCmd
{
  float phi = 0, theta = 0, psi = 0, thr = 0;
} g_cmd; // roll, pitch, yaw_ref, throttle(0..1)
struct StickScale
{
  float max_roll = 20.0f * M_PI / 180.0f, max_pitch = 25.0f * M_PI / 180.0f, max_yaw_rate = 10.0f * M_PI / 180.0f;
} g_scale;

// ====== Mixer X (y=[tau_x,tau_y,tau_z,Fz] → f=[f1..f4]) ======
using Mat44 = Eigen::Matrix<float, 4, 4>;
struct Mixer
{
  float l = 0.062f;
  float km = 0.0041f; // prima 0.016f
  Mat44 A; // Matrice di allocazione (Inversa): y -> f

  Mixer()
  {
    // Frame IMU (corpo): +X (Muso), +Y (Sinistra)
    // Layout Motori:
    // m0 (Pin 12): Anteriore-Sinistro (FL) [CCW]
    // m1 (Pin 13): Posteriore-Sinistro (RL) [CW]
    // m2 (Pin 14): Posteriore-Destro (RR)  [CCW]
    // m3 (Pin 15): Anteriore-Destro (FR)  [CW]
    
    // Calcoliamo i coefficienti di scala
    const float L = l / sqrtf(2.0f);
    const float c_l = 1.0f / (4.0f * L);  // Coeff. per Roll/Pitch
    const float c_k = 1.0f / (4.0f * km); // Coeff. per Yaw
    const float c_f = 0.25f;              // Coeff. per Spinta (Fz)

    // QUESTA È LA MATRICE INVERSA CORRETTA
    // Derivata dalla fisica (f = A_inv * y)
    // Frame X: pitch+ = nose up (+ davanti, - dietro)
    //          roll+ = ala dx giù (+ sinistra, - destra)
    A <<
        //  τx(Roll), τy(Pitch), τz(Yaw), Fz
        +c_l,     -c_l,      +c_k,     c_f,  // f(0) Pin12 (FL) [CCW]
        +c_l,     +c_l,      -c_k,     c_f,  // f(1) Pin13 (RL) [CW]
        -c_l,     +c_l,      +c_k,     c_f,  // f(2) Pin14 (RR) [CCW]
        -c_l,     -c_l,      -c_k,     c_f;  // f(3) Pin15 (FR) [CW]
  }

  Vec4 mix(const Vec4 &y) const
  {
    Vec4 f = A * y;
    return f;
  }
} g_mixer;

// Mapping N → duty 0..1 (coerente col clamp per-motore)
static inline void motors_from_forces_and_write(const Vec4 &f)
{
  const float F_tot_max = g_thr_max_factor * MASS * GRAV;
  const float f_full = F_tot_max / 4.0f; // ✅ per motore
  Vec4 u;
  for (int i = 0; i < 4; ++i)
    u(i) = std::clamp(f(i) / f_full, 0.0f, 1.0f);
  motors_write_from_forces(u);
}

// ====== MAVLink helpers ======
static void mav_send(const mavlink_message_t &msg)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket(qgc_ip, QGC_PORT);
  udp.write(buf, len);
  udp.endPacket();
}

static void send_heartbeat()
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
      sysid, compid, &msg,
      MAV_TYPE_QUADROTOR,
      MAV_AUTOPILOT_GENERIC,
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
      0, MAV_STATE_ACTIVE);
  mav_send(msg);
}

static void send_attitude(float roll, float pitch, float yaw, float p, float q, float r)
{
  mavlink_message_t msg;
  mavlink_msg_attitude_pack(sysid, compid, &msg, millis(), roll, pitch, yaw, p, q, r);
  mav_send(msg);
}

static void send_attitude_quat(Eigen::Quaternionf quat, float p, float q, float r)
{
  mavlink_message_t msg;
  float cov[9];
  for (int i = 0; i < 9; i++)
    cov[i] = NAN;
  mavlink_msg_attitude_quaternion_pack(sysid, compid, &msg, millis(), quat.w(), quat.x(), quat.y(), quat.z(), p, q, r, cov);
  mav_send(msg);
}

// Invia RPY "non filtrati" (roll/pitch da accelerometro, yaw da integrazione gyro Z)
static void send_raw_rpy(float roll_raw, float pitch_raw, float yaw_raw)
{
  mavlink_message_t msg;
  // Usa MOUNT_ORIENTATION (comune) per inviare roll/pitch/yaw "grezzi"
  mavlink_msg_mount_orientation_pack(
      sysid, compid, &msg,
      millis(),
    roll_raw, pitch_raw, yaw_raw, NAN);
  mav_send(msg);
}

// ====== MAVLink Parameters (runtime tuning) ======
struct ParamDef { const char *name; float *val; };

static ParamDef *get_param_table(size_t &count)
{
  static ParamDef params[] = {
      {"K_R_ROLL", &g_kR_roll},
      {"K_R_PITCH", &g_kR_pitch},
      {"K_R_YAW", &g_kR_yaw},
      {"K_W_ROLL", &g_kW_roll},
      {"K_W_PITCH", &g_kW_pitch},
      {"K_W_YAW", &g_kW_yaw},
      {"HOVER_THR", &g_hover_thr},
      {"THR_MAX", &g_thr_max_factor},
      {"MIX_L", &g_mixer.l},
      {"MIX_KM", &g_mixer.km},
  };
  count = sizeof(params) / sizeof(params[0]);
  return params;
}

static int param_find_index(const char *id)
{
  size_t n; auto tbl = get_param_table(n);
  for (size_t i = 0; i < n; ++i)
  {
    if (strncmp(id, tbl[i].name, MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN) == 0)
      return (int)i;
  }
  return -1;
}

static void send_param_value_index(int idx)
{
  size_t n; auto tbl = get_param_table(n);
  if (idx < 0 || (size_t)idx >= n) return;
  mavlink_message_t msg;
  char idbuf[16] = {0};
  strncpy(idbuf, tbl[idx].name, sizeof(idbuf) - 1);
  mavlink_msg_param_value_pack(sysid, compid, &msg,
                               idbuf, *tbl[idx].val,
                               MAV_PARAM_TYPE_REAL32,
                               (uint16_t)n, (uint16_t)idx);
  mav_send(msg);
}

// ====== Sicurezza ======
bool g_armed = false;
uint32_t last_cmd_ms = 0;

static void disarm_all()
{
  g_armed = false;
  motors_stop_all(); // duty=0
  Serial.println("[SAFE] DISARM");
}
static void arm_if_safe()
{
  if (!g_armed)
  {
    g_armed = true;
    motors_stop_all(); // per i brushed niente idle: restiamo a 0
    Serial.println("[SAFE] ARM");
  }
}

static mavlink_status_t rx_status;
static void qgc_parse_and_update_cmd()
{
  // Leggi eventuale pacchetto UDP
  int pkt = udp.parsePacket();
  if (!pkt)
    return;

  static uint8_t rxbuf[512];
  int n = udp.read(rxbuf, sizeof(rxbuf));

  mavlink_message_t msg;

  for (int i = 0; i < n; ++i)
  {
    if (!mavlink_parse_char(MAVLINK_COMM_0, rxbuf[i], &msg, &rx_status))
    {
      continue;
    }

    // Handle MAVLink messages
    if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
      mavlink_manual_control_t m{};
      mavlink_msg_manual_control_decode(&msg, &m);

    // comando valido → aggiorna timestamp
    last_cmd_ms = millis();

    // normalizzazione stick
    const float roll_norm = std::clamp((float)m.y / 1000.0f, -1.0f, 1.0f);
    const float pitch_norm = std::clamp((float)m.x / 1000.0f, -1.0f, 1.0f);
    const float thr_norm = std::clamp((float)m.z / 1000.0f, 0.0f, 1.0f);
    const float yaw_norm = std::clamp((float)m.r / 1000.0f, -1.0f, 1.0f);

    // mappa sui comandi pilota
    g_cmd.phi = roll_norm * g_scale.max_roll;
    g_cmd.theta = -pitch_norm * g_scale.max_pitch; // avanti → nose down
    g_cmd.thr = thr_norm;
    // g_cmd.psi viene gestito come hold semplice nel loop

      // —— Arming/Disarming con yaw tenuto agli estremi per 2s e throttle basso ——
    static uint32_t yaw_edge_ms = 0;
    static int state = 0; // 1=arm in corso, -1=disarm in corso, 0=neutro

    if (yaw_norm > 0.9f && thr_norm < 0.05f)
    {
      if (state != 1)
      {
        state = 1;
        yaw_edge_ms = millis();
      }
      if (millis() - yaw_edge_ms > 2000)
        arm_if_safe();
    }
    else if (yaw_norm < -0.9f && thr_norm < 0.05f)
    {
      if (state != -1)
      {
        state = -1;
        yaw_edge_ms = millis();
      }
      if (millis() - yaw_edge_ms > 2000)
        disarm_all();
    }
    }
    else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST)
    {
      // QGC chiede la lista dei parametri
      size_t n; get_param_table(n);
      for (size_t i = 0; i < n; ++i)
        send_param_value_index((int)i);
    }
    else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ)
    {
      mavlink_param_request_read_t prr{};
      mavlink_msg_param_request_read_decode(&msg, &prr);
      if (prr.param_index >= 0)
      {
        send_param_value_index(prr.param_index);
      }
      else
      {
        char id[17] = {0};
        memcpy(id, prr.param_id, 16);
        int idx = param_find_index(id);
        if (idx >= 0) send_param_value_index(idx);
      }
    }
    else if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET)
    {
      mavlink_param_set_t ps{};
      mavlink_msg_param_set_decode(&msg, &ps);
      char id[17] = {0};
      memcpy(id, ps.param_id, 16);
      int idx = param_find_index(id);
      if (idx >= 0)
      {
        size_t n; auto tbl = get_param_table(n);
        if (ps.param_type == MAV_PARAM_TYPE_REAL32 || ps.param_type == 0)
        {
          *(tbl[idx].val) = ps.param_value;
          // Clamp alcuni parametri a range sensato
          if (strcmp(tbl[idx].name, "HOVER_THR") == 0)
            *(tbl[idx].val) = std::clamp(*(tbl[idx].val), 0.05f, 0.95f);
          if (strcmp(tbl[idx].name, "THR_MAX") == 0)
            *(tbl[idx].val) = std::max(1.0f, *(tbl[idx].val));
          send_param_value_index(idx);
        }
      }
    }
    // other messages ignored
  }

  // (se vuoi: gestisci anche COMMAND_LONG per ARM/DISARM)
}

// ====== Calibrazione semplice ======
void calibrateAccelGyro(int samples = 500)
{
  float a[3] = {0}, g[3] = {0};
  Serial.println("\n*** Calibrazione IMU (NON MUOVERE) ***");
  for (int i = 0; i < samples; i++)
  {
    if (imu.dataReady())
    {
      imu.getAGMT();
      a[0] += imu.accX();
      a[1] += imu.accY();
      a[2] += imu.accZ() - 1000.0f; // 1g
      g[0] += imu.gyrX();
      g[1] += imu.gyrY();
      g[2] += imu.gyrZ();
      delay(5);
    }
  }
  for (int i = 0; i < 3; i++)
  {
    calibration.accelOffset[i] = a[i] / samples;
    calibration.gyroOffset[i] = g[i] / samples;
  }
  Serial.printf("Accel Off: %.2f %.2f %.2f\n", calibration.accelOffset[0], calibration.accelOffset[1], calibration.accelOffset[2]);
  Serial.printf("Gyro  Off: %.3f %.3f %.3f\n", calibration.gyroOffset[0], calibration.gyroOffset[1], calibration.gyroOffset[2]);
}

// ====== SETUP ======
void setup()
{
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(200);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  udp.begin(QGC_PORT);

  motors_init(); // PWM 78kHz duty-based
  disarm_all();
  Serial.println("Arma: yaw destra 2s con throttle a zero. Disarma: yaw sinistra 2s.");

  // IMU
  while (true)
  {
    imu.begin(CS_PIN, SPI);
    if (imu.status == ICM_20948_Stat_Ok)
    {
      Serial.println("IMU ok");
      break;
    }
    Serial.println("IMU fail, retry...");
    delay(500);
  }
  imu.swReset();
  delay(10);
  imu.sleep(false);
  imu.lowPower(false);
// === BLOCCO CORRETTO PER IL FILTRO DLPF (Versione 5) ===
  // Basato sugli esempi ufficiali della libreria SparkFun.

  // 1. Crea la "struttura" di configurazione del filtro.
  ICM_20948_dlpcfg_t mio_filtro;
  
  // 2. Imposta i membri interni della struttura.
  //    Scegliamo filtri attorno ai 50Hz, un ottimo inizio.
  mio_filtro.a = acc_d50bw4_n68bw8; // Filtro Accelerometro (~50.4 Hz)
  mio_filtro.g = gyr_d51bw2_n73bw3; // Filtro Giroscopio (~51.2 Hz)

  // 3. Crea il "bitmap" dei sensori con i nomi corretti.
  uint8_t sensor_bitmap = (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr);

  // 4. Chiama la funzione con la firma corretta:
  //    setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg)
  imu.setDLPFcfg(sensor_bitmap, mio_filtro);
  // ========================================================
  imu.startupMagnetometer();
  delay(100);

  calibrateAccelGyro();

  // EKF init
  x << 1, 0, 0, 0, 0, 0, 0;

  // --- P0: incertezza iniziale ---
  P.setZero();
  {
    const float DEG = M_PI / 180.0f;

    // orientamento: incognita ≈ 10°
    const float var_qvec = powf(0.5f * 10.0f * DEG, 2); // ≈ (0.087)^2 ≈ 7.6e-3
    P(0, 0) = 1e-6f;                                    // qw (teniamolo stretto: si rinormalizza)
    P(1, 1) = var_qvec;                                 // qx
    P(2, 2) = var_qvec;                                 // qy
    P(3, 3) = var_qvec;                                 // qz

    // bias gyro: incognita ≈ 1°/s
    const float var_bias = powf(1.0f * DEG, 2); // ≈ 3.0e-4
    P(4, 4) = var_bias;                         // bx
    P(5, 5) = var_bias;                         // by
    P(6, 6) = var_bias;                         // bz
  }

  // --- R: rumore misura accelerometro (unit vector) ---
  Rm.setIdentity();
  Rm *= powf(0.6f, 2); // sigma ≈ 0.10 -> meno fiducia nell'acc (test)

// --- Q: rumore di processo ---
  Qm.setZero();
  // "rumore orientamento" (effetto rumore gyro sulle eq. di stato)
  Qm(0, 0) = 1e-5f;  // Aumentato da 1e-6
  Qm(1, 1) = 1e-5f;  // Aumentato da 1e-6
  Qm(2, 2) = 1e-5f;  // Aumentato da 1e-6
  Qm(3, 3) = 1e-5f;  // Aumentato da 1e-6
  // random-walk dei bias (lenti ma non rigidi)
  Qm(4, 4) = powf(0.001f, 2); // ~0.001 rad/s /√s
  Qm(5, 5) = powf(0.001f, 2);
  Qm(6, 6) = powf(0.001f, 2);

  last_time_us = micros();
  Serial.println("Setup completo. Controller pronto (MANUAL_CONTROL via QGC).");
}

// ====== LOOP @ 250Hz ======
void loop()
{
  // Gestisci MAVLink senza bloccare (non-blocking)
  qgc_parse_and_update_cmd();

  // Rate limiter: esegui EKF + controller solo ogni 4ms (250Hz)
  unsigned long now_us = micros();
  if (now_us - last_time_us < LOOP_PERIOD_US) {
    // Troppo presto, aspetta prossimo ciclo
    delayMicroseconds(100); // Sleep breve per non saturare CPU
    return;
  }

  // Check se IMU ha dati pronti (non bloccare se non pronti)
  if (!imu.dataReady()) {
    return;
  }

  // Calcola dt preciso (in secondi)
  float dt = (now_us - last_time_us) * 1e-6f;  // micros → secondi
  last_time_us = now_us;  // Aggiorna timestamp per prossimo ciclo

  // IMU raw
  imu.getAGMT();
  float accX = imu.accX() - calibration.accelOffset[0];
  float accY = imu.accY() - calibration.accelOffset[1];
  float accZ = imu.accZ() - calibration.accelOffset[2];
  float gyrX = imu.gyrX() - calibration.gyroOffset[0];
  float gyrY = imu.gyrY() - calibration.gyroOffset[1];
  float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];

  // Copie per RPY "non filtrati" (prima di normalizzazione/EKF)
  float rawAccX = accX, rawAccY = accY, rawAccZ = accZ;   // in mg (unità SparkFun)
  float rawGyrZ = gyrZ;                                   // in deg/s (unità SparkFun)
  // CALCOLA ROLL/PITCH GREZZI DALL'ACCELEROMETRO
  // (Usa i valori in 'mg' prima della conversione in m/s^2)
  float roll_raw  = atan2f(rawAccY, sqrtf(rawAccX * rawAccX + rawAccZ * rawAccZ));
  float pitch_raw = atan2f(-rawAccX, rawAccZ);
  // Integra lo yaw grezzo (driftante)
  g_yaw_raw += (rawGyrZ * (M_PI / 180.0f)) * dt;

  
  // Sostituiscilo con la conversione da [mg] a [m/s^2]
  const float mg_to_ms2 = 0.001f * 9.81f;
  accX *= mg_to_ms2;
  accY *= mg_to_ms2;
  accZ *= mg_to_ms2;

  // EKF predict/update
  Vec3 z;
  z << -accX, -accY, -accZ; // misura verso gravità

  const float d2r = M_PI / 180.0f;
  Vec3 omega_measured_rads;
  omega_measured_rads << gyrX * d2r, gyrY * d2r, gyrZ * d2r;

  Vec7 x_pred;
  Mat77 P_pred;
  Predict(x, omega_measured_rads, dt, P, Qm, &x_pred, &P_pred);
  Vec7 x_upd;
  Mat77 P_upd;
  Update(x_pred, P_pred, z, Rm, &x_upd, &P_upd);
  x_upd.head<4>().normalize();
  x = x_upd;
  P = P_upd;

  // Estrai quat e bias
  float qw = x(0), qx = x(1), qy = x(2), qz = x(3);
  float bx = x(4), by = x(5), bz = x(6);

  // RPY (solo debug)
  float roll = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
  float pitch = asinf(2 * (qw * qy - qz * qx));
  float yaw = atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  // Simple heading hold: aggiorna sempre
  g_cmd.psi = yaw;

  // Gyro rad/s (SparkFun in deg/s)
  float p_rate = (gyrX - bx) * d2r;
  float q_rate = (gyrY - by) * d2r;
  float r_rate = (gyrZ - bz) * d2r;

  // MAVLink periodic
  uint32_t now_ms = millis();
  if (now_ms - last_hb_ms >= 1000)
  {
    last_hb_ms = now_ms;
    send_heartbeat();
  }
  if (now_ms - last_att_ms >= 20)
  {
    last_att_ms = now_ms;
    //send_attitude_quat({qw, qx, qy, qz}, p_rate, q_rate, r_rate);
    send_attitude(roll, pitch, yaw, p_rate, q_rate, r_rate);

    // AGGIUNGI QUESTE DUE RIGHE
    if (now_ms - last_raw_rpy_ms >= 50) { // Invia raw a 20Hz
         last_raw_rpy_ms = now_ms;
         send_raw_rpy(roll_raw, pitch_raw, g_yaw_raw);
    }
  }
  
  // ===== Leggi comandi QGC =====
  qgc_parse_and_update_cmd();

  /// ===== Controller =====
  Eigen::Matrix<float, 4, 1> q_storage;
  q_storage << qw, qx, qy, qz;
  sym::Quaternion<float> q_sym = sym::Quaternion<float>::FromStorage(q_storage);
  Eigen::Vector3f omega_body(p_rate, q_rate, r_rate);

  // 0..1 stick mappato con hover a 1.0: lineare sotto hover, headroom sopra
  float thr_eff;
  if (g_cmd.thr <= g_hover_thr)
  {
    // Da 0 a hover: scala lineare fino a 1.0
    thr_eff = (g_hover_thr > 1e-3f) ? (g_cmd.thr / g_hover_thr) : (g_cmd.thr * 2.0f);
  }
  else
  {
    // Da hover a full stick: sale linearmente fino a THRUST_MAX_FACTOR
    float denom = std::max(1e-3f, 1.0f - g_hover_thr);
    float slope = (g_thr_max_factor - 1.0f) / denom;
    thr_eff = 1.0f + (g_cmd.thr - g_hover_thr) * slope;
  }
  thr_eff = std::clamp(thr_eff, 0.0f, g_thr_max_factor);


  /// ===== Controller PD DISACCOPPIATO (sostituisce il geometric) =====
  // Errori diretti in angoli di Eulero (nessun coupling!)
  float e_roll = g_cmd.phi - roll;
  float e_pitch = g_cmd.theta - pitch;
  float e_yaw = g_cmd.psi - yaw;
  
  // Torques completamente indipendenti
  float tau_x = g_kR_roll * e_roll - g_kW_roll * p_rate;
  float tau_y = g_kR_pitch * e_pitch - g_kW_pitch * q_rate;
  float tau_z = g_kR_yaw * e_yaw - g_kW_yaw * r_rate;
  
  // Thrust
  float Fz = MASS * GRAV * thr_eff;
  
  Eigen::Vector4f y_wrench;
  y_wrench << tau_x, tau_y, tau_z, Fz;

  // DEBUG: stampa dettagliata (sempre attivo per diagnostica)
  static uint32_t last_debug = 0;
  if (millis() - last_debug > 500) {
    last_debug = millis();
    Serial.printf("RPY_ekf: R=%.1f P=%.1f Y=%.1f | Cmd: R=%.1f P=%.1f | τ: x=%.3f y=%.3f z=%.3f Fz=%.3f\n",
                  roll*57.3f, pitch*57.3f, yaw*57.3f,
                  g_cmd.phi*57.3f, g_cmd.theta*57.3f,
                  y_wrench(0), y_wrench(1), y_wrench(2), y_wrench(3));
  }

  // ===== Safety layer per BRUSHED con MOSFET =====
  // 1) Failsafe segnale
 /* if (millis() - last_cmd_ms > 300)
  {
    if (g_armed)
      disarm_all();
  }*/

  // 2) Deadman su throttle
 const bool throttle_low = (g_cmd.thr < 0.05f);
 static Eigen::Vector4f f_prev = Eigen::Vector4f::Zero();
  // 3) Se non armato o throttle basso → motori OFF
  if (!g_armed || throttle_low)
  {
    motors_stop_all();
    f_prev = Eigen::Vector4f::Zero();
    return;
  }
// 4) Mixer (y=[τx,τy,τz,Fz] → f=[f1..f4] in Newton)

  Eigen::Vector4f f = g_mixer.mix(y_wrench);

  // 5) Clamp di sicurezza: niente forze negative su brushed
  for (int i = 0; i < 4; ++i)
    if (f(i) < 0.0f)
      f(i) = 0.0f;

  // 6) Slew-rate limit (per motore)
  const float F_tot_max = g_thr_max_factor * MASS * GRAV;
  const float f_full = F_tot_max / 4.0f; // ✅ per motore
  // Slew rate time-based (in unità di f_full al secondo)
  const float SLEW_FFULL_PER_SEC = 4.0f; // regola: 0.5..4.0 in base alla risposta desiderata
  const float max_step = SLEW_FFULL_PER_SEC * f_full * dt;
  for (int i = 0; i < 4; ++i)
  {
    float df = f(i) - f_prev(i);
    if (df > max_step)
      f(i) = f_prev(i) + max_step;
    if (df < -max_step)
      f(i) = f_prev(i) - max_step;
  }
  f_prev = f;

  // 7) Scrittura motori (N → duty 0..1 @30 kHz)
  motors_from_forces_and_write(f);
  
  // Il loop torna subito: il rate è gestito dal check micros all'inizio
  // Frequenza effettiva: 250Hz (4ms period)
}
