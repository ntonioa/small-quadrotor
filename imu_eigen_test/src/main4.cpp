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

unsigned long last_time;

// ====== Calibrazione grezza ======
struct
{
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
} calibration;

// ======================= MOTORI: DUTY @ 78 kHz  =======================
#include <algorithm>

#define PWM_FREQUENCY 78000 // Hz
#define PWM_RESOLUTION 10   // bit

// prima: {12, 13, 14, 15}
static const int MOTOR_PINS[4] = {12, 15, 14, 13};
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

static constexpr float HOVER_THR = 0.50f;        // stick di hover ~50%
static constexpr float THRUST_MAX_FACTOR = 4.0f; // full stick = 5.0×(m*g) totale

static Eigen::Matrix3f J_body()
{
  Eigen::Matrix3f J;
  J << 7.1031997e-5, 1.32488e-7, -1.82229e-7,
      1.32488e-7, 6.8947577e-5, 6.66764e-7,
      -1.82229e-7, 6.66764e-7, 1.323e-4;
  return J;
}

static const Eigen::Vector3f kR(5.0f, 5.0f, 0.001f);
static const Eigen::Vector3f kW(10.0f, 10.0f, 0.001f);

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
  float km = 0.016f;
  Mat44 A;
  Mixer()
  {
    // 0=FL, 1=FR, 2=RR, 3=RL
    const float s[4] = {+1, -1, +1, -1};    // CCW, CW, CCW, CW
    float tx[4] = {+1, -1, +1, -1};         // τx (roll)
    float ty[4] = {-1, -1, +1, +1};         // τy (pitch)
    float tz[4] = {s[0], s[1], s[2], s[3]}; // τz (yaw drag)

    float txs = 1.0f / (2.0f * l), tys = 1.0f / (2.0f * l), tzs = 1.0f / (4.0f * km);
    A.setZero();
    for (int i = 0; i < 4; i++)
    {
      A(i, 0) = tx[i] * txs; // τx
      A(i, 1) = ty[i] * tys; // τy
      A(i, 2) = tz[i] * tzs; // τz
      A(i, 3) = 0.25f;       // Fz
    }
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
  const float F_tot_max = THRUST_MAX_FACTOR * MASS * GRAV;
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

    if (msg.msgid != MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
      continue;
    }
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
    else
    {
      state = 0;
    }
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
  Rm *= powf(0.03f, 2); // sigma ≈ 0.03 -> buono per ICM-20948 normalizzato

  // --- Q: rumore di processo ---
  Qm.setZero();
  // "rumore orientamento" (effetto rumore gyro sulle eq. di stato)
  Qm(0, 0) = 1e-5f;
  Qm(1, 1) = 1e-5f;
  Qm(2, 2) = 1e-5f;
  Qm(3, 3) = 1e-5f;
  // random-walk dei bias (lenti ma non rigidi)
  Qm(4, 4) = powf(0.001f, 2); // ~0.001 rad/s /√s
  Qm(5, 5) = powf(0.001f, 2);
  Qm(6, 6) = powf(0.001f, 2);

  last_time = millis();
  Serial.println("Setup completo. Controller pronto (MANUAL_CONTROL via QGC).");
}

// ====== LOOP ======
void loop()
{
  if (!imu.dataReady())
  {
    qgc_parse_and_update_cmd();
    delay(2);
    return;
  }

  // dt
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  // IMU raw
  imu.getAGMT();
  float accX = imu.accX() - calibration.accelOffset[0];
  float accY = imu.accY() - calibration.accelOffset[1];
  float accZ = imu.accZ() - calibration.accelOffset[2];
  float gyrX = imu.gyrX() - calibration.gyroOffset[0];
  float gyrY = imu.gyrY() - calibration.gyroOffset[1];
  float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];

  // Normalize acc (mg → unit vector)
  float acc_norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
  if (acc_norm > 1e-6f)
  {
    accX /= acc_norm;
    accY /= acc_norm;
    accZ /= acc_norm;
  }

  // EKF predict/update
  Vec3 z;
  z << -accX, -accY, -accZ; // misura verso gravità

  Vec7 x_pred;
  Mat77 P_pred;
  Predict(x, dt, P, Qm, &x_pred, &P_pred);
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
  const float d2r = M_PI / 180.0f;
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
    send_attitude_quat({qw, qx, qy, qz}, p_rate, q_rate, r_rate);
    send_attitude(roll, pitch, yaw, p_rate, q_rate, r_rate);
  }

  // ===== Leggi comandi QGC =====
  qgc_parse_and_update_cmd();

  /// ===== Controller =====
  Eigen::Matrix<float, 4, 1> q_storage;
  q_storage << qw, qx, qy, qz;
  sym::Quaternion<float> q_sym = sym::Quaternion<float>::FromStorage(q_storage);
  Eigen::Vector3f omega_body(p_rate, q_rate, r_rate);

  // 0..1 stick → 0..THRUST_MAX_FACTOR (hover a HOVER_THR)
  float thr_scale = (HOVER_THR > 1e-3f) ? (1.0f / HOVER_THR) : 2.0f;
  float thr_eff = std::clamp(g_cmd.thr * thr_scale, 0.0f, THRUST_MAX_FACTOR);

  Eigen::Vector4f y_wrench;
  sym::AttitudeControllerStep<float>(
      q_sym, omega_body,
      g_cmd.phi, g_cmd.theta, g_cmd.psi,
      kR, kW, J_body(),
      MASS, GRAV, thr_eff,
      &y_wrench);

  // ===== Safety layer per BRUSHED con MOSFET =====
  // 1) Failsafe segnale
  if (millis() - last_cmd_ms > 300)
  {
    if (g_armed)
      disarm_all();
  }

  // 2) Deadman su throttle
  const bool throttle_low = (g_cmd.thr < 0.05f);

  // 3) Se non armato o throttle basso → motori OFF
  if (!g_armed || throttle_low)
  {
    motors_stop_all();
    return;
  }

  // 4) Mixer (y=[τx,τy,τz,Fz] → f=[f1..f4] in Newton)
  Eigen::Vector4f f = g_mixer.mix(y_wrench);

  // 5) Clamp di sicurezza: niente forze negative su brushed
  for (int i = 0; i < 4; ++i)
    if (f(i) < 0.0f)
      f(i) = 0.0f;

  // 6) Slew-rate limit (per motore)
  static Eigen::Vector4f f_prev = Eigen::Vector4f::Zero();
  const float F_tot_max = THRUST_MAX_FACTOR * MASS * GRAV;
  const float f_full = F_tot_max / 4.0f; // ✅ per motore
  const float max_step = 0.8f * f_full;  // ~50% per ciclo
  for (int i = 0; i < 4; ++i)
  {
    float df = f(i) - f_prev(i);
    if (df > max_step)
      f(i) = f_prev(i) + max_step;
    if (df < -max_step)
      f(i) = f_prev(i) - max_step;
  }
  f_prev = f;

  // 7) Scrittura motori (N → duty 0..1 @78 kHz)
  motors_from_forces_and_write(f);

  delay(10); // ~100 Hz loop
}
