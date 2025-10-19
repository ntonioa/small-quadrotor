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
#include <SPI.h>
#include "ICM_20948.h"
#include <math.h>

#include "predict.h"
#include "update.h"
using namespace sym;

#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

ICM_20948_SPI imu;

// === WiFi AP ===
static const char* WIFI_SSID = "DRONE_AP";
static const char* WIFI_PASS = "12345678";

// === UDP/MAVLink ===
static const uint16_t QGC_PORT = 14550;
WiFiUDP udp;
IPAddress qgc_bcast(192,168,4,255); 
IPAddress qgc_ip = qgc_bcast; 

// === MAVLink IDs ===
uint8_t sysid = 1;
uint8_t compid = MAV_COMP_ID_AUTOPILOT1;

// timing
uint32_t last_hb_ms = 0;
uint32_t last_att_ms = 0;

// Eigen types
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Mat77 = Eigen::Matrix<float, 7, 7>;
using Mat33 = Eigen::Matrix<float, 3, 3>;

// EKF globals
Vec7 x;     // stato [qw,qx,qy,qz,bx,by,bz] (puoi anche mettere solo 4 se non hai bias)
Mat77 P;    // covarianza
Mat77 Q;    // rumore processo
Mat33 R;    // rumore misura (solo acc)

unsigned long last_time;

// Struct per calibrazione
struct {
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
} calibration;

// ----------- CALIBRAZIONE ------------

void calibrateAccelGyro(int samples = 500) {
  float accelSum[3] = {0};
  float gyroSum[3] = {0};
  Serial.println("\n*** Calibrazione accelerometro/giroscopio (NON MUOVERE IL DRONE) ***");
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      accelSum[0] += imu.accX();
      accelSum[1] += imu.accY();
      accelSum[2] += imu.accZ() - 1000.0; // sottrai 1g (1000 mg)
      gyroSum[0] += imu.gyrX();
      gyroSum[1] += imu.gyrY();
      gyroSum[2] += imu.gyrZ();
      delay(5);
    }
  }
  for(int i = 0; i < 3; i++) {
    calibration.accelOffset[i] = accelSum[i] / samples;
    calibration.gyroOffset[i] = gyroSum[i] / samples;
  }
  Serial.print("Accel Offset: ");
  Serial.print(calibration.accelOffset[0],2); Serial.print(" ");
  Serial.print(calibration.accelOffset[1],2); Serial.print(" ");
  Serial.println(calibration.accelOffset[2],2);
  Serial.print("Gyro Offset: ");
  Serial.print(calibration.gyroOffset[0],3); Serial.print(" ");
  Serial.print(calibration.gyroOffset[1],3); Serial.print(" ");
  Serial.println(calibration.gyroOffset[2],3);
}
// ----------- MAVLink Heartbeat ------------
static void mav_send(const mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket(qgc_ip, QGC_PORT);
  udp.write(buf, len);
  udp.endPacket();
}

static void send_heartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(
    sysid, compid, &msg,
    MAV_TYPE_QUADROTOR,
    MAV_AUTOPILOT_GENERIC,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    0,                 // custom_mode
    MAV_STATE_ACTIVE
  );
  mav_send(msg);
}

static void send_attitude(float roll, float pitch, float yaw, float p, float q, float r) {
  mavlink_message_t msg;
  mavlink_msg_attitude_pack(
    sysid, compid, &msg,
    millis(), roll, pitch, yaw, p, q, r   // <-- millis()
  );
  mav_send(msg);
}

static void send_attitude_quat(float qw, float qx, float qy, float qz,
                               float p, float q, float r) {
  mavlink_message_t msg;
  float cov[9];
  for (int i=0;i<9;i++) cov[i] = NAN;
  mavlink_msg_attitude_quaternion_pack(
    sysid, compid, &msg,
    millis(), qw, qx, qy, qz, p, q, r, cov   // <-- millis()
  );
  mav_send(msg);
}


// ----------- SETUP ------------

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(200);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  udp.begin(QGC_PORT);

  while(true) {
    imu.begin(CS_PIN, SPI);
    if(imu.status == ICM_20948_Stat_Ok) {
      Serial.println("IMU connessa correttamente");
      break;
    }
    Serial.println("Connessione IMU fallita, riprovo...");
    delay(500);
  }

  imu.swReset(); delay(10);
  imu.sleep(false); imu.lowPower(false);
  imu.startupMagnetometer(); // Puoi anche rimuovere se vuoi
  delay(100);

  calibrateAccelGyro();

  // EKF inizializzazione
  x << 1,0,0,0, 0,0,0; // oppure solo 4 se non hai bias
  P.setIdentity();  P *= 0.01f;
  Q.setIdentity();  Q *= 0.1f;
  R.setIdentity();  R *= 1.0f;

  last_time = millis();
  Serial.println("Setup completo, ready!");
}

// ----------- LOOP ------------

void loop() {
  if (!imu.dataReady()) {
    delay(2);
    return;
  }

  // Calcola dt
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  imu.getAGMT();

  // Lettura e calibrazione
  float accX = imu.accX() - calibration.accelOffset[0];
  float accY = imu.accY() - calibration.accelOffset[1];
  float accZ = imu.accZ() - calibration.accelOffset[2];
  float gyrX = imu.gyrX() - calibration.gyroOffset[0];
  float gyrY = imu.gyrY() - calibration.gyroOffset[1];
  float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];

  // Normalizza accelerometro!
  float acc_norm = sqrt(accX*accX + accY*accY + accZ*accZ);
  accX /= acc_norm;
  accY /= acc_norm;
  accZ /= acc_norm;

  Serial.print("GYRO: ");
  Serial.print(gyrX, 3); Serial.print(" ");
  Serial.print(gyrY, 3); Serial.print(" ");
  Serial.println(gyrZ, 3);

  Serial.print("ACC: ");
  Serial.print(accX, 3); Serial.print(" ");
  Serial.print(accY, 3); Serial.print(" ");
  Serial.println(accZ, 3);

  // Costruisci vettore misura z = acc normalizzato
  Vec3 z;
  z << accX, accY, accZ;

  // EKF predict/update (usa le tue funzioni generate da symforce)
  Vec7 x_pred; Mat77 P_pred;
  Predict(x, dt, P, Q, &x_pred, &P_pred);

  Vec7 x_upd; Mat77 P_upd;
  Update(x_pred, P_pred, z, R, &x_upd, &P_upd);

  // Normalizza il quaternione!
  x_upd.head<4>().normalize();
  x = x_upd;
  P = P_upd;

  // Estrai quat UNA volta
  float qw = x(0), qx = x(1), qy = x(2), qz = x(3);

  // (opzionale) RPY per debug
  float roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
  float pitch = asin (2*(qw*qy - qz*qx));
  float yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

  Serial.print("EKF RPY: ");
  Serial.print(roll  * 180.0/M_PI, 1); Serial.print(" ");
  Serial.print(pitch * 180.0/M_PI, 1); Serial.print(" ");
  Serial.println(yaw   * 180.0/M_PI, 1);

  // Bias dal tuo stato (se x(4..6) sono bias)
  float bx = x(4), by = x(5), bz = x(6);

  // Gyro in rad/s (SparkFun dà deg/s)
  const float d2r = M_PI / 180.0f;
  float p_rate = (gyrX - bx) * d2r;
  float q_rate = (gyrY - by) * d2r;
  float r_rate = (gyrZ - bz) * d2r;

  // Invio periodico MAVLink
  uint32_t now_ms = millis();
  if (now_ms - last_hb_ms >= 1000) {
    last_hb_ms = now_ms;
    send_heartbeat();
  }
  if (now_ms - last_att_ms >= 20) {   // ~50 Hz
    last_att_ms = now_ms;
    send_attitude_quat(qw, qx, qy, qz, p_rate, q_rate, r_rate);
    send_attitude(roll, pitch, yaw, p_rate, q_rate, r_rate);     // <-- aggiungi questa riga

  }

  // // Estrai roll/pitch/yaw (in gradi)
  // float qw = x(0), qx = x(1), qy = x(2), qz = x(3);
  // float roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
  // float pitch = asin (2*(qw*qy - qz*qx));
  // float yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

  // Serial.print("EKF RPY: ");
  // Serial.print(roll  * 180.0/M_PI, 1); Serial.print(" ");
  // Serial.print(pitch * 180.0/M_PI, 1); Serial.print(" ");
  // Serial.println(yaw   * 180.0/M_PI, 1);

  delay(10); // loop veloce, puoi aumentare se vuoi
}
