#include <Servo.h>

#define SERVO_AZ_PIN 9
#define SERVO_VER_PIN 10

Servo servoAz;
Servo servoVer;

struct AntennaConfig {
  double lat, lon, alt;
  int azZero, verZero;
  double azGearRatio, verGearRatio;
  int azMinAngle, azMaxAngle;
  int verMinAngle, verMaxAngle;
  double azBacklash, verBacklash;
  double minHorDistForAz;
} antenna;

struct ServoState {
  int currentAngle;
  int targetAngle;
  unsigned long lastStepTime;
  int moveDir; 
  double virtualAz; 
};

ServoState stateAz = {90, 90, 0, 0, 0.0};
ServoState stateVer = {90, 90, 0, 0, 0.0};

const int SERVO_BASE_DELAY = 20;
const int SERVO_MIN_DELAY = 10;
const double SERVO_ACCEL_THRESHOLD = 20.0;

const double WGS84_A = 6378137.0;
const double WGS84_F = 1.0 / 298.257223563;
const double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

struct Target {
  double lat, lon, alt;
};

// Создаем траекторию из 10 точек (дуга)
const int NUM_TARGETS = 10;
Target targets[NUM_TARGETS];

void generateTrajectory() {
  double startLat = 43.2300, endLat = 43.2600;
  double startLon = 76.8800, endLon = 76.9200;
  double maxAlt = 500.0; // Пик дуги

  for (int i = 0; i < NUM_TARGETS; i++) {
    double t = (double)i / (NUM_TARGETS - 1); // Параметр от 0 до 1
    targets[i].lat = startLat + (endLat - startLat) * t;
    targets[i].lon = startLon + (endLon - startLon) * t;
    // Парабола для высоты: h = maxAlt * (1 - (2t - 1)^2)
    targets[i].alt = maxAlt * (1.0 - pow(2.0 * t - 1.0, 2));
    if (targets[i].alt < 10) targets[i].alt = 10; // Не ниже земли
  }
}

inline double deg2rad(double deg) { return deg * 0.017453292519943295; }
inline double rad2deg(double rad) { return rad * 57.29577951308232; }
inline double normalizeAngle360(double angle) {
  angle = fmod(angle, 360.0);
  return (angle < 0.0) ? angle + 360.0 : angle;
}

void lla2ecef(double lat, double lon, double alt, double &x, double &y, double &z) {
  double phi = deg2rad(lat), lambda = deg2rad(lon);
  double sinPhi = sin(phi), cosPhi = cos(phi);
  double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sinPhi * sinPhi);
  x = (N + alt) * cosPhi * cos(lambda);
  y = (N + alt) * cosPhi * sin(lambda);
  z = (N * (1.0 - WGS84_E2) + alt) * sinPhi;
}

void ecef2enu(double dx, double dy, double dz, double refLat, double refLon, double &e, double &n, double &u) {
  double phi = deg2rad(refLat), lambda = deg2rad(refLon);
  double sP = sin(phi), cP = cos(phi), sL = sin(lambda), cL = cos(lambda);
  e = -sL * dx + cL * dy;
  n = -sP * cL * dx - sP * sL * dy + cP * dz;
  u = cP * cL * dx + cP * sL * dy + sP * dz;
}

void computeAzElDist(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2, double &az, double &el, double &dist, bool &azV) {
  double x1, y1, z1, x2, y2, z2;
  lla2ecef(lat1, lon1, alt1, x1, y1, z1);
  lla2ecef(lat2, lon2, alt2, x2, y2, z2);
  double dx = x2 - x1, dy = y2 - y1, dz = z2 - z1;
  dist = sqrt(dx * dx + dy * dy + dz * dz);
  double e, n, u;
  ecef2enu(dx, dy, dz, lat1, lon1, e, n, u);
  double hD = sqrt(e * e + n * n);
  if (hD < antenna.minHorDistForAz) { azV = false; az = 0; el = (u > 0) ? 90 : -90; }
  else { azV = true; az = normalizeAngle360(rad2deg(atan2(e, n))); el = rad2deg(atan2(u, hD)); }
}

int calculateTargetServo(double targetVal, double gearRatio, int zeroPos, double backlash, int minA, int maxA, ServoState &state, bool isAz) {
  double idealAngle = zeroPos + (targetVal / gearRatio);
  int newDir = (idealAngle > state.currentAngle) ? 1 : -1;
  if (state.moveDir != 0 && newDir != state.moveDir) idealAngle += newDir * backlash;
  state.moveDir = newDir;
  return constrain((int)round(idealAngle), minA, maxA);
}

void processServo(Servo &servo, ServoState &state) {
  if (state.currentAngle == state.targetAngle) return;
  int diff = abs(state.targetAngle - state.currentAngle);
  int dly = (diff > SERVO_ACCEL_THRESHOLD) ? SERVO_MIN_DELAY : SERVO_BASE_DELAY;
  int step = (diff > SERVO_ACCEL_THRESHOLD) ? 2 : 1;

  if (millis() - state.lastStepTime >= (unsigned long)dly) {
    state.lastStepTime = millis();
    int dir = (state.targetAngle > state.currentAngle) ? 1 : -1;
    state.currentAngle += dir * min(step, diff);
    servo.write(state.currentAngle);
  }
}

void printDetailedStatus(int id, double dist, double az, double el) {
  Serial.println(F(" TARGET UPDATE "));
  Serial.print(F("Point: ")); Serial.print(id);
  Serial.print(F(" | Dist: ")); Serial.print(dist, 1); Serial.println(F(" m"));
  
  Serial.print(F("REAL  -> Az: ")); Serial.print(az, 1); 
  Serial.print(F("° | El: ")); Serial.print(el, 1); Serial.println(F("°"));
  
  Serial.print(F("SERVO -> Az: ")); Serial.print(stateAz.targetAngle);
  Serial.print(F(" (now ")); Serial.print(stateAz.currentAngle);
  Serial.print(F(") | Ver: ")); Serial.print(stateVer.targetAngle);
  Serial.print(F(" (now ")); Serial.print(stateVer.currentAngle);
  Serial.println(F(")"));
  Serial.println();
}

void trackTarget(int index) {
  double az, el, dist;
  bool azV;
  computeAzElDist(antenna.lat, antenna.lon, antenna.alt, targets[index].lat, targets[index].lon, targets[index].alt, az, el, dist, azV);
  
  if (azV) stateAz.targetAngle = calculateTargetServo(az, antenna.azGearRatio, antenna.azZero, antenna.azBacklash, antenna.azMinAngle, antenna.azMaxAngle, stateAz, true);
  stateVer.targetAngle = calculateTargetServo(el, antenna.verGearRatio, antenna.verZero, antenna.verBacklash, antenna.verMinAngle, antenna.verMaxAngle, stateVer, false);

  printDetailedStatus(index, dist, az, el);
}

void setup() {
  Serial.begin(9600);
  // Настройки антенны: Lat, Lon, Alt, AzZero, VerZero, AzRatio, VerRatio, MinAz, MaxAz, MinVer, MaxVer, AzBack, VerBack, MinDist
  antenna = {43.238949, 76.889709, 100.0, 90, 90, 3.0, 1.0, 0, 180, 0, 180, 0.5, 0.3, 1.0};
  
  generateTrajectory();
  
  servoAz.attach(SERVO_AZ_PIN);
  servoVer.attach(SERVO_VER_PIN);
  
  stateAz.currentAngle = antenna.azZero; 
  stateVer.currentAngle = antenna.verZero;
  servoAz.write(antenna.azZero); 
  servoVer.write(antenna.verZero);
  
  Serial.println(F("System Ready. Starting Trajectory..."));
  delay(2000);
}

int currentTarget = 0;

void loop() {
  processServo(servoAz, stateAz);
  processServo(servoVer, stateVer);

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 3000) { // Каждые 3 секунды новая точка
    trackTarget(currentTarget);
    currentTarget = (currentTarget + 1) % NUM_TARGETS;
    lastUpdate = millis();
  }
}
