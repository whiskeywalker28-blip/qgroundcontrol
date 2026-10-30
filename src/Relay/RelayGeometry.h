// RelayGeometry.h
// Геометрія покриття антени ретранслятора та розрахунок орбіти.
// Чистий C++11 без Qt: компілюється й тестується окремо (див. tests/).
#ifndef RELAYGEOMETRY_H
#define RELAYGEOMETRY_H

#include <cmath>
#include <string>
#include <vector>

namespace relay {

const double kPi = 3.14159265358979323846;
const double kG = 9.80665;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

// Вектор у системі NED: x - північ, y - схід, z - вниз
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator*(double k) const { return Vec3(x * k, y * k, z * k); }
    double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3 &o) const { return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 norm() const { double l = length(); return l < 1e-12 ? *this : *this * (1.0 / l); }
};

struct GeoPoint {
    double lat, lon;
    GeoPoint() : lat(0), lon(0) {}
    GeoPoint(double a, double o) : lat(a), lon(o) {}
};

struct Attitude {             // кути борту, градуси
    double rollDeg, pitchDeg, yawDeg;
    Attitude() : rollDeg(0), pitchDeg(0), yawDeg(0) {}
    Attitude(double r, double p, double y) : rollDeg(r), pitchDeg(p), yawDeg(y) {}
};

struct Mount {                // кути підвісу у зв'язаній СК, градуси
    double panDeg, tiltDeg, rollDeg;   // tilt: 0 = горизонт, -90 = вниз
    Mount() : panDeg(0), tiltDeg(-45), rollDeg(0) {}
    Mount(double p, double t, double r = 0) : panDeg(p), tiltDeg(t), rollDeg(r) {}
};

struct BeamConfig {
    double beamWidthDeg;      // повна ширина по рівню -3 дБ
    int conePoints;           // точок контуру плями
    double maxRangeM;         // макс. дальність, якщо промінь не торкається землі
    bool panContinuous;       // true: ліміт Pan не перевіряється
    double panMinDeg, panMaxDeg;
    double tiltMinDeg, tiltMaxDeg;
    double rollMinDeg, rollMaxDeg;
    double warnMarginDeg;     // зона попередження до межі
    BeamConfig()
        : beamWidthDeg(50.0), conePoints(72), maxRangeM(20000.0), panContinuous(true),
          panMinDeg(-180), panMaxDeg(180), tiltMinDeg(-90), tiltMaxDeg(30),
          rollMinDeg(-25), rollMaxDeg(25), warnMarginDeg(5.0) {}
};

enum LimitState { LimitOk = 0, LimitWarning = 1, LimitHit = 2 };

struct CoverageResult {
    bool valid;
    GeoPoint center;                 // центр плями (перетин осі з землею або точка на MaxRange)
    bool axisHitsGround;
    std::vector<GeoPoint> spot;      // контур плями
    int horizonCut;                  // скільки точок контуру обрізано горизонтом/дальністю
    double slantRangeM;              // похила дальність до центру, NaN якщо вісь не торкається землі
    double panDeg, tiltDeg;          // кути підвісу, що використані для розрахунку
    LimitState limit;
    double limitMarginDeg;
    std::string limitAxis;           // "Pan", "Tilt" або "Roll"
    CoverageResult() : valid(false), axisHitsGround(false), horizonCut(0), slantRangeM(NAN),
                       panDeg(0), tiltDeg(0), limit(LimitOk), limitMarginDeg(0) {}
};

// ---- вектори й повороти ----
Vec3 bodyDir(double panDeg, double tiltDeg);
Vec3 bodyToNed(const Vec3 &v, const Attitude &att);
Vec3 nedToBody(const Vec3 &v, const Attitude &att);
Vec3 coneRay(const Vec3 &axis, double halfDeg, double phiDeg);

// Перетин променя із землею. hAglM: висота борту над землею, м.
// Повертає true, якщо промінь торкається землі на дальності <= maxRange.
bool intersectGround(const Vec3 &dirNed, double hAglM, double maxRangeM, double *northM, double *eastM);

// ---- географія (локальна плоска апроксимація, достатня до ~50 км) ----
GeoPoint offsetLatLon(const GeoPoint &origin, double northM, double eastM);
void offsetNE(const GeoPoint &from, const GeoPoint &to, double *northM, double *eastM);
double distanceM(const GeoPoint &a, const GeoPoint &b);
double bearingDeg(const GeoPoint &from, const GeoPoint &to);   // 0..360, від півночі за годинниковою

// ---- наведення ----
// Кути підвісу, які дивляться з борту на точку, зміщену на (north, east, down) метрів.
Mount mountForTarget(const Attitude &att, double northM, double eastM, double downM);

LimitState checkLimits(const BeamConfig &cfg, const Mount &m, double *marginDeg, std::string *axis);

// Пляма покриття за готовою віссю променя в NED (наприклад, після згладжування).
// Кути підвісу передаються лише для перевірки лімітів та для відображення.
CoverageResult computeCoverageFromAxis(const BeamConfig &cfg, const GeoPoint &uav, double hAglM,
                                       const Vec3 &axisNed, const Mount &mountForLimits);

// Пляма покриття. hAglM: висота борту над поверхнею, на яку проєктуємо пляму.
CoverageResult computeCoverage(const BeamConfig &cfg, const GeoPoint &uav, double hAglM,
                               const Attitude &att, const Mount &mount);

// ---- орбіта ----
// Крен (град.) у сталому віражі: tan(phi) = V^2 / (g R). Швидкість - повітряна + вітер (запас).
double orbitBankDeg(double radiusM, double airspeedMS, double windMS);
// Мінімальний радіус при заданому допустимому крені.
double orbitMinRadiusM(double airspeedMS, double windMS, double bankDeg);

struct OrbitCheck {
    bool ok;                 // true, якщо радіус прийнятний як є
    double requestedM;
    double usedM;            // радіус, який буде застосований (після обмежень)
    double bankDeg;          // крен при usedM
    double allowedBankDeg;   // межа крену мінус запас
    double minRadiusM;       // мінімально допустимий радіус
    std::string note;        // пояснення, якщо радіус змінено
};

OrbitCheck checkOrbit(double radiusM, double airspeedMS, double windMS,
                      double bankLimitDeg, double bankMarginDeg, double maxRadiusM);

// ---- згладжування осі променя ----
class AxisSmoother {
public:
    AxisSmoother() : has_(false), h_(0) {}
    void reset() { has_ = false; }
    // tauSec = 0 вимикає згладжування; jumpResetDeg - різка зміна осі, після якої фільтр стартує заново
    void update(const Vec3 &rawAxis, double rawH, double dtSec, double tauSec, double jumpResetDeg);
    const Vec3 &axis() const { return axis_; }
    double height() const { return h_; }
    bool hasState() const { return has_; }
private:
    bool has_;
    Vec3 axis_;
    double h_;
};

} // namespace relay

#endif // RELAYGEOMETRY_H
