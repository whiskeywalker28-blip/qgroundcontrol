#include "RelayGeometry.h"

#include <algorithm>
#include <cstdio>

namespace relay {

static const double kRelayEarthR = 6378137.0;

Vec3 bodyDir(double panDeg, double tiltDeg)
{
    double p = deg2rad(panDeg), t = deg2rad(tiltDeg);
    return Vec3(std::cos(t) * std::cos(p), std::cos(t) * std::sin(p), -std::sin(t));
}

// Матриця R = Rz(yaw) * Ry(pitch) * Rx(roll): борт -> NED
static void relayRotation(const Attitude &a, double R[3][3])
{
    double r = deg2rad(a.rollDeg), p = deg2rad(a.pitchDeg), y = deg2rad(a.yawDeg);
    double cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
    double cy = std::cos(y), sy = std::sin(y);
    R[0][0] = cp * cy; R[0][1] = sr * sp * cy - cr * sy; R[0][2] = cr * sp * cy + sr * sy;
    R[1][0] = cp * sy; R[1][1] = sr * sp * sy + cr * cy; R[1][2] = cr * sp * sy - sr * cy;
    R[2][0] = -sp;     R[2][1] = sr * cp;                R[2][2] = cr * cp;
}

Vec3 bodyToNed(const Vec3 &v, const Attitude &att)
{
    double R[3][3];
    relayRotation(att, R);
    return Vec3(R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
                R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
                R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z);
}

Vec3 nedToBody(const Vec3 &v, const Attitude &att)
{
    double R[3][3];
    relayRotation(att, R);
    return Vec3(R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z,
                R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z,
                R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z);
}

Vec3 coneRay(const Vec3 &axis, double halfDeg, double phiDeg)
{
    Vec3 up = std::fabs(axis.z) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
    Vec3 u = axis.cross(up).norm();
    Vec3 w = axis.cross(u).norm();
    double h = deg2rad(halfDeg), f = deg2rad(phiDeg);
    Vec3 dir = axis * std::cos(h) + (u * std::cos(f) + w * std::sin(f)) * std::sin(h);
    return dir.norm();
}

bool intersectGround(const Vec3 &d, double hAglM, double maxRangeM, double *northM, double *eastM)
{
    double hor = std::sqrt(d.x * d.x + d.y * d.y);
    if (d.z > 1e-3) {
        double t = hAglM / d.z;
        if (hor * t <= maxRangeM) {
            *northM = d.x * t;
            *eastM = d.y * t;
            return true;
        }
    }
    // не торкається землі або надто далеко: обрізаємо за maxRange у напрямку променя
    if (hor < 1e-9) {
        *northM = 0;
        *eastM = 0;
        return false;
    }
    *northM = d.x / hor * maxRangeM;
    *eastM = d.y / hor * maxRangeM;
    return false;
}

GeoPoint offsetLatLon(const GeoPoint &o, double northM, double eastM)
{
    double dLat = rad2deg(northM / kRelayEarthR);
    double dLon = rad2deg(eastM / (kRelayEarthR * std::cos(deg2rad(o.lat))));
    return GeoPoint(o.lat + dLat, o.lon + dLon);
}

void offsetNE(const GeoPoint &from, const GeoPoint &to, double *northM, double *eastM)
{
    *northM = deg2rad(to.lat - from.lat) * kRelayEarthR;
    *eastM = deg2rad(to.lon - from.lon) * kRelayEarthR * std::cos(deg2rad(from.lat));
}

double distanceM(const GeoPoint &a, const GeoPoint &b)
{
    double n, e;
    offsetNE(a, b, &n, &e);
    return std::sqrt(n * n + e * e);
}

double bearingDeg(const GeoPoint &from, const GeoPoint &to)
{
    double n, e;
    offsetNE(from, to, &n, &e);
    double b = rad2deg(std::atan2(e, n));
    return b < 0 ? b + 360.0 : b;
}

Mount mountForTarget(const Attitude &att, double northM, double eastM, double downM)
{
    Vec3 body = nedToBody(Vec3(northM, eastM, downM).norm(), att);
    double z = std::max(-1.0, std::min(1.0, body.z));
    return Mount(rad2deg(std::atan2(body.y, body.x)), -rad2deg(std::asin(z)), 0.0);
}

LimitState checkLimits(const BeamConfig &cfg, const Mount &m, double *marginDeg, std::string *axis)
{
    double mPan = cfg.panContinuous ? 1e9 : std::min(m.panDeg - cfg.panMinDeg, cfg.panMaxDeg - m.panDeg);
    double mTilt = std::min(m.tiltDeg - cfg.tiltMinDeg, cfg.tiltMaxDeg - m.tiltDeg);
    double mRoll = std::min(m.rollDeg - cfg.rollMinDeg, cfg.rollMaxDeg - m.rollDeg);
    double mn = std::min(mPan, std::min(mTilt, mRoll));
    const char *which = (mRoll <= mTilt && mRoll <= mPan) ? "Roll" : (mTilt <= mPan ? "Tilt" : "Pan");
    if (marginDeg) *marginDeg = mn;
    if (axis) *axis = which;
    if (mn <= 0.5) return LimitHit;
    if (mn <= cfg.warnMarginDeg) return LimitWarning;
    return LimitOk;
}

CoverageResult computeCoverageFromAxis(const BeamConfig &cfg, const GeoPoint &uav, double hAglM,
                                       const Vec3 &axis, const Mount &mount)
{
    CoverageResult r;
    double h = std::max(hAglM, 1.0);

    double an, ae;
    r.axisHitsGround = intersectGround(axis, h, cfg.maxRangeM, &an, &ae);
    r.center = offsetLatLon(uav, an, ae);
    if (r.axisHitsGround)
        r.slantRangeM = std::sqrt(an * an + ae * ae + h * h);

    int n = std::max(8, cfg.conePoints);
    for (int i = 0; i < n; ++i) {
        double phi = 360.0 * i / n;
        Vec3 ray = coneRay(axis, cfg.beamWidthDeg / 2.0, phi);
        double rn, re;
        if (!intersectGround(ray, h, cfg.maxRangeM, &rn, &re))
            r.horizonCut++;
        r.spot.push_back(offsetLatLon(uav, rn, re));
    }

    r.panDeg = mount.panDeg;
    r.tiltDeg = mount.tiltDeg;
    r.limit = checkLimits(cfg, mount, &r.limitMarginDeg, &r.limitAxis);
    r.valid = true;
    return r;
}

CoverageResult computeCoverage(const BeamConfig &cfg, const GeoPoint &uav, double hAglM,
                               const Attitude &att, const Mount &mount)
{
    return computeCoverageFromAxis(cfg, uav, hAglM, bodyToNed(bodyDir(mount.panDeg, mount.tiltDeg), att), mount);
}

double orbitBankDeg(double radiusM, double airspeedMS, double windMS)
{
    double v = std::max(airspeedMS + windMS, 0.1);
    double rr = std::max(radiusM, 1.0);
    return rad2deg(std::atan(v * v / (kG * rr)));
}

double orbitMinRadiusM(double airspeedMS, double windMS, double bankDeg)
{
    double v = std::max(airspeedMS + windMS, 0.1);
    double b = std::max(bankDeg, 1.0);
    return v * v / (kG * std::tan(deg2rad(b)));
}

OrbitCheck checkOrbit(double radiusM, double airspeedMS, double windMS,
                      double bankLimitDeg, double bankMarginDeg, double maxRadiusM)
{
    OrbitCheck c;
    c.requestedM = radiusM;
    c.allowedBankDeg = std::max(bankLimitDeg - bankMarginDeg, 5.0);
    c.minRadiusM = orbitMinRadiusM(airspeedMS, windMS, c.allowedBankDeg);
    c.usedM = radiusM;
    c.ok = true;

    char buf[256];
    if (c.usedM < c.minRadiusM) {
        c.usedM = std::ceil(c.minRadiusM);
        c.ok = false;
        std::snprintf(buf, sizeof(buf),
                      "Радіус збільшено з %.0f до %.0f м: при швидкості %.0f м/с (з вітром %.0f) крен перевищив би %.0f°.",
                      radiusM, c.usedM, airspeedMS, windMS, c.allowedBankDeg);
        c.note = buf;
    }
    if (c.usedM > maxRadiusM) {
        c.usedM = maxRadiusM;
        c.ok = false;
        std::snprintf(buf, sizeof(buf), "Радіус обмежено до %.0f м.", maxRadiusM);
        c.note = buf;
    }
    c.bankDeg = orbitBankDeg(c.usedM, airspeedMS, windMS);
    return c;
}

void AxisSmoother::update(const Vec3 &raw, double rawH, double dtSec, double tauSec, double jumpResetDeg)
{
    bool jump = has_ && axis_.dot(raw) < std::cos(deg2rad(jumpResetDeg));
    if (!has_ || jump) {
        axis_ = raw;
        h_ = rawH;
        has_ = true;
        return;
    }
    double a = tauSec <= 1e-6 ? 1.0 : 1.0 - std::exp(-std::max(dtSec, 0.0) / tauSec);
    axis_ = (axis_ * (1.0 - a) + raw * a).norm();
    h_ += a * (rawH - h_);
}

} // namespace relay
