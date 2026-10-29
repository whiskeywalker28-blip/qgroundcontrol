// FlyViewCustomLayer.qml  —  Relay Coverage v3 для QGroundControl v5.0.x (Qt 6), ArduPlane
//
//  • Пляма покриття антени ретранслятора (конус діаграми -> земля).
//  • ТОЧКА НАВЕДЕННЯ (ROI) і ТОЧКА БАРАЖУВАННЯ (центр орбіти) — незалежні:
//      Cmd+Shift+ЛКМ  — наведення;  Cmd+Option+ЛКМ — баражування;  ПКМ — меню.
//      Обидва маркери можна ПЕРЕТЯГУВАТИ мишкою.
//  • Будь-яка зміна орбіти (кнопка, меню, перенесення центру, підібране) — ТІЛЬКИ після підтвердження.
//  • Введення координат: MGRS або «lat, lon» текстом / з буфера обміну.
//  • «Підібрати (geofence)» — прихований розділ: пропонує центр/радіус орбіти в межах дозволеної зони.
//  • Журнал дій (зберігається між запусками, експорт CSV у буфер обміну).
//
// macOS: Qt міняє місцями Cmd і Ctrl, тому Cmd = Qt.ControlModifier.

import QtQuick
import QtQuick.Controls
import QtLocation
import QtPositioning
import QtCore

import QGroundControl
import QGroundControl.Controls

Item {
    id: _root

    // --- інтерфейс шару Fly View у v5.0.x ---
    property var parentToolInsets
    property var totalToolInsets: _toolInsets
    property var mapControl

    QGCToolInsets {
        id: _toolInsets
        leftEdgeTopInset:      parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:   parentToolInsets.leftEdgeCenterInset
        leftEdgeBottomInset:   parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:     parentToolInsets.rightEdgeTopInset
        rightEdgeCenterInset:  Math.max(parentToolInsets.rightEdgeCenterInset, _root.width - panel.x)
        rightEdgeBottomInset:  parentToolInsets.rightEdgeBottomInset
        topEdgeLeftInset:      parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:    parentToolInsets.topEdgeCenterInset
        topEdgeRightInset:     parentToolInsets.topEdgeRightInset
        bottomEdgeLeftInset:   parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset: parentToolInsets.bottomEdgeCenterInset
        bottomEdgeRightInset:  parentToolInsets.bottomEdgeRightInset
    }

    // ------------------------- НАЛАШТУВАННЯ -------------------------
    readonly property real beamWidthDeg:   50
    readonly property int  conePoints:     72
    readonly property real maxRangeM:      20000
    readonly property real groundOffsetM:  0      // висота місцевості відносно дому
    readonly property real planBankDeg:    30     // розрахунковий крен (ліміт літака 35°)
    readonly property real orbitMinFactor: 1.1
    readonly property bool orbitClockwise: true
    readonly property real tiltMinDeg:    -60     // ліміти підвісу для орбіти (з вашого плагіна)
    readonly property real tiltMaxDeg:     25
    readonly property real warnMarginDeg:  5
    readonly property real smoothTauSec:   1.5
    readonly property real jumpResetDeg:   25
    readonly property bool autoRoi:        true
    readonly property real fixedPanDeg:    0
    readonly property real fixedTiltDeg:  -45
    readonly property bool repositionAltIsAmsl: false   // ПЕРЕВІРИТИ В SITL
    readonly property int  logMaxEntries:  300
    readonly property real dragHitPx:      22

    readonly property real d2r: Math.PI / 180
    readonly property real r2d: 180 / Math.PI
    readonly property real earthR: 6378137
    readonly property string mgrsBands: "CDEFGHJKLMNPQRSTUVWX"
    readonly property var mgrsCols: ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"]
    readonly property string mgrsRows: "ABCDEFGHJKLMNPQRSTUV"

    // ------------------------- СТАН -------------------------
    property var vehicle: QGroundControl.multiVehicleManager.activeVehicle
    property var aimCoord: null       // точка наведення
    property var loiterCoord: null    // точка баражування
    property var activeOrbit: null    // орбіта, яку реально відправлено на борт
    property var pendingAction: null  // дія, що чекає підтвердження
    property var _ctxCoord: null
    property string statusText: ""
    property bool orbitActive: false

    property var zones: null          // geofence з QGC: {incl:[], excl:[]}
    property var suggestion: null
    property string finderText: ""
    property bool showFinder: false
    property bool showLog: false

    property string _drag: ""
    property var _dragStart: null
    property var _dragOff: Qt.point(0, 0)

    property bool _hasState: false
    property double _lastT: 0
    property var _sAxis: [1, 0, 0]
    property real _sH: 100

    property var _spot: null
    property var _contour: null
    property var _axisLine: null
    property var _orbitLine: null
    property var _aimMark: null
    property var _loiterMark: null

    Settings {
        id: store
        category: "RelayCoverage"
        property string logJson: "[]"
    }
    TextEdit { id: clipHelper; visible: false; width: 10; height: 10 }

    Component.onCompleted: logEvent("START", qsTr("Шар Relay Coverage завантажено"))

    // ===================================================================
    //  МАТЕМАТИКА (MGRS/UTM, геозони, підбір) — той самий код, що перевірений тестами в Node
    // ===================================================================
    // ===== Чиста математика (без QML-залежностей). Використовує глобальні d2r, r2d, earthR. =====

    function deltaNE(lat1, lon1, lat2, lon2) {
        return { n: (lat2-lat1)*d2r*earthR, e: (lon2-lon1)*d2r*earthR*Math.cos(lat1*d2r) }
    }
    function offsetLL(lat, lon, n, e) {
        return { lat: lat + n/earthR*r2d, lon: lon + e/(earthR*Math.cos(lat*d2r))*r2d }
    }
    function distM(lat1, lon1, lat2, lon2) {
        var d = deltaNE(lat1, lon1, lat2, lon2)
        return Math.sqrt(d.n*d.n + d.e*d.e)
    }

    // ---------------- UTM (WGS84, ряди Крюгера) ----------------
    function utmZoneFor(lat, lon) {
        var l = ((lon + 180) % 360 + 360) % 360 - 180
        var z = Math.floor((l + 180) / 6) + 1
        if (lat >= 56 && lat < 64 && l >= 3 && l < 12) z = 32
        if (lat >= 72 && lat < 84) {
            if (l >= 0 && l < 9) z = 31
            else if (l >= 9 && l < 21) z = 33
            else if (l >= 21 && l < 33) z = 35
            else if (l >= 33 && l < 42) z = 37
        }
        return z
    }
    function _kr() {
        var a = 6378137.0, f = 1/298.257223563
        var n = f/(2-f), n2 = n*n, n3 = n2*n, n4 = n3*n
        return {
            n: n, A: a/(1+n)*(1 + n2/4 + n4/64),
            al: [n/2 - 2*n2/3 + 5*n3/16 + 41*n4/180, 13*n2/48 - 3*n3/5 + 557*n4/1440,
                 61*n3/240 - 103*n4/140, 49561*n4/161280],
            be: [n/2 - 2*n2/3 + 37*n3/96 - n4/360, n2/48 + n3/15 - 437*n4/1440,
                 17*n3/480 - 37*n4/840, 4397*n4/161280],
            de: [2*n - 2*n2/3 - 2*n3 + 116*n4/45, 7*n2/3 - 8*n3/5 - 227*n4/45,
                 56*n3/15 - 136*n4/35, 4279*n4/630]
        }
    }
    function utmForward(lat, lon, zoneIn) {
        var k = _kr(), k0 = 0.9996
        var zone = zoneIn ? zoneIn : utmZoneFor(lat, lon)
        var lon0 = (zone - 1)*6 - 180 + 3
        var phi = lat*d2r, lam = (lon - lon0)*d2r
        var s = 2*Math.sqrt(k.n)/(1 + k.n), sp = Math.sin(phi)
        var t = Math.sinh(Math.atanh(sp) - s*Math.atanh(s*sp))
        var xi0 = Math.atan2(t, Math.cos(lam))
        var eta0 = Math.atanh(Math.sin(lam)/Math.sqrt(1 + t*t))
        var xi = xi0, eta = eta0
        for (var j = 1; j <= 4; ++j) {
            xi  += k.al[j-1]*Math.sin(2*j*xi0)*Math.cosh(2*j*eta0)
            eta += k.al[j-1]*Math.cos(2*j*xi0)*Math.sinh(2*j*eta0)
        }
        var E = 500000 + k0*k.A*eta, N = k0*k.A*xi
        if (lat < 0) N += 10000000
        return { zone: zone, hemi: lat < 0 ? "S" : "N", e: E, n: N }
    }
    function utmInverse(zone, hemi, E, N) {
        var k = _kr(), k0 = 0.9996
        var xi = (hemi === "S" ? N - 10000000 : N)/(k0*k.A), eta = (E - 500000)/(k0*k.A)
        var xi1 = xi, eta1 = eta
        for (var j = 1; j <= 4; ++j) {
            xi1  -= k.be[j-1]*Math.sin(2*j*xi)*Math.cosh(2*j*eta)
            eta1 -= k.be[j-1]*Math.cos(2*j*xi)*Math.sinh(2*j*eta)
        }
        var chi = Math.asin(Math.sin(xi1)/Math.cosh(eta1))
        var phi = chi
        for (var m = 1; m <= 4; ++m) phi += k.de[m-1]*Math.sin(2*m*chi)
        var lam = Math.atan2(Math.sinh(eta1), Math.cos(xi1))
        var lon0 = (zone - 1)*6 - 180 + 3
        return { lat: phi*r2d, lon: lon0 + lam*r2d }
    }

    // ---------------- MGRS ----------------

    function mgrsFormat(lat, lon, digits) {
        if (digits === undefined) digits = 5
        if (lat < -80 || lat > 84) return ""
        var u = utmForward(lat, lon)
        var band = mgrsBands.charAt(Math.min(Math.floor((lat + 80)/8), 19))
        var col = mgrsCols[(u.zone - 1) % 3].charAt(Math.floor(u.e/100000) - 1)
        var rowIdx = Math.floor(u.n/100000) % 20
        if (u.zone % 2 === 0) rowIdx = (rowIdx + 5) % 20
        var row = mgrsRows.charAt(rowIdx)
        var div = Math.pow(10, 5 - digits)
        var eS = String(Math.floor((u.e % 100000)/div)), nS = String(Math.floor((u.n % 100000)/div))
        while (eS.length < digits) eS = "0" + eS
        while (nS.length < digits) nS = "0" + nS
        var zs = u.zone < 10 ? "0" + u.zone : String(u.zone)
        return zs + band + " " + col + row + (digits > 0 ? " " + eS + " " + nS : "")
    }

    function mgrsParse(text) {
        var m = /^\s*(\d{1,2})\s*([C-HJ-NP-X])\s*([A-HJ-NP-Z])\s*([A-HJ-NP-V])\s*([0-9\s]*)$/i.exec(text)
        if (!m) return { ok: false, error: "не MGRS" }
        var zone = parseInt(m[1], 10), band = m[2].toUpperCase(), col = m[3].toUpperCase(), row = m[4].toUpperCase()
        var digs = m[5].replace(/\s+/g, "")
        if (zone < 1 || zone > 60) return { ok: false, error: "зона 1–60" }
        if (digs.length % 2 !== 0 || digs.length > 10) return { ok: false, error: "цифр має бути парна кількість (0–10)" }
        var ci = mgrsCols[(zone - 1) % 3].indexOf(col)
        if (ci < 0) return { ok: false, error: "літера стовпця не відповідає зоні" }
        var ri = mgrsRows.indexOf(row)
        if (zone % 2 === 0) ri = (ri - 5 + 20) % 20
        var half = digs.length / 2, scale = Math.pow(10, 5 - half)
        var eIn = half > 0 ? parseInt(digs.substr(0, half), 10) * scale : 0
        var nIn = half > 0 ? parseInt(digs.substr(half), 10) * scale : 0
        var cell = half > 0 ? scale / 2 : 50000          // центр комірки
        var E = (ci + 1)*100000 + eIn + cell
        var n100 = ri*100000 + nIn + cell
        var hemi = band >= "N" ? "N" : "S"
        var latBand = (mgrsBands.indexOf(band))*8 - 80
        var lon0 = (zone - 1)*6 - 180 + 3
        var nBand = Math.floor(utmForward(latBand, lon0, zone).n/100000)*100000
        var n2M = 0
        while (n2M + n100 < nBand) n2M += 2000000
        var N = n2M + n100
        var ll = utmInverse(zone, hemi, E, N)
        if (!isFinite(ll.lat) || !isFinite(ll.lon)) return { ok: false, error: "не вдалося перетворити" }
        return { ok: true, lat: ll.lat, lon: ll.lon, kind: "MGRS" }
    }

    // MGRS або "lat, lon" / "lat lon" / "lat; lon" (десяткові градуси)
    function parseCoordinate(text) {
        var s = (text || "").trim()
        if (s === "") return { ok: false, error: "" }
        var r = mgrsParse(s)
        if (r.ok) return r
        var parts
        if (s.indexOf(";") >= 0) parts = s.split(";").map(function (x) { return parseFloat(x.trim().replace(",", ".")) })
        else parts = s.split(/[\s,]+/).map(function (x) { return parseFloat(x) })
        if (parts.length === 2 && isFinite(parts[0]) && isFinite(parts[1])
            && Math.abs(parts[0]) <= 90 && Math.abs(parts[1]) <= 180)
            return { ok: true, lat: parts[0], lon: parts[1], kind: "lat/lon" }
        return { ok: false, error: r.error === "не MGRS" ? "не MGRS і не «lat, lon»" : r.error }
    }

    // ---------------- Геозони (geofence) ----------------
    // zones = { incl: [z], excl: [z] },  z = {type:"poly", pts:[{lat,lon}]} | {type:"circle", lat, lon, r}
    function pointInPoly(lat, lon, pts) {
        var inside = false
        for (var i = 0, j = pts.length - 1; i < pts.length; j = i++) {
            var yi = pts[i].lat, xi = pts[i].lon, yj = pts[j].lat, xj = pts[j].lon
            if (((yi > lat) !== (yj > lat)) && (lon < (xj - xi)*(lat - yi)/(yj - yi) + xi)) inside = !inside
        }
        return inside
    }
    function inZone(lat, lon, z) {
        if (z.type === "circle") return distM(z.lat, z.lon, lat, lon) <= z.r
        return pointInPoly(lat, lon, z.pts)
    }
    function pointAllowed(lat, lon, zones) {
        var ok = zones.incl.length === 0, i
        for (i = 0; i < zones.incl.length && !ok; ++i) if (inZone(lat, lon, zones.incl[i])) ok = true
        if (!ok) return false
        for (i = 0; i < zones.excl.length; ++i) if (inZone(lat, lon, zones.excl[i])) return false
        return true
    }
    function orbitFits(clat, clon, r, zones, margin) {
        var rr = r + margin, i
        for (i = 0; i < 48; ++i) {
            var a = 2*Math.PI*i/48
            var p = offsetLL(clat, clon, rr*Math.cos(a), rr*Math.sin(a))
            if (!pointAllowed(p.lat, p.lon, zones)) return false
        }
        for (i = 0; i < zones.excl.length; ++i) {           // виключені зони повністю всередині кола
            var z = zones.excl[i]
            if (z.type === "circle") { if (distM(clat, clon, z.lat, z.lon) <= rr + z.r) return false }
            else for (var k = 0; k < z.pts.length; ++k)
                if (distM(clat, clon, z.pts[k].lat, z.pts[k].lon) <= rr) return false
        }
        return true
    }
    function zonesBBox(zones) {
        var b = null
        function add(lat, lon) {
            if (!b) b = { minLat: lat, maxLat: lat, minLon: lon, maxLon: lon }
            else { b.minLat = Math.min(b.minLat, lat); b.maxLat = Math.max(b.maxLat, lat)
                   b.minLon = Math.min(b.minLon, lon); b.maxLon = Math.max(b.maxLon, lon) }
        }
        for (var i = 0; i < zones.incl.length; ++i) {
            var z = zones.incl[i]
            if (z.type === "circle") {
                var s = offsetLL(z.lat, z.lon, -z.r, -z.r), n = offsetLL(z.lat, z.lon, z.r, z.r)
                add(s.lat, s.lon); add(n.lat, n.lon)
            } else for (var k = 0; k < z.pts.length; ++k) add(z.pts[k].lat, z.pts[k].lon)
        }
        return b
    }

    // ---------------- Підбір орбіти (тільки пропозиція, нічого не відправляє) ----------------
    function turnRadiusFor(vMs, bankDeg) { return vMs*vMs/(9.80665*Math.tan(bankDeg*d2r)) }

    function evalCenter(clat, clon, aim, radii, p, zones, hmin) {
        var d = distM(clat, clon, aim.lat, aim.lon)
        for (var i = 0; i < radii.length; ++i) {
            var r = radii[i]
            if (d - r < hmin) continue
            var slant = Math.sqrt((d + r)*(d + r) + p.h*p.h)
            if (p.maxRange > 0 && slant > p.maxRange) continue
            if (!orbitFits(clat, clon, r, zones, p.fenceMargin)) continue
            return { ok: true, lat: clat, lon: clon, r: r, h: p.h, slant: slant, d: d,
                     depr: Math.atan2(p.h, Math.max(d - r, 1))*r2d }
        }
        return null
    }

    function suggestOrbit(p) {
        var zones = p.zones
        if (zones.incl.length === 0) return { ok: false, error: "У geofence немає зони дозволеного польоту (inclusion)" }
        var rt = turnRadiusFor(p.speedKmh/3.6, p.bankDeg)*1.1
        var rPref = Math.max(p.radiusPref, rt), radii = []
        for (var i = 0; i < 4; ++i) radii.push(rPref + (rt - rPref)*i/3)
        var hmin = p.h/Math.tan((p.tiltLimit - p.tiltMargin)*d2r)
        var bb = zonesBBox(zones)
        var sw = { lat: bb.minLat, lon: bb.minLon }
        var ext = deltaNE(bb.minLat, bb.minLon, bb.maxLat, bb.maxLon)
        var step = Math.max(50, Math.sqrt(ext.n*ext.e/6000))
        var tried = 0

        // Радіус із панелі — пріоритет; менший пробуємо, лише якщо з більшим нічого не вміщується.
        for (var ri = 0; ri < radii.length; ++ri) {
            var rl = [radii[ri]], best = null
            for (var n = 0; n <= ext.n; n += step) {
                for (var e = 0; e <= ext.e; e += step) {
                    var c = offsetLL(sw.lat, sw.lon, n, e)
                    if (!pointAllowed(c.lat, c.lon, zones)) continue
                    ++tried
                    var r = evalCenter(c.lat, c.lon, p.aim, rl, p, zones, hmin)
                    if (r && (!best || r.slant < best.slant)) best = r
                }
            }
            if (best) {                                       // уточнення навколо кращого
                var fs = step/5, b0 = best
                for (var n2 = -step; n2 <= step; n2 += fs) {
                    for (var e2 = -step; e2 <= step; e2 += fs) {
                        var c2 = offsetLL(b0.lat, b0.lon, n2, e2)
                        var r2 = evalCenter(c2.lat, c2.lon, p.aim, rl, p, zones, hmin)
                        if (r2 && r2.slant < best.slant) best = r2
                    }
                }
                best.tried = tried
                best.reduced = ri > 0
                return best
            }
        }
        return { ok: false, tried: tried,
                 error: "Немає допустимого варіанта: орбіта радіусом ≥ " + Math.round(rt) + " м не вміщується в зону з обмеженням нахилу підвісу"
                        + (p.maxRange > 0 ? " і дальністю " + p.maxRange + " м" : "") }
    }

    // ===================================================================
    //  ГЕОМЕТРІЯ (порт з C# RelayCoveragePlugin)
    // ===================================================================
    function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
    function cross(a, b) { return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]] }
    function scale(a, k) { return [a[0]*k, a[1]*k, a[2]*k] }
    function add(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]] }
    function norm(a) { var l = Math.sqrt(dot(a, a)); return l < 1e-12 ? a : scale(a, 1/l) }
    function norm180(a) { return ((a % 360) + 540) % 360 - 180 }

    function bodyDir(panDeg, tiltDeg) {
        var p = panDeg*d2r, t = tiltDeg*d2r
        return [Math.cos(t)*Math.cos(p), Math.cos(t)*Math.sin(p), -Math.sin(t)]
    }
    function bodyToNed(v, rollDeg, pitchDeg, yawDeg) {
        var r = rollDeg*d2r, p = pitchDeg*d2r, y = yawDeg*d2r
        var cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y)
        return [
            cp*cy*v[0] + (sr*sp*cy - cr*sy)*v[1] + (cr*sp*cy + sr*sy)*v[2],
            cp*sy*v[0] + (sr*sp*sy + cr*cy)*v[1] + (cr*sp*sy - sr*cy)*v[2],
            -sp*v[0]   +  sr*cp*v[1]             +  cr*cp*v[2]]
    }
    function nedToBody(n, rollDeg, pitchDeg, yawDeg) {
        var r = rollDeg*d2r, p = pitchDeg*d2r, y = yawDeg*d2r
        var cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y)
        return [
            cp*cy*n[0] + cp*sy*n[1] - sp*n[2],
            (sr*sp*cy - cr*sy)*n[0] + (sr*sp*sy + cr*cy)*n[1] + sr*cp*n[2],
            (cr*sp*cy + sr*sy)*n[0] + (cr*sp*sy - sr*cy)*n[1] + cr*cp*n[2]]
    }
    function coneRay(axis, halfDeg, phiDeg) {
        var up = Math.abs(axis[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]
        var u = norm(cross(axis, up)), w = norm(cross(axis, u))
        var h = halfDeg*d2r, f = phiDeg*d2r
        return norm(add(scale(axis, Math.cos(h)),
                        scale(add(scale(u, Math.cos(f)), scale(w, Math.sin(f))), Math.sin(h))))
    }
    function intersect(d, hAgl, maxRange) {
        if (d[2] > 1e-3) {
            var t = hAgl / d[2]
            var hor = Math.sqrt(d[0]*d[0] + d[1]*d[1]) * t
            if (hor <= maxRange) return { n: d[0]*t, e: d[1]*t, hit: true }
        }
        var hn = Math.sqrt(d[0]*d[0] + d[1]*d[1])
        if (hn < 1e-9) return { n: 0, e: 0, hit: false }
        return { n: d[0]/hn*maxRange, e: d[1]/hn*maxRange, hit: false }
    }
    function offsetCoord(lat, lon, n, e) {
        return QtPositioning.coordinate(lat + n/earthR*r2d, lon + e/(earthR*Math.cos(lat*d2r))*r2d)
    }

    // ===================================================================
    //  ВІСЬ ПІДВІСУ: телеметрія підвісу -> ідеальне наведення на ціль -> фіксований
    // ===================================================================
    function currentAxis(v, lat, lon, hAgl) {
        var roll = v.roll.rawValue, pitch = v.pitch.rawValue, yaw = v.heading.rawValue
        try {
            var g = v.gimbalController ? v.gimbalController.activeGimbal : null
            if (g) {
                var ap = g.absolutePitch.rawValue, ay = g.absoluteYaw.rawValue
                if (isFinite(ap) && isFinite(ay) && !(ap === 0 && ay === 0)) {
                    var p = ap*d2r, y = ay*d2r
                    return { ned: [Math.cos(p)*Math.cos(y), Math.cos(p)*Math.sin(y), -Math.sin(p)], src: qsTr("підвіс") }
                }
            }
        } catch (e) {}
        if (aimCoord) {
            var d = deltaNE(lat, lon, aimCoord.latitude, aimCoord.longitude)
            return { ned: norm([d.n, d.e, hAgl]), src: qsTr("ціль (ідеально)") }
        }
        return { ned: bodyToNed(bodyDir(fixedPanDeg, fixedTiltDeg), roll, pitch, yaw), src: qsTr("фікс.") }
    }

    // ===================================================================
    //  ПОКРИТТЯ
    // ===================================================================
    function clearCoverage() {
        if (_spot) _spot.path = []
        if (_contour) _contour.path = []
        if (_axisLine) _axisLine.path = []
    }

    function updateCoverage() {
        var v = vehicle
        if (!v || !v.coordinate || !v.coordinate.isValid) { _hasState = false; clearCoverage(); return }
        var lat = v.coordinate.latitude, lon = v.coordinate.longitude
        if (lat === 0 && lon === 0) { clearCoverage(); return }

        var hAgl = Math.max(v.altitudeRelative.rawValue - groundOffsetM, 1)
        var roll = v.roll.rawValue, pitch = v.pitch.rawValue, yaw = v.heading.rawValue
        var ax = currentAxis(v, lat, lon, hAgl)

        var now = Date.now()
        var dt = _hasState ? Math.min((now - _lastT) / 1000, 2) : 0
        _lastT = now
        var jump = _hasState && dot(_sAxis, ax.ned) < Math.cos(jumpResetDeg*d2r)
        if (!_hasState || jump) { _sAxis = ax.ned; _sH = hAgl; _hasState = true }
        else {
            var a = smoothTauSec <= 1e-6 ? 1 : 1 - Math.exp(-dt / smoothTauSec)
            _sAxis = norm(add(scale(_sAxis, 1 - a), scale(ax.ned, a)))
            _sH = _sH + a * (hAgl - _sH)
        }
        var axisNed = _sAxis
        hAgl = Math.max(_sH, 1)

        var ai = intersect(axisNed, hAgl, maxRangeM)
        var center = offsetCoord(lat, lon, ai.n, ai.e)

        var pts = [], cut = 0
        for (var i = 0; i < conePoints; ++i) {
            var ray = coneRay(axisNed, beamWidthDeg/2, 360*i/conePoints)
            var r = intersect(ray, hAgl, maxRangeM)
            if (!r.hit) cut++
            pts.push(offsetCoord(lat, lon, r.n, r.e))
        }
        var ring = pts.slice(); ring.push(pts[0])

        // ліміти підвісу (зв'язана СК)
        var b = nedToBody(axisNed, roll, pitch, yaw)
        var pan = Math.atan2(b[1], b[0]) * r2d
        var tilt = Math.asin(Math.max(-1, Math.min(1, -b[2]))) * r2d
        var margin = Math.min(tilt - tiltMinDeg, tiltMaxDeg - tilt)
        var col = margin <= 0.5 ? "#ff3030" : (margin <= warnMarginDeg ? "#ffd400" : "#32cd32")
        var limTxt = margin <= 0.5 ? qsTr("ЛІМІТ нахилу підвісу")
                   : margin <= warnMarginDeg ? qsTr("Біля ліміту, запас %1°").arg(margin.toFixed(0))
                   : qsTr("Запас до ліміту %1°").arg(margin.toFixed(0))

        if (_spot) { _spot.color = Qt.rgba(Qt.color(col).r, Qt.color(col).g, Qt.color(col).b, 0.12); _spot.path = pts }
        if (_contour) { _contour.line.color = col; _contour.path = ring }
        if (_axisLine) { _axisLine.line.color = col; _axisLine.path = [v.coordinate, center] }

        var slant = ai.hit ? Math.sqrt(ai.n*ai.n + ai.e*ai.e + hAgl*hAgl) : NaN
        var txt = qsTr("Вісь: %1\nPan %2°  Tilt %3°\nВисота %4 м, дальність %5\n%6")
            .arg(ax.src).arg(pan.toFixed(0)).arg(tilt.toFixed(0)).arg(Math.round(hAgl))
            .arg(isNaN(slant) ? qsTr("поза горизонтом") : Math.round(slant) + qsTr(" м")).arg(limTxt)
        if (cut > 0) txt += qsTr("\nПляма обрізана горизонтом")
        if (aimCoord) {
            var dd = deltaNE(lat, lon, aimCoord.latitude, aimCoord.longitude)
            var rel = norm180(Math.atan2(dd.e, dd.n) * r2d - yaw)
            txt += qsTr("\nНаведення: %1° від носа, %2 м").arg(rel.toFixed(0)).arg(Math.round(Math.sqrt(dd.n*dd.n + dd.e*dd.e)))
        }
        telemetryLabel.text = txt
    }


    // ===================================================================
    //  ДОПОМІЖНІ: формат точок, буфер обміну
    // ===================================================================
    function fmtShort(c) { return c ? mgrsFormat(c.latitude, c.longitude, 5) : qsTr("—") }
    function fmtPoint(c) {
        if (!c) return qsTr("—")
        return c.latitude.toFixed(5) + ", " + c.longitude.toFixed(5) + "\n" + mgrsFormat(c.latitude, c.longitude, 5)
    }
    function clipboardText() { clipHelper.text = ""; clipHelper.paste(); return clipHelper.text }
    function copyText(s) { clipHelper.text = s; clipHelper.selectAll(); clipHelper.copy() }

    // ===================================================================
    //  ЖУРНАЛ ДІЙ
    // ===================================================================
    function logRead() {
        try { var a = JSON.parse(store.logJson); return Array.isArray(a) ? a : [] } catch (e) { return [] }
    }
    function logEvent(kind, text) {
        var a = logRead()
        a.push({ t: Qt.formatDateTime(new Date(), "yyyy-MM-dd HH:mm:ss"), k: kind, x: text })
        if (a.length > logMaxEntries) a = a.slice(a.length - logMaxEntries)
        store.logJson = JSON.stringify(a)
    }
    function logTail(n) {
        var a = logRead(), s = []
        for (var i = Math.max(0, a.length - n); i < a.length; ++i)
            s.push(a[i].t.substr(11) + " " + a[i].k + " " + a[i].x)
        return s.length ? s.join("\n") : qsTr("(порожньо)")
    }
    function logCount() { return logRead().length }
    function logCsv() {
        var a = logRead(), s = ["time;event;details"]
        for (var i = 0; i < a.length; ++i)
            s.push(a[i].t + ";" + a[i].k + ";\"" + String(a[i].x).replace(/"/g, "\"\"") + "\"")
        return s.join("\n")
    }

    // ===================================================================
    //  GEOFENCE із QGC (зони дозволеного/забороненого польоту)
    // ===================================================================
    function readZones() {
        try {
            var pmc = mapControl ? mapControl.planMasterController : null
            var gfc = pmc ? pmc.geoFenceController : null
            if (!gfc) return null
            var res = { incl: [], excl: [] }, i, k
            var polys = gfc.polygons
            for (i = 0; i < polys.count; ++i) {
                var pg = polys.get(i), path = pg.path, pts = []
                for (k = 0; k < path.length; ++k) pts.push({ lat: path[k].latitude, lon: path[k].longitude })
                if (typeof pg.inclusion !== "boolean") return null      // невідомий формат — не вгадуємо
                if (pts.length >= 3) (pg.inclusion ? res.incl : res.excl).push({ type: "poly", pts: pts })
            }
            var circles = gfc.circles
            for (i = 0; i < circles.count; ++i) {
                var cc = circles.get(i), ctr = cc.center, rad = cc.radius.rawValue
                if (typeof cc.inclusion !== "boolean") return null
                if (ctr && ctr.isValid && rad > 0)
                    (cc.inclusion ? res.incl : res.excl).push({ type: "circle", lat: ctr.latitude, lon: ctr.longitude, r: rad })
            }
            return res
        } catch (e) { return null }
    }
    function fenceInfo() {
        if (!zones) return qsTr("Geofence: недоступний (QGC не віддав зони)")
        if (zones.incl.length + zones.excl.length === 0) return qsTr("Geofence: зон немає")
        return qsTr("Geofence: дозволених %1, заборонених %2").arg(zones.incl.length).arg(zones.excl.length)
    }

    // ===================================================================
    //  ОРБІТА: план/прев'ю
    // ===================================================================
    function turnRadius(vMs) { return vMs*vMs / (9.80665 * Math.tan(planBankDeg*d2r)) }

    function planOrbit() {
        if (!loiterCoord) return null
        var speedKmh = parseFloat(fSpeed.value), alt = parseFloat(fAlt.value), rad = parseFloat(fRadius.value)
        if (!(speedKmh > 0) || !(alt > 0) || !(rad > 0)) return null
        var rt = turnRadius(speedKmh / 3.6), r = rad, notes = []
        if (r < rt * orbitMinFactor) {
            r = rt * orbitMinFactor
            notes.push(qsTr("Радіус збільшено до %1 м (мін. для %2 км/год, крен %3°)").arg(Math.round(r)).arg(Math.round(speedKmh)).arg(planBankDeg))
        }
        var hT = Math.max(alt - groundOffsetM, 1), limit = -tiltMinDeg
        if (aimCoord) {                      // нахил підвісу до ТОЧКИ НАВЕДЕННЯ
            var dist = distM(loiterCoord.latitude, loiterCoord.longitude, aimCoord.latitude, aimCoord.longitude)
            var depr = Math.atan2(hT, Math.max(dist - r, 1)) * r2d
            if (depr > limit) notes.push(qsTr("⚠ Нахил підвісу до цілі до %1° > ліміт %2°").arg(Math.round(depr)).arg(limit))
            else notes.push(qsTr("Макс. нахил до цілі %1° (ліміт %2°)").arg(Math.round(depr)).arg(limit))
        } else {
            notes.push(qsTr("Точка наведення не задана"))
        }
        if (zones && zones.incl.length + zones.excl.length > 0
                && !orbitFits(loiterCoord.latitude, loiterCoord.longitude, r, zones, 0))
            notes.push(qsTr("⚠ Орбіта виходить за межі geofence"))
        var path = []
        for (var i = 0; i <= 72; ++i) {
            var a = 2*Math.PI*i/72
            path.push(offsetCoord(loiterCoord.latitude, loiterCoord.longitude, r*Math.cos(a), r*Math.sin(a)))
        }
        return { r: r, alt: alt, speedKmh: speedKmh, path: path, notes: notes, cw: orbitClockwise,
                 center: QtPositioning.coordinate(loiterCoord.latitude, loiterCoord.longitude) }
    }
    function updatePlan() {
        var p = planOrbit()
        if (_orbitLine) _orbitLine.path = p ? p.path : []
        planLabel.text = p ? p.notes.join("\n") : ""
    }

    // ===================================================================
    //  КОМАНДИ БОРТУ (числові MAVLink ID, щоб не залежати від enum-експорту в QML)
    // ===================================================================
    readonly property int cmdChangeSpeed: 178
    readonly property int cmdReposition:  192
    readonly property int cmdRoiLocation: 195
    readonly property int cmdRoiNone:     197

    function sendRoi(c) {
        vehicle.sendMavCommand(1, cmdRoiLocation, true, 0, 0, 0, 0, c.latitude, c.longitude, groundOffsetM)
    }
    function setAim(c, src) {                       // наведення — без підтвердження (не змінює орбіту)
        aimCoord = c
        if (autoRoi && vehicle) sendRoi(c)
        logEvent("AIM", (src ? src + ": " : "") + fmtShort(c) + " [" + c.latitude.toFixed(6) + ", " + c.longitude.toFixed(6) + "]")
        statusText = qsTr("Наведення: %1").arg(fmtShort(c))
    }
    function clearAim() {
        aimCoord = null
        if (vehicle) vehicle.sendMavCommand(1, cmdRoiNone, true, 0, 0, 0, 0, 0, 0, 0)
        logEvent("AIM_CLEAR", "")
        statusText = qsTr("Наведення скинуто")
    }
    function setLoiter(c, src) {                    // баражування: якщо орбіта вже летить — тільки з підтвердженням
        if (pendingAction) cancelPending()
        var prev = loiterCoord
        loiterCoord = c
        logEvent("LOITER_POINT", (src ? src + ": " : "") + fmtShort(c) + " [" + c.latitude.toFixed(6) + ", " + c.longitude.toFixed(6) + "]")
        statusText = qsTr("Баражування: %1").arg(fmtShort(c))
        if (orbitActive) requestOrbit(qsTr("Перенесення центру орбіти"), prev)
    }
    function clearLoiter() {
        if (pendingAction) cancelPending()
        loiterCoord = null
        logEvent("LOITER_CLEAR", "")
        statusText = qsTr("Точку баражування скинуто (орбіта на борту не змінена)")
    }

    // ---- підтвердження ----
    function requestOrbit(reason, revertCoord) {
        if (!vehicle) { statusText = qsTr("Немає борту"); return }
        var p = planOrbit()
        if (!p) { statusText = qsTr("Задайте точку баражування і параметри (радіус, висота, швидкість)"); return }
        var L = []
        L.push(qsTr("Центр: %1").arg(fmtShort(p.center)))
        L.push(qsTr("R %1 м · H(AGL) %2 м · %3 км/год · %4").arg(Math.round(p.r)).arg(Math.round(p.alt))
               .arg(Math.round(p.speedKmh)).arg(p.cw ? qsTr("за годинниковою") : qsTr("проти годинникової")))
        if (activeOrbit) {
            var a = activeOrbit
            L.push(qsTr("Було: R %1 м · H %2 м · %3 км/год; центр зсунуто на %4 м")
                   .arg(Math.round(a.r)).arg(Math.round(a.alt)).arg(Math.round(a.speedKmh))
                   .arg(Math.round(distM(a.lat, a.lon, p.center.latitude, p.center.longitude))))
        }
        var warns = []
        for (var i = 0; i < p.notes.length; ++i) if (p.notes[i].indexOf("⚠") === 0) warns.push(p.notes[i])
        pendingAction = { kind: "orbit", title: qsTr("Підтвердити зміну орбіти"), reason: reason,
                          plan: p, hasRevert: revertCoord !== undefined, revert: revertCoord === undefined ? null : revertCoord,
                          lines: L, warns: warns }
        logEvent("ORBIT_REQUEST", reason + " | " + L.join(" | "))
    }
    function requestClearLog() {
        pendingAction = { kind: "clearLog", title: qsTr("Очистити журнал?"), reason: "", warns: [],
                          lines: [qsTr("Записів: %1. Дію не можна скасувати.").arg(logCount())], hasRevert: false, revert: null }
    }
    function confirmPending() {
        var a = pendingAction
        pendingAction = null
        if (!a) return
        if (a.kind === "orbit") executeOrbit(a.plan, a.reason)
        else if (a.kind === "clearLog") { store.logJson = "[]"; logEvent("LOG_CLEARED", "") }
    }
    function cancelPending() {
        var a = pendingAction
        pendingAction = null
        if (!a) return
        if (a.kind === "orbit") {
            if (a.hasRevert) loiterCoord = a.revert
            logEvent("ORBIT_CANCEL", a.reason)
            statusText = qsTr("Зміну орбіти скасовано")
        }
    }

    function executeOrbit(p, reason) {
        if (!vehicle) { statusText = qsTr("Немає борту"); return }
        vehicle.sendMavCommand(1, cmdChangeSpeed, true, 0, p.speedKmh / 3.6, -1, 0, 0, 0, 0)
        var alt = repositionAltIsAmsl ? (vehicle.homePosition.altitude + p.alt) : p.alt
        var signedR = p.cw ? p.r : -p.r                       // + за годинниковою, - проти
        // p1 швидкість(-1), p2=1 перейти в GUIDED, p3 радіус, p4 yaw(NaN), p5 lat, p6 lon, p7 alt
        vehicle.sendMavCommand(1, cmdReposition, true, -1, 1, signedR, NaN,
                               p.center.latitude, p.center.longitude, alt)
        activeOrbit = { lat: p.center.latitude, lon: p.center.longitude, r: p.r, alt: p.alt, speedKmh: p.speedKmh, cw: p.cw }
        orbitActive = true
        var vc = vehicle.coordinate
        logEvent("ORBIT_APPLY", reason + " | " + fmtShort(p.center) + " R=" + Math.round(p.r) + " H=" + Math.round(p.alt)
                 + " V=" + Math.round(p.speedKmh) + " " + (p.cw ? "CW" : "CCW")
                 + " | борт " + vc.latitude.toFixed(5) + "," + vc.longitude.toFixed(5))
        statusText = qsTr("ОРБІТА: R=%1 м, %2 км/год, H=%3 м").arg(Math.round(p.r)).arg(Math.round(p.speedKmh)).arg(Math.round(p.alt))
    }
    function stopOrbit() {
        if (!vehicle) return
        vehicle.flightMode = "Loiter"           // кружляти навколо поточної точки (без підтвердження)
        orbitActive = false
        activeOrbit = null
        logEvent("ORBIT_STOP", "Loiter")
        statusText = qsTr("Зупинено (режим Loiter)")
    }

    // ---- введення координат ----
    function applyCoordInput(target) {
        var r = parseCoordinate(coordInput.text)
        if (!r.ok) { statusText = qsTr("Координати: ") + r.error; return }
        var c = QtPositioning.coordinate(r.lat, r.lon)
        if (target === "aim") setAim(c, qsTr("введено (%1)").arg(r.kind))
        else setLoiter(c, qsTr("введено (%1)").arg(r.kind))
    }

    // ---- «Підібрати» в межах geofence ----
    function runSuggest() {
        suggestion = null
        if (!aimCoord) { finderText = qsTr("Спочатку задайте точку наведення."); return }
        zones = readZones()
        if (!zones) { finderText = qsTr("Geofence недоступний: QGC не віддав зони. Задайте geofence у Plan і завантажте на борт."); return }
        var speed = parseFloat(fSpeed.value), alt = parseFloat(fAlt.value), rad = parseFloat(fRadius.value)
        if (!(speed > 0) || !(alt > 0) || !(rad > 0)) { finderText = qsTr("Заповніть радіус, висоту і швидкість."); return }
        var p = { aim: { lat: aimCoord.latitude, lon: aimCoord.longitude }, zones: zones,
                  h: Math.max(alt - groundOffsetM, 1), speedKmh: speed, radiusPref: rad, bankDeg: planBankDeg,
                  tiltLimit: -tiltMinDeg, tiltMargin: warnMarginDeg,
                  fenceMargin: parseFloat(fMargin.value) || 0, maxRange: parseFloat(fRange.value) || 0 }
        var r = suggestOrbit(p)
        if (!r.ok) { finderText = r.error; logEvent("SUGGEST_FAIL", r.error); return }
        suggestion = r
        finderText = qsTr("Центр: %1\nR %2 м%3 · макс. нахил до цілі %4° · макс. дальність %5 м")
            .arg(mgrsFormat(r.lat, r.lon, 5)).arg(Math.round(r.r))
            .arg(r.reduced ? qsTr(" (зменшено відносно заданого)") : "").arg(Math.round(r.depr)).arg(Math.round(r.slant))
        logEvent("SUGGEST", mgrsFormat(r.lat, r.lon, 5) + " R=" + Math.round(r.r) + " H=" + Math.round(alt))
    }
    function acceptSuggestion() {
        if (!suggestion) return
        var s = suggestion
        fRadius.value = String(Math.round(s.r))
        logEvent("SUGGEST_ACCEPT", mgrsFormat(s.lat, s.lon, 5) + " R=" + Math.round(s.r) + " (відправка — лише через «Застосувати»)")
        setLoiter(QtPositioning.coordinate(s.lat, s.lon), qsTr("підібрано"))
        suggestion = null
        finderText = qsTr("Прийнято: точку баражування й радіус виставлено. Натисніть «Застосувати» і підтвердіть.")
    }

    // ---- перетягування маркерів ----
    function markerHit(pt) {
        var best = "", bd = dragHitPx
        var cands = [["aim", aimCoord], ["loiter", loiterCoord]]
        for (var i = 0; i < cands.length; ++i) {
            if (!cands[i][1]) continue
            var sp = mapControl.fromCoordinate(cands[i][1], false)
            var d = Math.sqrt((sp.x - pt.x)*(sp.x - pt.x) + (sp.y - pt.y)*(sp.y - pt.y))
            if (d <= bd) { bd = d; best = cands[i][0] }
        }
        return best
    }
    function finishDrag(canceled) {
        if (_drag === "") return
        var which = _drag
        _drag = ""
        if (which === "aim") {
            if (canceled) { aimCoord = _dragStart; return }
            if (autoRoi && vehicle) sendRoi(aimCoord)
            logEvent("AIM_DRAG", fmtShort(aimCoord) + " [" + aimCoord.latitude.toFixed(6) + ", " + aimCoord.longitude.toFixed(6) + "]")
            statusText = qsTr("Наведення: %1").arg(fmtShort(aimCoord))
        } else {
            if (canceled) { loiterCoord = _dragStart; return }
            logEvent("LOITER_DRAG", fmtShort(loiterCoord))
            statusText = qsTr("Баражування: %1").arg(fmtShort(loiterCoord))
            if (orbitActive) requestOrbit(qsTr("Перенесення центру орбіти"), _dragStart)
        }
    }

    // ===================================================================
    //  MAP ITEMS
    // ===================================================================
    Component { id: polyC;    MapPolygon  { color: "#2000ff00"; border.width: 0 } }
    Component { id: lineC;    MapPolyline { line.width: 3; line.color: "#32cd32" } }
    Component { id: aimC
        MapQuickItem {
            anchorPoint.x: 14; anchorPoint.y: 14; z: 20
            sourceItem: Rectangle { width: 28; height: 28; radius: 14; color: "#22ff3030"; border.width: 3; border.color: "#ff3030"
                Rectangle { width: 2; height: 28; color: "#ff3030"; anchors.centerIn: parent }
                Rectangle { width: 28; height: 2; color: "#ff3030"; anchors.centerIn: parent } }
        }
    }
    Component { id: loiterC
        MapQuickItem {
            anchorPoint.x: 12; anchorPoint.y: 12; z: 20
            sourceItem: Rectangle { width: 24; height: 24; radius: 12; color: "#401e90ff"; border.width: 3; border.color: "#1e90ff" }
        }
    }

    function ensureItems() {
        if (_spot || !mapControl || !mapControl.addMapItem) return
        _spot = polyC.createObject(mapControl);     mapControl.addMapItem(_spot)
        _contour = lineC.createObject(mapControl);  mapControl.addMapItem(_contour)
        _axisLine = lineC.createObject(mapControl); _axisLine.line.width = 2; mapControl.addMapItem(_axisLine)
        _orbitLine = lineC.createObject(mapControl); _orbitLine.line.color = "#1e90ff"; mapControl.addMapItem(_orbitLine)
        _aimMark = aimC.createObject(mapControl);   _aimMark.visible = false;    mapControl.addMapItem(_aimMark)
        _loiterMark = loiterC.createObject(mapControl); _loiterMark.visible = false; mapControl.addMapItem(_loiterMark)
    }
    onMapControlChanged: ensureItems()
    onAimCoordChanged:    { if (_aimMark) { _aimMark.visible = aimCoord !== null; if (aimCoord) _aimMark.coordinate = aimCoord } }
    onLoiterCoordChanged: { if (_loiterMark) { _loiterMark.visible = loiterCoord !== null; if (loiterCoord) _loiterMark.coordinate = loiterCoord } }

    Component.onDestruction: {
        var items = [_spot, _contour, _axisLine, _orbitLine, _aimMark, _loiterMark]
        for (var i = 0; i < items.length; ++i)
            if (items[i]) { if (mapControl && mapControl.removeMapItem) mapControl.removeMapItem(items[i]); items[i].destroy() }
    }

    Timer {
        id: tick
        interval: 200; running: true; repeat: true
        property int n: 0
        onTriggered: {
            ensureItems()
            if (n % 10 === 0 && !pendingAction) zones = readZones()        // ~кожні 2 с
            ++n
            if (_aimMark) { _aimMark.visible = aimCoord !== null; if (aimCoord) _aimMark.coordinate = aimCoord }
            if (_loiterMark) { _loiterMark.visible = loiterCoord !== null; if (loiterCoord) _loiterMark.coordinate = loiterCoord }
            updateCoverage()
            updatePlan()
        }
    }

    // ===================================================================
    //  МИШКА: Cmd+Shift+ЛКМ, Cmd+Option+ЛКМ, ПКМ, перетягування маркерів
    // ===================================================================
    function coordAt(mouse) {
        var pt = _root.mapToItem(mapControl, mouse.x, mouse.y)
        return mapControl.toCoordinate(Qt.point(pt.x, pt.y), false)
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        propagateComposedEvents: true
        onPressed: (mouse) => {
            if (!mapControl) { mouse.accepted = false; return }
            var m = mouse.modifiers
            var cmd = (m & Qt.ControlModifier) !== 0      // macOS: Cmd
            var shift = (m & Qt.ShiftModifier) !== 0
            var alt = (m & Qt.AltModifier) !== 0
            if (mouse.button === Qt.LeftButton && cmd && shift && !alt) {
                setAim(coordAt(mouse), qsTr("клік")); mouse.accepted = true
            } else if (mouse.button === Qt.LeftButton && cmd && alt && !shift) {
                setLoiter(coordAt(mouse), qsTr("клік")); mouse.accepted = true
            } else if (mouse.button === Qt.RightButton && !cmd) {
                _ctxCoord = coordAt(mouse)
                ctxMenu.popup()
                mouse.accepted = true
            } else if (mouse.button === Qt.LeftButton && !cmd && !shift && !alt) {
                var pt = _root.mapToItem(mapControl, mouse.x, mouse.y)
                var h = markerHit(pt)
                if (h !== "") {
                    _drag = h
                    var cur = (h === "aim") ? aimCoord : loiterCoord
                    _dragStart = cur
                    var sp = mapControl.fromCoordinate(cur, false)
                    _dragOff = Qt.point(sp.x - pt.x, sp.y - pt.y)
                    mouse.accepted = true
                } else {
                    mouse.accepted = false
                }
            } else {
                mouse.accepted = false                    // решту віддаємо QGC
            }
        }
        onPositionChanged: (mouse) => {
            if (_drag === "") return
            var pt = _root.mapToItem(mapControl, mouse.x, mouse.y)
            var c = mapControl.toCoordinate(Qt.point(pt.x + _dragOff.x, pt.y + _dragOff.y), false)
            if (_drag === "aim") aimCoord = c; else loiterCoord = c
        }
        onReleased: finishDrag(false)
        onCanceled: finishDrag(true)
    }

    Menu {
        id: ctxMenu
        MenuItem { text: qsTr("Точка наведення тут");   onTriggered: setAim(_ctxCoord, qsTr("меню")) }
        MenuItem { text: qsTr("Точка баражування тут"); onTriggered: setLoiter(_ctxCoord, qsTr("меню")) }
        MenuItem { text: qsTr("Обидві точки тут");      onTriggered: { setAim(_ctxCoord, qsTr("меню")); setLoiter(_ctxCoord, qsTr("меню")) } }
        MenuSeparator {}
        MenuItem { text: qsTr("Орбіта навколо точки баражування…"); onTriggered: requestOrbit(qsTr("Контекстне меню")) }
        MenuItem { text: qsTr("Зупинити баражування");  onTriggered: stopOrbit() }
        MenuSeparator {}
        MenuItem { text: qsTr("Копіювати MGRS цієї точки")
                   onTriggered: { var s = mgrsFormat(_ctxCoord.latitude, _ctxCoord.longitude, 5); copyText(s); statusText = qsTr("Скопійовано: ") + s } }
        MenuItem { text: qsTr("Скинути наведення");     onTriggered: clearAim() }
        MenuItem { text: qsTr("Скинути баражування");   onTriggered: clearLoiter() }
    }

    Shortcut { sequences: ["Return", "Enter"]; enabled: pendingAction !== null; onActivated: confirmPending() }
    Shortcut { sequence: "Escape";             enabled: pendingAction !== null; onActivated: cancelPending() }

    // ===================================================================
    //  БІЧНА ПАНЕЛЬ
    // ===================================================================
    property var parsedInput: parseCoordinate(coordInput.text)

    component NumField: Row {
        property alias label: lbl.text
        property alias value: tf.text
        spacing: 6
        Label { id: lbl; width: 132; color: "white"; font.pixelSize: 12; anchors.verticalCenter: parent.verticalCenter }
        TextField { id: tf; width: 82; height: 28; selectByMouse: true; inputMethodHints: Qt.ImhFormattedNumbersOnly; font.pixelSize: 12 }
    }

    Rectangle {
        id: panel
        width: 268
        height: Math.min(contentCol.implicitHeight + 16, _root.height - 24)
        x: _root.width - width - 12
        y: Math.max(8, Math.min(Math.round(_root.height * 0.18), _root.height - height - 8))
        color: "#d91c1f26"; radius: 6; border.color: "#555b66"; border.width: 1

        Flickable {
            anchors.fill: parent; anchors.margins: 8
            contentWidth: width; contentHeight: contentCol.implicitHeight
            clip: true; boundsBehavior: Flickable.StopAtBounds

            Column {
                id: contentCol
                width: 252; spacing: 5

                Label { text: qsTr("Relay: баражування"); color: "white"; font.bold: true; font.pixelSize: 13 }
                NumField { id: fRadius; label: qsTr("Радіус орбіти, м");  value: "600" }
                NumField { id: fAlt;    label: qsTr("Висота (AGL), м");   value: "400" }
                NumField { id: fSpeed;  label: qsTr("Швидкість, км/год"); value: "100" }
                Row {
                    spacing: 6
                    Button { text: qsTr("Застосувати…"); width: 140; height: 30; onClicked: requestOrbit(qsTr("Кнопка «Застосувати»")) }
                    Button { text: qsTr("Стоп"); width: 82; height: 30; onClicked: stopOrbit() }
                }
                Label { color: "#c8d0dc"; font.pixelSize: 11; font.italic: true; wrapMode: Text.WordWrap; width: parent.width
                        text: statusText; visible: statusText !== "" }

                Rectangle { width: parent.width; height: 1; color: "#555b66" }
                Label { color: "#ff8a8a"; font.pixelSize: 11; wrapMode: Text.WordWrap; width: parent.width
                        text: qsTr("Наведення: ") + fmtPoint(aimCoord) }
                Label { color: "#8ac4ff"; font.pixelSize: 11; wrapMode: Text.WordWrap; width: parent.width
                        text: qsTr("Баражування: ") + fmtPoint(loiterCoord) }

                TextField {
                    id: coordInput
                    width: parent.width; height: 30; font.pixelSize: 12; selectByMouse: true
                    placeholderText: qsTr("MGRS або lat, lon")
                }
                Label {
                    width: parent.width; font.pixelSize: 10; wrapMode: Text.WordWrap
                    visible: coordInput.text !== ""
                    color: parsedInput.ok ? "#9be29b" : "#ffb3b3"
                    text: parsedInput.ok ? ("✓ " + parsedInput.kind + ": " + parsedInput.lat.toFixed(5) + ", " + parsedInput.lon.toFixed(5)
                                            + "\n" + mgrsFormat(parsedInput.lat, parsedInput.lon, 5))
                                         : parsedInput.error
                }
                Row {
                    spacing: 4
                    Button { text: qsTr("Вставити"); width: 74; height: 28; font.pixelSize: 11
                             onClicked: coordInput.text = clipboardText().split("\n")[0].trim() }
                    Button { text: qsTr("→ Наведення"); width: 86; height: 28; font.pixelSize: 11
                             enabled: parsedInput.ok; onClicked: applyCoordInput("aim") }
                    Button { text: qsTr("→ Бараж."); width: 82; height: 28; font.pixelSize: 11
                             enabled: parsedInput.ok; onClicked: applyCoordInput("loiter") }
                }

                Label { id: planLabel;      color: "gainsboro"; font.pixelSize: 11; wrapMode: Text.WordWrap; width: parent.width }
                Label { id: telemetryLabel; color: "white";     font.pixelSize: 11; wrapMode: Text.WordWrap; width: parent.width }

                Rectangle { width: parent.width; height: 1; color: "#555b66" }

                // ---- прихований розділ: підбір у межах geofence ----
                Label {
                    text: (showFinder ? "▾ " : "▸ ") + qsTr("Підібрати (geofence)")
                    color: "#7d8594"; font.pixelSize: 11
                    MouseArea { anchors.fill: parent; onClicked: showFinder = !showFinder }
                }
                Column {
                    visible: showFinder; width: parent.width; spacing: 4
                    Label { color: "#aab2c0"; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width
                            text: fenceInfo() + qsTr("\nОрбіта підбирається всередині дозволеної зони з урахуванням нахилу підвісу до точки наведення. Нічого не відправляє.") }
                    NumField { id: fMargin; label: qsTr("Відступ від межі, м"); value: "100" }
                    NumField { id: fRange;  label: qsTr("Дальність ≤, м (0=∞)"); value: "0" }
                    Row {
                        spacing: 6
                        Button { text: qsTr("Підібрати"); width: 110; height: 28; font.pixelSize: 11
                                 onClicked: { finderText = qsTr("Рахую…"); Qt.callLater(runSuggest) } }
                        Button { text: qsTr("Прийняти"); width: 110; height: 28; font.pixelSize: 11
                                 enabled: suggestion !== null; onClicked: acceptSuggestion() }
                    }
                    Label { color: "#c8d0dc"; font.pixelSize: 11; wrapMode: Text.WordWrap; width: parent.width; text: finderText; visible: finderText !== "" }
                }

                // ---- журнал ----
                Label {
                    text: (showLog ? "▾ " : "▸ ") + qsTr("Журнал (%1)").arg(logCount())
                    color: "#7d8594"; font.pixelSize: 11
                    MouseArea { anchors.fill: parent; onClicked: showLog = !showLog }
                }
                Column {
                    visible: showLog; width: parent.width; spacing: 4
                    Label { color: "#c8d0dc"; font.pixelSize: 10; wrapMode: Text.WrapAnywhere; width: parent.width; text: logTail(6) }
                    Row {
                        spacing: 6
                        Button { text: qsTr("Копіювати CSV"); width: 110; height: 28; font.pixelSize: 11
                                 onClicked: { copyText(logCsv()); statusText = qsTr("Журнал скопійовано (%1 записів)").arg(logCount()) } }
                        Button { text: qsTr("Очистити…"); width: 110; height: 28; font.pixelSize: 11; onClicked: requestClearLog() }
                    }
                }

                Label { color: "#9aa0aa"; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width
                        text: qsTr("Cmd+Shift+ЛКМ — наведення · Cmd+Option+ЛКМ — баражування · ПКМ — меню · маркери можна тягнути") }
            }
        }
    }

    // ===================================================================
    //  ПІДТВЕРДЖЕННЯ (модально, Enter — так, Esc — ні)
    // ===================================================================
    Rectangle {
        id: confirmOverlay
        anchors.fill: parent
        visible: pendingAction !== null
        color: "#80000000"
        z: 100
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.AllButtons
            onPressed: (mouse) => mouse.accepted = true
            onWheel: (wheel) => wheel.accepted = true
        }
        Rectangle {
            width: 400; height: cardCol.implicitHeight + 24
            anchors.centerIn: parent
            color: "#f0222630"; radius: 8; border.color: "#7d8594"; border.width: 1
            Column {
                id: cardCol
                x: 12; y: 12; width: parent.width - 24; spacing: 8
                Label { text: pendingAction ? pendingAction.title : ""; color: "white"; font.bold: true; font.pixelSize: 15 }
                Label { text: pendingAction ? pendingAction.reason : ""; color: "#9aa0aa"; font.pixelSize: 11
                        visible: pendingAction !== null && pendingAction.reason !== ""; width: parent.width; wrapMode: Text.WordWrap }
                Label { text: pendingAction ? pendingAction.lines.join("\n") : ""; color: "white"; font.pixelSize: 13
                        width: parent.width; wrapMode: Text.WordWrap }
                Label { text: pendingAction ? pendingAction.warns.join("\n") : ""; color: "#ffb74d"; font.pixelSize: 12; font.bold: true
                        visible: pendingAction !== null && pendingAction.warns.length > 0; width: parent.width; wrapMode: Text.WordWrap }
                Row {
                    spacing: 10
                    Button { text: qsTr("Підтвердити (Enter)"); width: 180; height: 34; onClicked: confirmPending() }
                    Button { text: qsTr("Скасувати (Esc)");     width: 180; height: 34; onClicked: cancelPending() }
                }
            }
        }
    }
}
