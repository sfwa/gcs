/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

function formatLabel(label) {
    if (label.match(/^[av][xyz]$/)) {
        return label.charAt(0) + '<sub>' + label.charAt(1) + '</sub>';
    } else if (label.match(/^∆/)) {
        return label.charAt(0) + ' <sub>' + label.substring(1) + '</sub>'
    } else {
        return label;
    }
}

function formatUnit(unit) {
    unit = unit.replace('/s/s', ' s<sup>-2</sup>');
    unit = unit.replace(/\/s$/, ' s<sup>-1</sup>');
    unit = unit.replace('*h', '•h');
    return unit;
}

function formatInt(val) {
    return (''+parseInt(val, 10)).replace('-', '\u2212');
}

function formatFloat(val, res) {
    if (val === null) {
        return '';
    } else if (val === undefined) {
        return '';
    } else if (isNaN(val)) {
        return 'NaN';
    } else if (val.toFixed) {
        return val.toFixed(res || 0).replace('-', '\u2212');
    } else {
        return val;
    }
}

function formatBool(val, opts) {
    if (!opts) {
        opts = 'true,false';
    }
    return opts.split(',')[val === true ? 0 : 1];
}

function formatText(val, opts) {
    return val;
}

function intChart() {
    return (
        '<svg class="scalar" width="120" height="20" version="1.1">' +
            '<rect class="bkgnd" x="0" y="0" width="120" height="20" />' +
            '<path class="target" d="M0 0" />' +
            '<path class="value" d="M0 0" />' +
            '<path class="alert" d="M0 0" />' +
            '<circle cx="0" cy="0" r="1.25" class="value" />' +
        '</svg>'
    );
}

function floatChart() {
    return intChart();
}

function boolChart() {
    return (
        '<svg class="bool" width="120" height="20" version="1.1">' +
            '<rect class="bkgnd" x="0" y="0" width="120" height="20" />' +
            '<path class="mid" d="M0 10L120 10" />' +
        '</svg>'
    );
}

function textChart() {
    return '';
}

function formatRow(row) {
    var formatType = row.type.split('(')[0],
        x = row.xScale, y = row.yScale;

    return (
        '<dt class="label">' + formatLabel(row.name) + '</dt>' +
        '<dd class="current ' + formatType + '"></dd>' +
        '<dd class="unit">' + formatUnit(row.unit) + '</dd>' +
        '<dd class="history period-60s">' +
        {int: intChart, float: floatChart, bool: boolChart, text: textChart}[formatType]() +
        '</dd>'
    );
}

function q_to_euler(q) {
    var qx, qy, qz, qw, norm, yaw, pitch, roll;

    qx = -q[0];
    qy = -q[1];
    qz = -q[2];
    qw = q[3];

    norm = Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 1e-6) {
        return {yaw: 0.0, pitch: 0.0, roll: 0.0};
    }

    qx /= norm;
    qy /= norm;
    qz /= norm;
    qw /= norm;

    yaw = Math.atan2(2.0 * (qx * qy + qw * qz),
                     qw * qw - qz * qz - qy * qy + qx * qx);
    pitch = Math.asin(-2.0 * (qx * qz - qy * qw));
    roll = Math.atan2(2.0 * (qy * qz + qx * qw),
                      qw * qw + qz * qz - qy * qy - qx * qx);

    yaw = yaw * 180.0 / Math.PI;
    pitch = pitch * 180.0 / Math.PI;
    roll = roll * 180.0 / Math.PI;

    if (yaw < 0.0) {
        yaw += 360.0;
    }

    return {yaw: yaw, pitch: pitch, roll: roll};
}

var WGS84_A = 6378137.0;

/*
2.3mm lens
var FX = 617.23001311, FY = 614.73771581, CX = 640.0, CY = 480.0;
var K1 = -0.23739513, K2 = 0.05558019, P1 = 0.00167173, P2 = -0.00086707, K3 = -0.00541909;
*/

/*
2.8mm lens:
*/
var FX = 762.19568191, FY = 762.44773083, CX = 627.51187132, CY = 547.71358277; // 640.0, 480.0
var K1 = -2.82925533e-1, K2 = 1.14217268e-1, P1 = 5.30644237e-5, P2 = 6.09206516e-5, K3 = -2.55778433e-2;


function camFromLens(lens) {
    var x0, y0, x, y, i, r2, icdist, deltaX, deltaY;

    x0 = x = (lens[0] - CX) / FX;
    y0 = y = (lens[1] - CY) / FY;

    for (i = 0; i < 10; i++) {
        r2 = x*x + y*y;
        icdist = 1.0 / (1.0 + ((K3 * r2 + K2) * r2 + K1) * r2);
        deltaX = 2.0 * P1 * x * y + P2 * (r2 + 2 * x * x);
        deltaY = P1 * (r2 + 2 * y * y) + 2 * P2 * x * y;
        x = (x0 - deltaX) * icdist;
        y = (y0 - deltaY) * icdist;
    }

    return [x, y];
}

function geoFromCam(cam, v_lat, v_lon, v_alt, q) {
    // Convert x, y image coordinates (in metres at 1.0m viewing distance) to
    // body-frame coordinates
    var cx, cy, cz, qx, qy, qz, qw, tx, ty, tz, rx, ry, rz, fac, n, e;

    cx = -cam[1];
    cy = cam[0];
    cz = 1.0;

    // Transform body frame to world frame
    qx = -q[0];
    qy = -q[1];
    qz = -q[2];
    qw = q[3];

    tx = 2.0 * (qy * cz - qz * cy);
    ty = 2.0 * (qz * cx - qx * cz);
    tz = 2.0 * (qx * cy - qy * cx);

    rx = cx + qw * tx + qy * tz - qz * ty;
    ry = cy + qw * ty + qz * tx - qx * tz;
    rz = cz + qw * tz - qy * tx + qx * ty;

    // Project the ray down to where it intersects with the ground plane.
    // If the z-component is negative or zero, the point is in the sky.
    if (rz < 0.001) {
        return null;
    }

    fac = v_alt / rz;
    n = rx * fac;
    e = ry * fac;

    // Convert north/east offsets to lat/lon
    return [
        v_lat + degrees(n / WGS84_A),
        v_lon + degrees(e / (WGS84_A * Math.cos(radians(v_lat)))),
    ];
}

var sample = {
    'Last Radio Update': 0,
    'Last Telemetry Update': 0,

    // RFD900 status packet
    'Noise': -112, 'RX Error Fixed Count': 0,
    'TX Buffer Length': 100, 'RX Error Count': 3,
    'Remote Noise': 0, 'RSSI': 50,
    'Remote RSSI': 0,

    // UAV state packet
    "Solution Time": 0, "Next Waypoint ID": null, "Altitude": 0,
    "Velocity N": 0, "Velocity E": 0, "Velocity D": 0,
    "Wind N": 0, "Wind E": 0, "Wind D": 0, "Yaw": 0, "Pitch": 0,
    "Roll": 0, "Yaw Rate": 0, "Pitch Rate": 0, "Roll Rate": 0,
    "Position Uncertainty": 4.5, "Altitude Uncertainty": 0.5,
    "Velocity N Uncertainty": 0.4,
    "Velocity E Uncertainty": 0.4,
    "Velocity D Uncertainty": 0.7, "Wind N Uncertainty": 0.8,
    "Wind E Uncertainty": 0.8, "Wind D Uncertainty": 0,
    "Yaw Uncertainty": 5.0, "Pitch Uncertainty": 3.0,
    "Roll Uncertainty": 3.0, "Yaw Rate Uncertainty": 8.0,
    "Pitch Rate Uncertainty": 5.0,
    "Roll Rate Uncertainty": 5.0, "FCS Mode": 0,
    "State Flags": 0, "Navigation State": 0,

    // UAV status packet
    "I/O Board 0 Resets": 0,
    "I/O Board 1 Resets": 0,  "TRICAL 0 Resets": 0,
    "TRICAL 1 Resets": 0, "UKF Resets": 0, "NMPC Resets": 0,
    "Peak Core 0 Usage": 0, "Peak Core 1 Usage": 0,
    "CPU Packets Received": 0, "CPU Packet Errors": 0,
    "GPS SVs": 9, "Telemetry RSSI": -18, "Telemetry Noise": -93,
    "Telemetry Packets Received": 0,
    "Telemetry Packet Errors": 0,

    // GCS packet
    "GCS Barometric Pressure": 0, "GCS Latitude": 0, "GCS Longitude": 0,
    "GCS Height": 41.1,

    // Reference data
    "Reference Latitude": 0.0, "Reference Longitude": 0.0,
    "Reference Altitude": 0.0, "Reference Yaw": 0.0, "Reference Pitch": 0.0,
    "Reference Roll": 0.0, "Reference Airspeed": 0.0,

    // Mission data
    "Nav Updates Remaining": 1,
    "Mission Config": {
        "waypoints": {},
        "paths": {},
        "defaults": {},
        "path_names": {},
        "waypoint_names": {}
    },
    "Mission Extras": {
        "path_names_lookup": {},
        "waypoint_names_lookup": {}
    },
    "Mission Path ID": null,
    "Mission Waypoint Markers": [],
    "Mission Path Lines": [],
    "Mission Boundary Line": null,
    "Ground Height": 0.0
};
var IMAGE_QUEUE = [], LAST_IMAGE = null;

d3.select('#metrics')
    .selectAll("section").data(groups).enter()
        .append("section").attr('id', function (g) { return g.name; })
        .selectAll("dl").data(function (g) { return g.metrics; }).enter()
            .append("dl").html(formatRow);

var ws = new WebSocket("ws://" + window.location.host + "/telemetry");
var flightPathColors = [
    "b38677", "ffd697", "669f46", "00c4ff", "e3a0fa", "ff9971", "dbe7c5",
    "3dffc4", "007fcf", "573f56", "926026", "404a2f", "00a2a1", "4233a6",
    "ac4c54"
];

function addRerouteHandler(marker, pathId, pathName) {
    marker.on('click', function (evt) {
        if (confirm("Reroute to " + pathName + "?")) {
            ws.send(JSON.stringify({"reroute": parseInt(pathId, 10)}));
        }
    });
}

function addDropHandler(marker) {
    marker.on('click', function (evt) {
        window.clearTimeout(marker.timeout);

        if (!evt.originalEvent.shiftKey) {
            var latLng = marker.getLatLng();
            ws.send(JSON.stringify(
                {"drop": {"lat": latLng.lat, "lon": latLng.lng}}));
        }
    });
}

function expireMarker(marker) {
    window.setTimeout(function() {
        map.removeLayer(marker);
    }, 3 * 60 * 10000);
}

ws.onopen = function() {
    ws.send("connect");
};

ws.onmessage = function(evt) {
    var msg = JSON.parse(evt.data);
    if (msg.name) {
        handle_image_ws_message(msg);
    } else {
        handle_telemetry_ws_message(msg);
    }
};

function handle_telemetry_ws_message(newSample) {
    var i;

    if (!newSample) {
        return;
    } else if (newSample["Noise"] !== undefined) {
        sample["Noise"] = newSample["Noise"];
        sample["RSSI"] = newSample["RSSI"];
        sample["Telemetry Noise"] = newSample["Remote Noise"];
        sample["Telemetry RSSI"] = newSample["Remote RSSI"];
        sample["RX Error Count"] = newSample["RX Errors"];
        sample["RX Error Fixed Count"] = newSample["RX Errors Fixed"];
    } else if (newSample["GCS Barometric Pressure"] !== undefined) {
        sample["GCS Barometric Pressure"] = newSample["GCS Barometric Pressure"];
        sample["GCS Latitude"] = newSample["GCS Latitude"] || sample["GCS Latitude"];
        sample["GCS Longitude"] = newSample["GCS Longitude"] || sample["GCS Longitude"];
        sample["GCS Height"] = newSample["GCS Height"] || sample["GCS Height"];
    } else {
        if (newSample["Nav Updates Remaining"] !== undefined) {
            sample["Nav Updates Remaining"] = newSample["Nav Updates Remaining"];
        }
        //try {
            if (newSample["Mission Config"] !== undefined) {
                sample["Mission Config"] = JSON.parse(newSample["Mission Config"]);

                // Create mappings for path/waypoint ID->path/waypoint name
                var pathNames = sample["Mission Config"]["path_names"];
                sample["Mission Extras"]["path_names_lookup"] = {
                    499: "HOLD",
                    498: "INTERP.",
                    497: "RESUME",
                    496: "STAB.",
                    495: "RALLY",
                    494: "HOME"
                };
                for (key in pathNames) {
                    if (pathNames.hasOwnProperty(key)) {
                        sample["Mission Extras"]["path_names_lookup"][pathNames[key]] = key;
                    }
                }

                var waypointNames = sample["Mission Config"]["waypoint_names"];
                sample["Mission Extras"]["waypoint_names_lookup"] = {
                    999: "HOLD",
                    998: "INTERP.",
                    997: "RESUME",
                    996: "STAB.",
                    995: "HOME",
                    994: "RALLY"
                };
                for (key in waypointNames) {
                    if (waypointNames.hasOwnProperty(key)) {
                        sample["Mission Extras"]["waypoint_names_lookup"][waypointNames[key]] = key;
                    }
                }

                // Clear out old markers
                for (i = 0; i < sample["Mission Waypoint Markers"].length; i++) {
                    map.removeLayer(sample["Mission Waypoint Markers"][i]);
                }
                sample["Mission Waypoint Markers"] = [];

                for (i = 0; i < sample["Mission Path Lines"].length; i++) {
                    map.removeLayer(sample["Mission Path Lines"][i]);
                }
                sample["Mission Path Lines"] = [];

                if (sample["Mission Boundary Line"]) {
                    map.removeLayer(sample["Mission Boundary Line"]);
                    sample["Mission Boundary Line"] = null;
                }

                // Create new waypoint markers
                var waypoints = sample["Mission Config"]["waypoints"]
                for (key in waypoints) {
                    if (waypoints.hasOwnProperty(key)) {
                        var waypointAlt = waypoints[key].pos[2];
                        var m = L.marker([0, 0], {
                            icon: L.divIcon({
                                className: 'waypoint-label',
                                html: '<span>' + sample["Mission Extras"]["waypoint_names_lookup"][key] +
                                      '(' + (waypointAlt > 0.0 ? '+' : '-') + waypointAlt.toFixed(1) + ' m)</span>',
                                clickable: false,
                                keyboard: false,
                                iconSize: [1, 16]
                            })
                        });

                        m.setLatLng(L.latLng(waypoints[key].pos[0], waypoints[key].pos[1]));
                        m.addTo(map);
                        sample["Mission Waypoint Markers"].push(m);
                    }
                }

                // Create new paths
                var paths = sample["Mission Config"]["paths"];
                for (key in paths) {
                    if (paths.hasOwnProperty(key)) {
                        var startWaypoint, endWaypoint;

                        startWaypoint = waypoints[sample["Mission Config"]["waypoint_names"][paths[key].start]];
                        endWaypoint = waypoints[sample["Mission Config"]["waypoint_names"][paths[key].end]];

                        var line = L.polyline([], {
                            color: '#666',
                            smoothFactor: 0.1,
                            lineJoin: 'round',
                            weight: 2.0,
                            opacity: 1.0,
                            dashArray: paths[key].type == "STRAIGHT" ? null : '5, 10',
                            clickable: false,
                            keyboard: false
                        });

                        line.addLatLng(L.latLng(startWaypoint.pos[0], startWaypoint.pos[1]));
                        line.addLatLng(L.latLng(endWaypoint.pos[0], endWaypoint.pos[1]));
                        line.addTo(map);
                        sample["Mission Path Lines"].push(line);

                        if (paths[key].start != paths[key].end) {
                            var m = L.marker([0, 0], {
                                icon: L.divIcon({
                                    className: 'path-label',
                                    html: '<span>' + sample["Mission Extras"]["path_names_lookup"][key] + '</span>',
                                    clickable: true,
                                    keyboard: false,
                                    iconSize: [1, 16]
                                })
                            });
                            m.setLatLng(L.latLng((startWaypoint.pos[0] + endWaypoint.pos[0]) * 0.5,
                                                 (startWaypoint.pos[1] + endWaypoint.pos[1]) * 0.5));
                            addRerouteHandler(m, key, sample["Mission Extras"]["path_names_lookup"][key]);
                            m.addTo(map);
                            sample["Mission Path Lines"].push(m);
                        }

                        if (sample["Mission Extras"]["path_names_lookup"][key] == "DROP-EXIT") {
                            var dropSafeIndicator = document.getElementById("dropenable");

                            if (paths[key].type == "RELEASE") {
                                dropSafeIndicator.checked = true;
                            } else {
                                dropSafeIndicator.checked = false;
                            }
                        }
                    }
                }

                // Create the boundary
                var boundary = (sample["Mission Config"]["defaults"] || {}).boundary;
                if (boundary) {
                    var line = L.polyline([], {
                        color: '#f00',
                        weight: 10.0,
                        opacity: 0.5,
                        clickable: false,
                        keyboard: false
                    }), waypoint;

                    boundary = boundary.split(",");

                    for (var i = 0, l = boundary.length; i < l; i++) {
                        waypoint = waypoints[sample["Mission Config"]["waypoint_names"][boundary[i].trim()]];
                        line.addLatLng(L.latLng(waypoint.pos[0], waypoint.pos[1]));
                    }

                    waypoint = waypoints[sample["Mission Config"]["waypoint_names"][boundary[0].trim()]];
                    line.addLatLng(L.latLng(waypoint.pos[0], waypoint.pos[1]));

                    line.addTo(map);
                    sample["Mission Boundary Line"] = line;
                }
            }
        //} catch (exc) {}

        // Merge the new sample data in with the current sample
        try {
            sample["Latitude"] = newSample["FCS_PARAMETER_ESTIMATED_POSITION_LLA"][0] * 180.0 / 2147483648.0;
            sample["Longitude"] = newSample["FCS_PARAMETER_ESTIMATED_POSITION_LLA"][1] * 180.0 / 2147483648.0;
            sample["Altitude"] = newSample["FCS_PARAMETER_ESTIMATED_POSITION_LLA"][2] * 1e-2 + sample["GCS Height"] - newSample["Ground Height"];
        } catch (exc) {}
        try {
            sample["Velocity N"] = newSample["FCS_PARAMETER_ESTIMATED_VELOCITY_NED"][0] * 1e-2;
            sample["Velocity E"] = newSample["FCS_PARAMETER_ESTIMATED_VELOCITY_NED"][1] * 1e-2;
            sample["Velocity D"] = newSample["FCS_PARAMETER_ESTIMATED_VELOCITY_NED"][2] * 1e-2;
        } catch (exc) {}
        try {
            sample["Wind N"] = newSample["FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED"][0] * 1e-2;
            sample["Wind E"] = newSample["FCS_PARAMETER_ESTIMATED_WIND_VELOCITY_NED"][1] * 1e-2;
            sample["Wind D"] = 0.0;
        } catch (exc) {}
        try {
            var x, y, z, w, result;
            x = newSample["FCS_PARAMETER_ESTIMATED_ATTITUDE_Q"][0] * (1.0 / 32767.0);
            y = newSample["FCS_PARAMETER_ESTIMATED_ATTITUDE_Q"][1] * (1.0 / 32767.0);
            z = newSample["FCS_PARAMETER_ESTIMATED_ATTITUDE_Q"][2] * (1.0 / 32767.0);
            w = newSample["FCS_PARAMETER_ESTIMATED_ATTITUDE_Q"][3] * (1.0 / 32767.0);
            result = q_to_euler([x, y, z, w]);
            sample["Yaw"] = result.yaw;
            sample["Pitch"] = result.pitch;
            sample["Roll"] = result.roll;
        } catch (exc) {}
        try {
            sample["FCS Mode"] = String.fromCharCode(newSample["FCS_PARAMETER_AHRS_MODE"][0]);
        } catch (exc) {}
        try {
            sample["Mission Path ID"] = newSample["FCS_PARAMETER_NAV_PATH_ID"][0];
        } catch (exc) {}

        // Set last update timestamps
        if (newSample['Noise'] !== undefined) {
            sample['Last Radio Update'] = new Date();
        }
        if (newSample['Solution Time'] !== undefined) {
            sample['Last Telemetry Update'] = new Date();
        }
        try {
            sample['Navigation State'] = newSample["FCS_PARAMETER_NAV_VERSION"][0];
        } catch (exc) {}
        try {
            sample['State Flags'] = (newSample["FCS_PARAMETER_GP_IN"][0] & 1) ? 'M' : 'P';
        } catch (exc) {}
        try {
            sample['NMPC Resets'] = newSample["FCS_PARAMETER_CONTROL_STATUS"][3];
        } catch (exc) {}
        try {
            sample['Reference Latitude'] = newSample["_reference_waypoint"]["lat"] * 180.0 / Math.PI;
            sample['Reference Longitude'] = newSample["_reference_waypoint"]["lon"] * 180.0 / Math.PI;
            sample['Reference Altitude'] = newSample["_reference_waypoint"]["alt"];
            sample['Reference Yaw'] = newSample["_reference_waypoint"]["yaw"] * 180.0 / Math.PI;
            sample['Reference Pitch'] = newSample["_reference_waypoint"]["pitch"] * 180.0 / Math.PI;
            sample['Reference Roll'] = newSample["_reference_waypoint"]["roll"] * 180.0 / Math.PI;
            sample['Reference Airspeed'] = newSample["_reference_waypoint"]["airspeed"];
        } catch (exc) {}

        if (sample["Latitude"]) {
            var pos = L.latLng(sample["Latitude"], sample["Longitude"]),
                referencePos = L.latLng(sample["Reference Latitude"],
                                        sample["Reference Longitude"]);
            if (!window.isMapViewSet) {
                map.setView([sample["Latitude"], sample["Longitude"]], 15);
                window.isMapViewSet = true;
            }
            marker.setLatLng(pos);
            document.getElementById("plane-marker").style["-webkit-transform"] =
                "rotate(" + (sample["Yaw"] - 90) + "deg)";

            if (!window.flightPath) {
                window.flightPath = L.polyline([], {
                    color: '#f00',
                    smoothFactor: 0.1,
                    lineJoin: 'round',
                    weight: 1.0,
                    opacity: 1.0,
                    clickable: false,
                    keyboard: false
                }).addTo(map);
            }

            window.flightPath.addLatLng(pos);
            if (window.flightPath.getLatLngs().length > 7200) {
                window.flightPath.spliceLatLngs(0, 1);
            }
        }
    }
};

function handle_image_ws_message(msg) {
    if (msg.targets) {
        d3.text("/alt/" + msg.lat.toString() + "/" + msg.lon.toString(), function(err, result) {
            var groundAlt = parseFloat(result), geo;
            for (var i = 0, l = msg.targets.length; i < l; i++) {
                geo = geoFromCam(
                    camFromLens([msg.targets[i].x, msg.targets[i].y]),
                    msg.lat, msg.lon, msg.alt - groundAlt + sample["GCS Height"], msg.q);

                if (geo) {
                    var m = L.marker([0, 0], {
                        icon: L.divIcon({
                            className: 'target-label',
                            html: '<div style="background: url(data:image/jpeg;base64,' + msg.thumb + ') -' + (i*16) + 'px;width:16px;height:16px"></div>',
                            iconSize: [16, 16]
                        })
                    });

                    m.setLatLng(L.latLng(geo[0], geo[1]));
                    addDropHandler(m);
                    expireMarker(m);
                    m.addTo(map);
                }
            }
        });
    }

    if (msg.name && msg.status) {
        var m = L.marker([0, 0], {
            icon: L.divIcon({
                className: 'photo-label',
                html: '<div>&#x29bf;</div>',
                iconSize: [16, 16]
            })
        }), loc, popup;
        loc = msg.name.substring(0, msg.name.length - 4).split('_');

        m.setLatLng(L.latLng(parseFloat(loc[1]), parseFloat(loc[2])));

        popup = L.popup({minWidth: 640, maxWidth: 640, minHeight: 480, maxHeight: 480, keepInView: true});
        popup.setContent("<img class='georef-img' data-loc='" + JSON.stringify(loc) + "' src='http://telemetry-relay.au.tono.my:31285/vQivxdjcFcUH34mLAEcfm77varwTmAA8/" + msg.session + "/" + msg.name + "' width=640 height=480>")
        m.bindPopup(popup);
        m.addTo(map);

        // Only queue images if they're above 20m altitude
        if (parseFloat(loc[3]) > 20.0) {
            IMAGE_QUEUE.push(m);
        }
    }
};

// Add an image click handler to set the drop location based on manual target
// recognition
document.addEventListener('click', function(evt) {
    if (evt.target.classList.contains('georef-img')) {
        // Calculate actual lat/lon based on location clicked in image
        var loc = JSON.parse(evt.target.dataset.loc), lat, lng, alt, q, geo, x, y, imgRect;
        lat = parseFloat(loc[1]);
        lon = parseFloat(loc[2]);
        alt = parseFloat(loc[3]);
        q = [parseFloat(loc[4]), parseFloat(loc[5]), parseFloat(loc[6]), parseFloat(loc[7])];

        imgRect = evt.target.getBoundingClientRect();

        x = (evt.clientX - imgRect.left) / imgRect.width * 1280.0;
        y = (evt.clientY - imgRect.top) / imgRect.height * 960.0;

        d3.text("/alt/" + lat.toString() + "/" + lon.toString(), function(err, result) {
            var groundAlt = parseFloat(result);
            geo = geoFromCam(camFromLens([x, y]), lat, lon, alt - groundAlt + sample["GCS Height"], q);

            if (geo) {
                var m = L.marker([0, 0], {
                    icon: L.divIcon({
                        className: 'drop-label',
                        html: '<div>&#x2716;</div>',
                        iconSize: [16, 16]
                    })
                });
                m.setLatLng(L.latLng(geo[0], geo[1]));
                addDropHandler(m);
                m.addTo(map);

                console.log("Georeferenced image ", evt.target.src, " point (", x, ", ", y, ") to (", geo[0], ", ", geo[1], ")");
            } else {
                alert("Couldn't georeference: don't click the sky.");

                console.log("Couldn't georeference image ", evt.target.src, " point (", x, ", ", y, ")");
            }
        });
    }
});

document.addEventListener('keydown', function(evt) {
    if (evt.keyCode != 65 && evt.keyCode != 83) {
        return true;
    }

    var nextImage, idx = -1, dir = evt.keyCode == 65 ? -1 : 1;
    if (LAST_IMAGE) {
        idx = IMAGE_QUEUE.indexOf(LAST_IMAGE);
    }

    nextImage = IMAGE_QUEUE.length > idx + dir && idx + dir >= 0 ? IMAGE_QUEUE[idx + dir] : null;
    if (nextImage) {
        nextImage.openPopup();
        LAST_IMAGE = nextImage;
    }

    return false;
});

// Update display based on sample data every second
window.setInterval(function () {
    updateCalculatedMetrics(sample);
    for (var i = 0; i < groups.length; i++) {
        groups[i].update(sample);
    }

    // Display the new data
    displaySample(sample);
}, 500);

function displaySample(newSample) {
    // Store the selection so we can refer back to it to update multiple
    // elements
    var sel = d3.selectAll("section").data(groups)
                .selectAll("dl").data(function (g) { return g.metrics; });

    sel.classed("alert",
        function (m) { return m.alertData[m.alertData.length - 1]; });

    sel.select("dd.current").html(function (m) {
        var formatParam = m.type.match(/[^(]+\(([^)]+)\)$/),
            formatType = m.type.split('(')[0],
            formatter = {
                'bool': formatBool,
                'int': formatInt,
                'float': formatFloat,
                'text': formatText
            }[formatType],
            out = "";

        out = formatter(m.valueData[m.valueData.length - 1],
                        formatParam ? formatParam[1] : null);

        if (m.intervalData[m.intervalData.length - 1] !== null) {
            out += "<span class='interval'>±" + formatFloat(
                m.intervalData[m.intervalData.length - 1], "1") + "</span>";
        }

        return out;
    });

    var scalarCharts = sel.select("dd.history svg.scalar")

    scalarCharts.select("path.target")
        .attr("d", function (m) {
            var x = m.xScale.range([0, 122]), y = m.yScale.range([20, 0]);

            return (d3.svg.area().defined(function (d, i) {
                        return d && d.min && d.max;
                    }).x(function (d, i) {
                        return parseInt(x(i), 10) + 0.5;
                    }).y0(function (d) {
                        return parseInt(y(d.min), 10) + 0.5;
                    }).y1(function (d) {
                        return parseInt(y(d.max), 10) + 0.5;
                    }).interpolate("linear"))(m.targetData) || "M0 0";
        });

    scalarCharts.select("path.value")
        .attr("d", function (m) {
            var x = m.xScale.range([0, 122]), y = m.yScale.range([20, 0]);

            return (d3.svg.line().defined(function (d, i) {
                        return d;
                    }).x(function (d, i) {
                        return parseInt(x(i), 10) + 0.5;
                    }).y(function (d, i) {
                        return parseInt(y(d), 10) + 0.5;
                    }).interpolate("linear"))(m.valueData) || "M0 0";
        });

    scalarCharts.select("path.alert")
        .attr("d", function (m) {
            var x = m.xScale.range([0, 122]), y = m.yScale.range([20, 0]);

            return (d3.svg.line().defined(function (d, i) {
                        return d && (m.alertData[i] ||
                            (i > 0 && m.alertData[i-1]) ||
                            (i < m.alertData.length - 1 && m.alertData[i+1]));
                    }).x(function (d, i) {
                        return parseInt(x(i), 10) + 0.5;
                    }).y(function (d, i) {
                        return parseInt(y(d), 10) + 0.5;
                    }).interpolate("linear"))(m.valueData) || "M0 0";
        });

    scalarCharts.select("circle.value").attr("cx", function(m) {
        var x = m.xScale.range([0, 122]);
        return parseInt(x(m.valueData.length - 1), 10) + 0.5;
    });

    scalarCharts.select("circle.value").attr("cy", function(m) {
        var y = m.yScale.range([20, 0]);
        return parseInt(y(m.valueData[m.valueData.length - 1]), 10) + 0.5;
    });

    var boolCharts = sel.select("dd.history svg.bool");

    boolCharts.each(function (m, i) {
        var x = m.xScale.range([0, 122]),
            y = m.yScale.range([20, 0]);

        var barSel = d3.select(this).selectAll(".value").data(m.valueData);

        barSel.enter().append("rect")
            .attr("class", function (d, i) {
                    return "value " + (m.alertData[i] ? "alert" : "");
                })
            .attr("x", function (d, i) { return x(i); })
            .attr("y", function (d, i) { return y(d ? 1 : 0); })
            .attr("height", function (d, i) {
                return Math.abs(y(1) - y(0)); })
            .attr("width", Math.abs(x(1) - x(0)) * 0.5);

        barSel.attr("class", function (d, i) {
                return "value " + (m.alertData[i] ? "alert" : "");
            })
            .attr("y", function (d, i) { return y(d ? 1 : 0); });

        barSel.exit().remove();
    });
}

window.onload = function() {
    document.getElementById("txdisable").onclick = function() {
        if (this.checked) {
            ws.send(JSON.stringify({"tx": "disable"}));
        } else {
            ws.send(JSON.stringify({"tx": "enable"}));
        }
    };

    document.getElementById("dropenable").onclick = function() {
        if (this.checked) {
            ws.send(JSON.stringify({"drop_enable": "enable"}));
        } else {
            ws.send(JSON.stringify({"drop_enable": "disable"}));
        }
    };

    document.getElementById("abort").onclick = function() {
        if (confirm("Abort?")) {
            ws.send(JSON.stringify({"abort": "abort"}));
        }
    };
};
