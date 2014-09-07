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

defineGroup("Summary")
    .add(defineMetric("mode").basedOn(tmpl.text)
            .field(function (s) {
                return {
                    "I": "initializing",
                    "C": "calibrating",
                    "S": "safe",
                    "R": "armed",
                    "A": "active",
                    "H": "holding",
                    "F": "aborting"
                }[s["FCS Mode"]];
            }))
    .add(defineMetric("mission state").basedOn(tmpl.text)
            .field(function(s) {
                if (s["Nav Updates Remaining"] == 0) {
                    return s["Mission Extras"]["path_names_lookup"][s["Mission Path ID"]];
                } else {
                    return s["Mission Extras"]["path_names_lookup"][s["Mission Path ID"]] + " (" + (s["Nav Updates Remaining"] || 0) + ")"
                }
            }))
    .add(defineMetric("airspeed").basedOn(tmpl.speed).field("Airspeed")
            .range(-10, 40).target(15, 30).alert(10, 35)
            .interval("Airspeed Uncertainty"))
    .add(defineMetric("altitude").basedOn(tmpl.distance).field("Altitude")
            .range(-10, 200).target(70, 140).alert(50, 150)
            .interval("Altitude Uncertainty"))
    .add(defineMetric("track").basedOn(tmpl.heading).field("Track")
            .interval("Track Uncertainty"))
    .add(defineMetric("ground speed").basedOn(tmpl.speed)
            .field("Ground Speed").interval("Ground Speed Uncertainty"))
    .add(defineMetric("climb rate").basedOn(tmpl.velocity).field("Climb Rate")
            .alert(-5, 5).interval("Velocity D Uncertainty"));

defineGroup("Environment")
    .add(defineMetric("latitude",
            {unit: "°", type: "float(6)", range: {min: -90, max: 90}})
            .field("Latitude"))
    .add(defineMetric("longitude",
            {unit: "°", type: "float(6)", range: {min: -180, max: 180}})
            .field("Longitude"))
    .add(defineMetric("wind direction").basedOn(tmpl.heading)
            .field("Wind Direction").interval("Wind Direction Uncertainty"))
    .add(defineMetric("wind speed").basedOn(tmpl.speed).field("Wind Speed")
            .interval("Wind Speed Uncertainty"));

defineGroup("Orientation")
    .add(defineMetric("heading").basedOn(tmpl.heading).field("Heading")
            .interval("Heading Uncertainty"))
    .add(defineMetric("pitch").basedOn(tmpl.angle).field("Pitch")
            .range(-90, 90).target(-30, 30).alert(-45, 45)
            .interval("Pitch Uncertainty"))
    .add(defineMetric("roll").basedOn(tmpl.angle).field("Roll")
            .range(-180, 180).target(-45, 45).alert(-60, 60)
            .interval("Roll Uncertainty"));

defineGroup("Radio")
    .add(defineMetric("GCS margin").basedOn(tmpl.signalMargin)
            .field(function (s) {
                return (s["RSSI"] / 1.9 - 127.0) - (s["Noise"] / 1.9 - 127.0);
            })
            .alert(function (s) { return s["RSSI"] - s["Noise"] < 19; }))
    .add(defineMetric("UAV margin").basedOn(tmpl.signalMargin)
            .field(function (s) {
                return (s["Telemetry RSSI"] / 1.9 - 127.0) - (s["Telemetry Noise"] / 1.9 - 127.0);
            })
            .alert(function (s) { return s["Telemetry RSSI"] - s["Telemetry Noise"] < 19; }))
    .add(defineMetric("error rate").basedOn(tmpl.percentage)
            .field(function (s) {
                return s["Telemetry Packet Errors"] /
                    Math.max(1.0, s["Telemetry Packets Received"]) * 100.0;
            }))
    .add(defineMetric("GPS SVs").basedOn(tmpl.count).field("GPS SVs")
            .range(0, 12).alert(6, 12));

defineGroup("Power")
    .add(defineMetric("main charge").basedOn(tmpl.charge).field("Main Charge")
            .range(0, 20).alert(5, 18))
    .add(defineMetric("aux charge").basedOn(tmpl.charge).field("Aux Charge")
            .range(0, 2).alert(0.5, 2))
    .add(defineMetric("payload", {unit: "", type: "bool(present,missing)"})
            .field(function (s) {
                return (s["State Flags"] || "").charAt(0) == "P" ? true : false;
            }));

defineGroup("GCS Status")
    .add(defineMetric("GCS pressure", {unit: "mbar", type: "float(2)"})
            .field("GCS Barometric Pressure")
            .range(950, 1100).alert(980, 1050));

function radians(v) {
    return v * (Math.PI / 180.0);
}

function degrees(v) {
    return v * (180.0 / Math.PI);
}

function updateCalculatedMetrics(sample) {
    var track = Math.atan2(sample["Velocity E"], sample["Velocity N"]),
        windDirection = Math.atan2(-sample["Wind E"], -sample["Wind N"]),
        airVelocityN = parseFloat(sample["Velocity N"]) -
                       parseFloat(sample["Wind N"]),
        airVelocityE = parseFloat(sample["Velocity E"]) -
                       parseFloat(sample["Wind E"]),
        airVelocityD = parseFloat(sample["Velocity D"]) -
                       parseFloat(sample["Wind D"]),
        airVelocityUN = parseFloat(sample["Velocity N Uncertainty"]) +
                        parseFloat(sample["Wind N Uncertainty"]),
        airVelocityUE = parseFloat(sample["Velocity E Uncertainty"]) +
                        parseFloat(sample["Wind E Uncertainty"]),
        airVelocityUD = parseFloat(sample["Velocity D Uncertainty"]) +
                        parseFloat(sample["Wind D Uncertainty"]);

    sample["Track"] = (degrees(track) + 360) % 360;
    sample["Ground Speed"] = Math.sqrt(
        parseFloat(sample["Velocity N"]) * parseFloat(sample["Velocity N"]) +
        parseFloat(sample["Velocity E"]) * parseFloat(sample["Velocity E"])
    );
    sample["Ground Speed Uncertainty"] = Math.sqrt(
        sample["Velocity N Uncertainty"] * sample["Velocity N Uncertainty"] +
        sample["Velocity E Uncertainty"] * sample["Velocity E Uncertainty"]
    );
    if (sample["Ground Speed"] < 5.0) {
        sample["Track Uncertainty"] = 180.0;
    } else {
        sample["Track Uncertainty"] = Math.min(180.0, 180.0 *
            sample["Ground Speed Uncertainty"] / sample["Ground Speed"]);
    }
    sample["Climb Rate"] = -sample["Velocity D"];

    sample["Wind Direction"] = (degrees(windDirection) + 360) % 360;
    sample["Wind Speed"] = Math.sqrt(
        sample["Wind N"] * sample["Wind N"] +
        sample["Wind E"] * sample["Wind E"]
    );
    sample["Wind Speed Uncertainty"] = Math.sqrt(
        sample["Wind N Uncertainty"] * sample["Wind N Uncertainty"] +
        sample["Wind E Uncertainty"] * sample["Wind E Uncertainty"]
    );
    if (sample["Wind Speed"] < 2.0) {
        sample["Wind Direction Uncertainty"] = 180.0;
    } else {
        sample["Wind Direction Uncertainty"] = Math.min(180.0, 180.0 *
            sample["Wind Speed Uncertainty"] / sample["Wind Speed"]);
    }
    sample["Wind Up"] = -sample["Wind D"];

    sample["Airspeed"] = Math.sqrt(
        airVelocityN * airVelocityN +
        airVelocityE * airVelocityE +
        airVelocityD * airVelocityD
    );
    sample["Airspeed Uncertainty"] = Math.sqrt(
        airVelocityUN * airVelocityUN +
        airVelocityUE * airVelocityUE +
        airVelocityUD * airVelocityUD
    );

    sample["Heading"] = sample["Yaw"];
    sample["Heading Uncertainty"] = sample["Yaw Uncertainty"];

    sample["I/O Board 0 Error"] =
        parseInt(sample["I/O Board 0 Resets"], 10) > 0 ? false : true;
    sample["I/O Board 1 Error"] =
        parseInt(sample["I/O Board 1 Resets"], 10) > 0 ? false : true;
    sample["UKF Error"] =
        parseInt(sample["UKF Resets"], 10) > 0 ? false : true;
    sample["NMPC Error"] =
        parseInt(sample["NMPC Resets"], 10) > 0 ? false : true;
    sample["TRICAL 0 Error"] =
        parseInt(sample["TRICAL 0 Resets"], 10) > 0 ? false : true;
    sample["TRICAL 1 Error"] =
        parseInt(sample["TRICAL 1 Resets"], 10) > 0 ? false : true;
}
