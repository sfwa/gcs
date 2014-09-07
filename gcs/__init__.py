#Copyright (C) 2013 Ben Dyer
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in
#all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

import os
import cgi
import json
import uuid
import stat
import time
import math
import email
import base64
import slimit
import cssmin
import urllib
import hashlib
import httplib
import tempfile
import datetime
import cStringIO
import subprocess
import tornado.web
import ConfigParser
import logging as log
import tornado.ioloop
import tornado.template
from tornado import gen
import tornado.websocket
from gcs import radio, plog


ASSET_MANIFEST = {
    "styles": [
        "/static/css/normalize.css",
        "/static/css/gcs.css"
    ],
    "js": [
        "/static/js/d3.js",
        "/static/js/metrics.js",
        "/static/js/gcsconfig.js"
    ],
    "body_js": [
        "/static/js/gcs.js"
    ]
}


WGS84_A = 6378137.0
NOTIFY_SOCKETS = set()
LAST_NAV_STATE_IDX = 0
LAST_FRAME = None
LAST_FRAME_TIME = time.time()
PENDING_CHANGES = 0
START_PATH_ID = -1
START_PATH_NAV_STATE_IDX = 0
GCS_STATE = {"Latitude": 0, "Longitude": 0, "Altitude": 0, "Pressure": 0}
MISSION = None
TX_ENABLE = True
ABORT = False
LAST_RADIO_UPDATE = time.time()
LAST_OUT_MSG = None


@tornado.gen.coroutine
def relay_conn():
    global LAST_OUT_MSG

    log.info("relay_conn()")
    io_loop = tornado.ioloop.IOLoop.instance()
    ws = None

    while True:
        # Rate-limit re-connects
        yield tornado.gen.Task(
                tornado.ioloop.IOLoop.instance().call_later, 2.0)

        try:
            # Ensure the web socket is connected
            log.info("relay_conn(): connecting web socket")
            ws = yield tornado.websocket.websocket_connect(
                "ws://telemetry-relay.au.tono.my:31285/mwPhW4f8wKjUqH2fsWMDDiqk6z3meHGZ",
                connect_timeout=30)

            while ws.protocol is not None:
                # Closing the socket will trigger a null message, which
                # will abort the coroutine
                timeout = io_loop.call_later(30.0, ws.close)

                # Send the packet over the relay link as well
                if LAST_OUT_MSG:
                    log.info("relay_conn(): writing {0}".format(repr(LAST_OUT_MSG)))
                    ws.write_message(LAST_OUT_MSG, binary=True)

                msg = yield ws.read_message()
                if not msg:
                    log.info("relay_conn(): socket was closed")
                    ws.close()  # For good measure
                    break

                handle_relay_message(msg)

                log.info(
                    "relay_conn(): read message {0}".format(repr(msg)))
                io_loop.remove_timeout(timeout)

            ws.close()  # Just in case
        except Exception:
            log.exception("relay_conn(): send run encountered an error")


def handle_radio_packet(nmea_conn, packet, conn):
    '''
    Invoked each time a packet is received from the radio connection. Publish
    to the websocket message queue, and add a callback to the main ioloop to
    ensure it's published to individual sockets.
    '''
    global LAST_NAV_STATE_IDX, PENDING_CHANGES, LAST_RADIO_UPDATE, \
           LAST_FRAME, LAST_FRAME_TIME
    if not packet[1]:
        return

    #log.info("handle_radio_packet(%s, %s)" % (repr(packet), repr(conn)))
    log.info("handle_radio_packet(..., %s)" % (repr(conn)))

    changed = False
    if "FCS_PARAMETER_NAV_VERSION" in packet[1]:
        LAST_FRAME = packet[1]
        LAST_FRAME_TIME = time.time()
        if conn:
            LAST_RADIO_UPDATE = LAST_FRAME_TIME

        try:
            if nmea_conn:
                send_nmea_packet(nmea_conn)
        except Exception:
            log.exception("handle_radio_packet(): failed to write NMEA")

        LAST_NAV_STATE_IDX = packet[1]["FCS_PARAMETER_NAV_VERSION"][0]
        if MISSION and MISSION.is_sync_complete():
            MISSION.current_nav_state_id = LAST_NAV_STATE_IDX
            changed = MISSION.parse_config()
    packet[1]["Nav Updates Remaining"] = PENDING_CHANGES
    if changed:
        packet[1]["Mission Config"] = MISSION.to_json()

    # Notify currently-open sockets
    msg = json.dumps(packet[1])
    for socket in NOTIFY_SOCKETS:
        socket.write_message(msg)


def send_heartbeat_packet(conn, mission):
    '''
    Invoked periodically to send GCS heartbeats to the UAV.
    '''
    global LAST_FRAME_ID, LAST_NAV_STATE_IDX, PENDING_CHANGES, START_PATH_ID,\
           START_PATH_NAV_STATE_IDX, MISSION, TX_ENABLE, ABORT, LAST_OUT_MSG

    param_log = plog.ParameterLog()

    PENDING_CHANGES = len(mission.pending_changes)
    if mission != MISSION:
        MISSION = mission
        msg = json.dumps({"Mission Config": MISSION.to_json()})
        for socket in NOTIFY_SOCKETS:
            socket.write_message(msg)

    if not mission.is_sync_complete():
        key, idx, state_idx, packet = mission.next_change(LAST_NAV_STATE_IDX)

        param_log.append(plog.KeyValueParameter(
            device_id=0, key=key,
            value=packet))
        param_log.append(plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_NAV_VERSION,
            value_type=plog.ValueType.FCS_VALUE_UNSIGNED,
            value_precision=32, values=[state_idx]))

        param_type = plog.ParameterType.FCS_PARAMETER_NAV_PATH_ID
        if key == plog.FCS_PARAMETER_KEY_WAYPOINT:
            param_type = plog.ParameterType.FCS_PARAMETER_NAV_WAYPOINT_ID

        param_log.append(plog.DataParameter(
            device_id=0,
            parameter_type=param_type,
            value_type=plog.ValueType.FCS_VALUE_UNSIGNED,
            value_precision=16, values=[idx]))
    elif START_PATH_ID >= 0:
        param_log.append(plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_NAV_PATH_ID,
            value_type=plog.ValueType.FCS_VALUE_UNSIGNED,
            value_precision=16, values=[START_PATH_ID]))
        param_log.append(plog.KeyValueParameter(
            device_id=0, key=plog.FCS_PARAMETER_KEY_REROUTE,
            value="0"))
        param_log.append(plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_NAV_VERSION,
            value_type=plog.ValueType.FCS_VALUE_UNSIGNED,
            value_precision=32, values=[START_PATH_NAV_STATE_IDX + 1]))
        print repr(param_log)
    elif ABORT == "abort":
        param_log.append(plog.KeyValueParameter(
            device_id=0, key=plog.FCS_PARAMETER_KEY_ABORT,
            value="0"))
        param_log.append(plog.DataParameter(
            device_id=0,
            parameter_type=plog.ParameterType.FCS_PARAMETER_NAV_VERSION,
            value_type=plog.ValueType.FCS_VALUE_UNSIGNED,
            value_precision=32, values=[START_PATH_NAV_STATE_IDX + 1]))

    if TX_ENABLE:
        LAST_OUT_MSG = param_log.serialize()
        conn.send(LAST_OUT_MSG)
    else:
        LAST_OUT_MSG = None


def send_nmea_packet(conn):
    global LAST_FRAME, LAST_FRAME_TIME

    if not LAST_FRAME:
        return

    fix_time = time.gmtime(LAST_FRAME_TIME)

    pos = LAST_FRAME["FCS_PARAMETER_ESTIMATED_POSITION_LLA"]
    lat_frac, lat_d = math.modf(pos[0] * 180.0 / 2147483648.0)
    lon_frac, lon_d = math.modf(pos[1] * 180.0 / 2147483648.0)

    v = LAST_FRAME["FCS_PARAMETER_ESTIMATED_VELOCITY_NED"]
    gs = math.sqrt(v[0]**2 + v[1]**2) * 1e-2
    track = (math.degrees(math.atan2(v[1], v[0])) + 360.0) % 360.0

    var = 0.0
    if "FCS_PARAMETER_ESTIMATED_WMM_FIELD" in LAST_FRAME:
        f = LAST_FRAME["FCS_PARAMETER_ESTIMATED_WMM_FIELD"]
        var = math.degrees(math.atan2(f[1], f[0]))

    mode = LAST_FRAME["FCS_PARAMETER_AHRS_MODE"]
    if chr(mode[0]) in ("S", "R", "A"):
        status = "A"
    else:
        status = "V"

    packet = (
            "GPRMC,{hh:02d}{mm:02d}{ss:02.0f}," +       # HHMMSS UTC
            "{status}," +                               # 'A' or 'V'
            "{lat_d:02.0f}{lat_m:06.3f},{lat_ns:s}," +  # DDMM.mmm,[N|S]
            "{lon_d:03.0f}{lon_m:06.3f},{lon_ew:s}," +  # DDDMM.mmm,[E|W]
            "{gs_kt:05.1f}," +                          # NNN.n (kt)
            "{track_d:05.1f}," +                        # DDD.d (deg true)
            "{DD:02d}{MM:02d}{YY:02d}," +               # DDMMYY date UTC
            "{var_d:05.1f},{var_ew:s},"                 # DDD.d,[E|W] magnetic
                                                        #     variation
        ).format(
            hh=fix_time.tm_hour,
            mm=fix_time.tm_min,
            ss=fix_time.tm_sec,
            status=status,
            lat_d=abs(lat_d),
            lat_m=abs(lat_frac) * 60.0,
            lat_ns="N" if pos[0] > 0.0 else "S",
            lon_d=abs(lon_d),
            lon_m=abs(lon_frac) * 60.0,
            lon_ew="E" if pos[1] > 0.0 else "W",
            gs_kt=gs * 1.94384449,
            track_d=track,
            DD=fix_time.tm_mday,
            MM=fix_time.tm_mon,
            YY=fix_time.tm_year % 100,
            var_d=abs(var),
            var_ew="E" if var > 0.0 else "W"
        )

    checksum = reduce(lambda s, c: s ^ ord(c), packet, 0)

    conn.write("$" + packet + "*{:02X}\r\n".format(checksum))
    conn.flush()


def handle_iomon_packet(packet, conn):
    '''
    Invoked each time a packet is received from the IO board USB connection.
    '''
    global GCS_STATE, NOTIFY_SOCKETS

    log.info("handle_iomon_packet(%s, %s)" % (repr(packet), repr(conn)))

    data = {
        "GCS Barometric Pressure": packet["pressure"],
        "GCS IO State": packet["status"],
        "GCS Flags": 0,
        "GCS CPU State": 0,
        "GCS Frame ID": 0,
        "GCS 3G Signal": 0
    }

    if packet.get("gps"):
        data.update({
            "GCS Latitude": packet["gps"].get("lat"),
            "GCS Longitude": packet["gps"].get("lon"),
            "GCS Altitude": packet["gps"].get("alt")
        })

        # FIXME
        GCS_STATE["Latitude"] = -37.0
        GCS_STATE["Longitude"] = 145.0

    GCS_STATE["Altitude"] = 50.0

    # Average pressure over ~10 seconds
    GCS_STATE["Pressure"] += \
        0.001 * ((float(packet["pressure"]) * 100.0) - GCS_STATE["Pressure"])

    # Notify currently-open sockets
    msg = json.dumps(data)
    for socket in NOTIFY_SOCKETS:
        socket.write_message(msg)


def handle_relay_message(msg):
    global NOTIFY_SOCKETS, LAST_RADIO_UPDATE

    log.info("handle_relay_message(%s)" % repr(msg))

    # If it's been a while since the last regular telemetry update, forward
    # the 3G update through
    try:
        msg = json.loads(msg)
        if msg.get("telemetry") and time.time() - LAST_RADIO_UPDATE > 8.0:
            log.warning("handle_relay_message(): using 3G telemetry!")
            handle_radio_packet(None, radio.parse_packet(base64.b64decode(msg["telemetry"])), None)

        # Pass image data etc straight through to the clients
        for socket in NOTIFY_SOCKETS:
            socket.write_message(msg)
    except Exception:
        raise
        log.info("handle_relay_message(): not a JSON packet")


class UI(tornado.web.RequestHandler):
    _cache_time = 1800  # 30 minutes

    def initialize(self, environment=None):
        self.environment = environment

    # Stupid override to stop Tornado removing whitespace from the template
    def create_template_loader(self, template_path):
        if "template_loader" in self.application.settings:
            return self.application.settings["template_loader"]

        opts = {}
        if "autoescape" in self.application.settings:
            opts["autoescape"] = self.application.settings["autoescape"]

        class Loader(tornado.template.Loader):
            def _create_template(self, name):
                with open(os.path.join(self.root, name), "rb") as f:
                    template = tornado.template.Template(f.read(), name=name,
                        loader=self, compress_whitespace=False)
                return template

        return Loader(template_path, **opts)

    def get(self):
        if self.request.headers.get("x-forwarded-proto") == "http":
            self.redirect("https://%s/" % self.request.host)
        else:
            self.set_header("Date", datetime.datetime.utcnow())
            self.set_header("Vary", "Accept-Encoding")
            self.set_header("Expires", datetime.datetime.utcnow() +
                datetime.timedelta(seconds=UI._cache_time))
            self.set_header("Cache-Control", "public, max-age=" +
                str(UI._cache_time))
            self.render("index.html", environment=self.environment)


class TelemetryHandler(tornado.websocket.WebSocketHandler):
    def __init__(self, *args, **kwargs):
        self.radio_conn = kwargs.pop("radio_conn", None)
        self.gcs_conn = kwargs.pop("gcs_conn", None)
        super(TelemetryHandler, self).__init__(*args, **kwargs)

    def open(self):
        log.info("TelemetryHandler.open(%s)" % repr(self))

    def on_message(self, message):
        global START_PATH_ID, START_PATH_NAV_STATE_IDX, LAST_NAV_STATE_IDX, \
               PENDING_CHANGES, NOTIFY_SOCKETS, MISSION, TX_ENABLE, ABORT
        log.info("TelemetryHandler.on_message(%s, %s)" %
                 (repr(self), repr(message)))

        # Add this socket to the notification set
        if self not in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.add(self)
            if MISSION:
                self.write_message(json.dumps({"Mission Config": MISSION.to_json()}))

        # Check the message type and send a command if requested
        message_data = None
        try:
            message_data = json.loads(message)
        except Exception:
            return

        if PENDING_CHANGES == 0 and "reroute" in message_data and \
                message_data["reroute"] in MISSION.paths:
            START_PATH_ID = message_data["reroute"]
            START_PATH_NAV_STATE_IDX = LAST_NAV_STATE_IDX
            log.info("TelemetryHandler.on_message: rerouting to %d" % START_PATH_ID)
        else:
            START_PATH_ID = -1

        if "drop" in message_data:
            drop_lat = float(message_data["drop"]["lat"])
            drop_lon = float(message_data["drop"]["lon"])

            # Adjust drop_lat based on the distance travelled by the bottle
            # when dropped from 60 m at 21 m/s -- 3.5 s falling and some
            # unknown deceleration, so 40 m is probably reasonable
            drop_lat -= math.degrees(40.0 / WGS84_A)

            # DREN and DREX are always south/north of the drop point
            dren_lat = drop_lat - math.degrees(75.0 / WGS84_A)
            dren_lon = drop_lon

            drex_lat = drop_lat + math.degrees(75.0 / WGS84_A)
            drex_lon = drop_lon

            # Update the position of the DREN, DROP and DREX waypoints in the
            # config file
            MISSION.update_waypoint_in_config(
                "DREN", {"pos": "(%f, %f, %f)" % (dren_lat, dren_lon, 60.0)})
            MISSION.update_waypoint_in_config(
                "DROP", {"pos": "(%f, %f, %f)" % (drop_lat, drop_lon, 60.0)})
            MISSION.update_waypoint_in_config(
                "DREX", {"pos": "(%f, %f, %f)" % (drex_lat, drex_lon, 60.0)})

        if "drop_enable" in message_data:
            if message_data["drop_enable"] == "enable":
                log.info("TelemetryHandler.on_message: DROP ENABLED")
                MISSION.update_path_in_config("DROP-EXIT", {"type": "RELEASE"})
            elif message_data["drop_enable"] == "disable":
                log.info("TelemetryHandler.on_message: DROP DISABLED")
                MISSION.update_path_in_config("DROP-EXIT", {"type": "STRAIGHT"})

        if "tx" in message_data:
            if message_data["tx"] == "disable":
                log.info("TelemetryHandler.on_message: TX DISABLED")
                TX_ENABLE = False
            elif message_data["tx"] == "enable":
                log.info("TelemetryHandler.on_message: TX ENABLED")
                TX_ENABLE = True

        if "abort" in message_data and message_data["abort"] == "abort":
            log.info("TelemetryHandler.on_message: ABORT")
            START_PATH_NAV_STATE_IDX = LAST_NAV_STATE_IDX
            ABORT = "abort"


    def on_close(self):
        log.info("TelemetryHandler.on_close(%s)" % repr(self))

        # Remove from the notification set
        if self in NOTIFY_SOCKETS:
            NOTIFY_SOCKETS.remove(self)


class StaticBuild(tornado.web.RequestHandler):
    _bundles = {}
    _cache_time = 86400*365*10  # 10 years

    @classmethod
    def build(cls, basepath, bundle_key, ext):
        bundle = {}

        # Iterate over files in bundle; determine last modified time and
        # assemble content
        last_mtime = 0
        contents = ""
        for path in ASSET_MANIFEST[bundle_key]:
            path = os.path.join(os.path.abspath(basepath),
                path[len('/static/'):])
            last_mtime = max(last_mtime, os.stat(path)[stat.ST_MTIME])
            contents += open(path, "rb").read() + "\n"

        if ext == "js":
            bundle["contents"] = slimit.minify(
                unicode(contents, encoding='utf-8')).encode('utf-8')
        elif ext == "css":
            bundle["contents"] = cssmin.cssmin(contents)
        else:
            assert False

        bundle["sha1"] = hashlib.sha1(bundle["contents"]).hexdigest()
        bundle["last_modified"] = datetime.datetime.fromtimestamp(last_mtime)
        bundle["mime_type"] = "text/javascript" if ext == "js" else "text/css"

        StaticBuild._bundles[bundle_key] = bundle

    def head(self, resource, bundle):
        self.get(resource, bundle, include_body=False)

    def get(self, resource, bundle, include_body=True):
        key, _, ext = bundle.partition(".")
        if key not in ASSET_MANIFEST or ext not in ("css", "js"):
            raise tornado.web.HTTPError(404)

        if key not in StaticBuild._bundles:
            StaticBuild.build(self.require_setting("static_path"), key, ext)

        bundle = StaticBuild._bundles[key]

        ims_value = self.request.headers.get("If-Modified-Since")
        if ims_value is not None:
            if_since = datetime.datetime.fromtimestamp(
                time.mktime(email.utils.parsedate(ims_value)))
            if if_since >= bundle["last_modified"]:
                self.set_status(304)
                return

        self.set_header("Etag", '"%s"' % bundle["sha1"])
        self.set_header("Date", datetime.datetime.utcnow())
        self.set_header("Last-Modified", bundle["last_modified"])
        self.set_header("Content-Type", bundle["mime_type"])
        self.set_header("Expires", datetime.datetime.utcnow() +
            datetime.timedelta(seconds=StaticBuild._cache_time))
        self.set_header("Cache-Control", "public, max-age=" +
            str(StaticBuild._cache_time))
        self.set_header("Vary", "Accept-Encoding")

        if include_body:
            self.write(bundle["contents"])
        else:
            assert self.request.method == "HEAD"
            self.set_header("Content-Length", len(bundle["contents"]))


# Custom static file handler because CloudFront requires a Date header to do
# any caching based on Expires or Cache-Contro: max-age. Also include a Vary
# header because it makes PageSpeed happy.
class StaticFile(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header("Date", datetime.datetime.utcnow())
        self.set_header("Vary", "Accept-Encoding")
        self.set_header("Cache-Control", "public, max-age=" +
            str(StaticBuild._cache_time))

