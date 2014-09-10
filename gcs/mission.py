# TODO:
# - Open mission file and parse config
# - Diff with current config -- iterate over all waypoint IDs and path IDs plus
#   mission boundary and identify changes
# - Create a change packet for each change, with appropriate data
# - Iterate:
#   - Compare current nav state ID with last requested change -- if equal then
#     push the next change and record the expected nav state ID

import collections
import plog
import json
import math
import ConfigParser
import logging as log


PATH_TYPE_LOOKUP = {
    None: "D",
    "CURVE": "D",
    "STRAIGHT": "L",
    "RELEASE": "R",
    "FIGURE EIGHT": "8"
}


class Mission(object):
    def __init__(self, config_path, current_nav_state_id, heightmap=None):
        self.current_config_json = "{}"
        self.paths = {}
        self.waypoints = {}
        self.boundary = []
        self.config_path = config_path
        self.pending_changes = collections.deque()
        self.current_nav_state_id = current_nav_state_id
        self.heightmap = heightmap

    def _next_state_idx(self):
        if len(self.pending_changes):
            return self.pending_changes[-1][2] + 1
        else:
            return self.current_nav_state_id + 1

    def to_json(self):
        return self.current_config_json

    def from_json(self, new_config_json):
        config = json.loads(new_config_json)

        paths = config["paths"]
        waypoints = config["waypoints"]
        defaults = config["defaults"]

        path_names = config["path_names"]
        waypoint_names = config["waypoint_names"]

        out_paths = {}
        out_waypoints = {}

        for idx, wp in waypoints.iteritems():
            out_waypoint = {}

            out_waypoint["airspeed"] = float(wp.get("airspeed",
                                             defaults.get("airspeed", 20.0)))
            out_waypoint["yaw"] = math.radians(float(wp.get("yaw", 0.0)))
            out_waypoint["pitch"] = math.radians(float(wp.get("pitch", 0.0)))
            out_waypoint["roll"] = math.radians(float(wp.get("roll", 0.0)))
            out_waypoint["alt"] = float(wp.get(
                "pos", [0, 0, defaults.get("alt", 70.0)])[2])
            if self.heightmap:
                out_waypoint["alt"] += self.heightmap.lookup(
                    out_waypoint["lat"], out_waypoint["lon"])
            out_waypoint["lat"] = math.radians(float(wp["pos"][0]))
            out_waypoint["lon"] = math.radians(float(wp["pos"][1]))
            out_waypoint["flags"] = int(wp.get("flags", "0x0"), 16)

            out_waypoints[int(idx)] = out_waypoint

        for idx, p in paths.iteritems():
            out_path = {}

            out_path["start_waypoint_id"] = waypoint_names[p["start"]]
            out_path["end_waypoint_id"] = waypoint_names[p["end"]]
            out_path["next_path_id"] = path_names[p["next"]]
            out_path["type"] = PATH_TYPE_LOOKUP[p.get("type")]
            out_path["flags"] = int(p.get("flags", "0x0"), 16)

            assert out_waypoints[out_path["start_waypoint_id"]]
            assert out_waypoints[out_path["end_waypoint_id"]]

            # For straight paths, update the waypoint yaw values to match the
            # heading between waypoints
            if out_path["type"] in ("R", "L"):
                start_wp = out_waypoints[out_path["start_waypoint_id"]]
                end_wp = out_waypoints[out_path["end_waypoint_id"]]

                d_lat = end_wp["lat"] - start_wp["lat"]
                d_lon = end_wp["lon"] - start_wp["lon"]

                heading = math.atan2(d_lon * math.cos(start_wp["lat"]),
                                     d_lat)
                start_wp["yaw"] = heading
                end_wp["yaw"] = heading

            out_paths[int(idx)] = out_path

        for idx, wp in out_waypoints.iteritems():
            if wp != self.waypoints.get(idx):
                self.pending_changes.append(
                    (plog.FCS_PARAMETER_KEY_WAYPOINT, idx,
                     self._next_state_idx(),
                     plog.pack_waypoint(wp))
                )
                self.waypoints[idx] = wp

        for idx, p in out_paths.iteritems():
            if p != self.paths.get(idx):
                self.pending_changes.append(
                    (plog.FCS_PARAMETER_KEY_PATH, idx,
                     self._next_state_idx(),
                     plog.pack_path(p))
                )
                self.paths[idx] = p

        boundary = []
        if "boundary" in defaults:
            boundary = list(waypoint_names[p.strip()]
                            for p in defaults["boundary"].split(","))

        if boundary != self.boundary:
            # Queue boundary update packet
            self.pending_changes.append(
                (plog.FCS_PARAMETER_KEY_NAV_BOUNDARY, idx,
                 self._next_state_idx(),
                 plog.pack_boundary(boundary))
            )
            self.boundary = boundary

        # Add a reroute to holding pattern at the start to ensure it's not
        # trying to follow a path that's being modified.
        if len(self.pending_changes):
            self.pending_changes.appendleft(
                (plog.FCS_PARAMETER_KEY_REROUTE, 499,
                 self.current_nav_state_id, "0"))

        self.current_config_json = new_config_json

        if len(self.pending_changes):
            return True
        else:
            return False

    def parse_config(self):
        parser = ConfigParser.SafeConfigParser()
        parser.read(self.config_path)

        paths = {}
        waypoints = {}
        defaults = {}

        path_names = {}
        waypoint_names = {}

        for section in parser.sections():
            options = dict(parser.items(section))
            if section.startswith("Path"):
                idx = int(section[len("Path"):])
                if idx in paths:
                    raise KeyError("Duplicate path ID %d" % idx)
                paths[idx] = options
                if options["name"] in path_names:
                    raise KeyError("Duplicate path name %s" % options["name"])
                path_names[options["name"]] = idx
            elif section.startswith("Waypoint"):
                idx = int(section[len("Waypoint"):])
                if idx in waypoints:
                    raise KeyError("Duplicate waypoint ID %d" % idx)
                waypoints[idx] = options
                if isinstance(waypoints[idx]["pos"], basestring):
                    waypoints[idx]["pos"] = list(eval(waypoints[idx]["pos"]))
                if options["name"] in waypoint_names:
                    raise KeyError("Duplicate waypoint name %s" %
                                   options["name"])
                waypoint_names[options["name"]] = idx
            elif section == "default":
                defaults = options

        return self.from_json(json.dumps({
            "paths": paths, "waypoints": waypoints, "defaults": defaults,
            "path_names": path_names, "waypoint_names": waypoint_names
        }))

    def update_waypoint_in_config(self, waypoint_name, values):
        parser = ConfigParser.SafeConfigParser()
        parser.read(self.config_path)

        for section in parser.sections():
            options = dict(parser.items(section))
            if section.startswith("Waypoint") and options["name"] == waypoint_name:
                for key, value in values.iteritems():
                    parser.set(section, key, value)

        with open(self.config_path, "w") as f:
            parser.write(f)

    def update_path_in_config(self, path_name, values):
        parser = ConfigParser.SafeConfigParser()
        parser.read(self.config_path)

        for section in parser.sections():
            options = dict(parser.items(section))
            if section.startswith("Path") and options["name"] == path_name:
                for key, value in values.iteritems():
                    parser.set(section, key, value)

        with open(self.config_path, "w") as f:
            parser.write(f)

    def next_change(self, nav_state_id):
        if self.is_sync_complete():
            return None

        log.info("next_change: current %d, next %d", nav_state_id, self.pending_changes[0][2])

        if nav_state_id == self.pending_changes[0][2]:
            self.pending_changes.popleft()

        if self.pending_changes:
            return self.pending_changes[0]
        else:
            return None

    def is_sync_complete(self):
        return not len(self.pending_changes)
