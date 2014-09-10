import os
import re
import math
import struct

COORD_RE = re.compile("^([SN])([0-9]+)([EW])([0-9]+)$")

class Heightmap(object):
    def __init__(self, path):
        self.path = path
        self.data = open(path, 'rb').read()

        coord = os.path.split(path)[1].partition('.')[0]
        ns, lat, ew, lon = COORD_RE.match(coord).groups()

        self.start_lat = float(lat) * (-1 if ns == "S" else 1)
        self.start_lon = float(lon) * (-1 if ew == "W" else 1)

        self.end_lat = self.start_lat + 1.0
        self.end_lon = self.start_lon + 1.0

    def _raw_lookup(self, row, col):
        i = row * 1201 + col

        elevation = (ord(self.data[i * 2]) << 8) + ord(self.data[i * 2 + 1])
        if elevation >= 32768:
            return None
        else:
            return elevation

    def lookup(self, lat, lon):
        if not (self.start_lat <= lat <= self.end_lat) or \
                not (self.start_lon <= lon <= self.end_lon):
            raise ValueError("Point ({}, {}) out of range".format(lat, lon))

        # First row is northernmost
        row_f = (self.end_lat - lat) * 1200.0
        col_f = (lon - self.start_lon) * 1200.0
        row_i = int(row_f)
        col_i = int(col_f)

        assert row_i > 0 and row_i < 1201
        assert col_i > 0 and col_i < 1201

        # Look up corner elevations for bilinear interpolation
        nw_h = self._raw_lookup(row_i, col_i)
        sw_h = self._raw_lookup(row_i + 1, col_i)
        ne_h = self._raw_lookup(row_i, col_i + 1)
        se_h = self._raw_lookup(row_i + 1, col_i + 1)

        # Fill in missing data with another point
        if not nw_h:
            nw_h = sw_h or ne_h or se_h
        if not sw_h:
            sw_h = nw_h or se_h or ne_h
        if not ne_h:
            ne_h = se_h or nw_h or sw_h
        if not se_h:
            se_h = ne_h or sw_h or nw_h

        if not nw_h or not sw_h or not ne_h or not se_h:
            raise ValueError(
                ("Elevation for point ({}, {}) unable to be "
                 "looked up; all adjacent data missing").format(lat, lon))

        # Run bilinear interpolation:
        # b1 + b2 * x + b3 * y + b4 * x * y
        # where b1 = f(0, 0)
        #       b2 = f(1, 0) - f(0, 0)
        #       b3 = f(0, 1) - f(0, 0)
        #       b4 = f(0, 0) - f(1, 0) - f(0, 1) + f(1, 1)
        result = nw_h + \
                 (sw_h - nw_h) * (row_f - row_i) + \
                 (ne_h - nw_h) * (col_f - col_i) + \
                 (nw_h - sw_h - ne_h + sw_h) * (row_f - row_i) * (col_f - col_i)

        return result

