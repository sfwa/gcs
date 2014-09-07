# coding=utf-8

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
import re
import sys
import zlib
import struct
import serial
import binascii
import functools
from cobs import cobsr
from gcs import plog


def split_stream(stream):
    '''
    Split a data stream into packets delimited by NULs, and return a tuple
    containing a list of packets and a remainder string.

    An incomplete packet at the beginning of the string will be ignored.
    '''
    packets = []
    current_packet = None
    tail = ""

    for ch in stream:
        if current_packet:
            if ch == "\x00" and current_packet.startswith("\x00"):
                # Found the terminating character for the current packet, so
                # append it and start a new one
                current_packet += ch
                # Don't bother saving the packet if it's too short to be valid
                if len(current_packet) > 2:
                    packets.append(current_packet)
                current_packet = None
                tail = ""
            else:
                # Not a terminating character, just append it
                current_packet += ch
        else:
            if ch == "\x00":
                # Initial character of a new packet, so start that now
                current_packet = ch
            else:
                tail += ch

    #print repr(current_packet or tail)

    return (packets, current_packet or tail)


def parse_packet(packet):
    if len(packet) < 4:
        return None, None

    #print "Receiving: " + binascii.b2a_hex(packet)

    if len(packet) < 18:
        packet = cobsr.decode(packet.strip("\x00"))
        # Telemetry packet
        try:
            pk_fields = struct.unpack("<BBBBBHH", packet[3:])
            pk_field_names = ("RSSI", "Remote RSSI", "TX Buffer Length",
                              "Noise", "Remote Noise", "RX Errors",
                              "RX Errors Fixed")
            pk_data = dict(zip(pk_field_names, pk_fields))

            return ("rfd900", pk_data)
        except Exception:
            #print "Failed to parse packet: " + binascii.b2a_hex(packet) + "\n"
            #raise
            return (None, None)
    else:
        # State packet
        try:
            log_data = plog.ParameterLog.deserialize(packet)

            pk_data = dict(
                (p.parameter_type.name, p.values) for p in log_data
                if isinstance(p, plog.DataParameter))

            refp = log_data.find_by(
                device_id=0,
                parameter_type=plog.ParameterType.FCS_PARAMETER_KEY_VALUE)
            if refp and refp.key == plog.FCS_PARAMETER_KEY_WAYPOINT:
                pk_data["_reference_waypoint"] = plog.extract_waypoint(refp.value)

            return ("state", pk_data)
        except Exception:
            print "Failed to parse packet: " + repr(packet)
            #raise
            return (None, None)


class RadioConn(object):
    def __init__(self, device, baudrate=57600):
        self.conn = serial.Serial(device, baudrate, timeout=0)
        self.stream = ""
        self._read_handler = self._get_bytes_sync

    def _get_bytes_sync(self):
        return self.conn.read(1)

    def _get_bytes_async(self):
        return os.read(self.conn.fd, 1024)

    def _ioloop_event_handler(self, fd, events, callback=None):
        self.recv(callback=callback)

    def add_to_ioloop(self, ioloop, callback=None):
        '''
        Installs appropriate read handlers into an IOLoop, and invokes
        callback for any packets read.
        '''
        self._read_handler = self._get_bytes_async
        ioloop.add_handler(
            self.conn.fd,
            functools.partial(self._ioloop_event_handler, callback=callback),
            ioloop.READ)

    def recv(self, callback=None):
        '''
        Read any available data from the radio port, and either return a list
        of packets (if callback is None) or invoke callback once for each
        packet with the packet and this object as a parameter.
        '''
        bytes = self._read_handler()
        if not bytes:
            if callback:
                return
            else:
                return []

        self.stream += bytes
        packets, self.stream = split_stream(self.stream)

        # Assemble a list of (type, data) tuples containing only the valid
        # packets we just received.
        out_packets = []
        for pk in packets:
            pk_type, pk_data = parse_packet(pk)
            if pk and pk_type:
                out_packets.append((pk_type, pk_data))

        if callback:
            for pk in out_packets:
                callback(pk, self)
        else:
            return out_packets

    def send(self, sentence):
        '''
        Format and send a sentence (e.g. "$PSWFAC,1234"). Calculate the CRC32
        and checksum, then complete the framing.
        '''

        self.conn.write(sentence)

        print "Transmitting: " + repr(sentence)

        return sentence


if __name__ == "__main__":
    radio = RadioConn(sys.argv[1])
    i = 0
    while True:
        packets = radio.recv()
        for packet in packets:
            if packet[0] == PACKET_TYPE_RADIO_V1:
                data = packet[1]
                # per http://code.google.com/p/ardupilot-mega/wiki/3DRadio
                data["RSSI"] = float(data["RSSI"]) / 1.9 - 127.0
                data["Remote RSSI"] = float(data["Remote RSSI"]) / 1.9 - 127.0
                data["Noise"] = float(data["Noise"]) / 1.9 - 127.0
                data["Remote Noise"] = float(data["Remote Noise"]) / 1.9 - 127.0
                print data
                radio.send(PACKET_TYPE_FILE_INIT_ACK_V1, {"Transfer ID": i})

                i = (i + 1) % 256
            else:
                print packet
