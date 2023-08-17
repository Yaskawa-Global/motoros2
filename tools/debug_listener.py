#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
# SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
#
# SPDX-License-Identifier: Apache-2.0

import argparse
import asyncio
import datetime
import os
import platform
import socket
import time
from functools import partial
from typing import Tuple, Union, Text

Address = Tuple[str, int]

# length of the stamp in a log msg: 'YYYY-MM-DD HH:MM:SS.nnnnnn'
# (where 'nnnnnn' is the microsecond part)
STAMP_STR_LEN=26

def main():
    default_bcast_port=21789
    default_local_bind_addr='0.0.0.0'

    parser = argparse.ArgumentParser(
        description='Receives MotoROS2 debug messages and prints them to '
            'the console and logs them to a file (unless configured not to).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--port', dest='listen_port',
        default=default_bcast_port,
        help='UDP port to listen on')
    parser.add_argument('-b', '--bind', dest='listen_ip',
        default=default_local_bind_addr,
        help='IP address to listen on')
    parser.add_argument('-n', '--no-file-sync', action='store_true',
        help='Do not log to files, only to console')
    args = parser.parse_args()

    # loop and prot
    loop = asyncio.get_event_loop()
    prot = DebugBroadcastProtocol(loop=loop)

    # always register console sink
    prot.register_sink_cb(cb=console_sink_cb)
    print('Listening for MotoROS2 debug msgs (format: "[time_msg_rcvd] [ip:port] msg") ..')
    print('---')

    # if not configured otherwise, register file cb
    if not args.no_file_sync:
        stamp = time.strftime('%Y%m%dT%H%M%S')
        fname = f"{stamp}_motoros2_debug_log.txt"
        # leaks f, but will be closed at program exit
        f = open(fname, 'w')
        prot.register_sink_cb(cb=partial(file_sink, f=f))

    try:
        # run it
        loop.run_until_complete(
            loop.create_datagram_endpoint(
                lambda: prot,
                local_addr=(args.listen_ip, args.listen_port),
                reuse_port=True if platform.system().lower() == 'linux' else False,
                # this is also needed for receiving broadcasts
                allow_broadcast=True))
        loop.run_forever()
    except KeyboardInterrupt:
        pass

    # convenience: if zero bytes logged, delete output file
    if not args.no_file_sync and f.tell() == 0:
        print("\nNo messages received, deleting empty log file")
        f.close()
        os.remove(path=fname)


def file_sink(f, msg, stamp, source_addr):
    ip, port = source_addr
    print(f'[{stamp}] [{ip}:{port}]: {msg}', file=f)
    f.flush()


def console_sink_cb(msg, stamp, source_addr):
    ip, port = source_addr
    print(f'[{stamp}] [{ip}:{port}]: {msg}')


class DebugBroadcastProtocol(asyncio.DatagramProtocol):
    def __init__(self, *, loop: asyncio.AbstractEventLoop = None):
        self.loop = asyncio.get_event_loop() if loop is None else loop
        self._sink_cbs = []

    def register_sink_cb(self, cb):
        self._sink_cbs.append(cb)

    def connection_made(self, transport: asyncio.transports.DatagramTransport):
        self.transport = transport

    def datagram_received(self, data: Union[bytes, Text], addr: Address):
        # note: we assume all sinks appreciate strings instead of raw data
        msg = data.decode('ascii')
        sent_stamp = msg[:STAMP_STR_LEN]
        msg = msg[STAMP_STR_LEN+1:]
        source_addr = (addr[0], addr[1])
        self._write_to_sinks(msg=msg, stamp=sent_stamp, source_addr=source_addr)

    def _write_to_sinks(self, msg, stamp, source_addr):
        for cb in self._sink_cbs:
            cb(msg=msg, stamp=stamp, source_addr=source_addr)


if __name__ == '__main__':
    main()
