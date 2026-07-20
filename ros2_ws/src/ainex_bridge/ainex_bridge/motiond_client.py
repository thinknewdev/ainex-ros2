#!/usr/bin/env python3
"""Thread-safe newline-delimited JSON client for the motiond Unix socket.

Protocol: one JSON object per line request -> one JSON object per line reply.
The client reconnects transparently on failure and serializes requests with a
lock so it can be shared by all ROS callbacks.
"""
import json
import socket
import threading


class MotiondError(Exception):
    """Raised when a request cannot be completed (I/O or protocol failure)."""


class MotiondClient:
    def __init__(self, socket_path='/tmp/motiond.sock', timeout=2.0, logger=None):
        self._path = socket_path
        self._timeout = timeout
        self._logger = logger
        self._lock = threading.Lock()
        self._sock = None
        self._rfile = None

    # -- connection management -------------------------------------------
    def _connect_locked(self):
        self._close_locked()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self._timeout)
        sock.connect(self._path)
        self._sock = sock
        self._rfile = sock.makefile('r', encoding='utf-8', newline='\n')

    def _close_locked(self):
        for obj in (self._rfile, self._sock):
            if obj is not None:
                try:
                    obj.close()
                except OSError:
                    pass
        self._rfile = None
        self._sock = None

    def close(self):
        with self._lock:
            self._close_locked()

    # -- request/response ------------------------------------------------
    def _roundtrip_locked(self, payload):
        self._sock.sendall(payload)
        line = self._rfile.readline()
        if not line:
            raise ConnectionError('motiond closed the connection')
        return json.loads(line)

    def request(self, msg):
        """Send one request dict, return the reply dict.

        Retries once with a fresh connection if the socket is stale.
        Raises MotiondError on failure.
        """
        payload = (json.dumps(msg) + '\n').encode('utf-8')
        with self._lock:
            for attempt in (0, 1):
                try:
                    if self._sock is None:
                        self._connect_locked()
                    return self._roundtrip_locked(payload)
                except (OSError, ValueError, ConnectionError) as e:
                    self._close_locked()
                    if attempt == 1:
                        raise MotiondError(
                            'motiond request failed ({}): {}'.format(
                                msg.get('op', '?'), e)) from e
                    if self._logger is not None:
                        self._logger.warn(
                            'motiond socket error ({}), reconnecting: {}'.format(
                                msg.get('op', '?'), e))

    def request_ok(self, msg):
        """request() variant returning (ok: bool, reply: dict); never raises."""
        try:
            reply = self.request(msg)
            return bool(reply.get('ok', False)), reply
        except MotiondError as e:
            if self._logger is not None:
                self._logger.error(str(e))
            return False, {}
