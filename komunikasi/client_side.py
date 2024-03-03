#!/usr/bin/env python3
import socket


class Client:
    def __init__(self, host:str):
        self._connected = False
        self._message_callback = None
        self.host = host
        self._client_socket = None

    def connect(self):
        if not self._connected:
            try:
                self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._client_socket.connect((self.host, 2003))
                self._connected = True
                if self._message_callback:
                        self._message_callback('Tersambung')
                self._receive_messages()
            except socket.timeout:
                self._handle_disconnect()
            except ConnectionResetError:
                self._handle_disconnect()
            except Exception:
                self._handle_disconnect()

    # Other
    def _receive_messages(self):
        while self._connected:
            try:
                _msg = self._client_socket.recv(1024)
                if not _msg:
                    self._handle_disconnect()
                    break
                if self._message_callback:
                    self._message_callback(_msg.decode('utf-8'))
            except Exception:
                self._handle_disconnect()

    def disconnect(self):
        if self._connected:
            try:
                if self._message_callback:
                    self._message_callback('k')
            except Exception as e:
                print(e)
            finally:
                self._client_socket.close()
                self._connected = False

    def _handle_disconnect(self):
        self.disconnect()
        self.connect()

    # Setter
    def set_message_callback(self, callback) -> str:
        self._message_callback = callback