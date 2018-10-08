import telnetlib
import time
import socket

class AprsIS(object):

	__instance = None

	wdg = None # Connection Watchdog
	tn = None # Telnet connection

	def __new__(cls):
		if AprsIS.__instance is None:
			AprsIS.__instance = object.__new__(cls)
		return AprsIS.__instance

	def __init__(self, ):
		self.connect()

	def __del__(self):
		self.disconnect()

	def resetWatchdog(self):
		self.wdg = time.time() + 10

	def hasWatchdogTimeout(self):
		return self.wdg < time.time()

	def connect(self):

		print('Connect to APRS-IS')

		while True:
			try:
				self.tn = telnetlib.Telnet("euro.aprs2.net", 14580, 3)
				self.tn.write(("user N0CALL filter u/APECAN filter t/p\n").encode('ascii'))
				print('Connected')
				self.resetWatchdog()
				return

			except Exception as e:
				print('Could not connect to APRS-IS: %s' % str(e))
				print('Try again in 5 seconds...')
				time.sleep(5)

	def disconnect(self):
		self.tn.get_socket().shutdown(socket.SHUT_WR)
		self.tn.read_all()
		self.tn.close()

	def reconnect(self):
		self.disconnect()
		self.connect()

	def getData(self):

		# Read data
		try:
			buf = self.tn.read_until(b"\n").decode('charmap')
		except EOFError: # Server has connection closed
			print('APRS-IS connection lost (EOFError)... reconnect')
			self.reconnect()
			return
		except UnicodeDecodeError:
			return

		# Filter non ASCII packets (bad packets)
		if not all(ord(c) < 128 for c in buf):
			return None

		# Watchdog reconnection
		if self.hasWatchdogTimeout():
			print('APRS-IS connection lost (Watchdog)... reconnect')
			self.reconnect()

		self.resetWatchdog()

		if '# aprsc' in buf: # Watchdog reload
			print('Ping from APRS-IS')
			return None
		else: # Data handling
			return buf

