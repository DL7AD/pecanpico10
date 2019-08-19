import telnetlib
import time
import socket
from terminal import Terminal

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

		Terminal().info('Connect to APRS-IS')

		while True:
			try:
				self.tn = telnetlib.Telnet("rotate.aprs2.net", 14580, 3)
				self.tn.write(("user N0CALL filter u/APECAN filter t/p\n").encode('ascii'))
				Terminal().info('Connected')
				self.resetWatchdog()
				return

			except Exception as e:
				Terminal().error('Could not connect to APRS-IS: %s' % str(e))
				Terminal().info('Try again in 5 seconds...')
				time.sleep(5)

	def disconnect(self):
		try:
			self.tn.get_socket().shutdown(socket.SHUT_WR)
			self.tn.read_all()
			self.tn.close()
		except:
			Terminal().error('Error raised at disconnection')

	def reconnect(self):
		self.disconnect()
		self.connect()

	def getData(self):

		# Read data
		try:
			buf = self.tn.read_until(b"\n").decode('charmap')
		except EOFError: # Server has connection closed
			Terminal().error('APRS-IS connection lost (EOFError)... reconnect')
			self.reconnect()
			return
		except UnicodeDecodeError:
			return

		# Filter non ASCII packets (bad packets)
		if not all(ord(c) < 128 for c in buf):
			return None

		# Watchdog reconnection
		if self.hasWatchdogTimeout():
			Terminal().error('APRS-IS connection lost (Watchdog)... reconnect')
			self.reconnect()

		self.resetWatchdog()

		if '# aprsc' in buf: # Watchdog reload
			Terminal().debug('Ping from APRS-IS')
			return None
		else: # Data handling
			return buf

