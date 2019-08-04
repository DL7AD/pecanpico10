from datetime import datetime,timezone

class Terminal(object):

	LEVEL_NONE  = 0
	LEVEL_ERROR = 1
	LEVEL_WARN  = 2
	LEVEL_INFO  = 3
	LEVEL_DEBUG = 4

	__instance = None
	verbosity = LEVEL_INFO
	lastTime = 0

	def __new__(self, verbosity=None):
		if Terminal.__instance is None:
			Terminal.__instance = object.__new__(self)
			if verbosity != None:
				Terminal.__instance.verbosity = verbosity
		return Terminal.__instance

	def error(self, str):
		if self.verbosity >= self.LEVEL_ERROR:
			print(str)

	def warn(self, str):
		if self.verbosity >= self.LEVEL_WARN:
			print(str)

	def info(self, str):
		if self.verbosity >= self.LEVEL_INFO:
			print(str)

	def debug(self, str):
		if self.verbosity >= self.LEVEL_DEBUG:
			print(str)

	def packet_apecan(self, rxtime, raw, decoded):
		if self.verbosity >= self.LEVEL_NONE and rxtime != self.lastTime:
			dt = datetime.fromtimestamp(rxtime, timezone.utc)
			print(('='*10) + ' ' + dt.strftime("%Y-%m-%d %H:%M:%S") + ' UTC ' + ('='*65))
			self.lastTime = rxtime

		if self.verbosity >= self.LEVEL_DEBUG:
			print(raw)
			print(decoded)
			print('-'*100)

	def packet_ext_info(self, rxtime, raw, decoded):
		pass

	def packet_ext_error(self, rxtime, raw, decoded):
		pass
