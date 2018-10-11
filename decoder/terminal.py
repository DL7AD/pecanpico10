class Terminal(object):

	LEVEL_NONE  = 0
	LEVEL_ERROR = 1
	LEVEL_WARN  = 2
	LEVEL_INFO  = 3
	LEVEL_DEBUG = 4

	__instance = None
	verbosity = None

	def __new__(cls, verbosity):
		if Terminal.__instance is None:
			Terminal.__instance = object.__new__(cls)
		Terminal.__instance.verbosity = verbosity
		return Terminal.__instance

	def error(self, str):
		if self.verbosity >= LEVEL_ERROR:
			print(str)

	def warn(self, str):
		if self.verbosity >= LEVEL_WARN:
			print(str)

	def info(self, str):
		if self.verbosity >= LEVEL_INFO:
			print(str)

	def debug(self, str):
		if self.verbosity >= LEVEL_DEBUG:
			print(str)

	def packet_apecan(self, rxtime, raw, decoded):
		print(('='*10) + ' ' + str(rxtime) + ' ' + ('='*78))
		print(raw)
		print('-'*100)
		print(decoded)

	def packet_ext_info(self, rxtime, raw, decoded):
		pass

	def packet_ext_error(self, rxtime, raw, decoded):
		pass

