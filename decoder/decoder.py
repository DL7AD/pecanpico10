#!/usr/bin/python3

import serial,re,io
import sys
import argparse
import telnetlib
import time
import mysql.connector as mariadb
import image
import position

# Parse arguments from terminal
parser = argparse.ArgumentParser(description='APRS/SSDV decoder')
parser.add_argument('-c', '--call', help='Callsign of the station', default='DL7AD')
parser.add_argument('-d', '--device', help='Serial device (\'-\' for stdin)', default='-')
parser.add_argument('-b', '--baudrate', help='Baudrate for serial device', default=9600, type=int)
parser.add_argument('-v', '--verbose', help='Activates more debug messages', action="store_true")
args = parser.parse_args()

# Open SQLite database
db = mariadb.connect(user='decoder', password='decoder', database='decoder')
db.cursor().execute("""
	CREATE TABLE IF NOT EXISTS `position`
	(
		`call` VARCHAR(10),
		`rxtime` INTEGER,
		`org` VARCHAR(3),

		`reset` INTEGER,
		`id` INTEGER,
		`time` INTEGER,

		`adc_vsol` INTEGER,
		`adc_vbat` INTEGER,
		`pac_vsol` INTEGER,
		`pac_vbat` INTEGER,
		`pac_pbat` INTEGER,
		`pac_psol` INTEGER,

		`light_intensity` INTEGER,

		`gps_time` INTEGER,
		`gps_lock` INTEGER,
		`gps_sats` INTEGER,
		`gps_ttff` INTEGER,
		`gps_pdop` INTEGER,
		`gps_alt` INTEGER,
		`gps_lat` INTEGER,
		`gps_lon` INTEGER,

		`sen_i1_press` INTEGER,
		`sen_e1_press` INTEGER,
		`sen_e2_press` INTEGER,
		`sen_i1_temp` INTEGER,
		`sen_e1_temp` INTEGER,
		`sen_e2_temp` INTEGER,
		`sen_i1_hum` INTEGER,
		`sen_e1_hum` INTEGER,
		`sen_e2_hum` INTEGER,

		`stm32_temp` INTEGER,
		`si4464_temp` INTEGER,

		`sys_time` INTEGER,
		`sys_error` INTEGER,
		PRIMARY KEY (`call`,`reset`,`id`,`rxtime`)
	)
""")
db.cursor().execute("""
	CREATE TABLE IF NOT EXISTS `image`
	(
		`id` INTEGER,
		`call` VARCHAR(10),
		`rxtime` INTEGER,
		`imageID` INTEGER,
		`packetID` INTEGER,
		`data` VARCHAR(1024),
		PRIMARY KEY (`call`,`id`,`packetID`)
	)
""")
db.cursor().execute("""
	CREATE TABLE IF NOT EXISTS `directs`
	(
		`call` VARCHAR(10),
		`rxtime` INTEGER,
		`directs` VARCHAR(256),
		PRIMARY KEY (`call`,`rxtime`)
	)
""")


""" Packet handler for received APRS packets"""
def received_data(data):

	data = data.strip()

	# Parse line and detect data
	callreg = "([A-Z]{2}[0-9][A-Z]{1,3}(?:-[0-9]{1,2})?)" # Callregex to filter bad igated packets

	all = re.search("^" + callreg + "\>APECAN(.*?):", data)
	pos = re.search("^" + callreg + "\>APECAN(.*?):[\=|!](.{13})(.*?)\|(.*)\|", data)
	dat = re.search("^" + callreg + "\>APECAN(.*?):\{\{(I|L)(.*)", data)
	dir = re.search("^" + callreg + "\>APECAN(.*?)::(.{9}):Directs=(.*)", data)

	if pos or dat or dir:
		# Debug
		if args.verbose:
			print('='*100)
			print(data)
			print('-'*100)

		call = all.group(1).split(' ')[-1]
		rxer = all.group(2).split(',')[-1]
		if not len(rxer): rxer = args.call

		if pos: # Position packet (with comment and telementry)

			comm = pos.group(4)
			position.insert_position(db, call, comm, 'pos')

		elif dat: # Data packet (Image or Logging)

			typ  = dat.group(3)
			data = dat.group(4)

			if typ is 'I': # Image packet
				image.insert_image(db, rxer, call, data)
			elif typ is 'L': # Log packet
				position.insert_position(db, call, data, 'log')

		elif dir: # Directs packet
			position.insert_directs(db, call, dir.group(4))

if args.device == 'I': # Source APRS-IS

	print('Connect to APRS-IS')
	try:
		tn = telnetlib.Telnet("euro.aprs2.net", 14580, 3)
		tn.write(("user %s filter u/APECAN\n" % args.call).encode('ascii'))
		print('Connected')
	except Exception as e:
		print('Could not connect to APRS-IS: %s' % str(e))
		print('exit...')
		sys.exit(1)

	wdg = time.time() + 10 # Connection watchdog
	while True:
		# Read data
		try:
			buf = tn.read_until(b"\n").decode('charmap')
			print(">> " + buf)
		except EOFError: # Server has connection closed
			print("EOF ERROR")
			wdg = 0 # Tell watchdog to reconnect
		except UnicodeDecodeError:
			print("UNICODE DECODE ERROR")
			pass

		# Watchdog reconnection
		if wdg < time.time():
			print('APRS-IS connection lost... reconnect')
			try:
				tn = telnetlib.Telnet("euro.aprs2.net", 14580, 3)
				tn.write(("user %s filter u/APECAN\n" % args.call).encode('ascii'))
				print('Connected')
				wdg = time.time() + 10
			except Exception as e:
				print('Could not connect to APRS-IS: %s' % str(e))
				print('Try again...')

		if '# aprsc' in buf: # Watchdog reload
			print('Ping from APRS-IS')
			wdg = time.time() + 30
		else: # Data handling
			received_data(buf)

		time.sleep(0.01)

elif args.device is '-': # Source stdin

	while True:
		received_data(sys.stdin.readline())

else: # Source Serial connection

	try:
		serr = serial.Serial(
			port=args.device,
			baudrate=args.baudrate,
		)
	except:
		sys.stderr.write('Error: Could not open serial port\n')
		sys.exit(1)

	while True:
		data = ''
		while True:
			b = serr.read(1)
			if chr(b[0]) == '\r' or chr(b[0]) == '\n':
				break
			data += chr(b[0])

		received_data(data)

