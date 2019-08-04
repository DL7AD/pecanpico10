#!/usr/bin/python3

import re,io,os,sys,time
import argparse
import mysql.connector as mariadb
import image
import position
import json
from datetime import datetime,timezone
from subprocess import *
from terminal import Terminal

# Parse arguments from terminal
parser = argparse.ArgumentParser(description='APRS/SSDV decoder')
parser.add_argument('-c', '--call', help='Callsign of the station', default='DL7AD')
parser.add_argument('-d', '--device', help='Serial device (\'-\' for stdin)', default='-')
parser.add_argument('-b', '--baudrate', help='Baudrate for serial device', default=9600, type=int)
parser.add_argument('-v', '--verbose', help='Activates more debug messages', type=int, default=Terminal.LEVEL_INFO)
args = parser.parse_args()

# Open SQLite database
db = mariadb.connect(user='decoder', password='decoder', database='decoder')

# Create all tables
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
db.cursor().execute("""
	CREATE TABLE IF NOT EXISTS `raw`
	(
		`call` VARCHAR(10),
		`rxtime` INTEGER,
		`data` VARCHAR(1024),
		`meta` VARCHAR(1024)
	)
""")

db.cursor().execute("""
	CREATE TABLE IF NOT EXISTS `location`
	(
		`call` VARCHAR(10),
		`rxtime` INTEGER,
		`lat` FLOAT,
		`lon` FLOAT,
		`packets` INTEGER DEFAULT 1,
		PRIMARY KEY (`call`)
	)
""")

# Create printer
Terminal(args.verbose)

last_db_commit = time.time()
def db_commit():
	global last_db_commit
	if last_db_commit+1 < time.time():
		db.commit()
		last_db_commit = time.time()

""" Packet handler for received APRS packets"""
def received_data(data):

	rawdata = data.strip()
	term = Terminal(args.verbose)

	# Callregex to filter bad igated packets and packets which couldn't have been transmitted
	# with the APRS protocol (because they don't fit in the APRS protocol)
	callreg = "((?:[A-Z][A-Z0-9]?|[A-Z0-9][A-Z])[0-9][A-Z]{1,3}(?:-[0-9]{1,2})?)>"

	# Callsign
	re_call = re.search("^" + callreg, rawdata)

	# APECAN packets
	re_all = re.search("^" + callreg + "APECAN(.*?):", rawdata)
	re_pos = re.search("^" + callreg + "APECAN(.*?):[!=](.{13})(.*?)\|(.*)\|", rawdata)
	re_dat = re.search("^" + callreg + "APECAN(.*?):\{\{(I|L)(.*)", rawdata)
	re_dir = re.search("^" + callreg + "APECAN(.*?)::(.{9}):Directs=(.*)", rawdata)

	# Regular position packets
	re_loc_uncomp_no_time = re.search("^" + callreg + ".*?:[!=]([0-9]{4}.[0-9]{2})([SN]).([0-9]{5}.[0-9]{2})([WE]).*|", rawdata)
	re_loc_uncomp_wi_time = re.search("^" + callreg + ".*?:[@\/][0-9]{6}[zh]([0-9]{4}.[0-9]{2})([SN]).([0-9]{5}.[0-9]{2})([WE]).*|", rawdata)

	# Log time
	rxtime = int(datetime.now(timezone.utc).timestamp())

	if re_pos or re_dat or re_dir: # Is recognized APECAN packet

		call = re_all.group(1).split(' ')[-1]

		meta = {}

		if re_all:

			# Position packet (with comment and telementry)
			if re_pos:

				comm = re_pos.group(4)
				meta = position.insert_position(db, call, comm, 'pos', rxtime)

			# Data packet (Image or Logging)
			elif re_dat:

				typ  = re_dat.group(3)
				data = re_dat.group(4)

				if typ is 'I': # Image packet
					meta = image.insert_image(db, call, data, rxtime)
				elif typ is 'L': # Log packet
					meta = position.insert_position(db, call, data, 'log', rxtime)

			# Directs packet
			elif re_dir:
				meta = position.insert_directs(db, call, re_dir.group(4), rxtime)

			Terminal().packet_apecan(rxtime, rawdata, meta)

		else:
			Terminal().packet_apecan(rxtime, rawdata, 'Unrecognized APECAN packet')

		# Insert into raw database
		db.cursor().execute("""
			INSERT INTO `raw` (`call`,`rxtime`,`data`,`meta`)
			VALUES (%s,%s,%s,%s)""",
			(call, str(rxtime), rawdata, json.dumps(meta))
		)

	elif (re_loc_uncomp_no_time and re_loc_uncomp_no_time.group(0) != '') or (re_loc_uncomp_wi_time and re_loc_uncomp_wi_time.group(0) != ''):

		re_un = re_loc_uncomp_no_time if re_loc_uncomp_no_time.group(0) != '' else re_loc_uncomp_wi_time

		call = re_un.group(1)
		try:
			lat = (1 if re_un.group(3) == 'N' else -1) * round(float(re_un.group(2)) / 100, 4)
			lon = (1 if re_un.group(5) == 'E' else -1) * round(float(re_un.group(4)) / 100, 4)

			# Debug
			Terminal().packet_ext_info(rxtime, rawdata, 'Position call=%s lat=%f lon=%f' % (call, lat, lon))

			db.cursor().execute("""
				INSERT INTO `location` (`call`,`rxtime`,`lat`,`lon`)
				VALUES (%s,%s,%s,%s)
				ON DUPLICATE KEY UPDATE
				`rxtime`=%s, `lat`=%s, `lon`=%s, packets=packets+1""",
				(call, str(rxtime), lat, lon, str(rxtime), lat, lon)
			)

		except ValueError:
			Terminal().packet_ext_error(rxtime, rawdata, 'Value error in packet')

	else:

		if re_call == None:
			# Invalid callsign: Must comply with APRS e.g. DL7AD-C => Inalid SSID
			Terminal().packet_ext_error(rxtime, rawdata, 'Invalid APRS callsign: ' + rawdata.split('>')[0])
		else:
			Terminal().packet_ext_error(rxtime, rawdata, 'Unknown format')

if args.device == 'I': # Source APRS-IS

	from aprsis import AprsIS
	conn = AprsIS()
	while True:
		data = conn.getData()
		if data is not None:
			received_data(data)

		db_commit()

elif args.device is '-': # Source stdin

	while True:
		received_data(sys.stdin.readline())
		db_commit()

else: # Source Serial connection

	try:
		import serial
		serr = serial.Serial(
			port=args.device,
			baudrate=args.baudrate,
		)
	except ImportError:
		sys.stderr.write('Error: Module \'serial\' not installed\n')
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
		db_commit()
