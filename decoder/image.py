import binascii
import urllib.request
import urllib.error
from subprocess import *
import time
import threading
from shutil import copyfile
import base91
from terminal import Terminal

cache = {}
timeout = 600 # 10 minutes

def ssdv_decode_callsign(code):
	callsign = ''

	while code > 0:
		s = code % 40
		if   s == 0: callsign += '-'
		elif s < 11: callsign += chr(47 + s)
		elif s < 14: callsign += '-'
		else: callsign += chr(51 + s)
		code /= 40

	return callsign

def ssdv_encode_callsign(callsign):
	x = 0
	for i in range(len(callsign)-1,-1,-1):
		x *= 40
		c = ord(callsign[i])
		if   c >= 65 and c <= 90:  x += c - 51
		elif c >= 97 and c <= 122: x += c - 83
		elif c >= 48 and c <= 57:  x += c - 47

	return x

imageProcessor = None
imageData = {}
lock = threading.RLock()

def imgproc():
	global imageData

	while True:
		with lock:
			for _id in imageData:
				(call, data) = imageData[_id]

				filename = 'html/images/%s-%d.jpg' % (call.replace('-',''), _id)
				f = open(filename, 'wb')
				process = Popen(['./ssdv/ssdv', '-d'], stdin=PIPE, stdout=f, stderr=PIPE)
				process.stdin.write(data)
				dummy,err = process.communicate()
				f.close()

				filename2 = 'html/images/%s.jpg' % (call.replace('-',''))
				copyfile(filename, filename2)

			imageData = {} # Clear data

		time.sleep(1)

def insert_image(db, call, data_b91, rxtime):
	global imageProcessor,imageData,w,timeout,cache

	data = base91.decode(data_b91)
	if len(data) != 174: # APRS message has invalid type or length (or both)
		return {'type': 'img', 'error': 'Invalid data: message too short'}

	cur = db.cursor()

	# Decode various meta data
	imageID  = data[0]
	packetID = (data[1] << 8) | data[2]
	data = binascii.hexlify(data[3:]).decode("ascii")

	# Encode callsign (ensure callsign has no more than 6 chars)
	bcall = call.split('-') # Split callsign and SSID
	if len(bcall) == 1: # No SSID available, so take the callsign
		bcall = bcall[0][0:6]
	elif(len(bcall[0]) < 5): # Callsign has 4 chars, so take it with the SSID
		bcall = bcall[0] + bcall[1][0:2]
	elif(len(bcall[0]) < 6): # Callsign has 5 chars, so take it with the last digit of the SSID
		bcall = bcall[0] + bcall[1][-1]
	else:
		bcall = bcall[0][0:6] # Callsign has 6 chars, so take the call without SSID

	data  = ('68%08x%02x%04x' % (ssdv_encode_callsign(bcall), imageID, packetID)) + data
	data += "%08x" % (binascii.crc32(binascii.unhexlify(data)) & 0xffffffff)

	# Find image ID (or generate new one)
	serverID = None
	
	# Search Server ID cache
	for k in cache:
		e = cache[k]
		if e['call'] == call and e['imageID'] == imageID and e['time']+timeout >= rxtime:
			serverID = e['serverID'] # Found Server ID in cache

	# Search Server ID in Database
	if serverID == None:
		cur.execute("""
			SELECT `id`,`packetID`
			FROM `image`
			WHERE `call` = %s
			AND `imageID` = %s
			AND `rxtime`+%s >= %s
			ORDER BY `rxtime`
			DESC LIMIT 1""",
			(call, imageID, str(timeout), str(rxtime))
		)
		fetch = cur.fetchall()
		if len(fetch):
			serverID = fetch[0][0]

	# Generate new Server ID if no ID found
	if serverID is None:
		# Generate ID
		cur.execute("SELECT `id`+1 FROM `image` ORDER BY `id` DESC LIMIT 1")
		fetch = cur.fetchall()
		if len(fetch):
			serverID = fetch[0][0]
		else: # No entries in the database
			serverID = 0

		serverIDcreated = True
	else:
		serverIDcreated = False

	# Debug
	Terminal().info('Received image packet Call=%s ImageID=%d PacketID=%d ServerID=%d' % (call, imageID, packetID, serverID))

	# Insert into database
	cur.execute("""
		INSERT IGNORE INTO `image` (`call`,`rxtime`,`imageID`,`packetID`,`data`,`id`)
		VALUES (%s,%s,%s,%s,%s,%s)""",
		(call, str(rxtime), imageID, packetID, data, serverID)
	)

	with lock:

		if serverID not in cache:
			# Create cache
			cache[serverID] = {
				'call': call,
				'imageID': imageID,
				'serverID': serverID,
				'data': [],
				'time': time.time()
			}

			if not serverIDcreated: # Packets already in database
				cur.execute("SELECT `packetID`,`data` FROM `image` WHERE `id` = %s", (serverID,))
				for sql_data in cur.fetchall():
					cache[serverID]['data'].append(sql_data)
		else:
			# Append to cache
			cache[serverID]['data'].append([packetID, data])
			cache[serverID]['time'] = time.time()

		# From now on only work on cache

		allData = ''
		for dummy,data in sorted(cache[serverID]['data'], key=lambda x: x[0]):
			allData += '55' + data + (144*'0')
		imageData[serverID] = (call, binascii.unhexlify(allData))

	# Start decoder thread if it hasn't already been done before
	if imageProcessor is None:
		imageProcessor = threading.Thread(target=imgproc)
		imageProcessor.start()

	# Clear up cache (timeout)
	for k in cache:
		if cache[k]['time']+timeout < time.time():
			del cache[k]
			break


	return {'type': 'img', 'imageID': imageID, 'packetID': packetID, 'serverID': serverID}
