#!/usr/bin/env python3
#
# Simple pyprofibus dummy example using dummy PHY.
# This example can be run without any PB hardware.
#

import argparse
import pyprofibus
import time
import struct
import itertools
import paho.mqtt.client as mqtt
import json

decoders = {
	0xA101:
	[
	('mill_current', 	'H', 'Actual value mill current', 'A'),
	('rot_speed', 		'H', 'Actual value feeder rotational speed in %', '%'),
	('t_mixer', 		'H', 'Actual value product temperature mixer outlet', '°C'),
	('rate_l1', 		'H', 'Actual value liquid 1', 'kg/h'),
	('rate_l2', 		'H', 'Actual value liquid 2', 'kg/h'),
	('rate_l3', 		'H', 'Actual value liquid 3', 'kg/h'),
	('gap_roll_adj', 	'H', 'Actual value gap roll adjustment', '1/10mm'),
	('phase', 			'B', 	'Phase in operating state 3 (only with retentioner)', ''),
	('state', 			'B', """Operating state of the DFBB
0 = Idle position/pre-start
1 = Start-up
2 = Feed readiness/Feed interruption
3 = Feed start-up
4 = Operation
5 = Shutdown""", ''),
	('hygien_mode', 	'B', '0=Normal mode; 1=Hygienisation mode', ''),
	('mode', 			'B', """Operating mode of the DFBB
0 = Automatic mode local
1 = Automatic mode Control system/Remote control
2 = Maintenance mode""", ''),
	(None, 				'H', 'Position slide gate counter current cooler', ''),
	('rate_l4', 		'H', 'Actual value liquid 4', 'kg/h'),
	('amount', 			'L', 'Pressed flour amount', 'kg'),
	('feed_capacity', 	'L', 'Actual value feed capacity ', 'kg/h'),
	],

	0xA105:
	[
	('t_mill_inlet', 	'H', 'Actual value Temperature mill inlet', '°C'),
	('t_mill_cavity', 	'H', 'Actual value Temperature mill cavity', '°C'),
	('t_cooler_inlet', 	'H', 'Actual value Temperature cooler inlet', '°C'),
	('t_cooler_exhaust', 'H', 'Actual value Temperature cooler exhaust hood', '°C'),
	(None, 				'H', 'Reserved', ''),
	('t_heating_mixer', 'H', 'Actual value Temperature heating mats mixer', '°C'),
	('t_heating_ret1', 	'H', 'Actual value Temperature heating mats retentioner 1', '°C'),
	('t_heating_ret2', 	'H', 'Actual value Temperature heating mats retentioner 2', '°C'),
	('steam_rate', 		'H', 'Actual value Steam quantity', 'kg/h'),
	('t_product', 		'H', 'Actual value Product temperature retentioner', '°C'),
	('amount_l1', 		'H', 'Fed liquid quantity 1', 'kg'),
	('amount_l2', 		'H', 'Fed liquid quantity 2', 'kg'),
	('amount_l3', 		'H', 'Fed liquid quantity 3', 'kg'),
	('amount_l4', 		'H', 'Fed liquid quantity 4', 'kg'),
	('t_mixer_inlet', 	'H', 'Actual value product temperature mixer inlet', '°C'),
	],

	0xA201:
	[

	('set_mill_current','H', 'Nominal value mill current', 'A'),
	('set_rot_speed1', 	'H', 'Nominal value feeder rotational speed 1', '%'),
	('set_rot_speed2', 	'H', 'Nominal value feeder rotational speed 2', '%'),
	('set_rot_speed3', 	'H', 'Nominal value feeder rotational speed 3', '%'),
	('set_t_mixer', 	'H', 'Nominal value Product temperature mixer', '°C'),
	('set_l1', 			'H', 'Nominal value Liquid 1', '1/1000'),
	('set_l2', 			'H', 'Nominal value Liquid 2', '1/1000'),
	('set_l3', 			'H', 'Nominal value Liquid 3', '1/1000'),
	('set_gap_rool_adj','H', "Nominal value Working gap mill adjustment", '1/10mm'),
	('set_param', 		'H', 'Parameter set', ''),
	(None, 				'H', 'Throughput Counter current cooler DFKG', ''),
	('set_ret_time', 	'H', 'Nominal value retention time', 's'),
	('set_density', 	'H', 'Bulk density mash', 'kg/m3'),
	('set_max_rate', 	'L', 'Maximum flour throughput', 'kg/h'),
	]
}

requests = [0xA101, 0xA105, 0xA201]

def decode_answer(msg):
	id = struct.unpack('>H', msg[0:2])[0]
	dec = decoders[id]

	# sanity check
	exp_len = 2 + sum([struct.calcsize(fmt) for jsonfield, fmt, desc, unity in dec])
	act_len = len(msg)
	if exp_len != act_len:
		print("Expected msg to be {} bytes long, received {}".format(exp_len, act_len))


	width = 40
	res_txt = []
	res_json = {}
	offset = 2
	for jsonfield, fmt, desc, unit in dec:
		val = struct.unpack_from('>'+fmt, msg, offset)[0]
		offset += struct.calcsize(fmt)
		entry = "{:_<42}: {:6d} [{:<6}]".format(desc, val, unit)
		res_txt.append(entry)
		if jsonfield:
			res_json[jsonfield] = val

	return res_txt, res_json

def on_connect():
	print("Connected!")

def on_disconnect():
	print("Disconencted!")



def main(watchdog=None):
	master = None
	client = None
	parser = argparse.ArgumentParser()
	parser.add_argument("-i", "--interactive", help="Interactive output to stderr", action="store_true")
	args = parser.parse_args()

	try:
		# Parse the config file.
		config = pyprofibus.PbConf.fromFile("dfbb.conf")

		# Create a DP master.
		master = config.makeDPM()

		# Create the slave descriptions.
		outData = {}
		for slaveConf in config.slaveConfs:
			slaveDesc = slaveConf.makeDpSlaveDesc()

			# Set User_Prm_Data
			dp1PrmMask = bytearray((pyprofibus.dp.DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE,
						pyprofibus.dp.DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,
						0x00))
			dp1PrmSet  = bytearray((pyprofibus.dp.DpTelegram_SetPrm_Req.DPV1PRM0_FAILSAFE,
						pyprofibus.dp.DpTelegram_SetPrm_Req.DPV1PRM1_REDCFG,
						0x00))
			slaveDesc.setUserPrmData(slaveConf.gsd.getUserPrmData(dp1PrmMask=dp1PrmMask,
									      dp1PrmSet=dp1PrmSet))


			# Register the slave at the DPM
			master.addSlave(slaveDesc)

			# Set initial output data.
			outData[slaveDesc.name] = bytearray((0xA2, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
			))

		print("Connecting to MQTT broker")
		client = mqtt.Client()
		client.loop_start()
		#client.message_callback_add('$SYS/broker/load/messages/received/1min', messages_received)
		client.on_connect = on_connect
		client.on_disconnect = on_disconnect
		client.connect("localhost", 1883, 60)
		client.loop_start()


		print("Initializing master")
		# Initialize the DPM
		master.initialize()

		last_answer = {}
		resp_ctr = 0
		mismatch_ctr = 0

		print("Starting")
		# Run the slave state machine.
		for req in itertools.cycle(requests):

			while True:
				# 20 ms looks like the absolute maximum that can be tolerated.
				# Anything longer will make it just fail.
				# So to be on the safe side just use 10ms.
				# This way we shouldn't have 100% CPU usage
				time.sleep(0.010)
				# Feed the system watchdog, if it is available.
				if watchdog is not None:
					watchdog()

				# make outgoing telegram
				outdata = struct.pack('>H', req) + b'\0' * 30
				# Write the output data.
				for slaveDesc in master.getSlaveList():
					slaveDesc.setOutData(outdata)

				# Run slave state machines.
				handledSlaveDesc = master.run()

				# Get the in-data (receive)
				if not handledSlaveDesc:
					continue

				inData = handledSlaveDesc.getInData()

				if inData is None:
					continue

				rsp = struct.unpack('>H',inData[0:2])[0]
				if req != rsp:
					#print("{} req={:04x}, rsp={:04x}".format(mismatch_ctr, req,rsp))
					mismatch_ctr += 1
					continue

				#print("{} req={:04x}, rsp={:04x}".format(mismatch_ctr, req,rsp))

				resp_ctr += 1
				unchanged = False
				if req in last_answer.keys() and inData == last_answer[req]:
					unchanged = True
					if args.interactive: print("Counter = {}, Mismatch = {}\r".format(resp_ctr, mismatch_ctr), end='', flush=True)

				last_answer[req] = inData
				# otherwise, decode answer
				res_txt, res_json = decode_answer(inData)

				if args.interactive and not unchanged:
					print ("===== Request   ===================== 0x{:4X} =====".format(req))
					print ("===== Response  ===================== 0x{:4X} =====".format(struct.unpack('>H',inData[0:2])[0]))
					for row in res_txt:
						print(row)

				client.publish("dfbb/{:04x}".format(req), payload=json.dumps(res_json))

				break

	except pyprofibus.ProfibusError as e:
		print("Terminating: %s" % str(e))
		return 1
	finally:
		if master:
			master.destroy()
		if client:
			client.disconnect()
			client.loop_stop()

	return 0

if __name__ == "__main__":
	import sys
	sys.exit(main())
