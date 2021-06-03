#!/usr/bin/env python3
#
# Simple pyprofibus dummy example using dummy PHY.
# This example can be run without any PB hardware.
#

import pyprofibus
import time
import struct
import itertools

decoders = {
	0xA101:
	[
	('H', 'Actual value mill current', 'A'),
	('H', 'Actual value feeder rotational speed in %', '%'),
	('H', 'Actual value product temperature mixer outlet', '°C'),
	('H', 'Actual value liquid 1', 'kg/h'),
	('H', 'Actual value liquid 2', 'kg/h'),
	('H', 'Actual value liquid 3', 'kg/h'),
	('H', 'Actual value gap roll adjustment', '1/10mm'),
	('B', 'Phase in operating state 3 (only with retentioner)', ''),
	('B', """Operating state of the DFBB
0 = Idle position/pre-start
1 = Start-up
2 = Feed readiness/Feed interruption
3 = Feed start-up
4 = Operation
5 = Shutdown""", ''),
	('B', '0=Normal mode; 1=Hygienisation mode', ''),
	('B', """Operating mode of the DFBB
0 = Automatic mode local
1 = Automatic mode Control system/Remote control
2 = Maintenance mode""", ''),
	('H', 'Position slide gate counter current cooler', ''),
	('H', 'Actual value liquid 4', 'kg/h'),
	('L', 'Pressed flour amount', 'kg'),
	('L', 'Actual value feed capacity ', 'kg/h'),
	],

	0xA105:
	[
	('H', 'Actual value Temperature mill inlet', '°C'),
	('H', 'Actual value Temperature mill cavity', '°C'),
	('H', 'Actual value Temperature cooler inlet', '°C'),
	('H', 'Actual value Temperature cooler exhaust hood', '°C'),
	('H', 'Reserve', ''),
	('H', 'Actual value Temperature heating mats mixer', '°C'),
	('H', 'Actual value Temperature heating mats retentioner 1', '°C'),
	('H', 'Actual value Temperature heating mats retentioner 2', '°C'),
	('H', 'Actual value Steam quantity', 'kg/h'),
	('H', 'Actual value Product temperature retentioner', '°C'),
	('H', 'Fed liquid quantity 1', 'kg'),
	('H', 'Fed liquid quantity 2', 'kg'),
	('H', 'Fed liquid quantity 3', 'kg'),
	('H', 'Fed liquid quantity 4', 'kg'),
	('H', 'Actual value product temperature mixer inlet', '°C'),
	],

	0xA201:
	[

	('H', 'Nominal value mill current', 'A'),
	('H', 'Nominal value feeder rotational speed 1', '%'),
	('H', 'Nominal value feeder rotational speed 2', '%'),
	('H', 'Nominal value feeder rotational speed 3', '%'),
	('H', 'Nominal value Product temperature mixer', '°C'),
	('H', 'Nominal value Liquid 1', '1/1000'),
	('H', 'Nominal value Liquid 2', '1/1000'),
	('H', 'Nominal value Liquid 3', '1/1000'),
	('H', "Nominal value Working gap mill adjustment", '1/10mm'),
	('H', 'Parameter set', ''),
	('H', 'Throughput Counter current cooler DFKG', ''),
	('H', 'Nominal value retention time', 's'),
	('H', 'Bulk density mash', 'kg/m3'),
	('L', 'Maximum flour throughput', 'kg/h'),
	]
}

requests = [0xA101, 0xA105, 0xA201]

def decode_answer(msg):
	id = struct.unpack('>H', msg[0:2])[0]
	dec = decoders[id]

	# sanity check
	exp_len = 2 + sum([struct.calcsize(fmt) for fmt, desc, unity in dec])
	act_len = len(msg)
	if exp_len != act_len:
		print("Expected msg to be {} bytes long, received {}".format(exp_len, act_len))


	width = 40
	res = []
	offset = 2
	for fmt, desc, unit in dec:
		val = struct.unpack_from('>'+fmt, msg, offset)[0]
		offset += struct.calcsize(fmt)
		entry = "{:_<42}: {:6d} [{:<6}]".format(desc, val, unit)
		res.append(entry)

	return res

def main(watchdog=None):
	master = None
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

		# Initialize the DPM
		master.initialize()

		last_answer = {}
		resp_ctr = 0

		# Run the slave state machine.
		for req in itertools.cycle(requests):
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
				continue

			resp_ctr += 1
			if req in last_answer.keys() and inData == last_answer[req]:
				print("Counter = {}\r".format(resp_ctr), end='')
				continue

			last_answer[req] = inData

			print ("===== Request   ===================== 0x{:4X} =====".format(req))
			print ("===== Response  ===================== 0x{:4X} =====".format(struct.unpack('>H',inData[0:2])[0]))
			# otherwise, decode answer
			res = decode_answer(inData)
			for row in res:
				print(row)
			#print("Received data: " + str(inData))
			# In our example the output data shall be the inverted input.
			#outData[handledSlaveDesc.name][0] = inData[1]
			#outData[handledSlaveDesc.name][1] = inData[0]


	except pyprofibus.ProfibusError as e:
		print("Terminating: %s" % str(e))
		return 1
	finally:
		if master:
			master.destroy()
	return 0

if __name__ == "__main__":
	import sys
	sys.exit(main())
