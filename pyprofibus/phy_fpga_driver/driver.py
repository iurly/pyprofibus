# -*- coding: utf-8 -*-
#
# Driver for FPGA based PROFIBUS PHY.
#
# Copyright (c) 2019 Michael Buesch <m@bues.ch>
#
# Licensed under the terms of the GNU General Public License version 2,
# or (at your option) any later version.
#

from __future__ import division, absolute_import, print_function, unicode_literals
from pyprofibus.compat import *

import time

from pyprofibus.phy_fpga_driver.exceptions import *
from pyprofibus.phy_fpga_driver.messages import *
from pyprofibus.phy_fpga_driver.io import *
from pyprofibus.util import monotonic_time


__all__ = [
	"FpgaPhyDriver",
]


class FpgaPhyDriver(object):
	"""Driver for FPGA based PROFIBUS PHY.
	"""

	FPGA_CLK_HZ		= 16000000
	PING_INTERVAL		= 0.1

	def __init__(self, spiDev=0, spiChipSelect=0, spiSpeedHz=1000000):
		self.__ioProc = None
		self.__nextPing = monotonic_time()
		self.__receivedPong = False
		self.__startup(spiDev, spiChipSelect, spiSpeedHz)

	def __startup(self, spiDev, spiChipSelect, spiSpeedHz):
		"""Startup the driver.
		"""
		self.shutdown()

		# Start the communication process.
		self.__ioProc = FpgaPhyProc(spiDev, spiChipSelect, spiSpeedHz)
		if not self.__ioProc.start():
			raise FpgaPhyError("Failed to start I/O process.")

		# Reset the FPGA.
		# But first ping the device to make sure SPI communication works.
		self.__ping()
		self.__controlSend(FpgaPhyMsgCtrl(FpgaPhyMsgCtrl.SPICTRL_SOFTRESET))
		time.sleep(0.01)
		self.__ping()

		# Get the FPGA status to clear all errors.
		self.__fetchStatus()

		# Clear all event counters in I/O proc.
		self.__ioProc.getEventStatus()

		self.__nextPing = monotonic_time() + self.PING_INTERVAL
		self.__receivedPong = True

	def __ping(self, tries=3, shutdown=True):
		"""Ping the FPGA and check if a pong can be received.
		Calls shutdown() and raises a FpgaPhyError on failure.
		"""
		for i in range(tries - 1, -1, -1):
			try:
				pingMsg = FpgaPhyMsgCtrl(FpgaPhyMsgCtrl.SPICTRL_PING)
				pongMsg = self.__controlTransferSync(pingMsg,
							FpgaPhyMsgCtrl.SPICTRL_PONG)
				if not pongMsg:
					raise FpgaPhyError("Cannot communicate with "
							    "PHY. Timeout.")
				break
			except FpgaPhyError as e:
				if i <= 0:
					if shutdown:
						try:
							self.shutdown()
						except FpgaPhyError as e:
							pass
					raise e

	def __fetchStatus(self):
		"""Fetch the FPGA status.
		"""
		txMsg = FpgaPhyMsgCtrl(FpgaPhyMsgCtrl.SPICTRL_GETSTATUS)
		rxMsg = self.__controlTransferSync(txMsg, FpgaPhyMsgCtrl.SPICTRL_STATUS)
		if not rxMsg:
			raise FpgaPhyError("Failed to get status.")
		return txMsg.ctrlData

	def shutdown(self):
		"""Shutdown the driver.
		"""
		if self.__ioProc is None:
			return
		self.__ioProc.shutdownProc()
		self.__ioProc = None

	def setBaudRate(self, baudrate):
		"""Configure the PHY baud rate.
		"""
		if self.__ioProc is None:
			raise FpgaPhyError("Cannot set baud rate. "
					    "Driver not initialized.")
		if baudrate < 9600 or baudrate > 12000000:
			raise FpgaPhyError("Invalid baud rate %d." % baudrate)

		clksPerSym = int(round(self.FPGA_CLK_HZ / baudrate))
		assert(1 <= clksPerSym <= 0xFFFFFF)
		#TODO calculate the baud rate error and reject if too big.

		txMsg = FpgaPhyMsgCtrl(FpgaPhyMsgCtrl.SPICTRL_BAUD,
					ctrlData=clksPerSym)
		rxMsg = self.__controlTransferSync(txMsg, FpgaPhyMsgCtrl.SPICTRL_BAUD)
		if not rxMsg or rxMsg.ctrlData != txMsg.ctrlData:
			raise FpgaPhyError("Failed to set baud rate.")

	def __controlTransferSync(self, ctrlMsg, rxCtrlMsgId):
		"""Transfer a control message and wait for a reply.
		"""
		self.__controlSend(ctrlMsg)
		for j in range(50):
			for rxMsg in self.__controlReceive():
				if rxMsg.ctrl == rxCtrlMsgId:
					return rxMsg
			time.sleep(0.01)
		return None

	def __controlSend(self, ctrlMsg):
		"""Send a FpgaPhyMsgCtrl() control message.
		"""
		return self.__ioProc.controlSend(ctrlMsg)

	def __controlReceive(self):
		"""Get a list of received control messages.
		Returns a list of FpgaPhyMsgCtrl().
		The returned list might be empty.
		"""
		return self.__ioProc.controlReceive()

	def __handleControl(self):
		"""Receive and handle pending control messages.
		"""
		rxMsgs = self.__controlReceive()
		for rxMsg in rxMsgs:
			ctrl = rxMsg.ctrl
			if ctrl == FpgaPhyMsgCtrl.SPICTRL_NOP:
				pass # Nothing to do.
			elif ctrl == FpgaPhyMsgCtrl.SPICTRL_PONG:
				self.__receivedPong = True
			else:
				raise FpgaPhyError("Received unexpected "
						   "control message: %s" % str(rxMsg))

	def __handleEvents(self, events):
		if events & (1 << FpgaPhyProc.EVENT_RESET):
			statusBits = self.__fetchStatus()
			raise FpgaPhyError("Reset detected. "
					   "Status = 0x%02X." % statusBits)
		if events & (1 << FpgaPhyProc.EVENT_NEWSTAT):
			statusBits = self.__fetchStatus()
			print("STAT 0x%X" % statusBits)
			pass#TODO
		if events & (1 << FpgaPhyProc.EVENT_PARERR):
			print("PARITY ERROR")
			pass#TODO
		if events & (1 << FpgaPhyProc.EVENT_NOMAGIC):
			print("MAGIC BYTE NOT FOUND")
			pass#TODO
		if events & (1 << FpgaPhyProc.EVENT_INVALLEN):
			print("INVALID LENGTH FIELD")
			pass#TODO
		if events & (1 << FpgaPhyProc.EVENT_PBLENERR):
			print("INVALID PROFIBUS TELEGRAM LENGTH")
			pass#TODO

	def telegramSend(self, txTelegramData):
		"""Send a PROFIBUS telegram.
		"""
		now = monotonic_time()
		if now >= self.__nextPing:
			if not self.__receivedPong:
				# We did not receive the PONG to the previous PING.
				raise FpgaPhyError("PING failed.")
			self.__nextPing = now + self.PING_INTERVAL
			self.__receivedPong = False
			# Send a PING to the FPGA to check if it is still alive.
			pingMsg = FpgaPhyMsgCtrl(FpgaPhyMsgCtrl.SPICTRL_PING)
			self.__controlSend(pingMsg)
		return self.__ioProc.dataSend(txTelegramData)

	def telegramReceive(self):
		"""Get a list of received PROFIBUS telegrams.
		Returns a list of bytes.
		The returned list might be empty.
		"""
		rxTelegrams = []
		ioProc = self.__ioProc
		events = ioProc.getEventStatus()
		if events:
			self.__nextPing = monotonic_time() + self.PING_INTERVAL
			self.__handleEvents(events)
		if ioProc.controlAvailable():
			self.__nextPing = monotonic_time() + self.PING_INTERVAL
			self.__handleControl()
		if ioProc.dataAvailable():
			self.__nextPing = monotonic_time() + self.PING_INTERVAL
			rxTelegrams = ioProc.dataReceive()
		return rxTelegrams