import logging
import sys, os
from optparse import OptionParser
# import Tkinter as tk
import sdl2
import sdl2.ext

# tell python where to find mavlink so we can import it
import time

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))
from pymavlink import mavutil


def process_kb_input(master, window_for_kb_input):
	"""
	Processes all keydown events, and takes the corresponding action depending on which key was pressed.
	:return: Boolean indicating whether the experiment shall go on: True while everything's fine, False to stop it
	"""

	amount = 400

	events = sdl2.ext.get_events()  # Fetch any new input event
	for event in events:  # Iterate through all of them
		if event.type == sdl2.SDL_KEYDOWN:  # And only process keydown events
			key_orig = event.key.keysym.sym  # Fetch the key that was pressed down
			try:
				key = chr(key_orig)  # Try to convert the key to char
			except:  # Usually, this exeption occurs when a key can't be converted to char (arrows, Esc, etc.)
				if key_orig == sdl2.SDLK_UP:
					key = "Up"
				elif key_orig == sdl2.SDLK_DOWN:
					key = "Down"
				elif key_orig == sdl2.SDLK_LEFT:
					key = "Left"
				elif key_orig == sdl2.SDLK_RIGHT:
					key = "Right"
				elif key_orig == sdl2.SDLK_RETURN:
					key = "Enter"
				elif key_orig == sdl2.SDLK_ESCAPE:
					key = "Esc"
				else:
					key = hex(key_orig)
			if key == '\r' or key == '\n': key = "Enter"

			print("Key pressed: '{}'".format(key))
			window_for_kb_input.title = "Last key pressed: '{}'".format(key)
			key = key.lower()  # Convert to lowercase so we don't have to worry about different cases
			roll = pitch = yaw = thrust = 1500
			if key == 'a':  # Move left
				yaw -= amount
			elif key == 's':  # Move back
				thrust -= amount
			elif key == 'd':  # Move right
				yaw += amount
			elif key == 'w':  # Move forward
				thrust += amount
			elif key == 'up':  # Move up
				pitch += amount
			elif key == 'down':  # Move down
				pitch -= amount
			elif key == 'left':  # Toggle altitude hold mode
				roll -= amount
			elif key == 'right':  # Stop taking off and start flying (hover at current position)
				roll += amount
			elif key == 'enter':
				master.set_mode_flag(8, True)  # Arm system
				master.set_mode_flag(128, True)  # Arm motors
				# master.arducopter_arm()  # Arm motors
				print("Is APM armed? {}".format(master.motors_armed()))
				return True
			else:  # Any other key ends the experiment
				return False

			# if roll == 1500: roll = 0
			# if pitch == 1500: pitch = 0
			# if yaw == 1500: yaw = 0
			# if thrust == 1500: thrust = 0

			send_rc(master, [roll, pitch, thrust, yaw, 0, 0, 0, 0])

	return True

# class App(tk.Frame):
# 	def __init__(self, tkroot, mavmaster):
# 		tk.Frame.__init__(self, tkroot)
# 		self.mavmaster = mavmaster
# 		self.tkroot = tkroot
# 		self.validkeys = ['Right', 'Left', 'Up', 'Down', 'Return']
# 		self.afterId = dict((k,None) for k in self.validkeys)
# 		self.keyState = dict((k,False) for k in self.validkeys)
# 		self.tkroot.bind_all('<Key>', self.keyPressed)
# 		self.tkroot.bind_all('<KeyRelease>', self.keyReleased)
#
# 	def keyPressed(self,event):
# 		if event.keysym == 'Escape':
# 			release_rc(self.mavmaster)
# 			self.quit()
# 		if event.keysym in self.validkeys:
# 			if self.afterId[event.keysym] != None:
# 				self.after_cancel(self.afterId[event.keysym])
# 				self.afterId[event.keysym] = None
# 			else:
# 				self.keyState[event.keysym] = event.time
# 				processKeyCommand(self.mavmaster, self.keyState)
#
# 	def keyReleased(self,event):
# 		if event.keysym in self.validkeys:
# 			self.afterId[event.keysym] = self.after_idle(self.releaseKey, event)
#
# 	def releaseKey(self, event):
# 		self.afterId[event.keysym] = None
# 		self.keyState[event.keysym] = False
# 		processKeyCommand(self.mavmaster, self.keyState)
#
# # release control back to the radio
# def release_rc(master):
# 	# a value of 0 releases the control to what the radio says
# 	values = [ 0 ] * 8
# 	for i in xrange(1):
# 		master.mav.rc_channels_override_send(master.target_system,
# 			master.target_component, *values)

# attempt to send a control.
# you can pass 0 to refer to what the radio says
# you can pass 0xFFFF to refer to keep the current value
def send_rc(master, data):
	master.mav.rc_channels_override_send(master.target_system, master.target_component, *data)
	print ("Sending rc values: {}".format(data))

# def processKeyCommand(master, keystate):
# 	if keystate['Return']:
# 		master.arducopter_arm()
# 		# master.mav.command_long_send(master.target_system, master.target_component, master.mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1)
#
# 	amt = 100
# 	pitch = 1500
# 	roll = 1500
# 	# handle elevator stick
# 	if keystate['Up']:
# 		pitch -= amt	# nose down (to go forward)
# 	if keystate['Down']:
# 		pitch += amt	# nose up (to go back)
# 	if pitch == 1500:
# 		# if this stick is centered, just release it
# 		pitch = 0
# 	# handle aile stick
# 	if keystate['Left']:
# 		roll -= amt		# roll left
# 	if keystate['Right']:
# 		roll += amt		# roll right
# 	if roll == 1500:
# 		# if this stick is centered, just release it
# 		roll = 0
# 	# set data (0 means channel is released)
# 	data = [ 0 ] * 8
# 	data[0] = roll
# 	data[1] = pitch
# 	data[2] = pitch  # Thrust
# 	data[3] = roll  # Yaw
# 	# send the data to the rc override command
# 	send_rc(master, data)


def main():

	# read command line options
	parser = OptionParser("readdata.py [options]")
	parser.add_option("--baudrate", dest="baudrate", type='int',
					  help="master port baud rate", default=57600)
	# parser.add_option("--device", dest="device", default=None, help="serial device")
	parser.add_option("--device", dest="device", default="/dev/tty.usbserial-DN0022OM", help="serial device")
	parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
	parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
					  default=250, help='MAVLink source system for this GCS')
	parser.add_option("--showmessages", dest="showmessages", action='store_true',
					  help="show incoming messages", default=True)
	(opts, args) = parser.parse_args()
	
	if opts.device is None:
		print("You must specify a serial device")
		sys.exit(1)

	# create a mavlink serial instance
	print("Attempting to open device '{}'...".format(opts.device))
	master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate, source_system=250, autoreconnect=True, retries=3)

	# wait for the heartbeat msg to find the system ID
	print("Successfully opened device '{}'. Waiting for heartbeat...".format(opts.device))
	master.wait_heartbeat()
	print("Heartbeat received! Initializing connection.")

	# request data to be sent at the given rate
	master.mav.request_data_stream_send(master.target_system, master.target_component, 
		mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)
	print("Opening GUI to detect key press events.")

	# start up the window to read arrow keys
	# tkroot = tk.Tk()
	# application = App(tkroot, master)
	# tkroot.mainloop()
	window_for_kb_input = None
	try:
		sdl2.ext.init()
		window_for_kb_input = sdl2.ext.Window("Window to receive keyboard input", size=(400, 300))
		window_for_kb_input.show()
		while process_kb_input(master, window_for_kb_input):
			time.sleep(0.5)
	except:
		logging.exception("Something went wrong...")

	print("Exiting...")
	send_rc(master, [0]*8)  # Release control back to RC for all channels
	if window_for_kb_input: window_for_kb_input.hide()
	sdl2.ext.quit()
	master.close()
	print("Bye!")


if __name__ == '__main__':
	main()