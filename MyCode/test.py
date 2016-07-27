# import cv2
# import numpy as np
# from datetime import datetime
#
# cv2.imshow("", np.zeros((1,1), dtype=np.uint8))
# cv2.waitKey(2000)
# print "Starting..."
# t = datetime.now()
# for i in range(5000):
#     key = cv2.waitKey(1)
#     t2 = datetime.now()
#     dT = (t2-t).total_seconds()*1000
#     if dT > 3:
#         print "dT = {:5.2f}ms\tkey={}".format(dT, key)
#     if key >= 0:
#         print "KeyPressed={}".format(chr(key))
#     t = t2

# import subprocess
#
# print "Finished: {}".format(subprocess.check_output(["sh"], shell=True))
# print "Finished: {}".format(subprocess.call(["python", "-c", "print 'Loading...'; import cv2; print cv2.__version__"], shell=True))

import sys
from threading import Thread

program_run = True
input_thread_timeout = 0.005 #seconds
quit_key = '\x1b' # x1b is ESC

#check stdin for input...
def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

#check n terminate program on terminal condition,
#from a separate thread
class waitOnInput(Thread):
    def run(self):
        # old_settings = termios.tcgetattr(sys.stdin)
        try:
            # tty.setcbreak(sys.stdin.fileno())
            global program_run
            thread_run = True
            while thread_run:
                if isData():
                    c = sys.stdin.read(1)
                    if c == quit_key:
                        break
                        thread_run = False
                        program_run = False
        finally:
            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            thread_run = False


t = waitOnInput()

#start work here...
i = 1

while program_run :
    if not t.is_alive():
        t.start()

    #test for terminal condition or timeout...
    t.join(input_thread_timeout)

    if t.is_alive():
        #continue work here...
        print i
        i += 1
    else:
        break
