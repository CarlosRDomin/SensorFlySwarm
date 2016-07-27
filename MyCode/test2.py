# import sys
# import sdl2
# import sdl2.ext
# from multiprocessing import Process, Value


# def func(val):
#     for j in range(50):
#         time.sleep(0.01)
#         with val.get_lock():
#             val.value += 1
#
# if __name__ == '__main__':
#     v = Value('L', 0)  # 'L' = unsigned long (https://docs.python.org/2/library/array.html#module-array)
#     procs = [Process(target=func, args=(v, )) for i in range(10)]
#
#     for p in procs: p.start()
#     for p in procs: p.join()
#
#     print v.value

# def run():
#     sdl2.ext.init()
#     window = sdl2.ext.Window("Test", size=(400,300))
#     window.show()
#     print "Ready!!"
#     t = datetime.now()
#     running = True
#     while running:
#         events = sdl2.ext.get_events()
#         for event in events:
#             if event.type == sdl2.SDL_QUIT:
#                 running = False
#                 break
#             if event.type == sdl2.SDL_KEYDOWN:
#                 try:
#                     window.title = "Last key pressed: {} at t={}".format(chr(event.key.keysym.sym), str(datetime.now().time())[:-3])
#                     print "Key: '{}'".format(chr(event.key.keysym.sym))
#                 except:
#                     window.title = "Last key pressed: \\x{:x} at t={}".format(event.key.keysym.sym, str(datetime.now().time())[:-3])
#                     pass
#                 if event.key.keysym.sym == sdl2.SDLK_UP:
#                     print "Up!"
#                 elif event.key.keysym.sym == sdl2.SDLK_DOWN:
#                     print "Down!"
#                 elif event.key.keysym.sym == sdl2.SDLK_CUT:
#                     print "Cut!"
#                 elif event.key.keysym.sym == sdl2.SDLK_ESCAPE:
#                     running = False
#                     print "ESC!"
#             elif event.type == sdl2.SDL_KEYUP:
#                 if event.key.keysym.sym in (sdl2.SDLK_UP, sdl2.SDLK_DOWN):
#                     print "Arrow released"
#         t2 = datetime.now()
#         dT = (t2-t).total_seconds()*1000
#         if dT > 0.5:
#             print "dT = {:5.2f}ms\tkey={}".format(dT, event.key.keysym.sym)
#         t = t2
#         # sdl2.SDL_Delay(10)
#         # world.process()
#
# if __name__ == "__main__":
#     exit(run())

from multiprocessing import Process, Queue, Value
import sharedmem
from uvc_capture import UvcCapture
import cv2
import numpy as np
from datetime import datetime
import time


def producer(v, v2, q, arr):
    cap = UvcCapture(0)
    cap.select_best_frame_mode(60)
    cap.load_settings("UVCcam settings - USB 2.0 Camera.txt")
    q.get()  # Wait for opencv to init

    t = []
    last_t = datetime.now()
    print "OpenCV init'ed! (consumer process says so)"
    while v.value < 300:
        t.append(datetime.now())
        frame = cap.get_frame_robust()
        t.append(datetime.now())
        img = frame.bgr
        t.append(datetime.now())
        arr[:] = frame.bgr
        v2.value = frame.index
        t.append(datetime.now())
        ### q.put([img, frame.index])
        # q.put(frame.index)
        t.append(datetime.now())
        print "\t\t\tPRODUCER: At t={} picture #{:04d} was taken; totalDeltaT={}ms;\t{}".format(str(t[3].time())[:-3], frame.index, (t[-1]-last_t).total_seconds()*1000, "\t\t".join(["{}->{}: {}ms".format(i+1, i+2, (t[i+1]-t[i]).total_seconds()*1000) for i in range(len(t)-1)]))
        last_t = t[-1]
        t = []
    q.put("STOP")
    print "Producer exiting with value {}".format(v.value)

def consumer(v, v2, q, arr):
    cv2.imshow("", cv2.imread("/Users/Carlitos/Pictures/Fotos de fondo/2300703248.jpg"))
    cv2.moveWindow("", 0,0)
    cv2.waitKey(1)
    q.put("Ready")

    t = []
    last_t = datetime.now()
    while v.value < 300:
        t.append(datetime.now())
        time.sleep(0.005)
        t.append(datetime.now())
        ### [frame, index] = q.get()  # Wait to get one frame
        # index = q.get()  # Wait to get one frame
        # try:
        #     while True:  # If there were more frames in the queue, read them as well to get the latest
        #         [frame, index] = q.get(True, 0.001)
        #         t.append(datetime.now())
        #         print "\t\t\t\t\t\t\t\t\t\t\t\tGot an extra frame this iteration!!! Took {} ms to get it".format((t[-1]-t[-2]).total_seconds()*1000)
        # except:
        #     logging.error("q.get_nowait.")
        #     pass
        #     # print "This should mean I have the latest frame!"
        t.append(datetime.now())
        cv2.imshow("", arr)
        t.append(datetime.now())
        key = cv2.waitKey(1)
        t.append(datetime.now())
        if key>=0: print"\t\t\t\t\t\t\t\t\t\t\tHEEEEEY, key is {}".format(key)
        if key==32: key = 500
        with v.get_lock():
            if v.value != 1000:
                v.value = key
        print "CONSUMER: At t={} picture #{:04d} was rcved; totalDeltaT={}ms;\t{}".format(str(t[1].time())[:-3], v2.value, (t[-1]-last_t).total_seconds()*1000, "\t\t".join(["{}->{}: {}ms".format(i+1, i+2, (t[i+1]-t[i]).total_seconds()*1000) for i in range(len(t)-1)]))
        last_t = t[-1]
        t = []

    print "Consumer exiting with value {}".format(v.value)
    cv2.destroyAllWindows()
    obj = q.get()
    while not isinstance(obj, str):
        obj = q.get()

def end(v):
    time.sleep(5)
    print "10 seconds elapsed. Ending!"
    v.value = 1000

# def do_work(data, start):
#     data[start] = 0;
#
# def split_work(num):
#     n = 20
#     width  = n/num
#     shared = sharedmem.empty(n)
#     shared[:] = numpy.random.rand(1, n)[0]
#     print "values are %s" % shared
#
#     processes = [Process(target=do_work, args=(shared, i*width)) for i in xrange(num)]
#     for p in processes:
#         p.start()
#     for p in processes:
#         p.join()
#
#     print "values are %s" % shared
#     print "type is %s" % type(shared[0])
#
# if __name__ == '__main__':
#     split_work(4)

if __name__ == '__main__':
    v = Value('l', 0)  # 'L' = unsigned long (https://docs.python.org/2/library/array.html#module-array)
    v2 = Value('L', 0)  # 'L' = unsigned long (https://docs.python.org/2/library/array.html#module-array)
    q = Queue(1)
    shared_arr = sharedmem.empty((720, 1280, 3), dtype=np.uint8)

    # cap = UvcCapture(0)
    # cap.select_best_frame_mode(60)
    # frame = cap.get_frame_robust()
    # shared_arr = Array(ctypes.c_uint8, frame.bgr.size)
    # shared_arr = np.empty_like(frame.bgr)
    # cap = None

    procs = []
    # with closing(Pool(processes=2, initializer=init, initargs=(shared_arr,))) as p:
    #     p.apply_async(producer, (v, q))
    #     p.apply_async(consumer, (v, q))
    # p.join()
    procs.append(Process(target=producer, args=(v, v2, q, shared_arr)))
    procs.append(Process(target=consumer, args=(v, v2, q, shared_arr)))
    # procs.append(Process(target=end, args=(v, )))

    for p in procs: p.start()
    for p in procs: p.join()

    print "Done!"
    exit()

    cap = UvcCapture(0)
    cap.select_best_frame_mode(60)
    frame = cap.get_frame_robust()
    cv2.imshow("", frame.bgr)
    key = cv2.waitKey(1)

    t = []
    last_t = datetime.now()
    print "OpenCV init'ed! (consumer process says so)"
    while key < 0:
        t.append(datetime.now())
        frame = cap.get_frame_robust()
        t.append(datetime.now())
        img = frame.bgr
        t.append(datetime.now())
        cv2.imshow("", img)
        t.append(datetime.now())
        key = cv2.waitKey(1)
        t.append(datetime.now())
        print "PRODUCER: At t={} picture #{:04d} was taken; totalDeltaT={}ms; {}".format(str(t[-1].time())[:-3], frame.index, (t[-1]-last_t).total_seconds()*1000, "\t".join(["{}->{}: {}ms".format(i+1, i+2, (t[i+1]-t[i]).total_seconds()*1000) for i in range(len(t)-1)]))
        last_t = t[-1]
        t = []
    cv2.destroyAllWindows()
