from multiprocessing import Process, Queue
import cv2
from datetime import datetime
import time


def producer(q):
    print "Consumer said: {}".format(q.get())
    last_t = datetime.now()
    t = []
    for i in range(10):
        t.append(datetime.now())
        q.put(i)
        t.append(datetime.now())
        print "\t\t\tPRODUCER: At t={} sent {} through the queue; totalDeltaT={}ms;\t{}".format(str(t[1].time())[:-3], i, (t[-1]-last_t).total_seconds()*1000, "\t\t".join(["{}->{}: {}ms".format(i+1, i+2, (t[i+1]-t[i]).total_seconds()*1000) for i in range(len(t)-1)]))
        last_t = t[-1]
        t = []

    q.put("STOP")
    print "Producer exiting"

def consumer(q):
    q.put("START")
    last_t = datetime.now()
    t = []
    for i in range(10):
        t.append(datetime.now())
        j = q.get()
        t.append(datetime.now())
        print "CONSUMER: At t={} received {} through the queue; totalDeltaT={}ms;\t{}".format(str(t[1].time())[:-3], j, (t[-1]-last_t).total_seconds()*1000, "\t\t".join(["{}->{}: {}ms".format(i+1, i+2, (t[i+1]-t[i]).total_seconds()*1000) for i in range(len(t)-1)]))
        last_t = t[-1]
        t = []

    print "Consumer exiting"
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
    q = Queue(1)
    procs = []
    procs.append(Process(target=producer, args=(q,)))
    procs.append(Process(target=consumer, args=(q,)))
    # procs.append(Process(target=end, args=(v, )))

    for p in procs: p.start()
    for p in procs: p.join()

    print "Done!"
