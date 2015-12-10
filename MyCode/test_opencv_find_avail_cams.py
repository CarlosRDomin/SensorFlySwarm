import cv2

video_capture = cv2.VideoCapture()

for i in range(0, 1501):
    if video_capture.open(i):
        print "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\tCAMERA " + str(i) + " OPENED!"
        ret, frame = video_capture.read()
        video_capture.release()
        if ret:
            cv2.imwrite('cam_' + str(i) + ".png", frame)
        else:
            print "Ooops, something went wrong accessing the frame! :S"
    else:
        print "Nothing on " + str(i) + "..."
