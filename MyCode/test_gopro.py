import goprohero
import cv2

if __name__ == '__main__':
    camera = goprohero.GoProHero()
    img = camera.image()

    print "GoPro status: " + str(camera.status())
