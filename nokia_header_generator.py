import cv2
import numpy



img = cv2.imread('/home/waltraut/img.png', cv2.CV_LOAD_IMAGE_GRAYSCALE) < 128
print(img.shape)


print('uint8_t img[]={'),
for y in range(0, 48, 8):
	for x in range(0, 84):
		v = numpy.packbits(img[y:y+8, x][::-1])[0]

		if y == 40 and x == 83:
			print('%d};' % v)
		else:
			print('%d,' % v),

