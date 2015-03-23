
CFLAGS = -I/usr/local/include
LDFLAGS = -L/usr/local/lib \
-lopencv_calib3d \
-lopencv_contrib \
-lopencv_core \
-lopencv_features2d \
-lopencv_flann \
-lopencv_highgui \
-lopencv_imgproc \
-lopencv_legacy \
-lopencv_ml \
-lopencv_objdetect \
-lopencv_video

CC = g++ -c $(CFLAGS) 
CPP = $(CC) 

.cpp.o :
	g++ -c $(CFLAGS) $(LDFLAGS)  $<
