CALIB_SRCS =  CameraCalibrator.cpp calibrate.cpp

CFLAGS = -I /usr/local/include 
LIBS = -L/usr/local/lib \
      -lopencv_core \
      -lopencv_highgui \
      -lopencv_calib3d \
      -lopencv_features2d \
      -lopencv_contrib \
      -lopencv_imgproc \
      -lopencv_ml \
      -lopencv_nonfree

camcalib :CameraCalibrator.h $(CALIB_SRCS)
	clang++ -o $@ $(CALIB_SRCS) $(CFLAGS) $(LIBS)
