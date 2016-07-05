# ImgSetCamera has been taken out of the build
# ...also commented out OfflineCamera and FlyCamera requirements below

include ../Makefile.common

# Interaction with Player required for OfflineCamera
#CPPFLAGS += `pkg-config --cflags playerc++`
#LDFLAGS += `pkg-config --libs playerc++`

# For FlyCamera
#CPPFLAGS += -I/usr/include/flycapture
#LDFLAGS += -lflycapture

# For valgrind
# CPPFLAGS += -g -O0

#OBJS = Camera.o OfflineCamera.o OCVCamera.o FlyCamera.o
OBJS = Camera.o OCVCamera.o

PROGS = testCamera

#all: camera.a copyToBin
all: camera.a

camera.a: $(OBJS)
	ar cr camera.a $(OBJS)

copyToBin: $(PROGS)
	cp $(PROGS) $(BINDIR)

testCamera: $(OBJS) $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

clean:
	rm -f $(OBJS) *.jpg camera.a testCamera
