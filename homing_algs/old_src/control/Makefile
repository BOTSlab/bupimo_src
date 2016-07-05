include ../Makefile.common

CPPFLAGS += `pkg-config --cflags playerc++`
LDFLAGS += `pkg-config --libs playerc++` -lcurses -lboost_thread

# For valgrind runs
#CPPFLAGS += -g -O0

# For FlyCamera
CPPFLAGS += -I/usr/include/flycapture
LDFLAGS += -lflycapture

# For Sim
CPPFLAGS += `pkg-config --cflags plplot$(LIB_TAG)-c++`
LDFLAGS += `pkg-config --libs plplot$(LIB_TAG)-c++`

OBJS = Driver.o DriverAdaptor.o ManualController.o SimpleOdoController.o Odo1Controller.o TestGoToController.o TestOdo1Controller.o VisualHomingController.o RouteHomingController.o MyManager.o CaptureController.o SeqAdaptor.o

PROGS = driver #testBFRoute seqRoute

all: copyToBin

copyToBin: $(PROGS)
	cp $(PROGS) $(BINDIR)

driver: driver.cpp $(OBJS) $(CAMERADIR)/camera.a $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/CursesListener.o $(SIMDIR)/sim.a $(RNGDIR)/rng.o $(UTILSDIR)/SubsetSummer.o

testBFRoute: testBFRoute.cpp $(OBJS) $(CAMERADIR)/camera.a $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/CursesListener.o $(SIMDIR)/sim.a

seqRoute: seqRoute.cpp $(OBJS) $(CAMERADIR)/camera.a $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/CursesListener.o $(SIMDIR)/sim.a

clean:
	rm -fr *.o $(PROGS)
