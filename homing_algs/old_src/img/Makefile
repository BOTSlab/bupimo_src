include ../Makefile.common

# For valgrind
#CPPFLAGS += -g -O0

CPPFLAGS += -pthread

LDLIBS += -lcurses -lX11

OBJS = Img.o ImgWindow.o ImgOps.o Imgdb.o ImgSequence.o Unfolder.o ImgSet.o 
#SiftExtractor.o

#PROGS = windowTest dbTest opsTest imgs seqTest unfold rotate hassoun gvd imageIsBlack setTest testSiftExtractor siftLoop windowTest drawKeys opticFlow computeKeys
#PROGS = imgs rotate windowTest dbTest drawHissMatches drawKeys computeKeys testSiftExtractor unfold
PROGS = unfold

all: img.a copyToBin

img.a: $(OBJS)
	ar cr img.a $(OBJS)

copyToBin: $(PROGS)
#	cp $(PROGS) $(BINDIR)
	cp $(PROGS) ~/av_scripts/.

windowTest: windowTest.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

dbTest: dbTest.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

opsTest: opsTest.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

imgs: imgs.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

seqTest: seqTest.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

unfold: unfold.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

rotate: rotate.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

hassoun: hassoun.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

gvd: gvd.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

imageIsBlack: imageIsBlack.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

setTest: setTest.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

testSiftExtractor: testSiftExtractor.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

cameraSift: cameraSift.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o $(CAMERADIR)/camera.a

drawKeys: drawKeys.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

bruneau: bruneau.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

siftLoop: siftLoop.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

opticFlow: opticFlow.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

computeKeys: computeKeys.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

drawHissMatches: drawHissMatches.cpp $(OBJS) $(UTILSDIR)/SingleConfig.o

clean:
	rm -f *.o img.a $(PROGS)
