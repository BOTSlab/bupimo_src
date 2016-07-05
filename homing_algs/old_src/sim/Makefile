include ../Makefile.common
OBJS = Sim.o SimWindow.o SeqWindow.o

CPPFLAGS += `pkg-config --cflags plplot$(LIB_TAG)-c++`
LDLIBS += `pkg-config --libs plplot$(LIB_TAG)-c++`

CPPFLAGS += `pkg-config --cflags plplot$(LIB_TAG)`
LDLIBS += `pkg-config --libs plplot$(LIB_TAG)` -lX11

PROGS = simTest swTest
#PROGS = simTest swTest seqTest

all: sim.a copyToBin

copyToBin: $(PROGS)
	cp $(PROGS) $(BINDIR)

sim.a: $(OBJS)
	ar cr sim.a $(OBJS)

swTest: swTest.cpp sim.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

simTest: simTest.cpp sim.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o $(RNGDIR)/rng.o

seqTest: seqTest.o sim.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

clean:
	rm -f *.o sim.a simTest swTest seqTest
