include ../Makefile.common

LDLIBS += -lcurses

OBJS = SingleConfig.o CursesListener.o SubsetSummer.o

PROGS = testSingleConfig testCursesListener testSubsetSummer testArrays

all: copyToBin

copyToBin: $(PROGS)
	cp $(PROGS) $(BINDIR)

testSingleConfig: testSingleConfig.cpp SingleConfig.o

testCursesListener: testCursesListener.cpp CursesListener.o

testSubsetSummer: testSubsetSummer.cpp SubsetSummer.o $(RNGDIR)/rng.o

testArrays: testArrays.cpp

clean:
	rm -fr *.o testSingleConfig testCursesListener testSubsetSummer

