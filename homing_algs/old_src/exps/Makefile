include ../Makefile.common

# For Sim
#CPPFLAGS += -pthread `pkg-config --cflags plplot$(LIB_TAG)-c++`
#LDLIBS += `pkg-config --libs plplot$(LIB_TAG)-c++` -lX11
CPPFLAGS += `pkg-config --cflags plplot$(LIB_TAG)-c++`
LDLIBS += `pkg-config --libs plplot$(LIB_TAG)-c++`

CPPFLAGS += `pkg-config --cflags plplot$(LIB_TAG)`
LDLIBS += `pkg-config --libs plplot$(LIB_TAG)` -lX11


#PROGS = seqHomeAngles ffilter routeExtractor homeVectors
#PROGS = oneShot homeVectors seqSimilarity visSimilarity
PROGS = oneShot homeVectors

all: copyToBin

copyToBin: $(PROGS)
	cp $(PROGS) $(BINDIR)

$(IMGDIR)/img.a:
	(cd $(IMGDIR); $(MAKE))
$(SIMDIR)/sim.a:
	(cd $(SIMDIR); $(MAKE))
$(HOMINGDIR)/homing.a:
	(cd $(HOMINGDIR); $(MAKE))

homeVectors: homeVectors.cpp DBVectorField.o $(SIMDIR)/sim.a $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(RNGDIR)/rng.o $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/SubsetSummer.o

similarityMap: similarityMap.cpp $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

seqHomeAngles: seqHomeAngles.cpp $(SIMDIR)/sim.a $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(RNGDIR)/rng.o $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/SubsetSummer.o

routeExtractor: routeExtractor.cpp RouteExtractor.o $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

ffilter: ffilter.cpp FeatureFilter.o $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

seqSimilarity: seqSimilarity.cpp $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

visSimilarity: visSimilarity.cpp $(SIMDIR)/sim.a $(IMGDIR)/img.a $(UTILSDIR)/SingleConfig.o

oneShot: oneShot.cpp $(HOMINGDIR)/homing.a $(IMGDIR)/img.a $(RNGDIR)/rng.o $(UTILSDIR)/SingleConfig.o $(UTILSDIR)/SubsetSummer.o

clean:
	rm -f *.o $(PROGS)
