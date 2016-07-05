include ../Makefile.common

WARPING_OBJS = warping/Math.o warping/Warping.o

#OLD_HISS_OBJS = old_hiss/util.o old_hiss/key.o old_hiss/SIFTHoming.o

OBJS = TotalHomingAlg.o BlockMatch.o DIDCompass.o ExactVectorMap.o HomeDirCombiner.o TwoStepHomingAlg.o WarpingAlg.o 
#HissAlg.o
#OBJS = TotalHomingAlg.o BlockMatch.o OpticFlowHomingAlg.o DIDCompass.o ExactVectorMap.o HomeDirCombiner.o TwoStepHomingAlg.o WarpingAlg.o HissAlg.o FBAlg.o FlowHomingAlg.o
#MFDID.o

all: homing.a

# Include all of the object files in the current dir, as well as subdir's which
# ought to be homing related.
homing.a: $(WARPING_OBJS) $(OBJS)
	ar cr homing.a $(WARPING_OBJS) $(OBJS)
#homing.a: $(WARPING_OBJS) $(OLD_HISS_OBJS) $(OBJS)
#	ar cr homing.a $(WARPING_OBJS) $(OLD_HISS_OBJS) $(OBJS)

clean:
	rm -f *.o homing.a
