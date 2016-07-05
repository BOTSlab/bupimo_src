include ../../Makefile.common

OBJS = util.o key.o SIFTHoming.o

all: $(OBJS) hiss_cpp_test

hiss_cpp_test: hiss_cpp_test.cpp $(OBJS) $(IMGDIR)/img.a
#	$(CXX) $^ -o $@ $(CPPFLAGS) $(LDFLAGS)

clean:
	rm -f *.o hiss_cpp_test
