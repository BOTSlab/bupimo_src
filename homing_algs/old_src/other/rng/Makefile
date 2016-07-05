
# Specify the compiler and compiler options
CC = g++
CCOPTS = -Wall -O
#CCOPTS = -Wall -O3 -march=pentium4
#CCOPTS = -pedantic -ansi -Wall -g

# The files used to make the program.
HEADERS = rng.h
SOURCES = randtest.C rng.C 
OBJECTS = $(SOURCES:.C=.o)

# Pattern rule
%.o : %.C $(HEADERS)
	$(CC) $(CCOPTS) -c $<

# Make the source
randtest : $(HEADERS) $(OBJECTS) 
	$(CC) $(CCOPTS) $(OBJECTS) -o randtest $(LDLIBS)

tags: $(HEADERS) $(SOURCES)
	etags --c++ $(HEADERS) $(SOURCES) 
 
clean :
	rm -f *.o rng randtest

checkin :
	ci -l $(HEADERS) $(SOURCES) Makefile index.html

archive: checkin
	ln -s . RNG
	tar chvf rng.tar RNG/rng.[Ch] RNG/randtest.C RNG/Makefile
	rm RNG

