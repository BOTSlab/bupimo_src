#
# Generates warping Python module using SWIG.
#
PYTHON_CFLAGS = `pkg-config --cflags python`
PYTHON_LIBS = `pkg-config --libs python`
MODULE_NAME = warping
LIB = _$(MODULE_NAME).so
OBJS = FloatArrays.o IntArrays.o Math.o Warping.o $(MODULE_NAME).o \
	$(MODULE_NAME)_wrap.o

all: $(LIB)

$(MODULE_NAME)_wrap.o: $(MODULE_NAME).i $(MODULE_NAME).o
	swig -python -c++ $(MODULE_NAME).i
	g++ -c $(MODULE_NAME)_wrap.cxx $(PYTHON_CFLAGS)

$(LIB): $(OBJS)
	g++ -shared $(OBJS) -o $(LIB) $(PYTHON_LIBS)

clean:
	rm -f $(OBJS) $(LIB) $(MODULE_NAME)_wrap.cxx $(MODULE_NAME).py $(MODULE_NAME).pyc
