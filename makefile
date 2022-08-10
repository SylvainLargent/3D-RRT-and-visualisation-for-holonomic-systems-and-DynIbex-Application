SRCS=$(wildcard *.cpp)
BINS=$(SRCS:.cpp=)

CXXFLAGS := $(shell pkg-config --cflags ibex)
LIBS	 := $(shell pkg-config --libs  ibex)

ifeq ($(DEBUG), yes)
CXXFLAGS := $(CXXFLAGS) -O0 -g -pg -Wall -frounding-math
else
CXXFLAGS := $(CXXFLAGS) -O2 -DNDEBUG -Wno-deprecated -Wno-logical-op-parentheses -frounding-math -g
endif

all: $(BINS)

% :	%.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f $(BINS)
