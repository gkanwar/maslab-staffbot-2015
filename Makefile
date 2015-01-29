GTEST = gtest
SRCDIR = src
OBJDIR = obj
LIBDIR = lib
BINDIR = bin
TESTDIR = test

INCLUDES = -I$(GTEST)/include -I$(GTEST)/ -I$(SRCDIR)/
CXXFLAGS += -std=c++0x -g -Wall -Wextra -pthread -pg 
ifdef EDISON
	CXXFLAGS += -DEDISON
endif
LINKFLAGS = 
ifndef EDISON
	LINKFLAGS += -lGL -lglut
else
	LINKFLAGS += -lmraa
endif

# ALL target: build every binary
all: builddir tests main_sim main

builddir:
	mkdir -p $(OBJDIR)
	mkdir -p $(BINDIR)
	mkdir -p $(LIBDIR)

# GTEST library
libgtest: $(OBJDIR)/gtest-all.o
	ar -rv $(LIBDIR)/libgtest.a $(OBJDIR)/gtest-all.o
$(OBJDIR)/gtest-all.o: $(GTEST)/src/gtest-all.cc
	g++ $(INCLUDES) $(CXXFLAGS) -c $(GTEST)/src/gtest-all.cc -o $@

# ROBOT modules library
ROBOT_SRCS = $(wildcard $(SRCDIR)/*.cpp)
ROBOT_OBJS = $(ROBOT_SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
ROBOT_DEPS = $(ROBOT_SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.d)
LIBROBOT = $(LIBDIR)/librobot.a
$(ROBOT_OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	g++ $(INCLUDES) $(CXXFLAGS) -c $< -MMD -MP -o $@
librobot: $(ROBOT_OBJS)
	ar -rv $(LIBROBOT) $(ROBOT_OBJS)

# TESTS
TEST_SRCS = $(wildcard $(TESTDIR)/*.cpp)
tests_depend: .tests_depend
.tests_depend: $(TEST_SRCS)
	rm -f .tests_depend
	g++ $(INCLUDES) $(CXXFLAGS) -MM $^ > .tests_depend
tests: $(TEST_SRCS) libgtest librobot | builddir
	g++ $(INCLUDES) $(CXXFLAGS) $(TEST_SRCS) $(LIBDIR)/libgtest.a $(LIBROBOT) $(LINKFLAGS) -o $(BINDIR)/tests

# MAIN_SIM
MAIN_SIM_SRCS = $(SRCDIR)/main_sim.cpp
main_sim_depend: .main_sim_depend
.main_sim_depend: $(MAIN_SIM_SRCS)
	rm -f .main_sim_depend
	g++ $(INCLUDES) $(CXXFLAGS) -MM $^ > .main_sim_depend
main_sim: $(MAIN_SIM_SRCS) librobot
	g++ $(INCLUDES) $(CXXFLAGS) $(MAIN_SIM_SRCS) $(LIBROBOT) $(LINKFLAGS) -o $(BINDIR)/$@

# MAIN
MAIN_SRCS = $(SRCDIR)/main.cpp
main_depend: .main_depend
.main_depend: $(MAIN_SRCS)
	rm -f .main_depend
	g++ $(INCLUDES) $(CXXFLAGS) -MM $^ > .main_depend
main: $(MAIN_SRCS) librobot
ifdef EDISON
	g++ $(INCLUDES) $(CXXFLAGS) $(MAIN_SRCS) $(LIBROBOT) $(LINKFLAGS) -o $(BINDIR)/$@
endif

# CLEAN
clean:
	rm $(OBJDIR)/*
	rm $(BINDIR)/*
	rm $(LIBDIR)/*

# Include generated deps files
-include $(ROBOT_DEPS)
