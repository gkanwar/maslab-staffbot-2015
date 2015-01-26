GTEST = gtest
SRCDIR = src
OBJDIR = obj
LIBDIR = lib
BINDIR = bin
TESTDIR = test

INCLUDES = -I$(GTEST)/include -I$(GTEST)/ -I$(SRCDIR)/
CXXFLAGS += -std=c++0x -g -Wall -Wextra -pthread
LINKFLAGS = -lGL -lglut

# GTEST library
libgtest: gtest-all.o
	ar -rv $(LIBDIR)/libgtest.a $(OBJDIR)/gtest-all.o
gtest-all.o:
	g++ $(INCLUDES) $(CXXFLAGS) -c $(GTEST)/src/gtest-all.cc -o $(OBJDIR)/gtest-all.o

# ROBOT modules library
ROBOT_SRCS = $(wildcard $(SRCDIR)/*.cpp)
ROBOT_OBJS = $(ROBOT_SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
$(ROBOT_OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	g++ $(INCLUDES) $(CXXFLAGS) -c $< -o $@
LIBROBOT = $(LIBDIR)/librobot.a
librobot: $(ROBOT_OBJS)
	ar -rv $(LIBROBOT) $(ROBOT_OBJS)

# TESTS
TEST_SRCS = $(wildcard $(TESTDIR)/*.cpp)
# TEST_OBJS = $(TEST_SRCS:$(TESTDIR)/%.cpp=$(OBJDIR)/%.o)
# $(TEST_OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
# 	g++ $(INCLUDES) $(CXXFLAGS) -c $< -o $@
tests: $(TEST_SRCS) libgtest librobot
	g++ $(INCLUDES) $(CXXFLAGS) $(TEST_SRCS) $(LIBDIR)/libgtest.a $(LIBROBOT) $(LINKFLAGS) -o $(BINDIR)/tests

# MAIN_SIM
MAIN_SIM_SRCS = $(SRCDIR)/main_sim.cpp
main_sim: $(MAIN_SIM_SRCS) librobot
	g++ $(INCLUDES) $(CXXFLAGS) $(MAIN_SIM_SRCS) $(LIBROBOT) $(LINKFLAGS) -o $(BINDIR)/$@

# CLEAN
clean:
	rm $(ROBOT_OBJS)
	rm 
