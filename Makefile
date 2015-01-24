GTEST = gtest
INCLUDES = -I$(GTEST)/include -I$(GTEST)/
CXXFLAGS += -std=c++0x -g -Wall -Wextra -pthread

# GTEST library
libgtest: gtest-all.o
	ar -rv libgtest.a gtest-all.o
gtest-all.o:
	g++ $(INCLUDES) $(CXXFLAGS) -c $(GTEST)/src/gtest-all.cc

# ROBOT modules library
ROBOT_SRCS = $(wildcard *.cpp)
ROBOT_OBJS = $(ROBOT_SRCS:.cpp=.o)
LIBROBOT = librobot.a
.cpp.o:
	g++ $(INCLUDES) $(CXXFLAGS) -c $< -o $@
librobot: $(ROBOT_OBJS)
	ar -rv $(LIBROBOT) $(ROBOT_OBJS)

# TESTS
TEST_SRCS = test/*.cpp
tests: $(TEST_SRCS) libgtest librobot
	g++ $(INCLUDES) $(CXXFLAGS) $(TEST_SRCS) libgtest.a $(LIBROBOT) -o tests



