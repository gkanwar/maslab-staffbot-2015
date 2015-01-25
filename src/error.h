#ifndef ERROR_H
#define ERROR_H

#include <iostream>
#include <sstream>

using namespace std;

class ErrorHandler {
 private:
  string msg;
  ostringstream msgStream;
  bool cond;

 public:
  ErrorHandler(const char *file, const char *func, 
               const char *condStr, int line, bool cond)
      : msgStream(msg), cond(cond) {
    if (cond) { return; }

    msgStream << "Error in " << file << ":" << line << " at " << func << endl;
    msgStream << "Condition failed: " << condStr << endl;
  }
  ~ErrorHandler() {
    if (!cond) {
      cerr << msgStream.str() << endl;
      abort();
    }
  }

  template <typename T>
  ErrorHandler& operator<<(T const& value) {
    msgStream << value;
    return *this;
  }
};

// Custom asserts with logging
#ifndef NDEBUG
// Keep asserts in debug mode
#define rassert(cond) \
  ErrorHandler(__FILE__, __FUNCTION__, #cond, __LINE__, (cond))
#else
// Remove asserts in release mode
#define rassert(cond)
#endif

#endif
