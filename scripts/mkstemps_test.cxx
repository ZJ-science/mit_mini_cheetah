#include <cstdlib>
#include <unistd.h>
int main() {
  char tmp[] = "mkstempstestXXXXXX.so";
  int flag = mkstemps(tmp, 3);
  return flag;
}
