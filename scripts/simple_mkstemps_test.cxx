#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
int main() {
  int fd = open("test.c", O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
}
