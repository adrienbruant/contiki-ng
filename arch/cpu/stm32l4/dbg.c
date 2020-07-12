#include "dbg.h"
#include <unistd.h>

unsigned int dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
  int result = write(STDOUT_FILENO, seq, len);
  if(result < 0) {
    //hide errors
    result = 0;
  }

  return result;
}

int dbg_putchar(int c)
{
  return putchar(c);
}