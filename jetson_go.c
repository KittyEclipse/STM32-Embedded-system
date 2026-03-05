

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

static int set_serial(int fd)
{
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) { perror("tcgetattr"); return -1; }

  cfmakeraw(&tty);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); return -1; }
  tcflush(fd, TCIOFLUSH);
  return 0;
}

static int write_all(int fd, const char *buf, size_t len)
{
  while (len) {
    ssize_t w = write(fd, buf, len);
    if (w < 0) { if (errno == EINTR) continue; perror("write"); return -1; }
    buf += (size_t)w;
    len -= (size_t)w;
  }
  return 0;
}

int main(void)
{
  const char *dev = "/dev/ttyTHS1";
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror("open /dev/ttyTHS1"); return 1; }
  if (set_serial(fd) != 0) { close(fd); return 1; }

  printf("Opened %s @115200 8N1\n", dev);
  printf("Type command then Enter (GO/FWD/BWD/STOP). Type QUIT to exit.\n\n");

  char rx[256];
  char line[128];

  while (1) {
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd, &rset);
    FD_SET(STDIN_FILENO, &rset);
    int maxfd = fd > STDIN_FILENO ? fd : STDIN_FILENO;

    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    int r = select(maxfd + 1, &rset, NULL, NULL, &tv);
    if (r < 0) { if (errno == EINTR) continue; perror("select"); break; }

    if (FD_ISSET(fd, &rset)) {
      ssize_t n = read(fd, rx, sizeof(rx) - 1);
      if (n > 0) { rx[n] = 0; printf("STM32: %s", rx); fflush(stdout); }
    }

    if (FD_ISSET(STDIN_FILENO, &rset)) {
      if (!fgets(line, sizeof(line), stdin)) break;

      // remove trailing newline
      size_t L = strlen(line);
      while (L && (line[L-1] == '\n' || line[L-1] == '\r')) line[--L] = 0;

      if (strcasecmp(line, "QUIT") == 0) break;
      if (L == 0) continue;

      // send command + '\n'
      char out[160];
      snprintf(out, sizeof(out), "%s\n", line);
      if (write_all(fd, out, strlen(out)) != 0) break;
      printf("Sent: %s\\n\n", line);
    }
  }

  close(fd);
  return 0;
}
