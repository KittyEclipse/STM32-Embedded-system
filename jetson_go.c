// jetson_go.c
// Build:  gcc -O2 -Wall jetson_go.c -o jetson_go
// Run:    ./jetson_go
//
// Opens /dev/ttyTHS1 @115200 8N1, sends "GO\n", then prints any STM32 replies.
// Press 's' + Enter to send STOP, 'g' + Enter to send GO again, 'q' to quit.

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

static int set_serial(int fd, int baud)
{
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr");
    return -1;
  }

  cfmakeraw(&tty);

  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;      // no HW flow control

  // Non-blocking reads with select()
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  speed_t spd = B115200;
  (void)baud;
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    return -1;
  }

  tcflush(fd, TCIOFLUSH);
  return 0;
}

static int write_all(int fd, const char *s)
{
  size_t n = strlen(s);
  while (n) {
    ssize_t w = write(fd, s, n);
    if (w < 0) {
      if (errno == EINTR) continue;
      perror("write");
      return -1;
    }
    s += (size_t)w;
    n -= (size_t)w;
  }
  return 0;
}

int main(void)
{
  const char *dev = "/dev/ttyTHS1";
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("open /dev/ttyTHS1");
    return 1;
  }

  if (set_serial(fd, 115200) != 0) {
    close(fd);
    return 1;
  }

  printf("Opened %s @115200 8N1\n", dev);

  // Send GO
  if (write_all(fd, "GO\n") != 0) {
    close(fd);
    return 1;
  }
  printf("Sent: GO\\n\n");
  printf("Commands: g=GO, s=STOP, q=quit (press key then Enter)\n\n");

  char rxbuf[256];
  char inbuf[64];

  while (1) {
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(fd, &rset);
    FD_SET(STDIN_FILENO, &rset);

    int maxfd = (fd > STDIN_FILENO) ? fd : STDIN_FILENO;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms

    int r = select(maxfd + 1, &rset, NULL, NULL, &tv);
    if (r < 0) {
      if (errno == EINTR) continue;
      perror("select");
      break;
    }

    if (FD_ISSET(fd, &rset)) {
      ssize_t n = read(fd, rxbuf, sizeof(rxbuf) - 1);
      if (n > 0) {
        rxbuf[n] = '\0';
        printf("STM32: %s", rxbuf); // STM32 prints \r\n
        fflush(stdout);
      }
    }

    if (FD_ISSET(STDIN_FILENO, &rset)) {
      if (!fgets(inbuf, sizeof(inbuf), stdin)) break;
      if (inbuf[0] == 'q') break;
      if (inbuf[0] == 'g') {
        write_all(fd, "GO\n");
        printf("Sent: GO\\n\n");
      } else if (inbuf[0] == 's') {
        write_all(fd, "STOP\n");
        printf("Sent: STOP\\n\n");
      } else {
        printf("Unknown. Use g/s/q.\n");
      }
    }
  }

  close(fd);
  return 0;
}
