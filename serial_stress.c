// Copyright (c) StarLead Ltd, 2019

#include <fcntl.h>
#include <stdio.h>
#include <sys/epoll.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <termios.h>

#define DBG(x, ...) fprintf(stderr, x "\n", ##__VA_ARGS__)
#define ARRAY_LENGTH(x) (sizeof(x)/sizeof((x)[0]))

typedef struct {
  size_t count;
  size_t ops;

  uint64_t tick_start_us;
  size_t bytes_in_tick;
} stats_t;

typedef struct {
  int serial_fd;
  stats_t stats[2];
  uint8_t send_seq;
  uint8_t recv_seq;
} ctx_t;

uint64_t time_get_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * (1000 * 1000) + ts.tv_nsec / 1000;
}

typedef enum {
  DIR_RX = 0,
  DIR_TX = 1,
} dir_t;

static const char *dir_to_string(dir_t dir)
{
  switch (dir) {
    case DIR_RX: return "RX";
    case DIR_TX: return "TX";
  }
  return NULL;
}

enum {
  STATS_TICK = 100,
  RATELIMIT_BYTES_IN_TICK = 8000,
};

static void stats_next(ctx_t *ctx, dir_t dir, size_t count)
{
  stats_t *stats = ctx->stats + dir;
  stats->bytes_in_tick += count;
  stats->count += count;
  stats->ops++;

  uint64_t now = time_get_us();
  const uint64_t ival_us = now - stats->tick_start_us;
  const uint64_t one_second = 1000 * 1000;

  if (ival_us >= one_second) {
    const double ival_secs = (double)ival_us / (double)one_second;
    const double bitrate = (stats->bytes_in_tick * 8.0) / ival_secs;
    const double bitrate_kbps = bitrate / 1024.0;

    DBG("%s: tick(bytes=%zu, secs=%.02f, kbps=%.02f), total(ops=%zu, bytes=%zu)",
      dir_to_string(dir),
      stats->bytes_in_tick,
      ival_secs,
      bitrate_kbps,
      stats->ops,
      stats->count);

    stats->tick_start_us = time_get_us();
    stats->bytes_in_tick = 0;
  }
}

static void close_dev(ctx_t *ctx)
{
  assert(ctx->serial_fd != -1);
  close(ctx->serial_fd);
  ctx->serial_fd = -1;
}

static void do_read(ctx_t *ctx)
{
  uint8_t buf[512];

  ssize_t r_zd = read(ctx->serial_fd, buf, sizeof(buf)-1);
  if (r_zd < 0) {
    if (errno == EAGAIN)
      return;

    perror("read");
    close_dev(ctx);
    return;
  }

  if (!r_zd) {
    DBG("read() -> EOF, closing device");
    close_dev(ctx);
    return;
  }

  size_t r = r_zd;
  assert(r <= sizeof(buf));

  for (size_t i = 0; i < r; i++) {
    const uint8_t exp = ctx->recv_seq + 1;
    if (ctx->recv_seq && buf[i] != exp) {
      DBG("[%zu/%zu]: expected 0x%x, got 0x%x", i, r, exp, buf[i]);
    }
    ctx->recv_seq = buf[i];
  }

  stats_next(ctx, DIR_RX, r);
}

static void do_write(ctx_t *ctx)
{
  uint8_t buf[256];
  for (unsigned i = 0; i < sizeof(buf); i++)
    buf[i] = i + ctx->send_seq;

  const size_t to_write = sizeof(buf);
  ssize_t w_zd = write(ctx->serial_fd, buf, to_write);
  if (w_zd < 0) {
    if (errno != EAGAIN) {
      fprintf(stderr, "write() -> %zd: %m", w_zd);
      close_dev(ctx);
    }
    return;
  }

  size_t w = w_zd;
  assert(w);
  ctx->send_seq += w;
  stats_next(ctx, DIR_TX, w);
}

static void setup_raw_serial_device(int fd)
{
  assert(isatty(fd));

  struct termios termios;
  memset(&termios, 0, sizeof(termios));
  assert(!tcgetattr(fd, &termios));

  /* input modes - clear indicated ones giving: no break, no CR to NL,
     no parity check, no strip char, no start/stop output (sic) control */
  termios.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  /* output modes - clear giving: no post processing such as NL to CR+NL */
  termios.c_oflag &= ~(OPOST);

  /* control modes - set 8 bit chars */
  termios.c_cflag |= (CS8);

  /* local modes - clear giving: echoing off, canonical off (no erase with
     backspace, ^U,...),  no extended functions, no signal chars (^Z,^C) */
  termios.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

  termios.c_cc[VMIN] = 4; // reads can return as soon as we have 4 bytes.
  termios.c_cc[VTIME] = 0;  // no timeout.

  assert(!tcsetattr(fd, TCSANOW, &termios));
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    DBG("Usage: serial_stress <serial_dev_path>");
    return 1;
  }

  ctx_t ctx = {
    .serial_fd = -1,
  };

  const char *dev = argv[1];

  int epoll_fd = epoll_create1(0);
  if (epoll_fd == -1) {
    DBG("epoll_create(): %m");
    return 1;
  }

  ctx.stats[0].tick_start_us = time_get_us();
  ctx.stats[1].tick_start_us = time_get_us();

  while (1) {
    bool was_closed = ctx.serial_fd == -1;
    while (ctx.serial_fd == -1) {
      ctx.serial_fd = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY);
      if (ctx.serial_fd == -1) {
        DBG("open(%s): %m", dev);
        sleep(1);
        continue;
      }
    }

    if (was_closed) {
      DBG("opened %s", dev);
      setup_raw_serial_device(ctx.serial_fd);

      struct epoll_event serial_ev;
      serial_ev.events = EPOLLIN | EPOLLOUT;
      serial_ev.data.fd = ctx.serial_fd;

      if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ctx.serial_fd, &serial_ev) == -1) {
        perror("epoll_ctl");
        return 1;
      }
    }

    struct epoll_event events[1];
    memset(events, 0, sizeof(events));

    int ret = epoll_wait(epoll_fd, events, ARRAY_LENGTH(events), -1);
    if (ret < 1) {
      perror("epoll_wait");
      return 1;
    }

    assert(events[0].data.fd == ctx.serial_fd);
    assert(events[0].events & (EPOLLIN | EPOLLOUT));

    if (events[0].events & EPOLLIN) {
      do_read(&ctx);
    }
    if (ctx.serial_fd != -1 && events[0].events & EPOLLOUT) {
      do_write(&ctx);
    }
  }

  return 0;
}
