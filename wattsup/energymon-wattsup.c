/**
* Energy reading from a Watts Up? Power Meter.
* Written for communications protocol serial data format 1.8.
* See: ./etc/CommunicationsProtocol090824.pdf
*
* @author Connor Imes
* @date 2016-02-08
*/
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "energymon.h"
#include "energymon-time-util.h"
#include "energymon-wattsup.h"
#include "wattsup-driver.h"

#ifdef ENERGYMON_DEFAULT
#include "energymon-default.h"
int energymon_get_default(energymon* em) {
  return energymon_get_wattsup(em);
}
#endif

// WattsUp values refresh every second
#define WU_MIN_INTERVAL_US 1000000
// we will poll the device 10x faster - data is often available (even if it hasn't changed)
#define WU_POLL_INTERVAL_US 100000
#define WU_POWER_INDEX 3
// buffer is large enough to hold a handful of complete data packets (usually ~80 bytes)
#define WU_BUFSIZE 256
// max number of attempts to find a complete data packet during init
#define WU_INIT_MAX_RETRIES 5
// configurations for handling incomplete data packets
#define WU_PACKET_MAX_RETRIES 10
#define WU_PACKET_WAIT_INTERVAL_US 10000

typedef struct energymon_wattsup {
  energymon_wattsup_ctx* ctx;
  pthread_t thread;
  uint64_t total_uj;
  int poll;
} energymon_wattsup;

// Only for use by the polling thread - enables pthread cancel while sleeping, then disables it
static int wattsup_thread_sleep_us(uint64_t us, volatile const int* poll) {
  assert(poll != NULL);
  int ret = 0;
  if (*poll) {
#ifndef __ANDROID__
    // Deadlock can occur during disconnect in some wattsup_driver impls if thread is canceled during I/O
    // Enable thread cancel while sleeping, disable during I/O
    int dummy_old_state;
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &dummy_old_state);
#endif
    ret = energymon_sleep_us(us, poll);
#ifndef __ANDROID__
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &dummy_old_state);
#endif
  }
  return ret;
}

// buf must be the start of a valid packet, i.e., begin with a '#'
static int data_packet_parse(char* buf, unsigned int* deciwatts) {
  assert(buf != NULL);
  assert(buf[0] == '#');
  assert(deciwatts != NULL);
  unsigned int tmp;
  int i;
  char* saveptr;
  char* tok = strtok_r(buf, ",", &saveptr);
  for (i = 0; tok; i++) {
    // 1st index = command, starting with a '#'
    // 2nd index = subcommand (should be a '-', but we don't care)
    if (i == WU_POWER_INDEX) {
      // treat this is as the average power since last successful read
      errno = 0;
      tmp = strtoul(tok, NULL, 0);
      if (errno) {
        // keep old value of deciwatts
        perror("data_packet_parse: strtoul");
        return -1;
      }
      *deciwatts = tmp;
      // ignore the rest of the packet
      return 0;
    }
    tok = strtok_r(NULL, ",", &saveptr);
  }
  errno = EBADMSG;
  perror("data_packet_parse: Syntax error while parsing WattsUp data packet");
  return -1;
}

// Returns a pointer (in buf) to the start of a valid packet, NULL otherwise
static char* data_packet_read(energymon_wattsup_ctx* ctx, char* buf, size_t bufsize, const int* poll) {
  assert(ctx != NULL);
  assert(buf != NULL);
  assert(poll != NULL);
  char* start;
  size_t len;
  int ret;
  int i = 0;
  if ((ret = wattsup_read(ctx, buf, bufsize - 1)) < 0) {
    perror("data_packet_read: wattsup_read");
    return NULL;
  }
  buf[ret] = '\0';
#ifdef ENERGYMON_WATTSUP_DEBUG
  fprintf(stdout, "Read %d characters:\n%s\n", ret, buf);
#endif
  if (!(*poll)) {
    // we were probably ordered to stop during I/O
    return NULL;
  }
  // try to find the start of the latest packet (ignore any prior packets that might be in the buffer)
  if (!(start = strrchr(buf, '#'))) {
    // first 2 bytes may be modem status codes (libftdi handles this for us)
    if (ret > 2) {
      // we probably got the remainder of a previously incomplete packet
      errno = EBADMSG;
      perror("Invalid data packet from WattsUp");
    } else {
      // data is not ready - this is common when polling faster than device's refresh rate
      errno = ENODATA;
    }
    // nothing we can do except try again later once the device refreshes
    return NULL;
  }
  // check for the end of the data packet (';')
  for (i = 0; !strchr(start, ';') && i < WU_PACKET_MAX_RETRIES; i++) {
    if ((len = strlen(buf)) >= bufsize - 1) {
      errno = ENOBUFS;
      perror("data_packet_read: Too much data");
      return NULL;
    }
    // short wait before reading again to try and get the rest of the packet
    wattsup_thread_sleep_us(WU_PACKET_WAIT_INTERVAL_US, poll);
    if (!(*poll)) {
      // we were probably ordered to stop during I/O or sleep
      return NULL;
    }
    if ((ret = wattsup_read(ctx, buf + len, bufsize - len - 1)) < 0) {
      perror("data_packet_read: wattsup_read");
      return NULL;
    }
    buf[len + ret] = '\0';
#ifdef ENERGYMON_WATTSUP_DEBUG
    fprintf(stdout, "Read %d additional characters:\n%s\n", ret, buf + len);
#endif
  }
  if (i >= WU_PACKET_MAX_RETRIES) {
    fprintf(stderr, "Failed to get complete data packet from WattsUp\n");
    errno = ENODATA;
    return NULL;
  }
  return start;
}

/**
 * pthread function to poll the device at regular intervals.
 * Always makes a best-effort to not read or sleep once ordered to stop (state->poll = 0).
 */
static void* wattsup_poll_sensors(void* args) {
  energymon_wattsup* state = (energymon_wattsup*) args;
  char buf[WU_BUFSIZE] = { 0 };
  char* pstart;
  uint64_t exec_us;
  uint64_t last_us;
  unsigned int deciwatts = 0;
  if (!(last_us = energymon_gettime_us())) {
    // must be that CLOCK_MONOTONIC is not supported
    perror("wattsup_poll_sensors: energymon_gettime_us");
    return (void*) NULL;
  }
  wattsup_thread_sleep_us(WU_POLL_INTERVAL_US, &state->poll);
  while (state->poll) {
    if ((pstart = data_packet_read(state->ctx, buf, sizeof(buf), &state->poll))) {
      data_packet_parse(pstart, &deciwatts);
    }
    exec_us = energymon_gettime_elapsed_us(&last_us);
    state->total_uj += deciwatts * exec_us / 10;
    wattsup_thread_sleep_us(WU_POLL_INTERVAL_US, &state->poll);
  }
  return (void*) NULL;
}

static int wattsup_flush_read(energymon_wattsup_ctx* ctx) {
  assert(ctx != NULL);
  char buf[WU_BUFSIZE] = { 0 };
  const int IGNORE_INTERRUPT = 0;
  int i;
  int ret;
  // try to get one good data packet from the device
  for (i = 0; i <= WU_INIT_MAX_RETRIES; i++) {
    if ((ret = wattsup_read(ctx, buf, WU_BUFSIZE)) < 0) {
      return -1;
    }
    buf[ret < WU_BUFSIZE ? ret : WU_BUFSIZE - 1] = '\0';
#ifdef ENERGYMON_WATTSUP_DEBUG
    fprintf(stdout, "Looking for good data packet: %d: %s\n", i, buf);
#endif
    if (ret == WU_BUFSIZE) {
      // cut through the data backlog (shouldn't be a problem if buffers were properly flushed)
      continue;
    }
    // good packets start with a '#' and end with a ';'
    if (strrchr(buf, '#') && strrchr(buf, ';')) {
      break;
    }
    if (i == WU_INIT_MAX_RETRIES) {
      return -1; // avoid unnecessary sleep
    }
    energymon_sleep_us(WU_MIN_INTERVAL_US, &IGNORE_INTERRUPT);
  }
  return 0;
}

int energymon_init_wattsup(energymon* em) {
  if (em == NULL || em->state != NULL) {
    errno = EINVAL;
    return -1;
  }

  int err_save;
  const char* dev_file = getenv(ENERGYMON_WATTSUP_DEV_FILE);
  if (dev_file == NULL) {
    dev_file = ENERGYMON_WATTSUP_DEV_FILE_DEFAULT;
  }

  // create the state struct
  energymon_wattsup* state = calloc(1, sizeof(energymon_wattsup));
  if (state == NULL) {
    return -1;
  }

  // connect to the device
  if ((state->ctx = wattsup_connect(dev_file, ENERGYMON_WATTSUP_TIMEOUT_MS)) == NULL) {
    free(state);
    return -1;
  }

  // clear device memory
  if (wattsup_write(state->ctx, WU_CLEAR, strlen(WU_CLEAR)) < 0) {
    wattsup_disconnect(state->ctx);
    free(state);
    return -1;
  }

  // start device logging
  if (wattsup_write(state->ctx, WU_LOG_START_EXTERNAL, strlen(WU_LOG_START_EXTERNAL)) < 0) {
    wattsup_disconnect(state->ctx);
    free(state);
    return -1;
  }

  // dummy reads - sometimes we get a bunch of junk to start with
  if (wattsup_flush_read(state->ctx)) {
    fprintf(stderr, "energymon_init_wattsup: Too much or no data from WattsUp\n");
    wattsup_disconnect(state->ctx);
    free(state);
    errno = ENODATA;
    return -1;
  }

  // start polling thread
  state->poll = 1;
  err_save = pthread_create(&state->thread, NULL, wattsup_poll_sensors, state);
  if (err_save) {
    wattsup_disconnect(state->ctx);
    free(state);
    errno = err_save;
    return -1;
  }

  em->state = state;
  return 0;
}

int energymon_finish_wattsup(energymon* em) {
  if (em == NULL || em->state == NULL) {
    errno = EINVAL;
    return -1;
  }

  int err_save = 0;
  energymon_wattsup* state = (energymon_wattsup*) em->state;

  // stop sensors polling thread and cleanup
  if (state->poll) {
    state->poll = 0;
#ifndef __ANDROID__
    pthread_cancel(state->thread);
#endif
    err_save = pthread_join(state->thread, NULL);
  }

  if (state->ctx != NULL) {
    // stop logging
    wattsup_write(state->ctx, WU_LOG_STOP, strlen(WU_LOG_STOP));
    // disconnect from device
    if (wattsup_disconnect(state->ctx)) {
      err_save = err_save ? err_save : errno;
    }
  }

  em->state = NULL;
  free(state);
  errno = err_save;

  return errno ? -1 : 0;
}

uint64_t energymon_read_total_wattsup(const energymon* em) {
  if (em == NULL || em->state == NULL) {
    errno = EINVAL;
    return 0;
  }
  return ((energymon_wattsup*) em->state)->total_uj;
}

char* energymon_get_source_wattsup(char* buffer, size_t n) {
  return wattsup_get_implementation(buffer, n);
}

uint64_t energymon_get_interval_wattsup(const energymon* em) {
  if (em == NULL) {
    // we don't need to access em, but it's still a programming error
    errno = EINVAL;
    return 0;
  }
  return WU_MIN_INTERVAL_US;
}

uint64_t energymon_get_precision_wattsup(const energymon* em) {
  if (em == NULL) {
    errno = EINVAL;
    return 0;
  }
  // deciwatts at 1 second interval
  return WU_MIN_INTERVAL_US / 10;
}

int energymon_is_exclusive_wattsup(void) {
  return 1;
}

int energymon_get_wattsup(energymon* em) {
  if (em == NULL) {
    errno = EINVAL;
    return -1;
  }
  em->finit = &energymon_init_wattsup;
  em->fread = &energymon_read_total_wattsup;
  em->ffinish = &energymon_finish_wattsup;
  em->fsource = &energymon_get_source_wattsup;
  em->finterval = &energymon_get_interval_wattsup;
  em->fprecision = &energymon_get_precision_wattsup;
  em->fexclusive = &energymon_is_exclusive_wattsup;
  em->state = NULL;
  return 0;
}
