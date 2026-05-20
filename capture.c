/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>

#include <syslog.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <semaphore.h>

/******************************************************************************/
// Macros / Constants
/******************************************************************************/

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 320
#define VRES 240
#define HRES_STR "320"
#define VRES_STR "240"

#define NUM_REQ_BUFS 32
#define NUM_BIT_BUCKET_IMGS 10

#define SCHED_POLICY SCHED_FIFO
#define SEQ_CPU 2
#define SERVICE_CPU 3
#define SEQ_FREQ 100
#define READ_FREQ 20
#define PROCESS_FREQ 5
#define WRITE_FREQ 1
#define NSEC_PER_SEC 1000000000L
#define NSEC_PER_USEC 1000L

#define CLOCKID CLOCK_MONOTONIC_RAW

static const uint32_t SEQ_PERIOD_NS = NSEC_PER_SEC/SEQ_FREQ;

/******************************************************************************/
// Types
/******************************************************************************/

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};

// Image buffer struct
struct frame_buf {
    unsigned char *start;
    size_t bytesused;
    struct timeval timestamp;
};

// Ring buffer struct
struct ring_buf {
    struct frame_buf *start;
    size_t size;
    size_t head;
    size_t tail;
    int head_wraps;
    int tail_wraps;
};

/* Thread argument structs */
struct seq_thread_arg {
    sem_t *seq_sem;
    sem_t *read_sem;
    sem_t *process_sem;
    sem_t *write_sem;
    uint8_t *read_finished;
    uint8_t *processing_finished;
    uint8_t *write_finished;
};
struct read_thread_arg {
    sem_t *sem;
    uint8_t *finished;
};
struct process_thread_arg {
    sem_t *sem;
    struct ring_buf *selected_frame_bufs;
    uint8_t *read_finished;
    uint8_t *processing_finished;
};
struct write_thread_arg {
    sem_t *sem;
    struct ring_buf *selected_frame_bufs;
    uint8_t *finished;
};

/******************************************************************************/
// Globals
/******************************************************************************/

static const char short_options[] = "d:hmruofc:";

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct ring_buf         raw_frame_bufs;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 180;

static int read_frames, process_frames, write_frames;
struct timespec start_time;
uint32_t frame_buf_length;

char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[]="test00000000.pgm";

char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="test00000000.ppm";

/******************************************************************************/
// Function Prototypes
/******************************************************************************/

static void open_device(void);
static void init_device(void);
static void init_mmap(void);
static void start_capturing(void);
static void run_threads(void);
static void init_ring_buffer(struct ring_buf *selected_frame_bufs);
static void free_ring_buffers(struct ring_buf *raw_ring_buf, struct ring_buf *selected_frame_bufs);
static void timer_handler(union sigval arg);
static void *sequencer(void *arg);
void *read_img_thread(void *arg);
static int read_img(int32_t read_count);
void *process_img_thread(void *arg);
static void process_img(struct ring_buf *selected_frame_bufs);
void *write_img_thread(void *arg);
void write_img(struct frame_buf *processed_img, int32_t write_count);
static void stop_capturing(void);
static void uninit_device(void);
static void close_device(void);

static void usage(FILE *fp, int argc, char **argv);
static void errno_exit(const char *s);
static int xioctl(int fh, int request, void *arg);
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time);
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time);
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b);
void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b);
double get_delta_time_real(struct timespec prev_time);

/******************************************************************************/
// Function Definitions
/******************************************************************************/

int main(int argc, char **argv)
{
    syslog(LOG_INFO, "====================STARTING CAPTURE.C====================\n");
    // Set dev_name
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    // Get command-line options
    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    read_frames = READ_FREQ * frame_count;
    process_frames = PROCESS_FREQ * frame_count;
    write_frames = WRITE_FREQ * frame_count;

    open_device();
    init_device();
    start_capturing();
    run_threads();
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}

/* Opens the camera for read/write */
static void open_device(void)
{
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
            fprintf(stderr, "Cannot open '%s': %d, %s\n",
                     dev_name, errno, strerror(errno));
            exit(EXIT_FAILURE);
        }
}

/* Configures camera for streaming and mmap */
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n",
                    dev_name);
        exit(EXIT_FAILURE);
    }

    /* Select video input, video standard and tune here. */

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    init_mmap();
}

/* Configures camera for mmap and allocates and maps buffers */
static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = NUM_REQ_BUFS;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        // Initialize raw_frame_bufs ring buffer
        raw_frame_bufs.start = calloc(req.count, sizeof(struct frame_buf));
        if (!raw_frame_bufs.start) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        raw_frame_bufs.size = req.count;
        printf("raw_frame_bufs.size = %d\n", req.count);
        raw_frame_bufs.head = 0;
        raw_frame_bufs.tail = 0;
        raw_frame_bufs.head_wraps = 0;
        raw_frame_bufs.tail_wraps = 0;

        // Map device buffers into memory
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                frame_buf_length = buf.length;

                raw_frame_bufs.start[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == raw_frame_bufs.start[n_buffers].start)
                        errno_exit("mmap");
        }
}

/* Enqueues camera buffers and turns stream on */
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        for (i = 0; i < n_buffers; ++i) 
        {
                struct v4l2_buffer buf;

                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;

                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                errno_exit("VIDIOC_STREAMON");
}

/* Configures and runs service threads until completion */
static void run_threads(void)
{
    int r;
    struct ring_buf selected_frame_bufs;
    cpu_set_t cpu_set;
    int max_prio;
    struct sched_param s_param;
    sem_t seq_sem, read_sem, process_sem, write_sem;
    pthread_attr_t thread_attr;
    struct seq_thread_arg seq_targ;
    struct read_thread_arg read_targ;
    struct process_thread_arg process_targ;
    struct write_thread_arg write_targ;
    pthread_t seq_thread_id, read_thread_id, process_thread_id, write_thread_id;
    uint8_t read_finished = 0, processing_finished = 0, write_finished = 0;

    // Initialize the ring buffer
    init_ring_buffer(&selected_frame_bufs);

    // Initialize the semaphores
    r = sem_init(&seq_sem, 0, 0);
    r |= sem_init(&read_sem, 0, 0);
    r |= sem_init(&process_sem, 0, 0);
    r |= sem_init(&write_sem, 0, 0);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to initialize semaphores: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Create the threads
    // Configure all threads with same scheduling policy
    r = pthread_attr_init(&thread_attr);
    r |= pthread_attr_setinheritsched(&thread_attr, PTHREAD_EXPLICIT_SCHED);
    r |= pthread_attr_setschedpolicy(&thread_attr, SCHED_POLICY);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }
    max_prio = sched_get_priority_max(SCHED_POLICY);

    // Create sequencer thread and assign to its own core
    CPU_ZERO(&cpu_set);
    CPU_SET(SEQ_CPU, &cpu_set);
    r = pthread_attr_setaffinity_np(&thread_attr, sizeof(cpu_set), &cpu_set);
    s_param.sched_priority = max_prio - 1;
    r |= pthread_attr_setschedparam(&thread_attr, &s_param);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }
    seq_targ.seq_sem = &seq_sem;
    seq_targ.read_sem = &read_sem;
    seq_targ.process_sem = &process_sem;
    seq_targ.write_sem = &write_sem;
    seq_targ.read_finished = &read_finished;
    seq_targ.processing_finished = &processing_finished;
    seq_targ.write_finished = &write_finished;
    pthread_create(&seq_thread_id, &thread_attr, sequencer, &seq_targ);

    // Assign all service threads to same core
    CPU_ZERO(&cpu_set);
    CPU_SET(SERVICE_CPU, &cpu_set);
    r = pthread_attr_setaffinity_np(&thread_attr, sizeof(cpu_set), &cpu_set);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }

    // Create the read thread
    s_param.sched_priority = max_prio;
    r = pthread_attr_setschedparam(&thread_attr, &s_param);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure prio thread attribute\n");
        exit(EXIT_FAILURE);
    }
    read_targ.sem = &read_sem;
    read_targ.finished = &read_finished;
    pthread_create(&read_thread_id, &thread_attr, read_img_thread, &read_targ);

    // Create the process thread
    s_param.sched_priority = max_prio - 1;
    r = pthread_attr_setschedparam(&thread_attr, &s_param);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }
    process_targ.sem = &process_sem;
    process_targ.selected_frame_bufs = &selected_frame_bufs;
    process_targ.read_finished = &read_finished;
    process_targ.processing_finished = &processing_finished;
    pthread_create(&process_thread_id, &thread_attr, process_img_thread, &process_targ);

    // Create the write thread
    s_param.sched_priority = max_prio - 2;
    r = pthread_attr_setschedparam(&thread_attr, &s_param);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }
    write_targ.sem = &write_sem;
    write_targ.selected_frame_bufs = &selected_frame_bufs;
    write_targ.finished = &write_finished;
    pthread_create(&write_thread_id, &thread_attr, write_img_thread, &write_targ);

    /* Wait for the threads to terminate */
    r = pthread_join(read_thread_id, NULL);
    r |= pthread_join(process_thread_id, NULL);
    r |= pthread_join(write_thread_id, NULL);
    r |= pthread_join(seq_thread_id, NULL);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to join threads\n");
        exit(EXIT_FAILURE);
    }

    // Destroy the semaphores
    r = sem_destroy(&seq_sem);
    r |= sem_destroy(&read_sem);
    r |= sem_destroy(&process_sem);
    r |= sem_destroy(&write_sem);
    if (r) {
        fprintf(stderr, "Error run_threads: Failed to destroy semaphores: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // TODO: Free ring buffers memory
    //free_ring_buffers(&raw_ring_buf, &selected_frame_bufs);
}

/* Allocates memory for the ring buffers */
static void init_ring_buffer(struct ring_buf *selected_frame_bufs)
{
    // Get necessary size of ring buffer
    size_t ring_buf_size;
    ring_buf_size = frame_count - ((frame_count / PROCESS_FREQ) - 1) * WRITE_FREQ;
    ring_buf_size += NUM_BIT_BUCKET_IMGS;
    printf("selected_frame_bufs.size = %lu\n", ring_buf_size);

    // Allocate ring buffer
    selected_frame_bufs->start = (struct frame_buf *)malloc(ring_buf_size * sizeof(struct frame_buf));
    if (selected_frame_bufs->start == NULL) {
        fprintf(stderr, "Error init_ring_buffers: Failed to allocate write ring buffer\n");
        exit(EXIT_FAILURE);
    }

    // Allocate image buffers within ring buffer
    for (int i = 0; i < ring_buf_size; i++) {
        selected_frame_bufs->start[i].start = (unsigned char *)malloc(frame_buf_length * sizeof(unsigned char));

        if (selected_frame_bufs->start[i].start == NULL) {
            fprintf(stderr, "Error init_ring_buffer: Failed to allocate image buffer within ring buffer\n");
            for (int j = 0; j < i; j++) {
                free(selected_frame_bufs->start[j].start);
            }
            free(selected_frame_bufs->start);
            exit(EXIT_FAILURE);
        }
    }

    selected_frame_bufs->size = ring_buf_size;
    selected_frame_bufs->head = 0;
    selected_frame_bufs->tail = 0;
    selected_frame_bufs->head_wraps = 0;
    selected_frame_bufs->tail_wraps = 0;
}

/* TODO: Frees ring buffers memory */
static void free_ring_buffers(struct ring_buf *raw_ring_buf, struct ring_buf *selected_frame_bufs)
{
}

/* Releases sequencer thread when timer expires */
static void timer_handler(union sigval arg)
{
    sem_t *seq_sem = arg.sival_ptr;

    if (sem_post(seq_sem)) {
        fprintf(stderr, "Error timer_handler: Failed to post seq_sem: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

/* Releases services at corresponding frequencies */
static void *sequencer(void *arg)
{
    int r;
    struct seq_thread_arg *targ = (struct seq_thread_arg *)arg;
    sem_t *seq_sem = targ->seq_sem, *read_sem = targ->read_sem,
          *process_sem = targ->process_sem, *write_sem = targ->write_sem;
    uint8_t *read_finished = targ->read_finished,
            *processing_finished = targ->processing_finished,
            *write_finished = targ->write_finished;
    pthread_attr_t timer_handler_attr;
    cpu_set_t cpu_set;
    int max_prio;
    struct sched_param s_param;
    clockid_t clockid = CLOCK_MONOTONIC;
    timer_t timerid;
    struct sigevent sev;
    struct itimerspec its = {{0, SEQ_PERIOD_NS}, {0, SEQ_PERIOD_NS}};
    uint32_t seq_count = 0;
    const uint8_t READ_RELEASE_FREQ = SEQ_FREQ/READ_FREQ,
          PROCESS_RELEASE_FREQ = SEQ_FREQ/PROCESS_FREQ,
          WRITE_RELEASE_FREQ = SEQ_FREQ/WRITE_FREQ;

    // Initialize timer handler pthread attributes
    CPU_ZERO(&cpu_set);
    CPU_SET(SEQ_CPU, &cpu_set);
    max_prio = sched_get_priority_max(SCHED_POLICY);
    s_param.sched_priority = max_prio;
    r = pthread_attr_init(&timer_handler_attr);
    r |= pthread_attr_setinheritsched(&timer_handler_attr, PTHREAD_EXPLICIT_SCHED);
    r |= pthread_attr_setschedpolicy(&timer_handler_attr, SCHED_POLICY);
    r |= pthread_attr_setaffinity_np(&timer_handler_attr, sizeof(cpu_set), &cpu_set);
    r |= pthread_attr_setschedparam(&timer_handler_attr, &s_param);
    if (r) {
        fprintf(stderr, "Error sequencer: Failed to configure thread attributes\n");
        exit(EXIT_FAILURE);
    }
    
    // Create timer
    memset(&sev, sizeof(sev), 0);
    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_value.sival_ptr = seq_sem;
    sev.sigev_notify_function = timer_handler;
    sev.sigev_notify_attributes = &timer_handler_attr;
    r = timer_create(clockid, &sev, &timerid);
    if (r) {
        fprintf(stderr, "Error sequencer: Failed to create timer: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
    // Arm timer
    r = timer_settime(timerid, 0, &its, NULL);
    if (r) {
        fprintf(stderr, "Error sequencer: Failed to arm timer: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Get services start time
    r = clock_gettime(CLOCKID, &start_time);
    if (r) {
        fprintf(stderr, "Error sequencer: Failed to get start time: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Sequencer loop
    while (!(*read_finished & *processing_finished & *write_finished)) {
        seq_count++;

        // Wait for release by timer_handler
        r = sem_wait(seq_sem);
        if (r) {
            fprintf(stderr, "Error sequencer: Failed to wait for semaphore: %d, %s\n",
                    errno, strerror(errno));
            exit(EXIT_FAILURE);
        }

        // Release read semaphore
        if ((!*read_finished) && (seq_count % READ_RELEASE_FREQ == 0)) {
            r = sem_post(read_sem);
            if (r) {
                fprintf(stderr, "Error sequencer: Failed to post read semaphore: %d, %s\n",
                        errno, strerror(errno));
                exit(EXIT_FAILURE);
            }
        }

        // Release process semaphore
        if ((!*processing_finished) && (seq_count % PROCESS_RELEASE_FREQ == 0)) {
            r = sem_post(process_sem);
            if (r) {
                fprintf(stderr, "Error sequencer: Failed to post process semaphore: %d, %s\n",
                        errno, strerror(errno));
                exit(EXIT_FAILURE);
            }
        }

        // Release write semaphore
        if ((!*write_finished) && (seq_count % WRITE_RELEASE_FREQ == 0)) {
            r = sem_post(write_sem);
            if (r) {
                fprintf(stderr, "Error sequencer: Failed to post write semaphore: %d, %s\n",
                        errno, strerror(errno));
                exit(EXIT_FAILURE);
            }
        }
    }

    // Disarm/delete timer
    r = timer_delete(timerid);
    if (r) {
        fprintf(stderr, "Error sequencer: Failed to disarm/delete timer: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void *read_img_thread(void *arg)
{
    double delta_time_real;
    struct read_thread_arg *targ = (struct read_thread_arg *)arg;
    sem_t *read_sem = targ->sem;
    uint8_t *read_finished = targ->finished;
    int32_t read_count = -NUM_BIT_BUCKET_IMGS;
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

    // Service loop
    while (1)
    {
        // Wait for release by sequencer
        if (sem_wait(read_sem) == -1) {
            fprintf(stderr, "Error read_img_thread: Failed to wait for read_sem: %d, %s\n",
                    errno, strerror(errno));
            exit(EXIT_FAILURE);
        }

        // Log time
        delta_time_real = get_delta_time_real(start_time);
        syslog(LOG_INFO, "Reading Frame %d @ %lf", read_count, delta_time_real);

        // Repeatedly call read() until succeeds
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            // Wait until camera device ready to read
            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_img(read_count))
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");

                read_count++;

                // Read succeeded
                break;
            }

            // Read failed; try again
        }

        if(read_count >= read_frames)
            break;
    }

    *read_finished = 1;
}

static int read_img(int32_t read_count)
{
    int r;
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    // Dequeue frame buffer
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                    non-fatal errors too.
                    */
                return 0;

            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    // Check buffer index in range
    assert(buf.index < n_buffers);
    syslog(LOG_DEBUG, "Dequeued index %d\n", buf.index);

    // Check for buffer overflow
    if ((raw_frame_bufs.head == raw_frame_bufs.tail) &&
            (raw_frame_bufs.head_wraps > raw_frame_bufs.tail_wraps)) {
        fprintf(stderr, "Error read_img: Ring buffer overwrite\n");
        exit(EXIT_FAILURE);
    }

    // Update frame buffer parameters
    raw_frame_bufs.start[raw_frame_bufs.head].bytesused = buf.bytesused;
    raw_frame_bufs.start[raw_frame_bufs.head].timestamp = buf.timestamp;

    // Increment head position
    raw_frame_bufs.head++;
    // Check for wrap
    if (raw_frame_bufs.head >= raw_frame_bufs.size) {
        raw_frame_bufs.head = 0;
        raw_frame_bufs.head_wraps++;
    }

    //printf("R");
    return 1;
}

void *process_img_thread(void *arg)
{
    int r;
    double delta_time_real;
    struct process_thread_arg *targ = (struct process_thread_arg *)arg;
    sem_t *process_sem = targ->sem;
    struct ring_buf *selected_frame_bufs = targ->selected_frame_bufs;
    uint8_t *read_finished = targ->read_finished,
            *processing_finished = targ->processing_finished;
    int32_t process_count = -NUM_BIT_BUCKET_IMGS;
    struct v4l2_buffer buf;
    size_t process_window_head, process_window_tail;
    ssize_t best_frame_idx;

    while (1) {
        /* Wait for processing semaphore */
        if (sem_wait(process_sem) == -1) {
            fprintf(stderr, "Error process_img_thread: Failed to wait for process_sem: %d, %s\n",
                    errno, strerror(errno));
            exit(EXIT_FAILURE);
        }

        /* Check processing not ahead of reading */
        if (!*read_finished) {
            process_window_head = raw_frame_bufs.tail;
            process_window_tail = (process_window_head + READ_FREQ/PROCESS_FREQ) %
                raw_frame_bufs.size;
            if ((process_window_head <= raw_frame_bufs.head)
                    && (process_window_tail >= raw_frame_bufs.head)) {
                syslog(LOG_DEBUG, "Process window overtook read: ph = %d, pt = %d, r = %d\n",
                        process_window_head, process_window_tail, raw_frame_bufs.head);
                continue;
            }
            // Process window wrapped around ring buffer
            if (process_window_tail < process_window_head) {
                if ((process_window_head <= raw_frame_bufs.head)
                        || (process_window_tail >= raw_frame_bufs.head)) {
                    syslog(LOG_DEBUG, "Process window wrapped and overtook read\n");
                    continue;
                }
            }
        }

        /* Log time */
        delta_time_real = get_delta_time_real(start_time);
        syslog(LOG_INFO, "Processing Frame %d @ %lf", process_count, delta_time_real);
        
        /* Find best frame in raw_frame_bufs window */
        process_img(selected_frame_bufs);

        process_count++;
        if (process_count >= process_frames)
            break;
    }

    *processing_finished = 1;
}

static void process_img(struct ring_buf *selected_frame_bufs)
{
    struct v4l2_buffer buf;
    ssize_t best_frame_idx = -1;
    double percent_diff_threshold = 0.15;
    size_t i;
    static int num_windows_skipped = 0;
    static ssize_t last_best_frame_idx = 0;

    // Get percent diff between each consecutive pair of frames
    for (i = 0; i < READ_FREQ/PROCESS_FREQ; i++) {
        size_t frame1_idx = (raw_frame_bufs.tail + i) % raw_frame_bufs.size,
               frame2_idx = (frame1_idx + 1) % raw_frame_bufs.size;
        double average_diff, percent_diff;
        size_t frame_size = HRES * VRES;
        size_t num_ys =  frame_size / 2;    // Assuming YUYV format
        uint32_t max_val = 255;
        size_t byte_idx;
        unsigned char frame1_val, frame2_val;
        int64_t sum = 0;

        // Find percent diff between bytes
        for (byte_idx = 0; byte_idx < frame_size; byte_idx++) {
            frame1_val = raw_frame_bufs.start[frame1_idx].start[byte_idx];
            frame2_val = raw_frame_bufs.start[frame2_idx].start[byte_idx];
            if (frame1_val < frame2_val)
                sum += (frame2_val - frame1_val);
            else
                sum += (frame1_val - frame2_val);
        }

        average_diff = (double)sum / frame_size;
        percent_diff = (average_diff / max_val) * 100;
        syslog(LOG_DEBUG, "frame1 = %d, frame2 = %d, percent_diff = %lf\n",
                frame1_idx, frame2_idx, percent_diff);

        // If tick detected, get best frame index
        if (percent_diff >= percent_diff_threshold) {
            best_frame_idx = (frame1_idx + (READ_FREQ/PROCESS_FREQ)/2) % raw_frame_bufs.size;
            break;
        }
    }

    // If no tick detected, get next best frame
    if (best_frame_idx == -1) {
        num_windows_skipped++;
        if (num_windows_skipped == PROCESS_FREQ) {
            best_frame_idx = (last_best_frame_idx + READ_FREQ) % raw_frame_bufs.size;
            syslog(LOG_DEBUG, "Skipped %d windows; best_frame_idx = %lu\n",
                    num_windows_skipped, best_frame_idx);
        }
    }

    // If best frame found, copy frame to selected_frame_bufs
    if (best_frame_idx >= 0) {
        memcpy(&selected_frame_bufs->start[selected_frame_bufs->head],
                &raw_frame_bufs.start[best_frame_idx], sizeof(struct frame_buf));
        selected_frame_bufs->head = (selected_frame_bufs->head + 1) % selected_frame_bufs->size;
        
        num_windows_skipped = 0;
        last_best_frame_idx = best_frame_idx;
    }

    // Enqueue video frame buffers
    for (i = 0; i < READ_FREQ/PROCESS_FREQ; i++) {
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = raw_frame_bufs.tail;
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF process_img");
        syslog(LOG_DEBUG, "Enqueued index %d\n", buf.index);

        // Update raw_frame_bufs tail position
        raw_frame_bufs.tail++;
        if (raw_frame_bufs.tail >= raw_frame_bufs.size) {
            raw_frame_bufs.tail %= raw_frame_bufs.size;
            raw_frame_bufs.tail_wraps++;
        }
    }
}

void *write_img_thread(void *arg)
{
    int r;
    double delta_time_real;
    struct write_thread_arg *targ = (struct write_thread_arg *)arg;
    sem_t *write_sem = targ->sem;
    uint8_t *write_finished = targ->finished;
    struct ring_buf *selected_frame_bufs = targ->selected_frame_bufs;
    int32_t write_count = -NUM_BIT_BUCKET_IMGS;

    // Write image service loop
    while (1) {
        if (sem_wait(write_sem) == -1) {
            fprintf(stderr, "Error write_img_thread: Failed to wait for write_sem: %d, %s\n",
                    errno, strerror(errno));
            exit(EXIT_FAILURE);
        }

        // Bit-bucket first frames
        if (write_count < 0) {
            syslog(LOG_INFO, "Bit-bucket frame %d\n", write_count);

            selected_frame_bufs->tail = (selected_frame_bufs->tail + 1) % selected_frame_bufs->size;
            write_count++;

            continue;
        }

        /* Log time */
        delta_time_real = get_delta_time_real(start_time);
        syslog(LOG_INFO, "Writing Frame %d @ %lf", write_count, delta_time_real);

        // Write image to memory
        write_img(&selected_frame_bufs->start[selected_frame_bufs->tail], write_count);

        // Update tail position
        selected_frame_bufs->tail++;
        if (selected_frame_bufs->tail >= selected_frame_bufs->size) {
            selected_frame_bufs->tail %= selected_frame_bufs->size;
            selected_frame_bufs->tail_wraps++;
        }

        write_count++;
        if (write_count >= write_frames)
            break;
    }

    *write_finished = 1;
}

void write_img(struct frame_buf *processed_img, int32_t write_count)
{
    int r;
    int i, newi, newsize=0;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = processed_img->start;
    int size = processed_img->bytesused;
    struct timeval timestamp = processed_img->timestamp;
    unsigned char bigbuffer[(1280*960)];
    struct timespec cur_time = {timestamp.tv_sec, timestamp.tv_usec * NSEC_PER_USEC};

    /* Dump frame */
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
#if defined(COLOR_CONVERT)
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6) {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }

        dump_ppm(bigbuffer, ((size*6)/4), write_count, &cur_time);
#else
        printf("Dump YUYV converted to YY size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2) {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=pptr[i];
            bigbuffer[newi+1]=pptr[i+2];
        }

        dump_pgm(bigbuffer, (size/2), write_count, &cur_time);
#endif
    }
    else {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                errno_exit("VIDIOC_STREAMOFF");
}

static void uninit_device(void)
{
        unsigned int i;

        for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(raw_frame_bufs.start[i].start, frame_buf_length))
                        errno_exit("munmap");

        free(raw_frame_bufs.start);
}

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

/******************************************************************************/
// Helper Functions
/******************************************************************************/

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 //"-r | --read          Use read() calls\n"
                 //"-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[4], 9, "%08d", tag);
    strncat(&pgm_dumpname[12], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 because sizeof for string includes null terminator
    written=write(dumpfd, pgm_header, sizeof(pgm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);

    // subtract 1 because sizeof for string includes null terminator
    written=write(dumpfd, ppm_header, sizeof(ppm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    close(dumpfd);
    
}

/* Converts YUYV to RGB */
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}

void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}

double get_delta_time_real(struct timespec prev_time)
{
    double delta_time_real;
    struct timespec cur_time;
    time_t delta_sec;
    long delta_nsec;

    // Get current time
    if (clock_gettime(CLOCKID, &cur_time) == -1) {
        fprintf(stderr, "Error get_delta_time_real: Failed to get cur time: %d, %s\n",
                errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    delta_sec = cur_time.tv_sec - prev_time.tv_sec;
    delta_nsec = cur_time.tv_nsec - prev_time.tv_nsec;

    if (delta_nsec < 0) {
        delta_sec--;
        delta_nsec += NSEC_PER_SEC;
    }

    delta_time_real = (double)delta_sec + (double)delta_nsec/NSEC_PER_SEC;

    return delta_time_real;
}
