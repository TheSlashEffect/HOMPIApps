/* File generated from [torc_funcs.pc] by OMPi compiler with torc extensions user/chriskar/hompi_start-v2.0.0-434-gbffeec9~, Sat May 18 21:18:49 2019
$OMPi__nfo:0
*/
/* (l216) typedef long unsigned int size_t; */

/* (l30) typedef unsigned char __u_char; */

/* (l31) typedef unsigned short int __u_short; */

/* (l32) typedef unsigned int __u_int; */

/* (l33) typedef unsigned long int __u_long; */

/* (l36) typedef signed char __int8_t; */

/* (l37) typedef unsigned char __uint8_t; */

/* (l38) typedef signed short int __int16_t; */

/* (l39) typedef unsigned short int __uint16_t; */

/* (l40) typedef signed int __int32_t; */

/* (l41) typedef unsigned int __uint32_t; */

/* (l43) typedef signed long int __int64_t; */

/* (l44) typedef unsigned long int __uint64_t; */

/* (l52) typedef long int __quad_t; */

/* (l53) typedef unsigned long int __u_quad_t; */

/* (l124) typedef unsigned long int __dev_t; */

/* (l125) typedef unsigned int __uid_t; */

/* (l126) typedef unsigned int __gid_t; */

/* (l127) typedef unsigned long int __ino_t; */

/* (l128) typedef unsigned long int __ino64_t; */

/* (l129) typedef unsigned int __mode_t; */

/* (l130) typedef unsigned long int __nlink_t; */

/* (l131) typedef long int __off_t; */

/* (l132) typedef long int __off64_t; */

/* (l133) typedef int __pid_t; */

struct _noname0_ {
    int __val[ 2];
  };

/* (l134) typedef struct _noname0_  __fsid_t; */

/* (l135) typedef long int __clock_t; */

/* (l136) typedef unsigned long int __rlim_t; */

/* (l137) typedef unsigned long int __rlim64_t; */

/* (l138) typedef unsigned int __id_t; */

/* (l139) typedef long int __time_t; */

/* (l140) typedef unsigned int __useconds_t; */

/* (l141) typedef long int __suseconds_t; */

/* (l143) typedef int __daddr_t; */

/* (l144) typedef int __key_t; */

/* (l147) typedef int __clockid_t; */

/* (l150) typedef void * __timer_t; */

/* (l153) typedef long int __blksize_t; */

/* (l158) typedef long int __blkcnt_t; */

/* (l159) typedef long int __blkcnt64_t; */

/* (l162) typedef unsigned long int __fsblkcnt_t; */

/* (l163) typedef unsigned long int __fsblkcnt64_t; */

/* (l166) typedef unsigned long int __fsfilcnt_t; */

/* (l167) typedef unsigned long int __fsfilcnt64_t; */

/* (l170) typedef long int __fsword_t; */

/* (l172) typedef long int __ssize_t; */

/* (l175) typedef long int __syscall_slong_t; */

/* (l177) typedef unsigned long int __syscall_ulong_t; */

/* (l181) typedef __off64_t __loff_t; */

/* (l182) typedef __quad_t * __qaddr_t; */

/* (l183) typedef char * __caddr_t; */

/* (l186) typedef long int __intptr_t; */

/* (l189) typedef unsigned int __socklen_t; */

struct _IO_FILE;

/* (l48) typedef struct _IO_FILE  FILE; */

/* (l64) typedef struct _IO_FILE  __FILE; */

struct _noname1_ {
    int __count;
    union {
        unsigned int __wch;
        char __wchb[ 4];
      } __value;
  };

/* (l94) typedef struct _noname1_  __mbstate_t; */

struct _noname2_ {
    long int __pos;
    struct _noname1_ __state;
  };

/* (l25) typedef struct _noname2_  _G_fpos_t; */

struct _noname3_ {
    long int __pos;
    struct _noname1_ __state;
  };

/* (l30) typedef struct _noname3_  _G_fpos64_t; */

/* (l40) typedef __builtin_va_list __gnuc_va_list; */

struct _IO_jump_t;
struct _IO_FILE;

/* (l150) typedef void _IO_lock_t; */

struct _IO_marker {
    struct _IO_marker * _next;
    struct _IO_FILE * _sbuf;
    int _pos;
  };
enum __codecvt_result {
    __codecvt_ok, __codecvt_partial, __codecvt_error, __codecvt_noconv
  };
struct _IO_FILE {
    int _flags;
    char * _IO_read_ptr;
    char * _IO_read_end;
    char * _IO_read_base;
    char * _IO_write_base;
    char * _IO_write_ptr;
    char * _IO_write_end;
    char * _IO_buf_base;
    char * _IO_buf_end;
    char * _IO_save_base;
    char * _IO_backup_base;
    char * _IO_save_end;
    struct _IO_marker * _markers;
    struct _IO_FILE * _chain;
    int _fileno;
    int _flags2;
    long int _old_offset;
    unsigned short _cur_column;
    signed char _vtable_offset;
    char _shortbuf[ 1];
    void (* _lock);
    long int _offset;
    void * __pad1;
    void * __pad2;
    void * __pad3;
    void * __pad4;
    long unsigned int __pad5;
    int _mode;
    char _unused2[ 15 * sizeof(int) - 4 * sizeof(void *) - sizeof(long unsigned int )];
  };

/* (l310) typedef struct _IO_FILE  _IO_FILE; */

struct _IO_FILE_plus;
extern struct _IO_FILE_plus _IO_2_1_stdin_;
extern struct _IO_FILE_plus _IO_2_1_stdout_;
extern struct _IO_FILE_plus _IO_2_1_stderr_;

/* (l333) typedef __ssize_t __io_read_fn(void * __cookie, char * __buf, size_t __nbytes); */

/* (l342) typedef __ssize_t __io_write_fn(void * __cookie, const char * __buf, size_t __n); */

/* (l350) typedef int __io_seek_fn(void * __cookie, __off64_t * __pos, int __w); */

/* (l353) typedef int __io_close_fn(void * __cookie); */

extern int __underflow(struct _IO_FILE (*));
extern int __uflow(struct _IO_FILE (*));
extern int __overflow(struct _IO_FILE (*), int);
extern int _IO_getc(struct _IO_FILE (* __fp));
extern int _IO_putc(int __c, struct _IO_FILE (* __fp));
extern int _IO_feof(struct _IO_FILE (* __fp));
extern int _IO_ferror(struct _IO_FILE (* __fp));
extern int _IO_peekc_locked(struct _IO_FILE (* __fp));
extern void _IO_flockfile(struct _IO_FILE (*));
extern void _IO_funlockfile(struct _IO_FILE (*));
extern int _IO_ftrylockfile(struct _IO_FILE (*));
extern int _IO_vfscanf(struct _IO_FILE (*), const char *, __builtin_va_list , int *);
extern int _IO_vfprintf(struct _IO_FILE (*), const char *, __builtin_va_list );
extern long int _IO_padn(struct _IO_FILE (*), int, long int );
extern long unsigned int _IO_sgetn(struct _IO_FILE (*), void *, long unsigned int );
extern long int _IO_seekoff(struct _IO_FILE (*), long int , int, int);
extern long int _IO_seekpos(struct _IO_FILE (*), long int , int);
extern void _IO_free_backup_area(struct _IO_FILE (*));

/* (l99) typedef __gnuc_va_list va_list; */

/* (l90) typedef __off_t off_t; */

/* (l104) typedef __ssize_t ssize_t; */

/* (l112) typedef _G_fpos_t fpos_t; */

extern struct _IO_FILE * stdin;
extern struct _IO_FILE * stdout;
extern struct _IO_FILE * stderr;
extern int remove(const char * __filename);
extern int rename(const char * __old, const char * __new);
extern int renameat(int __oldfd, const char * __old, int __newfd, const char * __new);
extern struct _IO_FILE (* tmpfile(void));
extern char * tmpnam(char * __s);
extern char * tmpnam_r(char * __s);
extern char * tempnam(const char * __dir, const char * __pfx);
extern int fclose(struct _IO_FILE (* __stream));
extern int fflush(struct _IO_FILE (* __stream));
extern int fflush_unlocked(struct _IO_FILE (* __stream));
extern struct _IO_FILE (* fopen(const char * __filename, const char * __modes));
extern struct _IO_FILE (* freopen(const char * __filename, const char * __modes, struct _IO_FILE (* __stream)));
extern struct _IO_FILE (* fdopen(int __fd, const char * __modes));
extern struct _IO_FILE (* fmemopen(void * __s, long unsigned int __len, const char * __modes));
extern struct _IO_FILE (* open_memstream(char ** __bufloc, long unsigned int (* __sizeloc)));
extern void setbuf(struct _IO_FILE (* __stream), char * __buf);
extern int setvbuf(struct _IO_FILE (* __stream), char * __buf, int __modes, long unsigned int __n);
extern void setbuffer(struct _IO_FILE (* __stream), char * __buf, long unsigned int __size);
extern void setlinebuf(struct _IO_FILE (* __stream));
extern int fprintf(struct _IO_FILE (* __stream), const char * __format, ...);
extern int printf(const char * __format, ...);
extern int sprintf(char * __s, const char * __format, ...);
extern int vfprintf(struct _IO_FILE (* __s), const char * __format, __builtin_va_list __arg);
extern int vprintf(const char * __format, __builtin_va_list __arg);
extern int vsprintf(char * __s, const char * __format, __builtin_va_list __arg);
extern int snprintf(char * __s, long unsigned int __maxlen, const char * __format, ...);
extern int vsnprintf(char * __s, long unsigned int __maxlen, const char * __format, __builtin_va_list __arg);
extern int vdprintf(int __fd, const char * __fmt, __builtin_va_list __arg);
extern int dprintf(int __fd, const char * __fmt, ...);
extern int fscanf(struct _IO_FILE (* __stream), const char * __format, ...);
extern int scanf(const char * __format, ...);
extern int sscanf(const char * __s, const char * __format, ...);
extern int __isoc99_fscanf(struct _IO_FILE (* __stream), const char * __format, ...);
extern int __isoc99_scanf(const char * __format, ...);
extern int __isoc99_sscanf(const char * __s, const char * __format, ...);
extern int vfscanf(struct _IO_FILE (* __s), const char * __format, __builtin_va_list __arg);
extern int vscanf(const char * __format, __builtin_va_list __arg);
extern int vsscanf(const char * __s, const char * __format, __builtin_va_list __arg);
extern int __isoc99_vfscanf(struct _IO_FILE (* __s), const char * __format, __builtin_va_list __arg);
extern int __isoc99_vscanf(const char * __format, __builtin_va_list __arg);
extern int __isoc99_vsscanf(const char * __s, const char * __format, __builtin_va_list __arg);
extern int fgetc(struct _IO_FILE (* __stream));
extern int getc(struct _IO_FILE (* __stream));
extern int getchar(void);
extern int getc_unlocked(struct _IO_FILE (* __stream));
extern int getchar_unlocked(void);
extern int fgetc_unlocked(struct _IO_FILE (* __stream));
extern int fputc(int __c, struct _IO_FILE (* __stream));
extern int putc(int __c, struct _IO_FILE (* __stream));
extern int putchar(int __c);
extern int fputc_unlocked(int __c, struct _IO_FILE (* __stream));
extern int putc_unlocked(int __c, struct _IO_FILE (* __stream));
extern int putchar_unlocked(int __c);
extern int getw(struct _IO_FILE (* __stream));
extern int putw(int __w, struct _IO_FILE (* __stream));
extern char * fgets(char * __s, int __n, struct _IO_FILE (* __stream));
extern long int __getdelim(char ** __lineptr, long unsigned int (* __n), int __delimiter, struct _IO_FILE (* __stream));
extern long int getdelim(char ** __lineptr, long unsigned int (* __n), int __delimiter, struct _IO_FILE (* __stream));
extern long int getline(char ** __lineptr, long unsigned int (* __n), struct _IO_FILE (* __stream));
extern int fputs(const char * __s, struct _IO_FILE (* __stream));
extern int puts(const char * __s);
extern int ungetc(int __c, struct _IO_FILE (* __stream));
extern long unsigned int fread(void * __ptr, long unsigned int __size, long unsigned int __n, struct _IO_FILE (* __stream));
extern long unsigned int fwrite(const void * __ptr, long unsigned int __size, long unsigned int __n, struct _IO_FILE (* __s));
extern long unsigned int fread_unlocked(void * __ptr, long unsigned int __size, long unsigned int __n, struct _IO_FILE (* __stream));
extern long unsigned int fwrite_unlocked(const void * __ptr, long unsigned int __size, long unsigned int __n, struct _IO_FILE (* __stream));
extern int fseek(struct _IO_FILE (* __stream), long int __off, int __whence);
extern long int ftell(struct _IO_FILE (* __stream));
extern void rewind(struct _IO_FILE (* __stream));
extern int fseeko(struct _IO_FILE (* __stream), long int __off, int __whence);
extern long int ftello(struct _IO_FILE (* __stream));
extern int fgetpos(struct _IO_FILE (* __stream), struct _noname2_ (* __pos));
extern int fsetpos(struct _IO_FILE (* __stream), const struct _noname2_ (* __pos));
extern void clearerr(struct _IO_FILE (* __stream));
extern int feof(struct _IO_FILE (* __stream));
extern int ferror(struct _IO_FILE (* __stream));
extern void clearerr_unlocked(struct _IO_FILE (* __stream));
extern int feof_unlocked(struct _IO_FILE (* __stream));
extern int ferror_unlocked(struct _IO_FILE (* __stream));
extern void perror(const char * __s);
extern int sys_nerr;
extern const char *const sys_errlist[];
extern int fileno(struct _IO_FILE (* __stream));
extern int fileno_unlocked(struct _IO_FILE (* __stream));
extern struct _IO_FILE (* popen(const char * __command, const char * __modes));
extern int pclose(struct _IO_FILE (* __stream));
extern char * ctermid(char * __s);
extern void flockfile(struct _IO_FILE (* __stream));
extern int ftrylockfile(struct _IO_FILE (* __stream));
extern void funlockfile(struct _IO_FILE (* __stream));
struct lpiImage {
    char name[ 512];
    int width;
    int height;
    unsigned char * imageData;
  };
struct lpiImage * lpiImage_lpiCreateImage(int width, int height, int pixel_size);
void lpiImage_lpiReleaseImage(struct lpiImage ** image);
void lpiImage_lpiResize(struct lpiImage * imgIn, struct lpiImage * imgOut);
int lpiImage_HA(int f1, float d);
int lpiImage_HB(int f2, float d);
int lpiImage_savePGM(struct lpiImage * lpiImage_v, char * filename);
struct lpiImage * lpiImage_loadPGM(char * filename);

/* (l328) typedef int wchar_t; */

enum _noname4_ {
    P_ALL, P_PID, P_PGID
  };

/* (l55) typedef enum _noname4_  idtype_t; */

struct _noname5_ {
    int quot;
    int rem;
  };

/* (l62) typedef struct _noname5_  div_t; */

struct _noname6_ {
    long int quot;
    long int rem;
  };

/* (l70) typedef struct _noname6_  ldiv_t; */

struct _noname7_ {
    long long int quot;
    long long int rem;
  };

/* (l82) typedef struct _noname7_  lldiv_t; */

extern long unsigned int __ctype_get_mb_cur_max(void);
extern double atof(const char * __nptr);
extern int atoi(const char * __nptr);
extern long int atol(const char * __nptr);
extern long long int atoll(const char * __nptr);
extern double strtod(const char * __nptr, char ** __endptr);
extern float strtof(const char * __nptr, char ** __endptr);
extern long double strtold(const char * __nptr, char ** __endptr);
extern long int strtol(const char * __nptr, char ** __endptr, int __base);
extern unsigned long int strtoul(const char * __nptr, char ** __endptr, int __base);
extern long long int strtoq(const char * __nptr, char ** __endptr, int __base);
extern unsigned long long int strtouq(const char * __nptr, char ** __endptr, int __base);
extern long long int strtoll(const char * __nptr, char ** __endptr, int __base);
extern unsigned long long int strtoull(const char * __nptr, char ** __endptr, int __base);
extern char * l64a(long int __n);
extern long int a64l(const char * __s);

/* (l33) typedef __u_char u_char; */

/* (l34) typedef __u_short u_short; */

/* (l35) typedef __u_int u_int; */

/* (l36) typedef __u_long u_long; */

/* (l37) typedef __quad_t quad_t; */

/* (l38) typedef __u_quad_t u_quad_t; */

/* (l39) typedef __fsid_t fsid_t; */

/* (l44) typedef __loff_t loff_t; */

/* (l48) typedef __ino_t ino_t; */

/* (l60) typedef __dev_t dev_t; */

/* (l65) typedef __gid_t gid_t; */

/* (l70) typedef __mode_t mode_t; */

/* (l75) typedef __nlink_t nlink_t; */

/* (l80) typedef __uid_t uid_t; */

/* (l98) typedef __pid_t pid_t; */

/* (l104) typedef __id_t id_t; */

/* (l115) typedef __daddr_t daddr_t; */

/* (l116) typedef __caddr_t caddr_t; */

/* (l122) typedef __key_t key_t; */

/* (l59) typedef __clock_t clock_t; */

/* (l75) typedef __time_t time_t; */

/* (l91) typedef __clockid_t clockid_t; */

/* (l103) typedef __timer_t timer_t; */

/* (l150) typedef unsigned long int ulong; */

/* (l151) typedef unsigned short int ushort; */

/* (l152) typedef unsigned int uint; */

/* (l162) typedef char int8_t; */

/* (l163) typedef short int int16_t; */

/* (l164) typedef int int32_t; */

/* (l166) typedef long int int64_t; */

/* (l173) typedef unsigned char u_int8_t; */

/* (l174) typedef unsigned short int u_int16_t; */

/* (l175) typedef unsigned int u_int32_t; */

/* (l177) typedef unsigned long int u_int64_t; */

/* (l182) typedef int register_t; */


static unsigned short int __bswap_16(unsigned short int __bsx)
{
  return (((unsigned short int) ((((__bsx) >> 8) & 0xff) | (((__bsx) & 0xff) << 8))));
}


static unsigned int __bswap_32(unsigned int __bsx)
{
  return (((((__bsx) & 0xff000000) >> 24) | (((__bsx) & 0x00ff0000) >> 8) | (((__bsx) & 0x0000ff00) << 8) | (((__bsx) & 0x000000ff) << 24)));
}


static unsigned long int __bswap_64(unsigned long int __bsx)
{
  return (((((__bsx) & 0xff00000000000000ull) >> 56) | (((__bsx) & 0x00ff000000000000ull) >> 40) | (((__bsx) & 0x0000ff0000000000ull) >> 24) | (((__bsx) & 0x000000ff00000000ull) >> 8) | (((__bsx) & 0x00000000ff000000ull) << 8) | (((__bsx) & 0x0000000000ff0000ull) << 24) | (((__bsx) & 0x000000000000ff00ull) << 40) | (((__bsx) & 0x00000000000000ffull) << 56)));
}

/* (l22) typedef int __sig_atomic_t; */

struct _noname8_ {
    unsigned long int __val[ (1024 / (8 * sizeof(unsigned long int)))];
  };

/* (l30) typedef struct _noname8_  __sigset_t; */

/* (l37) typedef __sigset_t sigset_t; */

struct timespec {
    long int tv_sec;
    long int tv_nsec;
  };
struct timeval {
    long int tv_sec;
    long int tv_usec;
  };

/* (l50) typedef __suseconds_t suseconds_t; */

/* (l56) typedef long int __fd_mask; */

struct _noname9_ {
    long int (__fds_bits[ 1024 / (8 * (int) sizeof(long int ))]);
  };

/* (l77) typedef struct _noname9_  fd_set; */

/* (l84) typedef __fd_mask fd_mask; */

extern int select(int __nfds, struct _noname9_ (* __readfds), struct _noname9_ (* __writefds), struct _noname9_ (* __exceptfds), struct timeval * __timeout);
extern int pselect(int __nfds, struct _noname9_ (* __readfds), struct _noname9_ (* __writefds), struct _noname9_ (* __exceptfds), const struct timespec * __timeout, const struct _noname8_ (* __sigmask));
extern unsigned int gnu_dev_major(unsigned long long int __dev);
extern unsigned int gnu_dev_minor(unsigned long long int __dev);
extern unsigned long long int gnu_dev_makedev(unsigned int __major, unsigned int __minor);

/* (l228) typedef __blksize_t blksize_t; */

/* (l235) typedef __blkcnt_t blkcnt_t; */

/* (l239) typedef __fsblkcnt_t fsblkcnt_t; */

/* (l243) typedef __fsfilcnt_t fsfilcnt_t; */

/* (l60) typedef unsigned long int pthread_t; */

union pthread_attr_t {
    char __size[ 56];
    long int __align;
  };

/* (l69) typedef union pthread_attr_t  pthread_attr_t; */

struct __pthread_internal_list {
    struct __pthread_internal_list * __prev;
    struct __pthread_internal_list * __next;
  };

/* (l79) typedef struct __pthread_internal_list  __pthread_list_t; */

union _noname10_ {
    struct __pthread_mutex_s {
        int __lock;
        unsigned int __count;
        int __owner;
        unsigned int __nusers;
        int __kind;
        short __spins;
        short __elision;
        struct __pthread_internal_list __list;
      } __data;
    char __size[ 40];
    long int __align;
  };

/* (l128) typedef union _noname10_  pthread_mutex_t; */

union _noname11_ {
    char __size[ 4];
    int __align;
  };

/* (l134) typedef union _noname11_  pthread_mutexattr_t; */

union _noname12_ {
    struct {
        int __lock;
        unsigned int __futex;
        unsigned long long int __total_seq;
        unsigned long long int __wakeup_seq;
        unsigned long long int __woken_seq;
        void * __mutex;
        unsigned int __nwaiters;
        unsigned int __broadcast_seq;
      } __data;
    char __size[ 48];
    long long int __align;
  };

/* (l154) typedef union _noname12_  pthread_cond_t; */

union _noname13_ {
    char __size[ 4];
    int __align;
  };

/* (l160) typedef union _noname13_  pthread_condattr_t; */

/* (l164) typedef unsigned int pthread_key_t; */

/* (l168) typedef int pthread_once_t; */

union _noname14_ {
    struct {
        int __lock;
        unsigned int __nr_readers;
        unsigned int __readers_wakeup;
        unsigned int __writer_wakeup;
        unsigned int __nr_readers_queued;
        unsigned int __nr_writers_queued;
        int __writer;
        int __shared;
        signed char __rwelision;
        unsigned char __pad1[ 7];
        unsigned long int __pad2;
        unsigned int __flags;
      } __data;
    char __size[ 56];
    long int __align;
  };

/* (l222) typedef union _noname14_  pthread_rwlock_t; */

union _noname15_ {
    char __size[ 8];
    long int __align;
  };

/* (l228) typedef union _noname15_  pthread_rwlockattr_t; */

/* (l234) typedef volatile int pthread_spinlock_t; */

union _noname16_ {
    char __size[ 32];
    long int __align;
  };

/* (l243) typedef union _noname16_  pthread_barrier_t; */

union _noname17_ {
    char __size[ 4];
    int __align;
  };

/* (l249) typedef union _noname17_  pthread_barrierattr_t; */

extern long int random(void);
extern void srandom(unsigned int __seed);
extern char * initstate(unsigned int __seed, char * __statebuf, long unsigned int __statelen);
extern char * setstate(char * __statebuf);
struct random_data {
    int (* fptr);
    int (* rptr);
    int (* state);
    int rand_type;
    int rand_deg;
    int rand_sep;
    int (* end_ptr);
  };
extern int random_r(struct random_data * __buf, int (* __result));
extern int srandom_r(unsigned int __seed, struct random_data * __buf);
extern int initstate_r(unsigned int __seed, char * __statebuf, long unsigned int __statelen, struct random_data * __buf);
extern int setstate_r(char * __statebuf, struct random_data * __buf);
extern int rand(void);
extern void srand(unsigned int __seed);
extern int rand_r(unsigned int * __seed);
extern double drand48(void);
extern double erand48(unsigned short int __xsubi[ 3]);
extern long int lrand48(void);
extern long int nrand48(unsigned short int __xsubi[ 3]);
extern long int mrand48(void);
extern long int jrand48(unsigned short int __xsubi[ 3]);
extern void srand48(long int __seedval);
extern unsigned short int * seed48(unsigned short int __seed16v[ 3]);
extern void lcong48(unsigned short int __param[ 7]);
struct drand48_data {
    unsigned short int __x[ 3];
    unsigned short int __old_x[ 3];
    unsigned short int __c;
    unsigned short int __init;
    unsigned long long int __a;
  };
extern int drand48_r(struct drand48_data * __buffer, double * __result);
extern int erand48_r(unsigned short int __xsubi[ 3], struct drand48_data * __buffer, double * __result);
extern int lrand48_r(struct drand48_data * __buffer, long int * __result);
extern int nrand48_r(unsigned short int __xsubi[ 3], struct drand48_data * __buffer, long int * __result);
extern int mrand48_r(struct drand48_data * __buffer, long int * __result);
extern int jrand48_r(unsigned short int __xsubi[ 3], struct drand48_data * __buffer, long int * __result);
extern int srand48_r(long int __seedval, struct drand48_data * __buffer);
extern int seed48_r(unsigned short int __seed16v[ 3], struct drand48_data * __buffer);
extern int lcong48_r(unsigned short int __param[ 7], struct drand48_data * __buffer);
extern void * malloc(long unsigned int __size);
extern void * calloc(long unsigned int __nmemb, long unsigned int __size);
extern void * realloc(void * __ptr, long unsigned int __size);
extern void free(void * __ptr);
extern void cfree(void * __ptr);
extern void * alloca(long unsigned int __size);
extern void * valloc(long unsigned int __size);
extern int posix_memalign(void ** __memptr, long unsigned int __alignment, long unsigned int __size);
extern void * aligned_alloc(long unsigned int __alignment, long unsigned int __size);
extern void abort(void);
extern int atexit(void (* __func)(void));
extern int at_quick_exit(void (* __func)(void));
extern int on_exit(void (* __func)(int __status, void * __arg), void * __arg);
extern void exit(int __status);
extern void quick_exit(int __status);
extern void _Exit(int __status);
extern char * getenv(const char * __name);
extern int putenv(char * __string);
extern int setenv(const char * __name, const char * __value, int __replace);
extern int unsetenv(const char * __name);
extern int clearenv(void);
extern char * mktemp(char * __template);
extern int mkstemp(char * __template);
extern int mkstemps(char * __template, int __suffixlen);
extern char * mkdtemp(char * __template);
extern int system(const char * __command);
extern char * realpath(const char * __name, char * __resolved);

/* (l702) typedef int (* __compar_fn_t) (const void *, const void *); */

extern void * bsearch(const void * __key, const void * __base, long unsigned int __nmemb, long unsigned int __size, int (* __compar)(const void *, const void *));
extern void qsort(void * __base, long unsigned int __nmemb, long unsigned int __size, int (* __compar)(const void *, const void *));
extern int abs(int __x);
extern long int labs(long int __x);
extern long long int llabs(long long int __x);
extern struct _noname5_ div(int __numer, int __denom);
extern struct _noname6_ ldiv(long int __numer, long int __denom);
extern struct _noname7_ lldiv(long long int __numer, long long int __denom);
extern char * ecvt(double __value, int __ndigit, int * __decpt, int * __sign);
extern char * fcvt(double __value, int __ndigit, int * __decpt, int * __sign);
extern char * gcvt(double __value, int __ndigit, char * __buf);
extern char * qecvt(long double __value, int __ndigit, int * __decpt, int * __sign);
extern char * qfcvt(long double __value, int __ndigit, int * __decpt, int * __sign);
extern char * qgcvt(long double __value, int __ndigit, char * __buf);
extern int ecvt_r(double __value, int __ndigit, int * __decpt, int * __sign, char * __buf, long unsigned int __len);
extern int fcvt_r(double __value, int __ndigit, int * __decpt, int * __sign, char * __buf, long unsigned int __len);
extern int qecvt_r(long double __value, int __ndigit, int * __decpt, int * __sign, char * __buf, long unsigned int __len);
extern int qfcvt_r(long double __value, int __ndigit, int * __decpt, int * __sign, char * __buf, long unsigned int __len);
extern int mblen(const char * __s, long unsigned int __n);
extern int mbtowc(int (* __pwc), const char * __s, long unsigned int __n);
extern int wctomb(char * __s, int __wchar);
extern long unsigned int mbstowcs(int (* __pwcs), const char * __s, long unsigned int __n);
extern long unsigned int wcstombs(char * __s, const int (* __pwcs), long unsigned int __n);
extern int rpmatch(const char * __response);
extern int getsubopt(char ** __optionp, char *const * __tokens, char ** __valuep);
extern int getloadavg(double __loadavg[], int __nelem);
struct FeatureMap {
    double m_subBias;
    double m_coeff;
    double m_bias;
    double ** m_weights;
    int m_sizeX;
    int m_sizeY;
    int m_windowSize;
  };
int FeatureMap_readState(struct FeatureMap * FeatureMap_v, struct _IO_FILE (* fp));
double ** FeatureMap_getKernel(struct FeatureMap * FeatureMap_v, int * size, double * bias);
void FeatureMap_getSubSample(struct FeatureMap * FeatureMap_v, double * coeff, double * bias);
struct Neuron {
    double * m_W;
    int m_Wlength;
  };
int Neuron_readState(struct Neuron * Neuron_v, struct _IO_FILE (* fp));
double * Neuron_getWeights(struct Neuron * Neuron_v);
double Neuron_ActivFunc(double input);
float Neuron_ActivFuncF(float input);
int * Alloc1DInt(int size);
double * Alloc1DDouble(int size);
float * Alloc1DFloat(int size);
int ** Alloc2DInt(int width, int height);
double ** Alloc2DDouble(int width, int height);
float ** Alloc2DFloat(int width, int height);
void Del1D(void * array);
void Del1D_i(int * array);
void Del1D_d(double * array);
void Del1D_f(float * array);
void Del2D(int height, void ** array);
void Del2D_i(int height, int ** array);
void Del2D_d(int height, double ** array);
void Del2D_f(int height, float ** array);
struct ConvKernelFM {
    float * kern;
    float bias;
    float coeff;
    float sbias;
    float * kern2;
  };
struct ConvKernelNeuron {
    float * kern;
    float bias;
  };
struct CNN {
    struct ConvKernelFM * m_kernels0;
    struct ConvKernelFM * m_kernels1;
    struct ConvKernelNeuron * m_kernels2;
    struct ConvKernelNeuron * m_kernels3;
    struct Neuron * m_N2;
    struct Neuron * m_N1;
    struct FeatureMap * m_FM2;
    struct FeatureMap * m_FM1;
    int m_nofLayers;
    int m_sizeInputX;
    int m_sizeInputY;
    int * m_LayerLength;
  };
int CNN_readState(struct CNN * CNN_v, char * filename);
void CNN_CreateConvolveKernels(struct CNN * CNN_v);
void CNN_CreateConvKernelN(struct CNN * CNN_v, int index);
void CNN_CreateConvKernelFM(struct CNN * CNN_v, int level, int index);
double ** CNN_getKernel(struct CNN * CNN_v, int level, int index, int * size, double * bias);
void CNN_getSubSample(struct CNN * CNN_v, int level, int index, double * coeff, double * bias);
double ** CNN_getNeuronKernel(struct CNN * CNN_v, int index, int * width, int * height, double * bias);
double * CNN_getOutputKernel(struct CNN * CNN_v);
struct CConvolver {
    struct CNN * m_cnn;
    struct lpiImage ** fm0;
    struct lpiImage ** fm0Sub;
    struct lpiImage ** fm1In;
    struct lpiImage ** fm1;
    struct lpiImage ** fm1Sub;
    struct lpiImage ** fm2;
    struct lpiImage * fmOut;
    struct lpiImage * fm_fus1;
    struct lpiImage * fm_fus2;
    int width;
    int height;
  };
void CConvolver_SetCNN(struct CConvolver * CConvolver_v, struct CNN * cnn);
float * CConvolver_ConvolveRoughly(struct CConvolver * CConvolver_v, struct lpiImage * img, int * ww, int * hh, int * tt);
struct lpiImage * CConvolver_CreateImage8U(int width, int height);
void CConvolver_DeallocateOutput(struct CConvolver * CConvolver_v);
int CConvolver_InitFMs(struct CConvolver * CConvolver_v, int width, int height);
void CConvolver_FreeFMs(struct CConvolver * CConvolver_v);
void CConvolver_ConvolveNeuron(struct CConvolver * CConvolver_v, struct lpiImage * fm, int index, struct lpiImage * out);
void CConvolver_SubSample(struct lpiImage * fm, float coeff, float bias, int disp, struct lpiImage * sub);
struct lpiImage * CConvolver_CreateImage(int width, int height);
float * CConvolver_ConvolveRoughlyStillImage(struct CConvolver * CConvolver_v, struct lpiImage * img, int width, int height, int * ww, int * hh, int * tt, int scale);
void CConvolver_Convolve(struct lpiImage * input, float * kernel, float bias, int kernel_dim, struct lpiImage * output);
void CConvolver_Convolve2(struct lpiImage * input, struct lpiImage * input2, float * kernel1, float * kernel2, float bias, int kernel_dim, struct lpiImage * output);
void CConvolver_ConvolveOutput(struct lpiImage ** fms, int nofFMS, float * weights, float bias, int width, int height, struct lpiImage * output);
struct per_image_data_t {
    float *** outputValues;
    struct lpiImage ** imgt;
    struct lpiImage ** imgt_f;
    struct CConvolver * cconv;
    char gl_FileTitle[ 256];
    int * Xs;
    int * Ys;
    float * outputs;
    int * heights;
    int * Xf;
    int * Yf;
    float * outf;
    int * Hf;
    int gl_S1;
    int gl_S2;
    double gl_ST;
    int gl_height;
    int gl_width;
    int allX[ 1000];
    int allY[ 1000];
    int allH[ 1000];
    float allOut[ 1000];
    int vg_deleted[ 1000];
    int discarded[ 1000];
  };


void torc_phase1(int iter, int itotal, void * img_source_ptr, void * img_input_ptr, void * data_ptr, int (* Sm), int * facesFound)
{
  int S, SH = 36;
  float SR = 36.0 / 32.0;
  struct per_image_data_t * data;
  struct lpiImage * img_source, * img_input;

  img_source = (struct lpiImage *) img_source_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  data = (struct per_image_data_t *) data_ptr;
  for (iter = 0; iter < itotal; iter++)
    {
      int i, j;
      int width = img_source->width;
      int height = img_source->height;
      int nbS;

      {
        int h1;
        int w1;
        struct lpiImage * img_tmp;
        unsigned char * p1;
        float * p2;
        int current_width, current_height, total_size;
        float * pp;

        nbS = iter;
        S = Sm[iter];
        h1 = (int) floor((double) height * ((double) SH / (double) S));
        w1 = (int) floor((double) width * ((double) SH / (double) S));
        if (h1 < 36 || w1 < 32)
          {
            printf("this should not happen!\n");
            exit(1);
            continue;
          }
        data->imgt[nbS] = lpiImage_lpiCreateImage(w1, h1, sizeof(unsigned char));
        lpiImage_lpiResize(img_input, data->imgt[nbS]);
        img_tmp = lpiImage_lpiCreateImage(w1, h1, sizeof(float));
        p1 = (unsigned char *) data->imgt[nbS]->imageData;
        p2 = (float *) img_tmp->imageData;
        for (i = 0; i < h1; i++)
          for (j = 0; j < w1; j++)
            p2[i * w1 + j] = (float) (((int) p1[i * data->imgt[nbS]->width + j]) - 128) / ((float) 128.0);
        pp = CConvolver_ConvolveRoughlyStillImage(&data->cconv[nbS], img_tmp, w1, h1, &current_width, &current_height, &total_size, S);
        for (i = 0; i < total_size; i++)
          {
            float output = pp[i];
            int y;
            int x;
            int ycenter;
            int xcenter;

            if (output <= (float) 0.0)
              continue;
            y = i / current_width;
            x = i % current_width;
            ycenter = (int) floor((double) (4 * y * S) / (double) SH) + (int) floor((double) S / 2.0);
            xcenter = (int) floor((double) (4 * x * S) / (double) SH) + (int) floor((double) S * SR / 2.0);
            {
              data->Xs[*facesFound] = xcenter;
              data->Ys[*facesFound] = ycenter;
              data->heights[*facesFound] = S;
              data->outputs[*facesFound] = output;
              *facesFound++;
            }
          }
        lpiImage_lpiReleaseImage(&data->imgt[nbS]);
        CConvolver_DeallocateOutput(&data->cconv[nbS]);
        lpiImage_lpiReleaseImage(&img_tmp);
      }
    }
}

/* #pragma ompix taskdef IN(iter, itotal, Sm) INOUT(img_source_ptr[ 1], img_input_ptr[ 1], facesFound[ 1], data_ptr[ 1]) */

void torc_phase1_torctask(int (* _ompix_iter), int (* _ompix_itotal), void * img_source_ptr, void * img_input_ptr, void * data_ptr, int (* Sm), int * facesFound)
{
  int itotal = *_ompix_itotal;
  int iter = *_ompix_iter;
  int S, SH = 36;
  float SR = 36.0 / 32.0;
  struct per_image_data_t * data;
  struct lpiImage * img_source, * img_input;

  img_source = (struct lpiImage *) img_source_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  data = (struct per_image_data_t *) data_ptr;
  for (iter = 0; iter < itotal; iter++)
    {
      int i, j;
      int width = img_source->width;
      int height = img_source->height;
      int nbS;

      {
        int h1;
        int w1;
        struct lpiImage * img_tmp;
        unsigned char * p1;
        float * p2;
        int current_width, current_height, total_size;
        float * pp;

        nbS = iter;
        S = Sm[iter];
        h1 = (int) floor((double) height * ((double) SH / (double) S));
        w1 = (int) floor((double) width * ((double) SH / (double) S));
        if (h1 < 36 || w1 < 32)
          {
            printf("this should not happen!\n");
            exit(1);
            continue;
          }
        data->imgt[nbS] = lpiImage_lpiCreateImage(w1, h1, sizeof(unsigned char));
        lpiImage_lpiResize(img_input, data->imgt[nbS]);
        img_tmp = lpiImage_lpiCreateImage(w1, h1, sizeof(float));
        p1 = (unsigned char *) data->imgt[nbS]->imageData;
        p2 = (float *) img_tmp->imageData;
        for (i = 0; i < h1; i++)
          for (j = 0; j < w1; j++)
            p2[i * w1 + j] = (float) (((int) p1[i * data->imgt[nbS]->width + j]) - 128) / ((float) 128.0);
        pp = CConvolver_ConvolveRoughlyStillImage(&data->cconv[nbS], img_tmp, w1, h1, &current_width, &current_height, &total_size, S);
        for (i = 0; i < total_size; i++)
          {
            float output = pp[i];
            int y;
            int x;
            int ycenter;
            int xcenter;

            if (output <= (float) 0.0)
              continue;
            y = i / current_width;
            x = i % current_width;
            ycenter = (int) floor((double) (4 * y * S) / (double) SH) + (int) floor((double) S / 2.0);
            xcenter = (int) floor((double) (4 * x * S) / (double) SH) + (int) floor((double) S * SR / 2.0);
            {
              data->Xs[*facesFound] = xcenter;
              data->Ys[*facesFound] = ycenter;
              data->heights[*facesFound] = S;
              data->outputs[*facesFound] = output;
              *facesFound++;
            }
          }
        lpiImage_lpiReleaseImage(&data->imgt[nbS]);
        CConvolver_DeallocateOutput(&data->cconv[nbS]);
        lpiImage_lpiReleaseImage(&img_tmp);
      }
    }
}


