/* File generated from [Convolver.pc] by OMPi compiler with torc extensions user/chriskar/hompi_start-v2.0.0-463-gde15e85~, Tue Jun 11 17:50:13 2019
$OMPi__nfo:0
*/
void ort_execute_serial(void * (* func)(void *), void * shared);
void ort_execute_parallel(void * (* func)(void *), void * shared, int num_threads, int iscombined, int procbind_type);
void * ort_get_thrpriv(void ** key, int size, void * origvar);
void ort_sglvar_allocate(void ** dataptr, int size, void * initer);
void ort_fence();
void ort_initreqs_add(void (* initfunc)(void));
void * ort_memalloc(int size);
void ort_memfree(void * ptr);
void ort_new_task(void * (* func)(void * arg), void * arg, int now, int final, int untied, int priority, void ** deparray, int noutdeps, int nindeps, int ninoutdeps);
void ort_taskwait(int waitall);
int ort_task_throttling(void);
void * ort_task_immediate_start(int final);
void ort_task_immediate_end(void * tn);
void ort_entering_taskgroup(void);
void ort_leaving_taskgroup(void);
void * ort_taskenv_alloc(int size, void * (* task_func)(void *));
void ort_taskenv_free(void * ptr, void * (* task_func)(void *));
void ort_atomic_begin();
void ort_atomic_end();
void ort_critical_begin(void ** cl);
void ort_critical_end(void ** cl);
void ort_broadcast_private(int num, ...);
void ort_copy_private(int num, ...);
int ort_barrier_me(void);
int ort_enable_cancel(int type);
int ort_check_cancel(int type);
struct _ort_gdopt_ {
    volatile unsigned long * data;
    volatile void * lock;
    int nth;
    void * me;
  };

/* (l72) typedef struct _ort_gdopt_  ort_gdopt_t; */

int ort_mysingle(int hasnowait);
void ort_leaving_single();
void ort_entering_sections(int hasnowait, int numberofsections);
void ort_leaving_sections();
int ort_get_section();
void ort_entering_for(int nowait, int hasordered, struct _ort_gdopt_ (* t));
void ort_entering_doacross(int nowait, int nestdepth, int collapsenum, int schedtype, int chsize, int niters, struct _ort_gdopt_ (* t));
int ort_leaving_for();
void ort_for_curriter(unsigned long iter);
void ort_ordered_begin();
void ort_ordered_end();
void ort_doacross_post(unsigned long iter);
void ort_doacross_wait(int ndeps, long * deps, long params[][ 3]);

/* (l93) typedef int (* chunky_t) (unsigned long niters, unsigned long chunksize, int monotonic, unsigned long * fiter, unsigned long * liter, int * extra, ort_gdopt_t * opt); */

unsigned long ort_num_iters(int num, long specs[][ 2], unsigned long * itp[]);
void ort_get_runtime_schedule_stuff(int (* (* func))(unsigned long niters, unsigned long chunksize, int monotonic, unsigned long * fiter, unsigned long * liter, int * extra, struct _ort_gdopt_ (* opt)), unsigned long * chunksize);
int ort_get_guided_chunk(unsigned long niters, unsigned long chunksize, int monotonic, unsigned long * fiter, unsigned long * liter, int * ignored, struct _ort_gdopt_ (* t));
int ort_get_dynamic_chunk(unsigned long niters, unsigned long chunksize, int monotonic, unsigned long * fiter, unsigned long * liter, int * ignored, struct _ort_gdopt_ (* t));
int ort_get_runtimestatic_chunk(unsigned long niters, unsigned long chunksize, int monotonic, unsigned long * fiter, unsigned long * liter, int * chunkid, struct _ort_gdopt_ (* t));
int ort_get_static_default_chunk(unsigned long niters, unsigned long * from, unsigned long * to);
void * ort_dev_gaddr(void * medaddr);
void * devpart_med2dev_addr(void * medaddr, unsigned long size);
void ort_reduction_begin(void ** cl);
void ort_reduction_end(void ** cl);
void ort_reduce_add(int, void *, void *, int);
void ort_reduce_subtract(int, void *, void *, int);
void ort_reduce_multiply(int, void *, void *, int);
void ort_reduce_and(int, void *, void *, int);
void ort_reduce_or(int, void *, void *, int);
void ort_reduce_max(int, void *, void *, int);
void ort_reduce_min(int, void *, void *, int);
void ort_reduce_bitand(int, void *, void *, int);
void ort_reduce_bitor(int, void *, void *, int);
void ort_reduce_bitxor(int, void *, void *, int);
void ort_offload_kernel(void * (* host_func)(void *), void * vars, void * declvars, char * kernel_filename_prefix, int devnum, ...);
void * ort_devdata_alloc(unsigned long size, int devnum);
void ort_devdata_free(void * data, int devnum);
void * ort_start_target_data(int tdvars, int devnum);
void ort_end_target_data(void * de);
void ort_map_tdvar(void * var, unsigned long size, void * varlb, int isptr, int init);
void ort_unmap_tdvar(void * var, int update);
void ort_map_var_dev(void * var, unsigned long size, void * varlb, int isptr, int devid, int init);
void ort_unmap_var_dev(void * var, int devid, int update, int delete);
void ort_read_var_dev(void * var, unsigned long size, void * varlb, int devnum);
void ort_write_var_dev(void * var, unsigned long size, void * varlb, int devnum);
void * ort_host2med_addr(void * var, int devnum);
void ort_decltarg_register(void * var, unsigned long size, const void * init, int bylink);
void * ort_decltarg_host2med_addr(void * var, int devnum);
void ort_kerneltable_add(char * name, void * (* kernel_function)(void *));
void * __ompi_defs__;

#define __OPENCL_ASQ
/* (l216) typedef long unsigned int size_t; */

/* (l328) typedef int wchar_t; */

enum _noname0_ {
    P_ALL, P_PID, P_PGID
  };

/* (l55) typedef enum _noname0_  idtype_t; */

struct _noname1_ {
    int quot;
    int rem;
  };

/* (l62) typedef struct _noname1_  div_t; */

struct _noname2_ {
    long int quot;
    long int rem;
  };

/* (l70) typedef struct _noname2_  ldiv_t; */

struct _noname3_ {
    long long int quot;
    long long int rem;
  };

/* (l82) typedef struct _noname3_  lldiv_t; */

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

struct _noname4_ {
    int __val[ 2];
  };

/* (l134) typedef struct _noname4_  __fsid_t; */

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

/* (l86) typedef __off_t off_t; */

/* (l98) typedef __pid_t pid_t; */

/* (l104) typedef __id_t id_t; */

/* (l109) typedef __ssize_t ssize_t; */

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

struct _noname5_ {
    unsigned long int __val[ (1024 / (8 * sizeof(unsigned long int)))];
  };

/* (l30) typedef struct _noname5_  __sigset_t; */

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

struct _noname6_ {
    long int (__fds_bits[ 1024 / (8 * (int) sizeof(long int ))]);
  };

/* (l77) typedef struct _noname6_  fd_set; */

/* (l84) typedef __fd_mask fd_mask; */

extern int select(int __nfds, struct _noname6_ (* __readfds), struct _noname6_ (* __writefds), struct _noname6_ (* __exceptfds), struct timeval * __timeout);
extern int pselect(int __nfds, struct _noname6_ (* __readfds), struct _noname6_ (* __writefds), struct _noname6_ (* __exceptfds), const struct timespec * __timeout, const struct _noname5_ (* __sigmask));
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

union _noname7_ {
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

/* (l128) typedef union _noname7_  pthread_mutex_t; */

union _noname8_ {
    char __size[ 4];
    int __align;
  };

/* (l134) typedef union _noname8_  pthread_mutexattr_t; */

union _noname9_ {
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

/* (l154) typedef union _noname9_  pthread_cond_t; */

union _noname10_ {
    char __size[ 4];
    int __align;
  };

/* (l160) typedef union _noname10_  pthread_condattr_t; */

/* (l164) typedef unsigned int pthread_key_t; */

/* (l168) typedef int pthread_once_t; */

union _noname11_ {
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

/* (l222) typedef union _noname11_  pthread_rwlock_t; */

union _noname12_ {
    char __size[ 8];
    long int __align;
  };

/* (l228) typedef union _noname12_  pthread_rwlockattr_t; */

/* (l234) typedef volatile int pthread_spinlock_t; */

union _noname13_ {
    char __size[ 32];
    long int __align;
  };

/* (l243) typedef union _noname13_  pthread_barrier_t; */

union _noname14_ {
    char __size[ 4];
    int __align;
  };

/* (l249) typedef union _noname14_  pthread_barrierattr_t; */

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
extern struct _noname1_ div(int __numer, int __denom);
extern struct _noname2_ ldiv(long int __numer, long int __denom);
extern struct _noname3_ lldiv(long long int __numer, long long int __denom);
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
struct _IO_FILE;

/* (l48) typedef struct _IO_FILE  FILE; */

/* (l64) typedef struct _IO_FILE  __FILE; */

struct _noname15_ {
    int __count;
    union {
        unsigned int __wch;
        char __wchb[ 4];
      } __value;
  };

/* (l94) typedef struct _noname15_  __mbstate_t; */

struct _noname16_ {
    long int __pos;
    struct _noname15_ __state;
  };

/* (l25) typedef struct _noname16_  _G_fpos_t; */

struct _noname17_ {
    long int __pos;
    struct _noname15_ __state;
  };

/* (l30) typedef struct _noname17_  _G_fpos64_t; */

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
extern int fgetpos(struct _IO_FILE (* __stream), struct _noname16_ (* __pos));
extern int fsetpos(struct _IO_FILE (* __stream), const struct _noname16_ (* __pos));
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
int omp_in_parallel(void);
int omp_get_thread_num(void);
void omp_set_num_threads(int num_threads);
int omp_get_num_threads(void);
int omp_get_max_threads(void);
int omp_get_num_procs(void);
void omp_set_dynamic(int dynamic_threads);
int omp_get_dynamic(void);
void omp_set_nested(int nested);
int omp_get_nested(void);
enum omp_sched_t {
    omp_sched_static = 1, omp_sched_dynamic = 2, omp_sched_guided = 3, omp_sched_auto = 4
  };

/* (l54) typedef enum omp_sched_t  omp_sched_t; */

enum omp_proc_bind_t {
    omp_proc_bind_false = 0, omp_proc_bind_true = 1, omp_proc_bind_master = 2, omp_proc_bind_close = 3, omp_proc_bind_spread = 4
  };

/* (l64) typedef enum omp_proc_bind_t  omp_proc_bind_t; */

enum omp_lock_hint_t {
    omp_lock_hint_none = 0, omp_lock_hint_uncontended = 1, omp_lock_hint_contended = 2, omp_lock_hint_nonspeculative = 4, omp_lock_hint_speculative = 8
  };

/* (l74) typedef enum omp_lock_hint_t  omp_lock_hint_t; */

/* (l77) typedef void * omp_lock_t; */

void omp_init_lock(void * (* lock));
void omp_init_lock_with_hint(void * (* lock), enum omp_lock_hint_t hint);
void omp_destroy_lock(void * (* lock));
void omp_set_lock(void * (* lock));
void omp_unset_lock(void * (* lock));
int omp_test_lock(void * (* lock));

/* (l87) typedef void * omp_nest_lock_t; */

void omp_init_nest_lock(void * (* lock));
void omp_init_next_lock_with_hint(void * (* lock), enum omp_lock_hint_t hint);
void omp_destroy_nest_lock(void * (* lock));
void omp_set_nest_lock(void * (* lock));
void omp_unset_nest_lock(void * (* lock));
int omp_test_nest_lock(void * (* lock));
double omp_get_wtime(void);
double omp_get_wtick(void);
void omp_set_schedule(enum omp_sched_t kind, int chunk);
void omp_get_schedule(enum omp_sched_t (* kind), int * chunk);
int omp_get_thread_limit(void);
void omp_set_max_active_levels(int levels);
int omp_get_max_active_levels(void);
int omp_get_level(void);
int omp_get_ancestor_thread_num(int level);
int omp_get_team_size(int level);
int omp_get_active_level(void);
int omp_in_final(void);
int omp_get_cancellation(void);
enum omp_proc_bind_t omp_get_proc_bind(void);
int omp_get_num_teams(void);
int omp_get_team_num(void);
int omp_is_initial_device(void);
int omp_get_max_task_priority(void);
int omp_get_num_places(void);
int omp_get_place_num_procs(int place_num);
void omp_get_place_proc_ids(int place_num, int * ids);
int omp_get_place_num(void);
int omp_get_partition_num_places(void);
void omp_get_partition_place_nums(int * place_nums);
void omp_set_default_device(int device_num);
int omp_get_default_device(void);
int omp_get_initial_device(void);
int omp_get_num_devices(void);
int omp_target_is_present(void * ptr, int devid);
void * omp_target_alloc(unsigned long size, int devid);
void omp_target_free(void * devaddr, int devid);
int omp_target_memcpy(void * dst, void * src, unsigned long length, unsigned long dst_off, unsigned long src_off, int dst_devid, int src_devid);
int omp_target_memcpy_rect(void * dst, void * src, unsigned long elemsize, int numdims, unsigned long * volume, unsigned long * dst_offs, unsigned long * src_offs, unsigned long * dst_dims, unsigned long * src_dims, int dst_devid, int src_devid);
int omp_target_associate_ptr(void * hostptr, void * devptr, unsigned long size, unsigned long devoff, int devid);
int omp_target_disassociate_ptr(void * hostptr, int devid);
union _noname18_ {
    unsigned char __c[ 8];
    double __d;
  };

/* (l41) typedef union _noname18_  __huge_val_t; */

static union _noname18_ __huge_val = { {
    0, 0, 0, 0, 0, 0, 0xf0, 0x7f
  } };
union _noname19_ {
    unsigned char __c[ 4];
    float __f;
  };

/* (l39) typedef union _noname19_  __huge_valf_t; */

static union _noname19_ __huge_valf = { {
    0, 0, 0x80, 0x7f
  } };
union _noname20_ {
    unsigned char __c[ 12];
    long double __ld;
  };
static union _noname20_ __huge_vall = { {
    0, 0, 0, 0, 0, 0, 0, 0x80, 0xff, 0x7f, 0, 0
  } };
union _noname21_ {
    unsigned char __c[ 4];
    float __d;
  };
static union _noname21_ __qnan_union = { {
    0, 0, 0xc0, 0x7f
  } };

/* (l28) typedef float float_t; */

/* (l29) typedef double double_t; */

extern double acos(double __x);
extern double __acos(double __x);
extern double asin(double __x);
extern double __asin(double __x);
extern double atan(double __x);
extern double __atan(double __x);
extern double atan2(double __y, double __x);
extern double __atan2(double __y, double __x);
extern double cos(double __x);
extern double __cos(double __x);
extern double sin(double __x);
extern double __sin(double __x);
extern double tan(double __x);
extern double __tan(double __x);
extern double cosh(double __x);
extern double __cosh(double __x);
extern double sinh(double __x);
extern double __sinh(double __x);
extern double tanh(double __x);
extern double __tanh(double __x);
extern double acosh(double __x);
extern double __acosh(double __x);
extern double asinh(double __x);
extern double __asinh(double __x);
extern double atanh(double __x);
extern double __atanh(double __x);
extern double exp(double __x);
extern double __exp(double __x);
extern double frexp(double __x, int * __exponent);
extern double __frexp(double __x, int * __exponent);
extern double ldexp(double __x, int __exponent);
extern double __ldexp(double __x, int __exponent);
extern double log(double __x);
extern double __log(double __x);
extern double log10(double __x);
extern double __log10(double __x);
extern double modf(double __x, double * __iptr);
extern double __modf(double __x, double * __iptr);
extern double expm1(double __x);
extern double __expm1(double __x);
extern double log1p(double __x);
extern double __log1p(double __x);
extern double logb(double __x);
extern double __logb(double __x);
extern double exp2(double __x);
extern double __exp2(double __x);
extern double log2(double __x);
extern double __log2(double __x);
extern double pow(double __x, double __y);
extern double __pow(double __x, double __y);
extern double sqrt(double __x);
extern double __sqrt(double __x);
extern double hypot(double __x, double __y);
extern double __hypot(double __x, double __y);
extern double cbrt(double __x);
extern double __cbrt(double __x);
extern double ceil(double __x);
extern double __ceil(double __x);
extern double fabs(double __x);
extern double __fabs(double __x);
extern double floor(double __x);
extern double __floor(double __x);
extern double fmod(double __x, double __y);
extern double __fmod(double __x, double __y);
extern int __isinf(double __value);
extern int __finite(double __value);
extern int isinf(double __value);
extern int finite(double __value);
extern double drem(double __x, double __y);
extern double __drem(double __x, double __y);
extern double significand(double __x);
extern double __significand(double __x);
extern double copysign(double __x, double __y);
extern double __copysign(double __x, double __y);
extern double nan(const char * __tagb);
extern double __nan(const char * __tagb);
extern int __isnan(double __value);
extern int isnan(double __value);
extern double j0(double);
extern double __j0(double);
extern double j1(double);
extern double __j1(double);
extern double jn(int, double);
extern double __jn(int, double);
extern double y0(double);
extern double __y0(double);
extern double y1(double);
extern double __y1(double);
extern double yn(int, double);
extern double __yn(int, double);
extern double erf(double);
extern double __erf(double);
extern double erfc(double);
extern double __erfc(double);
extern double lgamma(double);
extern double __lgamma(double);
extern double tgamma(double);
extern double __tgamma(double);
extern double gamma(double);
extern double __gamma(double);
extern double lgamma_r(double, int * __signgamp);
extern double __lgamma_r(double, int * __signgamp);
extern double rint(double __x);
extern double __rint(double __x);
extern double nextafter(double __x, double __y);
extern double __nextafter(double __x, double __y);
extern double nexttoward(double __x, long double __y);
extern double __nexttoward(double __x, long double __y);
extern double remainder(double __x, double __y);
extern double __remainder(double __x, double __y);
extern double scalbn(double __x, int __n);
extern double __scalbn(double __x, int __n);
extern int ilogb(double __x);
extern int __ilogb(double __x);
extern double scalbln(double __x, long int __n);
extern double __scalbln(double __x, long int __n);
extern double nearbyint(double __x);
extern double __nearbyint(double __x);
extern double round(double __x);
extern double __round(double __x);
extern double trunc(double __x);
extern double __trunc(double __x);
extern double remquo(double __x, double __y, int * __quo);
extern double __remquo(double __x, double __y, int * __quo);
extern long int lrint(double __x);
extern long int __lrint(double __x);
extern long long int llrint(double __x);
extern long long int __llrint(double __x);
extern long int lround(double __x);
extern long int __lround(double __x);
extern long long int llround(double __x);
extern long long int __llround(double __x);
extern double fdim(double __x, double __y);
extern double __fdim(double __x, double __y);
extern double fmax(double __x, double __y);
extern double __fmax(double __x, double __y);
extern double fmin(double __x, double __y);
extern double __fmin(double __x, double __y);
extern int __fpclassify(double __value);
extern int __signbit(double __value);
extern double fma(double __x, double __y, double __z);
extern double __fma(double __x, double __y, double __z);
extern double scalb(double __x, double __n);
extern double __scalb(double __x, double __n);
extern float acosf(float __x);
extern float __acosf(float __x);
extern float asinf(float __x);
extern float __asinf(float __x);
extern float atanf(float __x);
extern float __atanf(float __x);
extern float atan2f(float __y, float __x);
extern float __atan2f(float __y, float __x);
extern float cosf(float __x);
extern float __cosf(float __x);
extern float sinf(float __x);
extern float __sinf(float __x);
extern float tanf(float __x);
extern float __tanf(float __x);
extern float coshf(float __x);
extern float __coshf(float __x);
extern float sinhf(float __x);
extern float __sinhf(float __x);
extern float tanhf(float __x);
extern float __tanhf(float __x);
extern float acoshf(float __x);
extern float __acoshf(float __x);
extern float asinhf(float __x);
extern float __asinhf(float __x);
extern float atanhf(float __x);
extern float __atanhf(float __x);
extern float expf(float __x);
extern float __expf(float __x);
extern float frexpf(float __x, int * __exponent);
extern float __frexpf(float __x, int * __exponent);
extern float ldexpf(float __x, int __exponent);
extern float __ldexpf(float __x, int __exponent);
extern float logf(float __x);
extern float __logf(float __x);
extern float log10f(float __x);
extern float __log10f(float __x);
extern float modff(float __x, float * __iptr);
extern float __modff(float __x, float * __iptr);
extern float expm1f(float __x);
extern float __expm1f(float __x);
extern float log1pf(float __x);
extern float __log1pf(float __x);
extern float logbf(float __x);
extern float __logbf(float __x);
extern float exp2f(float __x);
extern float __exp2f(float __x);
extern float log2f(float __x);
extern float __log2f(float __x);
extern float powf(float __x, float __y);
extern float __powf(float __x, float __y);
extern float sqrtf(float __x);
extern float __sqrtf(float __x);
extern float hypotf(float __x, float __y);
extern float __hypotf(float __x, float __y);
extern float cbrtf(float __x);
extern float __cbrtf(float __x);
extern float ceilf(float __x);
extern float __ceilf(float __x);
extern float fabsf(float __x);
extern float __fabsf(float __x);
extern float floorf(float __x);
extern float __floorf(float __x);
extern float fmodf(float __x, float __y);
extern float __fmodf(float __x, float __y);
extern int __isinff(float __value);
extern int __finitef(float __value);
extern int isinff(float __value);
extern int finitef(float __value);
extern float dremf(float __x, float __y);
extern float __dremf(float __x, float __y);
extern float significandf(float __x);
extern float __significandf(float __x);
extern float copysignf(float __x, float __y);
extern float __copysignf(float __x, float __y);
extern float nanf(const char * __tagb);
extern float __nanf(const char * __tagb);
extern int __isnanf(float __value);
extern int isnanf(float __value);
extern float j0f(float);
extern float __j0f(float);
extern float j1f(float);
extern float __j1f(float);
extern float jnf(int, float);
extern float __jnf(int, float);
extern float y0f(float);
extern float __y0f(float);
extern float y1f(float);
extern float __y1f(float);
extern float ynf(int, float);
extern float __ynf(int, float);
extern float erff(float);
extern float __erff(float);
extern float erfcf(float);
extern float __erfcf(float);
extern float lgammaf(float);
extern float __lgammaf(float);
extern float tgammaf(float);
extern float __tgammaf(float);
extern float gammaf(float);
extern float __gammaf(float);
extern float lgammaf_r(float, int * __signgamp);
extern float __lgammaf_r(float, int * __signgamp);
extern float rintf(float __x);
extern float __rintf(float __x);
extern float nextafterf(float __x, float __y);
extern float __nextafterf(float __x, float __y);
extern float nexttowardf(float __x, long double __y);
extern float __nexttowardf(float __x, long double __y);
extern float remainderf(float __x, float __y);
extern float __remainderf(float __x, float __y);
extern float scalbnf(float __x, int __n);
extern float __scalbnf(float __x, int __n);
extern int ilogbf(float __x);
extern int __ilogbf(float __x);
extern float scalblnf(float __x, long int __n);
extern float __scalblnf(float __x, long int __n);
extern float nearbyintf(float __x);
extern float __nearbyintf(float __x);
extern float roundf(float __x);
extern float __roundf(float __x);
extern float truncf(float __x);
extern float __truncf(float __x);
extern float remquof(float __x, float __y, int * __quo);
extern float __remquof(float __x, float __y, int * __quo);
extern long int lrintf(float __x);
extern long int __lrintf(float __x);
extern long long int llrintf(float __x);
extern long long int __llrintf(float __x);
extern long int lroundf(float __x);
extern long int __lroundf(float __x);
extern long long int llroundf(float __x);
extern long long int __llroundf(float __x);
extern float fdimf(float __x, float __y);
extern float __fdimf(float __x, float __y);
extern float fmaxf(float __x, float __y);
extern float __fmaxf(float __x, float __y);
extern float fminf(float __x, float __y);
extern float __fminf(float __x, float __y);
extern int __fpclassifyf(float __value);
extern int __signbitf(float __value);
extern float fmaf(float __x, float __y, float __z);
extern float __fmaf(float __x, float __y, float __z);
extern float scalbf(float __x, float __n);
extern float __scalbf(float __x, float __n);
extern long double acosl(long double __x);
extern long double __acosl(long double __x);
extern long double asinl(long double __x);
extern long double __asinl(long double __x);
extern long double atanl(long double __x);
extern long double __atanl(long double __x);
extern long double atan2l(long double __y, long double __x);
extern long double __atan2l(long double __y, long double __x);
extern long double cosl(long double __x);
extern long double __cosl(long double __x);
extern long double sinl(long double __x);
extern long double __sinl(long double __x);
extern long double tanl(long double __x);
extern long double __tanl(long double __x);
extern long double coshl(long double __x);
extern long double __coshl(long double __x);
extern long double sinhl(long double __x);
extern long double __sinhl(long double __x);
extern long double tanhl(long double __x);
extern long double __tanhl(long double __x);
extern long double acoshl(long double __x);
extern long double __acoshl(long double __x);
extern long double asinhl(long double __x);
extern long double __asinhl(long double __x);
extern long double atanhl(long double __x);
extern long double __atanhl(long double __x);
extern long double expl(long double __x);
extern long double __expl(long double __x);
extern long double frexpl(long double __x, int * __exponent);
extern long double __frexpl(long double __x, int * __exponent);
extern long double ldexpl(long double __x, int __exponent);
extern long double __ldexpl(long double __x, int __exponent);
extern long double logl(long double __x);
extern long double __logl(long double __x);
extern long double log10l(long double __x);
extern long double __log10l(long double __x);
extern long double modfl(long double __x, long double * __iptr);
extern long double __modfl(long double __x, long double * __iptr);
extern long double expm1l(long double __x);
extern long double __expm1l(long double __x);
extern long double log1pl(long double __x);
extern long double __log1pl(long double __x);
extern long double logbl(long double __x);
extern long double __logbl(long double __x);
extern long double exp2l(long double __x);
extern long double __exp2l(long double __x);
extern long double log2l(long double __x);
extern long double __log2l(long double __x);
extern long double powl(long double __x, long double __y);
extern long double __powl(long double __x, long double __y);
extern long double sqrtl(long double __x);
extern long double __sqrtl(long double __x);
extern long double hypotl(long double __x, long double __y);
extern long double __hypotl(long double __x, long double __y);
extern long double cbrtl(long double __x);
extern long double __cbrtl(long double __x);
extern long double ceill(long double __x);
extern long double __ceill(long double __x);
extern long double fabsl(long double __x);
extern long double __fabsl(long double __x);
extern long double floorl(long double __x);
extern long double __floorl(long double __x);
extern long double fmodl(long double __x, long double __y);
extern long double __fmodl(long double __x, long double __y);
extern int __isinfl(long double __value);
extern int __finitel(long double __value);
extern int isinfl(long double __value);
extern int finitel(long double __value);
extern long double dreml(long double __x, long double __y);
extern long double __dreml(long double __x, long double __y);
extern long double significandl(long double __x);
extern long double __significandl(long double __x);
extern long double copysignl(long double __x, long double __y);
extern long double __copysignl(long double __x, long double __y);
extern long double nanl(const char * __tagb);
extern long double __nanl(const char * __tagb);
extern int __isnanl(long double __value);
extern int isnanl(long double __value);
extern long double j0l(long double);
extern long double __j0l(long double);
extern long double j1l(long double);
extern long double __j1l(long double);
extern long double jnl(int, long double);
extern long double __jnl(int, long double);
extern long double y0l(long double);
extern long double __y0l(long double);
extern long double y1l(long double);
extern long double __y1l(long double);
extern long double ynl(int, long double);
extern long double __ynl(int, long double);
extern long double erfl(long double);
extern long double __erfl(long double);
extern long double erfcl(long double);
extern long double __erfcl(long double);
extern long double lgammal(long double);
extern long double __lgammal(long double);
extern long double tgammal(long double);
extern long double __tgammal(long double);
extern long double gammal(long double);
extern long double __gammal(long double);
extern long double lgammal_r(long double, int * __signgamp);
extern long double __lgammal_r(long double, int * __signgamp);
extern long double rintl(long double __x);
extern long double __rintl(long double __x);
extern long double nextafterl(long double __x, long double __y);
extern long double __nextafterl(long double __x, long double __y);
extern long double nexttowardl(long double __x, long double __y);
extern long double __nexttowardl(long double __x, long double __y);
extern long double remainderl(long double __x, long double __y);
extern long double __remainderl(long double __x, long double __y);
extern long double scalbnl(long double __x, int __n);
extern long double __scalbnl(long double __x, int __n);
extern int ilogbl(long double __x);
extern int __ilogbl(long double __x);
extern long double scalblnl(long double __x, long int __n);
extern long double __scalblnl(long double __x, long int __n);
extern long double nearbyintl(long double __x);
extern long double __nearbyintl(long double __x);
extern long double roundl(long double __x);
extern long double __roundl(long double __x);
extern long double truncl(long double __x);
extern long double __truncl(long double __x);
extern long double remquol(long double __x, long double __y, int * __quo);
extern long double __remquol(long double __x, long double __y, int * __quo);
extern long int lrintl(long double __x);
extern long int __lrintl(long double __x);
extern long long int llrintl(long double __x);
extern long long int __llrintl(long double __x);
extern long int lroundl(long double __x);
extern long int __lroundl(long double __x);
extern long long int llroundl(long double __x);
extern long long int __llroundl(long double __x);
extern long double fdiml(long double __x, long double __y);
extern long double __fdiml(long double __x, long double __y);
extern long double fmaxl(long double __x, long double __y);
extern long double __fmaxl(long double __x, long double __y);
extern long double fminl(long double __x, long double __y);
extern long double __fminl(long double __x, long double __y);
extern int __fpclassifyl(long double __value);
extern int __signbitl(long double __value);
extern long double fmal(long double __x, long double __y, long double __z);
extern long double __fmal(long double __x, long double __y, long double __z);
extern long double scalbl(long double __x, long double __n);
extern long double __scalbl(long double __x, long double __n);
extern int signgam;
enum {
    FP_NAN = 0, FP_INFINITE = 1, FP_ZERO = 2, FP_SUBNORMAL = 3, FP_NORMAL = 4
  };
enum _noname22_ {
    _IEEE_ = -1, _SVID_, _XOPEN_, _POSIX_, _ISOC_
  };

/* (l354) typedef enum _noname22_  _LIB_VERSION_TYPE; */

extern enum _noname22_ _LIB_VERSION;
struct exception {
    int type;
    char * name;
    double arg1;
    double arg2;
    double retval;
  };
extern int matherr(struct exception * __exc);
extern double my_gettime();
void init_tanh();
extern float TANH_LUP[ 1600];


void CConvolver_SetCNN(struct CConvolver * CConvolver_v, struct CNN * cnn)
{
  CConvolver_v->m_cnn = cnn;
}


float * CConvolver_ConvolveRoughly(struct CConvolver * CConvolver_v, struct lpiImage * img, int * ww, int * hh, int * tt)
{
  int i, j;
  int current_width;
  int current_height;
  float * pOut;
  int total_size;

  for (i = 0; i < 4; i++)
    {
      CConvolver_Convolve(img, CConvolver_v->m_cnn->m_kernels0[i].kern, CConvolver_v->m_cnn->m_kernels0[i].bias, 5, CConvolver_v->fm0[i]);
      CConvolver_SubSample(CConvolver_v->fm0[i], CConvolver_v->m_cnn->m_kernels0[i].coeff, CConvolver_v->m_cnn->m_kernels0[i].sbias, 0, CConvolver_v->fm0Sub[i]);
    }
  current_width = (CConvolver_v->width - 4) / 2;
  current_height = (CConvolver_v->height - 4) / 2;
  for (i = 0; i < 8; i++)
    CConvolver_v->fm1In[i] = CConvolver_v->fm0Sub[i / 2];
  CConvolver_v->fm1In[8] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[9] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[10] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[11] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[12] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[13] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[14] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[15] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[16] = CConvolver_v->fm0Sub[3];
  CConvolver_v->fm1In[17] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[18] = CConvolver_v->fm0Sub[3];
  CConvolver_v->fm1In[19] = CConvolver_v->fm0Sub[3];
  for (i = 0; i < 8; i++)
    {
      CConvolver_Convolve(CConvolver_v->fm1In[i], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, CConvolver_v->fm1[i]);
      CConvolver_SubSample(CConvolver_v->fm1[i], CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
    }
  for (i = 8; i < 14; i++)
    {
      CConvolver_Convolve2(CConvolver_v->fm1In[i], CConvolver_v->fm1In[i + 6], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].kern2, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, CConvolver_v->fm1[i]);
      CConvolver_SubSample(CConvolver_v->fm1[i], CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
    }
  current_width = (current_width - 2) / 2;
  current_height = (current_height - 2) / 2;
  current_width = current_width - 6 + 1;
  current_height = current_height - 7 + 1;
  for (i = 0; i < 14; i++)
    CConvolver_ConvolveNeuron(CConvolver_v, CConvolver_v->fm1Sub[i], i, CConvolver_v->fm2[i]);
  CConvolver_ConvolveOutput(CConvolver_v->fm2, 14, CConvolver_v->m_cnn->m_kernels3[0].kern, CConvolver_v->m_cnn->m_kernels3[0].bias, current_width, current_height, CConvolver_v->fmOut);
  pOut = (float *) CConvolver_v->fmOut->imageData;
  total_size = current_width * current_height;
  *hh = current_height;
  *ww = current_width;
  *tt = total_size;
  return ((pOut));
}


struct lpiImage * CConvolver_CreateImage(int width, int height)
{
  struct lpiImage * img = lpiImage_lpiCreateImage(width, height, sizeof(float));

  return ((img));
}


void CConvolver_SubSample(struct lpiImage * fm, float coeff, float bias, int disp, struct lpiImage * sub)
{
  float * pb = (float *) fm->imageData;
  float * pc = (float *) sub->imageData;
  int ih2 = sub->height;
  int iw2 = sub->width;
  int i, j, w = fm->width;
  int sub_size;

  for (i = 0; i < ih2; i++)
    {
      int i2 = 2 * i + disp;
      int i2w = i2 * w;
      int i_iw2 = i * iw2;
      float * pc1 = &pc[i_iw2];

      for (j = 0; j < iw2; j++)
        {
          int j2 = 2 * j + disp;
          float * pb1 = &pb[i2w + j2];

          pc1[j] = (pb1[0] + pb1[1] + pb1[w] + pb1[w + 1]) * coeff + bias;
        }
    }
  sub_size = ih2 * iw2;
  for (i = 0; i < sub_size; i++)
    {
      if (pc[i] > 8.0)
        pc[i] = ((1.7159));
      else
        if (pc[i] < -8.0)
          pc[i] = -((1.7159));
        else
          {
            int tmp_index = ((int) (pc[i] * 100.0f)) + 800;

            pc[i] = TANH_LUP[tmp_index];
          }
      ;
    }
}


void CConvolver_ConvolveNeuron(struct CConvolver * CConvolver_v, struct lpiImage * fm, int index, struct lpiImage * out)
{
  float * pfm = (float *) fm->imageData;
  float * pout = (float *) out->imageData;
  int wfm = fm->width;
  int wout = out->width;
  float bias = CConvolver_v->m_cnn->m_kernels2[index].bias;
  float * kernel = CConvolver_v->m_cnn->m_kernels2[index].kern;
  int i, j, l;

  for (i = 3; i < fm->height - 3; i++)
    {
      int i_3_wout_3 = (i - 3) * wout - 3;
      float * pout1 = &pout[i_3_wout_3];

      for (j = 3; j <= fm->width - 3; j++)
        {
          float product = bias;
          float tmp;
          int i_wfm_j = i * wfm + j;
          int i_k_wfm_j;
          float * pfm1;
          float * kernel1;

          {
            i_k_wfm_j = i_wfm_j - 3 * wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[3];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j - 2 * wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[9];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j - wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[15];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[21];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j + 1 * wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[27];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j + 2 * wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[33];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          {
            i_k_wfm_j = i_wfm_j + 3 * wfm;
            pfm1 = &pfm[i_k_wfm_j];
            kernel1 = &kernel[39];
            product += kernel1[-3] * pfm1[-3] + kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
          }
          if (product > 8.0)
            tmp = ((1.7159));
          else
            if (product < -8.0)
              tmp = -((1.7159));
            else
              {
                int tmp_index = ((int) (product * 100.0f)) + 800;

                tmp = TANH_LUP[tmp_index];
              }
          ;
          pout1[j] = tmp;
        }
    }
}


int CConvolver_InitFMs(struct CConvolver * CConvolver_v, int width, int height)
{
  int i;
  int current_width;
  int current_height;

  CConvolver_v->width = width;
  CConvolver_v->height = height;
  CConvolver_v->fm0 = (struct lpiImage **) calloc(4, sizeof(struct lpiImage *));
  for (i = 0; i < 4; i++)
    CConvolver_v->fm0[i] = CConvolver_CreateImage(width, height);
  current_width = (width - 4) / 2;
  current_height = (height - 4) / 2;
  CConvolver_v->fm0Sub = (struct lpiImage **) calloc(4, sizeof(struct lpiImage *));
  for (i = 0; i < 4; i++)
    CConvolver_v->fm0Sub[i] = CConvolver_CreateImage(current_width, current_height);
  CConvolver_v->fm1In = (struct lpiImage **) calloc(20, sizeof(struct lpiImage *));
  CConvolver_v->fm1 = (struct lpiImage **) calloc(14, sizeof(struct lpiImage *));
  for (i = 0; i < 14; i++)
    CConvolver_v->fm1[i] = CConvolver_CreateImage(current_width, current_height);
  CConvolver_v->fm_fus1 = CConvolver_CreateImage(current_width, current_height);
  CConvolver_v->fm_fus2 = CConvolver_CreateImage(current_width, current_height);
  current_width = (current_width - 2) / 2;
  current_height = (current_height - 2) / 2;
  CConvolver_v->fm1Sub = (struct lpiImage **) calloc(14, sizeof(struct lpiImage *));
  for (i = 0; i < 14; i++)
    CConvolver_v->fm1Sub[i] = CConvolver_CreateImage(current_width, current_height);
  current_width = current_width - 6 + 1;
  current_height = current_height - 7 + 1;
  CConvolver_v->fm2 = (struct lpiImage **) calloc(14, sizeof(struct lpiImage *));
  for (i = 0; i < 14; i++)
    CConvolver_v->fm2[i] = CConvolver_CreateImage(current_width, current_height);
  CConvolver_v->fmOut = CConvolver_CreateImage(current_width, current_height);
  return ((1));
}


void CConvolver_FreeFMs(struct CConvolver * CConvolver_v)
{
  int i;

  for (i = 0; i < 4; i++)
    {
      lpiImage_lpiReleaseImage(&CConvolver_v->fm0[i]);
      lpiImage_lpiReleaseImage(&CConvolver_v->fm0Sub[i]);
    }
  free(CConvolver_v->fm0);
  free(CConvolver_v->fm0Sub);
  free(CConvolver_v->fm1In);
  for (i = 0; i < 14; i++)
    {
      lpiImage_lpiReleaseImage(&CConvolver_v->fm1[i]);
      lpiImage_lpiReleaseImage(&CConvolver_v->fm1Sub[i]);
      lpiImage_lpiReleaseImage(&CConvolver_v->fm2[i]);
    }
  free(CConvolver_v->fm1);
  free(CConvolver_v->fm1Sub);
  free(CConvolver_v->fm2);
  lpiImage_lpiReleaseImage(&CConvolver_v->fmOut);
  lpiImage_lpiReleaseImage(&CConvolver_v->fm_fus1);
  lpiImage_lpiReleaseImage(&CConvolver_v->fm_fus2);
}


void CConvolver_Convolve(struct lpiImage * input, float * kernel, float bias, int kernel_dim, struct lpiImage * output)
{
  int i, j, k, l;
  int width = input->width;
  int height = input->height;
  int outWidth = width - kernel_dim + 1;
  int outHeight = height - kernel_dim + 1;
  float * pfm = (float *) input->imageData;
  float * pout = (float *) output->imageData;
  int wfm = input->width;
  int wout = output->width;
  int ws = kernel_dim / 2;
  int displ = ws * kernel_dim + ws;
  int displ2 = -ws * wout - ws;

  if ((ws != 1) && (ws != 2))
    {
      printf("ws = %d\n", ws);
      exit(1);
    }
  for (i = ws; i < height - ws; i++)
    {
      int i_wout_displ2 = i * wout + displ2;
      float * pout1 = &pout[i_wout_displ2];

      for (j = ws; j < width - ws; j++)
        {
          register float product = bias;

          if (ws == 1)
            {
              int i_wfm_j = i * wfm + j;
              int kk_displ;
              int i_k_wfm_j;
              float * pfm1;
              float * kernel1;

              {
                kk_displ = (-3) + displ;
                i_k_wfm_j = i_wfm_j - wfm;
                pfm1 = &pfm[i_k_wfm_j];
                kernel1 = &kernel[kk_displ];
                product += kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1];
              }
              {
                kk_displ = displ;
                i_k_wfm_j = i_wfm_j;
                pfm1 = &pfm[i_k_wfm_j];
                kernel1 = &kernel[kk_displ];
                product += kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1];
              }
              {
                kk_displ = (3) + displ;
                i_k_wfm_j = i_wfm_j + wfm;
                pfm1 = &pfm[i_k_wfm_j];
                kernel1 = &kernel[kk_displ];
                product += kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1];
              }
            }
          else
            if (ws == 2)
              {
                int i_wfm_j = i * wfm + j;
                int kk_displ;
                int i_k_wfm_j;
                float * pfm1;
                float * kernel1;

                {
                  kk_displ = (-10) + displ;
                  i_k_wfm_j = i_wfm_j - 2 * wfm;
                  pfm1 = &pfm[i_k_wfm_j];
                  kernel1 = &kernel[kk_displ];
                  product += kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
                }
                {
                  kk_displ = (-5) + displ;
                  i_k_wfm_j = i_wfm_j - wfm;
                  pfm1 = &pfm[i_k_wfm_j];
                  kernel1 = &kernel[kk_displ];
                  product += kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
                }
                {
                  kk_displ = displ;
                  i_k_wfm_j = i_wfm_j;
                  pfm1 = &pfm[i_k_wfm_j];
                  kernel1 = &kernel[kk_displ];
                  product += kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
                }
                {
                  kk_displ = (5) + displ;
                  i_k_wfm_j = i_wfm_j + wfm;
                  pfm1 = &pfm[i_k_wfm_j];
                  kernel1 = &kernel[kk_displ];
                  product += kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
                }
                {
                  kk_displ = (10) + displ;
                  i_k_wfm_j = i_wfm_j + 2 * wfm;
                  pfm1 = &pfm[i_k_wfm_j];
                  kernel1 = &kernel[kk_displ];
                  product += kernel1[-2] * pfm1[-2] + kernel1[-1] * pfm1[-1] + kernel1[0] * pfm1[0] + kernel1[1] * pfm1[1] + kernel1[2] * pfm1[2];
                }
              }
            else
              {
                for (k = -ws; k <= ws; k++)
                  {
                    int kk_displ = k * kernel_dim + displ;
                    int i_k_wfm_j = (i + k) * wfm + j;
                    float * pfm1 = &pfm[i_k_wfm_j];
                    float * kernel1 = &kernel[kk_displ];

                    for (l = -ws; l <= ws; l++)
                      {
                        product += kernel1[l] * pfm1[l];
                      }
                  }
              }
          pout1[j] = product;
        }
    }
}


void CConvolver_Convolve2(struct lpiImage * input, struct lpiImage * input2, float * kernel1, float * kernel2, float bias, int kernel_dim, struct lpiImage * output)
{
  int i, j, k, l;
  int width = input->width;
  int height = input->height;
  int outWidth = width - kernel_dim + 1;
  int outHeight = height - kernel_dim + 1;
  float * pfm = (float *) input->imageData;
  float * pout = (float *) output->imageData;
  float * pfm2 = (float *) input2->imageData;
  int wfm = input->width;
  int wout = output->width;
  int ws = kernel_dim / 2;
  int displ = ws * kernel_dim + ws;
  int displ2 = -ws * wout - ws;

  if ((ws != 1) && (ws != 2))
    {
      printf("ws = %d\n", ws);
      exit(1);
    }
  for (i = ws; i < height - ws; i++)
    {
      int i_wout_displ2 = i * wout + displ2;
      float * pout1 = &pout[i_wout_displ2];

      for (j = ws; j < width - ws; j++)
        {
          float product = bias;

          if (ws == 1)
            {
              int i_wfm_j = i * wfm + j;
              int i_k_wfm_j;
              float * pfm1;
              float * pfm21;
              int kk_displ;
              float * kernel11;
              float * kernel21;

              {
                i_k_wfm_j = i_wfm_j - wfm;
                pfm1 = &pfm[i_k_wfm_j];
                pfm21 = &pfm2[i_k_wfm_j];
                kk_displ = (-3) + displ;
                kernel11 = &kernel1[kk_displ];
                kernel21 = &kernel2[kk_displ];
                product += kernel11[-1] * pfm1[-1] + kernel21[-1] * pfm21[-1] + kernel11[0] * pfm1[0] + kernel21[0] * pfm21[0] + kernel11[1] * pfm1[1] + kernel21[1] * pfm21[1];
              }
              {
                i_k_wfm_j = i_wfm_j;
                pfm1 = &pfm[i_k_wfm_j];
                pfm21 = &pfm2[i_k_wfm_j];
                kk_displ = displ;
                kernel11 = &kernel1[kk_displ];
                kernel21 = &kernel2[kk_displ];
                product += kernel11[-1] * pfm1[-1] + kernel21[-1] * pfm21[-1] + kernel11[0] * pfm1[0] + kernel21[0] * pfm21[0] + kernel11[1] * pfm1[1] + kernel21[1] * pfm21[1];
              }
              {
                i_k_wfm_j = i_wfm_j + wfm;
                pfm1 = &pfm[i_k_wfm_j];
                pfm21 = &pfm2[i_k_wfm_j];
                kk_displ = (3) + displ;
                kernel11 = &kernel1[kk_displ];
                kernel21 = &kernel2[kk_displ];
                product += kernel11[-1] * pfm1[-1] + kernel21[-1] * pfm21[-1] + kernel11[0] * pfm1[0] + kernel21[0] * pfm21[0] + kernel11[1] * pfm1[1] + kernel21[1] * pfm21[1];
              }
            }
          else
            if (ws == 2)
              {
                printf("!!! ws = %d\n", ws);
                for (k = -ws; k <= ws; k++)
                  {
                    int i_k_wfm_j = (i + k) * wfm + j;
                    float * pfm1 = &pfm[i_k_wfm_j];
                    float * pfm21 = &pfm2[i_k_wfm_j];
                    int kk_displ = k * kernel_dim + displ;
                    float * kernel11 = &kernel1[kk_displ];
                    float * kernel21 = &kernel2[kk_displ];

                    for (l = -ws; l <= ws; l++)
                      {
                        product += kernel11[l] * pfm1[l];
                        product += kernel21[l] * pfm21[l];
                      }
                  }
              }
            else
              {
                for (k = -ws; k <= ws; k++)
                  {
                    int i_k_wfm_j = (i + k) * wfm + j;
                    float * pfm1 = &pfm[i_k_wfm_j];
                    float * pfm21 = &pfm2[i_k_wfm_j];
                    int kk_displ = k * kernel_dim + displ;
                    float * kernel11 = &kernel1[kk_displ];
                    float * kernel21 = &kernel2[kk_displ];

                    for (l = -ws; l <= ws; l++)
                      {
                        product += kernel11[l] * pfm1[l];
                        product += kernel21[l] * pfm21[l];
                      }
                  }
              }
          pout1[j] = product;
        }
    }
}


void CConvolver_ConvolveOutput(struct lpiImage ** fms, int nofFMS, float * weights, float bias, int width, int height, struct lpiImage * output)
{
  int i, j, k;
  float * pout = (float *) output->imageData;

  for (i = 0; i < height; i++)
    {
      register int i_width = i * width;
      float * pout1 = &pout[i_width];

      for (j = 0; j < width; j++)
        {
          float product = bias;

          for (k = 0; k < nofFMS; k++)
            {
              float * pfm = (float *) fms[k]->imageData;

              product += pfm[i_width + j] * weights[k];
            }
          if (product > 8.0)
            pout1[j] = ((1.7159));
          else
            if (product < -8.0)
              pout1[j] = -((1.7159));
            else
              {
                int tmp_index = ((int) (product * 100.0f)) + 800;

                pout1[j] = TANH_LUP[tmp_index];
              }
          ;
        }
    }
}

void torc_conv_task0_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height));


void torc_conv_task0(int i, int width, int height, void * CConvolver_v_ptr, void * img_ptr, int current_width, int current_height)
{
  struct lpiImage * fm0_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm0_tmp = CConvolver_CreateImage(width - 4, height - 4);
  CConvolver_v->fm0Sub[i] = CConvolver_CreateImage(current_width, current_height);
  CConvolver_Convolve(img, CConvolver_v->m_cnn->m_kernels0[i].kern, CConvolver_v->m_cnn->m_kernels0[i].bias, 5, fm0_tmp);
  CConvolver_SubSample(fm0_tmp, CConvolver_v->m_cnn->m_kernels0[i].coeff, CConvolver_v->m_cnn->m_kernels0[i].sbias, 0, CConvolver_v->fm0Sub[i]);
  lpiImage_lpiReleaseImage(&fm0_tmp);
}

/* #pragma ompix taskdef IN(i, width, height, current_width, current_height) INOUT(CConvolver_v_ptr[ 1], img_ptr[ 1]) */

void torc_conv_task0_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height))
{
  int current_height = *_ompix_current_height;
  int current_width = *_ompix_current_width;
  int height = *_ompix_height;
  int width = *_ompix_width;
  int i = *_ompix_i;
    struct lpiImage * fm0_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm0_tmp = CConvolver_CreateImage(width - 4, height - 4);
  CConvolver_v->fm0Sub[i] = CConvolver_CreateImage(current_width, current_height);
  CConvolver_Convolve(img, CConvolver_v->m_cnn->m_kernels0[i].kern, CConvolver_v->m_cnn->m_kernels0[i].bias, 5, fm0_tmp);
  CConvolver_SubSample(fm0_tmp, CConvolver_v->m_cnn->m_kernels0[i].coeff, CConvolver_v->m_cnn->m_kernels0[i].sbias, 0, CConvolver_v->fm0Sub[i]);
  lpiImage_lpiReleaseImage(&fm0_tmp);
}

void torc_conv_task1_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height));


void torc_conv_task1(int i, int width, int height, void * CConvolver_v_ptr, void * img_ptr, int current_width, int current_height)
{
  struct lpiImage * fm1_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm1_tmp = CConvolver_CreateImage(current_width - 2, current_height - 2);
  CConvolver_v->fm1Sub[i] = CConvolver_CreateImage((current_width - 2) / 2, (current_height - 2) / 2);
  CConvolver_Convolve(CConvolver_v->fm1In[i], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, fm1_tmp);
  CConvolver_SubSample(fm1_tmp, CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
  lpiImage_lpiReleaseImage(&fm1_tmp);
}

/* #pragma ompix taskdef IN(i, width, height, current_width, current_height) INOUT(CConvolver_v_ptr[ 1], img_ptr[ 1]) */

void torc_conv_task1_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height))
{
  int current_height = *_ompix_current_height;
  int current_width = *_ompix_current_width;
  int height = *_ompix_height;
  int width = *_ompix_width;
  int i = *_ompix_i;
    struct lpiImage * fm1_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm1_tmp = CConvolver_CreateImage(current_width - 2, current_height - 2);
  CConvolver_v->fm1Sub[i] = CConvolver_CreateImage((current_width - 2) / 2, (current_height - 2) / 2);
  CConvolver_Convolve(CConvolver_v->fm1In[i], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, fm1_tmp);
  CConvolver_SubSample(fm1_tmp, CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
  lpiImage_lpiReleaseImage(&fm1_tmp);
}

void torc_conv_task2_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height));


void torc_conv_task2(int i, int width, int height, void * CConvolver_v_ptr, void * img_ptr, int current_width, int current_height)
{
  struct lpiImage * fm1_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm1_tmp = CConvolver_CreateImage(current_width - 2, current_height - 2);
  CConvolver_v->fm1Sub[i] = CConvolver_CreateImage((current_width - 2) / 2, (current_height - 2) / 2);
  CConvolver_Convolve2(CConvolver_v->fm1In[i], CConvolver_v->fm1In[i + 6], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].kern2, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, fm1_tmp);
  CConvolver_SubSample(fm1_tmp, CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
  lpiImage_lpiReleaseImage(&fm1_tmp);
}

/* #pragma ompix taskdef IN(i, width, height, current_width, current_height) INOUT(CConvolver_v_ptr[ 1], img_ptr[ 1]) */

void torc_conv_task2_torctask(int (* _ompix_i), int (* _ompix_width), int (* _ompix_height), void * CConvolver_v_ptr, void * img_ptr, int (* _ompix_current_width), int (* _ompix_current_height))
{
  int current_height = *_ompix_current_height;
  int current_width = *_ompix_current_width;
  int height = *_ompix_height;
  int width = *_ompix_width;
  int i = *_ompix_i;
    struct lpiImage * fm1_tmp;
  struct lpiImage * img;
  struct CConvolver * CConvolver_v;

  img = (struct lpiImage *) img_ptr;
  CConvolver_v = (struct CConvolver *) CConvolver_v_ptr;
  fm1_tmp = CConvolver_CreateImage(current_width - 2, current_height - 2);
  CConvolver_v->fm1Sub[i] = CConvolver_CreateImage((current_width - 2) / 2, (current_height - 2) / 2);
  CConvolver_Convolve2(CConvolver_v->fm1In[i], CConvolver_v->fm1In[i + 6], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].kern2, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, fm1_tmp);
  CConvolver_SubSample(fm1_tmp, CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
  lpiImage_lpiReleaseImage(&fm1_tmp);
}


float * CConvolver_ConvolveRoughlyStillImage(struct CConvolver * CConvolver_v, struct lpiImage * img, int width, int height, int * ww, int * hh, int * tt, int scale)
{
  int i, j, my_node;
  int current_width;
  int current_height;
  struct lpiImage * fm0_tmp;
  struct lpiImage * fm1_tmp;
  int total_size;
  float * pOut;

  CConvolver_v->width = width;
  CConvolver_v->height = height;
  CConvolver_v->fm0Sub = (struct lpiImage **) calloc(4, sizeof(struct lpiImage *));
  current_width = (width - 4) / 2;
  current_height = (height - 4) / 2;
  for (i = 0; i < 4; i++)
    {
      struct lpiImage * fm0_tmp;

      fm0_tmp = CConvolver_CreateImage(width - 4, height - 4);
      CConvolver_v->fm0Sub[i] = CConvolver_CreateImage(current_width, current_height);
      CConvolver_Convolve(img, CConvolver_v->m_cnn->m_kernels0[i].kern, CConvolver_v->m_cnn->m_kernels0[i].bias, 5, fm0_tmp);
      CConvolver_SubSample(fm0_tmp, CConvolver_v->m_cnn->m_kernels0[i].coeff, CConvolver_v->m_cnn->m_kernels0[i].sbias, 0, CConvolver_v->fm0Sub[i]);
      lpiImage_lpiReleaseImage(&fm0_tmp);
    }
  CConvolver_v->fm1In = (struct lpiImage **) calloc(20, sizeof(struct lpiImage *));
  CConvolver_v->fm1Sub = (struct lpiImage **) calloc(14, sizeof(struct lpiImage *));
  for (i = 0; i < 8; i++)
    CConvolver_v->fm1In[i] = CConvolver_v->fm0Sub[i / 2];
  CConvolver_v->fm1In[8] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[9] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[10] = CConvolver_v->fm0Sub[0];
  CConvolver_v->fm1In[11] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[12] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[13] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[14] = CConvolver_v->fm0Sub[1];
  CConvolver_v->fm1In[15] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[16] = CConvolver_v->fm0Sub[3];
  CConvolver_v->fm1In[17] = CConvolver_v->fm0Sub[2];
  CConvolver_v->fm1In[18] = CConvolver_v->fm0Sub[3];
  CConvolver_v->fm1In[19] = CConvolver_v->fm0Sub[3];
  for (i = 0; i < 8; i++)
    {
      {
        struct lpiImage * fm1_tmp;

        fm1_tmp = CConvolver_CreateImage(current_width - 2, current_height - 2);
        CConvolver_v->fm1Sub[i] = CConvolver_CreateImage((current_width - 2) / 2, (current_height - 2) / 2);
        CConvolver_Convolve(CConvolver_v->fm1In[i], CConvolver_v->m_cnn->m_kernels1[i].kern, CConvolver_v->m_cnn->m_kernels1[i].bias, 3, fm1_tmp);
        CConvolver_SubSample(fm1_tmp, CConvolver_v->m_cnn->m_kernels1[i].coeff, CConvolver_v->m_cnn->m_kernels1[i].sbias, 0, CConvolver_v->fm1Sub[i]);
        lpiImage_lpiReleaseImage(&fm1_tmp);
      }
    }
  for (i = 8; i < 14; i++)
    {
      /* #pragma ompix task untied atnode(here) */
      {
         
        int _ompix_0 = i;
        int _ompix_1 = width;
        int _ompix_2 = height;
        int _ompix_5 = current_width;
        int _ompix_6 = current_height;

                torc_create_ox(2, -1, -1, 0, 0, torc_conv_task2_torctask, (void *) 0, 7, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 2, 1, 0, 2, 1, 0, 1, 1, 0, 1, &_ompix_0, &_ompix_1, &_ompix_2, (void *) CConvolver_v, (void *) img, &_ompix_5, &_ompix_6);
      }
    }
  /* #pragma ompix tasksync  */
  torc_tasksync();
    for (i = 0; i < 4; i++)
    {
      lpiImage_lpiReleaseImage(&CConvolver_v->fm0Sub[i]);
    }
  free(CConvolver_v->fm0Sub);
  free(CConvolver_v->fm1In);
  current_width = (current_width - 2) / 2;
  current_height = (current_height - 2) / 2;
  current_width = current_width - 6 + 1;
  current_height = current_height - 7 + 1;
  CConvolver_v->fm2 = (struct lpiImage **) calloc(14, sizeof(struct lpiImage *));
  for (i = 0; i < 14; i++)
    {
      CConvolver_v->fm2[i] = CConvolver_CreateImage(current_width, current_height);
      CConvolver_ConvolveNeuron(CConvolver_v, CConvolver_v->fm1Sub[i], i, CConvolver_v->fm2[i]);
      lpiImage_lpiReleaseImage(&CConvolver_v->fm1Sub[i]);
    }
  free(CConvolver_v->fm1Sub);
  CConvolver_v->fmOut = CConvolver_CreateImage(current_width, current_height);
  CConvolver_ConvolveOutput(CConvolver_v->fm2, 14, CConvolver_v->m_cnn->m_kernels3[0].kern, CConvolver_v->m_cnn->m_kernels3[0].bias, current_width, current_height, CConvolver_v->fmOut);
  for (i = 0; i < 14; i++)
    {
      lpiImage_lpiReleaseImage(&CConvolver_v->fm2[i]);
    }
  free(CConvolver_v->fm2);
  total_size = current_width * current_height;
  *tt = total_size;
  *hh = current_height;
  *ww = current_width;
  pOut = (float *) CConvolver_v->fmOut->imageData;
  return ((pOut));
}


void CConvolver_DeallocateOutput(struct CConvolver * CConvolver_v)
{
  lpiImage_lpiReleaseImage(&CConvolver_v->fmOut);
}


struct lpiImage * CConvolver_CreateImage8U(int width, int height)
{
  struct lpiImage * img = lpiImage_lpiCreateImage(width, height, sizeof(unsigned char));

  return ((img));
}



