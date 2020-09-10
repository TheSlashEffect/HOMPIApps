/* File generated from [lolac.pc] by OMPi compiler with torc extensions user/chriskar/hompi_start-v2.0.0-463-gde15e85~, Tue Jun 11 17:50:14 2019
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

union _noname1_ {
    unsigned char __c[ 8];
    double __d;
  };

/* (l41) typedef union _noname1_  __huge_val_t; */

static union _noname1_ __huge_val = { {
    0, 0, 0, 0, 0, 0, 0xf0, 0x7f
  } };
union _noname2_ {
    unsigned char __c[ 4];
    float __f;
  };

/* (l39) typedef union _noname2_  __huge_valf_t; */

static union _noname2_ __huge_valf = { {
    0, 0, 0x80, 0x7f
  } };
union _noname3_ {
    unsigned char __c[ 12];
    long double __ld;
  };
static union _noname3_ __huge_vall = { {
    0, 0, 0, 0, 0, 0, 0, 0x80, 0xff, 0x7f, 0, 0
  } };
union _noname4_ {
    unsigned char __c[ 4];
    float __d;
  };
static union _noname4_ __qnan_union = { {
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
enum _noname5_ {
    _IEEE_ = -1, _SVID_, _XOPEN_, _POSIX_, _ISOC_
  };

/* (l354) typedef enum _noname5_  _LIB_VERSION_TYPE; */

extern enum _noname5_ _LIB_VERSION;
struct exception {
    int type;
    char * name;
    double arg1;
    double arg2;
    double retval;
  };
extern int matherr(struct exception * __exc);
void init_tanh();

/* (l216) typedef long unsigned int size_t; */

/* (l328) typedef int wchar_t; */

enum _noname6_ {
    P_ALL, P_PID, P_PGID
  };

/* (l55) typedef enum _noname6_  idtype_t; */

struct _noname7_ {
    int quot;
    int rem;
  };

/* (l62) typedef struct _noname7_  div_t; */

struct _noname8_ {
    long int quot;
    long int rem;
  };

/* (l70) typedef struct _noname8_  ldiv_t; */

struct _noname9_ {
    long long int quot;
    long long int rem;
  };

/* (l82) typedef struct _noname9_  lldiv_t; */

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

/* (l22) typedef int __sig_atomic_t; */

struct _noname10_ {
    unsigned long int __val[ (1024 / (8 * sizeof(unsigned long int)))];
  };

/* (l30) typedef struct _noname10_  __sigset_t; */

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

struct _noname11_ {
    long int (__fds_bits[ 1024 / (8 * (int) sizeof(long int ))]);
  };

/* (l77) typedef struct _noname11_  fd_set; */

/* (l84) typedef __fd_mask fd_mask; */

extern int select(int __nfds, struct _noname11_ (* __readfds), struct _noname11_ (* __writefds), struct _noname11_ (* __exceptfds), struct timeval * __timeout);
extern int pselect(int __nfds, struct _noname11_ (* __readfds), struct _noname11_ (* __writefds), struct _noname11_ (* __exceptfds), const struct timespec * __timeout, const struct _noname10_ (* __sigmask));
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

union _noname12_ {
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

/* (l128) typedef union _noname12_  pthread_mutex_t; */

union _noname13_ {
    char __size[ 4];
    int __align;
  };

/* (l134) typedef union _noname13_  pthread_mutexattr_t; */

union _noname14_ {
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

/* (l154) typedef union _noname14_  pthread_cond_t; */

union _noname15_ {
    char __size[ 4];
    int __align;
  };

/* (l160) typedef union _noname15_  pthread_condattr_t; */

/* (l164) typedef unsigned int pthread_key_t; */

/* (l168) typedef int pthread_once_t; */

union _noname16_ {
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

/* (l222) typedef union _noname16_  pthread_rwlock_t; */

union _noname17_ {
    char __size[ 8];
    long int __align;
  };

/* (l228) typedef union _noname17_  pthread_rwlockattr_t; */

/* (l234) typedef volatile int pthread_spinlock_t; */

union _noname18_ {
    char __size[ 32];
    long int __align;
  };

/* (l243) typedef union _noname18_  pthread_barrier_t; */

union _noname19_ {
    char __size[ 4];
    int __align;
  };

/* (l249) typedef union _noname19_  pthread_barrierattr_t; */

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
extern struct _noname7_ div(int __numer, int __denom);
extern struct _noname8_ ldiv(long int __numer, long int __denom);
extern struct _noname9_ lldiv(long long int __numer, long long int __denom);
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

struct _noname20_ {
    int __count;
    union {
        unsigned int __wch;
        char __wchb[ 4];
      } __value;
  };

/* (l94) typedef struct _noname20_  __mbstate_t; */

struct _noname21_ {
    long int __pos;
    struct _noname20_ __state;
  };

/* (l25) typedef struct _noname21_  _G_fpos_t; */

struct _noname22_ {
    long int __pos;
    struct _noname20_ __state;
  };

/* (l30) typedef struct _noname22_  _G_fpos64_t; */

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
extern int fgetpos(struct _IO_FILE (* __stream), struct _noname21_ (* __pos));
extern int fsetpos(struct _IO_FILE (* __stream), const struct _noname21_ (* __pos));
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
void convolveFine(struct CNN * cnn, double ** input, int width, int height, double ** res);
void convolveRoughly(struct CNN * cnn, double ** input, int width, int height, double ** res);
struct CConvolverFine {
    int m_width;
    int m_height;
    struct CNN * m_cnn;
  };
void CConvolverFine_SetCNN(struct CConvolverFine * CConvolverFine_v, struct CNN * cnn);
float * CConvolverFine_ConvolveFine(struct CConvolverFine * CConvolverFine_v, struct lpiImage * img, float * res);
void CConvolverFine_SecondLayer(struct CConvolverFine * CConvolverFine_v, struct lpiImage ** fmIn, int dispx, int dispy, float * res);
void CConvolverFine_NeuronLayer(struct CConvolverFine * CConvolverFine_v, struct lpiImage ** fm, int dispx1, int dispy1, int dispx2, int dispy2, float * res);
void CConvolverFine_ConvolveNeuron(struct CConvolverFine * CConvolverFine_v, struct lpiImage * fm, int index, struct lpiImage * out);
void CConvolverFine_SubSample(struct lpiImage * fm, float coeff, float bias, int disp, int x_disp, int y_disp, struct lpiImage * sub);
struct lpiImage * CConvolverFine_CreateImage(int width, int height);
struct timezone {
    int tz_minuteswest;
    int tz_dsttime;
  };

/* (l61) typedef struct timezone  * __timezone_ptr_t; */

extern int gettimeofday(struct timeval * __tv, struct timezone * __tz);
extern int settimeofday(const struct timeval * __tv, const struct timezone * __tz);
extern int adjtime(const struct timeval * __delta, struct timeval * __olddelta);
enum __itimer_which {
    ITIMER_REAL = 0, ITIMER_VIRTUAL = 1, ITIMER_PROF = 2
  };
struct itimerval {
    struct timeval it_interval;
    struct timeval it_value;
  };

/* (l120) typedef int __itimer_which_t; */

extern int getitimer(int __which, struct itimerval * __value);
extern int setitimer(int __which, const struct itimerval * __new, struct itimerval * __old);
extern int utimes(const char * __file, const struct timeval __tvp[ 2]);
extern int lutimes(const char * __file, const struct timeval __tvp[ 2]);
extern int futimes(int __fd, const struct timeval __tvp[ 2]);
struct sched_param {
    int __sched_priority;
  };
struct __sched_param {
    int __sched_priority;
  };

/* (l119) typedef unsigned long int __cpu_mask; */

struct _noname23_ {
    unsigned long int (__bits[ 1024 / (8 * sizeof(unsigned long int ))]);
  };

/* (l129) typedef struct _noname23_  cpu_set_t; */

extern int __sched_cpucount(long unsigned int __setsize, const struct _noname23_ (* __setp));
extern struct _noname23_ (* __sched_cpualloc(long unsigned int __count));
extern void __sched_cpufree(struct _noname23_ (* __set));
extern int sched_setparam(int __pid, const struct sched_param * __param);
extern int sched_getparam(int __pid, struct sched_param * __param);
extern int sched_setscheduler(int __pid, int __policy, const struct sched_param * __param);
extern int sched_getscheduler(int __pid);
extern int sched_yield(void);
extern int sched_get_priority_max(int __algorithm);
extern int sched_get_priority_min(int __algorithm);
extern int sched_rr_get_interval(int __pid, struct timespec * __t);
struct tm {
    int tm_sec;
    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
    int tm_yday;
    int tm_isdst;
    long int tm_gmtoff;
    const char * tm_zone;
  };
struct itimerspec {
    struct timespec it_interval;
    struct timespec it_value;
  };
struct sigevent;
extern long int clock(void);
extern long int time(long int (* __timer));
extern double difftime(long int __time1, long int __time0);
extern long int mktime(struct tm * __tp);
extern long unsigned int strftime(char * __s, long unsigned int __maxsize, const char * __format, const struct tm * __tp);
struct __locale_struct {
    struct __locale_data * __locales[ 13];
    const unsigned short int * __ctype_b;
    const int * __ctype_tolower;
    const int * __ctype_toupper;
    const char * __names[ 13];
  };

/* (l39) typedef struct __locale_struct  * __locale_t; */

/* (l42) typedef __locale_t locale_t; */

extern long unsigned int strftime_l(char * __s, long unsigned int __maxsize, const char * __format, const struct tm * __tp, struct __locale_struct * __loc);
extern struct tm * gmtime(const long int (* __timer));
extern struct tm * localtime(const long int (* __timer));
extern struct tm * gmtime_r(const long int (* __timer), struct tm * __tp);
extern struct tm * localtime_r(const long int (* __timer), struct tm * __tp);
extern char * asctime(const struct tm * __tp);
extern char * ctime(const long int (* __timer));
extern char * asctime_r(const struct tm * __tp, char * __buf);
extern char * ctime_r(const long int (* __timer), char * __buf);
extern char * __tzname[ 2];
extern int __daylight;
extern long int __timezone;
extern char * tzname[ 2];
extern void tzset(void);
extern int daylight;
extern long int timezone;
extern int stime(const long int (* __when));
extern long int timegm(struct tm * __tp);
extern long int timelocal(struct tm * __tp);
extern int dysize(int __year);
extern int nanosleep(const struct timespec * __requested_time, struct timespec * __remaining);
extern int clock_getres(int __clock_id, struct timespec * __res);
extern int clock_gettime(int __clock_id, struct timespec * __tp);
extern int clock_settime(int __clock_id, const struct timespec * __tp);
extern int clock_nanosleep(int __clock_id, int __flags, const struct timespec * __req, struct timespec * __rem);
extern int clock_getcpuclockid(int __pid, int (* __clock_id));
extern int timer_create(int __clock_id, struct sigevent * __evp, void * (* __timerid));
extern int timer_delete(void * __timerid);
extern int timer_settime(void * __timerid, int __flags, const struct itimerspec * __value, struct itimerspec * __ovalue);
extern int timer_gettime(void * __timerid, struct itimerspec * __value);
extern int timer_getoverrun(void * __timerid);
extern int timespec_get(struct timespec * __ts, int __base);

/* (l31) typedef long int __jmp_buf[ 8]; */

enum {
    PTHREAD_CREATE_JOINABLE, PTHREAD_CREATE_DETACHED
  };
enum {
    PTHREAD_MUTEX_TIMED_NP, PTHREAD_MUTEX_RECURSIVE_NP, PTHREAD_MUTEX_ERRORCHECK_NP, PTHREAD_MUTEX_ADAPTIVE_NP, PTHREAD_MUTEX_NORMAL = PTHREAD_MUTEX_TIMED_NP, PTHREAD_MUTEX_RECURSIVE = PTHREAD_MUTEX_RECURSIVE_NP, PTHREAD_MUTEX_ERRORCHECK = PTHREAD_MUTEX_ERRORCHECK_NP, PTHREAD_MUTEX_DEFAULT = PTHREAD_MUTEX_NORMAL
  };
enum {
    PTHREAD_MUTEX_STALLED, PTHREAD_MUTEX_STALLED_NP = PTHREAD_MUTEX_STALLED, PTHREAD_MUTEX_ROBUST, PTHREAD_MUTEX_ROBUST_NP = PTHREAD_MUTEX_ROBUST
  };
enum {
    PTHREAD_PRIO_NONE, PTHREAD_PRIO_INHERIT, PTHREAD_PRIO_PROTECT
  };
enum {
    PTHREAD_RWLOCK_PREFER_READER_NP, PTHREAD_RWLOCK_PREFER_WRITER_NP, PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP, PTHREAD_RWLOCK_DEFAULT_NP = PTHREAD_RWLOCK_PREFER_READER_NP
  };
enum {
    PTHREAD_INHERIT_SCHED, PTHREAD_EXPLICIT_SCHED
  };
enum {
    PTHREAD_SCOPE_SYSTEM, PTHREAD_SCOPE_PROCESS
  };
enum {
    PTHREAD_PROCESS_PRIVATE, PTHREAD_PROCESS_SHARED
  };
struct _pthread_cleanup_buffer {
    void (* __routine)(void *);
    void * __arg;
    int __canceltype;
    struct _pthread_cleanup_buffer * __prev;
  };
enum {
    PTHREAD_CANCEL_ENABLE, PTHREAD_CANCEL_DISABLE
  };
enum {
    PTHREAD_CANCEL_DEFERRED, PTHREAD_CANCEL_ASYNCHRONOUS
  };
extern int pthread_create(unsigned long int (* __newthread), const union pthread_attr_t (* __attr), void * (* __start_routine)(void *), void * __arg);
extern void pthread_exit(void * __retval);
extern int pthread_join(unsigned long int __th, void ** __thread_return);
extern int pthread_detach(unsigned long int __th);
extern unsigned long int pthread_self(void);
extern int pthread_equal(unsigned long int __thread1, unsigned long int __thread2);
extern int pthread_attr_init(union pthread_attr_t (* __attr));
extern int pthread_attr_destroy(union pthread_attr_t (* __attr));
extern int pthread_attr_getdetachstate(const union pthread_attr_t (* __attr), int * __detachstate);
extern int pthread_attr_setdetachstate(union pthread_attr_t (* __attr), int __detachstate);
extern int pthread_attr_getguardsize(const union pthread_attr_t (* __attr), long unsigned int (* __guardsize));
extern int pthread_attr_setguardsize(union pthread_attr_t (* __attr), long unsigned int __guardsize);
extern int pthread_attr_getschedparam(const union pthread_attr_t (* __attr), struct sched_param * __param);
extern int pthread_attr_setschedparam(union pthread_attr_t (* __attr), const struct sched_param * __param);
extern int pthread_attr_getschedpolicy(const union pthread_attr_t (* __attr), int * __policy);
extern int pthread_attr_setschedpolicy(union pthread_attr_t (* __attr), int __policy);
extern int pthread_attr_getinheritsched(const union pthread_attr_t (* __attr), int * __inherit);
extern int pthread_attr_setinheritsched(union pthread_attr_t (* __attr), int __inherit);
extern int pthread_attr_getscope(const union pthread_attr_t (* __attr), int * __scope);
extern int pthread_attr_setscope(union pthread_attr_t (* __attr), int __scope);
extern int pthread_attr_getstackaddr(const union pthread_attr_t (* __attr), void ** __stackaddr);
extern int pthread_attr_setstackaddr(union pthread_attr_t (* __attr), void * __stackaddr);
extern int pthread_attr_getstacksize(const union pthread_attr_t (* __attr), long unsigned int (* __stacksize));
extern int pthread_attr_setstacksize(union pthread_attr_t (* __attr), long unsigned int __stacksize);
extern int pthread_attr_getstack(const union pthread_attr_t (* __attr), void ** __stackaddr, long unsigned int (* __stacksize));
extern int pthread_attr_setstack(union pthread_attr_t (* __attr), void * __stackaddr, long unsigned int __stacksize);
extern int pthread_setschedparam(unsigned long int __target_thread, int __policy, const struct sched_param * __param);
extern int pthread_getschedparam(unsigned long int __target_thread, int * __policy, struct sched_param * __param);
extern int pthread_setschedprio(unsigned long int __target_thread, int __prio);
extern int pthread_once(int (* __once_control), void (* __init_routine)(void));
extern int pthread_setcancelstate(int __state, int * __oldstate);
extern int pthread_setcanceltype(int __type, int * __oldtype);
extern int pthread_cancel(unsigned long int __th);
extern void pthread_testcancel(void);
struct _noname24_ {
    struct {
        long int __cancel_jmp_buf[ 8];
        int __mask_was_saved;
      } __cancel_jmp_buf[ 1];
    void * __pad[ 4];
  };

/* (l531) typedef struct _noname24_  __pthread_unwind_buf_t; */

struct __pthread_cleanup_frame {
    void (* __cancel_routine)(void *);
    void * __cancel_arg;
    int __do_it;
    int __cancel_type;
  };
extern void __pthread_register_cancel(struct _noname24_ (* __buf));
extern void __pthread_unregister_cancel(struct _noname24_ (* __buf));
extern void __pthread_unwind_next(struct _noname24_ (* __buf));
struct __jmp_buf_tag;
extern int __sigsetjmp(struct __jmp_buf_tag * __env, int __savemask);
extern int pthread_mutex_init(union _noname12_ (* __mutex), const union _noname13_ (* __mutexattr));
extern int pthread_mutex_destroy(union _noname12_ (* __mutex));
extern int pthread_mutex_trylock(union _noname12_ (* __mutex));
extern int pthread_mutex_lock(union _noname12_ (* __mutex));
extern int pthread_mutex_timedlock(union _noname12_ (* __mutex), const struct timespec * __abstime);
extern int pthread_mutex_unlock(union _noname12_ (* __mutex));
extern int pthread_mutex_getprioceiling(const union _noname12_ (* __mutex), int * __prioceiling);
extern int pthread_mutex_setprioceiling(union _noname12_ (* __mutex), int __prioceiling, int * __old_ceiling);
extern int pthread_mutex_consistent(union _noname12_ (* __mutex));
extern int pthread_mutexattr_init(union _noname13_ (* __attr));
extern int pthread_mutexattr_destroy(union _noname13_ (* __attr));
extern int pthread_mutexattr_getpshared(const union _noname13_ (* __attr), int * __pshared);
extern int pthread_mutexattr_setpshared(union _noname13_ (* __attr), int __pshared);
extern int pthread_mutexattr_gettype(const union _noname13_ (* __attr), int * __kind);
extern int pthread_mutexattr_settype(union _noname13_ (* __attr), int __kind);
extern int pthread_mutexattr_getprotocol(const union _noname13_ (* __attr), int * __protocol);
extern int pthread_mutexattr_setprotocol(union _noname13_ (* __attr), int __protocol);
extern int pthread_mutexattr_getprioceiling(const union _noname13_ (* __attr), int * __prioceiling);
extern int pthread_mutexattr_setprioceiling(union _noname13_ (* __attr), int __prioceiling);
extern int pthread_mutexattr_getrobust(const union _noname13_ (* __attr), int * __robustness);
extern int pthread_mutexattr_setrobust(union _noname13_ (* __attr), int __robustness);
extern int pthread_rwlock_init(union _noname16_ (* __rwlock), const union _noname17_ (* __attr));
extern int pthread_rwlock_destroy(union _noname16_ (* __rwlock));
extern int pthread_rwlock_rdlock(union _noname16_ (* __rwlock));
extern int pthread_rwlock_tryrdlock(union _noname16_ (* __rwlock));
extern int pthread_rwlock_timedrdlock(union _noname16_ (* __rwlock), const struct timespec * __abstime);
extern int pthread_rwlock_wrlock(union _noname16_ (* __rwlock));
extern int pthread_rwlock_trywrlock(union _noname16_ (* __rwlock));
extern int pthread_rwlock_timedwrlock(union _noname16_ (* __rwlock), const struct timespec * __abstime);
extern int pthread_rwlock_unlock(union _noname16_ (* __rwlock));
extern int pthread_rwlockattr_init(union _noname17_ (* __attr));
extern int pthread_rwlockattr_destroy(union _noname17_ (* __attr));
extern int pthread_rwlockattr_getpshared(const union _noname17_ (* __attr), int * __pshared);
extern int pthread_rwlockattr_setpshared(union _noname17_ (* __attr), int __pshared);
extern int pthread_rwlockattr_getkind_np(const union _noname17_ (* __attr), int * __pref);
extern int pthread_rwlockattr_setkind_np(union _noname17_ (* __attr), int __pref);
extern int pthread_cond_init(union _noname14_ (* __cond), const union _noname15_ (* __cond_attr));
extern int pthread_cond_destroy(union _noname14_ (* __cond));
extern int pthread_cond_signal(union _noname14_ (* __cond));
extern int pthread_cond_broadcast(union _noname14_ (* __cond));
extern int pthread_cond_wait(union _noname14_ (* __cond), union _noname12_ (* __mutex));
extern int pthread_cond_timedwait(union _noname14_ (* __cond), union _noname12_ (* __mutex), const struct timespec * __abstime);
extern int pthread_condattr_init(union _noname15_ (* __attr));
extern int pthread_condattr_destroy(union _noname15_ (* __attr));
extern int pthread_condattr_getpshared(const union _noname15_ (* __attr), int * __pshared);
extern int pthread_condattr_setpshared(union _noname15_ (* __attr), int __pshared);
extern int pthread_condattr_getclock(const union _noname15_ (* __attr), int (* __clock_id));
extern int pthread_condattr_setclock(union _noname15_ (* __attr), int __clock_id);
extern int pthread_spin_init(volatile int (* __lock), int __pshared);
extern int pthread_spin_destroy(volatile int (* __lock));
extern int pthread_spin_lock(volatile int (* __lock));
extern int pthread_spin_trylock(volatile int (* __lock));
extern int pthread_spin_unlock(volatile int (* __lock));
extern int pthread_barrier_init(union _noname18_ (* __barrier), const union _noname19_ (* __attr), unsigned int __count);
extern int pthread_barrier_destroy(union _noname18_ (* __barrier));
extern int pthread_barrier_wait(union _noname18_ (* __barrier));
extern int pthread_barrierattr_init(union _noname19_ (* __attr));
extern int pthread_barrierattr_destroy(union _noname19_ (* __attr));
extern int pthread_barrierattr_getpshared(const union _noname19_ (* __attr), int * __pshared);
extern int pthread_barrierattr_setpshared(union _noname19_ (* __attr), int __pshared);
extern int pthread_key_create(unsigned int (* __key), void (* __destr_function)(void *));
extern int pthread_key_delete(unsigned int __key);
extern void * pthread_getspecific(unsigned int __key);
extern int pthread_setspecific(unsigned int __key, const void * __pointer);
extern int pthread_getcpuclockid(unsigned long int __thread_id, int (* __clock_id));
extern int pthread_atfork(void (* __prepare)(void), void (* __parent)(void), void (* __child)(void));

/* (l149) typedef long int ptrdiff_t; */

struct _noname25_ {
    long long __max_align_ll;
    long double __max_align_ld;
  };

/* (l429) typedef struct _noname25_  max_align_t; */

/* (l326) typedef ptrdiff_t MPI_Aint; */

/* (l327) typedef long long MPI_Offset; */

/* (l328) typedef long long MPI_Count; */

/* (l329) typedef struct ompi_communicator_t  * MPI_Comm; */

/* (l330) typedef struct ompi_datatype_t  * MPI_Datatype; */

/* (l331) typedef struct ompi_errhandler_t  * MPI_Errhandler; */

/* (l333) typedef struct ompi_file_t  * MPI_File; */

/* (l335) typedef struct ompi_group_t  * MPI_Group; */

/* (l336) typedef struct ompi_info_t  * MPI_Info; */

/* (l337) typedef struct ompi_op_t  * MPI_Op; */

/* (l338) typedef struct ompi_request_t  * MPI_Request; */

/* (l339) typedef struct ompi_message_t  * MPI_Message; */

/* (l340) typedef struct ompi_status_public_t  MPI_Status; */

/* (l341) typedef struct ompi_win_t  * MPI_Win; */

/* (l342) typedef struct mca_base_var_enum_t  * MPI_T_enum; */

/* (l343) typedef struct ompi_mpit_cvar_handle_t  * MPI_T_cvar_handle; */

/* (l344) typedef struct mca_base_pvar_handle_t  * MPI_T_pvar_handle; */

/* (l345) typedef struct mca_base_pvar_session_t  * MPI_T_pvar_session; */

struct ompi_status_public_t {
    int MPI_SOURCE;
    int MPI_TAG;
    int MPI_ERROR;
    int _cancelled;
    long unsigned int _ucount;
  };

/* (l363) typedef struct ompi_status_public_t  ompi_status_public_t; */

/* (l374) typedef int MPI_Copy_function(MPI_Comm, int, void *, void *, void *, int *); */

/* (l375) typedef int MPI_Delete_function(MPI_Comm, int, void *, void *); */

/* (l376) typedef int MPI_Datarep_extent_function(MPI_Datatype, MPI_Aint *, void *); */

/* (l378) typedef int MPI_Datarep_conversion_function(void *, MPI_Datatype, int, void *, MPI_Offset, void *); */

/* (l379) typedef void MPI_Comm_errhandler_function(MPI_Comm *, int *, ...); */

/* (l381) typedef MPI_Comm_errhandler_function MPI_Comm_errhandler_fn; */

/* (l388) typedef void ompi_file_errhandler_fn(MPI_File *, int *, ...); */

/* (l390) typedef ompi_file_errhandler_fn MPI_File_errhandler_fn; */

/* (l391) typedef ompi_file_errhandler_fn MPI_File_errhandler_function; */

/* (l396) typedef void MPI_Win_errhandler_function(MPI_Win *, int *, ...); */

/* (l398) typedef MPI_Win_errhandler_function MPI_Win_errhandler_fn; */

/* (l402) typedef void MPI_Handler_function(MPI_Comm *, int *, ...); */

/* (l403) typedef void MPI_User_function(void *, void *, int *, MPI_Datatype *); */

/* (l405) typedef int MPI_Comm_copy_attr_function(MPI_Comm, int, void *, void *, void *, int *); */

/* (l406) typedef int MPI_Comm_delete_attr_function(MPI_Comm, int, void *, void *); */

/* (l408) typedef int MPI_Type_copy_attr_function(MPI_Datatype, int, void *, void *, void *, int *); */

/* (l410) typedef int MPI_Type_delete_attr_function(MPI_Datatype, int, void *, void *); */

/* (l412) typedef int MPI_Win_copy_attr_function(MPI_Win, int, void *, void *, void *, int *); */

/* (l413) typedef int MPI_Win_delete_attr_function(MPI_Win, int, void *, void *); */

/* (l414) typedef int MPI_Grequest_query_function(void *, MPI_Status *); */

/* (l415) typedef int MPI_Grequest_free_function(void *); */

/* (l416) typedef int MPI_Grequest_cancel_function(void *, int); */

enum {
    MPI_TAG_UB, MPI_HOST, MPI_IO, MPI_WTIME_IS_GLOBAL, MPI_APPNUM, MPI_LASTUSEDCODE, MPI_UNIVERSE_SIZE, MPI_WIN_BASE, MPI_WIN_SIZE, MPI_WIN_DISP_UNIT, MPI_WIN_CREATE_FLAVOR, MPI_WIN_MODEL, IMPI_CLIENT_SIZE, IMPI_CLIENT_COLOR, IMPI_HOST_SIZE, IMPI_HOST_COLOR
  };
enum {
    MPI_IDENT, MPI_CONGRUENT, MPI_SIMILAR, MPI_UNEQUAL
  };
enum {
    MPI_THREAD_SINGLE, MPI_THREAD_FUNNELED, MPI_THREAD_SERIALIZED, MPI_THREAD_MULTIPLE
  };
enum {
    MPI_COMBINER_NAMED, MPI_COMBINER_DUP, MPI_COMBINER_CONTIGUOUS, MPI_COMBINER_VECTOR, MPI_COMBINER_HVECTOR_INTEGER, MPI_COMBINER_HVECTOR, MPI_COMBINER_INDEXED, MPI_COMBINER_HINDEXED_INTEGER, MPI_COMBINER_HINDEXED, MPI_COMBINER_INDEXED_BLOCK, MPI_COMBINER_STRUCT_INTEGER, MPI_COMBINER_STRUCT, MPI_COMBINER_SUBARRAY, MPI_COMBINER_DARRAY, MPI_COMBINER_F90_REAL, MPI_COMBINER_F90_COMPLEX, MPI_COMBINER_F90_INTEGER, MPI_COMBINER_RESIZED, MPI_COMBINER_HINDEXED_BLOCK
  };
enum {
    MPI_COMM_TYPE_SHARED, OMPI_COMM_TYPE_HWTHREAD, OMPI_COMM_TYPE_CORE, OMPI_COMM_TYPE_L1CACHE, OMPI_COMM_TYPE_L2CACHE, OMPI_COMM_TYPE_L3CACHE, OMPI_COMM_TYPE_SOCKET, OMPI_COMM_TYPE_NUMA, OMPI_COMM_TYPE_BOARD, OMPI_COMM_TYPE_HOST, OMPI_COMM_TYPE_CU, OMPI_COMM_TYPE_CLUSTER
  };
enum {
    MPI_T_VERBOSITY_USER_BASIC, MPI_T_VERBOSITY_USER_DETAIL, MPI_T_VERBOSITY_USER_ALL, MPI_T_VERBOSITY_TUNER_BASIC, MPI_T_VERBOSITY_TUNER_DETAIL, MPI_T_VERBOSITY_TUNER_ALL, MPI_T_VERBOSITY_MPIDEV_BASIC, MPI_T_VERBOSITY_MPIDEV_DETAIL, MPI_T_VERBOSITY_MPIDEV_ALL
  };
enum {
    MPI_T_SCOPE_CONSTANT, MPI_T_SCOPE_READONLY, MPI_T_SCOPE_LOCAL, MPI_T_SCOPE_GROUP, MPI_T_SCOPE_GROUP_EQ, MPI_T_SCOPE_ALL, MPI_T_SCOPE_ALL_EQ
  };
enum {
    MPI_T_BIND_NO_OBJECT, MPI_T_BIND_MPI_COMM, MPI_T_BIND_MPI_DATATYPE, MPI_T_BIND_MPI_ERRHANDLER, MPI_T_BIND_MPI_FILE, MPI_T_BIND_MPI_GROUP, MPI_T_BIND_MPI_OP, MPI_T_BIND_MPI_REQUEST, MPI_T_BIND_MPI_WIN, MPI_T_BIND_MPI_MESSAGE, MPI_T_BIND_MPI_INFO
  };
enum {
    MPI_T_PVAR_CLASS_STATE, MPI_T_PVAR_CLASS_LEVEL, MPI_T_PVAR_CLASS_SIZE, MPI_T_PVAR_CLASS_PERCENTAGE, MPI_T_PVAR_CLASS_HIGHWATERMARK, MPI_T_PVAR_CLASS_LOWWATERMARK, MPI_T_PVAR_CLASS_COUNTER, MPI_T_PVAR_CLASS_AGGREGATE, MPI_T_PVAR_CLASS_TIMER, MPI_T_PVAR_CLASS_GENERIC
  };
int OMPI_C_MPI_TYPE_NULL_DELETE_FN(struct ompi_datatype_t * datatype, int type_keyval, void * attribute_val_out, void * extra_state);
int OMPI_C_MPI_TYPE_NULL_COPY_FN(struct ompi_datatype_t * datatype, int type_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_TYPE_DUP_FN(struct ompi_datatype_t * datatype, int type_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_COMM_NULL_DELETE_FN(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val_out, void * extra_state);
int OMPI_C_MPI_COMM_NULL_COPY_FN(struct ompi_communicator_t * comm, int comm_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_COMM_DUP_FN(struct ompi_communicator_t * comm, int comm_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_NULL_DELETE_FN(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val_out, void * extra_state);
int OMPI_C_MPI_NULL_COPY_FN(struct ompi_communicator_t * comm, int comm_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_DUP_FN(struct ompi_communicator_t * comm, int comm_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_WIN_NULL_DELETE_FN(struct ompi_win_t * window, int win_keyval, void * attribute_val_out, void * extra_state);
int OMPI_C_MPI_WIN_NULL_COPY_FN(struct ompi_win_t * window, int win_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
int OMPI_C_MPI_WIN_DUP_FN(struct ompi_win_t * window, int win_keyval, void * extra_state, void * attribute_val_in, void * attribute_val_out, int * flag);
extern struct ompi_predefined_communicator_t ompi_mpi_comm_world;
extern struct ompi_predefined_communicator_t ompi_mpi_comm_self;
extern struct ompi_predefined_communicator_t ompi_mpi_comm_null;
extern struct ompi_predefined_group_t ompi_mpi_group_empty;
extern struct ompi_predefined_group_t ompi_mpi_group_null;
extern struct ompi_predefined_request_t ompi_request_null;
extern struct ompi_predefined_message_t ompi_message_null;
extern struct ompi_predefined_message_t ompi_message_no_proc;
extern struct ompi_predefined_op_t ompi_mpi_op_null;
extern struct ompi_predefined_op_t ompi_mpi_op_min;
extern struct ompi_predefined_op_t ompi_mpi_op_max;
extern struct ompi_predefined_op_t ompi_mpi_op_sum;
extern struct ompi_predefined_op_t ompi_mpi_op_prod;
extern struct ompi_predefined_op_t ompi_mpi_op_land;
extern struct ompi_predefined_op_t ompi_mpi_op_band;
extern struct ompi_predefined_op_t ompi_mpi_op_lor;
extern struct ompi_predefined_op_t ompi_mpi_op_bor;
extern struct ompi_predefined_op_t ompi_mpi_op_lxor;
extern struct ompi_predefined_op_t ompi_mpi_op_bxor;
extern struct ompi_predefined_op_t ompi_mpi_op_maxloc;
extern struct ompi_predefined_op_t ompi_mpi_op_minloc;
extern struct ompi_predefined_op_t ompi_mpi_op_replace;
extern struct ompi_predefined_op_t ompi_mpi_op_no_op;
extern struct ompi_predefined_datatype_t ompi_mpi_datatype_null;
extern struct ompi_predefined_datatype_t ompi_mpi_lb;
extern struct ompi_predefined_datatype_t ompi_mpi_ub;
extern struct ompi_predefined_datatype_t ompi_mpi_char;
extern struct ompi_predefined_datatype_t ompi_mpi_signed_char;
extern struct ompi_predefined_datatype_t ompi_mpi_unsigned_char;
extern struct ompi_predefined_datatype_t ompi_mpi_byte;
extern struct ompi_predefined_datatype_t ompi_mpi_short;
extern struct ompi_predefined_datatype_t ompi_mpi_unsigned_short;
extern struct ompi_predefined_datatype_t ompi_mpi_int;
extern struct ompi_predefined_datatype_t ompi_mpi_unsigned;
extern struct ompi_predefined_datatype_t ompi_mpi_long;
extern struct ompi_predefined_datatype_t ompi_mpi_unsigned_long;
extern struct ompi_predefined_datatype_t ompi_mpi_long_long_int;
extern struct ompi_predefined_datatype_t ompi_mpi_unsigned_long_long;
extern struct ompi_predefined_datatype_t ompi_mpi_float;
extern struct ompi_predefined_datatype_t ompi_mpi_double;
extern struct ompi_predefined_datatype_t ompi_mpi_long_double;
extern struct ompi_predefined_datatype_t ompi_mpi_wchar;
extern struct ompi_predefined_datatype_t ompi_mpi_packed;
extern struct ompi_predefined_datatype_t ompi_mpi_cxx_bool;
extern struct ompi_predefined_datatype_t ompi_mpi_cxx_cplex;
extern struct ompi_predefined_datatype_t ompi_mpi_cxx_dblcplex;
extern struct ompi_predefined_datatype_t ompi_mpi_cxx_ldblcplex;
extern struct ompi_predefined_datatype_t ompi_mpi_logical;
extern struct ompi_predefined_datatype_t ompi_mpi_character;
extern struct ompi_predefined_datatype_t ompi_mpi_integer;
extern struct ompi_predefined_datatype_t ompi_mpi_real;
extern struct ompi_predefined_datatype_t ompi_mpi_dblprec;
extern struct ompi_predefined_datatype_t ompi_mpi_cplex;
extern struct ompi_predefined_datatype_t ompi_mpi_dblcplex;
extern struct ompi_predefined_datatype_t ompi_mpi_ldblcplex;
extern struct ompi_predefined_datatype_t ompi_mpi_2int;
extern struct ompi_predefined_datatype_t ompi_mpi_2integer;
extern struct ompi_predefined_datatype_t ompi_mpi_2real;
extern struct ompi_predefined_datatype_t ompi_mpi_2dblprec;
extern struct ompi_predefined_datatype_t ompi_mpi_2cplex;
extern struct ompi_predefined_datatype_t ompi_mpi_2dblcplex;
extern struct ompi_predefined_datatype_t ompi_mpi_float_int;
extern struct ompi_predefined_datatype_t ompi_mpi_double_int;
extern struct ompi_predefined_datatype_t ompi_mpi_longdbl_int;
extern struct ompi_predefined_datatype_t ompi_mpi_short_int;
extern struct ompi_predefined_datatype_t ompi_mpi_long_int;
extern struct ompi_predefined_datatype_t ompi_mpi_logical1;
extern struct ompi_predefined_datatype_t ompi_mpi_logical2;
extern struct ompi_predefined_datatype_t ompi_mpi_logical4;
extern struct ompi_predefined_datatype_t ompi_mpi_logical8;
extern struct ompi_predefined_datatype_t ompi_mpi_integer1;
extern struct ompi_predefined_datatype_t ompi_mpi_integer2;
extern struct ompi_predefined_datatype_t ompi_mpi_integer4;
extern struct ompi_predefined_datatype_t ompi_mpi_integer8;
extern struct ompi_predefined_datatype_t ompi_mpi_integer16;
extern struct ompi_predefined_datatype_t ompi_mpi_real2;
extern struct ompi_predefined_datatype_t ompi_mpi_real4;
extern struct ompi_predefined_datatype_t ompi_mpi_real8;
extern struct ompi_predefined_datatype_t ompi_mpi_real16;
extern struct ompi_predefined_datatype_t ompi_mpi_complex8;
extern struct ompi_predefined_datatype_t ompi_mpi_complex16;
extern struct ompi_predefined_datatype_t ompi_mpi_complex32;
extern struct ompi_predefined_datatype_t ompi_mpi_int8_t;
extern struct ompi_predefined_datatype_t ompi_mpi_uint8_t;
extern struct ompi_predefined_datatype_t ompi_mpi_int16_t;
extern struct ompi_predefined_datatype_t ompi_mpi_uint16_t;
extern struct ompi_predefined_datatype_t ompi_mpi_int32_t;
extern struct ompi_predefined_datatype_t ompi_mpi_uint32_t;
extern struct ompi_predefined_datatype_t ompi_mpi_int64_t;
extern struct ompi_predefined_datatype_t ompi_mpi_uint64_t;
extern struct ompi_predefined_datatype_t ompi_mpi_aint;
extern struct ompi_predefined_datatype_t ompi_mpi_offset;
extern struct ompi_predefined_datatype_t ompi_mpi_count;
extern struct ompi_predefined_datatype_t ompi_mpi_c_bool;
extern struct ompi_predefined_datatype_t ompi_mpi_c_complex;
extern struct ompi_predefined_datatype_t ompi_mpi_c_float_complex;
extern struct ompi_predefined_datatype_t ompi_mpi_c_double_complex;
extern struct ompi_predefined_datatype_t ompi_mpi_c_long_double_complex;
extern struct ompi_predefined_errhandler_t ompi_mpi_errhandler_null;
extern struct ompi_predefined_errhandler_t ompi_mpi_errors_are_fatal;
extern struct ompi_predefined_errhandler_t ompi_mpi_errors_return;
extern struct ompi_predefined_win_t ompi_mpi_win_null;
extern struct ompi_predefined_file_t ompi_mpi_file_null;
extern struct ompi_predefined_info_t ompi_mpi_info_null;
extern struct ompi_predefined_info_t ompi_mpi_info_env;
extern int * MPI_F_STATUS_IGNORE;
extern int * MPI_F_STATUSES_IGNORE;
int MPI_Abort(struct ompi_communicator_t * comm, int errorcode);
int MPI_Accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win);
int MPI_Add_error_class(int * errorclass);
int MPI_Add_error_code(int errorclass, int * errorcode);
int MPI_Add_error_string(int errorcode, const char * string);
int MPI_Address(void * location, long int (* address));
int MPI_Allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Iallgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Iallgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Alloc_mem(long int size, struct ompi_info_t * info, void * baseptr);
int MPI_Allreduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int MPI_Iallreduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ialltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ialltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Alltoallw(const void * sendbuf, const int sendcounts[], const int sdispls[], const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const int rdispls[], const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm);
int MPI_Ialltoallw(const void * sendbuf, const int sendcounts[], const int sdispls[], const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const int rdispls[], const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Attr_delete(struct ompi_communicator_t * comm, int keyval);
int MPI_Attr_get(struct ompi_communicator_t * comm, int keyval, void * attribute_val, int * flag);
int MPI_Attr_put(struct ompi_communicator_t * comm, int keyval, void * attribute_val);
int MPI_Barrier(struct ompi_communicator_t * comm);
int MPI_Ibarrier(struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Bcast(void * buffer, int count, struct ompi_datatype_t * datatype, int root, struct ompi_communicator_t * comm);
int MPI_Bsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int MPI_Ibcast(void * buffer, int count, struct ompi_datatype_t * datatype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Bsend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Buffer_attach(void * buffer, int size);
int MPI_Buffer_detach(void * buffer, int * size);
int MPI_Cancel(struct ompi_request_t * (* request));
int MPI_Cart_coords(struct ompi_communicator_t * comm, int rank, int maxdims, int coords[]);
int MPI_Cart_create(struct ompi_communicator_t * old_comm, int ndims, const int dims[], const int periods[], int reorder, struct ompi_communicator_t * (* comm_cart));
int MPI_Cart_get(struct ompi_communicator_t * comm, int maxdims, int dims[], int periods[], int coords[]);
int MPI_Cart_map(struct ompi_communicator_t * comm, int ndims, const int dims[], const int periods[], int * newrank);
int MPI_Cart_rank(struct ompi_communicator_t * comm, const int coords[], int * rank);
int MPI_Cart_shift(struct ompi_communicator_t * comm, int direction, int disp, int * rank_source, int * rank_dest);
int MPI_Cart_sub(struct ompi_communicator_t * comm, const int remain_dims[], struct ompi_communicator_t * (* new_comm));
int MPI_Cartdim_get(struct ompi_communicator_t * comm, int * ndims);
int MPI_Close_port(const char * port_name);
int MPI_Comm_accept(const char * port_name, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_c2f(struct ompi_communicator_t * comm);
int MPI_Comm_call_errhandler(struct ompi_communicator_t * comm, int errorcode);
int MPI_Comm_compare(struct ompi_communicator_t * comm1, struct ompi_communicator_t * comm2, int * result);
int MPI_Comm_connect(const char * port_name, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_create_errhandler(void (* function)(struct ompi_communicator_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int MPI_Comm_create_keyval(int (* comm_copy_attr_fn)(struct ompi_communicator_t *, int, void *, void *, void *, int *), int (* comm_delete_attr_fn)(struct ompi_communicator_t *, int, void *, void *), int * comm_keyval, void * extra_state);
int MPI_Comm_create_group(struct ompi_communicator_t * comm, struct ompi_group_t * group, int tag, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_create(struct ompi_communicator_t * comm, struct ompi_group_t * group, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_delete_attr(struct ompi_communicator_t * comm, int comm_keyval);
int MPI_Comm_disconnect(struct ompi_communicator_t * (* comm));
int MPI_Comm_dup(struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_idup(struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm), struct ompi_request_t * (* request));
int MPI_Comm_dup_with_info(struct ompi_communicator_t * comm, struct ompi_info_t * info, struct ompi_communicator_t * (* newcomm));
struct ompi_communicator_t * MPI_Comm_f2c(int comm);
int MPI_Comm_free_keyval(int * comm_keyval);
int MPI_Comm_free(struct ompi_communicator_t * (* comm));
int MPI_Comm_get_attr(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val, int * flag);
int MPI_Dist_graph_create(struct ompi_communicator_t * comm_old, int n, const int nodes[], const int degrees[], const int targets[], const int weights[], struct ompi_info_t * info, int reorder, struct ompi_communicator_t * (* newcomm));
int MPI_Dist_graph_create_adjacent(struct ompi_communicator_t * comm_old, int indegree, const int sources[], const int sourceweights[], int outdegree, const int destinations[], const int destweights[], struct ompi_info_t * info, int reorder, struct ompi_communicator_t * (* comm_dist_graph));
int MPI_Dist_graph_neighbors(struct ompi_communicator_t * comm, int maxindegree, int sources[], int sourceweights[], int maxoutdegree, int destinations[], int destweights[]);
int MPI_Dist_graph_neighbors_count(struct ompi_communicator_t * comm, int * inneighbors, int * outneighbors, int * weighted);
int MPI_Comm_get_errhandler(struct ompi_communicator_t * comm, struct ompi_errhandler_t * (* erhandler));
int MPI_Comm_get_info(struct ompi_communicator_t * comm, struct ompi_info_t * (* info_used));
int MPI_Comm_get_name(struct ompi_communicator_t * comm, char * comm_name, int * resultlen);
int MPI_Comm_get_parent(struct ompi_communicator_t * (* parent));
int MPI_Comm_group(struct ompi_communicator_t * comm, struct ompi_group_t * (* group));
int MPI_Comm_join(int fd, struct ompi_communicator_t * (* intercomm));
int MPI_Comm_rank(struct ompi_communicator_t * comm, int * rank);
int MPI_Comm_remote_group(struct ompi_communicator_t * comm, struct ompi_group_t * (* group));
int MPI_Comm_remote_size(struct ompi_communicator_t * comm, int * size);
int MPI_Comm_set_attr(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val);
int MPI_Comm_set_errhandler(struct ompi_communicator_t * comm, struct ompi_errhandler_t * errhandler);
int MPI_Comm_set_info(struct ompi_communicator_t * comm, struct ompi_info_t * info);
int MPI_Comm_set_name(struct ompi_communicator_t * comm, const char * comm_name);
int MPI_Comm_size(struct ompi_communicator_t * comm, int * size);
int MPI_Comm_spawn(const char * command, char * argv[], int maxprocs, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* intercomm), int array_of_errcodes[]);
int MPI_Comm_spawn_multiple(int count, char * array_of_commands[], char ** array_of_argv[], const int array_of_maxprocs[], const struct ompi_info_t * (array_of_info[]), int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* intercomm), int array_of_errcodes[]);
int MPI_Comm_split(struct ompi_communicator_t * comm, int color, int key, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_split_type(struct ompi_communicator_t * comm, int split_type, int key, struct ompi_info_t * info, struct ompi_communicator_t * (* newcomm));
int MPI_Comm_test_inter(struct ompi_communicator_t * comm, int * flag);
int MPI_Compare_and_swap(const void * origin_addr, const void * compare_addr, void * result_addr, struct ompi_datatype_t * datatype, int target_rank, long int target_disp, struct ompi_win_t * win);
int MPI_Dims_create(int nnodes, int ndims, int dims[]);
int MPI_Errhandler_c2f(struct ompi_errhandler_t * errhandler);
int MPI_Errhandler_create(void (* function)(struct ompi_communicator_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
struct ompi_errhandler_t * MPI_Errhandler_f2c(int errhandler);
int MPI_Errhandler_free(struct ompi_errhandler_t * (* errhandler));
int MPI_Errhandler_get(struct ompi_communicator_t * comm, struct ompi_errhandler_t * (* errhandler));
int MPI_Errhandler_set(struct ompi_communicator_t * comm, struct ompi_errhandler_t * errhandler);
int MPI_Error_class(int errorcode, int * errorclass);
int MPI_Error_string(int errorcode, char * string, int * resultlen);
int MPI_Exscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int MPI_Fetch_and_op(const void * origin_addr, void * result_addr, struct ompi_datatype_t * datatype, int target_rank, long int target_disp, struct ompi_op_t * op, struct ompi_win_t * win);
int MPI_Iexscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_File_c2f(struct ompi_file_t * file);
struct ompi_file_t * MPI_File_f2c(int file);
int MPI_File_call_errhandler(struct ompi_file_t * fh, int errorcode);
int MPI_File_create_errhandler(void (* function)(struct ompi_file_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int MPI_File_set_errhandler(struct ompi_file_t * file, struct ompi_errhandler_t * errhandler);
int MPI_File_get_errhandler(struct ompi_file_t * file, struct ompi_errhandler_t * (* errhandler));
int MPI_File_open(struct ompi_communicator_t * comm, const char * filename, int amode, struct ompi_info_t * info, struct ompi_file_t * (* fh));
int MPI_File_close(struct ompi_file_t * (* fh));
int MPI_File_delete(const char * filename, struct ompi_info_t * info);
int MPI_File_set_size(struct ompi_file_t * fh, long long size);
int MPI_File_preallocate(struct ompi_file_t * fh, long long size);
int MPI_File_get_size(struct ompi_file_t * fh, long long (* size));
int MPI_File_get_group(struct ompi_file_t * fh, struct ompi_group_t * (* group));
int MPI_File_get_amode(struct ompi_file_t * fh, int * amode);
int MPI_File_set_info(struct ompi_file_t * fh, struct ompi_info_t * info);
int MPI_File_get_info(struct ompi_file_t * fh, struct ompi_info_t * (* info_used));
int MPI_File_set_view(struct ompi_file_t * fh, long long disp, struct ompi_datatype_t * etype, struct ompi_datatype_t * filetype, const char * datarep, struct ompi_info_t * info);
int MPI_File_get_view(struct ompi_file_t * fh, long long (* disp), struct ompi_datatype_t * (* etype), struct ompi_datatype_t * (* filetype), char * datarep);
int MPI_File_read_at(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_read_at_all(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write_at(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write_at_all(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_iread_at(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iwrite_at(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iread_at_all(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iwrite_at_all(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_read(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_read_all(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write_all(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_iread(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iwrite(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iread_all(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iwrite_all(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_seek(struct ompi_file_t * fh, long long offset, int whence);
int MPI_File_get_position(struct ompi_file_t * fh, long long (* offset));
int MPI_File_get_byte_offset(struct ompi_file_t * fh, long long offset, long long (* disp));
int MPI_File_read_shared(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write_shared(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_iread_shared(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_iwrite_shared(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int MPI_File_read_ordered(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_write_ordered(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int MPI_File_seek_shared(struct ompi_file_t * fh, long long offset, int whence);
int MPI_File_get_position_shared(struct ompi_file_t * fh, long long (* offset));
int MPI_File_read_at_all_begin(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_read_at_all_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int MPI_File_write_at_all_begin(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_write_at_all_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int MPI_File_read_all_begin(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_read_all_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int MPI_File_write_all_begin(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_write_all_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int MPI_File_read_ordered_begin(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_read_ordered_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int MPI_File_write_ordered_begin(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype);
int MPI_File_write_ordered_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int MPI_File_get_type_extent(struct ompi_file_t * fh, struct ompi_datatype_t * datatype, long int (* extent));
int MPI_File_set_atomicity(struct ompi_file_t * fh, int flag);
int MPI_File_get_atomicity(struct ompi_file_t * fh, int * flag);
int MPI_File_sync(struct ompi_file_t * fh);
int MPI_Finalize(void);
int MPI_Finalized(int * flag);
int MPI_Free_mem(void * base);
int MPI_Gather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int MPI_Igather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Gatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int MPI_Igatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Get_address(const void * location, long int (* address));
int MPI_Get_count(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int * count);
int MPI_Get_elements(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int * count);
int MPI_Get_elements_x(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, long long (* count));
int MPI_Get(void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win);
int MPI_Get_accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, void * result_addr, int result_count, struct ompi_datatype_t * result_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win);
int MPI_Get_library_version(char * version, int * resultlen);
int MPI_Get_processor_name(char * name, int * resultlen);
int MPI_Get_version(int * version, int * subversion);
int MPI_Graph_create(struct ompi_communicator_t * comm_old, int nnodes, const int index[], const int edges[], int reorder, struct ompi_communicator_t * (* comm_graph));
int MPI_Graph_get(struct ompi_communicator_t * comm, int maxindex, int maxedges, int index[], int edges[]);
int MPI_Graph_map(struct ompi_communicator_t * comm, int nnodes, const int index[], const int edges[], int * newrank);
int MPI_Graph_neighbors_count(struct ompi_communicator_t * comm, int rank, int * nneighbors);
int MPI_Graph_neighbors(struct ompi_communicator_t * comm, int rank, int maxneighbors, int neighbors[]);
int MPI_Graphdims_get(struct ompi_communicator_t * comm, int * nnodes, int * nedges);
int MPI_Grequest_complete(struct ompi_request_t * request);
int MPI_Grequest_start(int (* query_fn)(void *, struct ompi_status_public_t (*)), int (* free_fn)(void *), int (* cancel_fn)(void *, int), void * extra_state, struct ompi_request_t * (* request));
int MPI_Group_c2f(struct ompi_group_t * group);
int MPI_Group_compare(struct ompi_group_t * group1, struct ompi_group_t * group2, int * result);
int MPI_Group_difference(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int MPI_Group_excl(struct ompi_group_t * group, int n, const int ranks[], struct ompi_group_t * (* newgroup));
struct ompi_group_t * MPI_Group_f2c(int group);
int MPI_Group_free(struct ompi_group_t * (* group));
int MPI_Group_incl(struct ompi_group_t * group, int n, const int ranks[], struct ompi_group_t * (* newgroup));
int MPI_Group_intersection(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int MPI_Group_range_excl(struct ompi_group_t * group, int n, int ranges[][ 3], struct ompi_group_t * (* newgroup));
int MPI_Group_range_incl(struct ompi_group_t * group, int n, int ranges[][ 3], struct ompi_group_t * (* newgroup));
int MPI_Group_rank(struct ompi_group_t * group, int * rank);
int MPI_Group_size(struct ompi_group_t * group, int * size);
int MPI_Group_translate_ranks(struct ompi_group_t * group1, int n, const int ranks1[], struct ompi_group_t * group2, int ranks2[]);
int MPI_Group_union(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int MPI_Ibsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Improbe(int source, int tag, struct ompi_communicator_t * comm, int * flag, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int MPI_Imrecv(void * buf, int count, struct ompi_datatype_t * type, struct ompi_message_t * (* message), struct ompi_request_t * (* request));
int MPI_Info_c2f(struct ompi_info_t * info);
int MPI_Info_create(struct ompi_info_t * (* info));
int MPI_Info_delete(struct ompi_info_t * info, const char * key);
int MPI_Info_dup(struct ompi_info_t * info, struct ompi_info_t * (* newinfo));
struct ompi_info_t * MPI_Info_f2c(int info);
int MPI_Info_free(struct ompi_info_t * (* info));
int MPI_Info_get(struct ompi_info_t * info, const char * key, int valuelen, char * value, int * flag);
int MPI_Info_get_nkeys(struct ompi_info_t * info, int * nkeys);
int MPI_Info_get_nthkey(struct ompi_info_t * info, int n, char * key);
int MPI_Info_get_valuelen(struct ompi_info_t * info, const char * key, int * valuelen, int * flag);
int MPI_Info_set(struct ompi_info_t * info, const char * key, const char * value);
int MPI_Init(int * argc, char *** argv);
int MPI_Initialized(int * flag);
int MPI_Init_thread(int * argc, char *** argv, int required, int * provided);
int MPI_Intercomm_create(struct ompi_communicator_t * local_comm, int local_leader, struct ompi_communicator_t * bridge_comm, int remote_leader, int tag, struct ompi_communicator_t * (* newintercomm));
int MPI_Intercomm_merge(struct ompi_communicator_t * intercomm, int high, struct ompi_communicator_t * (* newintercomm));
int MPI_Iprobe(int source, int tag, struct ompi_communicator_t * comm, int * flag, struct ompi_status_public_t (* status));
int MPI_Irecv(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Irsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Isend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Issend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Is_thread_main(int * flag);
int MPI_Keyval_create(int (* copy_fn)(struct ompi_communicator_t *, int, void *, void *, void *, int *), int (* delete_fn)(struct ompi_communicator_t *, int, void *, void *), int * keyval, void * extra_state);
int MPI_Keyval_free(int * keyval);
int MPI_Lookup_name(const char * service_name, struct ompi_info_t * info, char * port_name);
int MPI_Message_c2f(struct ompi_message_t * message);
struct ompi_message_t * MPI_Message_f2c(int message);
int MPI_Mprobe(int source, int tag, struct ompi_communicator_t * comm, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int MPI_Mrecv(void * buf, int count, struct ompi_datatype_t * type, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int MPI_Neighbor_allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ineighbor_allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Neighbor_allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ineighbor_allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Neighbor_alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ineighbor_alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Neighbor_alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int MPI_Ineighbor_alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Neighbor_alltoallw(const void * sendbuf, const int sendcounts[], const long int (sdispls[]), const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const long int (rdispls[]), const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm);
int MPI_Ineighbor_alltoallw(const void * sendbuf, const int sendcounts[], const long int (sdispls[]), const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const long int (rdispls[]), const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Op_c2f(struct ompi_op_t * op);
int MPI_Op_commutative(struct ompi_op_t * op, int * commute);
int MPI_Op_create(void (* function)(void *, void *, int *, struct ompi_datatype_t * (*)), int commute, struct ompi_op_t * (* op));
int MPI_Open_port(struct ompi_info_t * info, char * port_name);
struct ompi_op_t * MPI_Op_f2c(int op);
int MPI_Op_free(struct ompi_op_t * (* op));
int MPI_Pack_external(const char datarep[], const void * inbuf, int incount, struct ompi_datatype_t * datatype, void * outbuf, long int outsize, long int (* position));
int MPI_Pack_external_size(const char datarep[], int incount, struct ompi_datatype_t * datatype, long int (* size));
int MPI_Pack(const void * inbuf, int incount, struct ompi_datatype_t * datatype, void * outbuf, int outsize, int * position, struct ompi_communicator_t * comm);
int MPI_Pack_size(int incount, struct ompi_datatype_t * datatype, struct ompi_communicator_t * comm, int * size);
int MPI_Pcontrol(const int level, ...);
int MPI_Probe(int source, int tag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int MPI_Publish_name(const char * service_name, struct ompi_info_t * info, const char * port_name);
int MPI_Put(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win);
int MPI_Query_thread(int * provided);
int MPI_Raccumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win, struct ompi_request_t * (* request));
int MPI_Recv_init(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Recv(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int MPI_Reduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, int root, struct ompi_communicator_t * comm);
int MPI_Ireduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Reduce_local(const void * inbuf, void * inoutbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op);
int MPI_Reduce_scatter(const void * sendbuf, void * recvbuf, const int recvcounts[], struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int MPI_Ireduce_scatter(const void * sendbuf, void * recvbuf, const int recvcounts[], struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Reduce_scatter_block(const void * sendbuf, void * recvbuf, int recvcount, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int MPI_Ireduce_scatter_block(const void * sendbuf, void * recvbuf, int recvcount, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Register_datarep(const char * datarep, int (* read_conversion_fn)(void *, struct ompi_datatype_t *, int, void *, long long , void *), int (* write_conversion_fn)(void *, struct ompi_datatype_t *, int, void *, long long , void *), int (* dtype_file_extent_fn)(struct ompi_datatype_t *, long int (*), void *), void * extra_state);
int MPI_Request_c2f(struct ompi_request_t * request);
struct ompi_request_t * MPI_Request_f2c(int request);
int MPI_Request_free(struct ompi_request_t * (* request));
int MPI_Request_get_status(struct ompi_request_t * request, int * flag, struct ompi_status_public_t (* status));
int MPI_Rget(void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win, struct ompi_request_t * (* request));
int MPI_Rget_accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, void * result_addr, int result_count, struct ompi_datatype_t * result_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win, struct ompi_request_t * (* request));
int MPI_Rput(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_cout, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win, struct ompi_request_t * (* request));
int MPI_Rsend(const void * ibuf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int MPI_Rsend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Scan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int MPI_Iscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Scatter(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int MPI_Iscatter(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Scatterv(const void * sendbuf, const int sendcounts[], const int displs[], struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int MPI_Iscatterv(const void * sendbuf, const int sendcounts[], const int displs[], struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Send_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Send(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int MPI_Sendrecv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, int dest, int sendtag, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int source, int recvtag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int MPI_Sendrecv_replace(void * buf, int count, struct ompi_datatype_t * datatype, int dest, int sendtag, int source, int recvtag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int MPI_Ssend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int MPI_Ssend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int MPI_Start(struct ompi_request_t * (* request));
int MPI_Startall(int count, struct ompi_request_t * (array_of_requests[]));
int MPI_Status_c2f(const struct ompi_status_public_t (* c_status), int * f_status);
int MPI_Status_f2c(const int * f_status, struct ompi_status_public_t (* c_status));
int MPI_Status_set_cancelled(struct ompi_status_public_t (* status), int flag);
int MPI_Status_set_elements(struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int count);
int MPI_Status_set_elements_x(struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, long long count);
int MPI_Testall(int count, struct ompi_request_t * (array_of_requests[]), int * flag, struct ompi_status_public_t (array_of_statuses[]));
int MPI_Testany(int count, struct ompi_request_t * (array_of_requests[]), int * index, int * flag, struct ompi_status_public_t (* status));
int MPI_Test(struct ompi_request_t * (* request), int * flag, struct ompi_status_public_t (* status));
int MPI_Test_cancelled(const struct ompi_status_public_t (* status), int * flag);
int MPI_Testsome(int incount, struct ompi_request_t * (array_of_requests[]), int * outcount, int array_of_indices[], struct ompi_status_public_t (array_of_statuses[]));
int MPI_Topo_test(struct ompi_communicator_t * comm, int * status);
int MPI_Type_c2f(struct ompi_datatype_t * datatype);
int MPI_Type_commit(struct ompi_datatype_t * (* type));
int MPI_Type_contiguous(int count, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_darray(int size, int rank, int ndims, const int gsize_array[], const int distrib_array[], const int darg_array[], const int psize_array[], int order, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_f90_complex(int p, int r, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_f90_integer(int r, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_f90_real(int p, int r, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_hindexed_block(int count, int blocklength, const long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_hindexed(int count, const int array_of_blocklengths[], const long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_hvector(int count, int blocklength, long int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_keyval(int (* type_copy_attr_fn)(struct ompi_datatype_t *, int, void *, void *, void *, int *), int (* type_delete_attr_fn)(struct ompi_datatype_t *, int, void *, void *), int * type_keyval, void * extra_state);
int MPI_Type_create_indexed_block(int count, int blocklength, const int array_of_displacements[], struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_struct(int count, const int array_of_block_lengths[], const long int (array_of_displacements[]), const struct ompi_datatype_t * (array_of_types[]), struct ompi_datatype_t * (* newtype));
int MPI_Type_create_subarray(int ndims, const int size_array[], const int subsize_array[], const int start_array[], int order, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_create_resized(struct ompi_datatype_t * oldtype, long int lb, long int extent, struct ompi_datatype_t * (* newtype));
int MPI_Type_delete_attr(struct ompi_datatype_t * type, int type_keyval);
int MPI_Type_dup(struct ompi_datatype_t * type, struct ompi_datatype_t * (* newtype));
int MPI_Type_extent(struct ompi_datatype_t * type, long int (* extent));
int MPI_Type_free(struct ompi_datatype_t * (* type));
int MPI_Type_free_keyval(int * type_keyval);
struct ompi_datatype_t * MPI_Type_f2c(int datatype);
int MPI_Type_get_attr(struct ompi_datatype_t * type, int type_keyval, void * attribute_val, int * flag);
int MPI_Type_get_contents(struct ompi_datatype_t * mtype, int max_integers, int max_addresses, int max_datatypes, int array_of_integers[], long int (array_of_addresses[]), struct ompi_datatype_t * (array_of_datatypes[]));
int MPI_Type_get_envelope(struct ompi_datatype_t * type, int * num_integers, int * num_addresses, int * num_datatypes, int * combiner);
int MPI_Type_get_extent(struct ompi_datatype_t * type, long int (* lb), long int (* extent));
int MPI_Type_get_extent_x(struct ompi_datatype_t * type, long long (* lb), long long (* extent));
int MPI_Type_get_name(struct ompi_datatype_t * type, char * type_name, int * resultlen);
int MPI_Type_get_true_extent(struct ompi_datatype_t * datatype, long int (* true_lb), long int (* true_extent));
int MPI_Type_get_true_extent_x(struct ompi_datatype_t * datatype, long long (* true_lb), long long (* true_extent));
int MPI_Type_hindexed(int count, int array_of_blocklengths[], long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_hvector(int count, int blocklength, long int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_indexed(int count, const int array_of_blocklengths[], const int array_of_displacements[], struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Type_lb(struct ompi_datatype_t * type, long int (* lb));
int MPI_Type_match_size(int typeclass, int size, struct ompi_datatype_t * (* type));
int MPI_Type_set_attr(struct ompi_datatype_t * type, int type_keyval, void * attr_val);
int MPI_Type_set_name(struct ompi_datatype_t * type, const char * type_name);
int MPI_Type_size(struct ompi_datatype_t * type, int * size);
int MPI_Type_size_x(struct ompi_datatype_t * type, long long (* size));
int MPI_Type_struct(int count, int array_of_blocklengths[], long int (array_of_displacements[]), struct ompi_datatype_t * (array_of_types[]), struct ompi_datatype_t * (* newtype));
int MPI_Type_ub(struct ompi_datatype_t * mtype, long int (* ub));
int MPI_Type_vector(int count, int blocklength, int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int MPI_Unpack(const void * inbuf, int insize, int * position, void * outbuf, int outcount, struct ompi_datatype_t * datatype, struct ompi_communicator_t * comm);
int MPI_Unpublish_name(const char * service_name, struct ompi_info_t * info, const char * port_name);
int MPI_Unpack_external(const char datarep[], const void * inbuf, long int insize, long int (* position), void * outbuf, int outcount, struct ompi_datatype_t * datatype);
int MPI_Waitall(int count, struct ompi_request_t * (array_of_requests[]), struct ompi_status_public_t (* array_of_statuses));
int MPI_Waitany(int count, struct ompi_request_t * (array_of_requests[]), int * index, struct ompi_status_public_t (* status));
int MPI_Wait(struct ompi_request_t * (* request), struct ompi_status_public_t (* status));
int MPI_Waitsome(int incount, struct ompi_request_t * (array_of_requests[]), int * outcount, int array_of_indices[], struct ompi_status_public_t (array_of_statuses[]));
int MPI_Win_allocate(long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, void * baseptr, struct ompi_win_t * (* win));
int MPI_Win_allocate_shared(long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, void * baseptr, struct ompi_win_t * (* win));
int MPI_Win_attach(struct ompi_win_t * win, void * base, long int size);
int MPI_Win_c2f(struct ompi_win_t * win);
int MPI_Win_call_errhandler(struct ompi_win_t * win, int errorcode);
int MPI_Win_complete(struct ompi_win_t * win);
int MPI_Win_create(void * base, long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, struct ompi_win_t * (* win));
int MPI_Win_create_dynamic(struct ompi_info_t * info, struct ompi_communicator_t * comm, struct ompi_win_t * (* win));
int MPI_Win_create_errhandler(void (* function)(struct ompi_win_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int MPI_Win_create_keyval(int (* win_copy_attr_fn)(struct ompi_win_t *, int, void *, void *, void *, int *), int (* win_delete_attr_fn)(struct ompi_win_t *, int, void *, void *), int * win_keyval, void * extra_state);
int MPI_Win_delete_attr(struct ompi_win_t * win, int win_keyval);
int MPI_Win_detach(struct ompi_win_t * win, const void * base);
struct ompi_win_t * MPI_Win_f2c(int win);
int MPI_Win_fence(int assert, struct ompi_win_t * win);
int MPI_Win_flush(int rank, struct ompi_win_t * win);
int MPI_Win_flush_all(struct ompi_win_t * win);
int MPI_Win_flush_local(int rank, struct ompi_win_t * win);
int MPI_Win_flush_local_all(struct ompi_win_t * win);
int MPI_Win_free(struct ompi_win_t * (* win));
int MPI_Win_free_keyval(int * win_keyval);
int MPI_Win_get_attr(struct ompi_win_t * win, int win_keyval, void * attribute_val, int * flag);
int MPI_Win_get_errhandler(struct ompi_win_t * win, struct ompi_errhandler_t * (* errhandler));
int MPI_Win_get_group(struct ompi_win_t * win, struct ompi_group_t * (* group));
int MPI_Win_get_info(struct ompi_win_t * win, struct ompi_info_t * (* info_used));
int MPI_Win_get_name(struct ompi_win_t * win, char * win_name, int * resultlen);
int MPI_Win_lock(int lock_type, int rank, int assert, struct ompi_win_t * win);
int MPI_Win_lock_all(int assert, struct ompi_win_t * win);
int MPI_Win_post(struct ompi_group_t * group, int assert, struct ompi_win_t * win);
int MPI_Win_set_attr(struct ompi_win_t * win, int win_keyval, void * attribute_val);
int MPI_Win_set_errhandler(struct ompi_win_t * win, struct ompi_errhandler_t * errhandler);
int MPI_Win_set_info(struct ompi_win_t * win, struct ompi_info_t * info);
int MPI_Win_set_name(struct ompi_win_t * win, const char * win_name);
int MPI_Win_shared_query(struct ompi_win_t * win, int rank, long int (* size), int * disp_unit, void * baseptr);
int MPI_Win_start(struct ompi_group_t * group, int assert, struct ompi_win_t * win);
int MPI_Win_sync(struct ompi_win_t * win);
int MPI_Win_test(struct ompi_win_t * win, int * flag);
int MPI_Win_unlock(int rank, struct ompi_win_t * win);
int MPI_Win_unlock_all(struct ompi_win_t * win);
int MPI_Win_wait(struct ompi_win_t * win);
double MPI_Wtick(void);
double MPI_Wtime(void);
int PMPI_Abort(struct ompi_communicator_t * comm, int errorcode);
int PMPI_Accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win);
int PMPI_Add_error_class(int * errorclass);
int PMPI_Add_error_code(int errorclass, int * errorcode);
int PMPI_Add_error_string(int errorcode, const char * string);
int PMPI_Address(void * location, long int (* address));
int PMPI_Allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Iallgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Iallgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Alloc_mem(long int size, struct ompi_info_t * info, void * baseptr);
int PMPI_Allreduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int PMPI_Iallreduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ialltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ialltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Alltoallw(const void * sendbuf, const int sendcounts[], const int sdispls[], const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const int rdispls[], const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm);
int PMPI_Ialltoallw(const void * sendbuf, const int sendcounts[], const int sdispls[], const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const int rdispls[], const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Attr_delete(struct ompi_communicator_t * comm, int keyval);
int PMPI_Attr_get(struct ompi_communicator_t * comm, int keyval, void * attribute_val, int * flag);
int PMPI_Dist_graph_create(struct ompi_communicator_t * comm_old, int n, const int nodes[], const int degrees[], const int targets[], const int weights[], struct ompi_info_t * info, int reorder, struct ompi_communicator_t * (* newcomm));
int PMPI_Dist_graph_create_adjacent(struct ompi_communicator_t * comm_old, int indegree, const int sources[], const int sourceweights[], int outdegree, const int destinations[], const int destweights[], struct ompi_info_t * info, int reorder, struct ompi_communicator_t * (* comm_dist_graph));
int PMPI_Dist_graph_neighbors(struct ompi_communicator_t * comm, int maxindegree, int sources[], int sourceweights[], int maxoutdegree, int destinations[], int destweights[]);
int PMPI_Dist_graph_neighbors_count(struct ompi_communicator_t * comm, int * inneighbors, int * outneighbors, int * weighted);
int PMPI_Attr_put(struct ompi_communicator_t * comm, int keyval, void * attribute_val);
int PMPI_Barrier(struct ompi_communicator_t * comm);
int PMPI_Ibarrier(struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Bcast(void * buffer, int count, struct ompi_datatype_t * datatype, int root, struct ompi_communicator_t * comm);
int PMPI_Ibcast(void * buffer, int count, struct ompi_datatype_t * datatype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Bsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int PMPI_Bsend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Buffer_attach(void * buffer, int size);
int PMPI_Buffer_detach(void * buffer, int * size);
int PMPI_Cancel(struct ompi_request_t * (* request));
int PMPI_Cart_coords(struct ompi_communicator_t * comm, int rank, int maxdims, int coords[]);
int PMPI_Cart_create(struct ompi_communicator_t * old_comm, int ndims, const int dims[], const int periods[], int reorder, struct ompi_communicator_t * (* comm_cart));
int PMPI_Cart_get(struct ompi_communicator_t * comm, int maxdims, int dims[], int periods[], int coords[]);
int PMPI_Cart_map(struct ompi_communicator_t * comm, int ndims, const int dims[], const int periods[], int * newrank);
int PMPI_Cart_rank(struct ompi_communicator_t * comm, const int coords[], int * rank);
int PMPI_Cart_shift(struct ompi_communicator_t * comm, int direction, int disp, int * rank_source, int * rank_dest);
int PMPI_Cart_sub(struct ompi_communicator_t * comm, const int remain_dims[], struct ompi_communicator_t * (* new_comm));
int PMPI_Cartdim_get(struct ompi_communicator_t * comm, int * ndims);
int PMPI_Close_port(const char * port_name);
int PMPI_Comm_accept(const char * port_name, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_c2f(struct ompi_communicator_t * comm);
int PMPI_Comm_call_errhandler(struct ompi_communicator_t * comm, int errorcode);
int PMPI_Comm_compare(struct ompi_communicator_t * comm1, struct ompi_communicator_t * comm2, int * result);
int PMPI_Comm_connect(const char * port_name, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_create_errhandler(void (* function)(struct ompi_communicator_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int PMPI_Comm_create_keyval(int (* comm_copy_attr_fn)(struct ompi_communicator_t *, int, void *, void *, void *, int *), int (* comm_delete_attr_fn)(struct ompi_communicator_t *, int, void *, void *), int * comm_keyval, void * extra_state);
int PMPI_Comm_create_group(struct ompi_communicator_t * comm, struct ompi_group_t * group, int tag, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_create(struct ompi_communicator_t * comm, struct ompi_group_t * group, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_delete_attr(struct ompi_communicator_t * comm, int comm_keyval);
int PMPI_Comm_disconnect(struct ompi_communicator_t * (* comm));
int PMPI_Comm_dup(struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_idup(struct ompi_communicator_t * comm, struct ompi_communicator_t * (* newcomm), struct ompi_request_t * (* request));
int PMPI_Comm_dup_with_info(struct ompi_communicator_t * comm, struct ompi_info_t * info, struct ompi_communicator_t * (* newcomm));
struct ompi_communicator_t * PMPI_Comm_f2c(int comm);
int PMPI_Comm_free_keyval(int * comm_keyval);
int PMPI_Comm_free(struct ompi_communicator_t * (* comm));
int PMPI_Comm_get_attr(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val, int * flag);
int PMPI_Comm_get_errhandler(struct ompi_communicator_t * comm, struct ompi_errhandler_t * (* erhandler));
int PMPI_Comm_get_info(struct ompi_communicator_t * comm, struct ompi_info_t * (* info_used));
int PMPI_Comm_get_name(struct ompi_communicator_t * comm, char * comm_name, int * resultlen);
int PMPI_Comm_get_parent(struct ompi_communicator_t * (* parent));
int PMPI_Comm_group(struct ompi_communicator_t * comm, struct ompi_group_t * (* group));
int PMPI_Comm_join(int fd, struct ompi_communicator_t * (* intercomm));
int PMPI_Comm_rank(struct ompi_communicator_t * comm, int * rank);
int PMPI_Comm_remote_group(struct ompi_communicator_t * comm, struct ompi_group_t * (* group));
int PMPI_Comm_remote_size(struct ompi_communicator_t * comm, int * size);
int PMPI_Comm_set_attr(struct ompi_communicator_t * comm, int comm_keyval, void * attribute_val);
int PMPI_Comm_set_errhandler(struct ompi_communicator_t * comm, struct ompi_errhandler_t * errhandler);
int PMPI_Comm_set_info(struct ompi_communicator_t * comm, struct ompi_info_t * info);
int PMPI_Comm_set_name(struct ompi_communicator_t * comm, const char * comm_name);
int PMPI_Comm_size(struct ompi_communicator_t * comm, int * size);
int PMPI_Comm_spawn(const char * command, char * argv[], int maxprocs, struct ompi_info_t * info, int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* intercomm), int array_of_errcodes[]);
int PMPI_Comm_spawn_multiple(int count, char * array_of_commands[], char ** array_of_argv[], const int array_of_maxprocs[], const struct ompi_info_t * (array_of_info[]), int root, struct ompi_communicator_t * comm, struct ompi_communicator_t * (* intercomm), int array_of_errcodes[]);
int PMPI_Comm_split(struct ompi_communicator_t * comm, int color, int key, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_split_type(struct ompi_communicator_t * comm, int split_type, int key, struct ompi_info_t * info, struct ompi_communicator_t * (* newcomm));
int PMPI_Comm_test_inter(struct ompi_communicator_t * comm, int * flag);
int PMPI_Compare_and_swap(const void * origin_addr, const void * compare_addr, void * result_addr, struct ompi_datatype_t * datatype, int target_rank, long int target_disp, struct ompi_win_t * win);
int PMPI_Dims_create(int nnodes, int ndims, int dims[]);
int PMPI_Errhandler_c2f(struct ompi_errhandler_t * errhandler);
int PMPI_Errhandler_create(void (* function)(struct ompi_communicator_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
struct ompi_errhandler_t * PMPI_Errhandler_f2c(int errhandler);
int PMPI_Errhandler_free(struct ompi_errhandler_t * (* errhandler));
int PMPI_Errhandler_get(struct ompi_communicator_t * comm, struct ompi_errhandler_t * (* errhandler));
int PMPI_Errhandler_set(struct ompi_communicator_t * comm, struct ompi_errhandler_t * errhandler);
int PMPI_Error_class(int errorcode, int * errorclass);
int PMPI_Error_string(int errorcode, char * string, int * resultlen);
int PMPI_Exscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int PMPI_Fetch_and_op(const void * origin_addr, void * result_addr, struct ompi_datatype_t * datatype, int target_rank, long int target_disp, struct ompi_op_t * op, struct ompi_win_t * win);
int PMPI_Iexscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_File_c2f(struct ompi_file_t * file);
struct ompi_file_t * PMPI_File_f2c(int file);
int PMPI_File_call_errhandler(struct ompi_file_t * fh, int errorcode);
int PMPI_File_create_errhandler(void (* function)(struct ompi_file_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int PMPI_File_set_errhandler(struct ompi_file_t * file, struct ompi_errhandler_t * errhandler);
int PMPI_File_get_errhandler(struct ompi_file_t * file, struct ompi_errhandler_t * (* errhandler));
int PMPI_File_open(struct ompi_communicator_t * comm, const char * filename, int amode, struct ompi_info_t * info, struct ompi_file_t * (* fh));
int PMPI_File_close(struct ompi_file_t * (* fh));
int PMPI_File_delete(const char * filename, struct ompi_info_t * info);
int PMPI_File_set_size(struct ompi_file_t * fh, long long size);
int PMPI_File_preallocate(struct ompi_file_t * fh, long long size);
int PMPI_File_get_size(struct ompi_file_t * fh, long long (* size));
int PMPI_File_get_group(struct ompi_file_t * fh, struct ompi_group_t * (* group));
int PMPI_File_get_amode(struct ompi_file_t * fh, int * amode);
int PMPI_File_set_info(struct ompi_file_t * fh, struct ompi_info_t * info);
int PMPI_File_get_info(struct ompi_file_t * fh, struct ompi_info_t * (* info_used));
int PMPI_File_set_view(struct ompi_file_t * fh, long long disp, struct ompi_datatype_t * etype, struct ompi_datatype_t * filetype, const char * datarep, struct ompi_info_t * info);
int PMPI_File_get_view(struct ompi_file_t * fh, long long (* disp), struct ompi_datatype_t * (* etype), struct ompi_datatype_t * (* filetype), char * datarep);
int PMPI_File_read_at(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_read_at_all(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write_at(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write_at_all(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_iread_at(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iwrite_at(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iread_at_all(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iwrite_at_all(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_read(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_read_all(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write_all(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_iread(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iwrite(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iread_all(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iwrite_all(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_seek(struct ompi_file_t * fh, long long offset, int whence);
int PMPI_File_get_position(struct ompi_file_t * fh, long long (* offset));
int PMPI_File_get_byte_offset(struct ompi_file_t * fh, long long offset, long long (* disp));
int PMPI_File_read_shared(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write_shared(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_iread_shared(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_iwrite_shared(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_request_t * (* request));
int PMPI_File_read_ordered(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_write_ordered(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype, struct ompi_status_public_t (* status));
int PMPI_File_seek_shared(struct ompi_file_t * fh, long long offset, int whence);
int PMPI_File_get_position_shared(struct ompi_file_t * fh, long long (* offset));
int PMPI_File_read_at_all_begin(struct ompi_file_t * fh, long long offset, void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_read_at_all_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int PMPI_File_write_at_all_begin(struct ompi_file_t * fh, long long offset, const void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_write_at_all_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int PMPI_File_read_all_begin(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_read_all_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int PMPI_File_write_all_begin(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_write_all_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int PMPI_File_read_ordered_begin(struct ompi_file_t * fh, void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_read_ordered_end(struct ompi_file_t * fh, void * buf, struct ompi_status_public_t (* status));
int PMPI_File_write_ordered_begin(struct ompi_file_t * fh, const void * buf, int count, struct ompi_datatype_t * datatype);
int PMPI_File_write_ordered_end(struct ompi_file_t * fh, const void * buf, struct ompi_status_public_t (* status));
int PMPI_File_get_type_extent(struct ompi_file_t * fh, struct ompi_datatype_t * datatype, long int (* extent));
int PMPI_File_set_atomicity(struct ompi_file_t * fh, int flag);
int PMPI_File_get_atomicity(struct ompi_file_t * fh, int * flag);
int PMPI_File_sync(struct ompi_file_t * fh);
int PMPI_Finalize(void);
int PMPI_Finalized(int * flag);
int PMPI_Free_mem(void * base);
int PMPI_Gather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int PMPI_Igather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Gatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int PMPI_Igatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Get_address(const void * location, long int (* address));
int PMPI_Get_count(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int * count);
int PMPI_Get_elements(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int * count);
int PMPI_Get_elements_x(const struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, long long (* count));
int PMPI_Get(void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win);
int PMPI_Get_accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, void * result_addr, int result_count, struct ompi_datatype_t * result_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win);
int PMPI_Get_library_version(char * version, int * resultlen);
int PMPI_Get_processor_name(char * name, int * resultlen);
int PMPI_Get_version(int * version, int * subversion);
int PMPI_Graph_create(struct ompi_communicator_t * comm_old, int nnodes, const int index[], const int edges[], int reorder, struct ompi_communicator_t * (* comm_graph));
int PMPI_Graph_get(struct ompi_communicator_t * comm, int maxindex, int maxedges, int index[], int edges[]);
int PMPI_Graph_map(struct ompi_communicator_t * comm, int nnodes, const int index[], const int edges[], int * newrank);
int PMPI_Graph_neighbors_count(struct ompi_communicator_t * comm, int rank, int * nneighbors);
int PMPI_Graph_neighbors(struct ompi_communicator_t * comm, int rank, int maxneighbors, int neighbors[]);
int PMPI_Graphdims_get(struct ompi_communicator_t * comm, int * nnodes, int * nedges);
int PMPI_Grequest_complete(struct ompi_request_t * request);
int PMPI_Grequest_start(int (* query_fn)(void *, struct ompi_status_public_t (*)), int (* free_fn)(void *), int (* cancel_fn)(void *, int), void * extra_state, struct ompi_request_t * (* request));
int PMPI_Group_c2f(struct ompi_group_t * group);
int PMPI_Group_compare(struct ompi_group_t * group1, struct ompi_group_t * group2, int * result);
int PMPI_Group_difference(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int PMPI_Group_excl(struct ompi_group_t * group, int n, const int ranks[], struct ompi_group_t * (* newgroup));
struct ompi_group_t * PMPI_Group_f2c(int group);
int PMPI_Group_free(struct ompi_group_t * (* group));
int PMPI_Group_incl(struct ompi_group_t * group, int n, const int ranks[], struct ompi_group_t * (* newgroup));
int PMPI_Group_intersection(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int PMPI_Group_range_excl(struct ompi_group_t * group, int n, int ranges[][ 3], struct ompi_group_t * (* newgroup));
int PMPI_Group_range_incl(struct ompi_group_t * group, int n, int ranges[][ 3], struct ompi_group_t * (* newgroup));
int PMPI_Group_rank(struct ompi_group_t * group, int * rank);
int PMPI_Group_size(struct ompi_group_t * group, int * size);
int PMPI_Group_translate_ranks(struct ompi_group_t * group1, int n, const int ranks1[], struct ompi_group_t * group2, int ranks2[]);
int PMPI_Group_union(struct ompi_group_t * group1, struct ompi_group_t * group2, struct ompi_group_t * (* newgroup));
int PMPI_Ibsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Improbe(int source, int tag, struct ompi_communicator_t * comm, int * flag, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int PMPI_Imrecv(void * buf, int count, struct ompi_datatype_t * type, struct ompi_message_t * (* message), struct ompi_request_t * (* request));
int PMPI_Info_c2f(struct ompi_info_t * info);
int PMPI_Info_create(struct ompi_info_t * (* info));
int PMPI_Info_delete(struct ompi_info_t * info, const char * key);
int PMPI_Info_dup(struct ompi_info_t * info, struct ompi_info_t * (* newinfo));
struct ompi_info_t * PMPI_Info_f2c(int info);
int PMPI_Info_free(struct ompi_info_t * (* info));
int PMPI_Info_get(struct ompi_info_t * info, const char * key, int valuelen, char * value, int * flag);
int PMPI_Info_get_nkeys(struct ompi_info_t * info, int * nkeys);
int PMPI_Info_get_nthkey(struct ompi_info_t * info, int n, char * key);
int PMPI_Info_get_valuelen(struct ompi_info_t * info, const char * key, int * valuelen, int * flag);
int PMPI_Info_set(struct ompi_info_t * info, const char * key, const char * value);
int PMPI_Init(int * argc, char *** argv);
int PMPI_Initialized(int * flag);
int PMPI_Init_thread(int * argc, char *** argv, int required, int * provided);
int PMPI_Intercomm_create(struct ompi_communicator_t * local_comm, int local_leader, struct ompi_communicator_t * bridge_comm, int remote_leader, int tag, struct ompi_communicator_t * (* newintercomm));
int PMPI_Intercomm_merge(struct ompi_communicator_t * intercomm, int high, struct ompi_communicator_t * (* newintercomm));
int PMPI_Iprobe(int source, int tag, struct ompi_communicator_t * comm, int * flag, struct ompi_status_public_t (* status));
int PMPI_Irecv(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Irsend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Isend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Issend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Is_thread_main(int * flag);
int PMPI_Keyval_create(int (* copy_fn)(struct ompi_communicator_t *, int, void *, void *, void *, int *), int (* delete_fn)(struct ompi_communicator_t *, int, void *, void *), int * keyval, void * extra_state);
int PMPI_Keyval_free(int * keyval);
int PMPI_Lookup_name(const char * service_name, struct ompi_info_t * info, char * port_name);
int PMPI_Message_c2f(struct ompi_message_t * message);
struct ompi_message_t * PMPI_Message_f2c(int message);
int PMPI_Mprobe(int source, int tag, struct ompi_communicator_t * comm, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int PMPI_Mrecv(void * buf, int count, struct ompi_datatype_t * type, struct ompi_message_t * (* message), struct ompi_status_public_t (* status));
int PMPI_Neighbor_allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ineighbor_allgather(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Neighbor_allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ineighbor_allgatherv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int displs[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Neighbor_alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ineighbor_alltoall(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Neighbor_alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm);
int PMPI_Ineighbor_alltoallv(const void * sendbuf, const int sendcounts[], const int sdispls[], struct ompi_datatype_t * sendtype, void * recvbuf, const int recvcounts[], const int rdispls[], struct ompi_datatype_t * recvtype, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Neighbor_alltoallw(const void * sendbuf, const int sendcounts[], const long int (sdispls[]), const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const long int (rdispls[]), const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm);
int PMPI_Ineighbor_alltoallw(const void * sendbuf, const int sendcounts[], const long int (sdispls[]), const struct ompi_datatype_t * (sendtypes[]), void * recvbuf, const int recvcounts[], const long int (rdispls[]), const struct ompi_datatype_t * (recvtypes[]), struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Op_c2f(struct ompi_op_t * op);
int PMPI_Op_commutative(struct ompi_op_t * op, int * commute);
int PMPI_Op_create(void (* function)(void *, void *, int *, struct ompi_datatype_t * (*)), int commute, struct ompi_op_t * (* op));
int PMPI_Open_port(struct ompi_info_t * info, char * port_name);
struct ompi_op_t * PMPI_Op_f2c(int op);
int PMPI_Op_free(struct ompi_op_t * (* op));
int PMPI_Pack_external(const char datarep[], const void * inbuf, int incount, struct ompi_datatype_t * datatype, void * outbuf, long int outsize, long int (* position));
int PMPI_Pack_external_size(const char datarep[], int incount, struct ompi_datatype_t * datatype, long int (* size));
int PMPI_Pack(const void * inbuf, int incount, struct ompi_datatype_t * datatype, void * outbuf, int outsize, int * position, struct ompi_communicator_t * comm);
int PMPI_Pack_size(int incount, struct ompi_datatype_t * datatype, struct ompi_communicator_t * comm, int * size);
int PMPI_Pcontrol(const int level, ...);
int PMPI_Probe(int source, int tag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int PMPI_Publish_name(const char * service_name, struct ompi_info_t * info, const char * port_name);
int PMPI_Put(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win);
int PMPI_Query_thread(int * provided);
int PMPI_Raccumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win, struct ompi_request_t * (* request));
int PMPI_Recv_init(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Recv(void * buf, int count, struct ompi_datatype_t * datatype, int source, int tag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int PMPI_Reduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, int root, struct ompi_communicator_t * comm);
int PMPI_Ireduce(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Reduce_local(const void * inbuf, void * inoutbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t *);
int PMPI_Reduce_scatter(const void * sendbuf, void * recvbuf, const int recvcounts[], struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int PMPI_Ireduce_scatter(const void * sendbuf, void * recvbuf, const int recvcounts[], struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Reduce_scatter_block(const void * sendbuf, void * recvbuf, int recvcount, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int PMPI_Ireduce_scatter_block(const void * sendbuf, void * recvbuf, int recvcount, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Register_datarep(const char * datarep, int (* read_conversion_fn)(void *, struct ompi_datatype_t *, int, void *, long long , void *), int (* write_conversion_fn)(void *, struct ompi_datatype_t *, int, void *, long long , void *), int (* dtype_file_extent_fn)(struct ompi_datatype_t *, long int (*), void *), void * extra_state);
int PMPI_Request_c2f(struct ompi_request_t * request);
struct ompi_request_t * PMPI_Request_f2c(int request);
int PMPI_Request_free(struct ompi_request_t * (* request));
int PMPI_Request_get_status(struct ompi_request_t * request, int * flag, struct ompi_status_public_t (* status));
int PMPI_Rget(void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win, struct ompi_request_t * (* request));
int PMPI_Rget_accumulate(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, void * result_addr, int result_count, struct ompi_datatype_t * result_datatype, int target_rank, long int target_disp, int target_count, struct ompi_datatype_t * target_datatype, struct ompi_op_t * op, struct ompi_win_t * win, struct ompi_request_t * (* request));
int PMPI_Rput(const void * origin_addr, int origin_count, struct ompi_datatype_t * origin_datatype, int target_rank, long int target_disp, int target_cout, struct ompi_datatype_t * target_datatype, struct ompi_win_t * win, struct ompi_request_t * (* request));
int PMPI_Rsend(const void * ibuf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int PMPI_Rsend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Scan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm);
int PMPI_Iscan(const void * sendbuf, void * recvbuf, int count, struct ompi_datatype_t * datatype, struct ompi_op_t * op, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Scatter(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int PMPI_Iscatter(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Scatterv(const void * sendbuf, const int sendcounts[], const int displs[], struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm);
int PMPI_Iscatterv(const void * sendbuf, const int sendcounts[], const int displs[], struct ompi_datatype_t * sendtype, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int root, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Send_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Send(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int PMPI_Sendrecv(const void * sendbuf, int sendcount, struct ompi_datatype_t * sendtype, int dest, int sendtag, void * recvbuf, int recvcount, struct ompi_datatype_t * recvtype, int source, int recvtag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int PMPI_Sendrecv_replace(void * buf, int count, struct ompi_datatype_t * datatype, int dest, int sendtag, int source, int recvtag, struct ompi_communicator_t * comm, struct ompi_status_public_t (* status));
int PMPI_Ssend_init(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm, struct ompi_request_t * (* request));
int PMPI_Ssend(const void * buf, int count, struct ompi_datatype_t * datatype, int dest, int tag, struct ompi_communicator_t * comm);
int PMPI_Start(struct ompi_request_t * (* request));
int PMPI_Startall(int count, struct ompi_request_t * (array_of_requests[]));
int PMPI_Status_c2f(const struct ompi_status_public_t (* c_status), int * f_status);
int PMPI_Status_f2c(const int * f_status, struct ompi_status_public_t (* c_status));
int PMPI_Status_set_cancelled(struct ompi_status_public_t (* status), int flag);
int PMPI_Status_set_elements(struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, int count);
int PMPI_Status_set_elements_x(struct ompi_status_public_t (* status), struct ompi_datatype_t * datatype, long long count);
int PMPI_Testall(int count, struct ompi_request_t * (array_of_requests[]), int * flag, struct ompi_status_public_t (array_of_statuses[]));
int PMPI_Testany(int count, struct ompi_request_t * (array_of_requests[]), int * index, int * flag, struct ompi_status_public_t (* status));
int PMPI_Test(struct ompi_request_t * (* request), int * flag, struct ompi_status_public_t (* status));
int PMPI_Test_cancelled(const struct ompi_status_public_t (* status), int * flag);
int PMPI_Testsome(int incount, struct ompi_request_t * (array_of_requests[]), int * outcount, int array_of_indices[], struct ompi_status_public_t (array_of_statuses[]));
int PMPI_Topo_test(struct ompi_communicator_t * comm, int * status);
int PMPI_Type_c2f(struct ompi_datatype_t * datatype);
int PMPI_Type_commit(struct ompi_datatype_t * (* type));
int PMPI_Type_contiguous(int count, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_darray(int size, int rank, int ndims, const int gsize_array[], const int distrib_array[], const int darg_array[], const int psize_array[], int order, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_f90_complex(int p, int r, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_f90_integer(int r, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_f90_real(int p, int r, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_hindexed(int count, const int array_of_blocklengths[], const long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_hvector(int count, int blocklength, long int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_keyval(int (* type_copy_attr_fn)(struct ompi_datatype_t *, int, void *, void *, void *, int *), int (* type_delete_attr_fn)(struct ompi_datatype_t *, int, void *, void *), int * type_keyval, void * extra_state);
int PMPI_Type_create_hindexed_block(int count, int blocklength, const long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_indexed_block(int count, int blocklength, const int array_of_displacements[], struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_struct(int count, const int array_of_block_lengths[], const long int (array_of_displacements[]), const struct ompi_datatype_t * (array_of_types[]), struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_subarray(int ndims, const int size_array[], const int subsize_array[], const int start_array[], int order, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_create_resized(struct ompi_datatype_t * oldtype, long int lb, long int extent, struct ompi_datatype_t * (* newtype));
int PMPI_Type_delete_attr(struct ompi_datatype_t * type, int type_keyval);
int PMPI_Type_dup(struct ompi_datatype_t * type, struct ompi_datatype_t * (* newtype));
int PMPI_Type_extent(struct ompi_datatype_t * type, long int (* extent));
int PMPI_Type_free(struct ompi_datatype_t * (* type));
int PMPI_Type_free_keyval(int * type_keyval);
struct ompi_datatype_t * PMPI_Type_f2c(int datatype);
int PMPI_Type_get_attr(struct ompi_datatype_t * type, int type_keyval, void * attribute_val, int * flag);
int PMPI_Type_get_contents(struct ompi_datatype_t * mtype, int max_integers, int max_addresses, int max_datatypes, int array_of_integers[], long int (array_of_addresses[]), struct ompi_datatype_t * (array_of_datatypes[]));
int PMPI_Type_get_envelope(struct ompi_datatype_t * type, int * num_integers, int * num_addresses, int * num_datatypes, int * combiner);
int PMPI_Type_get_extent(struct ompi_datatype_t * type, long int (* lb), long int (* extent));
int PMPI_Type_get_extent_x(struct ompi_datatype_t * type, long long (* lb), long long (* extent));
int PMPI_Type_get_name(struct ompi_datatype_t * type, char * type_name, int * resultlen);
int PMPI_Type_get_true_extent(struct ompi_datatype_t * datatype, long int (* true_lb), long int (* true_extent));
int PMPI_Type_get_true_extent_x(struct ompi_datatype_t * datatype, long long (* true_lb), long long (* true_extent));
int PMPI_Type_hindexed(int count, int array_of_blocklengths[], long int (array_of_displacements[]), struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_hvector(int count, int blocklength, long int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_indexed(int count, const int array_of_blocklengths[], const int array_of_displacements[], struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Type_lb(struct ompi_datatype_t * type, long int (* lb));
int PMPI_Type_match_size(int typeclass, int size, struct ompi_datatype_t * (* type));
int PMPI_Type_set_attr(struct ompi_datatype_t * type, int type_keyval, void * attr_val);
int PMPI_Type_set_name(struct ompi_datatype_t * type, const char * type_name);
int PMPI_Type_size(struct ompi_datatype_t * type, int * size);
int PMPI_Type_size_x(struct ompi_datatype_t * type, long long (* size));
int PMPI_Type_struct(int count, int array_of_blocklengths[], long int (array_of_displacements[]), struct ompi_datatype_t * (array_of_types[]), struct ompi_datatype_t * (* newtype));
int PMPI_Type_ub(struct ompi_datatype_t * mtype, long int (* ub));
int PMPI_Type_vector(int count, int blocklength, int stride, struct ompi_datatype_t * oldtype, struct ompi_datatype_t * (* newtype));
int PMPI_Unpack(const void * inbuf, int insize, int * position, void * outbuf, int outcount, struct ompi_datatype_t * datatype, struct ompi_communicator_t * comm);
int PMPI_Unpublish_name(const char * service_name, struct ompi_info_t * info, const char * port_name);
int PMPI_Unpack_external(const char datarep[], const void * inbuf, long int insize, long int (* position), void * outbuf, int outcount, struct ompi_datatype_t * datatype);
int PMPI_Waitall(int count, struct ompi_request_t * (array_of_requests[]), struct ompi_status_public_t (array_of_statuses[]));
int PMPI_Waitany(int count, struct ompi_request_t * (array_of_requests[]), int * index, struct ompi_status_public_t (* status));
int PMPI_Wait(struct ompi_request_t * (* request), struct ompi_status_public_t (* status));
int PMPI_Waitsome(int incount, struct ompi_request_t * (array_of_requests[]), int * outcount, int array_of_indices[], struct ompi_status_public_t (array_of_statuses[]));
int PMPI_Win_allocate(long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, void * baseptr, struct ompi_win_t * (* win));
int PMPI_Win_allocate_shared(long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, void * baseptr, struct ompi_win_t * (* win));
int PMPI_Win_attach(struct ompi_win_t * win, void * base, long int size);
int PMPI_Win_c2f(struct ompi_win_t * win);
int PMPI_Win_call_errhandler(struct ompi_win_t * win, int errorcode);
int PMPI_Win_complete(struct ompi_win_t * win);
int PMPI_Win_create(void * base, long int size, int disp_unit, struct ompi_info_t * info, struct ompi_communicator_t * comm, struct ompi_win_t * (* win));
int PMPI_Win_create_dynamic(struct ompi_info_t * info, struct ompi_communicator_t * comm, struct ompi_win_t * (* win));
int PMPI_Win_create_errhandler(void (* function)(struct ompi_win_t * (*), int *, ...), struct ompi_errhandler_t * (* errhandler));
int PMPI_Win_create_keyval(int (* win_copy_attr_fn)(struct ompi_win_t *, int, void *, void *, void *, int *), int (* win_delete_attr_fn)(struct ompi_win_t *, int, void *, void *), int * win_keyval, void * extra_state);
int PMPI_Win_delete_attr(struct ompi_win_t * win, int win_keyval);
int PMPI_Win_detach(struct ompi_win_t * win, const void * base);
struct ompi_win_t * PMPI_Win_f2c(int win);
int PMPI_Win_fence(int assert, struct ompi_win_t * win);
int PMPI_Win_flush(int rank, struct ompi_win_t * win);
int PMPI_Win_flush_all(struct ompi_win_t * win);
int PMPI_Win_flush_local(int rank, struct ompi_win_t * win);
int PMPI_Win_flush_local_all(struct ompi_win_t * win);
int PMPI_Win_free(struct ompi_win_t * (* win));
int PMPI_Win_free_keyval(int * win_keyval);
int PMPI_Win_get_attr(struct ompi_win_t * win, int win_keyval, void * attribute_val, int * flag);
int PMPI_Win_get_errhandler(struct ompi_win_t * win, struct ompi_errhandler_t * (* errhandler));
int PMPI_Win_get_group(struct ompi_win_t * win, struct ompi_group_t * (* group));
int PMPI_Win_get_info(struct ompi_win_t * win, struct ompi_info_t * (* info_used));
int PMPI_Win_get_name(struct ompi_win_t * win, char * win_name, int * resultlen);
int PMPI_Win_lock(int lock_type, int rank, int assert, struct ompi_win_t * win);
int PMPI_Win_lock_all(int assert, struct ompi_win_t * win);
int PMPI_Win_post(struct ompi_group_t * group, int assert, struct ompi_win_t * win);
int PMPI_Win_set_attr(struct ompi_win_t * win, int win_keyval, void * attribute_val);
int PMPI_Win_set_errhandler(struct ompi_win_t * win, struct ompi_errhandler_t * errhandler);
int PMPI_Win_set_info(struct ompi_win_t * win, struct ompi_info_t * info);
int PMPI_Win_set_name(struct ompi_win_t * win, const char * win_name);
int PMPI_Win_shared_query(struct ompi_win_t * win, int rank, long int (* size), int * disp_unit, void * baseptr);
int PMPI_Win_start(struct ompi_group_t * group, int assert, struct ompi_win_t * win);
int PMPI_Win_sync(struct ompi_win_t * win);
int PMPI_Win_test(struct ompi_win_t * win, int * flag);
int PMPI_Win_unlock(int rank, struct ompi_win_t * win);
int PMPI_Win_unlock_all(struct ompi_win_t * win);
int PMPI_Win_wait(struct ompi_win_t * win);
double PMPI_Wtick(void);
double PMPI_Wtime(void);
int PMPI_T_init_thread(int required, int * provided);
int PMPI_T_finalize(void);
int PMPI_T_cvar_get_num(int * num_cvar);
int PMPI_T_cvar_get_info(int cvar_index, char * name, int * name_len, int * verbosity, struct ompi_datatype_t * (* datatype), struct mca_base_var_enum_t * (* enumtype), char * desc, int * desc_len, int * bind, int * scope);
int PMPI_T_cvar_get_index(const char * name, int * cvar_index);
int PMPI_T_cvar_handle_alloc(int cvar_index, void * obj_handle, struct ompi_mpit_cvar_handle_t * (* handle), int * count);
int PMPI_T_cvar_handle_free(struct ompi_mpit_cvar_handle_t * (* handle));
int PMPI_T_cvar_read(struct ompi_mpit_cvar_handle_t * handle, void * buf);
int PMPI_T_cvar_write(struct ompi_mpit_cvar_handle_t * handle, const void * buf);
int PMPI_T_category_get_num(int * num_cat);
int PMPI_T_category_get_info(int cat_index, char * name, int * name_len, char * desc, int * desc_len, int * num_cvars, int * num_pvars, int * num_categories);
int PMPI_T_category_get_index(const char * name, int * category_index);
int PMPI_T_category_get_cvars(int cat_index, int len, int indices[]);
int PMPI_T_category_get_pvars(int cat_index, int len, int indices[]);
int PMPI_T_category_get_categories(int cat_index, int len, int indices[]);
int PMPI_T_category_changed(int * stamp);
int PMPI_T_pvar_get_num(int * num_pvar);
int PMPI_T_pvar_get_info(int pvar_index, char * name, int * name_len, int * verbosity, int * var_class, struct ompi_datatype_t * (* datatype), struct mca_base_var_enum_t * (* enumtype), char * desc, int * desc_len, int * bind, int * readonly, int * continuous, int * atomic);
int PMPI_T_pvar_get_index(const char * name, int var_class, int * pvar_index);
int PMPI_T_pvar_session_create(struct mca_base_pvar_session_t * (* session));
int PMPI_T_pvar_session_free(struct mca_base_pvar_session_t * (* session));
int PMPI_T_pvar_handle_alloc(struct mca_base_pvar_session_t * session, int pvar_index, void * obj_handle, struct mca_base_pvar_handle_t * (* handle), int * count);
int PMPI_T_pvar_handle_free(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * (* handle));
int PMPI_T_pvar_start(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int PMPI_T_pvar_stop(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int PMPI_T_pvar_read(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, void * buf);
int PMPI_T_pvar_write(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, const void * buf);
int PMPI_T_pvar_reset(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int PMPI_T_pvar_readreset(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, void * buf);
int PMPI_T_enum_get_info(struct mca_base_var_enum_t * enumtype, int * num, char * name, int * name_len);
int PMPI_T_enum_get_item(struct mca_base_var_enum_t * enumtype, int index, int * value, char * name, int * name_len);
int MPI_T_init_thread(int required, int * provided);
int MPI_T_finalize(void);
int MPI_T_cvar_get_num(int * num_cvar);
int MPI_T_cvar_get_info(int cvar_index, char * name, int * name_len, int * verbosity, struct ompi_datatype_t * (* datatype), struct mca_base_var_enum_t * (* enumtype), char * desc, int * desc_len, int * bind, int * scope);
int MPI_T_cvar_get_index(const char * name, int * cvar_index);
int MPI_T_cvar_handle_alloc(int cvar_index, void * obj_handle, struct ompi_mpit_cvar_handle_t * (* handle), int * count);
int MPI_T_cvar_handle_free(struct ompi_mpit_cvar_handle_t * (* handle));
int MPI_T_cvar_read(struct ompi_mpit_cvar_handle_t * handle, void * buf);
int MPI_T_cvar_write(struct ompi_mpit_cvar_handle_t * handle, const void * buf);
int MPI_T_category_get_num(int * num_cat);
int MPI_T_category_get_info(int cat_index, char * name, int * name_len, char * desc, int * desc_len, int * num_cvars, int * num_pvars, int * num_categories);
int MPI_T_category_get_index(const char * name, int * category_index);
int MPI_T_category_get_cvars(int cat_index, int len, int indices[]);
int MPI_T_category_get_pvars(int cat_index, int len, int indices[]);
int MPI_T_category_get_categories(int cat_index, int len, int indices[]);
int MPI_T_category_changed(int * stamp);
int MPI_T_pvar_get_num(int * num_pvar);
int MPI_T_pvar_get_info(int pvar_index, char * name, int * name_len, int * verbosity, int * var_class, struct ompi_datatype_t * (* datatype), struct mca_base_var_enum_t * (* enumtype), char * desc, int * desc_len, int * bind, int * readonly, int * continuous, int * atomic);
int MPI_T_pvar_get_index(const char * name, int var_class, int * pvar_index);
int MPI_T_pvar_session_create(struct mca_base_pvar_session_t * (* session));
int MPI_T_pvar_session_free(struct mca_base_pvar_session_t * (* session));
int MPI_T_pvar_handle_alloc(struct mca_base_pvar_session_t * session, int pvar_index, void * obj_handle, struct mca_base_pvar_handle_t * (* handle), int * count);
int MPI_T_pvar_handle_free(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * (* handle));
int MPI_T_pvar_start(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int MPI_T_pvar_stop(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int MPI_T_pvar_read(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, void * buf);
int MPI_T_pvar_write(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, const void * buf);
int MPI_T_pvar_reset(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle);
int MPI_T_pvar_readreset(struct mca_base_pvar_session_t * session, struct mca_base_pvar_handle_t * handle, void * buf);
int MPI_T_enum_get_info(struct mca_base_var_enum_t * enumtype, int * num, char * name, int * name_len);
int MPI_T_enum_get_item(struct mca_base_var_enum_t * enumtype, int index, int * value, char * name, int * name_len);
void torc_init2(void (* entry)(int, char **), int argc, char * argv[], int mode);
void torc_init(int argc, char * argv[], int mode);
void torc_reinit();
void torc_suspend();
void torc_reset_statistics();

/* (l22) typedef double torc_time_t; */

double torc_gettime();
int torc_i_worker_id(void);
int torc_i_num_workers();
int torc_worker_id();
int torc_num_workers();
int torc_getlevel();
void torc_yield(void);
void torc_enable_stealing();
void torc_disable_stealing();
void torc_register_task(void * f);
void torc_taskinit();
void torc_waitall();
void torc_tasksync();
void torc_create(int queue, void (* f)(), int narg, ...);
void torc_create_red(int queue, void (* f)(), int narg, ...);
void torc_create_ox(int flag, int atnode, int atworker, int detached, int tied, void (* work)(), void (* callback)(), int narg, ...);
void torc_create_ox1(int queue, void (* f)(), int narg, ...);
void torc_create_ox2(int queue, void (* f)(), void (* fc)(), int narg, ...);
void torc_create_ox4(int queue, int detached, int atnode, void (* work)(), void (* callback)(), int narg, ...);
void torc_create_local(void (* f)(), int narg, ...);
void torc_create_local_detached(void (* f)(), int narg, ...);
void torc_create_callback(int queue, void (* f)(), void (* fc)(), int narg, ...);
void torc_create_detached_callback(int queue, void (* f)(), void (* fc)(), int narg, ...);
void torc_spmd(int mi, void (* f)(), int narg, ...);
void torc_spmd_barrier();
void torc_op_red(void * global, void * local, int n, struct ompi_datatype_t * dtype, int redop);
void torc_op_red_ox(void * global, void * local, int n, int dtype, int redop);
int torc_node_id();
int torc_num_nodes();
void torc_broadcast(void * a, long count, struct ompi_datatype_t * dtype);
void torc_broadcast_ox(void * a, long count, int dtype);
void torc_rma_alloc(void ** mem, long count, struct ompi_datatype_t * dtype);
void torc_rma_mmap(void * mem, long count, struct ompi_datatype_t * dtype);
void thread_sleep(int ms);
int torc_sched_nextcpu(int cpu, int stride);
void torc_setschedule(int policy, int stealing, int base, int stride);
void torc_set_task_omp_init_routine(void (* routine)(int, void *));
void torc_end(void);
void torc_isomalloc(void ** mem, unsigned long size);
void torc_barrier(int me, int size);
void torc_lock_init(int * lock);
void torc_lock_acq(int * lock);
void torc_lock_rel(int * lock);
void torc_create_spmd(void (* f)(), int size);
int torc_spmd_rank();
int torc_spmd_size();
void torc_sendto(int rank, void * msg, int size);
void torc_recvfrom(int rank, void * msg, int size);
void torc_add_nodes(int nodes);
void torc_del_node(int nodeid);


double my_gettime()
{
  struct timeval tv;

  gettimeofday(&tv, 0);
  return ((double) (tv.tv_sec + tv.tv_usec / 1000000.0));
}

int SH = 36;
int SW = 32;
float SR = 36.0 / 32.0;
struct CNN gl_cnn;
struct per_image_data {
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


void InitData(struct per_image_data * data)
{
  data->outputValues = ((void *) 0);
  data->imgt = ((void *) 0);
  data->imgt_f = ((void *) 0);
  data->cconv = ((void *) 0);
  data->Xs = ((void *) 0);
  data->Ys = ((void *) 0);
  data->outputs = ((void *) 0);
  data->heights = ((void *) 0);
  data->Xf = ((void *) 0);
  data->Yf = ((void *) 0);
  data->outf = ((void *) 0);
  data->Hf = ((void *) 0);
  data->gl_S1 = 0;
  data->gl_S2 = 0;
  data->gl_ST = 0.0;
}

void torc_phase1_torctask(int (* _ompix_iter), int (* _ompix_itotal), void * img_source_ptr, void * img_input_ptr, void * data_ptr, int * Sm, int * facesFound);


void torc_phase1(int iter, int itotal, void * img_source_ptr, void * img_input_ptr, void * data_ptr, int * Sm, int * facesFound)
{
  int i, S, SH = 36;
  float SR = 36.0 / 32.0;
  struct per_image_data * data;
  struct lpiImage * img_source, * img_input;

  img_source = (struct lpiImage *) img_source_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  data = (struct per_image_data *) data_ptr;
  {
    int i, j, S;
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
          printf("iter = %d\n", iter);
          exit(1);
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
          data->Xs[*facesFound] = xcenter;
          data->Ys[*facesFound] = ycenter;
          data->heights[*facesFound] = S;
          data->outputs[*facesFound] = output;
          (*facesFound)++;
        }
      lpiImage_lpiReleaseImage(&data->imgt[nbS]);
      CConvolver_DeallocateOutput(&data->cconv[nbS]);
      lpiImage_lpiReleaseImage(&img_tmp);
    }
  }
}

/* #pragma ompix taskdef IN(iter, itotal) INOUT(img_source_ptr[ 1], img_input_ptr[ 1], facesFound[ 1], data_ptr[ 1], Sm[ 64]) */

void torc_phase1_torctask(int (* _ompix_iter), int (* _ompix_itotal), void * img_source_ptr, void * img_input_ptr, void * data_ptr, int * Sm, int * facesFound)
{
  int itotal = *_ompix_itotal;
  int iter = *_ompix_iter;
    int i, S, SH = 36;
  float SR = 36.0 / 32.0;
  struct per_image_data * data;
  struct lpiImage * img_source, * img_input;

  img_source = (struct lpiImage *) img_source_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  data = (struct per_image_data *) data_ptr;
  {
    int i, j, S;
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
          printf("iter = %d\n", iter);
          exit(1);
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
          data->Xs[*facesFound] = xcenter;
          data->Ys[*facesFound] = ycenter;
          data->heights[*facesFound] = S;
          data->outputs[*facesFound] = output;
          (*facesFound)++;
        }
      lpiImage_lpiReleaseImage(&data->imgt[nbS]);
      CConvolver_DeallocateOutput(&data->cconv[nbS]);
      lpiImage_lpiReleaseImage(&img_tmp);
    }
  }
}

int searchFacesConv(struct CNN * cnn, struct lpiImage * img_source, int S1, int S2, double ST);
int searchFacesConvFine(struct CNN * cnn, struct lpiImage * img_source, int S1, int S2, double ST, int * X, int * Y, int * H, int * W, float * O);
int searchFacesConvFineStatic(struct per_image_data * data, struct CNN * cnn, struct lpiImage * img_source, int S1, int S2, double ST, int * X, int * Y, int * H, int * W, float * O);
void drawFace(struct lpiImage * img, int CX, int CY, int CW, int CH, float vol, int size, struct lpiImage * img_input, int id);
void savePGM(struct lpiImage * img, int count);
void VoteGrouping2(struct per_image_data * data, int * Xs, int * Ys, float * outputs, int * heights, int size, double minVoteSize, double thrMaxVote, int * Xf, int * Yf, float * outf, int * Hf, int * nb);
int overlap(int i, int j, int Xri, int Yri, int Hri, int Wri, float outri, int Xrj, int Yrj, int Hrj, int Wrj, float outrj, float * perc);
int InitImages(struct per_image_data * data, int width, int height, int S1, int S2, double ST, int still_image);
void FreeImages(struct per_image_data * data, int still_image);
int my_round(double x);
int CropRescaleConvolve(struct CNN * cnn, struct lpiImage * img, float S, int XoS, int YoS, float * maxOut, int * Xo, int * Yo, float * So, float * mean);
int CropRescaleConvolveFloat(struct CNN * cnn, struct lpiImage * img, float S, int XoS, int YoS, float * maxOut, int * Xo, int * Yo, float * So, float * mean);
extern int gl_playcam;
static int videoInit = 0;
void torc_phase3_torctask(void * data_ptr, int (* _ompix_n), int (* _ompix_n_end), void * img_input_ptr, void * cnn_ptr, int * allSize);


void torc_phase3(void * data_ptr, int n, int n_end, void * img_input_ptr, void * cnn_ptr, int * allSize)
{
  int nbS;
  struct per_image_data * data;
  struct lpiImage * img_input;
  struct CNN * cnn;

  data = (struct per_image_data *) data_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  cnn = (struct CNN *) cnn_ptr;
  float S11 = 0.8 * data->Hf[n];
  float S21 = 1.5 * data->Hf[n];
  float dS = (S21 - S11) / 10.0;
  int recount = 0;
  float maxOut = -1.0;
  int Xo = 0;
  int Yo = 0;
  float So = 0.0;
  int XoS = data->Xf[n];
  int YoS = data->Yf[n];
  float mean = 0.0;
  int k;
  float S;
  double lt1, lt2;

  lt1 = my_gettime();
  nbS = 0;
  for (k = 0, S = S11; k < 10; k++, S += dS)
    {
      if (k == 0 || k == 2 || k == 4 || k == 5)
        {
          recount += CropRescaleConvolve(cnn, img_input, S, XoS, YoS, &maxOut, &Xo, &Yo, &So, &mean);
        }
    }
  if (mean < 10.0 || recount == 0)
    return;
  data->allX[*allSize] = my_round((double) Xo);
  data->allY[*allSize] = my_round((double) Yo);
  data->allH[*allSize] = my_round((double) So);
  data->allOut[*allSize] = mean;
  (*allSize)++;
  lt2 = my_gettime();
}

/* #pragma ompix taskdef IN(n, n_end) INOUT(data_ptr[ 1], img_input_ptr, cnn_ptr[ 1], allSize[ 1]) */

void torc_phase3_torctask(void * data_ptr, int (* _ompix_n), int (* _ompix_n_end), void * img_input_ptr, void * cnn_ptr, int * allSize)
{
  int n_end = *_ompix_n_end;
  int n = *_ompix_n;
    int nbS;
  struct per_image_data * data;
  struct lpiImage * img_input;
  struct CNN * cnn;

  data = (struct per_image_data *) data_ptr;
  img_input = (struct lpiImage *) img_input_ptr;
  cnn = (struct CNN *) cnn_ptr;
  float S11 = 0.8 * data->Hf[n];
  float S21 = 1.5 * data->Hf[n];
  float dS = (S21 - S11) / 10.0;
  int recount = 0;
  float maxOut = -1.0;
  int Xo = 0;
  int Yo = 0;
  float So = 0.0;
  int XoS = data->Xf[n];
  int YoS = data->Yf[n];
  float mean = 0.0;
  int k;
  float S;
  double lt1, lt2;

  lt1 = my_gettime();
  nbS = 0;
  for (k = 0, S = S11; k < 10; k++, S += dS)
    {
      if (k == 0 || k == 2 || k == 4 || k == 5)
        {
          recount += CropRescaleConvolve(cnn, img_input, S, XoS, YoS, &maxOut, &Xo, &Yo, &So, &mean);
        }
    }
  if (mean < 10.0 || recount == 0)
    return;
  data->allX[*allSize] = my_round((double) Xo);
  data->allY[*allSize] = my_round((double) Yo);
  data->allH[*allSize] = my_round((double) So);
  data->allOut[*allSize] = mean;
  (*allSize)++;
  lt2 = my_gettime();
}


void InitCFF()
{
  CNN_CreateConvolveKernels(&gl_cnn);
  init_tanh();
}


int callbackStillImage(struct lpiImage * image, int * X, int * Y, int * H, int * W, float * O)
{
  struct per_image_data data;
  int nofFaces;

  InitData(&data);
  InitImages(&data, image->width, image->height, 36, 360, 1.2, 1);
  nofFaces = searchFacesConvFineStatic(&data, &gl_cnn, image, 36, 360, 1.2, X, Y, H, W, O);
  FreeImages(&data, 1);
  return ((nofFaces));
}


int InitImages(struct per_image_data * data, int width, int height, int S1, int S2, double ST, int still_image)
{
  int S;
  int nbS = 0;

  for (S = S1; S <= S2; S = (int) floor((float) (S) * ST))
    {
      int h1 = (int) floor((double) height * ((double) SH / (double) S));
      int w1 = (int) floor((double) width * ((double) SH / (double) S));

      if (h1 >= 36 && w1 >= 32)
        nbS++;
    }
  data->imgt = (struct lpiImage **) calloc(nbS, sizeof(struct lpiImage *));
  data->cconv = (struct CConvolver *) calloc(nbS, sizeof(struct CConvolver));
  data->imgt_f = (struct lpiImage **) calloc(nbS, sizeof(struct lpiImage *));
  nbS = 0;
  for (S = S1; S <= S2; S = (int) floor((float) (S) * ST))
    {
      int h1 = (int) floor((double) height * ((double) SH / (double) S));
      int w1 = (int) floor((double) width * ((double) SH / (double) S));

      if (h1 < 36 || w1 < 32)
        continue;
      if (still_image == 0)
        {
          data->imgt[nbS] = lpiImage_lpiCreateImage(w1, h1, sizeof(unsigned char));
          data->imgt_f[nbS] = lpiImage_lpiCreateImage(w1, h1, sizeof(float));
          CConvolver_InitFMs(&data->cconv[nbS], w1, h1);
        }
      CConvolver_SetCNN(&data->cconv[nbS], &gl_cnn);
      nbS++;
    }
  data->Xs = Alloc1DInt(1000);
  data->Ys = Alloc1DInt(1000);
  data->outputs = Alloc1DFloat(1000);
  data->heights = Alloc1DInt(1000);
  data->Xf = Alloc1DInt(1000);
  data->Yf = Alloc1DInt(1000);
  data->outf = Alloc1DFloat(1000);
  data->Hf = Alloc1DInt(1000);
  data->gl_S1 = S1;
  data->gl_S2 = S2;
  data->gl_ST = ST;
  data->gl_width = width;
  data->gl_height = height;
  return ((1));
}


void FreeImages(struct per_image_data * data, int still_image)
{
  int S;
  int nbS = 0;

  for (S = data->gl_S1; S <= data->gl_S2; S = (int) floor((float) (S) * data->gl_ST))
    {
      int h1 = (int) floor((double) data->gl_height * ((double) SH / (double) S));
      int w1 = (int) floor((double) data->gl_width * ((double) SH / (double) S));

      if (h1 < 36 || w1 < 32)
        continue;
      if (still_image == 0)
        {
          lpiImage_lpiReleaseImage(&data->imgt[nbS]);
          CConvolver_FreeFMs(&data->cconv[nbS]);
          lpiImage_lpiReleaseImage(&data->imgt_f[nbS]);
        }
      nbS++;
    }
  free(data->imgt);
  free(data->cconv);
  free(data->imgt_f);
  Del1D(data->Xs);
  Del1D(data->Ys);
  Del1D(data->outputs);
  Del1D(data->heights);
  Del1D(data->Xf);
  Del1D(data->Yf);
  Del1D(data->outf);
  Del1D(data->Hf);
  videoInit = 0;
}


int CropRescaleConvolve(struct CNN * cnn, struct lpiImage * img, float S, int XoS, int YoS, float * maxOut, int * Xo, int * Yo, float * So, float * mean)
{
  float dispCenterY = ((float) 8.0) * S / (float) SH;
  float dispCenterX = ((float) 8.0) * S / (float) SH;
  float dispX = dispCenterX + (S / ((float) 2.0)) * ((float) SW) / ((float) SH);
  float dispY = dispCenterY + S / ((float) 2.0);
  int topLeftX = XoS - (int) floor((double) dispX);
  int topLeftY = YoS - (int) floor((double) dispY);
  int bottomRightX = XoS + (int) ceil((double) dispX);
  int bottomRightY = YoS + (int) ceil((double) dispY);
  int i, j;
  int ww = bottomRightX - topLeftX + 1;
  int hh = bottomRightY - topLeftY + 1;
  struct lpiImage * imgCr = lpiImage_lpiCreateImage(ww, hh, sizeof(unsigned char));
  unsigned char * pimg = (unsigned char *) img->imageData;
  unsigned char * pcr = (unsigned char *) imgCr->imageData;
  int wS = img->width;
  int wwS = imgCr->width;
  int scaledWidth;
  int scaledHeight;
  struct lpiImage * imgScaled;
  float * res;
  struct lpiImage * img_tmp;
  unsigned char * p1;
  float * p2;
  struct CConvolverFine cc;
  int nofPos;

  for (i = 0; i < hh; i++)
    {
      int i_wwS = i * wwS;
      int itopLeftY_wS = (i + topLeftY) * wS;

      for (j = 0; j < ww; j++)
        {
          if ((i + topLeftY) < 0 || (i + topLeftY) >= img->height || (j + topLeftX) < 0 || (j + topLeftX) >= img->width)
            pcr[i_wwS + j] = 0;
          else
            pcr[i_wwS + j] = pimg[itopLeftY_wS + j + topLeftX];
        }
    }
  scaledWidth = SW + 16;
  scaledHeight = SH + 16;
  imgScaled = lpiImage_lpiCreateImage(scaledWidth, scaledHeight, sizeof(unsigned char));
  lpiImage_lpiResize(imgCr, imgScaled);
  res = (float *) calloc(scaledWidth * scaledHeight, sizeof(float));
  img_tmp = lpiImage_lpiCreateImage(scaledWidth, scaledHeight, sizeof(float));
  p1 = (unsigned char *) imgScaled->imageData;
  p2 = (float *) img_tmp->imageData;
  for (i = 0; i < scaledHeight; i++)
    for (j = 0; j < scaledWidth; j++)
      p2[i * scaledWidth + j] = (float) (((int) p1[i * imgScaled->width + j]) - 128) / ((float) 128.0);
  lpiImage_lpiReleaseImage(&imgCr);
  lpiImage_lpiReleaseImage(&imgScaled);
  CConvolverFine_SetCNN(&cc, cnn);
  CConvolverFine_ConvolveFine(&cc, img_tmp, res);
  nofPos = 0;
  for (i = 0; i < scaledHeight - SH; i++)
    for (j = 0; j < scaledWidth - SW; j++)
      {
        int centerY;
        int centerX;
        int unScaledCenterX;
        int unScaledCenterY;
        float resf = res[i * scaledWidth + j];

        if (resf < 0.0)
          continue;
        nofPos++;
        *mean += resf;
        if (resf < *maxOut)
          continue;
        *maxOut = resf;
        centerY = i + SH / 2;
        centerX = j + SW / 2;
        unScaledCenterX = my_round(((double) centerX) * ((double) S) / ((double) SH));
        unScaledCenterY = my_round(((double) centerY) * ((double) S) / ((double) SH));
        *Xo = unScaledCenterX + topLeftX;
        *Yo = unScaledCenterY + topLeftY;
        *So = S;
      }
  lpiImage_lpiReleaseImage(&img_tmp);
  free(res);
  return ((nofPos));
}


void drawFace(struct lpiImage * img, int CX, int CY, int CW, int CH, float vol, int size, struct lpiImage * img_input, int id)
{
}


void VoteGrouping2(struct per_image_data * data, int * Xs, int * Ys, float * outputs, int * heights, int size, double minVoteSize, double thrMaxVote, int * Xf, int * Yf, float * outf, int * Hf, int * nb)
{
  int i, j;
  float cxNum = 0.0;
  float cyNum = 0.0;
  float chNum = 0.0;
  float Denom = 0.0;
  float totalOutput = 0.0;
  int CX = 0, CY = 0, CH = 0, CW = 0;
  int votesInCluster = 0;
  float maxout;
  float swapf;
  int swapi;
  int indmax = 0;
  int pointsLeft;

  for (i = 0; i < size; i++)
    data->vg_deleted[i] = 0;
  for (i = 0; i < size; i++)
    {
      maxout = outputs[i];
      indmax = i;
      for (j = i; j < size; j++)
        {
          if (outputs[j] > maxout)
            {
              maxout = outputs[j];
              indmax = j;
            }
        }
      swapf = outputs[i];
      outputs[i] = outputs[indmax];
      outputs[indmax] = swapf;
      swapi = Xs[i];
      Xs[i] = Xs[indmax];
      Xs[indmax] = swapi;
      swapi = Ys[i];
      Ys[i] = Ys[indmax];
      Ys[indmax] = swapi;
      swapi = heights[i];
      heights[i] = heights[indmax];
      heights[indmax] = swapi;
    }
  pointsLeft = 1;
  while (pointsLeft == 1)
    {
      pointsLeft = 0;
      for (i = 0; i < size; i++)
        {
          float fCX;
          float fCY;
          float fCH;
          float fCW;

          if (data->vg_deleted[i] == 1)
            continue;
          if (pointsLeft == 0)
            {
              pointsLeft = 1;
              cxNum = outputs[i] * ((float) Xs[i]);
              cyNum = outputs[i] * ((float) Ys[i]);
              chNum = outputs[i] * ((float) heights[i]);
              Denom = outputs[i];
              totalOutput = outputs[i];
              data->vg_deleted[i] = 1;
              votesInCluster = 1;
              continue;
            }
          fCX = cxNum / Denom;
          fCY = cyNum / Denom;
          fCH = chNum / Denom;
          fCW = SR * fCH;
          if (Xs[i] < fCX + fCW / 3.0 && Xs[i] > fCX - fCW / 3.0 && Ys[i] < fCY + fCH / 3.0 && Ys[i] > fCY - fCH / 3.0)
            {
              data->vg_deleted[i] = 1;
              cxNum += outputs[i] * ((float) Xs[i]);
              cyNum += outputs[i] * ((float) Ys[i]);
              chNum += outputs[i] * ((float) heights[i]);
              Denom += outputs[i];
              totalOutput += outputs[i];
              votesInCluster++;
            }
        }
      if (pointsLeft == 0)
        {
          return;
        }
      CX = (int) (cxNum / Denom);
      CY = (int) (cyNum / Denom);
      CH = (int) (chNum / Denom);
      CW = (int) (SR * (double) CH);
      Xf[*nb] = CX;
      Yf[*nb] = CY;
      Hf[*nb] = CH;
      outf[*nb] = totalOutput;
      (*nb)++;
    }
}


int overlap(int i, int j, int Xri, int Yri, int Hri, int Wri, float outri, int Xrj, int Yrj, int Hrj, int Wrj, float outrj, float * perc)
{
  int ind = -1;
  int Xmini = Xri - Wri / 2;
  int Xmaxi = Xri + Wri / 2;
  int Ymini = Yri - Hri / 2;
  int Ymaxi = Yri + Hri / 2;
  int Xminj = Xrj - Wrj / 2;
  int Xmaxj = Xrj + Wrj / 2;
  int Yminj = Yrj - Hrj / 2;
  int Ymaxj = Yrj + Hrj / 2;
  float perci;
  float percj;
  int nb = 0;
  int x, y;

  for (x = Xmini; x <= Xmaxi; x++)
    for (y = Ymini; y <= Ymaxi; y++)
      if ((x >= Xminj) && (x <= Xmaxj) && (y >= Yminj) && (y <= Ymaxj))
        nb++;
  perci = ((float) nb) / ((float) (Hri * Wri));
  percj = ((float) nb) / ((float) (Hrj * Wrj));
  if (perci > percj)
    *perc = perci;
  else
    *perc = percj;
  if ((*perc) > 0.2)
    {
      if (outrj < outri)
        ind = j;
      else
        ind = i;
    }
  return (ind);
}


int my_round(double x)
{
  double decimal = floor(x);

  if (x - decimal < 0.5)
    return (((int) x));
  else
    return (((int) (x + 1.0)));
}

int L2_step = 0;


int searchFacesConvFineStatic(struct per_image_data * data, struct CNN * cnn, struct lpiImage * img_source, int S1, int S2, double ST, int * X, int * Y, int * H, int * W, float * O)
{
  int iter;
  double t1, t2, t3, t4, t5, t6;
  int Sm[ 64], itotal;
  int S;
  int Si;
  void * lock;
  struct lpiImage * img_input = img_source;
  int nbS = 0;
  int facesFound = 0;
  int allClusters;
  int minVoteSize;
  int allSize;
  int n;
  int i, j, my_node;
  int nofFaces;

  t1 = my_gettime();
  iter = 0;
  for (Si = S1; Si <= S2; Si = (int) floor((float) (Si) * ST))
    {
      int width = img_source->width;
      int height = img_source->height;
      int h1 = (int) floor((double) height * ((double) SH / (double) Si));
      int w1 = (int) floor((double) width * ((double) SH / (double) Si));

      if (h1 < 36 || w1 < 32)
        continue;
      Sm[iter] = Si;
      iter++;
    }
  itotal = iter;
  for (iter = 0; iter < itotal; iter++)
    {
      /* #pragma ompix task atnode(here) */
      {
         
        int _ompix_0 = iter;
        int _ompix_1 = itotal;

                torc_create_ox(2, -1, -1, 0, -1, torc_phase1_torctask, (void *) 0, 7, 1, 0, 1, 1, 0, 1, 1, 0, 2, 1, 0, 2, 1, 0, 2, 64, 0, 2, 1, 0, 2, &_ompix_0, &_ompix_1, (void *) img_source, (void *) img_input, (void *) data, &Sm, &facesFound);
      }
    }
  /* #pragma ompix tasksync  */
  torc_tasksync();
    ;
  t2 = my_gettime();
  nbS = itotal + 1;
  if (facesFound < 1)
    {
      return ((0));
    }
  allClusters = 0;
  minVoteSize = 1;
  VoteGrouping2(data, data->Xs, data->Ys, data->outputs, data->heights, facesFound, minVoteSize, 0.0, data->Xf, data->Yf, data->outf, data->Hf, &allClusters);
  t3 = my_gettime();
  allSize = 0;
  for (n = 0; n < allClusters; n++)
    {
      /* #pragma ompix task atnode(here) */
      {
         
        int _ompix_1 = n;
        int _ompix_2 = n + 1;

                torc_create_ox(2, -1, -1, 0, -1, torc_phase3_torctask, (void *) 0, 6, 1, 0, 2, 1, 0, 1, 1, 0, 1, 1, 0, 2, 1, 0, 2, 1, 0, 2, data, &_ompix_1, &_ompix_2, img_input, cnn, &allSize);
      }
      fflush(0);
      continue;
      float S11 = 0.8 * data->Hf[n];
      float S21 = 1.5 * data->Hf[n];
      float dS = (S21 - S11) / 10.0;
      int recount = 0;
      float maxOut = -1.0;
      int Xo = 0;
      int Yo = 0;
      float So = 0.0;
      int XoS = data->Xf[n];
      int YoS = data->Yf[n];
      float mean = 0.0;
      int k;
      float S;
      int iter_cnt = 0;
      double lt1, lt2;

      lt1 = my_gettime();
      L2_step++;
      nbS = 0;
      for (k = 0, S = S11; k < 10; k++, S += dS, iter_cnt++)
        {
          if (k == 0 || k == 2 || k == 4 || k == 5)
            {
              recount += CropRescaleConvolve(cnn, img_input, S, XoS, YoS, &maxOut, &Xo, &Yo, &So, &mean);
            }
        }
      if (mean < 10.0 || recount == 0)
        continue;
      {
        data->allX[allSize] = my_round((double) Xo);
        data->allY[allSize] = my_round((double) Yo);
        data->allH[allSize] = my_round((double) So);
        data->allOut[allSize] = mean;
        allSize++;
      }
      lt2 = my_gettime();
    }
  /* #pragma ompix tasksync  */
  torc_tasksync();
    t4 = my_gettime();
  for (i = 0; i < allSize; i++)
    data->discarded[i] = 0;
  for (i = 0; i < allSize; i++)
    {
      for (j = i + 1; j < allSize; j++)
        {
          float perc = 0.0;
          int r;

          if (data->discarded[i] == 1 || data->discarded[j] == 1)
            continue;
          r = overlap(i, j, data->allX[i], data->allY[i], data->allH[i], (int) (SR * data->allH[i]), data->allOut[i], data->allX[j], data->allY[j], data->allH[j], (int) (SR * data->allH[j]), data->allOut[j], &perc);
          if (r == i)
            data->discarded[i] = 1;
          else
            if (r == j)
              data->discarded[j] = 1;
        }
    }
  t5 = my_gettime();
  nofFaces = 0;
  for (i = 0; i < allSize; i++)
    {
      int Sw;

      if (data->discarded[i] == 1)
        continue;
      Sw = (int) (SR * data->allH[i]);
      drawFace(img_source, data->allX[i], data->allY[i], Sw, data->allH[i], data->allOut[i], 1, img_input, i);
      X[nofFaces] = data->allX[i];
      Y[nofFaces] = data->allY[i];
      H[nofFaces] = data->allH[i];
      W[nofFaces] = Sw;
      O[nofFaces] = data->allOut[i];
      nofFaces++;
    }
  t6 = my_gettime();
  return ((nofFaces));
}



