/* Test-only libc for the Windows no-CRT behavioral DLL. Target uses SDK libc. */
#ifdef _WIN32
#include <stddef.h>
#include <stdint.h>
void *memcpy(void *to,const void *from,size_t n){unsigned char *d=to;const unsigned char *s=from;for(size_t i=0;i<n;++i)d[i]=s[i];return to;}
void *memset(void *to,int value,size_t n){unsigned char *d=to;for(size_t i=0;i<n;++i)d[i]=(unsigned char)value;return to;}
void *memmove(void *to,const void *from,size_t n){unsigned char *d=to;const unsigned char *s=from;if((uintptr_t)d<(uintptr_t)s){for(size_t i=0;i<n;++i)d[i]=s[i];}else if((uintptr_t)d>(uintptr_t)s){while(n){--n;d[n]=s[n];}}return to;}
int memcmp(const void *a,const void *b,size_t n){const unsigned char *x=a,*y=b;for(size_t i=0;i<n;++i){if(x[i]!=y[i])return x[i]<y[i]?-1:1;}return 0;}
size_t strlen(const char *s){size_t n=0;while(s[n])++n;return n;}
#endif
