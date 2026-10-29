#ifdef _WIN32
#include <stddef.h>
void *memcpy(void *d,const void *s,size_t n) {unsigned char *a=d; const unsigned char *b=s; for(size_t i=0;i<n;i++)a[i]=b[i]; return d;}
void *memmove(void *d,const void *s,size_t n) {unsigned char *a=d; const unsigned char *b=s; if(a<b)for(size_t i=0;i<n;i++)a[i]=b[i];else for(size_t i=n;i;i--)a[i-1]=b[i-1];return d;}
void *memset(void *d,int c,size_t n) {unsigned char *a=d;for(size_t i=0;i<n;i++)a[i]=(unsigned char)c;return d;}
int memcmp(const void *a,const void *b,size_t n) {const unsigned char *x=a,*y=b;for(size_t i=0;i<n;i++)if(x[i]!=y[i])return x[i]<y[i]?-1:1;return 0;}
#endif
