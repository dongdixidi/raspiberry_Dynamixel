#ifndef __DEBUG_H_
#define __DEBUG_H_
#include <time.h>

#define PRINT_LEVEL   3

#define debug

#if 1
#ifdef debug
#define DEBUG(level,format, ...) \
    do{  \
    if(level < PRINT_LEVEL){  \
        time_t timep;\
        time (&timep);\
        char *stime=ctime(&timep);\
        stime[24]=0;\
        printf("[%s]: " format,stime,  ##__VA_ARGS__);  \
        }\
}while(0)    
#else
#define DEBUG  
#endif
#endif

#if 0
#ifdef debug
#define DEBUG(level,format, ...) \
        do{  \
        if(level < PRINT_LEVEL){  \
            printf(format,  ##__VA_ARGS__);  \
            }\
    }while(0)  

//#ifdef  __cplusplus
//#endif  /* end of __cplusplus */ 
#else
#define DEBUG  
#endif


#endif
#endif


