#ifndef THREAD_CONFIG_H

#define THREAD_CONFIG_H

#ifdef _MSC_VER
typedef __int64 int64_t;
#else
#include <stdint.h>
#endif

namespace THREAD_CONFIG
{

unsigned int tc_timeGetTime(void);
void     tc_sleep(unsigned int ms);

void     tc_spinloop();
void     tc_interlockedExchange(void *dest, const int64_t exchange);
int      tc_interlockedCompareExchange(void *dest, int exchange, int compare);
int      tc_interlockedCompareExchange(void *dest, const int exchange1, const int exchange2, const int compare1, const int compare2);

class ThreadMutex
{
public:
  virtual void lock(void) = 0;
  virtual void unlock(void) = 0;
  virtual bool tryLock(void) = 0;
};


ThreadMutex * tc_createThreadMutex(void);
void          tc_releaseThreadMutex(ThreadMutex *tm);

class ThreadInterface
{
public:
  virtual void threadMain(void) = 0;
};

class Thread
{
public:
};

Thread      * tc_createThread(ThreadInterface *tinterface);
void          tc_releaseThread(Thread *t);

class ThreadEvent
{
public:
  virtual void setEvent(void) = 0; // signal the event
  virtual void resetEvent(void) = 0;
  virtual void waitForSingleObject(unsigned int ms) = 0;
};

ThreadEvent * tc_createThreadEvent(void);
void          tc_releaseThreadEvent(ThreadEvent *t);

}; // end of namespace


#endif
