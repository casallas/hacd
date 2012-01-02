#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <assert.h>
#include <new>

/*!
**
** Copyright (c) 20011 by John W. Ratcliff mailto:jratcliffscarab@gmail.com
**
**
** The MIT license:
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is furnished
** to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.

** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
** WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/
#include "JobSwarm.h"
#include "ThreadConfig.h"
#include "LockFreeQ.h"
#include "pool.h"

JOB_SWARM::JobSwarmContext *gJobSwarmContext=0;

namespace JOB_SWARM
{





class SwarmJob : public LOCK_FREE_Q::node_t
{
public:
	SwarmJob(void)
	{
		mNext      = NULL;
		mPrevious  = NULL;
		mInterface = NULL;
		mCancelled = false;
		mUserData  = NULL;
		mUserId    = 0;
	}

	SwarmJob(JobSwarmInterface *iface,void *userData,hacd::HaI32 userId)
	{
		mInterface = iface;
		mCancelled = false;
		mUserData  = userData;
		mUserId    = userId;
	}

	// completion event always run from the main thread!
	void notifyCompletion(void)
	{
		HACD_ASSERT(mInterface);
		if ( mInterface )
		{
			if ( mCancelled )
				mInterface->job_onCancel(mUserData,mUserId);
			else
				mInterface->job_onFinish(mUserData,mUserId);
		}
		mInterface = NULL;
	}


	// run from thread of course
	void onExecute(void)
	{
		mInterface->job_process(mUserData,mUserId);
	}


	bool isCancelled(void) const { return mCancelled; };

	void cancel(void)
	{
		mCancelled = true;
	}

	SwarmJob *  GetNext(void) const              { return mNext; };
	SwarmJob *  GetPrevious(void) const          { return mPrevious; };
	void        SetNext(SwarmJob *next)         { mNext = next; };
	void        SetPrevious(SwarmJob *previous) { mPrevious = previous; };

private:
	SwarmJob          *mNext;
	SwarmJob          *mPrevious;
	JobSwarmInterface *mInterface;
	bool                mCancelled;
	void               *mUserData;
	hacd::HaI32                 mUserId;
  };

  class JobScheduler;

  // Each thread worker can keep track of 4 jobs.
  // There can be as many thread workers as there application desires.

#define MAX_COMPLETION 4096 // maximum number of completed jobs we will queue up before they get despooled by the main thread.

class ThreadWorker : public THREAD_CONFIG::ThreadInterface, public UANS::UserAllocated
{
public:

	ThreadWorker(void)
	{
		mActiveThreadCount = NULL;
		mJobScheduler = 0;
		mExit         = false;
		mBusy         = 0;
		mThread     = 0;
		mFinished.init(MAX_COMPLETION);
	}

    ~ThreadWorker(void)
    {
      release();
    }

    void release(void)
    {
      THREAD_CONFIG::tc_releaseThreadEvent(mBusy);
      THREAD_CONFIG::tc_releaseThread(mThread);
      mBusy = 0;
      mThread = 0;
    }

	void setJobScheduler(JobScheduler *job,hacd::HaI32 *activeThreadCount);

    // occurs in another thread
    void threadMain(void);

    SwarmJob * getFinished(void)
    {
      SwarmJob *ret = 0;
      mFinished.pop(ret);
      return ret;
    }

    void setExit(void)
    {
      mExit = true;
      mBusy->setEvent(); // force a signal on the event!
    }

	void wakeUp(void)
	{
		mBusy->setEvent(); // force a signal on the event!
	}

  private:

	  void wait(void); // wait until we are signalled again.


	  hacd::HaI32				*mActiveThreadCount;	// shared integer for the number of threads currently active doing work.
    bool						mExit;					// exit condition
    THREAD_CONFIG::Thread		*mThread;
    THREAD_CONFIG::ThreadEvent	*mBusy;
    JobScheduler				*mJobScheduler;    // provides new jobs to perform
    LOCK_FREE_Q::CQueue< SwarmJob * > mFinished;     // jobs that have been completed and may be reported back to the application.
  };

  class JobScheduler : public JobSwarmContext, public UANS::UserAllocated
  {
  public:

    JobScheduler(hacd::HaI32 maxThreadCount)
    {
		mActiveThreadCount = 0;
		mUseThreads = true;
		maxThreadCount = maxThreadCount;
		mPending = LOCK_FREE_Q::createLockFreeQ();
		mJobs.Set(100,100,100000000,"JobScheduler->mJobs",__FILE__,__LINE__);
		mActiveThreadCount = maxThreadCount; // initial active thread count; until each thread puts itself back to sleep
		mMaxThreadCount = maxThreadCount;
		mThreads = HACD_NEW(ThreadWorker)[mMaxThreadCount]; // the number of worker threads....
		for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
		{
			mThreads[i].setJobScheduler(this,&mActiveThreadCount);
		}
	}

	~JobScheduler(void)
	{
		delete []mThreads;
		LOCK_FREE_Q::releaseLockFreeQ(mPending);
	}

	// Happens in main thread
	SwarmJob * createSwarmJob(JobSwarmInterface *tji,void *userData,hacd::HaI32 userId)
	{
		SwarmJob *vret[2] = { mJobs.GetFreeLink() , mJobs.GetFreeLink() };
		//
		SwarmJob *ret=0;
		if( vret[0]==mPending->getHead() )
		{
			ret=vret[1];
			mJobs.Release( vret[0] );
		}
		else
		{
			ret=vret[0];
			mJobs.Release( vret[1] );
		}

		//
		new ( ret ) SwarmJob(tji,userData,userId);
//		printf("Creating job\r\n");
		mPending->enqueue(ret);
		wakeUpThreads(); // if the worker threads are aslpeep; wake them up to process this job

		//
		return ret;
	}

	// Empty the finished list.  This happens in the main application thread!
	bool processSwarmJobs(void)
	{
		bool completion = true;

		THREAD_CONFIG::tc_sleep(0); // give up timeslice to threads

		while ( completion )
		{
			completion = false;
			for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
			{
				SwarmJob *job = mThreads[i].getFinished();
				if ( job )
				{
					completion = true;
//					printf("Notify job complete.\r\n");
					job->notifyCompletion();
					mJobs.Release(job);
				}
			}
		}
		if ( !mUseThreads )
		{
			hacd::HaU32 stime = THREAD_CONFIG::tc_timeGetTime();
			bool working = true;
			while ( working )
			{
				SwarmJob *job = static_cast< SwarmJob *>(mPending->dequeue());
				if ( job )
				{
					if ( !job->isCancelled() )
					{
						job->onExecute();
					}
					job->notifyCompletion();
					mJobs.Release(job);
				}
				else
				{
					working = false;
				}
				hacd::HaU32 etime = THREAD_CONFIG::tc_timeGetTime();
				hacd::HaU32 dtime = etime - stime;
				if ( dtime > 30 ) break;
			}
		}
		if ( mJobs.GetUsedCount() != 0 )
		{
			wakeUpThreads(); // if there is work to be done, then wake up the threads
		}
		return mJobs.GetUsedCount() ? true : false;
	}

	// called from other thread to get a new job to perform
	SwarmJob * getJob(void)
	{
		SwarmJob *ret = 0;
		if ( mUseThreads )
		{
			ret = static_cast< SwarmJob *>(mPending->dequeue());
		}
		return ret;
	}

	// called from main thread..tags a job as canceled, callbacks will still occur!!
	void cancel(SwarmJob *job)
	{
		job->cancel();
	}

	void setUseThreads(bool state)
	{
		mUseThreads = state;
	}

	void wakeUpThreads(void)
	{
		if ( mActiveThreadCount != (hacd::HaI32)mMaxThreadCount )
		{
			for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
			{
				mThreads[i].wakeUp();
			}
		}
	}

  private:

	  
    bool                    mUseThreads;
    hacd::HaU32            mMaxThreadCount;
    LOCK_FREE_Q::LockFreeQ *mPending;
    Pool< SwarmJob >       mJobs;
    ThreadWorker           *mThreads;
	hacd::HaI32				mActiveThreadCount;


  };


  void ThreadWorker::setJobScheduler(JobScheduler *job,hacd::HaI32 *activeThreadCount)
  {
	mActiveThreadCount = activeThreadCount;
	mJobScheduler = job;
	mBusy         = THREAD_CONFIG::tc_createThreadEvent();
	mThread       = THREAD_CONFIG::tc_createThread(this);
  }

	void ThreadWorker::threadMain(void)
	{
		while ( !mExit )
		{
			if ( !mFinished.isFull() )
			{
				SwarmJob *job = mJobScheduler->getJob(); // get a new job to perform.
				if ( job )
				{
					//printf("Found job to work on.\r\n");
					job->onExecute();              // execute the job.
					//printf("Finished job, putting it on the finished queue\r\n");
					mFinished.push(job);
				}
				else
				{
					//printf("No work to be done; going to sleep\r\n");
					wait();
					//printf("Thread is woken back up.\r\n");
				}
			}
			else
			{
				wait();
			}
		}
	}

	void ThreadWorker::wait(void)
	{
		THREAD_CONFIG::tc_atomicAdd(mActiveThreadCount,-1); // subtract one from the active thread count, to indicate that we are now asleep waiting for more work.
		mBusy->resetEvent();
		mBusy->waitForSingleObject(0xFFFFFFFF);
		THREAD_CONFIG::tc_atomicAdd(mActiveThreadCount,1); // subtract one from the active thread count, to indicate that we are now asleep waiting for more work.
	}



  JobSwarmContext * createJobSwarmContext(hacd::HaU32 maxThreads)
  {
    JobScheduler *tjf = HACD_NEW(JobScheduler)(maxThreads);
    JobSwarmContext *ret = static_cast< JobSwarmContext *>(tjf);
    return ret;
  }

  bool releaseJobSwarmContext(JobSwarmContext *tc)
  {
    bool ret = false;
    if ( tc )
    {
      JobScheduler *tjf = static_cast< JobScheduler *>(tc);
      delete tjf;
      ret = true;
    }
    return ret;
  }

}; // END OF NAMESPACE
