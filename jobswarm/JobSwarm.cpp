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

#pragma warning(disable:4996)

#define MAX_THREADS 64

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
		HACD_ASSERT(mInterface);
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
		mJobScheduler = 0;
		mExit         = false;
		mThread     = 0;
		mWaiting = true;
		mWaitPending = false;
		mExitComplete = false;
		mThreadFlag = 0;
		mFinished.init(MAX_COMPLETION);
	}

	~ThreadWorker(void)
	{
		release();
	}

	void release(void)
	{
		setExit();
		while ( !mExitComplete )
		{
			THREAD_CONFIG::tc_sleep(0);
		}
		THREAD_CONFIG::tc_releaseThread(mThread);
		mThread = 0;
		mWaiting = true;
		mWaitPending = false;
	}

	void setJobScheduler(JobScheduler *job,hacd::HaU32 threadFlag);

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
		wakeUp();
	}

	bool wakeUp(void)
	{
		bool ret = mWaiting;

		if ( mWaiting )
		{
			mWaiting = false; // no longer waiting, we have woken up the thread
			mThread->Resume();
		}

		return ret;
	}

	bool processWaitPending(void);

  private:

	void wait(void); // wait until we are signalled again.


	hacd::HaU32					mThreadFlag;	// bit flag identifying this thread
	bool						mWaitPending;
	bool						mWaiting;
	bool						mExit;					// exit condition
	bool						mExitComplete;
	THREAD_CONFIG::Thread		*mThread;
	JobScheduler				*mJobScheduler;			// provides new jobs to perform
	LOCK_FREE_Q::CQueue< SwarmJob * > mFinished;		// jobs that have been completed and may be reported back to the application.
};

  class JobScheduler : public JobSwarmContext, public UANS::UserAllocated
  {
  public:

    JobScheduler(hacd::HaI32 maxThreadCount)
    {
		if ( maxThreadCount > MAX_THREADS )
		{
			maxThreadCount = MAX_THREADS;
		}
		mPendingCount = 0;
		mUseThreads = true;
		maxThreadCount = maxThreadCount;
		mPending = LOCK_FREE_Q::createLockFreeQ();
		mJobs.Set(100,100,100000000,"JobScheduler->mJobs",__FILE__,__LINE__);
		mMaxThreadCount = maxThreadCount;
		mThreads = HACD_NEW(ThreadWorker)[mMaxThreadCount]; // the number of worker threads....
		mPendingSleepCount = 0;
		mThreadAwakeCount = 0;
		for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
		{
			mThreadAsleep[i] = 1;
			mPendingSleep[i] = 0;
			mThreads[i].setJobScheduler(this,i);
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
		SwarmJob *ret = mJobs.GetFreeLink();
		//
		new ( ret ) SwarmJob(tji,userData,userId);
		mPending->enqueue(ret);
		THREAD_CONFIG::tc_atomicAdd(&mPendingCount,1);
		wakeUpThreads(); // if the worker threads are aslpeep; wake them up to process this job
		return ret;
	}

	// Empty the finished list.  This happens in the main application thread!
	bool processSwarmJobs(void)
	{
		bool completion = true;

		THREAD_CONFIG::tc_sleep(0); // give up timeslice to threads
		applySleep(); // if there are any threads waiting to go to sleep; let's put them to sleep
		wakeUpThreads(); // if there is work to be done, then wake up the threads
		// de-queue all completed jobs in the main thread.
		while ( completion )
		{
			completion = false;
			for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
			{
				SwarmJob *job = mThreads[i].getFinished();
				if ( job )
				{
					completion = true;
					job->notifyCompletion();
					mJobs.Release(job);
				}
			}
		}
#if 0
		// do up to one job in the main thread each time this is called.
		if ( mJobs.GetUsedCount() )
		{
			SwarmJob *job = getJob();
			if ( job )
			{
				job->onExecute();
				job->notifyCompletion();
				mJobs.Release(job);
			}
		}
#endif
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
		return mJobs.GetUsedCount() ? true : false;
	}

	// called from other thread to get a new job to perform
	SwarmJob * getJob(void)
	{
		SwarmJob *ret = 0;
		if ( mUseThreads )
		{
			ret = static_cast< SwarmJob *>(mPending->dequeue());
			if ( ret )
			{
				THREAD_CONFIG::tc_atomicAdd(&mPendingCount,-1);
			}
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
		hacd::HaU32 jobsWaiting = mPendingCount;

		if ( jobsWaiting ) // if there are any threads not currently running jobs...
		{
			for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
			{
				if ( mThreadAsleep[i] )
				{
					if ( mThreads[i].wakeUp() ) // enable the signal on this thread to wake it up to process jobs
					{
						mThreadAwakeCount++;
						mThreadAsleep[i] = 0;
						jobsWaiting--;
						if ( jobsWaiting == 0 )  // if we have woken up enough threads to consume the pending jobs
							break;
					}
					else
					{
						HACD_ALWAYS_ASSERT(); // should always be able to wake it up!
					}
				}
			}
		}
	}

	void putToSleep(hacd::HaU32 threadIndex)
	{
		HACD_ASSERT( mPendingSleep[threadIndex] == false );
		THREAD_CONFIG::tc_atomicAdd(&mPendingSleep[threadIndex],1);
		THREAD_CONFIG::tc_atomicAdd(&mPendingSleepCount,1);
	}

	// always called from the main thread.  puts to sleep any treads that had no work to do.
	void applySleep(void)
	{
		if ( mPendingSleepCount ) // if there are any threads pending to be put to sleep...
		{
			for (hacd::HaU32 i=0; i<mMaxThreadCount; i++)
			{
				if ( mPendingSleep[i] )
				{
					mThreadAwakeCount--;
					mThreadAsleep[i] = 1; // mark it as actually being asleep now.
					THREAD_CONFIG::tc_atomicAdd(&mPendingSleep[i],-1);
					THREAD_CONFIG::tc_atomicAdd(&mPendingSleepCount,-1);
					mThreads[i].processWaitPending();
				}
			}
//			HACD_ASSERT(mPendingSleepCount==0);
		}
	}

private:
	bool					mUseThreads;
	hacd::HaU32				mMaxThreadCount;
	LOCK_FREE_Q::LockFreeQ *mPending;
	Pool< SwarmJob >		mJobs;
	ThreadWorker			*mThreads;
	hacd::HaI32				mPendingCount;
	hacd::HaU32				mWaitPending;
	hacd::HaI32				mPendingSleepCount;
	hacd::HaI32				mThreadAwakeCount;
	hacd::HaI32				mPendingSleep[MAX_THREADS];
	hacd::HaI32				mThreadAsleep[MAX_THREADS];
};


void ThreadWorker::setJobScheduler(JobScheduler *job,hacd::HaU32 threadFlag)
{
	mThreadFlag = threadFlag;
	mJobScheduler = job;
	mWaiting	= true; // threads begin in a suspended state!
	mThread		= THREAD_CONFIG::tc_createThread(this);
}

void ThreadWorker::threadMain(void)
{
	while ( !mExit )
	{
		if ( mWaitPending || mWaiting )
		{
			// already asleep, or waiting to be put to sleep by the main thread...
			THREAD_CONFIG::tc_sleep(0);
		}
		else
		{
			if ( !mFinished.isFull() )
			{
				SwarmJob *job = mJobScheduler->getJob(); // get a new job to perform.
				while ( job )
				{
					job->onExecute();              // execute the job.
					mFinished.push(job);
					job = mJobScheduler->getJob(); // get a new job to perform.
				}
				wait();
			}
			else
			{
				wait();
			}
		}
	}
	mExitComplete = true;
}

void ThreadWorker::wait(void)
{
	HACD_ASSERT(!mWaitPending);
	HACD_ASSERT(!mWaiting);
	mWaitPending = true;
	mJobScheduler->putToSleep(mThreadFlag);
}

bool ThreadWorker::processWaitPending(void)
{
	bool ret = mWaitPending;
	HACD_ASSERT(mWaiting==false);
	HACD_ASSERT(mWaitPending);
	if ( mWaitPending )
	{
		if ( mWaiting == false )
		{
			mThread->Suspend(); // suspend thread execution
			mWaiting = true; // indicate that we are currently waiting to be freshly signalled
		}
		mWaitPending = false;
	}
	return ret;
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
