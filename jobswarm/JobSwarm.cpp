#include "JobSwarm.h"
#include "dgThreadHive.h"

#pragma warning(disable:4100)

namespace JOB_SWARM
{

void dgWorkerThreadTaskCallback(void* const context, hacd::HaI32 threadID);

class SwarmJob : public UANS::UserAllocated
{
public:
	SwarmJob(JobSwarmInterface *iface,void *userData,hacd::HaI32 userId,dgThreadHive &hive)
	{
		mInterface = iface;
		mUserData = userData;
		mUserId = userId;
		hive.QueueJob(dgWorkerThreadTaskCallback,this);
	}

	~SwarmJob(void)
	{
	}

	void runJob(void)
	{
		mInterface->job_process(mUserData,mUserId);
	}

	JobSwarmInterface	*mInterface;
	void				*mUserData;
	hacd::HaI32			mUserId;
};

void dgWorkerThreadTaskCallback(void* const context, hacd::HaI32 threadID)
{
	SwarmJob *s = (SwarmJob *)context;
	s->runJob();
	delete s;
}


	// This is a single instance of a JobSwarm system.  You can have multiple JobSwarmContexts in a single application.
class MyJobSwarmContext : public JobSwarmContext, public UANS::UserAllocated
{
public:
	MyJobSwarmContext(hacd::HaU32 maxThreadCount)
	{
		mThreadHive.SetThreadsCount(maxThreadCount);
	}

	virtual void createSwarmJob(JobSwarmInterface *iface,void *userData,hacd::HaI32 userId) // creates a job to be processed and returns a handle.
	{
		HACD_NEW(SwarmJob)(iface,userData,userId,mThreadHive);
	}

	virtual bool processSwarmJobs(void) // This is a pump loop run in the main thread to handle the disposition of finished and/or cancelled jobs.  Returns true if there are still outstanding jobs not yet full procesed.
	{
		return mThreadHive.ProcessWorkerThreads(); // return true if there are still jobs pending...
	}

	dgThreadHive	mThreadHive;
};



JobSwarmContext * createJobSwarmContext(hacd::HaU32 maxThreadCount) // create a JobSwarmContext with the give number of physical threads
{
	MyJobSwarmContext *m = HACD_NEW(MyJobSwarmContext)(maxThreadCount);
	return static_cast<  JobSwarmContext *>(m);
}
bool              releaseJobSwarmContext(JobSwarmContext *c) // release a JobSwarmContet
{
	bool ret = false;
	if ( c )
	{
		ret = true;
		MyJobSwarmContext *m = static_cast< MyJobSwarmContext *>(c);
		delete m;
	}
	return ret;
}


}; // end of JOB_SWARM namespace