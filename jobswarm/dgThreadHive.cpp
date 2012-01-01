/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgTypes.h"
#include "dgThreadHive.h"


//#define	DG_SOFTWARE_THREAD_EMULATION	


dgThreadHive::dgThreadHive(void)
	:m_numOfThreads(0)
	,m_taskScheduler(0)
	,m_workerBee(NULL)
{
}

dgThreadHive::~dgThreadHive()
{
	DestroydgThreads();
}

void dgThreadHive::DestroydgThreads()
{
	if (m_numOfThreads) 
	{
		delete[] m_workerBee;
		m_workerBee = NULL;
		m_numOfThreads = 0;
	}
}

void dgThreadHive::SetThreadsCount (hacd::HaI32 threads)
{
	DestroydgThreads();
	m_taskScheduler = 0;
	m_numOfThreads = GetMin (threads, DG_MAX_THREADS_HIVE_COUNT);
	if (m_numOfThreads == 1) 
	{
		m_numOfThreads = 0;
	}
	if (m_numOfThreads) 
	{
		m_workerBee = HACD_NEW(dgWorkerThread)[m_numOfThreads];
		for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) 
		{
			m_workerBee[i].Init(i);
		}
	}
}

hacd::HaI32 dgThreadHive::GetThreadCount() const
{
	return m_numOfThreads ? m_numOfThreads : 1;
}

hacd::HaI32 dgThreadHive::GetMaxThreadCount() const
{
	return DG_MAX_THREADS_HIVE_COUNT;
}

void dgThreadHive::QueueJob(dgWorkerThreadTaskCallback callback, void* const context)
{
	if (!m_numOfThreads) 
	{
		callback (context, 0);
	} 
	else 
	{
		m_workerBee[m_taskScheduler].QueueJob(callback, context);
		m_taskScheduler = (m_taskScheduler + 1) % m_numOfThreads;
	}
}


bool dgThreadHive::ProcessWorkerThreads(void)
{
	bool ret = false;

	if (m_numOfThreads) 
	{
		for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) 
		{
			if ( m_workerBee[i].IsThreadBusy() ) // if this thread has any work outstanding...
			{
				#ifdef DG_SOFTWARE_THREAD_EMULATION
				m_workerBee[i].RunAllMicroTask();
				#else
				m_workerBee[i].ExecuteJobs();
				#endif
				ret = true;
			}
		}
		dgThreadYield(); // give up a litte bit of time to the threads.
	}

	return ret;
}
