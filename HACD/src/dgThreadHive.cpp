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
	,m_threadsStillWorking(0)
	,m_globalSpinLock(0)
	,m_workerBee(NULL)
{
}

dgThreadHive::~dgThreadHive()
{
	DestroydgThreads();
}

void dgThreadHive::DestroydgThreads()
{
	if (m_numOfThreads) {
		delete[] m_workerBee;
		m_workerBee = NULL;
		m_numOfThreads = 0;
	}
}

void dgThreadHive::SetThreadsCount (hacd::HaI32 threads)
{
	DestroydgThreads();

	m_taskScheduler = 0;
	m_threadsStillWorking = 0;
	m_numOfThreads = GetMin (threads, DG_MAX_THREADS_HIVE_COUNT);
	if (m_numOfThreads == 1) {
		m_numOfThreads = 0;
	}

	if (m_numOfThreads) {
		m_workerBee = HACD_NEW(dgWorkerThread)[hacd::HaU32 (m_numOfThreads)];
		for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) {
			m_workerBee[i].Init(i, (hacd::HaU32*)&m_threadsStillWorking);
		}
	}
}

void dgThreadHive::SetPerfomanceCounter(OnGetPerformanceCountCallback callback)
{
	for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) {
		m_workerBee[i].SetPerfomanceCounter(callback);
	}
}

void dgThreadHive::ClearTimers()
{
	for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) {
		m_workerBee[i].m_ticks = 0;
	}
}

hacd::HaU32 dgThreadHive::GetPerfomanceTicks (hacd::HaU32 threadIndex) const
{
	if (m_numOfThreads) {
		if ((threadIndex >= 0) && (threadIndex < hacd::HaU32(m_numOfThreads))) {
			return m_workerBee[threadIndex].m_ticks;
		}
	}
	return 0;
}

hacd::HaI32 dgThreadHive::GetThreadCount() const
{
	return m_numOfThreads ? m_numOfThreads : 1;
}

hacd::HaI32 dgThreadHive::GetMaxThreadCount() const
{
	return DG_MAX_THREADS_HIVE_COUNT;
}

void dgThreadHive::GetIndirectLock(hacd::HaI32* const lockVar, hacd::HaU32 threadIndex) const
{
	HACD_ASSERT (hacd::HaI32 (sizeof (hacd::HaI32)) == hacd::HaI32 (sizeof (long)));
	#ifdef DG_SOFTWARE_THREAD_EMULATION
		*lockVar = 1;
	#else 
		if (m_numOfThreads) {
			hacd::HaU32 timeYielded = 0;
			while (dgInterlockedExchange(lockVar, 1)) {
				if (threadIndex != hacd::HaU32(-1)) {
					hacd::HaU32 ticks = GetPerfomanceTicks(threadIndex);
					dgThreadYield();
					timeYielded += GetPerfomanceTicks(threadIndex) - ticks;
				} else {
					dgThreadYield();
				}
			}
			if (threadIndex != hacd::HaU32(-1)) {
				m_workerBee[threadIndex].m_ticks -= timeYielded;	
			}
		}
	#endif
}

void dgThreadHive::ReleaseIndirectLock(hacd::HaI32* const lockVar) const
{
	HACD_ASSERT (hacd::HaI32 (sizeof (hacd::HaI32)) == hacd::HaI32 (sizeof (long)));
	//	dgInterlockedExchange(lockVar, 0);
	*lockVar = 0;
}


void dgThreadHive::GetLock(hacd::HaU32 threadIndex) const
{
	GetIndirectLock(&m_globalSpinLock, threadIndex);
}

void dgThreadHive::ReleaseLock() const
{	
	ReleaseIndirectLock(&m_globalSpinLock);
}


/*
void dgThreadHive::QueueJob (dgWorkerThreadTaskCallback callback, void** const userParamArray, hacd::HaI32 paramCount)
{
	if (!m_numOfThreads) {
		callback (userParamArray, 0);
	} else {
		m_workerBee[m_taskScheduler].QueueJob (callback, userParamArray, paramCount);
		m_taskScheduler = (m_taskScheduler + 1) % m_numOfThreads;
	}
}
*/

void dgThreadHive::QueueJob (dgWorkerThreadTaskCallback callback, void* const context)
{
	if (!m_numOfThreads) {
		callback (context, 0);
	} else {
		m_workerBee[m_taskScheduler].QueueJob (callback, context);
		m_taskScheduler = (m_taskScheduler + 1) % m_numOfThreads;
	}
}


void dgThreadHive::SyncThreads(hacd::HaI32* const threads) const
{
	if (m_numOfThreads) {
		#ifdef DG_SOFTWARE_THREAD_EMULATION
			*threads = *threads + 1;
		#else 
			dgAtomicAdd(threads, 1);
			while (*threads < m_numOfThreads) {
				dgThreadYield();
			}	
		#endif
	} else {
		*threads = *threads + 1;
	}
}


void dgThreadHive::SynchronizationBarrier ()
{
	if (m_numOfThreads) {
		m_threadsStillWorking = 0;
		for (hacd::HaI32 i = 0; i < m_numOfThreads; i ++) {
#ifdef DG_SOFTWARE_THREAD_EMULATION
			m_workerBee[i].RunAllMicroTask();
#else
			m_workerBee[i].ExecuteJobs();
#endif
		}

#ifndef DG_SOFTWARE_THREAD_EMULATION
		while (m_threadsStillWorking < m_numOfThreads) {
			dgThreadYield();
		}
#endif
	}
}



