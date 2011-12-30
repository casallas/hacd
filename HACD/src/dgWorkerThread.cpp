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
#include "dgWorkerThread.h"

#if defined(WIN32) || defined(WIN64)
#include <process.h>
#endif

#pragma warning(disable:4996)

dgWorkerThread::dgWorkerThread()
	:m_queue(NULL)
	,m_threadId(0)
	,m_jobsCount(0)
	,m_jobsCapacity(64)
	,m_threaIsActive(0)
	,m_contineExecution(0)
	,m_ticks(0)
	,m_jobQueueCompletedSignal(NULL)
	,m_jobsInQueueSemaphore(NULL)
	,m_threadhandle(NULL)
	,m_getPerformanceCount(NULL)
{
}

dgWorkerThread::~dgWorkerThread(void)
{
	if (m_queue) {
		dgInterlockedExchange(&m_contineExecution, 0);
		ReleaseSemaphore (m_jobsInQueueSemaphore, 1, NULL);
		while (m_threaIsActive) {
			dgThreadYield();
		}
		Sleep (10);

		HACD_FREE(m_queue);  
	}
}



#if (defined (WIN32) || defined (WIN64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	#ifdef _DEBUG 
	#define MS_VC_EXCEPTION 0x406D1388 

	#pragma pack(push,8) 
	typedef struct tagTHREADNAME_INFO 
	{ 
		DWORD dwType; // Must be 0x1000. 
		LPCSTR szName; // Pointer to name (in user addr space). 
		DWORD dwThreadID; // Thread ID (-1=caller thread). 
		DWORD dwFlags; // Reserved for future use, must be zero. 
	} THREADNAME_INFO; 
	#pragma pack(pop) 

	void SetThreadName( DWORD dwThreadID, char* threadName) 
	{ 
		THREADNAME_INFO info; 
		info.dwType = 0x1000; 
		info.szName = threadName; 
		info.dwThreadID = dwThreadID; 
		info.dwFlags = 0; 

		__try
		{ 
			RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info ); 
		} 
		__except(EXCEPTION_EXECUTE_HANDLER) 
		{ 
			// printf("Not under debugger. Swallowing exception"); 
		} 
	}
	#endif
#endif


void dgWorkerThread::Init (hacd::HaI32 id, hacd::HaU32* const jobQueueCompletedSignal)
{
	HACD_ASSERT (!m_queue);
	
	m_jobsCount = 0;
	m_threadId = id;
	m_jobQueueCompletedSignal = jobQueueCompletedSignal;
	if (m_jobQueueCompletedSignal) {
		dgInterlockedExchange ((hacd::HaI32*) m_jobQueueCompletedSignal, 0); 
	}
	m_queue = (dgWorkerThreadJob*) HACD_ALLOC(hacd::HaI32 (m_jobsCapacity * sizeof (dgWorkerThreadJob)));  
	
#if (defined (WIN32) || defined (WIN64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	m_jobsInQueueSemaphore = CreateSemaphore(NULL, 0, 0x7fffffff, NULL);
	m_threadhandle = _beginthreadex( NULL, 0, ThreadSystemCallback, this, 0, NULL);
	#ifdef _DEBUG
		char name[256];
		sprintf (name, "physics_%d", id);
		SetThreadName (GetThreadId(HANDLE (m_threadhandle)), name);
	#endif
#endif

#if (defined (_LINUX_VER) || defined (_MAC_VER))
	pthread_create (&m_threadhandle, NULL, ThreadSystemCallback, this);
#endif

	while (!m_threaIsActive) {
		dgThreadYield();
	}
}


void dgWorkerThread::SetPerfomanceCounter(OnGetPerformanceCountCallback callback)
{
	m_getPerformanceCount = callback;
}



#if (defined (WIN32) || defined (WIN64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
unsigned _stdcall dgWorkerThread::ThreadSystemCallback(void *param)
#endif
#if (defined (_LINUX_VER) || defined (_MAC_VER))
void* dgWorkerThreadOld::ThreadSystemCallback(void *param)
#endif
{
	#if (defined (WIN32) || defined (WIN64))
		#ifndef __USE_DOUBLE_PRECISION__
			hacd::HaU32 controlWorld = dgControlFP (0xffffffff, 0);
			dgControlFP (_PC_53, _MCW_PC);
		#endif
	#endif

	dgWorkerThread* const me = (dgWorkerThread*) param;
	me->TaskExecuter();

	#if (defined (WIN32) || defined (WIN64))
		#ifndef __USE_DOUBLE_PRECISION__
			dgControlFP (controlWorld, _MCW_PC);
		#endif
	#endif

	return 0;
}

void dgWorkerThread::RunAllMicroTask()
{
	if (m_getPerformanceCount)	{
		for (hacd::HaI32 i = 0; i < m_jobsCount; i ++) {
			dgWorkerThreadJob* const job = &m_queue[i];

			hacd::HaU32 ticks = m_getPerformanceCount();
			//job->m_callback (&job->m_userParamArray[0], m_threadId);
			job->m_callback (job->m_context, m_threadId);
			m_ticks += (m_getPerformanceCount() - ticks);
		}
	} else {
		for (hacd::HaI32 i = 0; i < m_jobsCount; i ++) {
			dgWorkerThreadJob* const job = &m_queue[i];
			//job->m_callback (&job->m_userParamArray[0], m_threadId);
			job->m_callback (job->m_context, m_threadId);
		}
	}
	m_jobsCount = 0;

	if (m_jobQueueCompletedSignal) {
		dgAtomicAdd ((hacd::HaI32*) m_jobQueueCompletedSignal, 1); 
	}
}


void dgWorkerThread::TaskExecuter()
{
	dgAtomicAdd(&m_threaIsActive, 1);
	dgInterlockedExchange(&m_contineExecution, 1);
	while (m_contineExecution) {
		DWORD waitCode;
		waitCode = WaitForSingleObject(m_jobsInQueueSemaphore, INFINITE);
		HACD_ASSERT (waitCode == WAIT_OBJECT_0); 
		RunAllMicroTask();
	}

	dgAtomicAdd(&m_threaIsActive, -1);
}

/*
void dgWorkerThread::QueueJob (dgWorkerThreadTaskCallback callback, void** const userParamArray, hacd::HaI32 paramCount)
{
	if (m_jobsCount == m_jobsCapacity) {
		dgWorkerThreadJob* const newQueue = (dgWorkerThreadJob*) m_allocator->MallocLow (hacd::HaI32 (2 * m_jobsCapacity * sizeof (dgWorkerThreadJob)));  
		memcpy (newQueue, m_queue, m_jobsCapacity * sizeof (dgWorkerThreadJob));
		m_allocator->FreeLow(m_queue); 
		m_queue = newQueue;
		m_jobsCapacity *= 2;
	}

	dgWorkerThreadJob* const job = &m_queue[m_jobsCount];
	job->m_callback = callback;
	for (hacd::HaI32 i = 0; i < paramCount; i ++) {
		job->m_userParamArray[i] = userParamArray[i];
	}
	m_jobsCount ++;
}
*/

void dgWorkerThread::QueueJob (dgWorkerThreadTaskCallback callback, void* const context)
{
	if (m_jobsCount == m_jobsCapacity) {
		dgWorkerThreadJob* const newQueue = (dgWorkerThreadJob*) HACD_ALLOC(hacd::HaI32 (2 * m_jobsCapacity * sizeof (dgWorkerThreadJob)));  
		memcpy (newQueue, m_queue, m_jobsCapacity * sizeof (dgWorkerThreadJob));
		HACD_FREE(m_queue); 
		m_queue = newQueue;
		m_jobsCapacity *= 2;
	}

	dgWorkerThreadJob* const job = &m_queue[m_jobsCount];
	job->m_context = context;
	job->m_callback = callback;
	m_jobsCount ++;
}


void dgWorkerThread::ExecuteJobs ()
{
	if (m_jobsCount) {
		ReleaseSemaphore (m_jobsInQueueSemaphore, 1, NULL);
	}
}



