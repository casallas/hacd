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


#ifndef __DG_WORKER_TREAD_H__
#define __DG_WORKER_TREAD_H__

#include "dgTypes.h"

typedef void (*dgWorkerThreadTaskCallback) (void* const context, hacd::HaI32 threadID);


class dgWorkerThread : public UANS::UserAllocated
{
	class dgWorkerThreadJob
	{
		public:
		//void* m_userParamArray[DG_MAX_THREADS_HIVE_PARAMETERS];
		void* m_context;
		dgWorkerThreadTaskCallback m_callback;
	};

	public:

	dgWorkerThread();
	~dgWorkerThread(void);

	void Init (hacd::HaI32 id, hacd::HaU32* const jobQueueCompletedSignal);
	void QueueJob (dgWorkerThreadTaskCallback callback, void* const context);

	void ExecuteJobs ();
	bool IsThreadBusy () const {return m_jobsCount ? true : false;}
	void SetPerfomanceCounter(OnGetPerformanceCountCallback callback);

	private:
	void TaskExecuter();
	void RunAllMicroTask();
	hacd::HaI32 GetNextJob(dgWorkerThreadJob& job);

	#if (defined (WIN32) || defined (WIN64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
		static unsigned _stdcall ThreadSystemCallback(void *param);
	#endif
	#if (defined (_LINUX_VER) || defined (_MAC_VER))
		static void* ThreadSystemCallback(void *Param);
	#endif


	dgWorkerThreadJob* m_queue;

	hacd::HaI32 m_threadId;
	hacd::HaI32 m_jobsCount;
	hacd::HaI32 m_jobsCapacity;
	hacd::HaI32 m_threaIsActive;
	hacd::HaI32 m_contineExecution;
	hacd::HaU32 m_ticks;
	hacd::HaU32* m_jobQueueCompletedSignal;
	
	OnGetPerformanceCountCallback m_getPerformanceCount;	

#if (defined (WIN32) || defined (WIN64) || defined (_MINGW_32_VER) || defined (_MINGW_64_VER))
	HANDLE m_jobsInQueueSemaphore;
	uintptr_t m_threadhandle;
#endif

#if (defined (_LINUX_VER) || defined (_MAC_VER))
	pthread_t m_threadhandle;
#endif

	friend class dgThreadHive;
};

#endif
