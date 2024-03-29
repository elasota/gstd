/*
Copyright (c) 2024 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#define _CRT_SECURE_NO_WARNINGS
#include "zstd.h"
#include "zstdhl.h"
#include "gstdenc.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <thread>
#include <mutex>
#include <vector>
#include <limits>

struct ThreadedTaskWorkerState;
struct SerializedTaskGlobalState;
struct SerializedTaskWorkerState;

extern void DecompressGstdCPU32(const void *inData, uint32_t inSize, void *outData, uint32_t outCapacity, void *warnContext, void (*warnCallback)(void *, const char *));
extern "C" uint32_t crc32(uint32_t crc, const void *buf, size_t len);

class AutoResetEvent
{
public:
	AutoResetEvent();
	explicit AutoResetEvent(bool startSignalled);

	void WaitFor();
	void Signal();

private:
	std::mutex m_mutex;
	std::condition_variable m_cv;

	bool m_signalled;
	size_t m_numWaiting;
	size_t m_numReleasable;
};

class Semaphore
{
public:
	Semaphore();
	explicit Semaphore(size_t initialCount);

	void Acquire();
	void Release();
	void Release(size_t count);

private:
	std::mutex m_mutex;
	std::condition_variable m_cv;
	size_t m_count;
};

class ThreadedTaskBase
{
public:
	virtual ~ThreadedTaskBase();

	virtual void RunWorkUnit(size_t workUnit) = 0;
	virtual void FinishWritingWorkUnit() = 0;
};

AutoResetEvent::AutoResetEvent()
	: m_signalled(false), m_numWaiting(0), m_numReleasable(0)
{
}

AutoResetEvent::AutoResetEvent(bool startSignalled)
	: m_signalled(startSignalled), m_numWaiting(0), m_numReleasable(0)
{
}

void AutoResetEvent::WaitFor()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	if (!m_signalled)
	{
		while (!m_signalled)
			m_cv.wait(lock);

		m_signalled = false;
	}
}

void AutoResetEvent::Signal()
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_signalled = true;
	}

	m_cv.notify_one();
}

Semaphore::Semaphore()
	: m_count(0)
{
}

Semaphore::Semaphore(size_t initialCount)
	: m_count(initialCount)
{
}

void Semaphore::Acquire()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	while (m_count == 0)
		m_cv.wait(lock);

	m_count--;
}

void Semaphore::Release()
{
	Release(1);
}

void Semaphore::Release(size_t count)
{
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_count += count;
	}

	for (size_t i = 0; i < count; i++)
		m_cv.notify_one();
}

struct SerializedTaskGlobalState
{
	explicit SerializedTaskGlobalState(size_t numTasks, size_t numWorkers);
	~SerializedTaskGlobalState();

	void SetTaskRunner(size_t index, ThreadedTaskBase *taskRunner);
	void RunThread(size_t index);
	void WaitForCompletion(size_t index);

	SerializedTaskWorkerState *m_workers;
	size_t m_numWorkers;
	size_t m_numWorkersActive;

	size_t m_numTasks;

	size_t m_tasksCompleted;
	size_t m_tasksStarted;

	std::mutex m_taskQueueMutex;

	AutoResetEvent m_completionEvent;
	Semaphore m_kickSemaphore;
	Semaphore m_finishWorkSemaphore;

private:
	struct TaskBucket
	{
		SerializedTaskWorkerState *m_workerAssignedToTask;
		bool m_isFinished;
	};

	TaskBucket *m_taskQueue;

	void TryAssigningWorkToWorkerUnderLock(size_t workerIndex);
};

struct SerializedTaskWorkerState
{
	SerializedTaskWorkerState();

	bool m_started;
	bool m_terminated;
	size_t m_workUnitIndex;

	SerializedTaskGlobalState *m_globalState;
	ThreadedTaskBase *m_taskRunner;
	AutoResetEvent m_finalizeEvent;
};


SerializedTaskGlobalState::SerializedTaskGlobalState(size_t numTasks, size_t numWorkers)
	: m_numTasks(numTasks), m_numWorkers(numWorkers), m_tasksStarted(0), m_tasksCompleted(0), m_numWorkersActive(0)
{
	m_workers = new SerializedTaskWorkerState[numWorkers];
	m_taskQueue = new TaskBucket[numWorkers];

	for (size_t i = 0; i < numWorkers; i++)
	{
		m_workers[i].m_globalState = this;
		m_taskQueue[i].m_isFinished = false;
		m_taskQueue[i].m_workerAssignedToTask = nullptr;
	}

	m_kickSemaphore.Release(numWorkers);
}

SerializedTaskGlobalState::~SerializedTaskGlobalState()
{
	delete[] m_workers;
	delete[] m_taskQueue;
}

void SerializedTaskGlobalState::SetTaskRunner(size_t index, ThreadedTaskBase *taskRunner)
{
	m_workers[index].m_taskRunner = taskRunner;
}

void SerializedTaskGlobalState::RunThread(size_t workerIndex)
{
	SerializedTaskWorkerState *workerState = m_workers + workerIndex;

	{
		std::lock_guard<std::mutex> lock(m_taskQueueMutex);

		workerState->m_started = true;
		m_numWorkersActive++;
	}

	for (;;)
	{
		m_kickSemaphore.Acquire();

		size_t thisWorkUnit = 0;

		{
			std::lock_guard<std::mutex> lock(m_taskQueueMutex);
			if (m_tasksStarted == m_numTasks || workerState->m_terminated)
			{
				m_kickSemaphore.Release();
				break;
			}

			thisWorkUnit = m_tasksStarted++;

			TaskBucket &bucket = m_taskQueue[thisWorkUnit - m_tasksCompleted];
			bucket.m_isFinished = false;
			bucket.m_workerAssignedToTask = workerState;
		}

		workerState->m_taskRunner->RunWorkUnit(thisWorkUnit);

		bool shouldWaitForCompletionKick = false;

		{
			std::lock_guard<std::mutex> lock(m_taskQueueMutex);
			size_t relativeIndex = thisWorkUnit - m_tasksCompleted;

			m_taskQueue[relativeIndex].m_isFinished = true;

			shouldWaitForCompletionKick = (relativeIndex != 0);
		}

		if (shouldWaitForCompletionKick)
			workerState->m_finalizeEvent.WaitFor();

		workerState->m_taskRunner->FinishWritingWorkUnit();

		{
			std::lock_guard<std::mutex> lock(m_taskQueueMutex);

			m_tasksCompleted++;

			size_t numRemainingTasks = m_tasksStarted - m_tasksCompleted;

			for (size_t i = 0; i < numRemainingTasks; i++)
				m_taskQueue[i] = m_taskQueue[i + 1];

			if (m_taskQueue[0].m_isFinished)
				m_taskQueue[0].m_workerAssignedToTask->m_finalizeEvent.Signal();
		}

		m_kickSemaphore.Release();
	}

	bool isLastOut = false;

	{
		std::lock_guard<std::mutex> lock(m_taskQueueMutex);
		m_numWorkersActive--;

		isLastOut = (m_numWorkersActive == 0);
	}

	if (isLastOut)
		m_completionEvent.Signal();
}

void SerializedTaskGlobalState::WaitForCompletion(size_t index)
{
	size_t numWorkersActive = 0;

	{
		std::lock_guard<std::mutex> lock(m_taskQueueMutex);
		numWorkersActive = m_numWorkersActive;
	}

	if (numWorkersActive > 0)
		m_completionEvent.WaitFor();
}

// SerializedTaskWorkerState
SerializedTaskWorkerState::SerializedTaskWorkerState()
	: m_terminated(false), m_globalState(nullptr), m_taskRunner(nullptr), m_started(false), m_workUnitIndex(0)
{
}

ThreadedTaskBase::~ThreadedTaskBase()
{
}

void PrintUsageAndQuit()
{
	fprintf(stderr, "gstd - gstd command line tool\n");
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "    gstd <mode> <options> <input> <output>\n");
	fprintf(stderr, "Modes:\n");
	fprintf(stderr, "    c - Compresses input to output\n");
	fprintf(stderr, "    d - Decompresses input to output\n");
	fprintf(stderr, "Options:\n");
	fprintf(stderr, "    -pagesize <size> - Sets the size of a page (default is 65536 bytes)\n");
	fprintf(stderr, "    -level <level>   - Sets compression level (default is 9)\n");
	fprintf(stderr, "    -t <threads>     - Sets maximum thread count\n");
	fprintf(stderr, "    -f <path>        - Sets path to output failed blocks to (for debugging)\n");
	fprintf(stderr, "    -nofseshuffle    - Disables FSE table shuffling\n");

	exit(-1);
}

void DecompressWarn(void *context, const char *str)
{
	fprintf(stderr, "Decompressor threw warning for block %i: %s\n", *static_cast<int*>(context), str);
}

int DecompressMain(int optc, const char **optv, const char *inFileName, const char *outFileName)
{
	FILE *inF = fopen(inFileName, "rb");
	if (!inF)
	{
		fprintf(stderr, "Failed to open input file\n");
		return -1;
	}

	FILE *outF = fopen(outFileName, "wb");
	if (!inF)
	{
		fprintf(stderr, "Failed to open output file\n");
		return -1;
	}

	uint8_t sizeBytes[4];
	size_t bytesRead = fread(sizeBytes, 1, 4, inF);

	if (bytesRead != 4)
	{
		fprintf(stderr, "Failed to read page size");
		return -1;
	}

	uint32_t pageSize = sizeBytes[0] | (static_cast<uint32_t>(sizeBytes[1]) << 8) | (static_cast<uint32_t>(sizeBytes[2]) << 16) | (static_cast<uint32_t>(sizeBytes[3]) << 24);

	if (pageSize > 16 * 1024 * 1024)
	{
		fprintf(stderr, "Page size too large");
		return -1;
	}

	int blockIndex = 0;
	for (;;)
	{
		bytesRead = fread(sizeBytes, 1, 4, inF);

		if (bytesRead == 0)
			break;

		if (bytesRead != 4)
		{
			fprintf(stderr, "Failed to read block size");
			return -1;
		}

		uint32_t blockSize = sizeBytes[0] | (static_cast<uint32_t>(sizeBytes[1]) << 8) | (static_cast<uint32_t>(sizeBytes[2]) << 16) | (static_cast<uint32_t>(sizeBytes[3]) << 24);

		if (blockSize > pageSize || blockSize == 0)
		{
			fprintf(stderr, "Malformed block size");
			return -1;
		}

		///

		bytesRead = fread(sizeBytes, 1, 4, inF);

		if (bytesRead != 4)
		{
			fprintf(stderr, "Failed to read uncompressed size");
			return -1;
		}

		uint32_t uncompressedSize = sizeBytes[0] | (static_cast<uint32_t>(sizeBytes[1]) << 8) | (static_cast<uint32_t>(sizeBytes[2]) << 16) | (static_cast<uint32_t>(sizeBytes[3]) << 24);

		if (uncompressedSize < blockSize || uncompressedSize > pageSize)
		{
			fprintf(stderr, "Malformed uncompressed size");
			return -1;
		}

		///

		bytesRead = fread(sizeBytes, 1, 4, inF);

		if (bytesRead != 4)
		{
			fprintf(stderr, "Failed to read CRC");
			return -1;
		}

		uint32_t expectedCRC  = sizeBytes[0] | (static_cast<uint32_t>(sizeBytes[1]) << 8) | (static_cast<uint32_t>(sizeBytes[2]) << 16) | (static_cast<uint32_t>(sizeBytes[3]) << 24);

		std::vector<uint8_t> compressedPage;
		compressedPage.resize(blockSize);

		bytesRead = fread(&compressedPage[0], 1, blockSize, inF);

		uint32_t actualCRC = 0;
		if (bytesRead == pageSize)
		{
			fwrite(&compressedPage[0], 1, bytesRead, outF);

			uint32_t actualCRC = crc32(0, &compressedPage[0], uncompressedSize);
			if (actualCRC != expectedCRC)
			{
				fprintf(stderr, "Error in block %i: Expected CRC %x but CRC was %x", blockIndex, expectedCRC, actualCRC);
				return -1;
			}
		}
		else
		{
			std::vector<uint8_t> decompressedPage;
			decompressedPage.resize(uncompressedSize + 3);

			DecompressGstdCPU32(&compressedPage[0], blockSize, &decompressedPage[0], uncompressedSize, &blockIndex, DecompressWarn);

			uint32_t actualCRC = crc32(0, &decompressedPage[0], uncompressedSize);
			if (actualCRC != expectedCRC)
			{
				fprintf(stderr, "Error in block %i: Expected CRC %x but CRC was %x", blockIndex, expectedCRC, actualCRC);
				return -1;
			}

			fwrite(&decompressedPage[0], 1, uncompressedSize, outF);
		}

		blockIndex++;
	}

	fclose(inF);
	fclose(outF);

	return -1;
}

class CompressionGlobal
{
public:
	CompressionGlobal(FILE *inF, FILE *outF, size_t numPages, size_t pageSize, size_t globalSize, unsigned int compressionLevel, uint32_t tweaks, const char *failBlockPath);

	void ReadFromInput(void *dest, size_t offset, size_t size);
	void WriteToOutput(const void *src, uint32_t crc, size_t compressedSize, size_t uncompressedSize);

	size_t NumPages() const;
	size_t PageSize() const;
	size_t GlobalSize() const;
	unsigned int CompressionLevel() const;
	uint32_t Tweaks() const;
	const char* FailBlockPath() const;

private:
	std::mutex m_inFileMutex;
	FILE *m_inF;

	std::mutex m_outFileMutex;
	FILE *m_outF;

	std::mutex m_logMutex;

	size_t m_pageSize;
	size_t m_globalSize;
	size_t m_numPages;

	unsigned int m_compressionLevel;
	uint32_t m_tweaks;
	const char* m_failBlockPath;
};

CompressionGlobal::CompressionGlobal(FILE *inF, FILE *outF, size_t numPages, size_t pageSize, size_t globalSize, unsigned int compressionLevel, uint32_t tweaks, const char* failBlockPath)
	: m_inF(inF), m_outF(outF), m_numPages(numPages), m_pageSize(pageSize), m_globalSize(globalSize), m_compressionLevel(compressionLevel), m_tweaks(tweaks), m_failBlockPath(failBlockPath)
{
}

void CompressionGlobal::ReadFromInput(void *dest, size_t offset, size_t size)
{
	std::lock_guard<std::mutex> lock(m_inFileMutex);
	fseek(m_inF, static_cast<long>(offset), SEEK_SET);

	fread(dest, 1, size, m_inF);
}

void CompressionGlobal::WriteToOutput(const void *src, uint32_t crc, size_t compressedSize, size_t uncompressedSize)
{
	std::lock_guard<std::mutex> lock(m_outFileMutex);

	uint8_t chunkSizeBytes[12];
	chunkSizeBytes[0] = static_cast<uint8_t>((compressedSize >> 0) & 0xffu);
	chunkSizeBytes[1] = static_cast<uint8_t>((compressedSize >> 8) & 0xffu);
	chunkSizeBytes[2] = static_cast<uint8_t>((compressedSize >> 16) & 0xffu);
	chunkSizeBytes[3] = static_cast<uint8_t>((compressedSize >> 24) & 0xffu);
	chunkSizeBytes[4] = static_cast<uint8_t>((uncompressedSize >> 0) & 0xffu);
	chunkSizeBytes[5] = static_cast<uint8_t>((uncompressedSize >> 8) & 0xffu);
	chunkSizeBytes[6] = static_cast<uint8_t>((uncompressedSize >> 16) & 0xffu);
	chunkSizeBytes[7] = static_cast<uint8_t>((uncompressedSize >> 24) & 0xffu);
	chunkSizeBytes[8] = static_cast<uint8_t>((crc >> 0) & 0xffu);
	chunkSizeBytes[9] = static_cast<uint8_t>((crc >> 8) & 0xffu);
	chunkSizeBytes[10] = static_cast<uint8_t>((crc >> 16) & 0xffu);
	chunkSizeBytes[11] = static_cast<uint8_t>((crc >> 24) & 0xffu);

	fwrite(chunkSizeBytes, 1, 12, m_outF);
	fwrite(src, 1, compressedSize, m_outF);
}


size_t CompressionGlobal::NumPages() const
{
	return m_numPages;
}

size_t CompressionGlobal::PageSize() const
{
	return m_pageSize;
}

size_t CompressionGlobal::GlobalSize() const
{
	return m_globalSize;
}

unsigned int CompressionGlobal::CompressionLevel() const
{
	return m_compressionLevel;
}

unsigned int CompressionGlobal::Tweaks() const
{
	return m_tweaks;
}

const char* CompressionGlobal::FailBlockPath() const
{
	return m_failBlockPath;
}

class CompressionTask : public ThreadedTaskBase
{
public:
	CompressionTask();
	~CompressionTask();

	void Init(CompressionGlobal *cglobal);

	void RunWorkUnit(size_t workUnit) override;
	void FinishWritingWorkUnit() override;

private:
	size_t ComputeCurrentPageSize() const;

	size_t m_workUnit;
	size_t m_compressedSize;
	size_t m_maxCompressedSize;
	CompressionGlobal *m_cglobal;
	unsigned char *m_inputData;
	unsigned char *m_compressedData;
	ZSTD_CCtx *m_ctx;

	unsigned char *m_transcodedData;
	size_t m_transcodedSize;
	size_t m_transcodedCapacity;

	zstdhl_EncoderOutputObject_t m_encoderOutputObj;
	zstdhl_MemoryAllocatorObject_t m_memAlloc;
	gstd_EncoderState_t *m_encState;
	zstdhl_StreamSourceObject_t m_streamSource;

	size_t m_transcodeReadPos;
	size_t m_numLanes;

	static void *CBRealloc(void *userdata, void *ptr, size_t newSize);
	static zstdhl_ResultCode_t CBWriteBitstream(void *userdata, const void *data, size_t size);
	static size_t CBReadBytes(void *userdata, void *dest, size_t size);
};

CompressionTask::CompressionTask()
	: m_workUnit(0), m_cglobal(nullptr), m_inputData(nullptr), m_compressedData(nullptr), m_compressedSize(0), m_ctx(nullptr),
	m_transcodedData(nullptr), m_transcodedSize(0), m_transcodedCapacity(0), m_transcodeReadPos(0), m_encState(nullptr),
	m_maxCompressedSize(0)
{
	m_numLanes = 32;

	m_encoderOutputObj.m_userdata = nullptr;
	m_encoderOutputObj.m_writeBitstreamFunc = nullptr;

	m_memAlloc.m_userdata = nullptr;
	m_memAlloc.m_reallocFunc = nullptr;

	m_streamSource.m_readBytesFunc = nullptr;
	m_streamSource.m_userdata = nullptr;
}

CompressionTask::~CompressionTask()
{
	delete[] m_inputData;
	delete[] m_compressedData;
	delete[] m_transcodedData;

	if (m_ctx)
		ZSTD_freeCCtx(m_ctx);

	if (m_encState)
		gstd_Encoder_Destroy(m_encState);
}

void CompressionTask::Init(CompressionGlobal *cglobal)
{
	m_cglobal = cglobal;
	m_inputData = new unsigned char[cglobal->PageSize()];

	m_maxCompressedSize = ZSTD_compressBound(cglobal->PageSize());
	m_compressedData = new unsigned char[m_maxCompressedSize];

	m_ctx = ZSTD_createCCtx();

	m_memAlloc.m_userdata = this;
	m_memAlloc.m_reallocFunc = CBRealloc;

	m_encoderOutputObj.m_userdata = this;
	m_encoderOutputObj.m_writeBitstreamFunc = CBWriteBitstream;

	m_streamSource.m_userdata = this;
	m_streamSource.m_readBytesFunc = CBReadBytes;

	gstd_Encoder_Create(&m_encoderOutputObj, m_numLanes, gstd_ComputeMaxOffsetExtraBits(static_cast<uint32_t>(cglobal->PageSize())), m_cglobal->Tweaks(), &m_memAlloc, &m_encState);	// TODO: Error check
}

void CompressionTask::RunWorkUnit(size_t workUnit)
{
	m_workUnit = workUnit;

	size_t currentPageSize = ComputeCurrentPageSize();

	m_cglobal->ReadFromInput(m_inputData, workUnit * m_cglobal->PageSize(), currentPageSize);

	unsigned int clevel = std::min(m_cglobal->CompressionLevel(), static_cast<unsigned int>(ZSTD_maxCLevel()));

	ZSTD_CCtx_setPledgedSrcSize(m_ctx, currentPageSize);
	ZSTD_CCtx_setParameter(m_ctx, ZSTD_cParameter::ZSTD_c_compressionLevel, static_cast<int>(clevel));

	m_compressedSize = ZSTD_compress2(m_ctx, m_compressedData, m_maxCompressedSize, m_inputData, currentPageSize);

	ZSTD_CCtx_reset(m_ctx, ZSTD_reset_session_and_parameters);

	m_transcodeReadPos = 0;
	m_transcodedSize = 0;

	zstdhl_ResultCode_t transcodeResult = gstd_Encoder_Transcode(m_encState, &m_streamSource, &m_memAlloc);

	if (transcodeResult != ZSTDHL_RESULT_OK)
	{
		const char* failBlockBase = m_cglobal->FailBlockPath();

		if (failBlockBase[0] != 0)
		{
			std::string pathBase(failBlockBase);
			if (pathBase.back() != '/' && pathBase.back() != '\\')
				pathBase.append("/");

			fprintf(stderr, "Failed with result code %i in block %zu\n", static_cast<int>(transcodeResult), m_workUnit);
			m_transcodedSize = 0;

			char debugPath[128];
			sprintf_s(debugPath, "fail_block_%zu.bin", m_workUnit);

			std::string fullPath = pathBase + debugPath;

			if (FILE* debugFile = fopen(fullPath.c_str(), "wb"))
			{
				fwrite(m_inputData, 1, currentPageSize, debugFile);
				fclose(debugFile);
			}

			sprintf_s(debugPath, "fail_block_%zu.zstd", m_workUnit);

			fullPath = pathBase + debugPath;

			if (FILE* debugFile = fopen(fullPath.c_str(), "wb"))
			{
				fwrite(m_compressedData, 1, m_compressedSize, debugFile);
				fclose(debugFile);
			}
		}
	}
}

void CompressionTask::FinishWritingWorkUnit()
{
	size_t currentPageSize = ComputeCurrentPageSize();
	const unsigned char *compressedData = m_transcodedData;
	size_t compressedSize = m_transcodedSize;

	if (compressedSize >= currentPageSize)
	{
		compressedSize = currentPageSize;
		compressedData = m_inputData;
	}

	uint32_t crc = crc32(0, m_inputData, currentPageSize);

	m_cglobal->WriteToOutput(compressedData, crc, compressedSize, currentPageSize);
}

size_t CompressionTask::ComputeCurrentPageSize() const
{
	if (m_workUnit + 1u == m_cglobal->NumPages())
	{
		size_t lastPageSize = m_cglobal->GlobalSize() % m_cglobal->PageSize();
		if (lastPageSize != 0)
			return lastPageSize;
	}

	return m_cglobal->PageSize();
}


void *CompressionTask::CBRealloc(void *userdata, void *ptr, size_t newSize)
{
	if (newSize == 0 && ptr == nullptr)
		return nullptr;

	return realloc(ptr, newSize);
}

zstdhl_ResultCode_t CompressionTask::CBWriteBitstream(void *userdata, const void *data, size_t size)
{
	CompressionTask *self = static_cast<CompressionTask *>(userdata);

	size_t oldCapacity = self->m_transcodedCapacity;
	while (self->m_transcodedCapacity < self->m_transcodedSize + size)
		self->m_transcodedCapacity = std::max<size_t>(1024, self->m_transcodedCapacity * 2);

	if (oldCapacity != self->m_transcodedCapacity)
	{
		unsigned char *newData = new unsigned char[self->m_transcodedCapacity];
		memcpy(newData, self->m_transcodedData, self->m_transcodedSize);
		delete[] self->m_transcodedData;

		self->m_transcodedData = newData;
	}

	memcpy(self->m_transcodedData + self->m_transcodedSize, data, size);
	self->m_transcodedSize += size;

	return ZSTDHL_RESULT_OK;
}

size_t CompressionTask::CBReadBytes(void *userdata, void *dest, size_t size)
{
	CompressionTask *self = static_cast<CompressionTask *>(userdata);

	size_t available = self->m_compressedSize - self->m_transcodeReadPos;
	if (size > available)
		size = available;

	memcpy(dest, self->m_compressedData + self->m_transcodeReadPos, size);
	self->m_transcodeReadPos += size;

	return size;
}

int CompressMain(int optc, const char **optv, const char *inFileName, const char *outFileName)
{
	unsigned int maxThreads = std::thread::hardware_concurrency();
	unsigned int numThreads = maxThreads;
	unsigned int pageSize = 64 * 1024;
	unsigned int compressionLevel = static_cast<unsigned int>(ZSTD_defaultCLevel());
	const char* failBlockPath = "";
	uint32_t tweaks = 0;

	for (int i = 0; i < optc; i++)
	{
		const char *optName = optv[i];
		if (!strcmp(optName, "-pagesize"))
		{
			i++;
			if (i == optc || !sscanf(optv[i], "%u", &pageSize) || pageSize < 1024)
			{
				fprintf(stderr, "Invalid page size parameter for -pagesize");
				return -1;
			}
		}
		else if (!strcmp(optName, "-t"))
		{
			i++;
			if (i == optc || !sscanf(optv[i], "%u", &numThreads) || numThreads == 0)
			{
				fprintf(stderr, "Invalid thread count for -t");
				return -1;
			}
		}
		else if (!strcmp(optName, "-f"))
		{
			i++;
			if (i == optc)
			{
				fprintf(stderr, "Expected path for -f");
				return -1;
			}

			failBlockPath = optv[i];
		}
		else if (!strcmp(optName, "-level"))
		{
			i++;
			if (i == optc || !sscanf(optv[i], "%u", &compressionLevel))
			{
				fprintf(stderr, "Invalid level for -level");
				return -1;
			}
		}
		else if (!strcmp(optName, "-nofseshuffle"))
		{
			tweaks |= GSTD_TWEAK_NO_FSE_TABLE_SHUFFLE;
		}
		else
		{
			fprintf(stderr, "Invalid option %s", optName);
			return -1;
		}
	}

	if (numThreads > maxThreads)
		numThreads = maxThreads;
	else if (numThreads < 1)
		numThreads = 1;

	FILE *inF = fopen(inFileName, "rb");
	if (!inF)
	{
		fprintf(stderr, "Couldn't open input file %s", inFileName);
		return -1;
	}

	if (fseek(inF, 0, SEEK_END))
	{
		fprintf(stderr, "Failed to seek to end of input file");
		return -1;
	}

	long fileSizeL = ftell(inF);
	if (fileSizeL < 0 || fileSizeL > std::numeric_limits<size_t>::max())
	{
		fprintf(stderr, "Integer overflow");
		return -1;
	}

	if (fseek(inF, 0, SEEK_SET))
	{
		fprintf(stderr, "Failed to seek to start of input file");
		return -1;
	}

	size_t fileSize = static_cast<size_t>(fileSizeL);
	size_t numPages = fileSize / pageSize;
	if (fileSize % pageSize)
		numPages++;

	FILE *outF = fopen(outFileName, "wb");
	if (!outF)
	{
		fprintf(stderr, "Couldn't open output file %s", outFileName);
		return -1;
	}

	uint8_t pageSizeBytes[4];
	pageSizeBytes[0] = static_cast<uint8_t>((pageSize >> 0) & 0xffu);
	pageSizeBytes[1] = static_cast<uint8_t>((pageSize >> 8) & 0xffu);
	pageSizeBytes[2] = static_cast<uint8_t>((pageSize >> 16) & 0xffu);
	pageSizeBytes[3] = static_cast<uint8_t>((pageSize >> 24) & 0xffu);

	if (fwrite(pageSizeBytes, 1, 4, outF) != 4)
	{
		fprintf(stderr, "Failed to write page size");
		return -1;
	}

	SerializedTaskGlobalState globalState(numPages, numThreads);

	CompressionGlobal cglobal(inF, outF, numPages, pageSize, fileSize, compressionLevel, tweaks, failBlockPath);

	CompressionTask *tasks = new CompressionTask[numThreads];

	for (unsigned int i = 0; i < numThreads; i++)
	{
		tasks[i].Init(&cglobal);
		globalState.SetTaskRunner(i, &tasks[i]);
	}

	if (numThreads == 1)
	{
		globalState.RunThread(0);
	}
	else
	{
		std::thread **threads = new std::thread*[numThreads];
		for (unsigned int i = 0; i < numThreads; i++)
		{
			threads[i] = new std::thread([&globalState, i]
				{
					globalState.RunThread(i);
				});
		}

		for (unsigned int i = 0; i < numThreads; i++)
		{
			threads[i]->join();
			delete threads[i];
		}

		delete[] threads;

	}

	delete[] tasks;

	fclose(inF);
	fclose(outF);

	return 0;
}

int main(int argc, const char **argv)
{

	if (argc < 4)
	{
		PrintUsageAndQuit();
		return -1;
	}

	const char *inFileName = argv[argc - 2];
	const char *outFileName = argv[argc - 1];

	int numOptionArgs = argc - 4;
	const char **firstOption = argv + 2;

	if (!strcmp(argv[1], "c"))
		return CompressMain(numOptionArgs, firstOption, inFileName, outFileName);
	if (!strcmp(argv[1], "d"))
		return DecompressMain(numOptionArgs, firstOption, inFileName, outFileName);

	PrintUsageAndQuit();
	return -1;
}
