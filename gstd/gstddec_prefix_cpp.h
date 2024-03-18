/*
Copyright (c) 2023 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#ifdef __cplusplus

#define GSTDDEC_FORMAT_WIDTH		TFormatWidth
#define GSTDDEC_VECTOR_WIDTH		TVectorWidth


#include "gstddec_proto_cpp.h"


#define GSTDDEC_FUNCTION_PREFIX	template<unsigned int TVectorWidth, unsigned int TFormatWidth>

#define GSTDDEC_FUNCTION_CONTEXT gstddec::DecompressorContext<TVectorWidth, TFormatWidth>::
#define GSTDDEC_TYPE_CONTEXT typename gstddec::DecompressorContext<TVectorWidth, TFormatWidth>::

#define GSTDDEC_READ_CONSTANT(constName)	(this->m_constants.constName)

#define GSTDDEC_READ_INPUT_DWORD(pos) (this->ReadInputDWord(pos))
#define GSTDDEC_CONDITIONAL_READ_INPUT_DWORD(pos) (this->ReadInputDWord(executionMask, pos))

#define GSTDDEC_VECTOR_UINT32(fillValue) (vuint32_t(fillValue))
#define GSTDDEC_VECTOR_UINT64(fillValue) (vuint64_t(fillValue))
#define GSTDDEC_VECTOR_BOOL(fillValue) (vbool_t(fillValue))
#define GSTDDEC_PROMOTE_UINT32_TO_UINT64(n) (vuint64_t(n))
#define GSTDDEC_DEMOTE_UINT64_TO_UINT32(n) (vuint32_t(n))
#define GSTDDEC_VECTOR_ANY(n) (WaveActiveAnyTrue(n))
#define GSTDDEC_VECTOR_TEST_PREV(n) (WavePrefixCountBits(n))
#define GSTDDEC_VECTOR_FIRST_TRUE_INDEX(n) (FirstTrueIndex(n))
#define GSTDDEC_VECTOR_LAST_TRUE_INDEX(n) (LastTrueIndex(n))
#define GSTDDEC_VECTOR_READ_FROM_INDEX(n, index) (WaveReadLaneAt((n), (index)))

#define GSTDDEC_WARN(msg) do { if (m_warnCallback != nullptr) m_warnCallback(m_warnContext, (msg));  } while (false)

#define GSTDDEC_PARAM_INOUT(type, name) type &name
#define GSTDDEC_PARAM_OUT(type, name) type &name

#define GSTDDEC_VECTOR_IF(condition) do { vbool_t executionMask = (condition);
#define GSTDDEC_VECTOR_ELSE executionMask = ~executionMask;
#define GSTDDEC_VECTOR_END_IF } while (false);

#define GSTDDEC_VECTOR_LOGICAL_OR(l, r)  ((l) | (r))
#define GSTDDEC_VECTOR_LOGICAL_AND(l, r) ((l) & (r))


#define GSTDDEC_VECTOR_IF_NESTED(condition) do { vbool_t outerExecutionMask = executionMask; { vbool_t executionMask = outerExecutionMask & (condition);
#define GSTDDEC_VECTOR_END_IF_NESTED } } while(false);

#define GSTDDEC_BRANCH_HINT
#define GSTDDEC_UNROLL_HINT

#define GSTDDEC_CONDITIONAL_LOAD(value, storage) ConditionalLoad(executionMask, (value), (storage))
#define GSTDDEC_CONDITIONAL_LOAD_INDEX(value, storage, index) ConditionalLoadVector(executionMask, (value), (storage), (index))

#define GSTDDEC_CONDITIONAL_STORE(storage, value) ConditionalStore(executionMask, (storage), (value))
#define GSTDDEC_CONDITIONAL_STORE_INDEX(storage, index, value) ConditionalStoreVector(executionMask, (storage), (index), (value))

#define GSTDDEC_CONDITIONAL_OR_INDEX(storage, index, value) ConditionalOrVector(executionMask, (storage), (index), (value))

#define GSTDDEC_FLUSH_GS ((void)0)

#define GSTDDEC_NEXT_LOG2_POWER(value) FirstBitHighPlusOne(value)
#define GSTDDEC_SUM(value) WaveSum(value)
#define GSTDDEC_INCLUSIVE_RUNNING_SUM(value) (WavePrefixSum(value) + (value))
#define GSTDDEC_EXCLUSIVE_RUNNING_SUM(value) (WavePrefixSum(value))
#define GSTDDEC_MIN(a, b) (ArithMin((a), (b)))
#define GSTDDEC_MAX(a, b) (ArithMax((a), (b)))

#define GSTDDEC_SANITIZE						1
#define GSTDDEC_SUPPORT_FAST_SEQUENTIAL_FILL	0

#define GSTDDEC_PASS_EXECUTION_MASK executionMask,

#define GSTDDEC_PARAM_EXECUTION_MASK vbool_t executionMask,

#define GSTDDEC_LANE_INDEX (LaneIndex())

#define GSTDDEC_TWEAK_NO_SCATTER				0

#endif
