/*
Copyright (c) 2024 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#include "gstddec_prefix_cpp.h"

#include "gstd_constants.h"

//GSTDDEC_MAIN_FUNCTION_DEF(vuint32_t laneIndex)

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT Run(vuint32_t laneIndex)
{
	uint32_t writePos = 0;
	uint32_t readPos = 0;
	uint32_t lastClearedDWordPos = 0;
	uint8_t numBits = 0;

	vuint32_t bitstreamBits = GSTDDEC_VECTOR_UINT32(0);

	lastClearedDWordPos = 0;

	uint32_t controlWord = GSTDDEC_READ_INPUT_DWORD(readPos) | (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET);

	while (controlWord & (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET))
	{
		uint32_t decompressedSize = (controlWord >> GSTD_CONTROL_DECOMPRESSED_SIZE_OFFSET) & GSTD_CONTROL_DECOMPRESSED_SIZE_MASK;
		uint32_t blockType = (controlWord >> GSTD_CONTROL_BLOCK_TYPE_OFFSET) & GSTD_CONTROL_BLOCK_TYPE_MASK;
		uint32_t auxBit = (controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK;

		if (blockType == GSTD_BLOCK_TYPE_COMPRESSED)
		{
			//DecompressCompressedBlock(laneIndex);
		}
		else if (blockType == GSTD_BLOCK_TYPE_RLE)
		{
		}
		else if (blockType == GSTD_BLOCK_TYPE_RAW)
		{
		}
		else
		{
			GSTDDEC_WARN("Invalid block type");
		}

		controlWord = GSTDDEC_READ_INPUT_DWORD(readPos);
		readPos++;
	}
}


#ifdef __cplusplus

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ReadInputDWord(const vuint32_t& dwordPos) const
{
	vuint32_t result;
	for (unsigned int i = 0; i < TWidth; i++)
		result.Set(i, ReadInputDWord(dwordPos.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadInputDWord(uint32_t dwordPos) const
{
	if (dwordPos >= m_inSize)
		return 0;
	return m_inData[dwordPos];
}

void DecompressGstdCPU32(const void* inData, uint32_t inSize, void* outData, uint32_t outCapacity)
{
	const unsigned int laneCount = 32;
	const unsigned int formatLaneCount = 32;

	gstddec::DecompressorContext<laneCount, formatLaneCount> decompressor(static_cast<const uint32_t*>(inData), inSize, static_cast<uint32_t*>(outData), outCapacity);

	gstddec::VectorUInt<uint32_t, laneCount> laneIndexes;
	for (unsigned int i = 0; i < laneCount; i++)
		laneIndexes.Set(i, i);

	decompressor.Run(laneIndexes);
}


#endif
