#include "gstddec_prefix_cpp.h"

#include "../zstdhl/gstd_constants.h"

//GSTDDEC_MAIN_FUNCTION_DEF(vuint32_t laneIndex)

template<unsigned int TWidth>
void gstddec::DecompressorContext<TWidth>::Run(vuint32_t laneIndex)
{
	uint32_t writePos = 0;
	uint32_t readPos = 0;
	uint32_t lastClearedDWordPos = 0;
	uint8_t numBits = 0;

	vuint32_t bitstreamBits = GSTDDEC_VECTOR_UINT32(0);

	lastClearedDWordPos = 0;

	uint32_t controlWord = (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET);

	while (controlWord & (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET))
	{
		uint32_t decompressedSize = (controlWord >> GSTD_CONTROL_DECOMPRESSED_SIZE_OFFSET) & GSTD_CONTROL_DECOMPRESSED_SIZE_MASK;
		uint32_t blockType = (controlWord >> GSTD_CONTROL_BLOCK_TYPE_OFFSET) & GSTD_CONTROL_BLOCK_TYPE_MASK;

		controlWord = GSTDDEC_READ_INPUT_DWORD(readPos);
		readPos++;
	}
}
