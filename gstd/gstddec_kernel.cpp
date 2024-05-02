/*
Copyright (c) 2024 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#include "gstd_constants.h"

#include "gstddec_prefix_cpp.h"


//GSTDDEC_MAIN_FUNCTION_DEF(vuint32_t laneIndex)

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressRawBlock(vuint32_t laneIndex, uint32_t controlWord)
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressRLEBlock(vuint32_t laneIndex, uint32_t controlWord)
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressCompressedBlock(vuint32_t laneIndex, uint32_t controlWord)
{
	uint32_t litSectionType = ((controlWord >> GSTD_CONTROL_LIT_SECTION_TYPE_OFFSET) & GSTD_CONTROL_LIT_SECTION_TYPE_MASK);
	uint32_t litLengthsMode = ((controlWord >> GSTD_CONTROL_LIT_LENGTH_MODE_OFFSET) & GSTD_CONTROL_LIT_LENGTH_MODE_MASK);
	uint32_t offsetsMode = ((controlWord >> GSTD_CONTROL_OFFSET_MODE_OFFSET) & GSTD_CONTROL_OFFSET_MODE_MASK);
	uint32_t matchLengthsMode = ((controlWord >> GSTD_CONTROL_MATCH_LENGTH_MODE_OFFSET) & GSTD_CONTROL_MATCH_LENGTH_MODE_MASK);
	uint32_t auxBit = ((controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK);

	uint32_t regeneratedSize = 0;
	
	uint32_t literalsRegeneratedSize = ReadPackedSize();

	GSTDDEC_BRANCH_HINT if (litSectionType == GSTD_LITERALS_SECTION_TYPE_HUFFMAN)
		DecodeLitHuffmanTree(laneIndex, auxBit);
	else
	{
		GSTDDEC_BRANCH_HINT if (litSectionType == GSTD_LITERALS_SECTION_TYPE_RLE)
		{
			DecodeLitRLEByte();
		}
	}

	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}


GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT DecodeFSEValue(uint32_t numLanesToRefill, uint32_t vvecIndex, uint32_t accuracyLog, uint32_t firstCell)
{
	vuint32_t bits = BitstreamPeekNoTruncate(vvecIndex, numLanesToRefill, GSTD_MAX_ACCURACY_LOG);
	vuint32_t state = g_dstate.fseState[vvecIndex];

	uint32_t accuracyLogMask = (1 << accuracyLog) - 1;
	vuint32_t symbol = GSTDDEC_VECTOR_UINT32(0);

	BitstreamDiscard(vvecIndex, numLanesToRefill, g_dstate.fseDrainLevel[vvecIndex]);

	GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToRefill))
	{
		vuint32_t numBitsToRefill = g_dstate.fseDrainLevel[vvecIndex];
		vuint32_t bitsRefillMask = (GSTDDEC_VECTOR_UINT32(1) << numBitsToRefill) - GSTDDEC_VECTOR_UINT32(1);

		vuint32_t refilledState = state + (bits & bitsRefillMask);
		vuint32_t fseCellIndex = refilledState & GSTDDEC_VECTOR_UINT32(accuracyLogMask);

		vuint32_t fseCellData = GSTDDEC_VECTOR_UINT32(0);
		GSTDDEC_CONDITIONAL_LOAD_INDEX(fseCellData, gs_decompressorState.fseCells, GSTDDEC_VECTOR_UINT32(firstCell) + fseCellIndex);

		vuint32_t fseCellSymbol = (fseCellData >> GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_SYM_OFFSET)) & GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_SYM_MASK);
		vuint32_t fseCellNumBits = (fseCellData >> GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_NUMBITS_OFFSET)) & GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_NUMBITS_MASK);
		vuint32_t fseCellBaseline = (fseCellData >> GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_BASELINE_OFFSET)) & GSTDDEC_VECTOR_UINT32(GSTDDEC_FSE_TABLE_CELL_BASELINE_MASK);

		// Strip the bits corresponding to this FSE table's accuracy log and replace them with the baseline
		vuint32_t newState = (refilledState - fseCellIndex) + fseCellBaseline;

		GSTDDEC_CONDITIONAL_STORE(g_dstate.fseDrainLevel[vvecIndex], fseCellNumBits);
		GSTDDEC_CONDITIONAL_STORE(g_dstate.fseState[vvecIndex], newState);

		GSTDDEC_CONDITIONAL_STORE(symbol, fseCellSymbol);
	}
	GSTDDEC_VECTOR_END_IF

	return symbol;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeFSEHuffmanWeights(uint32_t numSpecifiedWeights, uint32_t accuracyLog, GSTDDEC_PARAM_OUT(uint32_t, huffmanWeightTotal))
{
	uint32_t bitstreamVVecToRefill = 0;

	uint32_t numWeightsRemaining = numSpecifiedWeights;
	uint32_t vvecIndex = 0;

	huffmanWeightTotal = 0;

	while (numWeightsRemaining > 0)
	{
		uint32_t refillSize = numWeightsRemaining;

		if (refillSize > GSTDDEC_VECTOR_WIDTH)
			refillSize = GSTDDEC_VECTOR_WIDTH;

		vuint32_t weights = DecodeFSEValue(refillSize, vvecIndex, accuracyLog, GSTDDEC_FSETAB_HUFF_WEIGHT_START);

		uint32_t firstWeight = numSpecifiedWeights - numWeightsRemaining;

		GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(refillSize))
		{
			vuint32_t weightIndex = GSTDDEC_VECTOR_UINT32(firstWeight) + GSTDDEC_LANE_INDEX;
			vuint32_t insertValue = weights << ((weightIndex & GSTDDEC_VECTOR_UINT32(7)) << GSTDDEC_VECTOR_UINT32(2));

			GSTDDEC_CONDITIONAL_OR_INDEX(gs_decompressorState.packedHuffmanWeights, weightIndex >> GSTDDEC_VECTOR_UINT32(3), insertValue);
		}
		GSTDDEC_VECTOR_END_IF

		// NOTE: This depends on DecodeFSEValue returning 0 for unused lanes
		vuint32_t weightIteration = (GSTDDEC_VECTOR_UINT32(1) << weights) >> GSTDDEC_VECTOR_UINT32(1);

		huffmanWeightTotal = huffmanWeightTotal + GSTDDEC_SUM(weightIteration);

		numWeightsRemaining -= refillSize;

		vvecIndex = (vvecIndex + 1) % GSTDDEC_VVEC_SIZE;
	}

	GSTDDEC_FLUSH_GS;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ClearLitHuffmanTree()
{
	GSTDDEC_UNROLL_HINT for (uint32_t firstWeight = 0; firstWeight < 64; firstWeight += GSTDDEC_VECTOR_WIDTH)
	{
		uint32_t numWeightsThisRound = 64 - firstWeight;
		if (numWeightsThisRound > GSTDDEC_VECTOR_WIDTH)
			numWeightsThisRound = GSTDDEC_VECTOR_WIDTH;

		GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numWeightsThisRound))
		{
			vuint32_t index = GSTDDEC_VECTOR_UINT32(firstWeight) + GSTDDEC_LANE_INDEX;
			GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.packedHuffmanWeights, index, GSTDDEC_VECTOR_UINT32(0));
		}
		GSTDDEC_VECTOR_END_IF
	}

	GSTDDEC_FLUSH_GS;
}


GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ExpandLitHuffmanTable(uint32_t numSpecifiedWeights, uint32_t weightTotal)
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLitHuffmanTree(vuint32_t laneIndex, uint32_t auxBit)
{
	uint32_t weightTotal = 0;

	ClearLitHuffmanTree();

	uint32_t numSpecifiedWeights = ReadRawByte();
	GSTDDEC_BRANCH_HINT if (numSpecifiedWeights == 0)
	{
		numSpecifiedWeights = ReadRawByte();
		if (numSpecifiedWeights == 0)
		{
			GSTDDEC_WARN("Huffman table had 0 specified weights");
			numSpecifiedWeights = 1;
		}

		uint32_t numWeightsInExistingUncompressedBytes = g_dstate.uncompressedBytesAvailable * 2;

		for (uint32_t firstWeight = 0; firstWeight < (numSpecifiedWeights + GSTDDEC_VECTOR_WIDTH - 1) / GSTDDEC_VECTOR_WIDTH; firstWeight++)
		{
			vuint32_t weightIndex = GSTDDEC_VECTOR_UINT32(firstWeight) + laneIndex;
			vuint32_t weight = GSTDDEC_VECTOR_UINT32(0);

			GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
			{
				GSTDDEC_VECTOR_IF_NESTED(weightIndex < GSTDDEC_VECTOR_UINT32(numWeightsInExistingUncompressedBytes))
				{
					vuint32_t extractedWeight = (GSTDDEC_VECTOR_UINT32(g_dstate.uncompressedBytes) >> (weightIndex * GSTDDEC_VECTOR_UINT32(4))) & GSTDDEC_VECTOR_UINT32(0xf);
					GSTDDEC_CONDITIONAL_STORE(weight, extractedWeight);
				}
				GSTDDEC_VECTOR_ELSE
				{
					vuint32_t weightOffsetFromReadPos = weightIndex - GSTDDEC_VECTOR_UINT32(numWeightsInExistingUncompressedBytes);
					vuint32_t dwordOffset = weightOffsetFromReadPos >> GSTDDEC_VECTOR_UINT32(3);
					vuint32_t bitOffset = (weightOffsetFromReadPos & GSTDDEC_VECTOR_UINT32(7)) * GSTDDEC_VECTOR_UINT32(4);

					vuint32_t extractedWeight = (GSTDDEC_CONDITIONAL_READ_INPUT_DWORD(GSTDDEC_VECTOR_UINT32(g_dstate.readPos) + dwordOffset) >> bitOffset) & GSTDDEC_VECTOR_UINT32(0xf);

					GSTDDEC_CONDITIONAL_STORE(weight, extractedWeight);
				}
				GSTDDEC_VECTOR_END_IF_NESTED

#if GSTDDEC_SANITIZE
				if (GSTDDEC_VECTOR_ANY(weight > GSTDDEC_VECTOR_UINT32(GSTD_MAX_HUFFMAN_WEIGHT)))
				{
					GSTDDEC_WARN("Weight exceeded maximum");
				}

				weight = GSTDDEC_MAX(weight, GSTDDEC_VECTOR_UINT32(GSTD_MAX_HUFFMAN_WEIGHT));
#endif
			}
			GSTDDEC_VECTOR_END_IF

			// Return to uniform control flow here and duplicate condition to handle no-maximal-reconvergence case
			GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
			{
				vuint32_t insertValue = weight << ((weightIndex & GSTDDEC_VECTOR_UINT32(7)) << GSTDDEC_VECTOR_UINT32(2));

				GSTDDEC_CONDITIONAL_OR_INDEX(gs_decompressorState.packedHuffmanWeights, weightIndex >> GSTDDEC_VECTOR_UINT32(3), insertValue);
			}
			GSTDDEC_VECTOR_END_IF

			weightTotal += GSTDDEC_SUM((GSTDDEC_VECTOR_UINT32(1) << weight) >> GSTDDEC_VECTOR_UINT32(1));
		}

		GSTDDEC_FLUSH_GS;

		if (numSpecifiedWeights > numWeightsInExistingUncompressedBytes)
		{
			uint32_t bytesConsumedFromReadPos = (numSpecifiedWeights + 1) / 2 - g_dstate.uncompressedBytesAvailable;
			g_dstate.readPos += bytesConsumedFromReadPos / 4;
			if ((bytesConsumedFromReadPos & 3) == 0)
				g_dstate.uncompressedBytesAvailable = 0;
			else
			{
				g_dstate.uncompressedBytes = GSTDDEC_READ_INPUT_DWORD(g_dstate.readPos);
				g_dstate.uncompressedBytesAvailable = 4 - (bytesConsumedFromReadPos & 3);
				g_dstate.readPos++;
			}
		}
	}
	else
	{
		uint32_t accuracyLog = auxBit + 5;

		DecodeFSETable(laneIndex, GSTDDEC_FSETAB_HUFF_WEIGHT_START, GSTD_MAX_HUFFMAN_WEIGHT, accuracyLog, GSTD_MAX_HUFFMAN_WEIGHT_ACCURACY_LOG);

		// Decode the actual weights
		DecodeFSEHuffmanWeights(numSpecifiedWeights, accuracyLog, weightTotal);
	}

	ExpandLitHuffmanTable(numSpecifiedWeights, weightTotal);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLitRLEByte()
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadRawByte()
{
	if (g_dstate.uncompressedBytesAvailable == 0)
	{
		g_dstate.uncompressedBytes = ReadInputDWord(g_dstate.readPos);
		g_dstate.uncompressedBytesAvailable = 4;
		g_dstate.readPos++;
	}

	uint32_t result = g_dstate.uncompressedBytes & 0xff;
	g_dstate.uncompressedBytes >>= 8;
	g_dstate.uncompressedBytesAvailable--;
	return result;
}


GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT WavePrefixSum(vuint32_t value)
{
	vuint32_t result;

	uint32_t runningTotal = 0;
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		result.Set(i, runningTotal);
		runningTotal += value.Get(i);
	}

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT WaveSum(vuint32_t value)
{
	uint32_t runningTotal = 0;
	for (unsigned int i = 0; i < TVectorWidth; i++)
		runningTotal += value.Get(i);

	return runningTotal;
}

GSTDDEC_FUNCTION_PREFIX
bool GSTDDEC_FUNCTION_CONTEXT WaveActiveAnyTrue(vbool_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
		if (value.Get(i))
			return true;
	return false;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT WavePrefixCountBits(vbool_t value)
{
	vuint32_t result;

	uint32_t runningTotal = 0;
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		result.Set(i, runningTotal);

		if (value.Get(i))
			runningTotal++;
	}

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT WaveReadLaneAt(vuint32_t value, uint32_t index)
{
	return value.Get(index);
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT WaveReadLaneAt(vuint32_t value, vuint32_t index)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, value.Get(index.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT FirstTrueIndex(vbool_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
		if (value.Get(i))
			return i;

	return TVectorWidth;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT LastTrueIndex(vbool_t value)
{
	for (unsigned int ri = 0; ri < TVectorWidth; ri++)
	{
		unsigned int i = TVectorWidth - 1 - ri;
		if (value.Get(i))
			return i;
	}

	return TVectorWidth;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT FirstBitHighPlusOne(vuint32_t value)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, FirstBitHighPlusOne(value.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT FirstBitLowPlusOne(uint32_t value)
{
	if (value == 0)
		return 1;

	uint32_t result = 0;
	if ((value & 0xffff) == 0)
	{
		result += 16;
		value >>= 16;
	}

	if ((value & 0xff) == 0)
	{
		result += 8;
		value >>= 8;
	}

	if ((value & 0xf) == 0)
	{
		result += 4;
		value >>= 4;
	}

	if ((value & 0x3) == 0)
	{
		result += 2;
		value >>= 2;
	}

	if ((value & 0x1) == 0)
	{
		result += 1;
		value >>= 1;
	}

	return result + 1;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT FirstBitHighPlusOne(uint32_t value)
{
	if (value == 0)
		return 0;

	uint32_t result = 0;
	if ((value & 0xffff0000u) != 0)
	{
		result += 16;
		value >>= 16;
	}

	if ((value & 0xff00) != 0)
	{
		result += 8;
		value >>= 8;
	}

	if ((value & 0xf0) != 0)
	{
		result += 4;
		value >>= 4;
	}

	if ((value & 0xc) != 0)
	{
		result += 2;
		value >>= 2;
	}

	if ((value & 0x2) != 0)
	{
		result += 1;
		value >>= 1;
	}

	return result + 1;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ArithMin(vuint32_t a, vuint32_t b)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, ArithMin(a.Get(i), b.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ArithMax(vuint32_t a, vuint32_t b)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, ArithMax(a.Get(i), b.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT LaneIndex()
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, i);

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ArithMin(uint32_t a, uint32_t b)
{
	if (a < b)
		return a;
	return b;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ArithMax(uint32_t a, uint32_t b)
{
	if (a > b)
		return a;
	return b;
}


GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadPackedSize()
{
	uint32_t packedSize = ReadRawByte();

	if ((packedSize & 1) == 0)
	{
		return (packedSize >> 1);
	}
	else
	{
		if ((packedSize & 2) == 1)
		{
			packedSize |= ReadRawByte() << 8;
			return (packedSize >> 2) + 128;
		}
		else
		{
			if (g_dstate.uncompressedBytesAvailable >= 2)
			{
				packedSize = (packedSize | (g_dstate.uncompressedBytes << 8)) & 0xffffff;
				g_dstate.uncompressedBytes >>= 16;
				g_dstate.uncompressedBytesAvailable -= 2;
			}
			else
			{
				// Either 0 or 1 uncompressed bytes available
				uint32_t bytesToTakeFromCurrent = g_dstate.uncompressedBytesAvailable;
				uint32_t bytesToTakeFromNext = 2 - bytesToTakeFromCurrent;

				packedSize |= (g_dstate.uncompressedBytes << 8);
				
				uint32_t nextDWord = GSTDDEC_READ_INPUT_DWORD(g_dstate.readPos);
				packedSize |= nextDWord << (bytesToTakeFromCurrent * 8 + 8);
				packedSize &= 0xffffff;

				g_dstate.uncompressedBytesAvailable = 4 - bytesToTakeFromNext;
				g_dstate.uncompressedBytes = nextDWord >> (bytesToTakeFromNext * 8);
				g_dstate.readPos++;
			}

			return (packedSize >> 2) + (128 + 16384);
		}
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT BitstreamDiscard(uint32_t vvecIndex, uint32_t numLanesToDiscard, vuint32_t numBits)
{
	if (GSTDDEC_VECTOR_ANY(g_dstate.bitstreamAvailable[vvecIndex] < numBits))
	{
		GSTDDEC_WARN("Flushed too many bits from the stream");
	}

	GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToDiscard))
	{
		GSTDDEC_CONDITIONAL_STORE(g_dstate.bitstreamAvailable[vvecIndex], g_dstate.bitstreamAvailable[vvecIndex] - numBits);
		GSTDDEC_CONDITIONAL_STORE(g_dstate.bitstreamBits[vvecIndex], g_dstate.bitstreamBits[vvecIndex] >> numBits);
	}
	GSTDDEC_VECTOR_END_IF
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT BitstreamPeekNoTruncate(uint32_t vvecIndex, uint32_t numLanesToLoad, uint32_t numBits)
{
	vuint32_t reloadIterator = GSTDDEC_VECTOR_UINT32(0);

	vbool_t isLaneInBounds = (GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToLoad));

	GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_LOGICAL_AND(g_dstate.bitstreamAvailable[vvecIndex] < GSTDDEC_VECTOR_UINT32(numBits), isLaneInBounds))
	{
		GSTDDEC_CONDITIONAL_STORE(reloadIterator, GSTDDEC_VECTOR_UINT32(1));

		vuint32_t loadedBits = GSTDDEC_CONDITIONAL_READ_INPUT_DWORD(GSTDDEC_VECTOR_UINT32(g_dstate.readPos) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(reloadIterator));
		vuint64_t toMergeBits = GSTDDEC_PROMOTE_UINT32_TO_UINT64(loadedBits);
		toMergeBits = toMergeBits << g_dstate.bitstreamAvailable[vvecIndex];

		vuint64_t mergedBits = g_dstate.bitstreamBits[vvecIndex] | toMergeBits;

		GSTDDEC_CONDITIONAL_STORE(g_dstate.bitstreamBits[vvecIndex], mergedBits);
		GSTDDEC_CONDITIONAL_STORE(g_dstate.bitstreamAvailable[vvecIndex], g_dstate.bitstreamAvailable[vvecIndex] + GSTDDEC_VECTOR_UINT32(32));
	}
	GSTDDEC_VECTOR_END_IF

		uint32_t loadSum = GSTDDEC_SUM(reloadIterator);

	g_dstate.readPos += loadSum;
	return GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]) & GSTDDEC_VECTOR_UINT32((1 << numBits) - 1);
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT BitstreamPeek(uint32_t vvecIndex, uint32_t numLanesToLoad, uint32_t numBits)
{
	return BitstreamPeekNoTruncate(vvecIndex, numLanesToLoad, numBits) & GSTDDEC_VECTOR_UINT32((1 << numBits) - 1);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeFSETable(vuint32_t laneIndex, uint32_t fseTabStart, uint32_t fseTabMaxSymInclusive, uint32_t accuracyLog, uint32_t maxAccuracyLog)
{
	uint32_t targetProbLimit = (1 << accuracyLog);
	uint32_t peekSize = maxAccuracyLog + 1 + GSTD_ZERO_PROB_REPEAT_BITS;

	uint32_t leadInCumulativeProb = 0;
	uint32_t leadInSymbol = 0;
	uint32_t lastCodedSymbol = 0;
	uint32_t numSymbols = 0;

	for (uint32_t firstInitLane = 0; firstInitLane < targetProbLimit; firstInitLane += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t slotIndex = GSTDDEC_VECTOR_UINT32(firstInitLane) + laneIndex;

		vbool_t isInBounds = GSTDDEC_VECTOR_BOOL(true);

		if (GSTDDEC_VECTOR_WIDTH > 32)
		{
			isInBounds = (slotIndex < GSTDDEC_VECTOR_UINT32(targetProbLimit));
		}

		GSTDDEC_VECTOR_IF(isInBounds)
		{
			GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.probTemps, slotIndex, GSTDDEC_VECTOR_UINT32(0));
		}
		GSTDDEC_VECTOR_END_IF
	}

	GSTDDEC_FLUSH_GS;

	while (leadInCumulativeProb < targetProbLimit && leadInSymbol <= fseTabMaxSymInclusive)
	{
		GSTDDEC_UNROLL_HINT
		for (uint32_t vvecIndex = 0; vvecIndex < GSTDDEC_VVEC_SIZE; vvecIndex++)
		{
			uint32_t lanesToProcessThisVVec = GSTDDEC_FORMAT_WIDTH - (vvecIndex * GSTDDEC_VECTOR_WIDTH);
			if (lanesToProcessThisVVec > GSTDDEC_VECTOR_WIDTH)
				lanesToProcessThisVVec = GSTDDEC_VECTOR_WIDTH;

			vuint32_t bitstreamIndex = laneIndex + GSTDDEC_VECTOR_UINT32(vvecIndex * GSTDDEC_VECTOR_WIDTH);

			vuint32_t probBits = BitstreamPeek(vvecIndex, lanesToProcessThisVVec, peekSize);
			vuint32_t probs = probBits;

			// Strip any unused bits
			GSTDDEC_BRANCH_HINT
			if (leadInCumulativeProb < targetProbLimit && leadInSymbol <= fseTabMaxSymInclusive)
			{
				uint32_t initialPrecision = GSTDDEC_NEXT_LOG2_POWER(targetProbLimit - leadInCumulativeProb);

				vuint32_t bitMasks = GSTDDEC_VECTOR_UINT32((1 << initialPrecision) - 1);

				bool anyOverflowed = false;
				do
				{
					probs = (probs & bitMasks);
					vuint32_t leadInProbs = GSTDDEC_VECTOR_UINT32(leadInCumulativeProb) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(probs);
					vuint32_t cumulativeProbs = leadInProbs + probs;

					vbool_t overflowed = (GSTDDEC_VECTOR_UINT32(targetProbLimit) < cumulativeProbs);

					anyOverflowed = GSTDDEC_VECTOR_ANY(overflowed);
					if (anyOverflowed)
					{
						uint32_t firstOverflowingLaneIndex = GSTDDEC_VECTOR_FIRST_TRUE_INDEX(overflowed);
						uint32_t probLimitInOverflowingLane = targetProbLimit - GSTDDEC_VECTOR_READ_FROM_INDEX(leadInProbs, firstOverflowingLaneIndex);
						uint32_t reducedPrecision = GSTDDEC_NEXT_LOG2_POWER(probLimitInOverflowingLane);

						GSTDDEC_VECTOR_IF(overflowed)
						{
							uint32_t reducedPrecMask = (1 << reducedPrecision) - 1;
							GSTDDEC_CONDITIONAL_STORE(bitMasks, GSTDDEC_VECTOR_UINT32(reducedPrecMask));

#if GSTDDEC_SANITIZE
							// Cap the value in the first overflowing lane at the maximum allowed value
							GSTDDEC_VECTOR_IF_NESTED(laneIndex == GSTDDEC_VECTOR_UINT32(firstOverflowingLaneIndex))
							{
								if ((GSTDDEC_VECTOR_READ_FROM_INDEX(probs, firstOverflowingLaneIndex) & reducedPrecMask) > probLimitInOverflowingLane)
								{
									GSTDDEC_WARN("Probability table had an overflowing entry that is still overflowing after bit reduction");
								}
								GSTDDEC_CONDITIONAL_STORE(probs, GSTDDEC_MIN((probs & bitMasks), GSTDDEC_VECTOR_UINT32(probLimitInOverflowingLane)));
							}
							GSTDDEC_VECTOR_END_IF_NESTED
#endif
						}
						GSTDDEC_VECTOR_END_IF
					}

				} while (anyOverflowed);

				// Compute actual bit usage
				vuint32_t leadInProbs = GSTDDEC_VECTOR_UINT32(leadInCumulativeProb) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(probs);
				vuint32_t vectorMaxProb = GSTDDEC_VECTOR_UINT32(targetProbLimit) - leadInProbs;
				vuint32_t probBitUsage = GSTDDEC_NEXT_LOG2_POWER(vectorMaxProb);

				vuint32_t symbolUsage = GSTDDEC_VECTOR_UINT32(1);

				GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_LOGICAL_AND(vectorMaxProb != GSTDDEC_VECTOR_UINT32(0), probs == GSTDDEC_VECTOR_UINT32(0)))
				{
					uint32_t zeroRepeatMask = ((1 << GSTD_ZERO_PROB_REPEAT_BITS) - 1);
					vuint32_t zeroRepeatCount = (probBits >> probBitUsage) & GSTDDEC_VECTOR_UINT32(zeroRepeatMask);
					GSTDDEC_CONDITIONAL_STORE(symbolUsage, symbolUsage + zeroRepeatCount);
					GSTDDEC_CONDITIONAL_STORE(probBitUsage, probBitUsage + GSTDDEC_VECTOR_UINT32(GSTD_ZERO_PROB_REPEAT_BITS));
				}
				GSTDDEC_VECTOR_END_IF

				BitstreamDiscard(vvecIndex, lanesToProcessThisVVec, probBitUsage);

				// Determine symbols
				vuint32_t symbol = GSTDDEC_VECTOR_UINT32(leadInSymbol) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(symbolUsage);
				vuint32_t symbolTablePass1SlotUsage = GSTDDEC_VECTOR_UINT32(0);
				vbool_t isNonZeroProbability = GSTDDEC_VECTOR_LOGICAL_AND(symbol <= GSTDDEC_VECTOR_UINT32(fseTabMaxSymInclusive), probs != GSTDDEC_VECTOR_UINT32(0));
				GSTDDEC_VECTOR_IF(isNonZeroProbability)
				{
					GSTDDEC_CONDITIONAL_STORE(symbolTablePass1SlotUsage, probs);

					vuint32_t highUsageStartIndex = GSTDDEC_VECTOR_UINT32(numSymbols) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(symbolTablePass1SlotUsage);

					vuint32_t bitsForProbRoundedUp = GSTDDEC_NEXT_LOG2_POWER(probs - GSTDDEC_VECTOR_UINT32(1));
					vuint32_t probNumericRangeRoundedToNextPO2 = (GSTDDEC_VECTOR_UINT32(1) << bitsForProbRoundedUp);

					vuint32_t highBitUsage = GSTDDEC_VECTOR_UINT32(accuracyLog + 1) - bitsForProbRoundedUp;

					//vuint32_t highBitUsage = GSTDDEC_NEXT_LOG2_POWER(probs - GSTDDEC_VECTOR_UINT32(1));
					vuint32_t numHighBitUsage = probNumericRangeRoundedToNextPO2 - probs;
					vuint32_t numLowBitUsage = probs - numHighBitUsage;

					vuint32_t highUsageBaselineOffset = probs - (probNumericRangeRoundedToNextPO2 >> GSTDDEC_VECTOR_UINT32(1));

					vuint32_t highBitUsageBaselineZeroPoint = highUsageStartIndex - highUsageBaselineOffset;

					vuint32_t lowBitUsage = highBitUsage - GSTDDEC_VECTOR_UINT32(1);
					vuint32_t lowUsageStartIndex = highUsageStartIndex + numHighBitUsage;
					vuint32_t lowUsageBaselineZeroPoint = lowUsageStartIndex;

					// Bit-pack into ascending-order values.
					vuint32_t combinedValueLow = (symbol << GSTDDEC_VECTOR_UINT32(24)) | (lowBitUsage << GSTDDEC_VECTOR_UINT32(16)) | lowUsageBaselineZeroPoint;
					vuint32_t combinedValueHigh = (symbol << GSTDDEC_VECTOR_UINT32(24)) | (highBitUsage << GSTDDEC_VECTOR_UINT32(16)) | (highBitUsageBaselineZeroPoint & GSTDDEC_VECTOR_UINT32(0xffff));

#if !GSTDDEC_SUPPORT_FAST_SEQUENTIAL_FILL
					combinedValueLow = combinedValueLow ^ GSTDDEC_VECTOR_UINT32(0x00ff0000);
					combinedValueHigh = combinedValueHigh ^ GSTDDEC_VECTOR_UINT32(0x00ff0000);
#endif
					
					// High start and low start may be the same, but will never overlap other writes, so this should be OK.
					// Low usage takes priority, since in the PO2 case, high count will be zero
					GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.probTemps, highUsageStartIndex, combinedValueHigh);
					GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.probTemps, lowUsageStartIndex, combinedValueLow);
				}
				GSTDDEC_VECTOR_END_IF

				if (GSTDDEC_VECTOR_ANY(isNonZeroProbability))
				{
					lastCodedSymbol = GSTDDEC_VECTOR_READ_FROM_INDEX(symbol, GSTDDEC_VECTOR_LAST_TRUE_INDEX(isNonZeroProbability));
				}

				leadInCumulativeProb += GSTDDEC_SUM(probs);
				leadInSymbol += GSTDDEC_SUM(symbolUsage);
				numSymbols += GSTDDEC_SUM(symbolTablePass1SlotUsage);
			}
		}
	}

	GSTDDEC_FLUSH_GS;

	if (leadInCumulativeProb < targetProbLimit)
	{
		GSTDDEC_WARN("Probability table didn't fill all probs");
	}

	// Expand probs table
	uint32_t leadInTemp = 0;
	uint32_t advanceStep = (5 << (accuracyLog - 3)) + 3;

	for (uint32_t firstFillLane = 0; firstFillLane < targetProbLimit; firstFillLane += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t slotIndex = GSTDDEC_VECTOR_UINT32(firstFillLane) + laneIndex;

		vbool_t isInBounds = GSTDDEC_VECTOR_BOOL(true);

		if (GSTDDEC_VECTOR_WIDTH > GSTDDEC_FORMAT_WIDTH)
		{
			isInBounds = (slotIndex < GSTDDEC_VECTOR_UINT32(targetProbLimit));
		}

		vuint32_t probTemp = GSTDDEC_VECTOR_UINT32(0);
		GSTDDEC_VECTOR_IF(isInBounds)
		{
			GSTDDEC_CONDITIONAL_LOAD_INDEX(probTemp, gs_decompressorState.probTemps, slotIndex);
		}
		GSTDDEC_VECTOR_END_IF

#if GSTDDEC_SUPPORT_FAST_SEQUENTIAL_FILL
#error "NYI"
#else
		probTemp = FastFillAscending(probTemp, leadInTemp) ^ GSTDDEC_VECTOR_UINT32(0x00ff0000);
#endif

		// Compute actual baseline
		vuint32_t numBits = (probTemp & GSTDDEC_VECTOR_UINT32(0xff0000)) >> GSTDDEC_VECTOR_UINT32(16);
		vuint32_t adjustedBaseline = ((slotIndex - probTemp) & GSTDDEC_VECTOR_UINT32(0xffff)) << numBits;
		probTemp = (probTemp & GSTDDEC_VECTOR_UINT32(0xffff0000)) | adjustedBaseline;

		GSTDDEC_VECTOR_IF(isInBounds)
		{
			vuint32_t placementIndex = ((slotIndex * GSTDDEC_VECTOR_UINT32(advanceStep)) & GSTDDEC_VECTOR_UINT32(targetProbLimit - 1));

			GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.fseCells, placementIndex + GSTDDEC_VECTOR_UINT32(fseTabStart), probTemp);
		}
		GSTDDEC_VECTOR_END_IF
	}

	GSTDDEC_FLUSH_GS;
}


GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT Run(vuint32_t laneIndex)
{
	uint32_t writePos = 0;
	uint32_t lastClearedDWordPos = 0;
	uint8_t numBits = 0;

	g_dstate.readPos = 0;
	g_dstate.uncompressedBytesAvailable = 0;
	g_dstate.uncompressedBytes = 0;
	g_dstate.litRLEByte = 0;

	for (uint32_t i = 0; i < GSTDDEC_VVEC_SIZE; i++)
	{
		g_dstate.bitstreamBits[i] = GSTDDEC_VECTOR_UINT64(0);
		g_dstate.bitstreamAvailable[i] = GSTDDEC_VECTOR_UINT32(0);
		g_dstate.fseState[i] = GSTDDEC_VECTOR_UINT32(0);
		g_dstate.fseDrainLevel[i] = GSTDDEC_VECTOR_UINT32(GSTD_MAX_ACCURACY_LOG);
	}

	lastClearedDWordPos = 0;

	uint32_t controlWord = GSTDDEC_READ_INPUT_DWORD(g_dstate.readPos) | (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET);
	g_dstate.readPos++;

	while (controlWord & (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET))
	{
		uint32_t decompressedSize = (controlWord >> GSTD_CONTROL_DECOMPRESSED_SIZE_OFFSET) & GSTD_CONTROL_DECOMPRESSED_SIZE_MASK;
		uint32_t blockType = (controlWord >> GSTD_CONTROL_BLOCK_TYPE_OFFSET) & GSTD_CONTROL_BLOCK_TYPE_MASK;
		uint32_t auxBit = (controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK;

		if (blockType == GSTD_BLOCK_TYPE_COMPRESSED)
		{
			DecompressCompressedBlock(laneIndex, controlWord);
		}
		else if (blockType == GSTD_BLOCK_TYPE_RLE)
		{
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
		else if (blockType == GSTD_BLOCK_TYPE_RAW)
		{
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
		else
		{
			GSTDDEC_WARN("Invalid block type");
		}

		controlWord = GSTDDEC_READ_INPUT_DWORD(g_dstate.readPos);
		g_dstate.readPos++;
	}
}


#ifdef __cplusplus

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ReadInputDWord(const vuint32_t& dwordPos) const
{
	vuint32_t result;
	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, ReadInputDWord(dwordPos.Get(i)));

	return result;
}


GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ReadInputDWord(vbool_t executionMask, vuint32_t dwordPos) const
{
	vuint32_t result;
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			result.Set(i, ReadInputDWord(dwordPos.Get(i)));
		else
			result.Set(i, 0xcccccccc);
	}

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadInputDWord(uint32_t dwordPos) const
{
	if (dwordPos >= m_inSize)
		return 0;
	return m_inData[dwordPos];
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT PutOutputDWord(const vuint32_t &dwordPos, const vuint32_t &dword) const
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
		PutOutputDWord(dwordPos.Get(i), dword.Get(i));
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT PutOutputDWord(uint32_t dwordPos, uint32_t dword) const
{
	if (dwordPos < m_outSize)
		m_outData[dwordPos] = dword;
}

// This fills all zero values in a vuint32_t with the preceding value, and the preceding value must be less
GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT FastFillAscending(vuint32_t value, GSTDDEC_PARAM_INOUT(uint32_t, runningFillValue))
{
#if GSTDDEC_SUPPORT_FAST_SEQUENTIAL_FILL
#error "NYI"
#else
	value = GSTDDEC_MAX(value, GSTDDEC_VECTOR_UINT32(runningFillValue));

	GSTDDEC_UNROLL_HINT
	for (uint32_t backDistance = 1; backDistance < GSTDDEC_VECTOR_WIDTH; backDistance <<= 1)
	{
		vuint32_t altIndex = (GSTDDEC_LANE_INDEX - GSTDDEC_VECTOR_UINT32(backDistance));

#ifdef __cplusplus
		// Don't support conditional WaveReadLaneAt, just do this instead
		altIndex = (altIndex & GSTDDEC_VECTOR_UINT32(GSTDDEC_VECTOR_WIDTH - 1));
#endif

		GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_UINT32(backDistance) <= GSTDDEC_LANE_INDEX)
		{
			GSTDDEC_CONDITIONAL_STORE(value, GSTDDEC_MAX(value, GSTDDEC_VECTOR_READ_FROM_INDEX(value, altIndex)));
		}
		GSTDDEC_VECTOR_END_IF
	}

	runningFillValue = GSTDDEC_VECTOR_READ_FROM_INDEX(value, GSTDDEC_VECTOR_WIDTH - 1);
#endif

	return value;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalStoreVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage[index.Get(i)] = value.Get(i);
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalStore(vbool_t executionMask, vuint32_t &storage, vuint32_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage.Set(i, value.Get(i));
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalStore(vbool_t executionMask, vuint64_t &storage, vuint64_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage.Set(i, value.Get(i));
	}
}


GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalLoadVector(vbool_t executionMask, vuint32_t &value, const uint32_t *storage, vuint32_t index)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			value.Set(i, storage[index.Get(i)]);
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalLoad(vbool_t executionMask, vuint32_t &value, const vuint32_t &storage)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			value.Set(i, storage.Get(i));
	}
}


GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalOrVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage[index.Get(i)] |= value.Get(i);
	}
}

void DecompressGstdCPU32(const void *inData, uint32_t inSize, void *outData, uint32_t outCapacity, void *warnContext, void (*warnCallback)(void *, const char *))
{
	const unsigned int laneCount = 32;
	const unsigned int formatLaneCount = 32;

	gstddec::DecompressorContext<laneCount, formatLaneCount> decompressor(static_cast<const uint32_t*>(inData), inSize, static_cast<uint32_t*>(outData), outCapacity, warnContext, warnCallback);

	gstddec::VectorUInt<uint32_t, laneCount> laneIndexes;
	for (unsigned int i = 0; i < laneCount; i++)
		laneIndexes.Set(i, i);

	decompressor.Run(laneIndexes);
}


#endif
