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
void GSTDDEC_FUNCTION_CONTEXT DecompressRawBlock(vuint32_t laneIndex, uint32_t controlWord, GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressRLEBlock(vuint32_t laneIndex, uint32_t controlWord, GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressCompressedBlock(vuint32_t laneIndex, uint32_t controlWord, GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	uint32_t litSectionType = ((controlWord >> GSTD_CONTROL_LIT_SECTION_TYPE_OFFSET) & GSTD_CONTROL_LIT_SECTION_TYPE_MASK);
	uint32_t litLengthsMode = ((controlWord >> GSTD_CONTROL_LIT_LENGTH_MODE_OFFSET) & GSTD_CONTROL_LIT_LENGTH_MODE_MASK);
	uint32_t offsetsMode = ((controlWord >> GSTD_CONTROL_OFFSET_MODE_OFFSET) & GSTD_CONTROL_OFFSET_MODE_MASK);
	uint32_t matchLengthsMode = ((controlWord >> GSTD_CONTROL_MATCH_LENGTH_MODE_OFFSET) & GSTD_CONTROL_MATCH_LENGTH_MODE_MASK);
	uint32_t auxBit = ((controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK);

	uint32_t regeneratedSize = 0;
	
	uint32_t literalsRegeneratedSize = ReadPackedSize(dstate);

	GSTDDEC_BRANCH_HINT if (litSectionType == GSTD_LITERALS_SECTION_TYPE_HUFFMAN)
		DecodeLitHuffmanTree(laneIndex, auxBit, dstate);
	else
	{
		GSTDDEC_BRANCH_HINT if (litSectionType == GSTD_LITERALS_SECTION_TYPE_RLE)
		{
			DecodeLitRLEByte(dstate);
		}
	}

	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLitHuffmanTree(vuint32_t laneIndex, uint32_t auxBit, GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	uint32_t numSpecifiedWeights = ReadRawByte(dstate);
	GSTDDEC_BRANCH_HINT if (numSpecifiedWeights == 0)
	{
		numSpecifiedWeights = ReadRawByte(dstate);
		if (numSpecifiedWeights == 0)
		{
			GSTDDEC_WARN("Huffman table had 0 specified weights");
			numSpecifiedWeights = 1;
		}

		uint32_t numPackedHuffmanSlots = (numSpecifiedWeights + 3) / 4;
		for (uint32_t firstInit = 0; firstInit < (numPackedHuffmanSlots + GSTDDEC_VECTOR_WIDTH - 1) / GSTDDEC_VECTOR_WIDTH; firstInit++)
		{
			vuint32_t initIndex = GSTDDEC_VECTOR_UINT32(firstInit) + laneIndex;

			GSTDDEC_VECTOR_IF(initIndex < GSTDDEC_VECTOR_UINT32(numPackedHuffmanSlots))
			{
				GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.packedHuffmanWeights, initIndex, GSTDDEC_VECTOR_UINT32(0));
			}
			GSTDDEC_VECTOR_END_IF
		}

		uint32_t numWeightsInExistingUncompressedBytes = dstate.uncompressedBytesAvailable * 2;

		for (uint32_t firstWeight = 0; firstWeight < (numSpecifiedWeights + GSTDDEC_VECTOR_WIDTH - 1) / GSTDDEC_VECTOR_WIDTH; firstWeight++)
		{
			vuint32_t weightIndex = GSTDDEC_VECTOR_UINT32(firstWeight) + laneIndex;
			vuint32_t weight = GSTDDEC_VECTOR_UINT32(0);

			GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
			{
				GSTDDEC_VECTOR_IF_NESTED(weightIndex < GSTDDEC_VECTOR_UINT32(numWeightsInExistingUncompressedBytes))
				{
					vuint32_t extractedWeight = (GSTDDEC_VECTOR_UINT32(dstate.uncompressedBytes) >> (weightIndex * GSTDDEC_VECTOR_UINT32(4))) & GSTDDEC_VECTOR_UINT32(0xf);
					GSTDDEC_CONDITIONAL_STORE(weight, extractedWeight);
				}
				GSTDDEC_VECTOR_ELSE
				{
					vuint32_t weightOffsetFromReadPos = weightIndex - GSTDDEC_VECTOR_UINT32(numWeightsInExistingUncompressedBytes);
					vuint32_t dwordOffset = weightOffsetFromReadPos >> GSTDDEC_VECTOR_UINT32(3);
					vuint32_t bitOffset = (weightOffsetFromReadPos & GSTDDEC_VECTOR_UINT32(7)) * GSTDDEC_VECTOR_UINT32(4);

					vuint32_t extractedWeight = (GSTDDEC_CONDITIONAL_READ_INPUT_DWORD(GSTDDEC_VECTOR_UINT32(dstate.readPos) + dwordOffset) >> bitOffset) & GSTDDEC_VECTOR_UINT32(0xf);

					GSTDDEC_CONDITIONAL_STORE(weight, extractedWeight);
				}
				GSTDDEC_VECTOR_END_IF_NESTED
			}
			GSTDDEC_VECTOR_END_IF

			// Return to uniform control flow here and duplicate condition to handle no-maximal-reconvergence case
			GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
			{
				GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.packedHuffmanWeights, weightIndex, weight);
			}
			GSTDDEC_VECTOR_END_IF
		}

		if (numSpecifiedWeights > numWeightsInExistingUncompressedBytes)
		{
			uint32_t bytesConsumedFromReadPos = (numSpecifiedWeights + 1) / 2 - dstate.uncompressedBytesAvailable;
			dstate.readPos += bytesConsumedFromReadPos / 4;
			if ((bytesConsumedFromReadPos & 3) == 0)
				dstate.uncompressedBytesAvailable = 0;
			else
			{
				dstate.uncompressedBytes = GSTDDEC_READ_INPUT_DWORD(dstate.readPos);
				dstate.uncompressedBytesAvailable = 4 - (bytesConsumedFromReadPos & 3);
				dstate.readPos++;
			}
		}
	}
	else
	{
		uint32_t accuracyLog = auxBit + 5;

		DecodeFSETable(laneIndex, GSTDDEC_FSETAB_HUFF_WEIGHT_START, GSTD_MAX_HUFFMAN_WEIGHT, accuracyLog, GSTD_MAX_HUFFMAN_WEIGHT_ACCURACY_LOG, dstate);

		// Decode the actual weights
		GSTDDEC_WARN("NOT YET IMPLEMENTED");
	}

	GSTDDEC_FLUSH_GS;

	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLitRLEByte(DecompressorState &dstate)
{

	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadRawByte(GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	if (dstate.uncompressedBytesAvailable == 0)
	{
		dstate.uncompressedBytes = ReadInputDWord(dstate.readPos);
		dstate.uncompressedBytesAvailable = 4;
		dstate.readPos++;
	}

	uint32_t result = dstate.uncompressedBytes & 0xff;
	dstate.uncompressedBytes >>= 8;
	dstate.uncompressedBytesAvailable--;
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
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadPackedSize(GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
{
	uint32_t packedSize = ReadRawByte(dstate);

	if ((packedSize & 1) == 0)
	{
		return (packedSize >> 1);
	}
	else
	{
		if ((packedSize & 2) == 1)
		{
			packedSize |= ReadRawByte(dstate) << 8;
			return (packedSize >> 2) + 128;
		}
		else
		{
			if (dstate.uncompressedBytesAvailable >= 2)
			{
				packedSize = (packedSize | (dstate.uncompressedBytes << 8)) & 0xffffff;
				dstate.uncompressedBytes >>= 16;
				dstate.uncompressedBytesAvailable -= 2;
			}
			else
			{
				// Either 0 or 1 uncompressed bytes available
				uint32_t bytesToTakeFromCurrent = dstate.uncompressedBytesAvailable;
				uint32_t bytesToTakeFromNext = 2 - bytesToTakeFromCurrent;

				packedSize |= (dstate.uncompressedBytes << 8);
				
				uint32_t nextDWord = GSTDDEC_READ_INPUT_DWORD(dstate.readPos);
				packedSize |= nextDWord << (bytesToTakeFromCurrent * 8 + 8);
				packedSize &= 0xffffff;

				dstate.uncompressedBytesAvailable = 4 - bytesToTakeFromNext;
				dstate.uncompressedBytes = nextDWord >> (bytesToTakeFromNext * 8);
				dstate.readPos++;
			}

			return (packedSize >> 2) + (128 + 16384);
		}
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT BitstreamDiscard(GSTDDEC_PARAM_INOUT(DecompressorState, dstate), uint32_t vvecIndex, vuint32_t laneIndex, vuint32_t numBits)
{
	if (GSTDDEC_VECTOR_ANY(dstate.bitstreamAvailable[vvecIndex] < numBits))
	{
		GSTDDEC_WARN("Flushed too many bits from the stream");
	}

	dstate.bitstreamAvailable[vvecIndex] = dstate.bitstreamAvailable[vvecIndex] - numBits;
	dstate.bitstreamBits[vvecIndex] = dstate.bitstreamBits[vvecIndex] >> numBits;
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT BitstreamPeek(GSTDDEC_PARAM_INOUT(DecompressorState, dstate), uint32_t vvecIndex, vuint32_t laneIndex, uint32_t numBits)
{
	vuint32_t reloadIterator = GSTDDEC_VECTOR_UINT32(0);

	vbool_t isLaneInBounds = GSTDDEC_VECTOR_LOGICAL_OR(GSTDDEC_VECTOR_BOOL(!GSTDDEC_IS_WIDER_THAN_FORMAT), laneIndex < GSTDDEC_VECTOR_UINT32(GSTDDEC_FORMAT_WIDTH));

	GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_LOGICAL_AND(dstate.bitstreamAvailable[vvecIndex] < GSTDDEC_VECTOR_UINT32(numBits), isLaneInBounds))
	{
		GSTDDEC_CONDITIONAL_STORE(reloadIterator, GSTDDEC_VECTOR_UINT32(1));

		vuint32_t loadedBits = GSTDDEC_CONDITIONAL_READ_INPUT_DWORD(GSTDDEC_VECTOR_UINT32(dstate.readPos) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(reloadIterator));
		vuint64_t toMergeBits = GSTDDEC_PROMOTE_UINT32_TO_UINT64(loadedBits);
		toMergeBits = toMergeBits << dstate.bitstreamAvailable[vvecIndex];

		vuint64_t mergedBits = dstate.bitstreamBits[vvecIndex] | toMergeBits;

		GSTDDEC_CONDITIONAL_STORE(dstate.bitstreamBits[vvecIndex], mergedBits);
		GSTDDEC_CONDITIONAL_STORE(dstate.bitstreamAvailable[vvecIndex], dstate.bitstreamAvailable[vvecIndex] + GSTDDEC_VECTOR_UINT32(32));
	}
	GSTDDEC_VECTOR_END_IF

	uint32_t loadSum = GSTDDEC_SUM(reloadIterator);

	dstate.readPos += loadSum;
	return GSTDDEC_DEMOTE_UINT64_TO_UINT32(dstate.bitstreamBits[vvecIndex]) & GSTDDEC_VECTOR_UINT32((1 << numBits) - 1);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeFSETable(vuint32_t laneIndex, uint32_t fseTabStart, uint32_t fseTabMaxSymInclusive, uint32_t accuracyLog, uint32_t maxAccuracyLog, GSTDDEC_PARAM_INOUT(DecompressorState, dstate))
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
			vuint32_t bitstreamIndex = laneIndex + GSTDDEC_VECTOR_UINT32(vvecIndex * GSTDDEC_VECTOR_WIDTH);

			vuint32_t probBits = BitstreamPeek(dstate, vvecIndex, laneIndex, peekSize);
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

				BitstreamDiscard(dstate, vvecIndex, laneIndex, probBitUsage);

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

		if (GSTDDEC_VECTOR_WIDTH > 32)
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
	DecompressorState dstate;

	uint32_t writePos = 0;
	uint32_t lastClearedDWordPos = 0;
	uint8_t numBits = 0;

	dstate.readPos = 0;
	dstate.uncompressedBytesAvailable = 0;
	dstate.uncompressedBytes = 0;
	dstate.litRLEByte = 0;

	for (uint32_t i = 0; i < GSTDDEC_VVEC_SIZE; i++)
	{
		dstate.bitstreamBits[i] = GSTDDEC_VECTOR_UINT64(0);
		dstate.bitstreamAvailable[i] = GSTDDEC_VECTOR_UINT32(0);
		dstate.fseState[i] = GSTDDEC_VECTOR_UINT32(0);
		dstate.fseDrainLevel[i] = GSTDDEC_VECTOR_UINT32(GSTD_MAX_ACCURACY_LOG);
	}

	lastClearedDWordPos = 0;

	uint32_t controlWord = GSTDDEC_READ_INPUT_DWORD(dstate.readPos) | (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET);
	dstate.readPos++;

	while (controlWord & (1 << GSTD_CONTROL_MORE_BLOCKS_BIT_OFFSET))
	{
		uint32_t decompressedSize = (controlWord >> GSTD_CONTROL_DECOMPRESSED_SIZE_OFFSET) & GSTD_CONTROL_DECOMPRESSED_SIZE_MASK;
		uint32_t blockType = (controlWord >> GSTD_CONTROL_BLOCK_TYPE_OFFSET) & GSTD_CONTROL_BLOCK_TYPE_MASK;
		uint32_t auxBit = (controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK;

		if (blockType == GSTD_BLOCK_TYPE_COMPRESSED)
		{
			DecompressCompressedBlock(laneIndex, controlWord, dstate);
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

		controlWord = GSTDDEC_READ_INPUT_DWORD(dstate.readPos);
		dstate.readPos++;
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
