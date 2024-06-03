/*
Copyright (c) 2024 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#include "gstd_constants.h"

#include "gstddec_prefix_cpp.h"

#include "gstddec_public_constants.h"
#include "gstddec_fsetab_predefined.h"

//GSTDDEC_MAIN_FUNCTION_DEF(vuint32_t laneIndex)

#define GSTDDEC_NUMERIC_STARTS_VVEC_SIZE ((GSTD_MAX_HUFFMAN_WEIGHT + GSTDDEC_VECTOR_WIDTH) / GSTDDEC_VECTOR_WIDTH)

#define GSTDDEC_LIT_BUFFER_BYTE_SIZE (GSTDDEC_FORMAT_WIDTH * 4)

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressRLEBlock(uint32_t controlWord)
{
	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT DecodeLiteralVector(uint32_t numLanes, vuint32_t codeBits, uint32_t huffmanCodeMask, GSTDDEC_PARAM_INOUT(vuint32_t, inOutDiscardBits))
{
	vuint32_t tableIndex = ((codeBits >> inOutDiscardBits) & GSTDDEC_VECTOR_UINT32(huffmanCodeMask));

	vuint32_t symbolDWordIndex = GSTDDEC_VECTOR_UINT32(0);
	vuint32_t symbolBitPos = GSTDDEC_VECTOR_UINT32(0);
	vuint32_t lengthDWordIndex = GSTDDEC_VECTOR_UINT32(0);
	vuint32_t lengthBitPos = GSTDDEC_VECTOR_UINT32(0);
	HuffmanTableIndexToDecodeTableCell(tableIndex, lengthDWordIndex, lengthBitPos, symbolDWordIndex, symbolBitPos);

	vuint32_t symbol = GSTDDEC_VECTOR_UINT32(0);
	vuint32_t length = GSTDDEC_VECTOR_UINT32(0);

	vuint32_t symbolValue = GSTDDEC_VECTOR_UINT32(0);
	vuint32_t lengthValue = GSTDDEC_VECTOR_UINT32(0);

	GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanes))
	{
		GSTDDEC_CONDITIONAL_LOAD_INDEX(symbolValue, gs_decompressorState.huffmanDecTable, symbolDWordIndex);
		GSTDDEC_CONDITIONAL_LOAD_INDEX(lengthValue, gs_decompressorState.huffmanDecTable, lengthDWordIndex);

		GSTDDEC_CONDITIONAL_STORE(symbol, (symbolValue >> symbolBitPos) & GSTDDEC_VECTOR_UINT32(0xff));
		GSTDDEC_CONDITIONAL_STORE(length, (lengthValue >> lengthBitPos) & GSTDDEC_VECTOR_UINT32(0xf));

		GSTDDEC_CONDITIONAL_STORE(inOutDiscardBits, inOutDiscardBits + length);
	}
	GSTDDEC_VECTOR_END_IF

	return symbol;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT RefillHuffmanLiteralsPartial(uint32_t literalsToRefill, uint32_t huffmanCodeMask, uint32_t passIndex)
{
	// This function refills 0-2 literals into each lane
	uint32_t refillNudge = 3;
	if (passIndex > 0)
		refillNudge -= 2;

	uint32_t dwordsToRefill = (literalsToRefill + refillNudge) / 4u;
	uint32_t vvecToRefill = (dwordsToRefill + GSTDDEC_VECTOR_WIDTH - 1u) / GSTDDEC_VECTOR_WIDTH;

	// Help compiler
	if (GSTDDEC_FORMAT_WIDTH <= GSTDDEC_VECTOR_WIDTH)
	{
		if (dwordsToRefill > 0)
			vvecToRefill = 1;
		else
			vvecToRefill = 0;
	}

	for (uint32_t vvecIndex = 0; vvecIndex < vvecToRefill; vvecIndex++)
	{
		uint32_t vvecFirstLiteral = vvecIndex * GSTDDEC_VECTOR_WIDTH * 4u;
		uint32_t litsToRefillThisVVec = GSTDDEC_MIN(GSTDDEC_VECTOR_WIDTH * 4u, GSTDDEC_MAX(literalsToRefill, vvecFirstLiteral) - vvecFirstLiteral);

		uint32_t numVVec0Refill = (litsToRefillThisVVec + 3u) / 4u;
		uint32_t numVVec1Refill = (litsToRefillThisVVec + 2u) / 4u;
		uint32_t numVVec2Refill = (litsToRefillThisVVec + 1u) / 4u;
		uint32_t numVVec3Refill = (litsToRefillThisVVec + 0u) / 4u;

		uint32_t numRefill0 = numVVec0Refill;
		uint32_t numRefill1 = numVVec1Refill;

		if (passIndex > 0)
		{
			numRefill0 = numVVec2Refill;
			numRefill1 = numVVec3Refill;
		}

		BitstreamPeekNoTruncate(vvecIndex, numRefill0, GSTD_MAX_HUFFMAN_CODE_LENGTH * 2u);

		vuint32_t huffmanCodeBits = GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]);
		vuint32_t discardBits = GSTDDEC_VECTOR_UINT32(0);

		vuint32_t decodedLiterals = DecodeLiteralVector(numRefill0, huffmanCodeBits, huffmanCodeMask, discardBits);
		decodedLiterals = decodedLiterals | (DecodeLiteralVector(numRefill1, huffmanCodeBits, huffmanCodeMask, discardBits) << GSTDDEC_VECTOR_UINT32(8));

		// numRefill0 will always be >= numRefill1, so it is the number of lanes to discard bits from
		BitstreamDiscard(vvecIndex, numRefill0, discardBits);

		if (passIndex == 0)
			g_dstate.literalsBuffer[vvecIndex] = decodedLiterals;
		else
			g_dstate.literalsBuffer[vvecIndex] = g_dstate.literalsBuffer[vvecIndex] | (decodedLiterals << GSTDDEC_VECTOR_UINT32(16));
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT RefillHuffmanLiterals(uint32_t huffmanCodeMask)
{
	uint32_t maxLiteralsToRefill = g_dstate.maxLiterals - g_dstate.numLiteralsEmitted;
	uint32_t literalsToRefill = GSTDDEC_FORMAT_WIDTH * 4;

#if GSTDDEC_SANITIZE
	if (g_dstate.maxLiterals < g_dstate.numLiteralsEmitted)
	{
		GSTDDEC_WARN("Literals read overrun");
		maxLiteralsToRefill = 0;
	}
#endif

	literalsToRefill = GSTDDEC_MIN(literalsToRefill, maxLiteralsToRefill);

	for (uint32_t vvecIndex = 0; vvecIndex < GSTDDEC_VVEC_SIZE; vvecIndex++)
		g_dstate.literalsBuffer[vvecIndex] = GSTDDEC_VECTOR_UINT32(0);

	RefillHuffmanLiteralsPartial(literalsToRefill, huffmanCodeMask, 0);
	RefillHuffmanLiteralsPartial(literalsToRefill, huffmanCodeMask, 1);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT RefillRawLiterals()
{
	uint32_t maxLiteralsToRefill = g_dstate.maxLiterals - g_dstate.numLiteralsEmitted;
	uint32_t literalsToRefill = GSTDDEC_FORMAT_WIDTH * 4;

#if GSTDDEC_SANITIZE
	if (g_dstate.maxLiterals < g_dstate.numLiteralsEmitted)
	{
		GSTDDEC_WARN("Literals read overrun");
		maxLiteralsToRefill = 0;
	}
#endif

	literalsToRefill = GSTDDEC_MIN(literalsToRefill, maxLiteralsToRefill);

	for (uint32_t vvecIndex = 0; vvecIndex < GSTDDEC_VVEC_SIZE; vvecIndex++)
		g_dstate.literalsBuffer[vvecIndex] = GSTDDEC_VECTOR_UINT32(0);

	uint32_t numLanesToRefill = (literalsToRefill + 3) / 4;
	uint32_t numVVecToRefill = (numLanesToRefill + GSTDDEC_VECTOR_WIDTH - 1) / GSTDDEC_VECTOR_WIDTH;

	for (uint32_t vvecIndex = 0; vvecIndex < numVVecToRefill; vvecIndex++)
	{
		uint32_t litRefillStart = vvecIndex * GSTDDEC_VECTOR_WIDTH * 4;
		uint32_t laneStart = vvecIndex * GSTDDEC_VECTOR_WIDTH;

		uint32_t numLanesToRefillThisVVec = GSTDDEC_MIN(GSTDDEC_VECTOR_WIDTH, numLanesToRefill - laneStart);
		vuint32_t laneLitStart = (GSTDDEC_VECTOR_UINT32(vvecIndex * GSTDDEC_VECTOR_WIDTH) + GSTDDEC_LANE_INDEX) << GSTDDEC_VECTOR_UINT32(2);
		vuint32_t litsAvailableInLane = GSTDDEC_MAX(GSTDDEC_VECTOR_UINT32(maxLiteralsToRefill), laneLitStart) - laneLitStart;
		vuint32_t litsToRefillInLane = GSTDDEC_MIN(litsAvailableInLane, GSTDDEC_VECTOR_UINT32(4));

		vuint32_t litsToDropFromLane = GSTDDEC_VECTOR_UINT32(4) - litsToRefillInLane;

		// This produces undefined values for lanes that drop 4 bits, so we need to be careful to ignore those lanes
		vuint32_t litKeepMask = GSTDDEC_VECTOR_UINT32(0xffffffffu) >> (litsToDropFromLane << GSTDDEC_VECTOR_UINT32(3));

		BitstreamPeekNoTruncate(vvecIndex, numLanesToRefillThisVVec, 32);
		vuint32_t litValues = GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]) & litKeepMask;

		BitstreamDiscard(vvecIndex, numLanesToRefillThisVVec, litsToRefillInLane << GSTDDEC_VECTOR_UINT32(3));

		GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToRefillThisVVec))
		{
			GSTDDEC_CONDITIONAL_STORE(g_dstate.literalsBuffer[vvecIndex], litValues);
		}
		GSTDDEC_VECTOR_END_IF
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT EmitLiterals(uint32_t numLiteralsToEmit)
{
	while (numLiteralsToEmit > 0)
	{
		uint32_t litBufferReadPos = (g_dstate.numLiteralsEmitted % GSTDDEC_LIT_BUFFER_BYTE_SIZE);
		uint32_t litBufferVVecIndex = litBufferReadPos / (GSTDDEC_VECTOR_WIDTH * 4u);

		// Help compiler
		if (GSTDDEC_VECTOR_WIDTH >= GSTDDEC_FORMAT_WIDTH)
			litBufferVVecIndex = 0;

		vuint32_t litBufferVector = g_dstate.literalsBuffer[litBufferVVecIndex];

		uint32_t litBufferVectorInternalPos = litBufferReadPos % (GSTDDEC_VECTOR_WIDTH * 4u);

		// This won't go into unused vector space because numLiteralsToEmit is never larger than the lit buffer size
		uint32_t litsToEmitFromVector = GSTDDEC_MIN(numLiteralsToEmit, (GSTDDEC_VECTOR_WIDTH * 4u) - litBufferVectorInternalPos);

		for (uint32_t litOffsetBase = 0; litOffsetBase < litsToEmitFromVector; litOffsetBase += GSTDDEC_VECTOR_WIDTH)
		{
			vuint32_t litOffset = GSTDDEC_LANE_INDEX + GSTDDEC_VECTOR_UINT32(litOffsetBase);

			vuint32_t lit = GSTDDEC_VECTOR_UINT32(0);

			GSTDDEC_VECTOR_IF(litOffset < GSTDDEC_VECTOR_UINT32(litsToEmitFromVector))
			{
				vuint32_t litReadPosByte = GSTDDEC_VECTOR_UINT32(litBufferReadPos) + litOffset;
				vuint32_t litReadLaneIndex = GSTDDEC_VECTOR_UINT32(0);
				vuint32_t litReadBitPos = GSTDDEC_VECTOR_UINT32(0);
				uint32_t litReadMask = 0;

				ResolvePackedAddress8(litReadPosByte, litReadLaneIndex, litReadBitPos, litReadMask);

				vuint32_t lit = GSTDDEC_VECTOR_CONDITIONAL_READ_FROM_INDEX(litBufferVector, litReadLaneIndex);
				lit = (lit >> litReadBitPos) & GSTDDEC_VECTOR_UINT32(litReadMask);

				vuint32_t litWritePosByte = GSTDDEC_VECTOR_UINT32(g_dstate.writePosByte) + litOffset;

				vuint32_t litWriteDWordIndex = GSTDDEC_VECTOR_UINT32(0);
				vuint32_t litWriteBitPos = GSTDDEC_VECTOR_UINT32(0);
				uint32_t litWriteMask = 0;

				ResolvePackedAddress8(litWritePosByte, litWriteDWordIndex, litWriteBitPos, litWriteMask);

				vuint32_t litWriteValue = (lit & GSTDDEC_VECTOR_UINT32(litWriteMask)) << litWriteBitPos;

				InterlockedOrOutputDWord(GSTDDEC_CALL_EXECUTION_MASK litWriteDWordIndex, litWriteValue);
			}
			GSTDDEC_VECTOR_END_IF
		}

		numLiteralsToEmit -= litsToEmitFromVector;
		g_dstate.writePosByte += litsToEmitFromVector;
		g_dstate.numLiteralsEmitted += litsToEmitFromVector;
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLiteralsToTarget(uint32_t targetLiteralsEmitted, uint32_t litSectionType, uint32_t huffmanCodeMask)
{
	GSTDDEC_BRANCH_HINT
	if (litSectionType == GSTD_LITERALS_SECTION_TYPE_RLE)
	{
		uint32_t lit = g_dstate.litRLEByte;

		while (g_dstate.numLiteralsEmitted < targetLiteralsEmitted)
		{
			uint32_t literalsToEmit = GSTDDEC_MIN(GSTDDEC_VECTOR_WIDTH, targetLiteralsEmitted - g_dstate.numLiteralsEmitted);

			GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(literalsToEmit))
			{
				vuint32_t litWritePosByte = GSTDDEC_VECTOR_UINT32(g_dstate.writePosByte) + GSTDDEC_LANE_INDEX;

				vuint32_t litWriteDWordIndex = GSTDDEC_VECTOR_UINT32(0);
				vuint32_t litWriteBitPos = GSTDDEC_VECTOR_UINT32(0);
				uint32_t litWriteMask = 0;

				ResolvePackedAddress8(litWritePosByte, litWriteDWordIndex, litWriteBitPos, litWriteMask);

				vuint32_t litWriteValue = (GSTDDEC_VECTOR_UINT32(lit) & GSTDDEC_VECTOR_UINT32(litWriteMask)) << litWriteBitPos;

				InterlockedOrOutputDWord(GSTDDEC_CALL_EXECUTION_MASK litWriteDWordIndex, litWriteValue);
			}
			GSTDDEC_VECTOR_END_IF
			
			g_dstate.writePosByte += literalsToEmit;
			g_dstate.numLiteralsEmitted += literalsToEmit;
		}

		GSTDDEC_WARN("NOT TESTED");
	}
	else
	{
		while (g_dstate.numLiteralsEmitted < targetLiteralsEmitted)
		{
			uint32_t literalsInBufferAfterRefill = GSTDDEC_LIT_BUFFER_BYTE_SIZE - (g_dstate.numLiteralsEmitted % GSTDDEC_LIT_BUFFER_BYTE_SIZE);

			// The literals buffer is never completely full after an emit, so if this condition is true, then
			// the literals buffer was completely drained and must be refilled.
			if (literalsInBufferAfterRefill == GSTDDEC_LIT_BUFFER_BYTE_SIZE)
			{
				GSTDDEC_BRANCH_HINT
				if (litSectionType == GSTD_LITERALS_SECTION_TYPE_HUFFMAN)
					RefillHuffmanLiterals(huffmanCodeMask);
				else
					RefillRawLiterals();
			}

			uint32_t literalsToEmit = GSTDDEC_MIN(literalsInBufferAfterRefill, targetLiteralsEmitted - g_dstate.numLiteralsEmitted);

			EmitLiterals(literalsToEmit);
		}
	}

	GSTDDEC_FLUSH_OUTPUT;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ExecuteMatchCopy(uint32_t matchLength, uint32_t matchOffset)
{
#if GSTDDEC_SANITIZE
	if (matchOffset > g_dstate.writePosByte)
	{
		GSTDDEC_WARN("Match offset was out of range");
	}

	matchOffset = GSTDDEC_MIN(matchOffset, g_dstate.writePosByte);

	if (matchOffset == 0)
		return;
#endif

	uint32_t copySourceBaseAddress = g_dstate.writePosByte - matchOffset;

	for (uint32_t readOffset = 0; readOffset < matchLength; readOffset += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t offsetFromStart = GSTDDEC_VECTOR_UINT32(readOffset) + GSTDDEC_LANE_INDEX;

		vuint32_t readBytePos = GSTDDEC_VECTOR_UINT32(copySourceBaseAddress) + (offsetFromStart % matchOffset);

		vuint32_t byteReadDWordIndex = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t byteReadBitPos = GSTDDEC_VECTOR_UINT32(0);
		uint32_t byteReadMask = 0;

		ResolvePackedAddress8(readBytePos, byteReadDWordIndex, byteReadBitPos, byteReadMask);

		vuint32_t byteWriteDWordIndex = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t byteWriteBitPos = GSTDDEC_VECTOR_UINT32(0);
		uint32_t byteWriteMask = 0;

		ResolvePackedAddress8(GSTDDEC_VECTOR_UINT32(g_dstate.writePosByte) + offsetFromStart, byteWriteDWordIndex, byteWriteBitPos, byteWriteMask);

		GSTDDEC_VECTOR_IF(offsetFromStart < GSTDDEC_VECTOR_UINT32(matchLength))
		{
			vuint32_t inputByte = (GSTDDEC_CONDITIONAL_READ_OUTPUT_DWORD(byteReadDWordIndex) >> byteReadBitPos) & GSTDDEC_VECTOR_UINT32(byteReadMask);
			vuint32_t outputDWord = (inputByte & GSTDDEC_VECTOR_UINT32(byteWriteMask)) << byteWriteBitPos;

			InterlockedOrOutputDWord(GSTDDEC_CALL_EXECUTION_MASK byteWriteDWordIndex, outputDWord);
		}
		GSTDDEC_VECTOR_END_IF
	}

	g_dstate.writePosByte += matchLength;

	GSTDDEC_FLUSH_OUTPUT;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeAndExecuteSequences(uint32_t litSectionType, uint32_t huffmanCodeMask)
{
	vuint32_t litLengthValues[GSTDDEC_VVEC_SIZE];
	vuint32_t matchLengthValues[GSTDDEC_VVEC_SIZE];
	vuint32_t offsetValues[GSTDDEC_VVEC_SIZE];

	//uint32_t litSectionType = ((controlWord >> GSTD_CONTROL_LIT_SECTION_TYPE_OFFSET) & GSTD_CONTROL_LIT_SECTION_TYPE_MASK);
	//uint32_t litLengthsMode = ((controlWord >> GSTD_CONTROL_LIT_LENGTH_MODE_OFFSET) & GSTD_CONTROL_LIT_LENGTH_MODE_MASK);
	//uint32_t offsetsMode = ((controlWord >> GSTD_CONTROL_OFFSET_MODE_OFFSET) & GSTD_CONTROL_OFFSET_MODE_MASK);
	//uint32_t matchLengthsMode = ((controlWord >> GSTD_CONTROL_MATCH_LENGTH_MODE_OFFSET) & GSTD_CONTROL_MATCH_LENGTH_MODE_MASK);

	uint32_t numSequences = 0;
	
	numSequences = ReadPackedSize();

	for (uint32_t i = 0; i < GSTDDEC_VVEC_SIZE; i++)
	{
		litLengthValues[i] = GSTDDEC_VECTOR_UINT32(0);
		matchLengthValues[i] = GSTDDEC_VECTOR_UINT32(0);
		offsetValues[i] = GSTDDEC_VECTOR_UINT32(0);
	}

	int prevSequenceNumber = 0;

	for (uint32_t firstSequence = 0; firstSequence < numSequences; firstSequence += GSTDDEC_FORMAT_WIDTH)
	{
		uint32_t numValuesToRefill = GSTDDEC_MIN(numSequences - firstSequence, GSTDDEC_FORMAT_WIDTH);
		uint32_t vvecToRefill = (numValuesToRefill + (GSTDDEC_VECTOR_WIDTH - 1)) / GSTDDEC_VECTOR_WIDTH;

		// Help compiler in full SIMD case
		if (GSTDDEC_VECTOR_WIDTH >= GSTDDEC_FORMAT_WIDTH)
			vvecToRefill = 1;

		for (uint32_t vvecIndex = 0; vvecIndex < vvecToRefill; vvecIndex++)
		{
			uint32_t firstValueOffset = vvecIndex * GSTDDEC_VECTOR_WIDTH;
			uint32_t lanesToLoad = GSTDDEC_MIN(numValuesToRefill - firstValueOffset, GSTDDEC_VECTOR_WIDTH);
			
			BitstreamPeekNoTruncate(vvecIndex, lanesToLoad, GSTD_MAX_ACCURACY_LOG * 3);

			vuint32_t litLengthCode = DecodeFSEValueNoPeek(lanesToLoad, vvecIndex, g_dstate.litLengthAccuracyLog, GSTDDEC_FSETAB_LIT_LENGTH_START);
			vuint32_t matchLengthCode = DecodeFSEValueNoPeek(lanesToLoad, vvecIndex, g_dstate.matchLengthAccuracyLog, GSTDDEC_FSETAB_MATCH_LENGTH_START);
			vuint32_t offsetCode = DecodeFSEValueNoPeek(lanesToLoad, vvecIndex, g_dstate.offsetAccuracyLog, GSTDDEC_FSETAB_OFFSET_START);

			vuint32_t litLengthBits = GSTDDEC_VECTOR_UINT32(0);
			vuint32_t litLengthBase = litLengthCode;

			GSTDDEC_VECTOR_IF(litLengthCode >= GSTDDEC_VECTOR_UINT32(16))
			{
				GSTDDEC_VECTOR_IF_NESTED(litLengthCode <= GSTDDEC_VECTOR_UINT32(24))
				{
					// Weird math that replicates the Zstd lit length code table for 16..24
					vuint32_t offsetFrom16 = litLengthCode - GSTDDEC_VECTOR_UINT32(16);
					GSTDDEC_CONDITIONAL_STORE(litLengthBits, GSTDDEC_MAX(GSTDDEC_VECTOR_UINT32(1), offsetFrom16 >> GSTDDEC_VECTOR_UINT32(1)));
					vuint32_t baselineNudge = GSTDDEC_MIN(offsetFrom16, GSTDDEC_VECTOR_UINT32(2) ^ (litLengthCode & GSTDDEC_VECTOR_UINT32(1)));
					GSTDDEC_CONDITIONAL_STORE(litLengthBase, (baselineNudge << litLengthBits) + GSTDDEC_VECTOR_UINT32(16));
				}
				GSTDDEC_VECTOR_ELSE_NESTED
				{
					GSTDDEC_CONDITIONAL_STORE(litLengthBits, litLengthCode - GSTDDEC_VECTOR_UINT32(19));
					GSTDDEC_CONDITIONAL_STORE(litLengthBase, GSTDDEC_VECTOR_UINT32(1) << litLengthBits);
				}
				GSTDDEC_VECTOR_END_IF_NESTED
			}
			GSTDDEC_VECTOR_END_IF

			vuint32_t matchLengthBits = GSTDDEC_VECTOR_UINT32(0);
			vuint32_t matchLengthBaseMinus3 = matchLengthCode;

			GSTDDEC_VECTOR_IF(matchLengthCode >= GSTDDEC_VECTOR_UINT32(32))
			{
				GSTDDEC_VECTOR_IF_NESTED(matchLengthCode <= GSTDDEC_VECTOR_UINT32(42))
				{
					// Weird math that replicates the Zstd match length code table for 32..42
					vuint32_t offsetFrom32 = matchLengthCode - GSTDDEC_VECTOR_UINT32(32);
					GSTDDEC_CONDITIONAL_STORE(matchLengthBits, GSTDDEC_MAX(GSTDDEC_VECTOR_UINT32(1), offsetFrom32 >> GSTDDEC_VECTOR_UINT32(1)));
					vuint32_t baselineNudge = GSTDDEC_MIN(offsetFrom32, GSTDDEC_VECTOR_UINT32(2) ^ (matchLengthCode & GSTDDEC_VECTOR_UINT32(1)));
					GSTDDEC_CONDITIONAL_STORE(matchLengthBaseMinus3, (baselineNudge << matchLengthBits) + GSTDDEC_VECTOR_UINT32(32));
				}
				GSTDDEC_VECTOR_ELSE_NESTED
				{
					GSTDDEC_CONDITIONAL_STORE(matchLengthBits, matchLengthCode - GSTDDEC_VECTOR_UINT32(36));
					GSTDDEC_CONDITIONAL_STORE(matchLengthBaseMinus3, GSTDDEC_VECTOR_UINT32(1) << matchLengthBits);
				}
				GSTDDEC_VECTOR_END_IF_NESTED
			}
			GSTDDEC_VECTOR_END_IF

			BitstreamPeekNoTruncate(vvecIndex, lanesToLoad, GSTD_MAX_LIT_LENGTH_EXTRA_BITS + GSTD_MAX_MATCH_LENGTH_EXTRA_BITS);

			// Lit length
			vuint32_t litLengthBitsMask = (GSTDDEC_VECTOR_UINT32(1) << litLengthBits) - GSTDDEC_VECTOR_UINT32(1);
			litLengthValues[vvecIndex] = litLengthBase + (GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]) & litLengthBitsMask);

			BitstreamDiscard(vvecIndex, lanesToLoad, litLengthBits);

			// Match length
			vuint32_t matchLengthBitsMask = (GSTDDEC_VECTOR_UINT32(1) << matchLengthBits) - GSTDDEC_VECTOR_UINT32(1);
			matchLengthValues[vvecIndex] = matchLengthBaseMinus3 + GSTDDEC_VECTOR_UINT32(3) + (GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]) & matchLengthBitsMask);

			BitstreamDiscard(vvecIndex, lanesToLoad, matchLengthBits);

			BitstreamPeekNoTruncate(vvecIndex, lanesToLoad, GSTD_MAX_OFFSET_CODE);

			vuint32_t offsetBitsMask = (GSTDDEC_VECTOR_UINT32(1) << offsetCode) - GSTDDEC_VECTOR_UINT32(1);
			offsetValues[vvecIndex] = (GSTDDEC_VECTOR_UINT32(1) << offsetCode) + (GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]) & offsetBitsMask);

			BitstreamDiscard(vvecIndex, lanesToLoad, offsetCode);
		}

		for (uint32_t vvecIndex = 0; vvecIndex < vvecToRefill; vvecIndex++)
		{
			uint32_t firstValueOffset = vvecIndex * GSTDDEC_VECTOR_WIDTH;
			uint32_t numSequencesToExecute = GSTDDEC_MIN(numValuesToRefill - firstValueOffset, GSTDDEC_VECTOR_WIDTH);

			for (uint32_t seqLaneIndex = 0; seqLaneIndex < numSequencesToExecute; seqLaneIndex++)
			{
				int sequenceNumber = prevSequenceNumber++;

				uint32_t litLengthValue = GSTDDEC_VECTOR_READ_FROM_INDEX(litLengthValues[vvecIndex], seqLaneIndex);
				uint32_t matchLength = GSTDDEC_VECTOR_READ_FROM_INDEX(matchLengthValues[vvecIndex], seqLaneIndex);
				uint32_t offsetValue = GSTDDEC_VECTOR_READ_FROM_INDEX(offsetValues[vvecIndex], seqLaneIndex);

				uint32_t realOffset = 0;
				if (offsetValue <= 3)
				{
					uint32_t offsetBehavior = offsetValue;
					if (litLengthValue != 0)
						offsetBehavior--;

					if (offsetBehavior == 0)
					{
						// RepeatedOffset1
						realOffset = g_dstate.repeatedOffset1;
					}
					else if (offsetBehavior == 1)
					{
						// RepeatedOffset2
						realOffset = g_dstate.repeatedOffset2;
						g_dstate.repeatedOffset2 = g_dstate.repeatedOffset1;
						g_dstate.repeatedOffset1 = realOffset;
					}
					else
					{
						// RepeatedOffset3 or RepeatedOffset1 - 1
						if (offsetBehavior == 2)
							realOffset = g_dstate.repeatedOffset3;
						else
						{
							realOffset = g_dstate.repeatedOffset1 - 1;

#if GSTDDEC_SANITIZE
							if (realOffset == 0)
							{
								GSTDDEC_WARN("RepeatedOffset1-1 was 0");
							}
							realOffset = GSTDDEC_MAX(1, realOffset);
#endif
						}

						g_dstate.repeatedOffset3 = g_dstate.repeatedOffset2;
						g_dstate.repeatedOffset2 = g_dstate.repeatedOffset1;
						g_dstate.repeatedOffset1 = realOffset;
					}
				}
				else
				{
					realOffset = offsetValue - 3;
					g_dstate.repeatedOffset3 = g_dstate.repeatedOffset2;
					g_dstate.repeatedOffset2 = g_dstate.repeatedOffset1;
					g_dstate.repeatedOffset1 = realOffset;
				}

				GSTDDEC_DIAGNOSTIC("Sequence %i: Lit length %u  Match length %u  Offset code %u\n", sequenceNumber, litLengthValue, matchLength, offsetValue);

				GSTDDEC_BRANCH_HINT
				if (litLengthValue > 0)
					DecodeLiteralsToTarget(g_dstate.numLiteralsEmitted + litLengthValue, litSectionType, huffmanCodeMask);

				ExecuteMatchCopy(matchLength, realOffset);
			}
		}
	}
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressRawBlock(uint32_t controlWord)
{
	uint32_t decompressedSize = (controlWord >> GSTD_CONTROL_DECOMPRESSED_SIZE_OFFSET) & GSTD_CONTROL_DECOMPRESSED_SIZE_MASK;
	uint32_t auxByte = (controlWord >> GSTD_CONTROL_RAW_FIRST_BYTE_OFFSET) & GSTD_CONTROL_RAW_FIRST_BYTE_MASK;

	GSTDDEC_WARN("NOT YET IMPLEMENTED");
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecompressCompressedBlock(uint32_t controlWord)
{
	uint32_t litSectionType = ((controlWord >> GSTD_CONTROL_LIT_SECTION_TYPE_OFFSET) & GSTD_CONTROL_LIT_SECTION_TYPE_MASK);
	uint32_t litLengthsMode = ((controlWord >> GSTD_CONTROL_LIT_LENGTH_MODE_OFFSET) & GSTD_CONTROL_LIT_LENGTH_MODE_MASK);
	uint32_t offsetsMode = ((controlWord >> GSTD_CONTROL_OFFSET_MODE_OFFSET) & GSTD_CONTROL_OFFSET_MODE_MASK);
	uint32_t matchLengthsMode = ((controlWord >> GSTD_CONTROL_MATCH_LENGTH_MODE_OFFSET) & GSTD_CONTROL_MATCH_LENGTH_MODE_MASK);
	uint32_t auxBit = ((controlWord >> GSTD_CONTROL_AUX_BIT_OFFSET) & GSTD_CONTROL_AUX_BIT_MASK);

	uint32_t regeneratedSize = 0;

	uint32_t huffmanCodeMask = 0;

	uint32_t numSequences = 0;
	uint32_t fseTableAccuracyByte = 0;
	
	g_dstate.maxLiterals = ReadPackedSize();

	GSTDDEC_BRANCH_HINT
	if (litSectionType == GSTD_LITERALS_SECTION_TYPE_HUFFMAN)
	{
		uint32_t finalWeightTotal = 0;
		DecodeLitHuffmanTree(auxBit, finalWeightTotal);
		huffmanCodeMask = finalWeightTotal - 1;
	}
	else
	{
		GSTDDEC_BRANCH_HINT
		if (litSectionType == GSTD_LITERALS_SECTION_TYPE_RLE)
		{
			DecodeLitRLEByte();
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
		else
		{
			GSTDDEC_BRANCH_HINT
			if (litSectionType == GSTD_LITERALS_SECTION_TYPE_HUFFMAN_REUSE)
			{
				GSTDDEC_WARN("NOT YET IMPLEMENTED");
			}
			// else if (litSectioNType == GSTD_LITERALS_SECTION_TYPE_RAW)
			//{
			//	Nothing to do in this case
			//}
		}
	}

	GSTDDEC_BRANCH_HINT
	if (offsetsMode == GSTD_SEQ_COMPRESSION_MODE_FSE || litLengthsMode == GSTD_SEQ_COMPRESSION_MODE_FSE || matchLengthsMode == GSTD_SEQ_COMPRESSION_MODE_FSE)
	{
		fseTableAccuracyByte = ReadRawByte();
	}

	// Decode trees
	GSTDDEC_BRANCH_HINT
	if (offsetsMode == GSTD_SEQ_COMPRESSION_MODE_FSE)
	{
		DecodeFSETable(GSTDDEC_FSETAB_OFFSET_START, GSTD_MAX_OFFSET_CODE, ((fseTableAccuracyByte >> GSTD_ACCURACY_BYTE_OFFSET_POS) & GSTD_ACCURACY_BYTE_OFFSET_MASK) + GSTD_MIN_ACCURACY_LOG, GSTD_MAX_OFFSET_ACCURACY_LOG, g_dstate.offsetAccuracyLog);
	}
	else
	{
		GSTDDEC_BRANCH_HINT
		if (offsetsMode == GSTD_SEQ_COMPRESSION_MODE_PREDEFINED)
		{
			uint32_t numCells = 1 << GSTDDEC_PREDEFINED_OFFSET_CODE_ACCURACY_LOG;

			for (uint32_t firstCell = 0; firstCell < numCells; firstCell += GSTDDEC_VECTOR_WIDTH)
			{
				vuint32_t cell = GSTDDEC_VECTOR_UINT32(firstCell) + GSTDDEC_LANE_INDEX;

				GSTDDEC_VECTOR_IF(cell < GSTDDEC_VECTOR_UINT32(numCells))
				{
					vuint32_t cellData = GSTDDEC_VECTOR_UINT32(0);
					GSTDDEC_CONDITIONAL_LOAD_INDEX(cellData, kPredefinedOffsetCodeTable, cell);
					GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.fseCells, cell + GSTDDEC_VECTOR_UINT32(GSTDDEC_FSETAB_OFFSET_START), cellData);
				}
				GSTDDEC_VECTOR_END_IF
			}

			g_dstate.offsetAccuracyLog = GSTDDEC_PREDEFINED_OFFSET_CODE_ACCURACY_LOG;
		}
		else
		{
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
	}

	GSTDDEC_BRANCH_HINT
	if (matchLengthsMode == GSTD_SEQ_COMPRESSION_MODE_FSE)
	{
		DecodeFSETable(GSTDDEC_FSETAB_MATCH_LENGTH_START, GSTD_MAX_MATCH_LENGTH_CODE, ((fseTableAccuracyByte >> GSTD_ACCURACY_BYTE_MATCH_LENGTH_POS) & GSTD_ACCURACY_BYTE_MATCH_LENGTH_MASK) + GSTD_MIN_ACCURACY_LOG, GSTD_MAX_MATCH_LENGTH_ACCURACY_LOG, g_dstate.matchLengthAccuracyLog);
	}
	else
	{
		GSTDDEC_BRANCH_HINT
		if (matchLengthsMode == GSTD_SEQ_COMPRESSION_MODE_PREDEFINED)
		{
			uint32_t numCells = 1 << GSTDDEC_PREDEFINED_MATCH_LENGTH_ACCURACY_LOG;

			for (uint32_t firstCell = 0; firstCell < numCells; firstCell += GSTDDEC_VECTOR_WIDTH)
			{
				vuint32_t cell = GSTDDEC_VECTOR_UINT32(firstCell) + GSTDDEC_LANE_INDEX;

				GSTDDEC_VECTOR_IF(cell < GSTDDEC_VECTOR_UINT32(numCells))
				{
					vuint32_t cellData = GSTDDEC_VECTOR_UINT32(0);
					GSTDDEC_CONDITIONAL_LOAD_INDEX(cellData, kPredefinedMatchLengthTable, cell);
					GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.fseCells, cell + GSTDDEC_VECTOR_UINT32(GSTDDEC_FSETAB_MATCH_LENGTH_START), cellData);
				}
				GSTDDEC_VECTOR_END_IF
			}

			g_dstate.matchLengthAccuracyLog = GSTDDEC_PREDEFINED_MATCH_LENGTH_ACCURACY_LOG;
		}
		else
		{
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
	}

	GSTDDEC_BRANCH_HINT
	if (litLengthsMode == GSTD_SEQ_COMPRESSION_MODE_FSE)
	{
		DecodeFSETable(GSTDDEC_FSETAB_LIT_LENGTH_START, GSTD_MAX_LIT_LENGTH_CODE, ((fseTableAccuracyByte >> GSTD_ACCURACY_BYTE_LIT_LENGTH_POS) & GSTD_ACCURACY_BYTE_LIT_LENGTH_MASK) + GSTD_MIN_ACCURACY_LOG, GSTD_MAX_LIT_LENGTH_ACCURACY_LOG, g_dstate.litLengthAccuracyLog);
	}
	else
	{
		GSTDDEC_BRANCH_HINT
		if (litLengthsMode == GSTD_SEQ_COMPRESSION_MODE_PREDEFINED)
		{
			uint32_t numCells = 1 << GSTDDEC_PREDEFINED_LIT_LENGTH_ACCURACY_LOG;

			for (uint32_t firstCell = 0; firstCell < numCells; firstCell += GSTDDEC_VECTOR_WIDTH)
			{
				vuint32_t cell = GSTDDEC_VECTOR_UINT32(firstCell) + GSTDDEC_LANE_INDEX;

				GSTDDEC_VECTOR_IF(cell < GSTDDEC_VECTOR_UINT32(numCells))
				{
					vuint32_t cellData = GSTDDEC_VECTOR_UINT32(0);
					GSTDDEC_CONDITIONAL_LOAD_INDEX(cellData, kPredefinedLitLengthTable, cell);
					GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.fseCells, cell + GSTDDEC_VECTOR_UINT32(GSTDDEC_FSETAB_LIT_LENGTH_START), cellData);
				}
				GSTDDEC_VECTOR_END_IF
			}

			g_dstate.litLengthAccuracyLog = GSTDDEC_PREDEFINED_LIT_LENGTH_ACCURACY_LOG;
		}
		else
		{
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
	}

	GSTDDEC_FLUSH_GS;

	DecodeAndExecuteSequences(litSectionType, huffmanCodeMask);

	// Flush trailing literals
	GSTDDEC_BRANCH_HINT
	if (g_dstate.numLiteralsEmitted < g_dstate.maxLiterals)
		DecodeLiteralsToTarget(g_dstate.maxLiterals, litSectionType, huffmanCodeMask);
	else
	{
		if (g_dstate.numLiteralsEmitted > g_dstate.maxLiterals)
		{
			GSTDDEC_WARN("Too many literals emitted");
		}
	}
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT DecodeFSEValueNoPeek(uint32_t numLanesToRefill, uint32_t vvecIndex, uint32_t accuracyLog, uint32_t firstCell)
{
	vuint32_t state = g_dstate.fseState[vvecIndex];

	uint32_t accuracyLogMask = (1 << accuracyLog) - 1;
	vuint32_t symbol = GSTDDEC_VECTOR_UINT32(0);

	vuint32_t preDiscardBits = GSTDDEC_DEMOTE_UINT64_TO_UINT32(g_dstate.bitstreamBits[vvecIndex]);

	BitstreamDiscard(vvecIndex, numLanesToRefill, g_dstate.fseDrainLevel[vvecIndex]);

	GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToRefill))
	{
		vuint32_t numBitsToRefill = g_dstate.fseDrainLevel[vvecIndex];
		vuint32_t bitsRefillMask = (GSTDDEC_VECTOR_UINT32(1) << numBitsToRefill) - GSTDDEC_VECTOR_UINT32(1);

		vuint32_t refilledState = state + (preDiscardBits & bitsRefillMask);
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
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT DecodeFSEValue(uint32_t numLanesToRefill, uint32_t vvecIndex, uint32_t accuracyLog, uint32_t firstCell)
{
	BitstreamPeekNoTruncate(vvecIndex, numLanesToRefill, GSTD_MAX_ACCURACY_LOG);
	return DecodeFSEValueNoPeek(numLanesToRefill, vvecIndex, accuracyLog, firstCell);
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
void GSTDDEC_FUNCTION_CONTEXT ResolvePackedAddress4(vuint32_t index, GSTDDEC_PARAM_OUT(vuint32_t, outDWordIndex), GSTDDEC_PARAM_OUT(vuint32_t, outBitPos), GSTDDEC_PARAM_OUT(uint32_t, outMask))
{
	outDWordIndex = index >> GSTDDEC_VECTOR_UINT32(3);
	outBitPos = (index & GSTDDEC_VECTOR_UINT32(7)) << GSTDDEC_VECTOR_UINT32(2);
	outMask = 0xf;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ResolvePackedAddress8(vuint32_t index, GSTDDEC_PARAM_OUT(vuint32_t, outDWordIndex), GSTDDEC_PARAM_OUT(vuint32_t, outBitPos), GSTDDEC_PARAM_OUT(uint32_t, outMask))
{
	outDWordIndex = index >> GSTDDEC_VECTOR_UINT32(2);
	outBitPos = (index & GSTDDEC_VECTOR_UINT32(3)) << GSTDDEC_VECTOR_UINT32(3);
	outMask = 0xff;
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT HuffmanTableIndexToDecodeTableCell(vuint32_t tableIndex, GSTDDEC_PARAM_OUT(vuint32_t, lengthDWordIndex), GSTDDEC_PARAM_OUT(vuint32_t, lengthBitPos), GSTDDEC_PARAM_OUT(vuint32_t, symbolDWordIndex), GSTDDEC_PARAM_OUT(vuint32_t, symbolBitPos))
{
	vuint32_t clusterPos = (tableIndex >> GSTDDEC_VECTOR_UINT32(3)) * GSTDDEC_VECTOR_UINT32(3);
	vuint32_t clusterInternalOffset = (tableIndex & GSTDDEC_VECTOR_UINT32(7));

	symbolDWordIndex = clusterPos + GSTDDEC_VECTOR_UINT32(1) + (clusterInternalOffset >> GSTDDEC_VECTOR_UINT32(2));
	symbolBitPos = (tableIndex & GSTDDEC_VECTOR_UINT32(3)) << GSTDDEC_VECTOR_UINT32(3);

	lengthDWordIndex = clusterPos;
	lengthBitPos = clusterInternalOffset << GSTDDEC_VECTOR_UINT32(2);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT StoreHuffmanLookupCodes(GSTDDEC_PARAM_EXECUTION_MASK vuint32_t tableIndex, vuint32_t symbol, uint32_t length)
{
	vbool_t isInBounds = GSTDDEC_VECTOR_BOOL(true);

#if GSTDDEC_SANITIZE
	GSTDDEC_VECTOR_IF_NESTED(tableIndex >= GSTDDEC_VECTOR_UINT32(1 << GSTD_MAX_HUFFMAN_CODE_LENGTH))
	{
		GSTDDEC_CONDITIONAL_STORE(isInBounds, GSTDDEC_VECTOR_BOOL(false));
	}
	GSTDDEC_VECTOR_END_IF_NESTED

	if (!GSTDDEC_VECTOR_ALL(isInBounds))
	{
		GSTDDEC_WARN("Huffman symbol index was out of range");
	}
	length = GSTDDEC_MIN(length, GSTD_MAX_HUFFMAN_CODE_LENGTH);
#endif

	GSTDDEC_VECTOR_IF_NESTED(isInBounds)
	{
		vuint32_t lengthDWordIndex = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t lengthBitPos = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t symbolDWordIndex = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t symbolBitPos = GSTDDEC_VECTOR_UINT32(0);
		HuffmanTableIndexToDecodeTableCell(tableIndex, lengthDWordIndex, lengthBitPos, symbolDWordIndex, symbolBitPos);

		GSTDDEC_CONDITIONAL_OR_INDEX(gs_decompressorState.huffmanDecTable, lengthDWordIndex, GSTDDEC_VECTOR_UINT32(length) << lengthBitPos);
		GSTDDEC_CONDITIONAL_OR_INDEX(gs_decompressorState.huffmanDecTable, symbolDWordIndex, symbol << symbolBitPos);
	}
	GSTDDEC_VECTOR_END_IF_NESTED

	for (uint32_t i = 0; i < GSTDDEC_VECTOR_WIDTH; i++)
	{
		uint32_t ti = tableIndex.Get(i);
		if (ti < (1 << GSTD_MAX_HUFFMAN_CODE_LENGTH))
		{
			m_huffmanDebug[ti].m_symbol = symbol.Get(i);
			m_huffmanDebug[ti].m_length = length;
		}
	}
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
void GSTDDEC_FUNCTION_CONTEXT ExpandLitHuffmanTable(uint32_t numSpecifiedWeights, uint32_t weightTotal, GSTDDEC_PARAM_OUT(uint32_t, outWeightTotal))
{
	uint32_t weightBits = GSTDDEC_NEXT_LOG2_POWER(weightTotal);

	// FIXME CRITICAL: Check for weightTotal == 0
#if GSTDDEC_SANITIZE
	if (weightBits > GSTD_MAX_HUFFMAN_CODE_LENGTH)
	{
		GSTDDEC_WARN("Maximum Huffman code length exceeded");
	}
#endif

	uint32_t remainingBitSpace = (1 << weightBits) - weightTotal;
	uint32_t unspecifiedWeight = GSTDDEC_NEXT_LOG2_POWER(remainingBitSpace);

	for (uint32_t firstPackedWeightCount = 0; firstPackedWeightCount < GSTDDEC_NUM_PACKED_HUFFMAN_WEIGHT_DWORDS; firstPackedWeightCount += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t writeIndex = GSTDDEC_VECTOR_UINT32(firstPackedWeightCount) + GSTDDEC_LANE_INDEX;

		GSTDDEC_VECTOR_IF(writeIndex < GSTDDEC_VECTOR_UINT32(GSTDDEC_NUM_PACKED_HUFFMAN_WEIGHT_DWORDS))
		{
			GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.packedHuffmanWeightCounts, writeIndex, GSTDDEC_VECTOR_UINT32(0));
		}
		GSTDDEC_VECTOR_END_IF
	}

	uint32_t numEncodedClusters = ((1 << GSTDDEC_MIN(weightBits, GSTD_MAX_HUFFMAN_CODE_LENGTH)) + 3) / 4;
	uint32_t numDecodeTableDWords = numEncodedClusters * 3;

	for (uint32_t firstPackedDecDWord = 0; firstPackedDecDWord < numDecodeTableDWords; firstPackedDecDWord += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t writeIndex = GSTDDEC_VECTOR_UINT32(firstPackedDecDWord) + GSTDDEC_LANE_INDEX;

		GSTDDEC_VECTOR_IF(writeIndex < GSTDDEC_VECTOR_UINT32(numDecodeTableDWords))
		{
			GSTDDEC_CONDITIONAL_STORE_INDEX(gs_decompressorState.huffmanDecTable, writeIndex, GSTDDEC_VECTOR_UINT32(0));
		}
		GSTDDEC_VECTOR_END_IF
	}

	GSTDDEC_FLUSH_GS;

	for (uint32_t firstPackedWeight = 0; firstPackedWeight < numSpecifiedWeights; firstPackedWeight += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t weightIndex = GSTDDEC_VECTOR_UINT32(firstPackedWeight) + GSTDDEC_LANE_INDEX;

		GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
		{
			vuint32_t readIndex = (weightIndex >> GSTDDEC_VECTOR_UINT32(3));
			vuint32_t bitPos = (weightIndex & GSTDDEC_VECTOR_UINT32(7)) << GSTDDEC_VECTOR_UINT32(2);

			vuint32_t packedWeight = GSTDDEC_VECTOR_UINT32(0);
			GSTDDEC_CONDITIONAL_LOAD_INDEX(packedWeight, gs_decompressorState.packedHuffmanWeights, readIndex);
			packedWeight = (packedWeight >> bitPos) & GSTDDEC_VECTOR_UINT32(15);

			vuint32_t incIndex = (packedWeight >> GSTDDEC_VECTOR_UINT32(2));
			vuint32_t incValue = (GSTDDEC_VECTOR_UINT32(1) << ((packedWeight & GSTDDEC_VECTOR_UINT32(3)) << GSTDDEC_VECTOR_UINT32(3)));

			GSTDDEC_CONDITIONAL_ADD_INDEX(gs_decompressorState.packedHuffmanWeightCounts, incIndex, incValue);
		}
		GSTDDEC_VECTOR_END_IF
	}

	GSTDDEC_FLUSH_GS;

	// Find the numeric range start positions

	// NOTE: This skips weight 0!
	vuint32_t weightNumericStarts[GSTDDEC_NUMERIC_STARTS_VVEC_SIZE];

	for (uint32_t wneIndex = 0; wneIndex < GSTDDEC_NUMERIC_STARTS_VVEC_SIZE; wneIndex++)
		weightNumericStarts[wneIndex] = GSTDDEC_VECTOR_UINT32(0);

	uint32_t weightStartRunningIterator = 0;

	GSTDDEC_UNROLL_HINT
	for (uint32_t weightBlockIndex = 0; weightBlockIndex < GSTD_MAX_HUFFMAN_WEIGHT; weightBlockIndex += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t weightMinusOne = GSTDDEC_VECTOR_UINT32(weightBlockIndex * GSTDDEC_VECTOR_WIDTH) + GSTDDEC_LANE_INDEX;
		vuint32_t weight = weightMinusOne + GSTDDEC_VECTOR_UINT32(1);

		vuint32_t numericUsage = GSTDDEC_VECTOR_UINT32(0);

		GSTDDEC_VECTOR_IF(weight <= GSTDDEC_VECTOR_UINT32(GSTD_MAX_HUFFMAN_WEIGHT))
		{
			vuint32_t countDWordIndex = GSTDDEC_VECTOR_UINT32(0);
			vuint32_t countBitPos = GSTDDEC_VECTOR_UINT32(0);
			uint32_t countMask = 0;

			ResolvePackedAddress8(weight, countDWordIndex, countBitPos, countMask);
			vuint32_t count = GSTDDEC_VECTOR_UINT32(0);

			GSTDDEC_CONDITIONAL_LOAD_INDEX(count, gs_decompressorState.packedHuffmanWeightCounts, countDWordIndex);
			count = (count >> countBitPos) & GSTDDEC_VECTOR_UINT32(countMask);

			GSTDDEC_CONDITIONAL_STORE(numericUsage, count << weightMinusOne);
		}
		GSTDDEC_VECTOR_END_IF

		// This handles the case where the remaining bit space is invalid
		GSTDDEC_VECTOR_IF(weight == GSTDDEC_VECTOR_UINT32(unspecifiedWeight))
		{
			GSTDDEC_CONDITIONAL_STORE(numericUsage, numericUsage + GSTDDEC_VECTOR_UINT32(remainingBitSpace));
		}
		GSTDDEC_VECTOR_END_IF

		weightNumericStarts[weightBlockIndex] = GSTDDEC_VECTOR_UINT32(weightStartRunningIterator) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(numericUsage);
		weightStartRunningIterator += GSTDDEC_SUM(numericUsage);
	}

	uint32_t upperPaddingBits = 32 - weightBits;

	for (uint32_t firstSymbol = 0; firstSymbol < numSpecifiedWeights; firstSymbol += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t symbol = GSTDDEC_VECTOR_UINT32(firstSymbol) + GSTDDEC_LANE_INDEX;

		vuint32_t weightDWordIndex = GSTDDEC_VECTOR_UINT32(0);
		vuint32_t weightBitPos = GSTDDEC_VECTOR_UINT32(0);
		uint32_t weightMask = 0;

		ResolvePackedAddress4(symbol, weightDWordIndex, weightBitPos, weightMask);
		vuint32_t weight = GSTDDEC_VECTOR_UINT32(0);

		GSTDDEC_VECTOR_IF(symbol < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
		{
			GSTDDEC_CONDITIONAL_LOAD_INDEX(weight, gs_decompressorState.packedHuffmanWeights, weightDWordIndex);
			weight = (weight >> weightBitPos) & GSTDDEC_VECTOR_UINT32(weightMask);
		}
		GSTDDEC_VECTOR_END_IF

		while (GSTDDEC_VECTOR_ANY(weight != GSTDDEC_VECTOR_UINT32(0)))
		{
			uint32_t firstCodedWeight = GSTDDEC_VECTOR_READ_FROM_INDEX(weight, GSTDDEC_VECTOR_FIRST_TRUE_INDEX(weight != GSTDDEC_VECTOR_UINT32(0)));

			vbool_t isExpectedWeight = (weight == GSTDDEC_VECTOR_UINT32(firstCodedWeight));

			uint32_t wns = 0;
			uint32_t firstCodedWeightMinusOne = firstCodedWeight - 1;
			uint32_t expectedWNSIndex = firstCodedWeightMinusOne / GSTDDEC_VECTOR_WIDTH;
			uint32_t wnsLaneIndex = firstCodedWeightMinusOne % GSTDDEC_VECTOR_WIDTH;
			uint32_t numTableEntries = 1 << firstCodedWeightMinusOne;
			uint32_t codeLength = weightBits - firstCodedWeightMinusOne;

			GSTDDEC_UNROLL_HINT
			for (uint32_t wnsIndex = 0; wnsIndex < GSTDDEC_NUMERIC_STARTS_VVEC_SIZE; wnsIndex++)
			{
				if (wnsIndex == expectedWNSIndex)
					wns = GSTDDEC_VECTOR_READ_FROM_INDEX(weightNumericStarts[wnsIndex], wnsLaneIndex);
			}

			uint32_t numValuesToBroadcast = GSTDDEC_VECTOR_NUM_TRUE(isExpectedWeight);

			GSTDDEC_BRANCH_HINT
			if (numTableEntries >= numValuesToBroadcast)
			{
				// Vector broadcast fill
				while (GSTDDEC_VECTOR_ANY(isExpectedWeight))
				{
					uint32_t laneToFlush = GSTDDEC_VECTOR_FIRST_TRUE_INDEX(weight == GSTDDEC_VECTOR_UINT32(firstCodedWeight));
					uint32_t laneSymbol = firstSymbol + laneToFlush;

					uint32_t insertStart = wns;
					wns += numTableEntries;

					for (uint32_t writeOffsetStart = 0; writeOffsetStart < numTableEntries; writeOffsetStart += GSTDDEC_VECTOR_WIDTH)
					{
						vuint32_t numericSpacePos = GSTDDEC_VECTOR_UINT32(insertStart) + GSTDDEC_VECTOR_UINT32(writeOffsetStart) + GSTDDEC_LANE_INDEX;
						vuint32_t tablePos = GSTDDEC_REVERSEBITS_UINT32(numericSpacePos << GSTDDEC_VECTOR_UINT32(upperPaddingBits));

						GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_UINT32(writeOffsetStart) + GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numTableEntries))
						{
							StoreHuffmanLookupCodes(GSTDDEC_CALL_EXECUTION_MASK tablePos, GSTDDEC_VECTOR_UINT32(laneSymbol), codeLength);
						}
						GSTDDEC_VECTOR_END_IF
					}

					GSTDDEC_VECTOR_IF(GSTDDEC_VECTOR_UINT32(laneToFlush) == GSTDDEC_LANE_INDEX)
					{
						GSTDDEC_CONDITIONAL_STORE(weight, GSTDDEC_VECTOR_UINT32(0));
						GSTDDEC_CONDITIONAL_STORE(isExpectedWeight, GSTDDEC_VECTOR_BOOL(false));
					}
					GSTDDEC_VECTOR_END_IF
				}
			}
			else
			{
				// Scatter fill
				vuint32_t vNumTableEntries = GSTDDEC_VECTOR_UINT32(0);

				GSTDDEC_VECTOR_IF(isExpectedWeight)
				{
					GSTDDEC_CONDITIONAL_STORE(vNumTableEntries, GSTDDEC_VECTOR_UINT32(numTableEntries));

					// NOTE: Running sum is active lanes only here but non-active lanes are zero so that's okay
					vuint32_t tableStart = (GSTDDEC_VECTOR_UINT32(wns) + GSTDDEC_EXCLUSIVE_RUNNING_SUM(vNumTableEntries));

					for (uint32_t insertOffset = 0; insertOffset < numTableEntries; insertOffset++)
					{
						vuint32_t numericSpacePos = tableStart + GSTDDEC_VECTOR_UINT32(insertOffset);
						vuint32_t tablePos = GSTDDEC_REVERSEBITS_UINT32(numericSpacePos << GSTDDEC_VECTOR_UINT32(upperPaddingBits));

						StoreHuffmanLookupCodes(GSTDDEC_CALL_EXECUTION_MASK tablePos, symbol, codeLength);
					}

					GSTDDEC_CONDITIONAL_STORE(weight, GSTDDEC_VECTOR_UINT32(0));
				}
				GSTDDEC_VECTOR_END_IF

				wns = wns + GSTDDEC_SUM(vNumTableEntries);
			}

			// Update the numeric range end
			GSTDDEC_UNROLL_HINT
			for (uint32_t wnsIndex = 0; wnsIndex < GSTDDEC_NUMERIC_STARTS_VVEC_SIZE; wnsIndex++)
			{
				if (wnsIndex == expectedWNSIndex)
				{
					GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX == GSTDDEC_VECTOR_UINT32(wnsLaneIndex))
					{
						GSTDDEC_CONDITIONAL_STORE(weightNumericStarts[wnsIndex], GSTDDEC_VECTOR_UINT32(wns));
					}
					GSTDDEC_VECTOR_END_IF
				}
			}
		}
	}

	// Fill unspecified entries.  This will fill all entries even if the table is unbalanced,
	// with indeterminate but safe results.
	{
		uint32_t unspecifiedWeightMinusOne = unspecifiedWeight - 1;
		uint32_t unspecifiedWNSIndex = unspecifiedWeightMinusOne / GSTDDEC_VECTOR_WIDTH;
		uint32_t unspecifiedWNSLaneIndex = unspecifiedWeightMinusOne % GSTDDEC_VECTOR_WIDTH;
		uint32_t unspecifiedWNS = 0;
		uint32_t unspecifiedCodeLength = weightBits - unspecifiedWeightMinusOne;

		GSTDDEC_UNROLL_HINT
		for (uint32_t wnsIndex = 0; wnsIndex < GSTDDEC_NUMERIC_STARTS_VVEC_SIZE; wnsIndex++)
		{
			if (wnsIndex == unspecifiedWNSIndex)
				unspecifiedWNS = GSTDDEC_VECTOR_READ_FROM_INDEX(weightNumericStarts[wnsIndex], unspecifiedWNSLaneIndex);
		}

		for (uint32_t unspecifiedOffsetStart = 0; unspecifiedOffsetStart < remainingBitSpace; unspecifiedOffsetStart += GSTDDEC_VECTOR_WIDTH)
		{
			vuint32_t unspecifiedOffset = GSTDDEC_VECTOR_UINT32(unspecifiedOffsetStart) + GSTDDEC_LANE_INDEX;

			GSTDDEC_VECTOR_IF(unspecifiedOffset < GSTDDEC_VECTOR_UINT32(remainingBitSpace))
			{
				vuint32_t numericSpacePos = GSTDDEC_VECTOR_UINT32(unspecifiedWNS) + GSTDDEC_VECTOR_UINT32(unspecifiedOffset);
				vuint32_t tablePos = GSTDDEC_REVERSEBITS_UINT32(numericSpacePos << GSTDDEC_VECTOR_UINT32(upperPaddingBits));

				StoreHuffmanLookupCodes(GSTDDEC_CALL_EXECUTION_MASK tablePos, GSTDDEC_VECTOR_UINT32(numSpecifiedWeights), unspecifiedCodeLength);
			}
			GSTDDEC_VECTOR_END_IF
		}
	}

	GSTDDEC_FLUSH_GS;

	outWeightTotal = 1 << GSTDDEC_MIN(weightBits, GSTD_MAX_HUFFMAN_CODE_LENGTH);
}

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT DecodeLitHuffmanTree(uint32_t auxBit, GSTDDEC_PARAM_OUT(uint32_t, outWeightTotal))
{
	uint32_t weightTotal = 0;

	ClearLitHuffmanTree();

	uint32_t numSpecifiedWeights = ReadRawByte();
	GSTDDEC_BRANCH_HINT
	if (numSpecifiedWeights == 0)
	{
		numSpecifiedWeights = ReadRawByte();
#if GSTDDEC_SANITIZE
		if (numSpecifiedWeights == 0)
		{
			GSTDDEC_WARN("Huffman table had 0 specified weights");
			numSpecifiedWeights = 1;
		}
#endif

		uint32_t numWeightsInExistingUncompressedBytes = g_dstate.uncompressedBytesAvailable * 2;

		for (uint32_t firstWeight = 0; firstWeight < numSpecifiedWeights; firstWeight += GSTDDEC_VECTOR_WIDTH)
		{
			vuint32_t weightIndex = GSTDDEC_VECTOR_UINT32(firstWeight) + GSTDDEC_LANE_INDEX;
			vuint32_t weight = GSTDDEC_VECTOR_UINT32(0);

			GSTDDEC_VECTOR_IF(weightIndex < GSTDDEC_VECTOR_UINT32(numSpecifiedWeights))
			{
				GSTDDEC_VECTOR_IF_NESTED(weightIndex < GSTDDEC_VECTOR_UINT32(numWeightsInExistingUncompressedBytes))
				{
					vuint32_t extractedWeight = (GSTDDEC_VECTOR_UINT32(g_dstate.uncompressedBytes) >> (weightIndex * GSTDDEC_VECTOR_UINT32(4))) & GSTDDEC_VECTOR_UINT32(0xf);
					GSTDDEC_CONDITIONAL_STORE(weight, extractedWeight);
				}
				GSTDDEC_VECTOR_ELSE_NESTED
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

				weight = GSTDDEC_MIN(weight, GSTDDEC_VECTOR_UINT32(GSTD_MAX_HUFFMAN_WEIGHT));
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

		uint32_t weightBytesConsumed = (numSpecifiedWeights + 1) / 2;
		if (weightBytesConsumed > numWeightsInExistingUncompressedBytes)
		{
			uint32_t bytesConsumedFromReadPos = weightBytesConsumed - g_dstate.uncompressedBytesAvailable;
			g_dstate.readPos += bytesConsumedFromReadPos / 4;

			uint32_t bytesConsumedFromNextDWord = (bytesConsumedFromReadPos & 3);
			if (bytesConsumedFromNextDWord == 0)
				g_dstate.uncompressedBytesAvailable = 0;
			else
			{
				g_dstate.uncompressedBytes = GSTDDEC_READ_INPUT_DWORD(g_dstate.readPos) >> (bytesConsumedFromNextDWord * 8);
				g_dstate.uncompressedBytesAvailable = 4 - bytesConsumedFromNextDWord;
				g_dstate.readPos++;
			}
		}
	}
	else
	{
		uint32_t accuracyLog = auxBit + 5;
		uint32_t sanitizedAccuracyLog = 0;

		DecodeFSETable(GSTDDEC_FSETAB_HUFF_WEIGHT_START, GSTD_MAX_HUFFMAN_WEIGHT, accuracyLog, GSTD_MAX_HUFFMAN_WEIGHT_ACCURACY_LOG, sanitizedAccuracyLog);

		// Decode the actual weights
		DecodeFSEHuffmanWeights(numSpecifiedWeights, sanitizedAccuracyLog, weightTotal);
	}

	ExpandLitHuffmanTable(numSpecifiedWeights, weightTotal, outWeightTotal);
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
uint32_t GSTDDEC_FUNCTION_CONTEXT WaveMax(vuint32_t value)
{
	uint32_t result = 0;
	for (unsigned int i = 0; i < TVectorWidth; i++)
		result = ArithMax(result, value.Get(i));

	return result;
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
uint32_t GSTDDEC_FUNCTION_CONTEXT WaveActiveCountTrue(vbool_t value)
{
	uint32_t numTrue = 0;
	for (unsigned int i = 0; i < TVectorWidth; i++)
		if (value.Get(i))
			numTrue++;
	return numTrue;
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
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT WaveReadLaneAtConditional(vbool_t executionMask, vuint32_t value, vuint32_t index)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			result.Set(i, value.Get(index.Get(i)));
	}

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
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ReverseBits(vuint32_t value)
{
	vuint32_t result;

	for (unsigned int i = 0; i < TVectorWidth; i++)
		result.Set(i, ReverseBits(value.Get(i)));

	return result;
}

GSTDDEC_FUNCTION_PREFIX
uint32_t GSTDDEC_FUNCTION_CONTEXT ReverseBits(uint32_t value)
{
	value = ((value << 16) & 0xffff0000u) | ((value >> 16) & 0x0000ffffu);
	value = ((value << 8) & 0xff00ff00u) | ((value >> 8) & 0x00ff00ffu);
	value = ((value << 4) & 0xf0f0f0f0u) | ((value >> 4) & 0x0f0f0f0fu);
	value = ((value << 2) & 0xccccccccu) | ((value >> 2) & 0x33333333u);
	value = ((value << 1) & 0xaaaaaaaau) | ((value >> 1) & 0x55555555u);

	return value;
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
		if ((packedSize & 2) == 0)
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
	vbool_t anyProblem = GSTDDEC_VECTOR_BOOL(false);

	GSTDDEC_VECTOR_IF(GSTDDEC_LANE_INDEX < GSTDDEC_VECTOR_UINT32(numLanesToDiscard))
	{
		vbool_t laneHasProblem = g_dstate.bitstreamAvailable[vvecIndex] < numBits;
		GSTDDEC_CONDITIONAL_STORE(anyProblem, laneHasProblem);
	}
	GSTDDEC_VECTOR_END_IF

	if (GSTDDEC_VECTOR_ANY(anyProblem))
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
void GSTDDEC_FUNCTION_CONTEXT DecodeFSETable(uint32_t fseTabStart, uint32_t fseTabMaxSymInclusive, uint32_t unsanitizedAccuracyLog, uint32_t maxAccuracyLog, GSTDDEC_PARAM_OUT(uint32_t, outAccuracyLog))
{
#if GSTDDEC_SANITIZE
	uint32_t accuracyLog = GSTDDEC_MIN(unsanitizedAccuracyLog, maxAccuracyLog);
#else
	uint32_t accuracyLog = unsanitizedAccuracyLog;
#endif
	uint32_t targetProbLimit = (1 << accuracyLog);
	uint32_t peekSize = maxAccuracyLog + 1 + GSTD_ZERO_PROB_REPEAT_BITS;

	uint32_t leadInCumulativeProb = 0;
	uint32_t leadInSymbol = 0;
	uint32_t lastCodedSymbol = 0;
	uint32_t numSymbols = 0;

#if GSTDDEC_SANITIZE
	if (accuracyLog != unsanitizedAccuracyLog)
	{
		GSTDDEC_WARN("Accuracy log was invalid");
	}
#endif

	outAccuracyLog = accuracyLog;

	for (uint32_t firstInitLane = 0; firstInitLane < targetProbLimit; firstInitLane += GSTDDEC_VECTOR_WIDTH)
	{
		vuint32_t slotIndex = GSTDDEC_VECTOR_UINT32(firstInitLane) + GSTDDEC_LANE_INDEX;

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

			vuint32_t bitstreamIndex = GSTDDEC_LANE_INDEX + GSTDDEC_VECTOR_UINT32(vvecIndex * GSTDDEC_VECTOR_WIDTH);

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
							GSTDDEC_VECTOR_IF_NESTED(GSTDDEC_LANE_INDEX == GSTDDEC_VECTOR_UINT32(firstOverflowingLaneIndex))
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
		vuint32_t slotIndex = GSTDDEC_VECTOR_UINT32(firstFillLane) + GSTDDEC_LANE_INDEX;

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

	// NOTE: No flush GS here, we assume that it is called in the parent function instead and that
	// no FSE table expansions overlap
}


GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT Run(vuint32_t laneIndex)
{
	uint32_t lastClearedDWordPos = 0;
	uint8_t numBits = 0;

	g_dstate.readPos = 0;
	g_dstate.writePosByte = 0;
	g_dstate.uncompressedBytesAvailable = 0;
	g_dstate.uncompressedBytes = 0;
	g_dstate.litRLEByte = 0;
	g_dstate.litLengthAccuracyLog = 0;
	g_dstate.matchLengthAccuracyLog = 0;
	g_dstate.offsetAccuracyLog = 0;
	g_dstate.repeatedOffset1 = 1;
	g_dstate.repeatedOffset2 = 4;
	g_dstate.repeatedOffset3 = 8;
	g_dstate.numLiteralsEmitted = 0;
	g_dstate.maxLiterals = 0;

	for (uint32_t i = 0; i < GSTDDEC_VVEC_SIZE; i++)
	{
		g_dstate.bitstreamBits[i] = GSTDDEC_VECTOR_UINT64(0);
		g_dstate.bitstreamAvailable[i] = GSTDDEC_VECTOR_UINT32(0);
		g_dstate.fseState[i] = GSTDDEC_VECTOR_UINT32(0);
		g_dstate.fseDrainLevel[i] = GSTDDEC_VECTOR_UINT32(GSTD_MAX_ACCURACY_LOG);
		g_dstate.literalsBuffer[i] = GSTDDEC_VECTOR_UINT32(0);
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
			DecompressCompressedBlock(controlWord);
		}
		else if (blockType == GSTD_BLOCK_TYPE_RLE)
		{
			DecompressRLEBlock(controlWord);
			GSTDDEC_WARN("NOT YET IMPLEMENTED");
		}
		else if (blockType == GSTD_BLOCK_TYPE_RAW)
		{
			DecompressRawBlock(controlWord);
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
uint32_t GSTDDEC_FUNCTION_CONTEXT ReadOutputDWord(uint32_t dwordPos) const
{
	if (dwordPos >= m_outSize)
		return 0;
	return m_outData[dwordPos];
}

GSTDDEC_FUNCTION_PREFIX
GSTDDEC_TYPE_CONTEXT vuint32_t GSTDDEC_FUNCTION_CONTEXT ReadOutputDWord(vbool_t executionMask, vuint32_t dwordPos) const
{
	vuint32_t result;
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			result.Set(i, ReadOutputDWord(dwordPos.Get(i)));
		else
			result.Set(i, 0xcccccccc);
	}

	return result;
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

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT InterlockedOrOutputDWord(vbool_t executionMask, const vuint32_t &dwordPos, const vuint32_t &dword) const
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
		{
			uint32_t iDWordPos = dwordPos.Get(i);
			if (iDWordPos < m_outSize)
				m_outData[iDWordPos] |= dword.Get(i);
		}
	}
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
void GSTDDEC_FUNCTION_CONTEXT ConditionalStore(vbool_t executionMask, vbool_t &storage, vbool_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage.Set(i, value.Get(i));
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

GSTDDEC_FUNCTION_PREFIX
void GSTDDEC_FUNCTION_CONTEXT ConditionalAddVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value)
{
	for (unsigned int i = 0; i < TVectorWidth; i++)
	{
		if (executionMask.Get(i))
			storage[index.Get(i)] += value.Get(i);
	}
}

void DecompressGstdCPU32(const void *inData, uint32_t inSize, void *outData, uint32_t outCapacity, void *warnContext, void (*warnCallback)(void *, const char *), void *diagContext, void (*diagCallback)(void *, const char *, ...))
{
	const unsigned int laneCount = 32;
	const unsigned int formatLaneCount = 32;

	gstddec::DecompressorContext<laneCount, formatLaneCount> decompressor(static_cast<const uint32_t*>(inData), inSize, static_cast<uint32_t*>(outData), outCapacity, warnContext, warnCallback, diagContext, diagCallback);

	gstddec::VectorUInt<uint32_t, laneCount> laneIndexes;
	for (unsigned int i = 0; i < laneCount; i++)
		laneIndexes.Set(i, i);

	decompressor.Run(laneIndexes);
}


#endif
