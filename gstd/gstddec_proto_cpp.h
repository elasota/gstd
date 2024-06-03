/*
Copyright (c) 2023 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#include <stdint.h>

namespace gstddec
{
	template<unsigned int TWidth> class VectorBool;
	template<class TNumber, unsigned int TWidth> class VectorUInt;

	template<class TNumber, unsigned int TWidth>
	class VectorUInt
	{
	public:
		VectorUInt();
		template<class TOtherNumber>
		explicit VectorUInt(const VectorUInt<TOtherNumber, TWidth> &other);
		explicit VectorUInt(const TNumber &value);

		VectorUInt operator+(const VectorUInt &other) const;
		VectorUInt operator-(const VectorUInt &other) const;
		VectorUInt operator*(const VectorUInt &other) const;
		template<class TOtherNumber>
		VectorUInt operator>>(const VectorUInt<TOtherNumber, TWidth> &other) const;
		template<class TOtherNumber>
		VectorUInt operator<<(const VectorUInt<TOtherNumber, TWidth> &other) const;
		VectorUInt operator|(const VectorUInt &other) const;
		VectorUInt operator&(const VectorUInt &other) const;
		VectorUInt operator^(const VectorUInt &other) const;
		VectorUInt operator%(uint32_t other) const;

		VectorBool<TWidth> operator<(const VectorUInt &other) const;
		VectorBool<TWidth> operator<=(const VectorUInt &other) const;
		VectorBool<TWidth> operator>(const VectorUInt &other) const;
		VectorBool<TWidth> operator>=(const VectorUInt &other) const;
		VectorBool<TWidth> operator==(const VectorUInt &other) const;
		VectorBool<TWidth> operator!=(const VectorUInt &other) const;

		void Set(unsigned int index, const TNumber &value);
		const TNumber &Get(unsigned int index) const;

	private:
		TNumber m_values[TWidth];
	};

	template<unsigned int TWidth>
	class VectorBool
	{
	public:
		VectorBool();
		explicit VectorBool(bool value);

		VectorBool operator&(const VectorBool &other) const;
		VectorBool operator|(const VectorBool &other) const;
		VectorBool operator~() const;


		void Set(unsigned int index, bool value);
		bool Get(unsigned int index) const;

	private:
		bool m_values[TWidth];
	};

	template<unsigned int TVectorWidth, unsigned int TFormatWidth>
	class DecompressorContext
	{
	public:
		typedef VectorUInt<uint64_t, TVectorWidth> vuint64_t;
		typedef VectorUInt<uint32_t, TVectorWidth> vuint32_t;
		typedef VectorUInt<int32_t, TVectorWidth> vint32_t;
		typedef VectorUInt<uint16_t, TVectorWidth> vuint16_t;
		typedef VectorBool<TVectorWidth> vbool_t;
		typedef void (*WarnCallback_t)(void *, const char *);
		typedef void (*DiagCallback_t)(void *, const char *, ...);

#include "gstddec_decompressor_state.h"

		DecompressorContext(const uint32_t *inData, uint32_t inSize, uint32_t *outData, uint32_t outSize, void *warnContext, WarnCallback_t warnCallback, void *diagContext, DiagCallback_t diagCallback);

		void Run(vuint32_t laneIndex);

	private:
		struct Constants
		{
			uint32_t InSizeDWords;
			uint32_t OutSizeDWords;
		};

		vuint32_t ReadInputDWord(vbool_t executionMask, vuint32_t dwordPos) const;
		vuint32_t ReadInputDWord(const vuint32_t &dwordPos) const;
		uint32_t ReadInputDWord(uint32_t dwordPos) const;
		uint32_t ReadOutputDWord(uint32_t dwordPos) const;
		vuint32_t ReadOutputDWord(vbool_t executionMask, vuint32_t dwordPos) const;
		void PutOutputDWord(const vuint32_t &dwordPos, const vuint32_t &dword) const;
		void PutOutputDWord(uint32_t dwordPos, uint32_t dword) const;
		void InterlockedOrOutputDWord(vbool_t executionMask, const vuint32_t &dwordPos, const vuint32_t &dword) const;

		static vuint32_t FastFillAscending(vuint32_t value, uint32_t &runningValue);

		static void ConditionalStoreVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value);
		static void ConditionalStore(vbool_t executionMask, vbool_t &storage, vbool_t value);
		static void ConditionalStore(vbool_t executionMask, vuint32_t &storage, vuint32_t value);
		static void ConditionalStore(vbool_t executionMask, vuint64_t &storage, vuint64_t value);

		static void ConditionalLoadVector(vbool_t executionMask, vuint32_t &value, const uint32_t *storage, vuint32_t index);
		static void ConditionalLoad(vbool_t executionMask, vuint32_t &value, const vuint32_t &storage);

		static void ConditionalOrVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value);
		static void ConditionalAddVector(vbool_t executionMask, uint32_t *storage, vuint32_t index, vuint32_t value);

		void DecompressRawBlock(uint32_t size);
		void DecompressRLEBlock(uint32_t controlWord);
		void DecompressCompressedBlock(uint32_t controlWord);

		vuint32_t DecodeLiteralVector(uint32_t numLanes, vuint32_t codeBits, uint32_t huffmanCodeMask, vuint32_t &inOutDiscardBits);
		void RefillHuffmanLiteralsPartial(uint32_t literalsToRefill, uint32_t huffmanCodeMask, uint32_t passIndex);
		void RefillHuffmanLiterals(uint32_t huffmanCodeMask);
		void RefillRawLiterals();
		void EmitLiterals(uint32_t literalsToEmit);

		void DecodeLiteralsToTarget(uint32_t targetLiteralsEmitted, uint32_t litSectionType, uint32_t huffmanCodeMask);
		void ExecuteMatchCopy(uint32_t matchLength, uint32_t matchOffset);
		void DecodeAndExecuteSequences(uint32_t litSectionType, uint32_t huffmanCodeMask);
		void ClearLitHuffmanTree();
		void DecodeLitHuffmanTree(uint32_t auxBit, uint32_t &outWeightTotal);
		void ExpandLitHuffmanTable(uint32_t numSpecifiedWeights, uint32_t weightTotal, uint32_t &outWeightTotal);
		void DecodeFSEHuffmanWeights(uint32_t numSpecifiedWeights, uint32_t accuracyLog, uint32_t &huffmanWeightTotal);

		void ResolvePackedAddress4(vuint32_t index, vuint32_t &outDWordIndex, vuint32_t &outBitPos, uint32_t &outMask);
		void ResolvePackedAddress8(vuint32_t index, vuint32_t &outDWordIndex, vuint32_t &outBitPos, uint32_t &outMask);

		void HuffmanTableIndexToDecodeTableCell(vuint32_t tableIndex, vuint32_t &lengthDWordIndex, vuint32_t &lengthBitPos, vuint32_t &symbolDWordIndex, vuint32_t &symbolBitPos);
		void StoreHuffmanLookupCodes(vbool_t executionMask, vuint32_t tableIndex, vuint32_t symbol, uint32_t length);

		void DecodeLitRLEByte();
		uint32_t ReadPackedSize();

		vuint32_t BitstreamPeekNoTruncate(uint32_t vvecIndex, uint32_t numLanesToLoad, uint32_t numBits);
		vuint32_t BitstreamPeek(uint32_t vvecIndex, uint32_t numLanesToLoad, uint32_t numBits);
		void BitstreamDiscard(uint32_t vvecIndex, uint32_t numLanesToDiscard, vuint32_t numBits);

		void DecodeFSETable(uint32_t fseTabStart, uint32_t fseTabMaxSymInclusive, uint32_t accuracyLog, uint32_t maxAccuracyLog, uint32_t &outAccuracyLog);

		// This decodes a number of FSE values, all non-decoded values are filled with zero
		vuint32_t DecodeFSEValue(uint32_t numLanesToRefill, uint32_t vvecIndex, uint32_t accuracyLog, uint32_t firstCell);

		// This decodes an FSE value without refilling bits
		vuint32_t DecodeFSEValueNoPeek(uint32_t numLanesToRefill, uint32_t vvecIndex, uint32_t accuracyLog, uint32_t firstCell);

		uint32_t ReadRawByte();

		static vuint32_t WavePrefixSum(vuint32_t value);
		static uint32_t WaveMax(vuint32_t value);
		static uint32_t WaveSum(vuint32_t value);
		static bool WaveActiveAnyTrue(vbool_t value);
		static uint32_t WaveActiveCountTrue(vbool_t value);
		static vuint32_t WavePrefixCountBits(vbool_t value);
		static uint32_t WaveReadLaneAt(vuint32_t value, uint32_t index);
		static vuint32_t WaveReadLaneAt(vuint32_t value, vuint32_t index);
		static vuint32_t WaveReadLaneAtConditional(vbool_t executionMask, vuint32_t value, vuint32_t index);
		static uint32_t FirstTrueIndex(vbool_t value);
		static uint32_t LastTrueIndex(vbool_t value);
		static vuint32_t FirstBitHighPlusOne(vuint32_t value);
		static vuint32_t FirstBitLowPlusOne(vuint32_t value);
		static uint32_t FirstBitHighPlusOne(uint32_t value);
		static uint32_t FirstBitLowPlusOne(uint32_t value);
		static vuint32_t ArithMin(vuint32_t a, vuint32_t b);
		static vuint32_t ArithMax(vuint32_t a, vuint32_t b);
		static uint32_t ArithMin(uint32_t a, uint32_t b);
		static uint32_t ArithMax(uint32_t a, uint32_t b);
		static vuint32_t ReverseBits(vuint32_t value);
		static uint32_t ReverseBits(uint32_t value);
		static vuint32_t LaneIndex();

		const uint32_t *m_inData;
		uint32_t m_inSize;
		uint32_t *m_outData;
		uint32_t m_outSize;

		Constants m_constants;

		void *m_warnContext;
		WarnCallback_t m_warnCallback;

		void *m_diagContext;
		DiagCallback_t m_diagCallback;

		GroupSharedDecompressorState gs_decompressorState;
		DecompressorState g_dstate;

		struct HuffmanCodesDebug
		{
			uint8_t m_symbol;
			uint8_t m_length;
		};

		HuffmanCodesDebug m_huffmanDebug[1 << GSTD_MAX_HUFFMAN_CODE_LENGTH];
	};

	template<unsigned int TVectorWidth, unsigned int TFormatWidth>
	DecompressorContext<TVectorWidth, TFormatWidth>::DecompressorContext(const uint32_t *inData, uint32_t inSize, uint32_t *outData, uint32_t outSize, void *warnContext, WarnCallback_t warnCallback, void *diagContext, DiagCallback_t diagCallback)
		: m_inData(inData), m_inSize(inSize / 4), m_outData(outData), m_outSize(outSize / 4), m_warnContext(warnContext), m_warnCallback(warnCallback), m_diagContext(diagContext), m_diagCallback(diagCallback)
	{
		m_constants.InSizeDWords = m_inSize;
		m_constants.OutSizeDWords = m_outSize;
	}

	template<unsigned int TWidth>
	VectorBool<TWidth>::VectorBool()
	{
		for (unsigned int i = 0; i < TWidth; i++)
			m_values[i] = true;
	}

	template<unsigned int TWidth>
	VectorBool<TWidth>::VectorBool(bool value)
	{
		for (unsigned int i = 0; i < TWidth; i++)
			m_values[i] = value;
	}

	template<unsigned int TWidth>
	VectorBool<TWidth> VectorBool<TWidth>::operator&(const VectorBool &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] & other.m_values[i];

		return result;
	}

	template<unsigned int TWidth>
	VectorBool<TWidth> VectorBool<TWidth>::operator|(const VectorBool &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] | other.m_values[i];

		return result;
	}

	template<unsigned int TWidth>
	VectorBool<TWidth> VectorBool<TWidth>::operator~() const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = !m_values[i];

		return result;
	}

	template<unsigned int TWidth>
	void VectorBool<TWidth>::Set(unsigned int index, bool value)
	{
		m_values[index] = value;
	}

	template<unsigned int TWidth>
	bool VectorBool<TWidth>::Get(unsigned int index) const
	{
		return m_values[index];
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth>::VectorUInt(const TNumber &value)
	{
		for (unsigned int i = 0; i < TWidth; i++)
			m_values[i] = value;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth>::VectorUInt()
	{
		for (unsigned int i = 0; i < TWidth; i++)
			m_values[i] = 0xcc;
	}

	template<class TNumber, unsigned int TWidth>
	template<class TOtherNumber>
	VectorUInt<TNumber, TWidth>::VectorUInt(const VectorUInt<TOtherNumber, TWidth> &other)
	{
		for (unsigned int i = 0; i < TWidth; i++)
			m_values[i] = static_cast<TNumber>(other.Get(i));
	}


	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator+(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] + other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator-(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] - other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator*(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] * other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	template<class TOtherNumber>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator>>(const VectorUInt<TOtherNumber, TWidth> &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] >> other.Get(i);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	template<class TOtherNumber>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator<<(const VectorUInt<TOtherNumber, TWidth> &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] << other.Get(i);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator|(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] | other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator&(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] & other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator%(uint32_t other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] % other;

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorUInt<TNumber, TWidth> VectorUInt<TNumber, TWidth>::operator^(const VectorUInt &other) const
	{
		VectorUInt<TNumber, TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.m_values[i] = m_values[i] ^ other.m_values[i];

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator<(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] < other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator<=(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] <= other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator>(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] > other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator>=(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] >= other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator==(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] == other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	VectorBool<TWidth> VectorUInt<TNumber, TWidth>::operator!=(const VectorUInt &other) const
	{
		VectorBool<TWidth> result;

		for (unsigned int i = 0; i < TWidth; i++)
			result.Set(i, m_values[i] != other.m_values[i]);

		return result;
	}

	template<class TNumber, unsigned int TWidth>
	void VectorUInt<TNumber, TWidth>::Set(unsigned int index, const TNumber &value)
	{
		m_values[index] = value;
	}

	template<class TNumber, unsigned int TWidth>
	const TNumber &VectorUInt<TNumber, TWidth>::Get(unsigned int index) const
	{
		return m_values[index];
	}
}
