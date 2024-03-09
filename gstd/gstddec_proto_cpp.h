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
		explicit VectorUInt(const TNumber &value);

		void Set(unsigned int index, const TNumber &value);
		const TNumber &Get(unsigned int index) const;

	private:
		TNumber m_values[TWidth];
	};

	template<unsigned int TWidth>
	class VectorBool
	{
	public:
	private:
		bool m_values[TWidth];
	};

	template<unsigned int TVectorWidth, unsigned int TFormatWidth>
	class DecompressorContext
	{
	public:
		typedef VectorUInt<uint32_t, TVectorWidth> vuint32_t;
		typedef VectorUInt<uint16_t, TVectorWidth> vuint16_t;
		typedef VectorBool<TVectorWidth> vbool_t;

		struct DecompressorState;

		DecompressorContext(const uint32_t *inData, uint32_t inSize, uint32_t *outData, uint32_t outSize);

		void DecompressRawBlock(vuint32_t laneIndex, DecompressorState *dstate);
		void DecompressRLEBlock(vuint32_t laneIndex, DecompressorState *dstate);
		void DecompressCompressedBlock(vuint32_t laneIndex, DecompressorState *dstate);
		void Run(vuint32_t laneIndex);

	private:
		struct Constants
		{
			uint32_t InSizeDWords;
			uint32_t OutSizeDWords;
		};

		vuint32_t ReadInputDWord(const vuint32_t &dwordPos) const;
		uint32_t ReadInputDWord(uint32_t dwordPos) const;

		const uint32_t *m_inData;
		uint32_t m_inSize;
		uint32_t *m_outData;
		uint32_t m_outSize;

		Constants m_constants;
	};

	template<unsigned int TVectorWidth, unsigned int TFormatWidth>
	DecompressorContext<TVectorWidth, TFormatWidth>::DecompressorContext(const uint32_t *inData, uint32_t inSize, uint32_t *outData, uint32_t outSize)
		: m_inData(inData), m_inSize(inSize / 4), m_outData(outData), m_outSize(outSize / 4)
	{
		m_constants.InSizeDWords = m_inSize;
		m_constants.OutSizeDWords = m_outSize;
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
