#include "gstddec_kernel.inl"

template<unsigned int TWidth>
typename gstddec::DecompressorContext<TWidth>::vuint32_t gstddec::DecompressorContext<TWidth>::ReadInputDWord(const vuint32_t &dwordPos) const
{
	vuint32_t result;
	for (unsigned int i = 0; i < TWidth; i++)
		result.Set(i, ReadInputDWord(dwordPos.Get(i)));

	return result;
}

template<unsigned int TWidth>
uint32_t gstddec::DecompressorContext<TWidth>::ReadInputDWord(uint32_t dwordPos) const
{
	if (dwordPos >= m_inSize)
		return 0;
	return m_inData[dwordPos];
}

void DecompressGstdCPU32(const void *inData, uint32_t inSize, void *outData, uint32_t outCapacity)
{
	const unsigned int laneCount = 32;

	gstddec::DecompressorContext<laneCount> decompressor(static_cast<const uint32_t *>(inData), inSize, static_cast<uint32_t *>(outData), outCapacity);

	gstddec::VectorUInt<uint32_t, laneCount> laneIndexes;
	for (unsigned int i = 0; i < laneCount; i++)
		laneIndexes.Set(i, i);

	decompressor.Run(laneIndexes);
}
