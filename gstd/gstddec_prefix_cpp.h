/*
Copyright (c) 2023 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#ifdef __cplusplus

#include "gstddec_proto_cpp.h"

#define GSTDDEC_WAVE_SIZE	TWidth

#define GSTDDEC_MAIN_FUNCTION_DEF	\
	template<unsigned int TWidth>	\
	void gstddec::DecompressorContext<TWidth>::Run

#define GSTDDEC_READ_CONSTANT(constName)	(this->m_constants.constName)

#define GSTDDEC_READ_INPUT_DWORD(pos) (this->ReadInputDWord(pos))

#define GSTDDEC_VECTOR_UINT32(fillValue) (vuint32_t(fillValue))

#endif
