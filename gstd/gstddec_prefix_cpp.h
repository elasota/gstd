/*
Copyright (c) 2023 Eric Lasota

This software is available under the terms of the MIT license
or the Apache License, Version 2.0.  For more information, see
the included LICENSE.txt file.
*/

#ifdef __cplusplus

#include "gstddec_proto_cpp.h"

#define GSTDDEC_FORMAT_WIDTH		TFormatWidth
#define GSTDDEC_VECTOR_WIDTH		TVectorWidth

#define GSTDDEC_FUNCTION_PREFIX	template<unsigned int TVectorWidth, unsigned int TFormatWidth>

#define GSTDDEC_FUNCTION_CONTEXT gstddec::DecompressorContext<TVectorWidth, TFormatWidth>::
#define GSTDDEC_TYPE_CONTEXT typename gstddec::DecompressorContext<TVectorWidth, TFormatWidth>::

#define GSTDDEC_READ_CONSTANT(constName)	(this->m_constants.constName)

#define GSTDDEC_READ_INPUT_DWORD(pos) (this->ReadInputDWord(pos))

#define GSTDDEC_VECTOR_UINT32(fillValue) (vuint32_t(fillValue))

#define GSTDDEC_WARN(msg)

#endif
