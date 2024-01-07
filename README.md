Gstandard
-
********************************************************************************
Gstandard is a lossless compression format based on Zstandard designed for fast
decompression on GPU compute kernels.

While it follows the same parallel data stream principles as GDEFLATE and
Brotli-G, Gstandard doesn't use a customized compressor to produce bitstreams,
it uses a transcoder that converts a Zstandard stream into a Gstandard
stream.  By doing this, Gstandard automatically benefits from technological
improvements to third-party Zstandard compressors without needing to integrate.

This is largely made possible by several technical innovations that permit fast
construction of FSE tables on GPU compute, and decoding of mixed-accuracy FSE
streams using a single state value using partial bit replacement.
