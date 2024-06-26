cmake_minimum_required(VERSION 3.10)
project (gstd)

message(${CMAKE_BINARY_DIR})

add_library(zstd STATIC
	modules/zstd/lib/common/debug.c
	modules/zstd/lib/common/entropy_common.c
	modules/zstd/lib/common/error_private.c
	modules/zstd/lib/common/fse_decompress.c
	modules/zstd/lib/common/pool.c
	modules/zstd/lib/common/threading.c
	modules/zstd/lib/common/xxhash.c
	modules/zstd/lib/common/zstd_common.c
	modules/zstd/lib/compress/fse_compress.c
	modules/zstd/lib/compress/hist.c
	modules/zstd/lib/compress/huf_compress.c
	modules/zstd/lib/compress/zstd_compress.c
	modules/zstd/lib/compress/zstd_compress_literals.c
	modules/zstd/lib/compress/zstd_compress_sequences.c
	modules/zstd/lib/compress/zstd_compress_superblock.c
	modules/zstd/lib/compress/zstd_double_fast.c
	modules/zstd/lib/compress/zstd_fast.c
	modules/zstd/lib/compress/zstd_lazy.c
	modules/zstd/lib/compress/zstd_ldm.c
	modules/zstd/lib/compress/zstd_opt.c
	modules/zstd/lib/compress/zstdmt_compress.c
	modules/zstd/lib/decompress/huf_decompress.c
	modules/zstd/lib/decompress/zstd_ddict.c
	modules/zstd/lib/decompress/zstd_decompress.c
	modules/zstd/lib/decompress/zstd_decompress_block.c
	modules/zstd/lib/dictBuilder/cover.c
	modules/zstd/lib/dictBuilder/divsufsort.c
	modules/zstd/lib/dictBuilder/fastcover.c
	modules/zstd/lib/dictBuilder/zdict.c
	)

add_library(libdeflate STATIC
	modules/libdeflate/lib/deflate_compress.c
	modules/libdeflate/lib/utils.c
	)

target_include_directories(libdeflate PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modules/libdeflate>
	)

add_executable(gstdcmd
	gstd/gstd.cpp
	gstd/gstddec_kernel.cpp
	gstd/crc32.c
	)

add_executable(zstdasm
	modules/zstdhl/zstdasm.c
	)

add_subdirectory(modules/zstdhl)

target_include_directories(gstdcmd PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modules/zstd/lib>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/modules/zstdhl>
	)

target_link_libraries(gstdcmd zstdhl zstd libdeflate)
target_link_libraries(zstdasm zstdhl)
