/*
 * gdeflate_decompress_template.h
 *
 * Copyright 2016 Eric Biggers
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2020, 2021, 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This is the actual GDEFLATE decompression routine, lifted out of
 * gdeflate_decompress.c so that it can be compiled multiple times with
 * different target instruction sets.
 */

/*
 * Does the bitbuffer variable currently contain at least 'n' bits?
 */
#undef HAVE_BITS
#define HAVE_BITS(n) (s->bitsleft[s->idx] >= (n))

/*
 * Load more bits from the input buffer until the specified number of bits is
 * present in the bitbuffer variable.  'n' cannot be too large; see MAX_ENSURE
 * and CAN_ENSURE().
 */
#undef ENSURE_BITS
#define ENSURE_BITS(n)							\
if (!HAVE_BITS(n)) {							\
	s->bitbuf[s->idx] |= (bitbuf_t)(get_unaligned_le32(in_next)) << \
		s->bitsleft[s->idx];					\
	in_next += BITS_PER_PACKET/8;					\
	s->bitsleft[s->idx] += BITS_PER_PACKET;				\
}

/*
 * Return the next 'n' bits from the bitbuffer variable without removing them.
 */
#undef BITS
#define BITS(n) ((u32)s->bitbuf[s->idx] & (((u32)1 << (n)) - 1))

/*
 * Remove the next 'n' bits from the bitbuffer variable.
 */
#undef REMOVE_BITS
#define REMOVE_BITS(n) (s->bitbuf[s->idx] >>= (n), s->bitsleft[s->idx] -= (n))

/*
 * Setup copy advance method depending on a number of streams used.
 */
#if (NUM_STREAMS == 32)
#define ADVANCE_COPIES() is_copy = _rotr(is_copy, 1)
#else
#  pragma message("Invalid number of GDeflate streams used!")
#endif

/*
 * Reset GDeflate stream index.
 */
#define RESET() s->idx = 0

/*
 * Advance GDeflate stream index. Refill bits if necessary.
 */
#define ADVANCE() do {				\
	ENSURE_BITS(LOW_WATERMARK_BITS);	\
	s->idx = (s->idx + 1)%NUM_STREAMS;	\
	ADVANCE_COPIES();			\
} while(0)

/*
 * Tells if current GDeflate stream has a deferred copy.
 */
#define IS_COPY() (is_copy&1)

/*
 * Stores a deferred copy in current GDeflate stream.
 */
#define STORE_COPY(len, out) do {		\
	s->copies[s->idx].length = len;		\
	s->copies[s->idx].out_next = out;	\
	is_copy |= 1;				\
} while(0)

/*
 * Marks a copy in current current GDeflate stream as complete.
 */
#define COPY_COMPLETE() (is_copy &= ~1)

/*
 * Prevent multiple type declarations.
 */
#ifndef GDEFLATE_TYPES_DECLARED

/*
 * Setup is_copy type depending on a number of streams used.
 */
#if (NUM_STREAMS == 32)
typedef u32 is_copy_t;
#else
#  pragma message("Invalid number of GDeflate streams used!")
#endif

/*
 * GDeflate deferred copy state structure.
 */
struct gdeflate_deferred_copy {
	unsigned length;
	u8 * out_next;
};

/*
 * GDeflate state structure.
 */
struct gdeflate_state {
	bitbuf_t bitbuf[NUM_STREAMS];
	unsigned bitsleft[NUM_STREAMS];
	struct gdeflate_deferred_copy copies[NUM_STREAMS];
	unsigned idx;
};

/*
 * Prevent multiple type declarations.
 */
#define GDEFLATE_TYPES_DECLARED

#endif /* GDEFLATE_TYPES_DECLARED */

/*
 * Specialize gdeflate_do_copy function name.
 */
#define DO_COPY CONCAT(FUNCNAME, _gdeflate_do_copy)

/*
 * Perform a deferred GDeflate copy.
 */
static forceinline enum libdeflate_result ATTRIBUTES
DO_COPY(struct libdeflate_decompressor * restrict d,
	struct gdeflate_state * s, u8 * out, u8 * const out_end)
{
	u32 entry;
	u32 offset;
	const u8 *src;
	u8 *dst;
	u32 tmp32;

	/* Pop match params.  */
	u32 length = s->copies[s->idx].length;
	u8 * out_next = s->copies[s->idx].out_next;

	/* Decode the match offset.  */

	entry = d->offset_decode_table[BITS(OFFSET_TABLEBITS)];
	if (entry & HUFFDEC_SUBTABLE_POINTER) {
		/* Offset subtable required (uncommon case)  */
		REMOVE_BITS(OFFSET_TABLEBITS);
		entry = d->offset_decode_table[
			((entry >> HUFFDEC_RESULT_SHIFT) & 0xFFFF) +
			BITS(entry & HUFFDEC_LENGTH_MASK)];
	}
	REMOVE_BITS(entry & HUFFDEC_LENGTH_MASK);

	entry >>= HUFFDEC_RESULT_SHIFT;

	/* Pop the extra offset bits and add them to the offset base to
	 * produce the full offset.  */
	offset = (entry & HUFFDEC_OFFSET_BASE_MASK) +
		 POP_BITS(entry >> HUFFDEC_EXTRA_OFFSET_BITS_SHIFT);

	/* The match source must not begin before the beginning of the
	 * output buffer.  */
	SAFETY_CHECK(offset <= out_next - (const u8 *)out);

	/*
	 * Copy the match: 'length' bytes at 'out_next - offset' to
	 * 'out_next', possibly overlapping.  If the match doesn't end
	 * too close to the end of the buffer and offset >= WORDBYTES ||
	 * offset == 1, take a fast path which copies a word at a time
	 * -- potentially more than the length of the match, but that's
	 * fine as long as we check for enough extra space.
	 *
	 * The remaining cases are not performance-critical so are
	 * handled by a simple byte-by-byte copy.
	 */
	src = out_next - offset;
	dst = out_next;
	out_next += length;

	if (UNALIGNED_ACCESS_IS_FAST &&
	    likely(out_end - out_next >= WORDBYTES && length >= WORDBYTES)) {
		if (offset >= WORDBYTES) { /* words don't overlap? */
			while (dst < out_next - WORDBYTES) {
				copy_word_unaligned(src, dst);
				src += WORDBYTES;
				dst += WORDBYTES;
			}
			/* Tail. */
			while (dst < out_next) *dst++ = *src++;
		} else if (offset == 1) {
			/* RLE encoding of previous byte, common if the
			 * data contains many repeated bytes */
			machine_word_t v = repeat_byte(*src);

			while (dst < out_next - WORDBYTES) {
				store_word_unaligned(v, dst);
				dst += WORDBYTES;
			}
			/* Tail. */
			while (dst < out_next) *dst++ = (u8)v;
		} else {
			*dst++ = *src++;
			*dst++ = *src++;
			do {
				*dst++ = *src++;
			} while (dst < out_next);
		}
	} else {
		STATIC_ASSERT(DEFLATE_MIN_MATCH_LEN == 3);
		*dst++ = *src++;
		*dst++ = *src++;
		do {
			*dst++ = *src++;
		} while (dst < out_next);
	}

	return LIBDEFLATE_SUCCESS;
}

static enum libdeflate_result ATTRIBUTES
FUNCNAME(struct libdeflate_decompressor * restrict d,
	 const void * restrict in, size_t in_nbytes,
	 void * restrict out, size_t out_nbytes_avail,
	 size_t *actual_in_nbytes_ret, size_t *actual_out_nbytes_ret)
{
	u8 *out_next = out;
	u8 * const out_end = out_next + out_nbytes_avail;
	const u8 *in_next = in;
	const u8 * const in_end = in_next + in_nbytes;
	struct gdeflate_state state;
	struct gdeflate_state * s = &state;
	unsigned i;
	unsigned is_final_block;
	unsigned block_type;
	u16 len;
	unsigned num_litlen_syms;
	unsigned num_offset_syms;
	u32 tmp32;
	is_copy_t is_copy = 0;

	/* Starting to read GDeflate stream.  */
	RESET();
	for (unsigned n = 0; n < NUM_STREAMS; n++) {
		s->bitbuf[n] = 0;
		s->bitsleft[n] = 0;
		s->copies[n].length = 0;
		ADVANCE();
	}

next_block:
	/* Starting to read the next block.  */
	RESET();

	/* BFINAL: 1 bit  */
	is_final_block = POP_BITS(1);

	/* BTYPE: 2 bits  */
	block_type = POP_BITS(2);

	ENSURE_BITS(LOW_WATERMARK_BITS);

	if (block_type == DEFLATE_BLOCKTYPE_DYNAMIC_HUFFMAN) {

		/* Dynamic Huffman block.  */

		/* The order in which precode lengths are stored.  */
		static const u8 deflate_precode_lens_permutation[DEFLATE_NUM_PRECODE_SYMS] = {
			16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
		};

		unsigned num_explicit_precode_lens;

		/* Read the codeword length counts.  */

		STATIC_ASSERT(DEFLATE_NUM_LITLEN_SYMS == ((1 << 5) - 1) + 257);
		num_litlen_syms = POP_BITS(5) + 257;

		STATIC_ASSERT(DEFLATE_NUM_OFFSET_SYMS == ((1 << 5) - 1) + 1);
		num_offset_syms = POP_BITS(5) + 1;

		STATIC_ASSERT(DEFLATE_NUM_PRECODE_SYMS == ((1 << 4) - 1) + 4);
		num_explicit_precode_lens = POP_BITS(4) + 4;

		d->static_codes_loaded = false;

		ENSURE_BITS(LOW_WATERMARK_BITS);

		/* Read the precode codeword lengths.  */
		STATIC_ASSERT(DEFLATE_MAX_PRE_CODEWORD_LEN == (1 << 3) - 1);
		for (i = 0; i < num_explicit_precode_lens; i++) {
			d->u.precode_lens[deflate_precode_lens_permutation[i]] = POP_BITS(3);

			ADVANCE();
		}

		for (; i < DEFLATE_NUM_PRECODE_SYMS; i++)
			d->u.precode_lens[deflate_precode_lens_permutation[i]] = 0;

		/* Build the decode table for the precode.  */
		SAFETY_CHECK(build_precode_decode_table(d));

		RESET();

		/* Expand the literal/length and offset codeword lengths.  */
		for (i = 0; i < num_litlen_syms + num_offset_syms; ) {
			u32 entry;
			unsigned presym;
			u8 rep_val;
			unsigned rep_count;

			/* (The code below assumes that the precode decode table
			 * does not have any subtables.)  */
			STATIC_ASSERT(PRECODE_TABLEBITS == DEFLATE_MAX_PRE_CODEWORD_LEN);

			/* Read the next precode symbol.  */
			entry = d->u.l.precode_decode_table[BITS(DEFLATE_MAX_PRE_CODEWORD_LEN)];
			REMOVE_BITS(entry & HUFFDEC_LENGTH_MASK);

			presym = entry >> HUFFDEC_RESULT_SHIFT;

			if (presym < 16) {
				/* Explicit codeword length  */
				d->u.l.lens[i++] = presym;
				ADVANCE();
				continue;
			}

			/* Run-length encoded codeword lengths  */

			/* Note: we don't need verify that the repeat count
			 * doesn't overflow the number of elements, since we
			 * have enough extra spaces to allow for the worst-case
			 * overflow (138 zeroes when only 1 length was
			 * remaining).
			 *
			 * In the case of the small repeat counts (presyms 16
			 * and 17), it is fastest to always write the maximum
			 * number of entries.  That gets rid of branches that
			 * would otherwise be required.
			 *
			 * It is not just because of the numerical order that
			 * our checks go in the order 'presym < 16', 'presym ==
			 * 16', and 'presym == 17'.  For typical data this is
			 * ordered from most frequent to least frequent case.
			 */
			STATIC_ASSERT(DEFLATE_MAX_LENS_OVERRUN == 138 - 1);

			if (presym == 16) {
				/* Repeat the previous length 3 - 6 times  */
				SAFETY_CHECK(i != 0);
				rep_val = d->u.l.lens[i - 1];
				STATIC_ASSERT(3 + ((1 << 2) - 1) == 6);
				rep_count = 3 + POP_BITS(2);
				d->u.l.lens[i + 0] = rep_val;
				d->u.l.lens[i + 1] = rep_val;
				d->u.l.lens[i + 2] = rep_val;
				d->u.l.lens[i + 3] = rep_val;
				d->u.l.lens[i + 4] = rep_val;
				d->u.l.lens[i + 5] = rep_val;
				i += rep_count;
			} else if (presym == 17) {
				/* Repeat zero 3 - 10 times  */
				STATIC_ASSERT(3 + ((1 << 3) - 1) == 10);
				rep_count = 3 + POP_BITS(3);
				d->u.l.lens[i + 0] = 0;
				d->u.l.lens[i + 1] = 0;
				d->u.l.lens[i + 2] = 0;
				d->u.l.lens[i + 3] = 0;
				d->u.l.lens[i + 4] = 0;
				d->u.l.lens[i + 5] = 0;
				d->u.l.lens[i + 6] = 0;
				d->u.l.lens[i + 7] = 0;
				d->u.l.lens[i + 8] = 0;
				d->u.l.lens[i + 9] = 0;
				i += rep_count;
			} else {
				/* Repeat zero 11 - 138 times  */
				STATIC_ASSERT(11 + ((1 << 7) - 1) == 138);
				rep_count = 11 + POP_BITS(7);
				memset(&d->u.l.lens[i], 0,
				       rep_count * sizeof(d->u.l.lens[i]));
				i += rep_count;
			}

			ADVANCE();
		}
	} else if (block_type == DEFLATE_BLOCKTYPE_UNCOMPRESSED) {

		/* Uncompressed block: copy 'len' bytes literally from the input
		 * buffer to the output buffer.  */

		/* Count bits in the bit buffers. */
		u32 num_buffered_bits = 0;
		for (u32 n = 0; n < NUM_STREAMS; n++)
			num_buffered_bits += s->bitsleft[n];

		SAFETY_CHECK(in_end - in_next + (num_buffered_bits + 7)/8 >= 2);

		len = POP_BITS(16);

		if (unlikely(len > out_end - out_next))
			return LIBDEFLATE_INSUFFICIENT_SPACE;
		SAFETY_CHECK(len <= in_end - in_next + (num_buffered_bits + 7)/8);

		while (len) {
			*out_next++ = POP_BITS(8);
			len--;
			ADVANCE();
		}

		goto block_done;

	} else {
		SAFETY_CHECK(block_type == DEFLATE_BLOCKTYPE_STATIC_HUFFMAN);

		/*
		 * Static Huffman block: build the decode tables for the static
		 * codes.  Skip doing so if the tables are already set up from
		 * an earlier static block; this speeds up decompression of
		 * degenerate input of many empty or very short static blocks.
		 *
		 * Afterwards, the remainder is the same as decompressing a
		 * dynamic Huffman block.
		 */

		if (d->static_codes_loaded)
			goto have_decode_tables;

		d->static_codes_loaded = true;

		STATIC_ASSERT(DEFLATE_NUM_LITLEN_SYMS == 288);
		STATIC_ASSERT(DEFLATE_NUM_OFFSET_SYMS == 32);

		for (i = 0; i < 144; i++)
			d->u.l.lens[i] = 8;
		for (; i < 256; i++)
			d->u.l.lens[i] = 9;
		for (; i < 280; i++)
			d->u.l.lens[i] = 7;
		for (; i < 288; i++)
			d->u.l.lens[i] = 8;

		for (; i < 288 + 32; i++)
			d->u.l.lens[i] = 5;

		num_litlen_syms = 288;
		num_offset_syms = 32;
	}

	/* Decompressing a Huffman block (either dynamic or static)  */

	SAFETY_CHECK(build_offset_decode_table(d, num_litlen_syms, num_offset_syms));
	SAFETY_CHECK(build_litlen_decode_table(d, num_litlen_syms, num_offset_syms));
have_decode_tables:

	RESET();

	/* The main GDEFLATE decode loop  */
	for (;;) {
		u32 entry;
		u32 length;

		if (likely(!IS_COPY())) {
			/* Decode a litlen symbol.  */
			entry = d->u.litlen_decode_table[BITS(LITLEN_TABLEBITS)];
			if (entry & HUFFDEC_SUBTABLE_POINTER) {
				/* Litlen subtable required (uncommon case)  */
				REMOVE_BITS(LITLEN_TABLEBITS);
				entry = d->u.litlen_decode_table[
					((entry >> HUFFDEC_RESULT_SHIFT) & 0xFFFF) +
					BITS(entry & HUFFDEC_LENGTH_MASK)];
			}
			REMOVE_BITS(entry & HUFFDEC_LENGTH_MASK);
			if (entry & HUFFDEC_LITERAL) {
				/* Literal  */
				if (unlikely(out_next == out_end))
					return LIBDEFLATE_INSUFFICIENT_SPACE;
				*out_next++ = (u8)(entry >> HUFFDEC_RESULT_SHIFT);
				ADVANCE();
				continue;
			}

			/* Match or end-of-block  */

			entry >>= HUFFDEC_RESULT_SHIFT;

			/* Pop the extra length bits and add them to the length base to
			 * produce the full length.  */
			length = (entry >> HUFFDEC_LENGTH_BASE_SHIFT) +
				 POP_BITS(entry & HUFFDEC_EXTRA_LENGTH_BITS_MASK);

			/* The match destination must not end after the end of the
			 * output buffer.  For efficiency, combine this check with the
			 * end-of-block check.  We're using 0 for the special
			 * end-of-block length, so subtract 1 and it turn it into
			 * SIZE_MAX.  */
			STATIC_ASSERT(HUFFDEC_END_OF_BLOCK_LENGTH == 0);
			if (unlikely((size_t)length - 1 >= out_end - out_next)) {
				if (unlikely(length != HUFFDEC_END_OF_BLOCK_LENGTH))
					return LIBDEFLATE_INSUFFICIENT_SPACE;
				goto block_done;
			}

			/* Store copy for use later.  */
			STORE_COPY(length, out_next);

			/* Advance output stream.  */
			out_next += length;
		} else {
			enum libdeflate_result res =
				DO_COPY(d, s, out, out_end);

			if (unlikely(res))
				return res;

			COPY_COMPLETE();
		}

		ADVANCE();
	}

block_done:

	/* Run the outstanding deferred copies.  */

	for (unsigned n = 0; n < NUM_STREAMS; n++) {
		if (IS_COPY()) {
			enum libdeflate_result res =
				DO_COPY(d, s, out, out_end);

			if (unlikely(res))
				return res;

			COPY_COMPLETE();
		}

		ADVANCE();
	}

	/* Finished decoding a block.  */

	if (!is_final_block)
		goto next_block;

	/* That was the last block.  */

	/* Optionally return the actual number of bytes read */
	if (actual_in_nbytes_ret)
		*actual_in_nbytes_ret = in_next - (u8 *)in;

	/* Optionally return the actual number of bytes written */
	if (actual_out_nbytes_ret) {
		*actual_out_nbytes_ret = out_next - (u8 *)out;
	} else {
		if (out_next != out_end)
			return LIBDEFLATE_SHORT_OUTPUT;
	}
	return LIBDEFLATE_SUCCESS;
}

#undef FUNCNAME
#undef ATTRIBUTES
#undef IS_COPY
#undef DO_COPY
#undef ADVANCE_COPIES
#undef STORE_COPY
#undef COPY_COMPLETE
#undef RESET
#undef ADVANCE
