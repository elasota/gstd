Gstandard
-
********************************************************************************
This project is currently archived and not currently functional.  The goal was
to create a transcoder from Zstandard to a format that would decompress
efficiently on GPU.

While this project yielded useful insights, ultimately it was only getting about
3% smaller file sizes than GDEFLATE, and dictionary compression failed to yield
much benefit, so this project has been shelved indefinitely.

The decoder is currently non-functional, as the format was transitioned from
using FSE to rANS for its entropy coder, but the decoder hasn't been updated to
use rANS.

I think the basic format improves over GDEFLATE in a few meaningful ways:
- Bitstream loads happen less often
- Minimum bit availability is contextual instead of always needing 32 bits
- Literals are decoded in chunks of up to 128 literals

These would likely improve over GDEFLATE if seen through to completion, but
given the low compression ratio improvement, this project is not currently
being pursued any further.