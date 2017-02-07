
Simple VectorNav VN300 codec



## Decoder

- `#include "vn300_decoder.h" `
- Call `vn300_alloc_msg_wrap` to initialize a message wrapper 
- You may receive message bytes and copy them into the wrapper's inner buffer up to the maximum length given by the wrapper
- Call `vn300_decode_standard_msg` with the `vn300_standard_msg_t` and the `vn300_alloc_msg_wrap`
- When you're finished with the message wrapper, relese it with `vn300_release_msg_wrap`


## Encoder

- `#include "vn300_encoder.h"`
- Call `vn300_alloc_msg_wrap` to allocate a new buffer wrapper  `vn300_msg_buf_wrap_t`.  
- Call `vn300_encode_standard_msg` with your wrapper
- The encoded bytes will be available in the wrapper's inner buffer with the given length
- Send the encoded bytes 
- When you're finished, call `vn300_release_msg_wrap` with the wrapper to release it.



