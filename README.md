
Simple VectorNav VN300 codec



## Decoder

- `#include "vn300_decoder.h" `
- Call `vn300_decode_standard_msg` with a `vn300_standard_msg_t`


## Encoder

- `#include "vn300_encoder.h"`
- Create a buffer wrapper `vn300_msg_buf_wrap_t`.  Do not allocate the inner `buf`: it will be allocated for you.  TODO: leaks
- Call `vn300_encode_standard_msg` 



