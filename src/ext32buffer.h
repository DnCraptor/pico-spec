#include <inttypes.h>
#include "ff.h"

class ext32buffer {
  static const size_t b_sz = 128;
  size_t off = 0; // in 4-bytes
  uint32_t t[b_sz] = { 0 };
  inline void sync_buf(size_t a) {
    FIL f;
    f_open(&f, "spec_page.tmp", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    UINT brw;
    f_lseek(&f, off << 2);
    f_write(&f, t, b_sz << 2, &brw);
    off = a & 0xFFFFFF80; // alligned 512 bytes (4*128)
    f_lseek(&f, off << 2);
    f_read(&f, t, b_sz << 2, &brw);
    f_close(&f);
  }
public:
  inline uint32_t& operator[](size_t a) {
    if (a < off || a >= off + b_sz) {
      sync_buf(a);
    }
    return t[a - off];
  }
  inline const uint32_t& operator[](size_t a) const {
    if (a < off || a >= off + b_sz) {
      const_cast<ext32buffer*>(this)->sync_buf(a);
    }
    return t[a - off];
  }
};
inline void memcpy(ext32buffer& dst, const void* src, size_t sz) {
  const uint32_t* src32 = static_cast<const uint32_t*>(src);
  for ( size_t i = 0; i < (sz >> 2); ++i) {
    dst[i] = src32[i];
  }
}
inline void memcpy(void* dst, const ext32buffer& src, size_t sz) {
  uint32_t* dst32 = static_cast<uint32_t*>(dst);
  for ( size_t i = 0; i < (sz >> 2); ++i) {
    dst32[i] = src[i];
  }
}
