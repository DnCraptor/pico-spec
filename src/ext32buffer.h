#include <inttypes.h>
#include "ff.h"

class ext32buffer {
  FIL f;
  size_t prev;
  uint32_t v;
  inline void sync_buf(size_t a) {
    UINT brw;
    f_lseek(&f, prev << 2);
    f_write(&f, &v, sizeof(v), &brw);
    prev = a;
    f_lseek(&f, a << 2);
    f_read(&f, &v, sizeof(v), &brw);
    f_close(&f);
  }
public:
  inline ext32buffer(): prev(0), v(0) {
    f_open(&f, "/tmp/spec_page.tmp", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
  }
  inline uint32_t& operator[](size_t a) {
    if (a != prev) {
      sync_buf(a);
    }
    return v;
  }
  inline const uint32_t& operator[](size_t a) const {
    if (a != prev) {
      const_cast<ext32buffer*>(this)->sync_buf(a);
    }
    return v;
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
