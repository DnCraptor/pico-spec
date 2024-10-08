#include <inttypes.h>
#include "ff.h"
/*
extern "C" uint8_t buffer[2048];

class ext32buffer {
  size_t prev8;
  FIL f;
  inline void sync_buf(size_t a8) {
    UINT brw;
    f_lseek(&f, prev8);
    f_write(&f, buffer, 2048, &brw);
    prev8 = a8;
    f_lseek(&f, a8);
    f_read(&f, buffer, 2048, &brw);
  }
public:
  inline ext32buffer(): prev8(0) {
    f_open(&f, "/tmp/spec_page.tmp", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
  }
  ~ext32buffer() {
    f_close(&f);
  }
  inline uint32_t& operator[](size_t a32) {
    size_t a8 = a32 << 2;
    if (a8 < prev8 || a8 >= prev8 + 2048) {
      sync_buf(a8);
    }
    return *(uint32_t*)(buffer + a8 - prev8);
  }
  inline const uint32_t& operator[](size_t a32) const {
    size_t a8 = a32 << 2;
    if (a8 < prev8 || a8 >= prev8 + 2048) {
      const_cast<ext32buffer*>(this)->sync_buf(a8);
    }
    return *(uint32_t*)(buffer + a8 - prev8);
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

class ext8buffer {
  size_t prev8;
  inline void sync_buf(size_t a8) {
    UINT brw;
    FIL f;
    f_open(&f, "/tmp/spec_page.tmp", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    f_lseek(&f, prev8);
    f_write(&f, buffer, 2048, &brw);
    prev8 = a8;
    f_lseek(&f, a8);
    f_read(&f, buffer, 2048, &brw);
    f_close(&f);
  }
public:
  inline ext8buffer(): prev8(0) { }
  inline uint8_t& operator[](size_t a8) {
    if (a8 < prev8 || a8 >= prev8 + 2048) {
      sync_buf(a8);
    }
    return *(buffer + a8 - prev8);
  }
  inline const uint8_t& operator[](size_t a8) const {
    if (a8 < prev8 || a8 >= prev8 + 2048) {
      const_cast<ext8buffer*>(this)->sync_buf(a8);
    }
    return *(buffer + a8 - prev8);
  }
};
inline void memcpy(ext8buffer& dst, const void* src, size_t sz) {
  const uint8_t* src8 = static_cast<const uint8_t*>(src);
  for ( size_t i = 0; i < sz; ++i) {
    dst[i] = src8[i];
  }
}
inline void memcpy(void* dst, const ext8buffer& src, size_t sz) {
  uint8_t* dst8 = static_cast<uint8_t*>(dst);
  for ( size_t i = 0; i < sz; ++i) {
    dst8[i] = src[i];
  }
}
*/