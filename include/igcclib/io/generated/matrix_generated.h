// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_MATRIX_FBSDATA_H_
#define FLATBUFFERS_GENERATED_MATRIX_FBSDATA_H_

#include "flatbuffers/flatbuffers.h"

namespace fbsdata {

struct Matrix_d;

struct Matrix_f;

struct Matrix_i32;

struct Matrix_i64;

struct Matrix_u8;

struct Vector_3d;

struct Vector_2d;

struct Vector_2i32;

MANUALLY_ALIGNED_STRUCT(8) Vector_3d FLATBUFFERS_FINAL_CLASS {
 private:
  double x_;
  double y_;
  double z_;

 public:
  Vector_3d() {
    memset(this, 0, sizeof(Vector_3d));
  }
  Vector_3d(double _x, double _y, double _z)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)),
        z_(flatbuffers::EndianScalar(_z)) {
  }
  double x() const {
    return flatbuffers::EndianScalar(x_);
  }
  double y() const {
    return flatbuffers::EndianScalar(y_);
  }
  double z() const {
    return flatbuffers::EndianScalar(z_);
  }
};
STRUCT_END(Vector_3d, 24);

MANUALLY_ALIGNED_STRUCT(8) Vector_2d FLATBUFFERS_FINAL_CLASS {
 private:
  double x_;
  double y_;

 public:
  Vector_2d() {
    memset(this, 0, sizeof(Vector_2d));
  }
  Vector_2d(double _x, double _y)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)) {
  }
  double x() const {
    return flatbuffers::EndianScalar(x_);
  }
  double y() const {
    return flatbuffers::EndianScalar(y_);
  }
};
STRUCT_END(Vector_2d, 16);

MANUALLY_ALIGNED_STRUCT(4) Vector_2i32 FLATBUFFERS_FINAL_CLASS {
 private:
  int32_t x_;
  int32_t y_;

 public:
  Vector_2i32() {
    memset(this, 0, sizeof(Vector_2i32));
  }
  Vector_2i32(int32_t _x, int32_t _y)
      : x_(flatbuffers::EndianScalar(_x)),
        y_(flatbuffers::EndianScalar(_y)) {
  }
  int32_t x() const {
    return flatbuffers::EndianScalar(x_);
  }
  int32_t y() const {
    return flatbuffers::EndianScalar(y_);
  }
};
STRUCT_END(Vector_2i32, 8);

struct Matrix_d FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_CONTENT = 4,
    VT_SHAPE = 6
  };
  const flatbuffers::Vector<double> *content() const {
    return GetPointer<const flatbuffers::Vector<double> *>(VT_CONTENT);
  }
  const flatbuffers::Vector<int32_t> *shape() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_CONTENT) &&
           verifier.Verify(content()) &&
           VerifyOffset(verifier, VT_SHAPE) &&
           verifier.Verify(shape()) &&
           verifier.EndTable();
  }
};

struct Matrix_dBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_content(flatbuffers::Offset<flatbuffers::Vector<double>> content) {
    fbb_.AddOffset(Matrix_d::VT_CONTENT, content);
  }
  void add_shape(flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape) {
    fbb_.AddOffset(Matrix_d::VT_SHAPE, shape);
  }
  explicit Matrix_dBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Matrix_dBuilder &operator=(const Matrix_dBuilder &);
  flatbuffers::Offset<Matrix_d> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Matrix_d>(end);
    return o;
  }
};

inline flatbuffers::Offset<Matrix_d> CreateMatrix_d(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<double>> content = 0,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape = 0) {
  Matrix_dBuilder builder_(_fbb);
  builder_.add_shape(shape);
  builder_.add_content(content);
  return builder_.Finish();
}

inline flatbuffers::Offset<Matrix_d> CreateMatrix_dDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<double> *content = nullptr,
    const std::vector<int32_t> *shape = nullptr) {
  return fbsdata::CreateMatrix_d(
      _fbb,
      content ? _fbb.CreateVector<double>(*content) : 0,
      shape ? _fbb.CreateVector<int32_t>(*shape) : 0);
}

struct Matrix_f FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_CONTENT = 4,
    VT_SHAPE = 6
  };
  const flatbuffers::Vector<float> *content() const {
    return GetPointer<const flatbuffers::Vector<float> *>(VT_CONTENT);
  }
  const flatbuffers::Vector<int32_t> *shape() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_CONTENT) &&
           verifier.Verify(content()) &&
           VerifyOffset(verifier, VT_SHAPE) &&
           verifier.Verify(shape()) &&
           verifier.EndTable();
  }
};

struct Matrix_fBuilder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_content(flatbuffers::Offset<flatbuffers::Vector<float>> content) {
    fbb_.AddOffset(Matrix_f::VT_CONTENT, content);
  }
  void add_shape(flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape) {
    fbb_.AddOffset(Matrix_f::VT_SHAPE, shape);
  }
  explicit Matrix_fBuilder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Matrix_fBuilder &operator=(const Matrix_fBuilder &);
  flatbuffers::Offset<Matrix_f> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Matrix_f>(end);
    return o;
  }
};

inline flatbuffers::Offset<Matrix_f> CreateMatrix_f(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<float>> content = 0,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape = 0) {
  Matrix_fBuilder builder_(_fbb);
  builder_.add_shape(shape);
  builder_.add_content(content);
  return builder_.Finish();
}

inline flatbuffers::Offset<Matrix_f> CreateMatrix_fDirect(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<float> *content = nullptr,
    const std::vector<int32_t> *shape = nullptr) {
  return fbsdata::CreateMatrix_f(
      _fbb,
      content ? _fbb.CreateVector<float>(*content) : 0,
      shape ? _fbb.CreateVector<int32_t>(*shape) : 0);
}

struct Matrix_i32 FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_CONTENT = 4,
    VT_SHAPE = 6
  };
  const flatbuffers::Vector<int32_t> *content() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_CONTENT);
  }
  const flatbuffers::Vector<int32_t> *shape() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_CONTENT) &&
           verifier.Verify(content()) &&
           VerifyOffset(verifier, VT_SHAPE) &&
           verifier.Verify(shape()) &&
           verifier.EndTable();
  }
};

struct Matrix_i32Builder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_content(flatbuffers::Offset<flatbuffers::Vector<int32_t>> content) {
    fbb_.AddOffset(Matrix_i32::VT_CONTENT, content);
  }
  void add_shape(flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape) {
    fbb_.AddOffset(Matrix_i32::VT_SHAPE, shape);
  }
  explicit Matrix_i32Builder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Matrix_i32Builder &operator=(const Matrix_i32Builder &);
  flatbuffers::Offset<Matrix_i32> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Matrix_i32>(end);
    return o;
  }
};

inline flatbuffers::Offset<Matrix_i32> CreateMatrix_i32(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> content = 0,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape = 0) {
  Matrix_i32Builder builder_(_fbb);
  builder_.add_shape(shape);
  builder_.add_content(content);
  return builder_.Finish();
}

inline flatbuffers::Offset<Matrix_i32> CreateMatrix_i32Direct(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<int32_t> *content = nullptr,
    const std::vector<int32_t> *shape = nullptr) {
  return fbsdata::CreateMatrix_i32(
      _fbb,
      content ? _fbb.CreateVector<int32_t>(*content) : 0,
      shape ? _fbb.CreateVector<int32_t>(*shape) : 0);
}

struct Matrix_i64 FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_CONTENT = 4,
    VT_SHAPE = 6
  };
  const flatbuffers::Vector<int64_t> *content() const {
    return GetPointer<const flatbuffers::Vector<int64_t> *>(VT_CONTENT);
  }
  const flatbuffers::Vector<int32_t> *shape() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_CONTENT) &&
           verifier.Verify(content()) &&
           VerifyOffset(verifier, VT_SHAPE) &&
           verifier.Verify(shape()) &&
           verifier.EndTable();
  }
};

struct Matrix_i64Builder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_content(flatbuffers::Offset<flatbuffers::Vector<int64_t>> content) {
    fbb_.AddOffset(Matrix_i64::VT_CONTENT, content);
  }
  void add_shape(flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape) {
    fbb_.AddOffset(Matrix_i64::VT_SHAPE, shape);
  }
  explicit Matrix_i64Builder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Matrix_i64Builder &operator=(const Matrix_i64Builder &);
  flatbuffers::Offset<Matrix_i64> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Matrix_i64>(end);
    return o;
  }
};

inline flatbuffers::Offset<Matrix_i64> CreateMatrix_i64(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<int64_t>> content = 0,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape = 0) {
  Matrix_i64Builder builder_(_fbb);
  builder_.add_shape(shape);
  builder_.add_content(content);
  return builder_.Finish();
}

inline flatbuffers::Offset<Matrix_i64> CreateMatrix_i64Direct(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<int64_t> *content = nullptr,
    const std::vector<int32_t> *shape = nullptr) {
  return fbsdata::CreateMatrix_i64(
      _fbb,
      content ? _fbb.CreateVector<int64_t>(*content) : 0,
      shape ? _fbb.CreateVector<int32_t>(*shape) : 0);
}

struct Matrix_u8 FLATBUFFERS_FINAL_CLASS : private flatbuffers::Table {
  enum {
    VT_CONTENT = 4,
    VT_SHAPE = 6
  };
  const flatbuffers::Vector<uint8_t> *content() const {
    return GetPointer<const flatbuffers::Vector<uint8_t> *>(VT_CONTENT);
  }
  const flatbuffers::Vector<int32_t> *shape() const {
    return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  }
  bool Verify(flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_CONTENT) &&
           verifier.Verify(content()) &&
           VerifyOffset(verifier, VT_SHAPE) &&
           verifier.Verify(shape()) &&
           verifier.EndTable();
  }
};

struct Matrix_u8Builder {
  flatbuffers::FlatBufferBuilder &fbb_;
  flatbuffers::uoffset_t start_;
  void add_content(flatbuffers::Offset<flatbuffers::Vector<uint8_t>> content) {
    fbb_.AddOffset(Matrix_u8::VT_CONTENT, content);
  }
  void add_shape(flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape) {
    fbb_.AddOffset(Matrix_u8::VT_SHAPE, shape);
  }
  explicit Matrix_u8Builder(flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  Matrix_u8Builder &operator=(const Matrix_u8Builder &);
  flatbuffers::Offset<Matrix_u8> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = flatbuffers::Offset<Matrix_u8>(end);
    return o;
  }
};

inline flatbuffers::Offset<Matrix_u8> CreateMatrix_u8(
    flatbuffers::FlatBufferBuilder &_fbb,
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> content = 0,
    flatbuffers::Offset<flatbuffers::Vector<int32_t>> shape = 0) {
  Matrix_u8Builder builder_(_fbb);
  builder_.add_shape(shape);
  builder_.add_content(content);
  return builder_.Finish();
}

inline flatbuffers::Offset<Matrix_u8> CreateMatrix_u8Direct(
    flatbuffers::FlatBufferBuilder &_fbb,
    const std::vector<uint8_t> *content = nullptr,
    const std::vector<int32_t> *shape = nullptr) {
  return fbsdata::CreateMatrix_u8(
      _fbb,
      content ? _fbb.CreateVector<uint8_t>(*content) : 0,
      shape ? _fbb.CreateVector<int32_t>(*shape) : 0);
}

}  // namespace fbsdata

#endif  // FLATBUFFERS_GENERATED_MATRIX_FBSDATA_H_