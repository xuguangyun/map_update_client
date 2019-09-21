// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/2d/grid_2d.proto

#ifndef PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto
#define PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3007000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3007000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "cartographer/mapping/proto/2d/map_limits.pb.h"
#include "cartographer/mapping/proto/2d/probability_grid.pb.h"
#include "cartographer/mapping/proto/2d/tsdf_2d.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto();
namespace cartographer {
namespace mapping {
namespace proto {
class Grid2D;
class Grid2DDefaultTypeInternal;
extern Grid2DDefaultTypeInternal _Grid2D_default_instance_;
class Grid2D_CellBox;
class Grid2D_CellBoxDefaultTypeInternal;
extern Grid2D_CellBoxDefaultTypeInternal _Grid2D_CellBox_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> ::cartographer::mapping::proto::Grid2D* Arena::CreateMaybeMessage<::cartographer::mapping::proto::Grid2D>(Arena*);
template<> ::cartographer::mapping::proto::Grid2D_CellBox* Arena::CreateMaybeMessage<::cartographer::mapping::proto::Grid2D_CellBox>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

class Grid2D_CellBox final :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.Grid2D.CellBox) */ {
 public:
  Grid2D_CellBox();
  virtual ~Grid2D_CellBox();

  Grid2D_CellBox(const Grid2D_CellBox& from);

  inline Grid2D_CellBox& operator=(const Grid2D_CellBox& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Grid2D_CellBox(Grid2D_CellBox&& from) noexcept
    : Grid2D_CellBox() {
    *this = ::std::move(from);
  }

  inline Grid2D_CellBox& operator=(Grid2D_CellBox&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Grid2D_CellBox& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Grid2D_CellBox* internal_default_instance() {
    return reinterpret_cast<const Grid2D_CellBox*>(
               &_Grid2D_CellBox_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Grid2D_CellBox* other);
  friend void swap(Grid2D_CellBox& a, Grid2D_CellBox& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Grid2D_CellBox* New() const final {
    return CreateMaybeMessage<Grid2D_CellBox>(nullptr);
  }

  Grid2D_CellBox* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Grid2D_CellBox>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Grid2D_CellBox& from);
  void MergeFrom(const Grid2D_CellBox& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Grid2D_CellBox* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // int32 max_x = 1;
  void clear_max_x();
  static const int kMaxXFieldNumber = 1;
  ::google::protobuf::int32 max_x() const;
  void set_max_x(::google::protobuf::int32 value);

  // int32 max_y = 2;
  void clear_max_y();
  static const int kMaxYFieldNumber = 2;
  ::google::protobuf::int32 max_y() const;
  void set_max_y(::google::protobuf::int32 value);

  // int32 min_x = 3;
  void clear_min_x();
  static const int kMinXFieldNumber = 3;
  ::google::protobuf::int32 min_x() const;
  void set_min_x(::google::protobuf::int32 value);

  // int32 min_y = 4;
  void clear_min_y();
  static const int kMinYFieldNumber = 4;
  ::google::protobuf::int32 min_y() const;
  void set_min_y(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.Grid2D.CellBox)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 max_x_;
  ::google::protobuf::int32 max_y_;
  ::google::protobuf::int32 min_x_;
  ::google::protobuf::int32 min_y_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto;
};
// -------------------------------------------------------------------

class Grid2D final :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.Grid2D) */ {
 public:
  Grid2D();
  virtual ~Grid2D();

  Grid2D(const Grid2D& from);

  inline Grid2D& operator=(const Grid2D& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Grid2D(Grid2D&& from) noexcept
    : Grid2D() {
    *this = ::std::move(from);
  }

  inline Grid2D& operator=(Grid2D&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Grid2D& default_instance();

  enum GridCase {
    kProbabilityGrid2D = 4,
    kTsdf2D = 5,
    GRID_NOT_SET = 0,
  };

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Grid2D* internal_default_instance() {
    return reinterpret_cast<const Grid2D*>(
               &_Grid2D_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Grid2D* other);
  friend void swap(Grid2D& a, Grid2D& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Grid2D* New() const final {
    return CreateMaybeMessage<Grid2D>(nullptr);
  }

  Grid2D* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Grid2D>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Grid2D& from);
  void MergeFrom(const Grid2D& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Grid2D* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef Grid2D_CellBox CellBox;

  // accessors -------------------------------------------------------

  // repeated int32 cells = 2;
  int cells_size() const;
  void clear_cells();
  static const int kCellsFieldNumber = 2;
  ::google::protobuf::int32 cells(int index) const;
  void set_cells(int index, ::google::protobuf::int32 value);
  void add_cells(::google::protobuf::int32 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      cells() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_cells();

  // .cartographer.mapping.proto.MapLimits limits = 1;
  bool has_limits() const;
  void clear_limits();
  static const int kLimitsFieldNumber = 1;
  const ::cartographer::mapping::proto::MapLimits& limits() const;
  ::cartographer::mapping::proto::MapLimits* release_limits();
  ::cartographer::mapping::proto::MapLimits* mutable_limits();
  void set_allocated_limits(::cartographer::mapping::proto::MapLimits* limits);

  // .cartographer.mapping.proto.Grid2D.CellBox known_cells_box = 3;
  bool has_known_cells_box() const;
  void clear_known_cells_box();
  static const int kKnownCellsBoxFieldNumber = 3;
  const ::cartographer::mapping::proto::Grid2D_CellBox& known_cells_box() const;
  ::cartographer::mapping::proto::Grid2D_CellBox* release_known_cells_box();
  ::cartographer::mapping::proto::Grid2D_CellBox* mutable_known_cells_box();
  void set_allocated_known_cells_box(::cartographer::mapping::proto::Grid2D_CellBox* known_cells_box);

  // float min_correspondence_cost = 6;
  void clear_min_correspondence_cost();
  static const int kMinCorrespondenceCostFieldNumber = 6;
  float min_correspondence_cost() const;
  void set_min_correspondence_cost(float value);

  // float max_correspondence_cost = 7;
  void clear_max_correspondence_cost();
  static const int kMaxCorrespondenceCostFieldNumber = 7;
  float max_correspondence_cost() const;
  void set_max_correspondence_cost(float value);

  // .cartographer.mapping.proto.ProbabilityGrid probability_grid_2d = 4;
  bool has_probability_grid_2d() const;
  void clear_probability_grid_2d();
  static const int kProbabilityGrid2DFieldNumber = 4;
  const ::cartographer::mapping::proto::ProbabilityGrid& probability_grid_2d() const;
  ::cartographer::mapping::proto::ProbabilityGrid* release_probability_grid_2d();
  ::cartographer::mapping::proto::ProbabilityGrid* mutable_probability_grid_2d();
  void set_allocated_probability_grid_2d(::cartographer::mapping::proto::ProbabilityGrid* probability_grid_2d);

  // .cartographer.mapping.proto.TSDF2D tsdf_2d = 5;
  bool has_tsdf_2d() const;
  void clear_tsdf_2d();
  static const int kTsdf2DFieldNumber = 5;
  const ::cartographer::mapping::proto::TSDF2D& tsdf_2d() const;
  ::cartographer::mapping::proto::TSDF2D* release_tsdf_2d();
  ::cartographer::mapping::proto::TSDF2D* mutable_tsdf_2d();
  void set_allocated_tsdf_2d(::cartographer::mapping::proto::TSDF2D* tsdf_2d);

  void clear_grid();
  GridCase grid_case() const;
  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.Grid2D)
 private:
  class HasBitSetters;
  void set_has_probability_grid_2d();
  void set_has_tsdf_2d();

  inline bool has_grid() const;
  inline void clear_has_grid();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > cells_;
  mutable std::atomic<int> _cells_cached_byte_size_;
  ::cartographer::mapping::proto::MapLimits* limits_;
  ::cartographer::mapping::proto::Grid2D_CellBox* known_cells_box_;
  float min_correspondence_cost_;
  float max_correspondence_cost_;
  union GridUnion {
    GridUnion() {}
    ::cartographer::mapping::proto::ProbabilityGrid* probability_grid_2d_;
    ::cartographer::mapping::proto::TSDF2D* tsdf_2d_;
  } grid_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::uint32 _oneof_case_[1];

  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Grid2D_CellBox

// int32 max_x = 1;
inline void Grid2D_CellBox::clear_max_x() {
  max_x_ = 0;
}
inline ::google::protobuf::int32 Grid2D_CellBox::max_x() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.CellBox.max_x)
  return max_x_;
}
inline void Grid2D_CellBox::set_max_x(::google::protobuf::int32 value) {
  
  max_x_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.CellBox.max_x)
}

// int32 max_y = 2;
inline void Grid2D_CellBox::clear_max_y() {
  max_y_ = 0;
}
inline ::google::protobuf::int32 Grid2D_CellBox::max_y() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.CellBox.max_y)
  return max_y_;
}
inline void Grid2D_CellBox::set_max_y(::google::protobuf::int32 value) {
  
  max_y_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.CellBox.max_y)
}

// int32 min_x = 3;
inline void Grid2D_CellBox::clear_min_x() {
  min_x_ = 0;
}
inline ::google::protobuf::int32 Grid2D_CellBox::min_x() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.CellBox.min_x)
  return min_x_;
}
inline void Grid2D_CellBox::set_min_x(::google::protobuf::int32 value) {
  
  min_x_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.CellBox.min_x)
}

// int32 min_y = 4;
inline void Grid2D_CellBox::clear_min_y() {
  min_y_ = 0;
}
inline ::google::protobuf::int32 Grid2D_CellBox::min_y() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.CellBox.min_y)
  return min_y_;
}
inline void Grid2D_CellBox::set_min_y(::google::protobuf::int32 value) {
  
  min_y_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.CellBox.min_y)
}

// -------------------------------------------------------------------

// Grid2D

// .cartographer.mapping.proto.MapLimits limits = 1;
inline bool Grid2D::has_limits() const {
  return this != internal_default_instance() && limits_ != nullptr;
}
inline const ::cartographer::mapping::proto::MapLimits& Grid2D::limits() const {
  const ::cartographer::mapping::proto::MapLimits* p = limits_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.limits)
  return p != nullptr ? *p : *reinterpret_cast<const ::cartographer::mapping::proto::MapLimits*>(
      &::cartographer::mapping::proto::_MapLimits_default_instance_);
}
inline ::cartographer::mapping::proto::MapLimits* Grid2D::release_limits() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.Grid2D.limits)
  
  ::cartographer::mapping::proto::MapLimits* temp = limits_;
  limits_ = nullptr;
  return temp;
}
inline ::cartographer::mapping::proto::MapLimits* Grid2D::mutable_limits() {
  
  if (limits_ == nullptr) {
    auto* p = CreateMaybeMessage<::cartographer::mapping::proto::MapLimits>(GetArenaNoVirtual());
    limits_ = p;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.Grid2D.limits)
  return limits_;
}
inline void Grid2D::set_allocated_limits(::cartographer::mapping::proto::MapLimits* limits) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(limits_);
  }
  if (limits) {
    ::google::protobuf::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      limits = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, limits, submessage_arena);
    }
    
  } else {
    
  }
  limits_ = limits;
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.Grid2D.limits)
}

// repeated int32 cells = 2;
inline int Grid2D::cells_size() const {
  return cells_.size();
}
inline void Grid2D::clear_cells() {
  cells_.Clear();
}
inline ::google::protobuf::int32 Grid2D::cells(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.cells)
  return cells_.Get(index);
}
inline void Grid2D::set_cells(int index, ::google::protobuf::int32 value) {
  cells_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.cells)
}
inline void Grid2D::add_cells(::google::protobuf::int32 value) {
  cells_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.Grid2D.cells)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
Grid2D::cells() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.Grid2D.cells)
  return cells_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
Grid2D::mutable_cells() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.Grid2D.cells)
  return &cells_;
}

// .cartographer.mapping.proto.Grid2D.CellBox known_cells_box = 3;
inline bool Grid2D::has_known_cells_box() const {
  return this != internal_default_instance() && known_cells_box_ != nullptr;
}
inline void Grid2D::clear_known_cells_box() {
  if (GetArenaNoVirtual() == nullptr && known_cells_box_ != nullptr) {
    delete known_cells_box_;
  }
  known_cells_box_ = nullptr;
}
inline const ::cartographer::mapping::proto::Grid2D_CellBox& Grid2D::known_cells_box() const {
  const ::cartographer::mapping::proto::Grid2D_CellBox* p = known_cells_box_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.known_cells_box)
  return p != nullptr ? *p : *reinterpret_cast<const ::cartographer::mapping::proto::Grid2D_CellBox*>(
      &::cartographer::mapping::proto::_Grid2D_CellBox_default_instance_);
}
inline ::cartographer::mapping::proto::Grid2D_CellBox* Grid2D::release_known_cells_box() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.Grid2D.known_cells_box)
  
  ::cartographer::mapping::proto::Grid2D_CellBox* temp = known_cells_box_;
  known_cells_box_ = nullptr;
  return temp;
}
inline ::cartographer::mapping::proto::Grid2D_CellBox* Grid2D::mutable_known_cells_box() {
  
  if (known_cells_box_ == nullptr) {
    auto* p = CreateMaybeMessage<::cartographer::mapping::proto::Grid2D_CellBox>(GetArenaNoVirtual());
    known_cells_box_ = p;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.Grid2D.known_cells_box)
  return known_cells_box_;
}
inline void Grid2D::set_allocated_known_cells_box(::cartographer::mapping::proto::Grid2D_CellBox* known_cells_box) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete known_cells_box_;
  }
  if (known_cells_box) {
    ::google::protobuf::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      known_cells_box = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, known_cells_box, submessage_arena);
    }
    
  } else {
    
  }
  known_cells_box_ = known_cells_box;
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.Grid2D.known_cells_box)
}

// .cartographer.mapping.proto.ProbabilityGrid probability_grid_2d = 4;
inline bool Grid2D::has_probability_grid_2d() const {
  return grid_case() == kProbabilityGrid2D;
}
inline void Grid2D::set_has_probability_grid_2d() {
  _oneof_case_[0] = kProbabilityGrid2D;
}
inline ::cartographer::mapping::proto::ProbabilityGrid* Grid2D::release_probability_grid_2d() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.Grid2D.probability_grid_2d)
  if (has_probability_grid_2d()) {
    clear_has_grid();
      ::cartographer::mapping::proto::ProbabilityGrid* temp = grid_.probability_grid_2d_;
    grid_.probability_grid_2d_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline const ::cartographer::mapping::proto::ProbabilityGrid& Grid2D::probability_grid_2d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.probability_grid_2d)
  return has_probability_grid_2d()
      ? *grid_.probability_grid_2d_
      : *reinterpret_cast< ::cartographer::mapping::proto::ProbabilityGrid*>(&::cartographer::mapping::proto::_ProbabilityGrid_default_instance_);
}
inline ::cartographer::mapping::proto::ProbabilityGrid* Grid2D::mutable_probability_grid_2d() {
  if (!has_probability_grid_2d()) {
    clear_grid();
    set_has_probability_grid_2d();
    grid_.probability_grid_2d_ = CreateMaybeMessage< ::cartographer::mapping::proto::ProbabilityGrid >(
        GetArenaNoVirtual());
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.Grid2D.probability_grid_2d)
  return grid_.probability_grid_2d_;
}

// .cartographer.mapping.proto.TSDF2D tsdf_2d = 5;
inline bool Grid2D::has_tsdf_2d() const {
  return grid_case() == kTsdf2D;
}
inline void Grid2D::set_has_tsdf_2d() {
  _oneof_case_[0] = kTsdf2D;
}
inline ::cartographer::mapping::proto::TSDF2D* Grid2D::release_tsdf_2d() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.Grid2D.tsdf_2d)
  if (has_tsdf_2d()) {
    clear_has_grid();
      ::cartographer::mapping::proto::TSDF2D* temp = grid_.tsdf_2d_;
    grid_.tsdf_2d_ = nullptr;
    return temp;
  } else {
    return nullptr;
  }
}
inline const ::cartographer::mapping::proto::TSDF2D& Grid2D::tsdf_2d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.tsdf_2d)
  return has_tsdf_2d()
      ? *grid_.tsdf_2d_
      : *reinterpret_cast< ::cartographer::mapping::proto::TSDF2D*>(&::cartographer::mapping::proto::_TSDF2D_default_instance_);
}
inline ::cartographer::mapping::proto::TSDF2D* Grid2D::mutable_tsdf_2d() {
  if (!has_tsdf_2d()) {
    clear_grid();
    set_has_tsdf_2d();
    grid_.tsdf_2d_ = CreateMaybeMessage< ::cartographer::mapping::proto::TSDF2D >(
        GetArenaNoVirtual());
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.Grid2D.tsdf_2d)
  return grid_.tsdf_2d_;
}

// float min_correspondence_cost = 6;
inline void Grid2D::clear_min_correspondence_cost() {
  min_correspondence_cost_ = 0;
}
inline float Grid2D::min_correspondence_cost() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.min_correspondence_cost)
  return min_correspondence_cost_;
}
inline void Grid2D::set_min_correspondence_cost(float value) {
  
  min_correspondence_cost_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.min_correspondence_cost)
}

// float max_correspondence_cost = 7;
inline void Grid2D::clear_max_correspondence_cost() {
  max_correspondence_cost_ = 0;
}
inline float Grid2D::max_correspondence_cost() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.Grid2D.max_correspondence_cost)
  return max_correspondence_cost_;
}
inline void Grid2D::set_max_correspondence_cost(float value) {
  
  max_correspondence_cost_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.Grid2D.max_correspondence_cost)
}

inline bool Grid2D::has_grid() const {
  return grid_case() != GRID_NOT_SET;
}
inline void Grid2D::clear_has_grid() {
  _oneof_case_[0] = GRID_NOT_SET;
}
inline Grid2D::GridCase Grid2D::grid_case() const {
  return Grid2D::GridCase(_oneof_case_[0]);
}
#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2f2d_2fgrid_5f2d_2eproto