// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto

#ifndef PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto
#define PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto();
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {
class RealTimeCorrelativeScanMatcherOptions;
class RealTimeCorrelativeScanMatcherOptionsDefaultTypeInternal;
extern RealTimeCorrelativeScanMatcherOptionsDefaultTypeInternal _RealTimeCorrelativeScanMatcherOptions_default_instance_;
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions* Arena::CreateMaybeMessage<::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {

// ===================================================================

class RealTimeCorrelativeScanMatcherOptions final :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions) */ {
 public:
  RealTimeCorrelativeScanMatcherOptions();
  virtual ~RealTimeCorrelativeScanMatcherOptions();

  RealTimeCorrelativeScanMatcherOptions(const RealTimeCorrelativeScanMatcherOptions& from);

  inline RealTimeCorrelativeScanMatcherOptions& operator=(const RealTimeCorrelativeScanMatcherOptions& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RealTimeCorrelativeScanMatcherOptions(RealTimeCorrelativeScanMatcherOptions&& from) noexcept
    : RealTimeCorrelativeScanMatcherOptions() {
    *this = ::std::move(from);
  }

  inline RealTimeCorrelativeScanMatcherOptions& operator=(RealTimeCorrelativeScanMatcherOptions&& from) noexcept {
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
  static const RealTimeCorrelativeScanMatcherOptions& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RealTimeCorrelativeScanMatcherOptions* internal_default_instance() {
    return reinterpret_cast<const RealTimeCorrelativeScanMatcherOptions*>(
               &_RealTimeCorrelativeScanMatcherOptions_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(RealTimeCorrelativeScanMatcherOptions* other);
  friend void swap(RealTimeCorrelativeScanMatcherOptions& a, RealTimeCorrelativeScanMatcherOptions& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RealTimeCorrelativeScanMatcherOptions* New() const final {
    return CreateMaybeMessage<RealTimeCorrelativeScanMatcherOptions>(nullptr);
  }

  RealTimeCorrelativeScanMatcherOptions* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<RealTimeCorrelativeScanMatcherOptions>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const RealTimeCorrelativeScanMatcherOptions& from);
  void MergeFrom(const RealTimeCorrelativeScanMatcherOptions& from);
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
  void InternalSwap(RealTimeCorrelativeScanMatcherOptions* other);
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

  // double linear_search_window = 1;
  void clear_linear_search_window();
  static const int kLinearSearchWindowFieldNumber = 1;
  double linear_search_window() const;
  void set_linear_search_window(double value);

  // double angular_search_window = 2;
  void clear_angular_search_window();
  static const int kAngularSearchWindowFieldNumber = 2;
  double angular_search_window() const;
  void set_angular_search_window(double value);

  // double translation_delta_cost_weight = 3;
  void clear_translation_delta_cost_weight();
  static const int kTranslationDeltaCostWeightFieldNumber = 3;
  double translation_delta_cost_weight() const;
  void set_translation_delta_cost_weight(double value);

  // double rotation_delta_cost_weight = 4;
  void clear_rotation_delta_cost_weight();
  static const int kRotationDeltaCostWeightFieldNumber = 4;
  double rotation_delta_cost_weight() const;
  void set_rotation_delta_cost_weight(double value);

  // @@protoc_insertion_point(class_scope:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double linear_search_window_;
  double angular_search_window_;
  double translation_delta_cost_weight_;
  double rotation_delta_cost_weight_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RealTimeCorrelativeScanMatcherOptions

// double linear_search_window = 1;
inline void RealTimeCorrelativeScanMatcherOptions::clear_linear_search_window() {
  linear_search_window_ = 0;
}
inline double RealTimeCorrelativeScanMatcherOptions::linear_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.linear_search_window)
  return linear_search_window_;
}
inline void RealTimeCorrelativeScanMatcherOptions::set_linear_search_window(double value) {
  
  linear_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.linear_search_window)
}

// double angular_search_window = 2;
inline void RealTimeCorrelativeScanMatcherOptions::clear_angular_search_window() {
  angular_search_window_ = 0;
}
inline double RealTimeCorrelativeScanMatcherOptions::angular_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.angular_search_window)
  return angular_search_window_;
}
inline void RealTimeCorrelativeScanMatcherOptions::set_angular_search_window(double value) {
  
  angular_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.angular_search_window)
}

// double translation_delta_cost_weight = 3;
inline void RealTimeCorrelativeScanMatcherOptions::clear_translation_delta_cost_weight() {
  translation_delta_cost_weight_ = 0;
}
inline double RealTimeCorrelativeScanMatcherOptions::translation_delta_cost_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.translation_delta_cost_weight)
  return translation_delta_cost_weight_;
}
inline void RealTimeCorrelativeScanMatcherOptions::set_translation_delta_cost_weight(double value) {
  
  translation_delta_cost_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.translation_delta_cost_weight)
}

// double rotation_delta_cost_weight = 4;
inline void RealTimeCorrelativeScanMatcherOptions::clear_rotation_delta_cost_weight() {
  rotation_delta_cost_weight_ = 0;
}
inline double RealTimeCorrelativeScanMatcherOptions::rotation_delta_cost_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.rotation_delta_cost_weight)
  return rotation_delta_cost_weight_;
}
inline void RealTimeCorrelativeScanMatcherOptions::set_rotation_delta_cost_weight(double value) {
  
  rotation_delta_cost_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.rotation_delta_cost_weight)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto
