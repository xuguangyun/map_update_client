// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/motion_filter_options.proto

#include "cartographer/mapping/proto/motion_filter_options.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

namespace cartographer {
namespace mapping {
namespace proto {
class MotionFilterOptionsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<MotionFilterOptions> _instance;
} _MotionFilterOptions_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsMotionFilterOptions_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_MotionFilterOptions_default_instance_;
    new (ptr) ::cartographer::mapping::proto::MotionFilterOptions();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::MotionFilterOptions::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_MotionFilterOptions_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsMotionFilterOptions_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto}, {}};

void InitDefaults_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_MotionFilterOptions_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto[1];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::MotionFilterOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::MotionFilterOptions, max_time_seconds_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::MotionFilterOptions, max_distance_meters_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::MotionFilterOptions, max_angle_radians_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::MotionFilterOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::mapping::proto::_MotionFilterOptions_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto = {
  {}, AddDescriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, "cartographer/mapping/proto/motion_filter_options.proto", schemas,
  file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto,
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto[] =
  "\n6cartographer/mapping/proto/motion_filt"
  "er_options.proto\022\032cartographer.mapping.p"
  "roto\"g\n\023MotionFilterOptions\022\030\n\020max_time_"
  "seconds\030\001 \001(\001\022\033\n\023max_distance_meters\030\002 \001"
  "(\001\022\031\n\021max_angle_radians\030\003 \001(\001b\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto = {
  false, InitDefaults_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, 
  descriptor_table_protodef_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto,
  "cartographer/mapping/proto/motion_filter_options.proto", &assign_descriptors_table_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, 197,
};

void AddDescriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto, deps, 0);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto = []() { AddDescriptors_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto(); return true; }();
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void MotionFilterOptions::InitAsDefaultInstance() {
}
class MotionFilterOptions::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MotionFilterOptions::kMaxTimeSecondsFieldNumber;
const int MotionFilterOptions::kMaxDistanceMetersFieldNumber;
const int MotionFilterOptions::kMaxAngleRadiansFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MotionFilterOptions::MotionFilterOptions()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.MotionFilterOptions)
}
MotionFilterOptions::MotionFilterOptions(const MotionFilterOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&max_time_seconds_, &from.max_time_seconds_,
    static_cast<size_t>(reinterpret_cast<char*>(&max_angle_radians_) -
    reinterpret_cast<char*>(&max_time_seconds_)) + sizeof(max_angle_radians_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.MotionFilterOptions)
}

void MotionFilterOptions::SharedCtor() {
  ::memset(&max_time_seconds_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&max_angle_radians_) -
      reinterpret_cast<char*>(&max_time_seconds_)) + sizeof(max_angle_radians_));
}

MotionFilterOptions::~MotionFilterOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.MotionFilterOptions)
  SharedDtor();
}

void MotionFilterOptions::SharedDtor() {
}

void MotionFilterOptions::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const MotionFilterOptions& MotionFilterOptions::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_MotionFilterOptions_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto.base);
  return *internal_default_instance();
}


void MotionFilterOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.MotionFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&max_time_seconds_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&max_angle_radians_) -
      reinterpret_cast<char*>(&max_time_seconds_)) + sizeof(max_angle_radians_));
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* MotionFilterOptions::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<MotionFilterOptions*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // double max_time_seconds = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 9) goto handle_unusual;
        msg->set_max_time_seconds(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double max_distance_meters = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 17) goto handle_unusual;
        msg->set_max_distance_meters(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double max_angle_radians = 3;
      case 3: {
        if (static_cast<::google::protobuf::uint8>(tag) != 25) goto handle_unusual;
        msg->set_max_angle_radians(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->EndGroup(tag);
          return ptr;
        }
        auto res = UnknownFieldParse(tag, {_InternalParse, msg},
          ptr, end, msg->_internal_metadata_.mutable_unknown_fields(), ctx);
        ptr = res.first;
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr != nullptr);
        if (res.second) return ptr;
      }
    }  // switch
  }  // while
  return ptr;
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool MotionFilterOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.MotionFilterOptions)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double max_time_seconds = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (9 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_time_seconds_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_distance_meters = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (17 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_distance_meters_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_angle_radians = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (25 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_angle_radians_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.MotionFilterOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.MotionFilterOptions)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void MotionFilterOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.MotionFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double max_time_seconds = 1;
  if (this->max_time_seconds() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->max_time_seconds(), output);
  }

  // double max_distance_meters = 2;
  if (this->max_distance_meters() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->max_distance_meters(), output);
  }

  // double max_angle_radians = 3;
  if (this->max_angle_radians() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->max_angle_radians(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.MotionFilterOptions)
}

::google::protobuf::uint8* MotionFilterOptions::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.MotionFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double max_time_seconds = 1;
  if (this->max_time_seconds() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->max_time_seconds(), target);
  }

  // double max_distance_meters = 2;
  if (this->max_distance_meters() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->max_distance_meters(), target);
  }

  // double max_angle_radians = 3;
  if (this->max_angle_radians() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->max_angle_radians(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.MotionFilterOptions)
  return target;
}

size_t MotionFilterOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.MotionFilterOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double max_time_seconds = 1;
  if (this->max_time_seconds() != 0) {
    total_size += 1 + 8;
  }

  // double max_distance_meters = 2;
  if (this->max_distance_meters() != 0) {
    total_size += 1 + 8;
  }

  // double max_angle_radians = 3;
  if (this->max_angle_radians() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void MotionFilterOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.MotionFilterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const MotionFilterOptions* source =
      ::google::protobuf::DynamicCastToGenerated<MotionFilterOptions>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.MotionFilterOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.MotionFilterOptions)
    MergeFrom(*source);
  }
}

void MotionFilterOptions::MergeFrom(const MotionFilterOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.MotionFilterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.max_time_seconds() != 0) {
    set_max_time_seconds(from.max_time_seconds());
  }
  if (from.max_distance_meters() != 0) {
    set_max_distance_meters(from.max_distance_meters());
  }
  if (from.max_angle_radians() != 0) {
    set_max_angle_radians(from.max_angle_radians());
  }
}

void MotionFilterOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.MotionFilterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MotionFilterOptions::CopyFrom(const MotionFilterOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.MotionFilterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MotionFilterOptions::IsInitialized() const {
  return true;
}

void MotionFilterOptions::Swap(MotionFilterOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MotionFilterOptions::InternalSwap(MotionFilterOptions* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(max_time_seconds_, other->max_time_seconds_);
  swap(max_distance_meters_, other->max_distance_meters_);
  swap(max_angle_radians_, other->max_angle_radians_);
}

::google::protobuf::Metadata MotionFilterOptions::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto);
  return ::file_level_metadata_cartographer_2fmapping_2fproto_2fmotion_5ffilter_5foptions_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::MotionFilterOptions* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::MotionFilterOptions >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::MotionFilterOptions >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
