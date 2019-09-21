// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto

#include "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

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
namespace scan_matching {
namespace proto {
class RealTimeCorrelativeScanMatcherOptionsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<RealTimeCorrelativeScanMatcherOptions> _instance;
} _RealTimeCorrelativeScanMatcherOptions_default_instance_;
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsRealTimeCorrelativeScanMatcherOptions_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::scan_matching::proto::_RealTimeCorrelativeScanMatcherOptions_default_instance_;
    new (ptr) ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_RealTimeCorrelativeScanMatcherOptions_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsRealTimeCorrelativeScanMatcherOptions_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto}, {}};

void InitDefaults_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_RealTimeCorrelativeScanMatcherOptions_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto[1];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions, linear_search_window_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions, angular_search_window_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions, translation_delta_cost_weight_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions, rotation_delta_cost_weight_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::mapping::scan_matching::proto::_RealTimeCorrelativeScanMatcherOptions_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto = {
  {}, AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto", schemas,
  file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto,
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto[] =
  "\nYcartographer/mapping/proto/scan_matchi"
  "ng/real_time_correlative_scan_matcher_op"
  "tions.proto\022(cartographer.mapping.scan_m"
  "atching.proto\"\257\001\n%RealTimeCorrelativeSca"
  "nMatcherOptions\022\034\n\024linear_search_window\030"
  "\001 \001(\001\022\035\n\025angular_search_window\030\002 \001(\001\022%\n\035"
  "translation_delta_cost_weight\030\003 \001(\001\022\"\n\032r"
  "otation_delta_cost_weight\030\004 \001(\001b\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto = {
  false, InitDefaults_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, 
  descriptor_table_protodef_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto,
  "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto", &assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, 319,
};

void AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto, deps, 0);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto = []() { AddDescriptors_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto(); return true; }();
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {

// ===================================================================

void RealTimeCorrelativeScanMatcherOptions::InitAsDefaultInstance() {
}
class RealTimeCorrelativeScanMatcherOptions::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RealTimeCorrelativeScanMatcherOptions::kLinearSearchWindowFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kAngularSearchWindowFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kTranslationDeltaCostWeightFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kRotationDeltaCostWeightFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RealTimeCorrelativeScanMatcherOptions::RealTimeCorrelativeScanMatcherOptions()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}
RealTimeCorrelativeScanMatcherOptions::RealTimeCorrelativeScanMatcherOptions(const RealTimeCorrelativeScanMatcherOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&linear_search_window_, &from.linear_search_window_,
    static_cast<size_t>(reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
    reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}

void RealTimeCorrelativeScanMatcherOptions::SharedCtor() {
  ::memset(&linear_search_window_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
      reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
}

RealTimeCorrelativeScanMatcherOptions::~RealTimeCorrelativeScanMatcherOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  SharedDtor();
}

void RealTimeCorrelativeScanMatcherOptions::SharedDtor() {
}

void RealTimeCorrelativeScanMatcherOptions::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const RealTimeCorrelativeScanMatcherOptions& RealTimeCorrelativeScanMatcherOptions::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_RealTimeCorrelativeScanMatcherOptions_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto.base);
  return *internal_default_instance();
}


void RealTimeCorrelativeScanMatcherOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&linear_search_window_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
      reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* RealTimeCorrelativeScanMatcherOptions::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<RealTimeCorrelativeScanMatcherOptions*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // double linear_search_window = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 9) goto handle_unusual;
        msg->set_linear_search_window(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double angular_search_window = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 17) goto handle_unusual;
        msg->set_angular_search_window(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double translation_delta_cost_weight = 3;
      case 3: {
        if (static_cast<::google::protobuf::uint8>(tag) != 25) goto handle_unusual;
        msg->set_translation_delta_cost_weight(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double rotation_delta_cost_weight = 4;
      case 4: {
        if (static_cast<::google::protobuf::uint8>(tag) != 33) goto handle_unusual;
        msg->set_rotation_delta_cost_weight(::google::protobuf::io::UnalignedLoad<double>(ptr));
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
bool RealTimeCorrelativeScanMatcherOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double linear_search_window = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (9 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double angular_search_window = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (17 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &angular_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double translation_delta_cost_weight = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (25 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &translation_delta_cost_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double rotation_delta_cost_weight = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (33 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &rotation_delta_cost_weight_)));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void RealTimeCorrelativeScanMatcherOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->linear_search_window(), output);
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->angular_search_window(), output);
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->translation_delta_cost_weight(), output);
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->rotation_delta_cost_weight(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}

::google::protobuf::uint8* RealTimeCorrelativeScanMatcherOptions::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->linear_search_window(), target);
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->angular_search_window(), target);
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->translation_delta_cost_weight(), target);
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->rotation_delta_cost_weight(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return target;
}

size_t RealTimeCorrelativeScanMatcherOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    total_size += 1 + 8;
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    total_size += 1 + 8;
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    total_size += 1 + 8;
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void RealTimeCorrelativeScanMatcherOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const RealTimeCorrelativeScanMatcherOptions* source =
      ::google::protobuf::DynamicCastToGenerated<RealTimeCorrelativeScanMatcherOptions>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
    MergeFrom(*source);
  }
}

void RealTimeCorrelativeScanMatcherOptions::MergeFrom(const RealTimeCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.linear_search_window() != 0) {
    set_linear_search_window(from.linear_search_window());
  }
  if (from.angular_search_window() != 0) {
    set_angular_search_window(from.angular_search_window());
  }
  if (from.translation_delta_cost_weight() != 0) {
    set_translation_delta_cost_weight(from.translation_delta_cost_weight());
  }
  if (from.rotation_delta_cost_weight() != 0) {
    set_rotation_delta_cost_weight(from.rotation_delta_cost_weight());
  }
}

void RealTimeCorrelativeScanMatcherOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RealTimeCorrelativeScanMatcherOptions::CopyFrom(const RealTimeCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RealTimeCorrelativeScanMatcherOptions::IsInitialized() const {
  return true;
}

void RealTimeCorrelativeScanMatcherOptions::Swap(RealTimeCorrelativeScanMatcherOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RealTimeCorrelativeScanMatcherOptions::InternalSwap(RealTimeCorrelativeScanMatcherOptions* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(linear_search_window_, other->linear_search_window_);
  swap(angular_search_window_, other->angular_search_window_);
  swap(translation_delta_cost_weight_, other->translation_delta_cost_weight_);
  swap(rotation_delta_cost_weight_, other->rotation_delta_cost_weight_);
}

::google::protobuf::Metadata RealTimeCorrelativeScanMatcherOptions::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto);
  return ::file_level_metadata_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions* Arena::CreateMaybeMessage< ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::scan_matching::proto::RealTimeCorrelativeScanMatcherOptions >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>