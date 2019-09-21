// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/3d/range_data_inserter_options_3d.proto

#include "cartographer/mapping/proto/3d/range_data_inserter_options_3d.pb.h"

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
class RangeDataInserterOptions3DDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<RangeDataInserterOptions3D> _instance;
} _RangeDataInserterOptions3D_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
static void InitDefaultsRangeDataInserterOptions3D_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::cartographer::mapping::proto::_RangeDataInserterOptions3D_default_instance_;
    new (ptr) ::cartographer::mapping::proto::RangeDataInserterOptions3D();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::proto::RangeDataInserterOptions3D::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_RangeDataInserterOptions3D_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsRangeDataInserterOptions3D_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto}, {}};

void InitDefaults_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto() {
  ::google::protobuf::internal::InitSCC(&scc_info_RangeDataInserterOptions3D_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto.base);
}

::google::protobuf::Metadata file_level_metadata_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto[1];
constexpr ::google::protobuf::EnumDescriptor const** file_level_enum_descriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto = nullptr;
constexpr ::google::protobuf::ServiceDescriptor const** file_level_service_descriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto = nullptr;

const ::google::protobuf::uint32 TableStruct_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions3D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions3D, hit_probability_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions3D, miss_probability_),
  PROTOBUF_FIELD_OFFSET(::cartographer::mapping::proto::RangeDataInserterOptions3D, num_free_space_voxels_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::proto::RangeDataInserterOptions3D)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::mapping::proto::_RangeDataInserterOptions3D_default_instance_),
};

::google::protobuf::internal::AssignDescriptorsTable assign_descriptors_table_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto = {
  {}, AddDescriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, "cartographer/mapping/proto/3d/range_data_inserter_options_3d.proto", schemas,
  file_default_instances, TableStruct_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto::offsets,
  file_level_metadata_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, 1, file_level_enum_descriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, file_level_service_descriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto,
};

const char descriptor_table_protodef_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto[] =
  "\nBcartographer/mapping/proto/3d/range_da"
  "ta_inserter_options_3d.proto\022\032cartograph"
  "er.mapping.proto\"n\n\032RangeDataInserterOpt"
  "ions3D\022\027\n\017hit_probability\030\001 \001(\001\022\030\n\020miss_"
  "probability\030\002 \001(\001\022\035\n\025num_free_space_voxe"
  "ls\030\003 \001(\005b\006proto3"
  ;
::google::protobuf::internal::DescriptorTable descriptor_table_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto = {
  false, InitDefaults_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, 
  descriptor_table_protodef_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto,
  "cartographer/mapping/proto/3d/range_data_inserter_options_3d.proto", &assign_descriptors_table_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, 216,
};

void AddDescriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto() {
  static constexpr ::google::protobuf::internal::InitFunc deps[1] =
  {
  };
 ::google::protobuf::internal::AddDescriptors(&descriptor_table_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto, deps, 0);
}

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto = []() { AddDescriptors_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto(); return true; }();
namespace cartographer {
namespace mapping {
namespace proto {

// ===================================================================

void RangeDataInserterOptions3D::InitAsDefaultInstance() {
}
class RangeDataInserterOptions3D::HasBitSetters {
 public:
};

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RangeDataInserterOptions3D::kHitProbabilityFieldNumber;
const int RangeDataInserterOptions3D::kMissProbabilityFieldNumber;
const int RangeDataInserterOptions3D::kNumFreeSpaceVoxelsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RangeDataInserterOptions3D::RangeDataInserterOptions3D()
  : ::google::protobuf::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.RangeDataInserterOptions3D)
}
RangeDataInserterOptions3D::RangeDataInserterOptions3D(const RangeDataInserterOptions3D& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(nullptr) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&hit_probability_, &from.hit_probability_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_free_space_voxels_) -
    reinterpret_cast<char*>(&hit_probability_)) + sizeof(num_free_space_voxels_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.RangeDataInserterOptions3D)
}

void RangeDataInserterOptions3D::SharedCtor() {
  ::memset(&hit_probability_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_free_space_voxels_) -
      reinterpret_cast<char*>(&hit_probability_)) + sizeof(num_free_space_voxels_));
}

RangeDataInserterOptions3D::~RangeDataInserterOptions3D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.RangeDataInserterOptions3D)
  SharedDtor();
}

void RangeDataInserterOptions3D::SharedDtor() {
}

void RangeDataInserterOptions3D::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const RangeDataInserterOptions3D& RangeDataInserterOptions3D::default_instance() {
  ::google::protobuf::internal::InitSCC(&::scc_info_RangeDataInserterOptions3D_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto.base);
  return *internal_default_instance();
}


void RangeDataInserterOptions3D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&hit_probability_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_free_space_voxels_) -
      reinterpret_cast<char*>(&hit_probability_)) + sizeof(num_free_space_voxels_));
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* RangeDataInserterOptions3D::_InternalParse(const char* begin, const char* end, void* object,
                  ::google::protobuf::internal::ParseContext* ctx) {
  auto msg = static_cast<RangeDataInserterOptions3D*>(object);
  ::google::protobuf::int32 size; (void)size;
  int depth; (void)depth;
  ::google::protobuf::uint32 tag;
  ::google::protobuf::internal::ParseFunc parser_till_end; (void)parser_till_end;
  auto ptr = begin;
  while (ptr < end) {
    ptr = ::google::protobuf::io::Parse32(ptr, &tag);
    GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
    switch (tag >> 3) {
      // double hit_probability = 1;
      case 1: {
        if (static_cast<::google::protobuf::uint8>(tag) != 9) goto handle_unusual;
        msg->set_hit_probability(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // double miss_probability = 2;
      case 2: {
        if (static_cast<::google::protobuf::uint8>(tag) != 17) goto handle_unusual;
        msg->set_miss_probability(::google::protobuf::io::UnalignedLoad<double>(ptr));
        ptr += sizeof(double);
        break;
      }
      // int32 num_free_space_voxels = 3;
      case 3: {
        if (static_cast<::google::protobuf::uint8>(tag) != 24) goto handle_unusual;
        msg->set_num_free_space_voxels(::google::protobuf::internal::ReadVarint(&ptr));
        GOOGLE_PROTOBUF_PARSER_ASSERT(ptr);
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
bool RangeDataInserterOptions3D::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double hit_probability = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (9 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &hit_probability_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double miss_probability = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (17 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &miss_probability_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 num_free_space_voxels = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) == (24 & 0xFF)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_free_space_voxels_)));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.RangeDataInserterOptions3D)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.RangeDataInserterOptions3D)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void RangeDataInserterOptions3D::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double hit_probability = 1;
  if (this->hit_probability() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->hit_probability(), output);
  }

  // double miss_probability = 2;
  if (this->miss_probability() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->miss_probability(), output);
  }

  // int32 num_free_space_voxels = 3;
  if (this->num_free_space_voxels() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->num_free_space_voxels(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.RangeDataInserterOptions3D)
}

::google::protobuf::uint8* RangeDataInserterOptions3D::InternalSerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double hit_probability = 1;
  if (this->hit_probability() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->hit_probability(), target);
  }

  // double miss_probability = 2;
  if (this->miss_probability() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->miss_probability(), target);
  }

  // int32 num_free_space_voxels = 3;
  if (this->num_free_space_voxels() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->num_free_space_voxels(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.RangeDataInserterOptions3D)
  return target;
}

size_t RangeDataInserterOptions3D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double hit_probability = 1;
  if (this->hit_probability() != 0) {
    total_size += 1 + 8;
  }

  // double miss_probability = 2;
  if (this->miss_probability() != 0) {
    total_size += 1 + 8;
  }

  // int32 num_free_space_voxels = 3;
  if (this->num_free_space_voxels() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->num_free_space_voxels());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void RangeDataInserterOptions3D::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  GOOGLE_DCHECK_NE(&from, this);
  const RangeDataInserterOptions3D* source =
      ::google::protobuf::DynamicCastToGenerated<RangeDataInserterOptions3D>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.RangeDataInserterOptions3D)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.RangeDataInserterOptions3D)
    MergeFrom(*source);
  }
}

void RangeDataInserterOptions3D::MergeFrom(const RangeDataInserterOptions3D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.hit_probability() != 0) {
    set_hit_probability(from.hit_probability());
  }
  if (from.miss_probability() != 0) {
    set_miss_probability(from.miss_probability());
  }
  if (from.num_free_space_voxels() != 0) {
    set_num_free_space_voxels(from.num_free_space_voxels());
  }
}

void RangeDataInserterOptions3D::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RangeDataInserterOptions3D::CopyFrom(const RangeDataInserterOptions3D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.RangeDataInserterOptions3D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RangeDataInserterOptions3D::IsInitialized() const {
  return true;
}

void RangeDataInserterOptions3D::Swap(RangeDataInserterOptions3D* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RangeDataInserterOptions3D::InternalSwap(RangeDataInserterOptions3D* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(hit_probability_, other->hit_probability_);
  swap(miss_probability_, other->miss_probability_);
  swap(num_free_space_voxels_, other->num_free_space_voxels_);
}

::google::protobuf::Metadata RangeDataInserterOptions3D::GetMetadata() const {
  ::google::protobuf::internal::AssignDescriptors(&::assign_descriptors_table_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto);
  return ::file_level_metadata_cartographer_2fmapping_2fproto_2f3d_2frange_5fdata_5finserter_5foptions_5f3d_2eproto[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace mapping
}  // namespace cartographer
namespace google {
namespace protobuf {
template<> PROTOBUF_NOINLINE ::cartographer::mapping::proto::RangeDataInserterOptions3D* Arena::CreateMaybeMessage< ::cartographer::mapping::proto::RangeDataInserterOptions3D >(Arena* arena) {
  return Arena::CreateInternal< ::cartographer::mapping::proto::RangeDataInserterOptions3D >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
