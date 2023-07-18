/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/joao/PESTA/control_system/v3/dsdl_source/Autopilot_3.uavcan
 */

#ifndef DSDL_SOURCE_AUTOPILOT_3_HPP_INCLUDED
#define DSDL_SOURCE_AUTOPILOT_3_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <dsdl_source/SensorsData.hpp>
#include <dsdl_source/SensorsStatus.hpp>

/******************************* Source text **********************************
#SENSORS DATA
SensorsData[1] autopilot_index

#SENSORS STATUS
SensorsStatus[1] status_index
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl_source.Autopilot_3
dsdl_source.SensorsData[1] autopilot_index
dsdl_source.SensorsStatus[1] status_index
******************************************************************************/

#undef autopilot_index
#undef status_index

namespace dsdl_source
{

template <int _tmpl>
struct UAVCAN_EXPORT Autopilot_3_
{
    typedef const Autopilot_3_<_tmpl>& ParameterType;
    typedef Autopilot_3_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::dsdl_source::SensorsData, ::uavcan::ArrayModeStatic, 1 > autopilot_index;
        typedef ::uavcan::Array< ::dsdl_source::SensorsStatus, ::uavcan::ArrayModeStatic, 1 > status_index;
    };

    enum
    {
        MinBitLen
            = FieldTypes::autopilot_index::MinBitLen
            + FieldTypes::status_index::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::autopilot_index::MaxBitLen
            + FieldTypes::status_index::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::autopilot_index >::Type autopilot_index;
    typename ::uavcan::StorageType< typename FieldTypes::status_index >::Type status_index;

    Autopilot_3_()
        : autopilot_index()
        , status_index()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<2854 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "dsdl_source.Autopilot_3";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Autopilot_3_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        autopilot_index == rhs.autopilot_index &&
        status_index == rhs.status_index;
}

template <int _tmpl>
bool Autopilot_3_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(autopilot_index, rhs.autopilot_index) &&
        ::uavcan::areClose(status_index, rhs.status_index);
}

template <int _tmpl>
int Autopilot_3_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::autopilot_index::encode(self.autopilot_index, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status_index::encode(self.status_index, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Autopilot_3_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::autopilot_index::decode(self.autopilot_index, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::status_index::decode(self.status_index, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Autopilot_3_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x2B76E46BF91A94B3ULL);

    FieldTypes::autopilot_index::extendDataTypeSignature(signature);
    FieldTypes::status_index::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Autopilot_3_<0> Autopilot_3;

// No default registration

} // Namespace dsdl_source

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::dsdl_source::Autopilot_3 >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::dsdl_source::Autopilot_3::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::dsdl_source::Autopilot_3 >::stream(Stream& s, ::dsdl_source::Autopilot_3::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "autopilot_index: ";
    YamlStreamer< ::dsdl_source::Autopilot_3::FieldTypes::autopilot_index >::stream(s, obj.autopilot_index, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "status_index: ";
    YamlStreamer< ::dsdl_source::Autopilot_3::FieldTypes::status_index >::stream(s, obj.status_index, level + 1);
}

}

namespace dsdl_source
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::dsdl_source::Autopilot_3::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::dsdl_source::Autopilot_3 >::stream(s, obj, 0);
    return s;
}

} // Namespace dsdl_source

#endif // DSDL_SOURCE_AUTOPILOT_3_HPP_INCLUDED