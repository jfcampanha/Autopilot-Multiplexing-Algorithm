/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/joao/PESTA/control_system/v3/dsdl_source/Auto1_control.uavcan
 */

#ifndef DSDL_SOURCE_AUTO1_CONTROL_HPP_INCLUDED
#define DSDL_SOURCE_AUTO1_CONTROL_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#AUTOPILOT STATUS
uint16 WORKING
uint16 ALERT
uint16 FAILURE
uint16 UNTRUSTED

#MODULE STATUS
bool CHECKCONTROL_ACC
bool CHECKCONTROL_AIR
bool CHECKCONTROL_BARO
bool CHECKCONTROL_GYRO
bool CHECKCONTROL_MAG
bool CHECKCONTROL_GNSS

#DATA STATUS
bool UNTRUSTED_ACC
bool UNTRUSTED_AIR
bool UNTRUSTED_BARO
bool UNTRUSTED_GYRO
bool UNTRUSTED_MAG
bool UNTRUSTED_GNSS
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl_source.Auto1_control
saturated uint16 WORKING
saturated uint16 ALERT
saturated uint16 FAILURE
saturated uint16 UNTRUSTED
saturated bool CHECKCONTROL_ACC
saturated bool CHECKCONTROL_AIR
saturated bool CHECKCONTROL_BARO
saturated bool CHECKCONTROL_GYRO
saturated bool CHECKCONTROL_MAG
saturated bool CHECKCONTROL_GNSS
saturated bool UNTRUSTED_ACC
saturated bool UNTRUSTED_AIR
saturated bool UNTRUSTED_BARO
saturated bool UNTRUSTED_GYRO
saturated bool UNTRUSTED_MAG
saturated bool UNTRUSTED_GNSS
******************************************************************************/

#undef WORKING
#undef ALERT
#undef FAILURE
#undef UNTRUSTED
#undef CHECKCONTROL_ACC
#undef CHECKCONTROL_AIR
#undef CHECKCONTROL_BARO
#undef CHECKCONTROL_GYRO
#undef CHECKCONTROL_MAG
#undef CHECKCONTROL_GNSS
#undef UNTRUSTED_ACC
#undef UNTRUSTED_AIR
#undef UNTRUSTED_BARO
#undef UNTRUSTED_GYRO
#undef UNTRUSTED_MAG
#undef UNTRUSTED_GNSS

namespace dsdl_source
{

template <int _tmpl>
struct UAVCAN_EXPORT Auto1_control_
{
    typedef const Auto1_control_<_tmpl>& ParameterType;
    typedef Auto1_control_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > WORKING;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > ALERT;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > FAILURE;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_ACC;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_AIR;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_BARO;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_GYRO;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_MAG;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > CHECKCONTROL_GNSS;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_ACC;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_AIR;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_BARO;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_GYRO;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_MAG;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > UNTRUSTED_GNSS;
    };

    enum
    {
        MinBitLen
            = FieldTypes::WORKING::MinBitLen
            + FieldTypes::ALERT::MinBitLen
            + FieldTypes::FAILURE::MinBitLen
            + FieldTypes::UNTRUSTED::MinBitLen
            + FieldTypes::CHECKCONTROL_ACC::MinBitLen
            + FieldTypes::CHECKCONTROL_AIR::MinBitLen
            + FieldTypes::CHECKCONTROL_BARO::MinBitLen
            + FieldTypes::CHECKCONTROL_GYRO::MinBitLen
            + FieldTypes::CHECKCONTROL_MAG::MinBitLen
            + FieldTypes::CHECKCONTROL_GNSS::MinBitLen
            + FieldTypes::UNTRUSTED_ACC::MinBitLen
            + FieldTypes::UNTRUSTED_AIR::MinBitLen
            + FieldTypes::UNTRUSTED_BARO::MinBitLen
            + FieldTypes::UNTRUSTED_GYRO::MinBitLen
            + FieldTypes::UNTRUSTED_MAG::MinBitLen
            + FieldTypes::UNTRUSTED_GNSS::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::WORKING::MaxBitLen
            + FieldTypes::ALERT::MaxBitLen
            + FieldTypes::FAILURE::MaxBitLen
            + FieldTypes::UNTRUSTED::MaxBitLen
            + FieldTypes::CHECKCONTROL_ACC::MaxBitLen
            + FieldTypes::CHECKCONTROL_AIR::MaxBitLen
            + FieldTypes::CHECKCONTROL_BARO::MaxBitLen
            + FieldTypes::CHECKCONTROL_GYRO::MaxBitLen
            + FieldTypes::CHECKCONTROL_MAG::MaxBitLen
            + FieldTypes::CHECKCONTROL_GNSS::MaxBitLen
            + FieldTypes::UNTRUSTED_ACC::MaxBitLen
            + FieldTypes::UNTRUSTED_AIR::MaxBitLen
            + FieldTypes::UNTRUSTED_BARO::MaxBitLen
            + FieldTypes::UNTRUSTED_GYRO::MaxBitLen
            + FieldTypes::UNTRUSTED_MAG::MaxBitLen
            + FieldTypes::UNTRUSTED_GNSS::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::WORKING >::Type WORKING;
    typename ::uavcan::StorageType< typename FieldTypes::ALERT >::Type ALERT;
    typename ::uavcan::StorageType< typename FieldTypes::FAILURE >::Type FAILURE;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED >::Type UNTRUSTED;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_ACC >::Type CHECKCONTROL_ACC;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_AIR >::Type CHECKCONTROL_AIR;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_BARO >::Type CHECKCONTROL_BARO;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_GYRO >::Type CHECKCONTROL_GYRO;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_MAG >::Type CHECKCONTROL_MAG;
    typename ::uavcan::StorageType< typename FieldTypes::CHECKCONTROL_GNSS >::Type CHECKCONTROL_GNSS;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_ACC >::Type UNTRUSTED_ACC;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_AIR >::Type UNTRUSTED_AIR;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_BARO >::Type UNTRUSTED_BARO;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_GYRO >::Type UNTRUSTED_GYRO;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_MAG >::Type UNTRUSTED_MAG;
    typename ::uavcan::StorageType< typename FieldTypes::UNTRUSTED_GNSS >::Type UNTRUSTED_GNSS;

    Auto1_control_()
        : WORKING()
        , ALERT()
        , FAILURE()
        , UNTRUSTED()
        , CHECKCONTROL_ACC()
        , CHECKCONTROL_AIR()
        , CHECKCONTROL_BARO()
        , CHECKCONTROL_GYRO()
        , CHECKCONTROL_MAG()
        , CHECKCONTROL_GNSS()
        , UNTRUSTED_ACC()
        , UNTRUSTED_AIR()
        , UNTRUSTED_BARO()
        , UNTRUSTED_GYRO()
        , UNTRUSTED_MAG()
        , UNTRUSTED_GNSS()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<76 == MaxBitLen>::check();
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
        return "dsdl_source.Auto1_control";
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
bool Auto1_control_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        WORKING == rhs.WORKING &&
        ALERT == rhs.ALERT &&
        FAILURE == rhs.FAILURE &&
        UNTRUSTED == rhs.UNTRUSTED &&
        CHECKCONTROL_ACC == rhs.CHECKCONTROL_ACC &&
        CHECKCONTROL_AIR == rhs.CHECKCONTROL_AIR &&
        CHECKCONTROL_BARO == rhs.CHECKCONTROL_BARO &&
        CHECKCONTROL_GYRO == rhs.CHECKCONTROL_GYRO &&
        CHECKCONTROL_MAG == rhs.CHECKCONTROL_MAG &&
        CHECKCONTROL_GNSS == rhs.CHECKCONTROL_GNSS &&
        UNTRUSTED_ACC == rhs.UNTRUSTED_ACC &&
        UNTRUSTED_AIR == rhs.UNTRUSTED_AIR &&
        UNTRUSTED_BARO == rhs.UNTRUSTED_BARO &&
        UNTRUSTED_GYRO == rhs.UNTRUSTED_GYRO &&
        UNTRUSTED_MAG == rhs.UNTRUSTED_MAG &&
        UNTRUSTED_GNSS == rhs.UNTRUSTED_GNSS;
}

template <int _tmpl>
bool Auto1_control_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(WORKING, rhs.WORKING) &&
        ::uavcan::areClose(ALERT, rhs.ALERT) &&
        ::uavcan::areClose(FAILURE, rhs.FAILURE) &&
        ::uavcan::areClose(UNTRUSTED, rhs.UNTRUSTED) &&
        ::uavcan::areClose(CHECKCONTROL_ACC, rhs.CHECKCONTROL_ACC) &&
        ::uavcan::areClose(CHECKCONTROL_AIR, rhs.CHECKCONTROL_AIR) &&
        ::uavcan::areClose(CHECKCONTROL_BARO, rhs.CHECKCONTROL_BARO) &&
        ::uavcan::areClose(CHECKCONTROL_GYRO, rhs.CHECKCONTROL_GYRO) &&
        ::uavcan::areClose(CHECKCONTROL_MAG, rhs.CHECKCONTROL_MAG) &&
        ::uavcan::areClose(CHECKCONTROL_GNSS, rhs.CHECKCONTROL_GNSS) &&
        ::uavcan::areClose(UNTRUSTED_ACC, rhs.UNTRUSTED_ACC) &&
        ::uavcan::areClose(UNTRUSTED_AIR, rhs.UNTRUSTED_AIR) &&
        ::uavcan::areClose(UNTRUSTED_BARO, rhs.UNTRUSTED_BARO) &&
        ::uavcan::areClose(UNTRUSTED_GYRO, rhs.UNTRUSTED_GYRO) &&
        ::uavcan::areClose(UNTRUSTED_MAG, rhs.UNTRUSTED_MAG) &&
        ::uavcan::areClose(UNTRUSTED_GNSS, rhs.UNTRUSTED_GNSS);
}

template <int _tmpl>
int Auto1_control_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::WORKING::encode(self.WORKING, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ALERT::encode(self.ALERT, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::FAILURE::encode(self.FAILURE, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED::encode(self.UNTRUSTED, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_ACC::encode(self.CHECKCONTROL_ACC, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_AIR::encode(self.CHECKCONTROL_AIR, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_BARO::encode(self.CHECKCONTROL_BARO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_GYRO::encode(self.CHECKCONTROL_GYRO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_MAG::encode(self.CHECKCONTROL_MAG, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_GNSS::encode(self.CHECKCONTROL_GNSS, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_ACC::encode(self.UNTRUSTED_ACC, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_AIR::encode(self.UNTRUSTED_AIR, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_BARO::encode(self.UNTRUSTED_BARO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_GYRO::encode(self.UNTRUSTED_GYRO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_MAG::encode(self.UNTRUSTED_MAG, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_GNSS::encode(self.UNTRUSTED_GNSS, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Auto1_control_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::WORKING::decode(self.WORKING, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::ALERT::decode(self.ALERT, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::FAILURE::decode(self.FAILURE, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED::decode(self.UNTRUSTED, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_ACC::decode(self.CHECKCONTROL_ACC, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_AIR::decode(self.CHECKCONTROL_AIR, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_BARO::decode(self.CHECKCONTROL_BARO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_GYRO::decode(self.CHECKCONTROL_GYRO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_MAG::decode(self.CHECKCONTROL_MAG, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::CHECKCONTROL_GNSS::decode(self.CHECKCONTROL_GNSS, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_ACC::decode(self.UNTRUSTED_ACC, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_AIR::decode(self.UNTRUSTED_AIR, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_BARO::decode(self.UNTRUSTED_BARO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_GYRO::decode(self.UNTRUSTED_GYRO, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_MAG::decode(self.UNTRUSTED_MAG, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::UNTRUSTED_GNSS::decode(self.UNTRUSTED_GNSS, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Auto1_control_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x15EE89CF71931BD9ULL);

    FieldTypes::WORKING::extendDataTypeSignature(signature);
    FieldTypes::ALERT::extendDataTypeSignature(signature);
    FieldTypes::FAILURE::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_ACC::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_AIR::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_BARO::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_GYRO::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_MAG::extendDataTypeSignature(signature);
    FieldTypes::CHECKCONTROL_GNSS::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_ACC::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_AIR::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_BARO::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_GYRO::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_MAG::extendDataTypeSignature(signature);
    FieldTypes::UNTRUSTED_GNSS::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Auto1_control_<0> Auto1_control;

// No default registration

} // Namespace dsdl_source

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::dsdl_source::Auto1_control >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::dsdl_source::Auto1_control::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::dsdl_source::Auto1_control >::stream(Stream& s, ::dsdl_source::Auto1_control::ParameterType obj, const int level)
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
    s << "WORKING: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::WORKING >::stream(s, obj.WORKING, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "ALERT: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::ALERT >::stream(s, obj.ALERT, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "FAILURE: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::FAILURE >::stream(s, obj.FAILURE, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED >::stream(s, obj.UNTRUSTED, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_ACC: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_ACC >::stream(s, obj.CHECKCONTROL_ACC, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_AIR: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_AIR >::stream(s, obj.CHECKCONTROL_AIR, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_BARO: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_BARO >::stream(s, obj.CHECKCONTROL_BARO, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_GYRO: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_GYRO >::stream(s, obj.CHECKCONTROL_GYRO, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_MAG: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_MAG >::stream(s, obj.CHECKCONTROL_MAG, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "CHECKCONTROL_GNSS: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::CHECKCONTROL_GNSS >::stream(s, obj.CHECKCONTROL_GNSS, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_ACC: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_ACC >::stream(s, obj.UNTRUSTED_ACC, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_AIR: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_AIR >::stream(s, obj.UNTRUSTED_AIR, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_BARO: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_BARO >::stream(s, obj.UNTRUSTED_BARO, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_GYRO: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_GYRO >::stream(s, obj.UNTRUSTED_GYRO, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_MAG: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_MAG >::stream(s, obj.UNTRUSTED_MAG, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "UNTRUSTED_GNSS: ";
    YamlStreamer< ::dsdl_source::Auto1_control::FieldTypes::UNTRUSTED_GNSS >::stream(s, obj.UNTRUSTED_GNSS, level + 1);
}

}

namespace dsdl_source
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::dsdl_source::Auto1_control::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::dsdl_source::Auto1_control >::stream(s, obj, 0);
    return s;
}

} // Namespace dsdl_source

#endif // DSDL_SOURCE_AUTO1_CONTROL_HPP_INCLUDED