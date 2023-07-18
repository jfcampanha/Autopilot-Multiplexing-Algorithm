/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/joao/PESTA/control_system/v3/dsdl_source/Barometer.uavcan
 */

#ifndef DSDL_SOURCE_BAROMETER_HPP_INCLUDED
#define DSDL_SOURCE_BAROMETER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Altitude in meters
#
float16   altitude
#
# Pressure in Pascal
#
float16   pressure
#
# Temperature reading in degC
#
int16 temperature

bool health
******************************************************************************/

/********************* DSDL signature source definition ***********************
dsdl_source.Barometer
saturated float16 altitude
saturated float16 pressure
saturated int16 temperature
saturated bool health
******************************************************************************/

#undef altitude
#undef pressure
#undef temperature
#undef health

namespace dsdl_source
{

template <int _tmpl>
struct UAVCAN_EXPORT Barometer_
{
    typedef const Barometer_<_tmpl>& ParameterType;
    typedef Barometer_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > altitude;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > pressure;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > temperature;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > health;
    };

    enum
    {
        MinBitLen
            = FieldTypes::altitude::MinBitLen
            + FieldTypes::pressure::MinBitLen
            + FieldTypes::temperature::MinBitLen
            + FieldTypes::health::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::altitude::MaxBitLen
            + FieldTypes::pressure::MaxBitLen
            + FieldTypes::temperature::MaxBitLen
            + FieldTypes::health::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::altitude >::Type altitude;
    typename ::uavcan::StorageType< typename FieldTypes::pressure >::Type pressure;
    typename ::uavcan::StorageType< typename FieldTypes::temperature >::Type temperature;
    typename ::uavcan::StorageType< typename FieldTypes::health >::Type health;

    Barometer_()
        : altitude()
        , pressure()
        , temperature()
        , health()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<49 == MaxBitLen>::check();
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
        return "dsdl_source.Barometer";
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
bool Barometer_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        altitude == rhs.altitude &&
        pressure == rhs.pressure &&
        temperature == rhs.temperature &&
        health == rhs.health;
}

template <int _tmpl>
bool Barometer_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(altitude, rhs.altitude) &&
        ::uavcan::areClose(pressure, rhs.pressure) &&
        ::uavcan::areClose(temperature, rhs.temperature) &&
        ::uavcan::areClose(health, rhs.health);
}

template <int _tmpl>
int Barometer_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::altitude::encode(self.altitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pressure::encode(self.pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::temperature::encode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::health::encode(self.health, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Barometer_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::altitude::decode(self.altitude, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pressure::decode(self.pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::temperature::decode(self.temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::health::decode(self.health, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Barometer_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x205D5D24CB882A4AULL);

    FieldTypes::altitude::extendDataTypeSignature(signature);
    FieldTypes::pressure::extendDataTypeSignature(signature);
    FieldTypes::temperature::extendDataTypeSignature(signature);
    FieldTypes::health::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Barometer_<0> Barometer;

// No default registration

} // Namespace dsdl_source

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::dsdl_source::Barometer >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::dsdl_source::Barometer::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::dsdl_source::Barometer >::stream(Stream& s, ::dsdl_source::Barometer::ParameterType obj, const int level)
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
    s << "altitude: ";
    YamlStreamer< ::dsdl_source::Barometer::FieldTypes::altitude >::stream(s, obj.altitude, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pressure: ";
    YamlStreamer< ::dsdl_source::Barometer::FieldTypes::pressure >::stream(s, obj.pressure, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "temperature: ";
    YamlStreamer< ::dsdl_source::Barometer::FieldTypes::temperature >::stream(s, obj.temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "health: ";
    YamlStreamer< ::dsdl_source::Barometer::FieldTypes::health >::stream(s, obj.health, level + 1);
}

}

namespace dsdl_source
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::dsdl_source::Barometer::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::dsdl_source::Barometer >::stream(s, obj, 0);
    return s;
}

} // Namespace dsdl_source

#endif // DSDL_SOURCE_BAROMETER_HPP_INCLUDED