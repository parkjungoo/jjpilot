/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/jj/backup_delay/20171229_1/modules/uavcan/dsdl/uavcan/equipment/air_data/1027.RawAirData.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Raw Air Data.
#

# Note: unused vars should be assigned NaN

#
# Heater State
#
uint8 FLAG_HEATER_AVAILABLE      = 1
uint8 FLAG_HEATER_WORKING        = 2
uint8 FLAG_HEATER_OVERCURRENT    = 4
uint8 FLAG_HEATER_OPENCIRCUIT    = 8
uint8 flags

#
# Pressure Data
#
float32 static_pressure                 # Pascal
float32 differential_pressure           # Pascal

#
# Temperature Data
#
float16 static_pressure_sensor_temperature          # Kelvin
float16 differential_pressure_sensor_temperature    # Kelvin

float16 static_air_temperature          # Kelvin
                                        # This field contains the raw temperature reading
                                        # from the externally mounted temperature sensor or,
                                        # in absence of one, the raw temperature of the pressure sensor.

float16 pitot_temperature               # Kelvin

float16[<=16] covariance                # order of diagonal elements :
                                        # static_pressure, differential_pressure,
                                        # static_air_temperature, pitot_temperature
                                        # Pascal^2 for pressure variance and covariance
                                        # Kevin^2 for Temperature variance and covariance
                                        # Pascal*Kelvin for pressure/temperature covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.RawAirData
saturated uint8 flags
saturated float32 static_pressure
saturated float32 differential_pressure
saturated float16 static_pressure_sensor_temperature
saturated float16 differential_pressure_sensor_temperature
saturated float16 static_air_temperature
saturated float16 pitot_temperature
saturated float16[<=16] covariance
******************************************************************************/

#undef flags
#undef static_pressure
#undef differential_pressure
#undef static_pressure_sensor_temperature
#undef differential_pressure_sensor_temperature
#undef static_air_temperature
#undef pitot_temperature
#undef covariance
#undef FLAG_HEATER_AVAILABLE
#undef FLAG_HEATER_WORKING
#undef FLAG_HEATER_OVERCURRENT
#undef FLAG_HEATER_OPENCIRCUIT

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <int _tmpl>
struct UAVCAN_EXPORT RawAirData_
{
    typedef const RawAirData_<_tmpl>& ParameterType;
    typedef RawAirData_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > FLAG_HEATER_AVAILABLE;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > FLAG_HEATER_WORKING;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > FLAG_HEATER_OVERCURRENT;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > FLAG_HEATER_OPENCIRCUIT;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > flags;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > static_pressure;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > differential_pressure;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > static_pressure_sensor_temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > differential_pressure_sensor_temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > static_air_temperature;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > pitot_temperature;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 16 > covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::flags::MinBitLen
            + FieldTypes::static_pressure::MinBitLen
            + FieldTypes::differential_pressure::MinBitLen
            + FieldTypes::static_pressure_sensor_temperature::MinBitLen
            + FieldTypes::differential_pressure_sensor_temperature::MinBitLen
            + FieldTypes::static_air_temperature::MinBitLen
            + FieldTypes::pitot_temperature::MinBitLen
            + FieldTypes::covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::flags::MaxBitLen
            + FieldTypes::static_pressure::MaxBitLen
            + FieldTypes::differential_pressure::MaxBitLen
            + FieldTypes::static_pressure_sensor_temperature::MaxBitLen
            + FieldTypes::differential_pressure_sensor_temperature::MaxBitLen
            + FieldTypes::static_air_temperature::MaxBitLen
            + FieldTypes::pitot_temperature::MaxBitLen
            + FieldTypes::covariance::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::FLAG_HEATER_AVAILABLE >::Type FLAG_HEATER_AVAILABLE; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::FLAG_HEATER_WORKING >::Type FLAG_HEATER_WORKING; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::FLAG_HEATER_OVERCURRENT >::Type FLAG_HEATER_OVERCURRENT; // 4
    static const typename ::uavcan::StorageType< typename ConstantTypes::FLAG_HEATER_OPENCIRCUIT >::Type FLAG_HEATER_OPENCIRCUIT; // 8

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::flags >::Type flags;
    typename ::uavcan::StorageType< typename FieldTypes::static_pressure >::Type static_pressure;
    typename ::uavcan::StorageType< typename FieldTypes::differential_pressure >::Type differential_pressure;
    typename ::uavcan::StorageType< typename FieldTypes::static_pressure_sensor_temperature >::Type static_pressure_sensor_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::differential_pressure_sensor_temperature >::Type differential_pressure_sensor_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::static_air_temperature >::Type static_air_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::pitot_temperature >::Type pitot_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::covariance >::Type covariance;

    RawAirData_()
        : flags()
        , static_pressure()
        , differential_pressure()
        , static_pressure_sensor_temperature()
        , differential_pressure_sensor_temperature()
        , static_air_temperature()
        , pitot_temperature()
        , covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<397 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1027 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.air_data.RawAirData";
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
bool RawAirData_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        flags == rhs.flags &&
        static_pressure == rhs.static_pressure &&
        differential_pressure == rhs.differential_pressure &&
        static_pressure_sensor_temperature == rhs.static_pressure_sensor_temperature &&
        differential_pressure_sensor_temperature == rhs.differential_pressure_sensor_temperature &&
        static_air_temperature == rhs.static_air_temperature &&
        pitot_temperature == rhs.pitot_temperature &&
        covariance == rhs.covariance;
}

template <int _tmpl>
bool RawAirData_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(flags, rhs.flags) &&
        ::uavcan::areClose(static_pressure, rhs.static_pressure) &&
        ::uavcan::areClose(differential_pressure, rhs.differential_pressure) &&
        ::uavcan::areClose(static_pressure_sensor_temperature, rhs.static_pressure_sensor_temperature) &&
        ::uavcan::areClose(differential_pressure_sensor_temperature, rhs.differential_pressure_sensor_temperature) &&
        ::uavcan::areClose(static_air_temperature, rhs.static_air_temperature) &&
        ::uavcan::areClose(pitot_temperature, rhs.pitot_temperature) &&
        ::uavcan::areClose(covariance, rhs.covariance);
}

template <int _tmpl>
int RawAirData_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::flags::encode(self.flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure::encode(self.static_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::differential_pressure::encode(self.differential_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure_sensor_temperature::encode(self.static_pressure_sensor_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::differential_pressure_sensor_temperature::encode(self.differential_pressure_sensor_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_air_temperature::encode(self.static_air_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pitot_temperature::encode(self.pitot_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::encode(self.covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RawAirData_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::flags::decode(self.flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure::decode(self.static_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::differential_pressure::decode(self.differential_pressure, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_pressure_sensor_temperature::decode(self.static_pressure_sensor_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::differential_pressure_sensor_temperature::decode(self.differential_pressure_sensor_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::static_air_temperature::decode(self.static_air_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::pitot_temperature::decode(self.pitot_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::decode(self.covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RawAirData_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xC77DF38BA122F5DAULL);

    FieldTypes::flags::extendDataTypeSignature(signature);
    FieldTypes::static_pressure::extendDataTypeSignature(signature);
    FieldTypes::differential_pressure::extendDataTypeSignature(signature);
    FieldTypes::static_pressure_sensor_temperature::extendDataTypeSignature(signature);
    FieldTypes::differential_pressure_sensor_temperature::extendDataTypeSignature(signature);
    FieldTypes::static_air_temperature::extendDataTypeSignature(signature);
    FieldTypes::pitot_temperature::extendDataTypeSignature(signature);
    FieldTypes::covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename RawAirData_<_tmpl>::ConstantTypes::FLAG_HEATER_AVAILABLE >::Type
    RawAirData_<_tmpl>::FLAG_HEATER_AVAILABLE = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename RawAirData_<_tmpl>::ConstantTypes::FLAG_HEATER_WORKING >::Type
    RawAirData_<_tmpl>::FLAG_HEATER_WORKING = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename RawAirData_<_tmpl>::ConstantTypes::FLAG_HEATER_OVERCURRENT >::Type
    RawAirData_<_tmpl>::FLAG_HEATER_OVERCURRENT = 4U; // 4

template <int _tmpl>
const typename ::uavcan::StorageType< typename RawAirData_<_tmpl>::ConstantTypes::FLAG_HEATER_OPENCIRCUIT >::Type
    RawAirData_<_tmpl>::FLAG_HEATER_OPENCIRCUIT = 8U; // 8

/*
 * Final typedef
 */
typedef RawAirData_<0> RawAirData;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::air_data::RawAirData > _uavcan_gdtr_registrator_RawAirData;

}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::air_data::RawAirData >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::air_data::RawAirData::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::air_data::RawAirData >::stream(Stream& s, ::uavcan::equipment::air_data::RawAirData::ParameterType obj, const int level)
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
    s << "flags: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::flags >::stream(s, obj.flags, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "static_pressure: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::static_pressure >::stream(s, obj.static_pressure, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "differential_pressure: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::differential_pressure >::stream(s, obj.differential_pressure, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "static_pressure_sensor_temperature: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::static_pressure_sensor_temperature >::stream(s, obj.static_pressure_sensor_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "differential_pressure_sensor_temperature: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::differential_pressure_sensor_temperature >::stream(s, obj.differential_pressure_sensor_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "static_air_temperature: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::static_air_temperature >::stream(s, obj.static_air_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "pitot_temperature: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::pitot_temperature >::stream(s, obj.pitot_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "covariance: ";
    YamlStreamer< ::uavcan::equipment::air_data::RawAirData::FieldTypes::covariance >::stream(s, obj.covariance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::air_data::RawAirData::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::air_data::RawAirData >::stream(s, obj, 0);
    return s;
}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_HPP_INCLUDED