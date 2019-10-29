//! EEPROM configuration manager for the Earth Rover (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER__VCU_CONFIGURATION_MANAGER__
#define __EARTH_ROVER__VCU_CONFIGURATION_MANAGER__


#include <cstdint>
#include <EEPROM.h>
#include "from-to-integral.hpp"


namespace earth_rover_vcu
{

  //! EEPROM configuration manager for the Earth Rover.
  /*!
   *  The configuration manager is responsible to store all configuration and calibration settings in the EEPROM and
   *  implements some kind of wear leveling algorithm to enhance the life of the Teensy's EEPROM. The actual
   *  serialization of configuration and calibration settings is done by the subsystems.
   * 
   *  \tparam Steering_t Actual type of the steering device driver.
   *  \tparam Powertrain_t Actual type of the powertrain device driver.
   *  \tparam PositionEncoder_t Actual type of the position encoder device driver.
   *  \tparam Imu_t Actual type of the IMU device driver.
   *  \tparam Radio_t Actual type of the radio configuration.
   * 
   *  \ingroup VCU
   */
  template <typename Steering_t, typename Powertrain_t, typename PositionEncoder_t, typename Imu_t, typename Radio_t>
  class VcuConfigurationManager
  {
    private:

      static constexpr uint16_t record_size {32u};                    //!< Record size, including metadata.
      static constexpr uint16_t record_data_size {record_size - 8u};  //!< Record size, excluding metadata.

      //! Configuration record types.
      enum class RecordType: uint8_t
      {
        Odometer,                    //!< Odometer value.
        PositionEncoderCalibration,  //!< Calibration of the position encoder.
        ImuCalibration,              //!< Calibration of the IMU.
        SteeringServoConfiguration,  //!< Configuration of the steering servo.
        PowertrainConfiguration,     //!< Configuration of the powertrain (ESC and gearbox servo).
        RadioConfiguration           //!< Configuration of the radios.
      };
      static constexpr uint8_t record_type_count {6u};  //!< Number of record types (for array sizes).

      uint32_t eeprom_offset;  //!< Location of the configuration area in the EEPROM memory.
      uint32_t eeprom_size;    //!< Size of the configuration area in the EEPROM memory.

      uint16_t used_record_indices[record_type_count];  //!< Used (active) configuration records.
      uint16_t newest_record_index;                     //!< Most recent record index used.
      uint32_t newest_record_sequence;                  //!< Most recent sequence number used.

      Steering_t & steering;                 //!< Reference to the steering servo.
      Powertrain_t & powertrain;             //!< Reference to the powertrain.
      PositionEncoder_t & position_encoder;  //!< Reference to the position encoder.
      Imu_t & imu;                           //!< Reference to the IMU.
      Radio_t & radio;                       //!< Reference to the radio configuration.

      //! Calculate the Fletcher-16 checksum.
      /*!
       *  \param buffer Buffer (of size record_size), last two bytes are used for the Fletcher-16 checksum.
       *  \return The Fletcher-16 checksum.
       */
      uint16_t fletcher16(uint8_t * buffer)
      {
        constexpr uint32_t initial_value {0u};
        static_assert(initial_value + UINT8_MAX * ((record_size - 2) + 1) * (record_size - 2) / 2 <= UINT32_MAX,
                      "record too big for fletcher16 checksum algorithm");

        uint32_t sum1 {initial_value};
        uint32_t sum2 {initial_value};

        for(uint32_t index = 0; index < record_size - sizeof(uint16_t); ++ index)
        {
          sum1 += buffer[index];
          sum2 += sum1;
        }

        return ((sum2 % 0xff) << 8) | (sum1 % 0xff);
      }

      //! Find the index (location) of the next available record.
      /*!
       *  \return The index (location) of the next available record.
       */
      uint16_t findNextAvailableRecordIndex()
      {
        uint16_t record_count = eeprom_size / record_size;
        do
        {
          newest_record_index = (newest_record_index + 1) % record_count;
        } while(!isRecordIndexAvailable(newest_record_index));
        return newest_record_index;
      }

      //! Check whether an index (location) is available.
      /*!
       *  \param record_index Record index (location) to check.
       *  \return True if the index(location) is available, false if it is used by a configuration record.
       */
      bool isRecordIndexAvailable(uint16_t record_index)
      {
        for(uint8_t type_index = 0; type_index < record_type_count; ++ type_index)
        {
          if(used_record_indices[type_index] == record_index)
          {
            return false;
          }
        }
        return true;
      }

      //! Check whether an object's configuration or calibration needs to be saved and save it if necessary.
      /*!
       *  \tparam Object_t Object type.
       *  \param object Object instance.
       *  \param type Record type.
       *  \param has_checksum True if the value has a checksum, false if it doesn't want a checksum
       *                      (e.g. odometer to lower the number of EEPROM writes).
       *  \return True if the object's configuration is saved to EEPROM.
       */
      template <typename Object_t>
      bool updateStoredConfiguration(Object_t & object, RecordType type, bool has_checksum = true)
      {
        uint8_t record[record_size];
        if(object.saveRequired())
        {
          bool record_exists = false;
          if(used_record_indices[to_integral(type)] != 0xffff)
          {
            // DEBUG
            // Serial.println("Reading current record");
            EEPROM.get(eeprom_offset + used_record_indices[to_integral(type)] * record_size, record);
            record_exists = true;
          }
          else
          {
            // DEBUG
            // Serial.println("No current record registered");
            record_exists = false;
          }
          bool write_new_record = object.serialize(&record[6], record_data_size);
          if(!record_exists || write_new_record)
          {
            // DEBUG
            // Serial.println("New record");
            record[0] = to_integral(type);
            record[1] = has_checksum? 0xff: 0x00;
            uint32_t sequence = ++ newest_record_sequence;
            record[2] = sequence & 0x000000ff;
            record[3] = (sequence & 0x0000ff00) >> 8;
            record[4] = (sequence & 0x00ff0000) >> 16;
            record[5] = (sequence & 0xff000000) >> 24;
            used_record_indices[to_integral(type)] = findNextAvailableRecordIndex();
          }
          if(has_checksum)
          {
            uint16_t checksum = fletcher16(record);
            record[30] = checksum & 0x00ff;
            record[31] = (checksum & 0xff00) >> 8;
          }
          else
          {
            record[30] = 0xff;
            record[31] = 0xff;
          }
          EEPROM.put(eeprom_offset + used_record_indices[to_integral(type)] * record_size, record);
          // BEGIN DEBUG
          /*
          Serial.printf("Wrote record %u: ", used_record_indices[to_integral(type)]);
          for(unsigned i = 0; i < record_size; ++ i)
          {
            Serial.printf("%02x ", record[i]);
          }
          Serial.println("");
          */
          // END DEBUG
          return true;
        }
        else
        {
          return false;
        }
      }

    public:

      //! Constructor.
      /*!
       *  \param steering Steering servo (for steering servo configuration).
       *  \param powertrain Powertrain (for ESC and gearbox servo configuration).
       *  \param position_encoder Position encoder (for odometer value).
       *  \param imu IMU (for calibration).
       *  \param radio Radio configuration.
       *  \param eeprom_offset Location of the configuration area in the EEPROM memory.
       *  \param eeprom_size Size of the configuration area in the EEPROM memory.
       */
      VcuConfigurationManager(
        Steering_t & steering, Powertrain_t & powertrain,
        PositionEncoder_t & position_encoder, Imu_t & imu, Radio_t & radio,
        uint32_t eeprom_offset = 0u, uint32_t eeprom_size = 2048u)
      :
        eeprom_offset {eeprom_offset},
        eeprom_size {eeprom_size},
        newest_record_index {UINT16_MAX},
        newest_record_sequence {0u},
        steering {steering},
        powertrain {powertrain},
        position_encoder {position_encoder},
        imu {imu},
        radio {radio}
      {
        for(uint8_t index = 0; index < record_type_count; ++ index)
        {
          used_record_indices[index] = 0xffff;
        }
      }

      //! Default destructor.
      ~VcuConfigurationManager() = default;

      //! Read the configuration from memory and configure all subsystems.
      void setup()
      {
        uint16_t record_count = eeprom_size / record_size;
        uint8_t record[record_size];
        uint32_t used_record_sequences[record_type_count] {0};
        for(uint16_t record_index = 0; record_index < record_count; ++ record_index)
        {
          // Read record from EEPROM.
          EEPROM.get(eeprom_offset + record_index * record_size, record);
          // Parse record structure.
          RecordType record_type = from_integral<RecordType>(record[0]);
          bool record_has_checksum = record[1] == 0? false: true;
          uint32_t record_sequence = record[2] | (record[3] << 8) | (record[4] << 16) | (record[5] << 24);
          uint8_t * record_data = &record[6];
          uint16_t record_checksum = record[30] | (record[31] << 8);
          // Parse record.
          if((record_sequence != 0xffffffff && record_sequence != 0x00000000)
            && (record_has_checksum == false || fletcher16((uint8_t *)(&record)) == record_checksum))
          {
            // DEBUG
            // Serial.printf("Parsing valid configuration record %d\n", record_index);
            // If the record number is larger than the previous largest record number, this is a more recent message.
            if(record_sequence > newest_record_sequence)
            {
              newest_record_index = record_index;
              newest_record_sequence = record_sequence;
            }
            // Parse record, store record if valid and more recent than previous cached record.
            if(record_sequence > used_record_sequences[to_integral(record_type)])
            {
              bool success {false};
              switch(record_type)
              {
                case RecordType::Odometer:
                {
                  success = position_encoder.deserialize(record_data, record_data_size);
                } break;
                case RecordType::PositionEncoderCalibration:
                {
                  // Currently unused.
                } break;
                case RecordType::ImuCalibration:
                {
                  success = imu.deserialize(record_data, record_data_size);
                } break;
                case RecordType::SteeringServoConfiguration:
                {
                  success = steering.deserialize(record_data, record_data_size);
                } break;
                case RecordType::PowertrainConfiguration:
                {
                  success = powertrain.deserialize(record_data, record_data_size);
                } break;
                case RecordType::RadioConfiguration:
                {
                  success = radio.deserialize(record_data, record_data_size);
                } break;
              }
              if(success)
              {
                // BEGIN DEBUG
                /*
                Serial.printf("Successcully deserialized record %u: ", record_index);
                for(unsigned i = 0; i < record_size; ++ i)
                {
                  Serial.printf("%02x ", record[i]);
                }
                Serial.println("");
                */
                // END DEBUG
                used_record_indices[to_integral(record_type)] = record_index;
                used_record_sequences[to_integral(record_type)] = record_sequence;
              }
            }
          }
          else
          {
            // DEBUG
            // Serial.printf("Skipping empty or invalid configuration record %d\n", record_index);
            ;
          }
        }
      }

      //! Spinning loop.
      /*!
       *  The spinning loop makes sure all settings get saved when necessary. To prevent blocking the main loop too
       *  long, maximum one configuration setting or calibration is saved during each call.
       */
      void spinOnce()
      {
        if(updateStoredConfiguration(position_encoder, RecordType::Odometer, false))
        {
          ;
        }
        else if(updateStoredConfiguration(imu, RecordType::ImuCalibration, true))
        {
          ;
        }
        else if(updateStoredConfiguration(steering, RecordType::SteeringServoConfiguration, true))
        {
          Serial.println("Stored steering configuration");  // DEBUG
        }
        else if(updateStoredConfiguration(powertrain, RecordType::PowertrainConfiguration, true))
        {
          Serial.println("Stored powertrain configuration");  // DEBUG
        }
        else if(updateStoredConfiguration(radio, RecordType::RadioConfiguration, true))
        {
          Serial.println("Stored radio configuration");  // DEBUG
        }
      }

  };


  //! Factory function to create a VcuConfigurationManager object.
  /*!
   *  \tparam Steering_t Steering device driver type.
   *  \tparam Powertrain_t Powertrain device driver type.
   *  \tparam PositionEncoder_t Position encoder device driver type.
   *  \tparam Imu_t IMU device driver type.
   *  \tparam Radio_t Radio configuration type.
   *  \param steering Steering device driver.
   *  \param powertrain Powertrain device driver.
   *  \param position_encoder Position encoder device driver.
   *  \param imu IMU device driver.
   *  \param radio Radio configuration.
   *  \param eeprom_offset Offset of the configuration area in the EEPROM memory.
   *  \param eeprom_size Size of the configuration area in the EEPROM memory.
   *  \return VCU configuration manager with the given device drivers.
   * 
   *  \ingroup VCU
   */
  template<typename Steering_t, typename Powertrain_t, typename PositionEncoder_t, typename Imu_t, typename Radio_t>
  auto makeVcuConfigurationManager(
    Steering_t & steering, Powertrain_t powertrain,
    PositionEncoder_t & position_encoder, Imu_t & imu, Radio_t & radio,
    uint32_t eeprom_offset = 0u, uint32_t eeprom_size = 2048u)
  {
    return VcuConfigurationManager<Steering_t, Powertrain_t, PositionEncoder_t, Imu_t, Radio_t>(
      steering, powertrain, position_encoder, imu, radio, eeprom_offset, eeprom_size);
  }

}

#endif