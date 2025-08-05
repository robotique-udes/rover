#ifndef ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
#define ROVER_LIB2_SENSORS_GNSS_PARSER_HPP

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <array>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <charconv>
#include <system_error>

DEFINE_LOG_NODE(GNSS_PARSER, Logger::eNodeState::ON);

/**
 * @brief
 * To add a new GNSS message, add its data structure and its message parser
 * function
 *
 */
namespace GNSSParser
{
    constexpr size_t MAX_FIELDS = 16UL;
    constexpr size_t MAX_FIELD_LENGTH = 16UL;

    enum class eGGAFields : size_t
    {
        MESSAGE_ID = 0,       // GGA protocol header
        UTC_TIME,             // UTC time (hhmmss.sss)
        LATITUDE,             // Latitude (ddmm.mmmm)
        NS_INDICATOR,         // N/S Indicator ('N' or 'S')
        LONGITUDE,            // Longitude (dddmm.mmmm)
        EW_INDICATOR,         // E/W Indicator ('E' or 'W')
        FIX_QUALITY,          // Position Fix Indicator
        SATELLITES_USED,      // Number of satellites used
        HDOP,                 // Horizontal Dilution of Precision
        MSL_ALTITUDE,         // Mean Sea Level Altitude
        ALTITUDE_UNITS,       // Altitude units ('M' for meters)
        GEOID_SEPERATION,     // Geoid separation
        GEOID_UNITS,          // Geoid separation units ('M' for meters)
        AGE_OF_DIFF_CORR,     // Age of differential corrections (seconds)
        DIFF_RED_STATION_ID,  // Differential reference station ID
        CHECKSUM,             // Checksum
        eLAST,                // Number of GGA fields
    };

    enum class eUniHeadingFields : size_t
    {
        MYSTERE0 = 0,
        MYSTERE1,
        MYSTERE2,
        MYSTERE3,
        MYSTERE4,
        MYSTERE5,
        MYSTERE6,
        MYSTERE7,
        MYSTERE8,
        MYSTERE9,
        HEADING_QUALITY,
        MYSTERE11,
        HEADING,
        MYSTERE13,  // There is 26 fields in total, but all the next are unknown
        eLAST,
    };

    struct sUTCTime
    {
        uint8_t hours = 0U;
        uint8_t minutes = 0U;
        float seconds = 0.0F;
    };

    struct sGGADataUsed
    {
        float latitude = 0.0F;
        float longitude = 0.0F;
        Constants::eGGAQuality fixQuality = Constants::eGGAQuality::NO_FIX;
        uint8_t satellitesUsed = 0U;
    };

    struct sUniHeadingDataUsed
    {
        float headingDeg = 0.0F;
        Constants::eHeadingQuality headingQuality = Constants::eHeadingQuality::NO_HEADING;
    };

    template<size_t SENTENCE_LENGTH>
    bool parseGGA(std::array<char, SENTENCE_LENGTH>& rawSentence_, sGGADataUsed& out_, Constants::eHeadingQuality uhQuality_);
    template<size_t SENTENCE_LENGTH>
    bool parseUniHeading(std::array<char, SENTENCE_LENGTH>& rawSentence_, sUniHeadingDataUsed& out_);

    // ======================================================================================================================
    // Functions definitions
    // ======================================================================================================================

    /**
     * @brief Find all the comma indices from a sentence
     *
     */
    template<size_t SENTENCE_LENGTH, size_t FIELD_COUNT>
    size_t commaSegmenter(std::array<char, SENTENCE_LENGTH>& sentence_, std::array<size_t, FIELD_COUNT>& commaIndices_)
    {
        size_t count = 0UL;
        for (size_t i = 0UL; i < sentence_.size() && sentence_[i] != '\0'; ++i)
        {
            if (sentence_[i] == ',')
            {
                if (count < commaIndices_.size())
                {
                    commaIndices_[count++] = i;
                }
            }
        }

        // Add end of sentence as final index
        if (count < commaIndices_.size())
        {
            commaIndices_[count++] = sentence_.size();
        }

        return count;
    }

    template<size_t N>
    inline std::optional<float> convertToDecimalDegrees(const std::array<char, N>& nmeaCoord_, char direction_)
    {
        if (nmeaCoord_[0] == '\0' || std::strlen(nmeaCoord_.data()) < 6U)
        {
            LOG_WARN(Logger::Nodes::GNSS_PARSER, "Invalid NMEA Coord: %s", nmeaCoord_.data());
            return std::nullopt;
        }

        if (direction_ != 'N' && direction_ != 'S' && direction_ != 'E' && direction_ != 'W')
        {
            LOG_WARN(Logger::Nodes::GNSS_PARSER, "Invalid latitude/longitude direction: %c", direction_);
            return std::nullopt;
        }

        float degMin = 0.0F;
        auto result = std::from_chars(nmeaCoord_.data(), nmeaCoord_.data() + std::strlen(nmeaCoord_.data()), degMin);
        if (result.ec != std::errc())
        {
            LOG_WARN(Logger::Nodes::GNSS_PARSER, "Failed to parse NMEA Coord: %s", nmeaCoord_.data());
            return std::nullopt;
        }
        int degrees = static_cast<int>(degMin / 100.0F);
        float minutes = degMin - degrees * 100.0F;
        float decimal = degrees + minutes / 60.0F;

        if (direction_ == 'S' || direction_ == 'W')
        {
            decimal *= -1.0F;
        }

        return decimal;
    }

    template<size_t SENTENCE_LENGTH>
    bool getField(const std::array<char, SENTENCE_LENGTH>& sentence_,
                  const std::array<size_t, MAX_FIELDS>& commaIndices_,
                  std::array<char, MAX_FIELD_LENGTH>& field_,
                  size_t index_)
    {
        if (index_ >= commaIndices_.size())
        {
            return false;
        }

        size_t start;
        if (index_ == 0)
        {
            start = 0;
        }
        else
        {
            start = commaIndices_[index_ - 1] + 1;
        }
        size_t end = commaIndices_[index_];

        if (start >= end || end > sentence_.size())
        {
            return false;
        }

        size_t length = std::min(end - start, field_.size() - 1);
        std::copy_n(&sentence_[start], length, field_.data());
        field_[length] = '\0';

        return true;
    }

    /**
     * @brief
     *
     * @attention Assume rawSentence is a GGA message
     */
    template<size_t SENTENCE_LENGTH>
    bool parseGGA(std::array<char, SENTENCE_LENGTH>& rawSentence_, sGGADataUsed& out_, Constants::eHeadingQuality headingQuality_)
    {
        std::array<size_t, std::to_underlying(eGGAFields::eLAST)> commaIndices = {};
        size_t count = commaSegmenter(rawSentence_, commaIndices);

        if (count >= static_cast<size_t>(eGGAFields::SATELLITES_USED))
        {
            std::array<char, MAX_FIELD_LENGTH> field{};
            std::array<char, MAX_FIELD_LENGTH> secondField{};
            bool problem = false;

            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eGGAFields::FIX_QUALITY)))
            {
                uint8_t tempQuality = 0U;
                auto result = std::from_chars(field.data(), field.data() + std::strlen(field.data()), tempQuality);
                if (result.ec != std::errc())
                {
                    LOG_WARN(Logger::Nodes::GNSS_PARSER, "Failed to parse the heading");
                    tempQuality = 0U;
                }

                // Fix quality compared with uniheading for GPS vs GNSS
                if (tempQuality == 4U)
                {
                    out_.fixQuality = Constants::eGGAQuality::RTK;
                }
                else if (tempQuality == 1U && headingQuality_ != Constants::eHeadingQuality::NO_HEADING)
                {
                    out_.fixQuality = Constants::eGGAQuality::GNSS;
                }
                else if (tempQuality == 1U)
                {
                    out_.fixQuality = Constants::eGGAQuality::GPS;
                }
                else
                {
                    out_.fixQuality = Constants::eGGAQuality::NO_FIX;
                    problem = true;
                }
            }

            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eGGAFields::LATITUDE))
                && getField(rawSentence_, commaIndices, secondField, static_cast<size_t>(eGGAFields::NS_INDICATOR)) && !problem)
            {
                auto latitude = convertToDecimalDegrees(field, secondField[0]);
                if (latitude)
                {
                    out_.latitude = *latitude;
                }
                else
                {
                    problem = true;
                }
            }

            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eGGAFields::LONGITUDE))
                && getField(rawSentence_, commaIndices, secondField, static_cast<size_t>(eGGAFields::EW_INDICATOR)) && !problem)
            {
                auto longitude = convertToDecimalDegrees(field, secondField[0]);
                if (longitude)
                {
                    out_.longitude = *longitude;
                }
                else
                {
                    problem = true;
                }
            }

            if (problem)
            {
                out_.latitude = 0.0F;
                out_.longitude = 0.0F;
                out_.fixQuality = Constants::eGGAQuality::NO_FIX;
            }

            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eGGAFields::SATELLITES_USED)))
            {
                uint8_t tempSatellites = 0U;
                auto result = std::from_chars(field.data(), field.data() + std::strlen(field.data()), tempSatellites);
                if (result.ec != std::errc())
                {
                    LOG_WARN(Logger::Nodes::GNSS_PARSER, "Failed to parse the Satellites Used");
                    tempSatellites = 0U;
                }

                out_.satellitesUsed = tempSatellites;
            }
            return true;
        }
        return false;
    }

    /**
     * @brief
     *
     * @attention Assume rawSentence is a UniHeading message
     */
    template<size_t SENTENCE_LENGTH>
    bool parseUniHeading(std::array<char, SENTENCE_LENGTH>& rawSentence_, sUniHeadingDataUsed& out_)
    {
        std::array<size_t, MAX_FIELDS> commaIndices{};
        size_t count = commaSegmenter(rawSentence_, commaIndices);

        if (count >= static_cast<size_t>(eUniHeadingFields::HEADING))
        {
            std::array<char, MAX_FIELD_LENGTH> field{};
            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eUniHeadingFields::HEADING_QUALITY)))
            {
                if (std::strncmp(field.data(), "L1_FLOAT", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eHeadingQuality::UNRELIABLE;
                }
                else if (std::strncmp(field.data(), "L1_INT", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eHeadingQuality::RELIABLE;
                }
                else if (std::strncmp(field.data(), "L1_FIXED", MAX_FIELD_LENGTH) == 0)
                {
                    out_.headingQuality = Constants::eHeadingQuality::BEST;
                }
                else
                {
                    out_.headingQuality = Constants::eHeadingQuality::NO_HEADING;
                    out_.headingDeg = 0.0F;
                }
            }

            if (getField(rawSentence_, commaIndices, field, static_cast<size_t>(eUniHeadingFields::HEADING))
                && out_.headingQuality != Constants::eHeadingQuality::NO_HEADING)
            {
                float heading = 0.0f;
                auto result = std::from_chars(field.data(), field.data() + std::strlen(field.data()), heading);
                if (result.ec == std::errc())
                {
                    out_.headingDeg = heading;
                }
                else
                {
                    out_.headingDeg = 0.0F;
                    out_.headingQuality = Constants::eHeadingQuality::NO_HEADING;
                    LOG_WARN(Logger::Nodes::GNSS_PARSER, "Failed to parse the heading");
                }
            }
            return true;
        }
        return false;
    }

};  // namespace GNSSParser

#endif  // ROVER_LIB2_SENSORS_GNSS_PARSER_HPP
