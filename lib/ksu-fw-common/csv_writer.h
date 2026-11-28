#ifndef CSV_WRITER_H
#define CSV_WRITER_H
#include <map>
#include <string>
#include <sstream>
#include <functional>
#include <Arduino.h>

enum class Type {
    INT,
    UINT,
    LONG,
    ULONG,
    LLONG,
    ULLONG,
    FLOAT,
    DOUBLE,
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    INT16,
    INT32
};

class smartCsv {
private:
    std::map<std::string, void*> values;
    std::map<std::string, Type> types;
    // std::map<std::string, float> scales;

public:
    smartCsv(const std::initializer_list<std::tuple<std::string, void*, Type>>& entries) {
        for (const auto& entry : entries) {
            values[std::get<0>(entry)] = std::get<1>(entry);
            types[std::get<0>(entry)] = std::get<2>(entry);
            // scales[std::get<0>(entry)] = std::get<3>(entry);
        }
    }

    String getKeysAsCSVRow() {
        String result;
        for (const auto& pair : values) {
            result += pair.first.c_str();
            result += ",";
        }
        result.remove(result.length() - 1); // Remove last comma
        result += "\n";
        return result;
    }

    String getValuesAsCSVRow() {
        String result;
        for (const auto& pair : values) {
            switch (types[pair.first]) {
                case Type::INT:
                    result += *reinterpret_cast<int*>(pair.second);
                    break;
                case Type::UINT:
                    result += *reinterpret_cast<unsigned int*>(pair.second);
                    break;
                case Type::LONG:
                    result += *reinterpret_cast<long*>(pair.second);
                    break;
                case Type::ULONG:
                    result += *reinterpret_cast<unsigned long*>(pair.second);
                    break;
                case Type::LLONG:
                    result += *reinterpret_cast<long long*>(pair.second);
                    break;
                case Type::ULLONG:
                    result += *reinterpret_cast<unsigned long long*>(pair.second);
                    break;
                case Type::FLOAT:
                    result += *reinterpret_cast<float*>(pair.second);
                    break;
                case Type::DOUBLE:
                    result += *reinterpret_cast<double*>(pair.second);
                    break;
                case Type::UINT64:
                    result += *reinterpret_cast<uint64_t*>(pair.second);
                    break;
                case Type::INT16:
                    result += *reinterpret_cast<int16_t*>(pair.second);
                    break;
                case Type::UINT8:
                    result += *reinterpret_cast<uint8_t*>(pair.second);
                    break;
                case Type::UINT16:
                    result += *reinterpret_cast<uint16_t*>(pair.second);
                    break;
                case Type::UINT32:
                    result += *reinterpret_cast<uint32_t*>(pair.second);
                    break;
                case Type::INT32:
                    result += *reinterpret_cast<int32_t*>(pair.second);
                    break;
            }
            result += ",";
        }
        result.remove(result.length() - 1); // Remove last comma
        result += "\n";
        return result;
    }
    String getValuesAndKeysAsCSVRow() {
        String result;
        for (const auto& pair : values) {
                result += pair.first.c_str();
                result += ": ";
            switch (types[pair.first]) {
                case Type::INT:
                    result += *reinterpret_cast<int*>(pair.second);
                    break;
                case Type::UINT:
                    result += *reinterpret_cast<unsigned int*>(pair.second);
                    break;
                case Type::LONG:
                    result += *reinterpret_cast<long*>(pair.second);
                    break;
                case Type::ULONG:
                    result += *reinterpret_cast<unsigned long*>(pair.second);
                    break;
                case Type::LLONG:
                    result += *reinterpret_cast<long long*>(pair.second);
                    break;
                case Type::ULLONG:
                    result += *reinterpret_cast<unsigned long long*>(pair.second);
                    break;
                case Type::FLOAT:
                    result += *reinterpret_cast<float*>(pair.second);
                    break;
                case Type::DOUBLE:
                    result += *reinterpret_cast<double*>(pair.second);
                    break;
                case Type::UINT64:
                    result += *reinterpret_cast<uint64_t*>(pair.second);
                    break;
                case Type::INT16:
                    result += *reinterpret_cast<int16_t*>(pair.second);
                    break;
                case Type::UINT8:
                    result += *reinterpret_cast<uint8_t*>(pair.second);
                    break;
                case Type::UINT16:
                    result += *reinterpret_cast<uint16_t*>(pair.second);
                    break;
                case Type::UINT32:
                    result += *reinterpret_cast<uint32_t*>(pair.second);
                    break;
                case Type::INT32:
                    result += *reinterpret_cast<int32_t*>(pair.second);
                    break;
            }
            result += "\n";
        }
        result.remove(result.length() - 1); // Remove last comma
        result += "\n";
        return result;
    }

};

#endif