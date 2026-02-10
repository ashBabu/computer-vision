#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <iomanip>


using json = nlohmann::json;

json read_json(const std::string& input_json) {
  if (input_json.empty()) {
    throw std::invalid_argument("Input JSON string is empty");
  }
  std::ifstream input_file(input_json);
  if (!input_file.is_open()) {
    throw std::runtime_error("Could not open JSON file: " + input_json);
  }

  json data;
  input_file >> data;
  return data;
}

void write_json(const std::string& output_json, const json& j) {
  if (output_json.empty()) {
    throw std::invalid_argument("Output JSON string is empty");
  }
  std::ofstream output_file(output_json);
  if (!output_file.is_open()) {
    throw std::runtime_error("Could not open JSON file for writing: " + output_json);
  }
  output_file << std::setw(4) << j << std::endl;
}