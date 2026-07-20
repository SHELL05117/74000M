#include "robot/telemetry/recording_codec.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

class FileRecordingByteReader final : public robot::RecordingByteReader {
 public:
  explicit FileRecordingByteReader(const std::string& path)
      : file_(path, std::ios::binary) {
    if (!file_) return;
    file_.seekg(0, std::ios::end);
    const std::streamoff end = file_.tellg();
    if (end < 0 ||
        static_cast<std::uint64_t>(end) >
            std::numeric_limits<std::size_t>::max()) {
      file_.close();
      return;
    }
    size_ = static_cast<std::size_t>(end);
    file_.seekg(0, std::ios::beg);
    valid_ = static_cast<bool>(file_);
  }

  bool valid() const noexcept { return valid_; }

  std::size_t sizeBytes() const noexcept override { return size_; }

  bool readBytes(std::size_t offset, void* destination,
                 std::size_t size) noexcept override {
    if (!valid_ || destination == nullptr || offset > size_ ||
        size > size_ - offset ||
        offset >
            static_cast<std::size_t>(
                std::numeric_limits<std::streamoff>::max()) ||
        size >
            static_cast<std::size_t>(
                std::numeric_limits<std::streamsize>::max()))
      return false;
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
    file_.read(static_cast<char*>(destination),
               static_cast<std::streamsize>(size));
    return file_.good() ||
           static_cast<std::size_t>(file_.gcount()) == size;
  }

 private:
  std::ifstream file_;
  std::size_t size_{};
  bool valid_{};
};

struct FaultName {
  std::uint32_t bit;
  const char* name;
};

constexpr FaultName kFaultNames[] = {
    {robot::kRecordingTruncated, "TRUNCATED"},
    {robot::kRecordingBadHeader, "BAD_HEADER"},
    {robot::kRecordingUnsupportedFormat, "UNSUPPORTED_FORMAT"},
    {robot::kRecordingBadBlockHeader, "BAD_BLOCK_HEADER"},
    {robot::kRecordingBadPayload, "BAD_PAYLOAD"},
    {robot::kRecordingSequenceGap, "SEQUENCE_GAP"},
    {robot::kRecordingTimeRegression, "TIME_REGRESSION"},
    {robot::kRecordingRunMismatch, "RUN_MISMATCH"},
    {robot::kRecordingBadFooter, "BAD_FOOTER"},
    {robot::kRecordingFooterMismatch, "FOOTER_MISMATCH"},
    {robot::kRecordingTrailingData, "TRAILING_DATA"},
    {robot::kRecordingNoFrames, "NO_FRAMES"},
};

std::string fixedText(const char* text, std::size_t capacity) {
  std::size_t length = 0;
  while (length < capacity && text[length] != '\0') ++length;
  return std::string(text, length);
}

std::string jsonEscape(const std::string& value) {
  std::string result;
  result.reserve(value.size() + 8);
  for (const unsigned char c : value) {
    switch (c) {
      case '"': result += "\\\""; break;
      case '\\': result += "\\\\"; break;
      case '\b': result += "\\b"; break;
      case '\f': result += "\\f"; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default:
        if (c < 0x20) {
          const char hex[] = "0123456789abcdef";
          result += "\\u00";
          result += hex[(c >> 4) & 0x0F];
          result += hex[c & 0x0F];
        } else {
          result += static_cast<char>(c);
        }
    }
  }
  return result;
}

bool integrityPass(const robot::RecordingVerifyReport& report) {
  return report.complete &&
         report.fault_bits == robot::kRecordingVerifyOk &&
         report.producer_drops == 0;
}

void printFaultsHuman(std::uint32_t bits) {
  if (bits == 0) {
    std::cout << "NONE";
    return;
  }
  bool first = true;
  for (const auto& fault : kFaultNames) {
    if ((bits & fault.bit) == 0) continue;
    if (!first) std::cout << ',';
    std::cout << fault.name;
    first = false;
  }
}

void printFaultsJson(std::uint32_t bits) {
  std::cout << '[';
  bool first = true;
  for (const auto& fault : kFaultNames) {
    if ((bits & fault.bit) == 0) continue;
    if (!first) std::cout << ',';
    std::cout << '"' << fault.name << '"';
    first = false;
  }
  std::cout << ']';
}

void printHuman(const std::string& path,
                const robot::RecordingVerifyReport& report,
                std::size_t file_size) {
  const auto& h = report.header;
  const double duration_s =
      report.last_time_us >= report.first_time_us
          ? static_cast<double>(report.last_time_us -
                                report.first_time_us) *
                1e-6
          : 0.0;
  std::cout << "FILE: " << path << '\n'
            << "STATUS: "
            << (integrityPass(report) ? "PASS" : "REPEAT") << '\n'
            << "FAULTS: ";
  printFaultsHuman(report.fault_bits);
  std::cout << "\nROBOT: "
            << fixedText(h.robot_id, sizeof(h.robot_id))
            << "\nSOFTWARE: "
            << fixedText(h.software_version,
                         sizeof(h.software_version))
            << "\nCOMMIT: "
            << fixedText(h.source_commit,
                         sizeof(h.source_commit))
            << "\nDIRTY_BUILD: " << (h.dirty_build ? "true" : "false")
            << "\nFORMAT: " << h.format_major << '.' << h.format_minor
            << "  LOG_SCHEMA: " << h.log_schema_major << '.'
            << h.log_schema_minor
            << "  FRAME_BYTES: " << h.frame_size_bytes
            << "\nSESSION: " << h.session_sequence
            << "  STORAGE_SEQUENCE: " << h.storage_sequence
            << "\nBOOT_ID: " << h.boot_id << "  RUN_HASH: 0x"
            << std::hex << std::setw(8) << std::setfill('0')
            << h.run_id_hash << std::dec << std::setfill(' ')
            << "\nCONFIG_HASH: 0x" << std::hex << std::setw(8)
            << std::setfill('0') << h.config_hash << std::dec
            << std::setfill(' ')
            << "\nFRAMES: " << report.recoverable_frames
            << "  BLOCKS: " << report.valid_blocks
            << "  PRODUCER_DROPS: " << report.producer_drops
            << "\nSEQUENCE: " << report.first_frame_sequence << ".."
            << report.last_frame_sequence
            << "  DURATION_S: " << std::fixed << std::setprecision(6)
            << duration_s
            << "\nVALID_BYTES: " << report.valid_bytes << '/'
            << file_size << "\n\n";
}

void printJson(const std::string& path,
               const robot::RecordingVerifyReport& report,
               std::size_t file_size) {
  const auto& h = report.header;
  std::cout << "{\"path\":\"" << jsonEscape(path)
            << "\",\"status\":\""
            << (integrityPass(report) ? "PASS" : "REPEAT")
            << "\",\"complete\":" << (report.complete ? "true" : "false")
            << ",\"fault_bits\":" << report.fault_bits
            << ",\"faults\":";
  printFaultsJson(report.fault_bits);
  std::cout << ",\"robot_id\":\""
            << jsonEscape(fixedText(h.robot_id, sizeof(h.robot_id)))
            << "\",\"software_version\":\""
            << jsonEscape(fixedText(h.software_version,
                                    sizeof(h.software_version)))
            << "\",\"source_commit\":\""
            << jsonEscape(fixedText(h.source_commit,
                                    sizeof(h.source_commit)))
            << "\",\"dirty_build\":"
            << (h.dirty_build ? "true" : "false")
            << ",\"format_major\":" << h.format_major
            << ",\"format_minor\":" << h.format_minor
            << ",\"log_schema_major\":" << h.log_schema_major
            << ",\"log_schema_minor\":" << h.log_schema_minor
            << ",\"frame_size_bytes\":" << h.frame_size_bytes
            << ",\"session_sequence\":" << h.session_sequence
            << ",\"storage_sequence\":" << h.storage_sequence
            << ",\"boot_id\":" << h.boot_id
            << ",\"run_id_hash\":" << h.run_id_hash
            << ",\"robot_id_hash\":" << h.robot_id_hash
            << ",\"config_hash\":" << h.config_hash
            << ",\"software_identity_hash\":"
            << h.software_identity_hash
            << ",\"hardware_revision\":" << h.hardware_revision
            << ",\"config_schema\":" << h.config_schema
            << ",\"calibration_revision\":" << h.calibration_revision
            << ",\"hardware_verification\":"
            << static_cast<unsigned>(h.hardware_verification)
            << ",\"recoverable_frames\":" << report.recoverable_frames
            << ",\"valid_blocks\":" << report.valid_blocks
            << ",\"producer_drops\":" << report.producer_drops
            << ",\"first_frame_sequence\":"
            << report.first_frame_sequence
            << ",\"last_frame_sequence\":"
            << report.last_frame_sequence
            << ",\"first_time_us\":" << report.first_time_us
            << ",\"last_time_us\":" << report.last_time_us
            << ",\"valid_bytes\":" << report.valid_bytes
            << ",\"file_bytes\":" << file_size << '}';
}

}  // namespace

int main(int argc, char** argv) {
  bool json = false;
  std::vector<std::string> inputs;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--json") {
      json = true;
    } else {
      inputs.push_back(argument);
    }
  }
  if (inputs.empty()) {
    std::cerr
        << "Usage: v5l_inspect [--json] <file-or-directory> [...]\n";
    return 1;
  }

  std::vector<std::string> paths;
  bool scan_error = false;
  for (const auto& input : inputs) {
    std::error_code error;
    const std::filesystem::path path(input);
    if (!std::filesystem::is_directory(path, error)) {
      paths.push_back(input);
      continue;
    }
    const auto options =
        std::filesystem::directory_options::skip_permission_denied;
    std::filesystem::recursive_directory_iterator current(
        path, options, error);
    const std::filesystem::recursive_directory_iterator end;
    if (error) {
      scan_error = true;
      continue;
    }
    while (current != end) {
      if (current->is_regular_file(error)) {
        std::string extension = current->path().extension().string();
        std::transform(
            extension.begin(), extension.end(), extension.begin(),
            [](unsigned char value) {
              return static_cast<char>(std::toupper(value));
            });
        if (extension == ".V5L" || extension == ".TMP")
          paths.push_back(current->path().string());
      }
      current.increment(error);
      if (error) {
        scan_error = true;
        error.clear();
      }
    }
  }
  std::sort(paths.begin(), paths.end());
  paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
  if (paths.empty()) {
    std::cerr << "No V5L/TMP recording files found.\n";
    return 1;
  }

  bool any_io_error = scan_error;
  bool any_integrity_failure = false;
  if (json) std::cout << '[';
  bool first_json = true;
  for (const auto& path : paths) {
    FileRecordingByteReader reader(path);
    if (!reader.valid()) {
      any_io_error = true;
      if (json) {
        if (!first_json) std::cout << ',';
        std::cout << "{\"path\":\"" << jsonEscape(path)
                  << "\",\"status\":\"IO_ERROR\"}";
        first_json = false;
      } else {
        std::cerr << "FILE: " << path << "\nSTATUS: IO_ERROR\n\n";
      }
      continue;
    }
    const auto report = robot::verifyRecording(reader);
    if (!integrityPass(report)) any_integrity_failure = true;
    if (json) {
      if (!first_json) std::cout << ',';
      printJson(path, report, reader.sizeBytes());
      first_json = false;
    } else {
      printHuman(path, report, reader.sizeBytes());
    }
  }
  if (json) std::cout << "]\n";
  if (any_io_error) return 1;
  return any_integrity_failure ? 2 : 0;
}
