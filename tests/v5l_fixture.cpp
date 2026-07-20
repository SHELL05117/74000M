#include "robot/config/robot_config.hpp"
#include "robot/telemetry/recording_codec.hpp"

#include <cstddef>
#include <fstream>
#include <string>

namespace {

class FileWriter final : public robot::RecordingByteWriter {
 public:
  explicit FileWriter(const char* path)
      : output_(path, std::ios::binary | std::ios::trunc) {}

  bool valid() const { return static_cast<bool>(output_); }

  bool writeBytes(const void* data, std::size_t size) override {
    output_.write(static_cast<const char*>(data),
                  static_cast<std::streamsize>(size));
    return static_cast<bool>(output_);
  }

 private:
  std::ofstream output_;
};

}  // namespace

int main(int argc, char** argv) {
  const bool empty =
      argc == 3 && std::string(argv[1]) == "--empty";
  if ((!empty && argc != 2) || (empty && argc != 3)) return 1;
  const char* path = empty ? argv[2] : argv[1];
  FileWriter writer(path);
  if (!writer.valid()) return 2;

  const robot::RobotConfig config =
      robot::make1690XCommissioningConfig();
  const robot::RecordingMetadata metadata =
      robot::makeRecordingMetadata(
          config, 0x1122334455667788ull,
          "0123456789abcdef0123456789abcdef01234567", false);
  robot::RecordingFileEncoder encoder;
  if (!encoder.begin(writer, metadata, 3, 17, 9000)) return 3;
  if (empty) return encoder.finish(0) ? 0 : 5;
  const std::uint32_t run_id =
      robot::recordingRunId(metadata.boot_id, 3);
  robot::LogFrame frames[2] = {
      robot::makeEmptyLogFrame({10000, 8, 7}, run_id),
      robot::makeEmptyLogFrame({20000, 9, 7}, run_id),
  };
  frames[0].controller.left_y = 0.25;
  frames[1].left_motor[0].position_rad = 1.5;
  if (!encoder.append(frames, 2)) return 4;
  return encoder.finish(0) ? 0 : 5;
}
