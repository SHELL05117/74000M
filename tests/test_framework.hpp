#pragma once

#include <exception>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace test {

using TestFunction = void (*)();

struct Case {
  const char* name;
  TestFunction function;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

class Registrar {
 public:
  Registrar(const char* name, TestFunction function) {
    registry().push_back({name, function});
  }
};

class Failure final : public std::exception {
 public:
  explicit Failure(std::string message) : message_(std::move(message)) {}
  const char* what() const noexcept override { return message_.c_str(); }

 private:
  std::string message_;
};

inline void require(bool condition, const char* expression, const char* file,
                    int line) {
  if (condition) return;
  throw Failure(std::string(file) + ":" + std::to_string(line) +
                ": requirement failed: " + expression);
}

inline void requireNear(double actual, double expected, double tolerance,
                        const char* actual_expression,
                        const char* expected_expression, const char* file,
                        int line) {
  if (std::isfinite(actual) && std::isfinite(expected) &&
      std::abs(actual - expected) <= tolerance) {
    return;
  }
  throw Failure(std::string(file) + ":" + std::to_string(line) +
                ": expected " + actual_expression + " ~= " +
                expected_expression + ", actual=" + std::to_string(actual) +
                ", expected=" + std::to_string(expected));
}

inline int runAll() {
  int failures = 0;
  for (const auto& test_case : registry()) {
    try {
      test_case.function();
      std::cout << "[PASS] " << test_case.name << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": " << error.what()
                << '\n';
    } catch (...) {
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": unknown exception\n";
    }
  }
  std::cout << registry().size() - static_cast<std::size_t>(failures) << "/"
            << registry().size() << " tests passed\n";
  return failures == 0 ? 0 : 1;
}

}  // namespace test

#define ROBOT_TEST_JOIN_IMPL(left, right) left##right
#define ROBOT_TEST_JOIN(left, right) ROBOT_TEST_JOIN_IMPL(left, right)
#define ROBOT_TEST(name)                                                       \
  static void ROBOT_TEST_JOIN(robot_test_function_, __LINE__)();               \
  static const ::test::Registrar ROBOT_TEST_JOIN(robot_test_registrar_,         \
                                                  __LINE__)(                    \
      name, &ROBOT_TEST_JOIN(robot_test_function_, __LINE__));                 \
  static void ROBOT_TEST_JOIN(robot_test_function_, __LINE__)()

#define ROBOT_REQUIRE(expression)                                              \
  ::test::require(static_cast<bool>(expression), #expression, __FILE__,        \
                  __LINE__)

#define ROBOT_REQUIRE_NEAR(actual, expected, tolerance)                        \
  ::test::requireNear((actual), (expected), (tolerance), #actual, #expected,   \
                      __FILE__, __LINE__)
