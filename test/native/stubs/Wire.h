#pragma once
constexpr int I2C_SCL_1 = 1;
constexpr int I2C_SDA_1 = 0;
struct TwoWire {
  void setSCL(int) {}
  void setSDA(int) {}
  void begin() {}
};
inline TwoWire Wire1;
