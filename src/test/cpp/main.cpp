#include <hal/HAL.h>//cannot open source file "hal/HAL.h"

#include "gtest/gtest.h"//cannot open source file "gtest/gtest.h"

int main(int argc, char **argv)
{
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
