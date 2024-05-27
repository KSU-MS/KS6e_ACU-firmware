#ifndef IMD_TEST
#define IMD_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <iostream>
#include "imd.h"
#include "imd.cpp"

TEST(imdTesting, test_imd_resistance_calc)
{
    imd imd_;
    imd_.updateInsulationReading(5);
    EXPECT_EQ(imd_.getInsulationgReading(),50000);
    imd_.updateInsulationReading(95);
    EXPECT_EQ(imd_.getInsulationgReading(),0);

}
TEST(imdTesting, test_imd_state)
{
    imd imd_;
    imd_.updateState(0);
    EXPECT_EQ(imd_.getState(),0);
    for (uint8_t j = 0; j < 6; j++)
    {
        std::cout << "Expecting all "<< static_cast<int>(j) << "\n";

        for (int i = -5; i < 5; i++)
        {
            int input = i+(j*10);
            if (input <0)
            {
                input = 0;
            }
            imd_.updateState(input);
            // std::cout << imd_.getState() << "\n";
            std::cout << "Input: " << (i+(j*10))  << " Output: "<< static_cast<int>(imd_.getState()) << "\n";
            EXPECT_EQ(imd_.getState(),(j));
        }
    }
}
#endif