#include "angle_codec.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// test_valid_bcd_angles：验证合法BCD角度能解码为0.1度单位
// 参数：无
// 返回值：无
static void test_valid_bcd_angles(void)
{
    uint16_t tenths = 0;

    assert(AngleCodec_DecodeBcdTenths(0x0050U, &tenths));
    assert(tenths == 50U);

    assert(AngleCodec_DecodeBcdTenths(0x0300U, &tenths));
    assert(tenths == 300U);

    assert(AngleCodec_DecodeBcdTenths(0x0555U, &tenths));
    assert(tenths == 555U);

    assert(AngleCodec_DecodeBcdTenths(0x0900U, &tenths));
    assert(tenths == 900U);
}

// test_invalid_bcd_angles：验证含非十进制数码的BCD输入会被拒绝
// 参数：无
// 返回值：无
static void test_invalid_bcd_angles(void)
{
    uint16_t tenths = 123U;

    assert(!AngleCodec_DecodeBcdTenths(0x03AFU, &tenths));
    assert(tenths == 123U);

    assert(!AngleCodec_DecodeBcdTenths(0x0A00U, &tenths));
    assert(tenths == 123U);
}

// test_null_output：验证空输出指针会被安全拒绝
// 参数：无
// 返回值：无
static void test_null_output(void)
{
    assert(!AngleCodec_DecodeBcdTenths(0x0300U, NULL));
}

// main：运行BCD角度解码单元测试
// 参数：无
// 返回值：0表示全部测试通过
int main(void)
{
    test_valid_bcd_angles();
    test_invalid_bcd_angles();
    test_null_output();
    puts("角度BCD解码测试通过");
    return 0;
}
