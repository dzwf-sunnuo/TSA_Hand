#include "angle_codec.h"
#include <stddef.h>

// AngleCodec_DecodeBcdTenths：将四位压缩BCD角度解码为0.1度单位
// 参数：encoded - BCD编码角度；tenths - 接收解码结果的指针
// 返回值：true表示编码合法，false表示含非法数码或输出指针为空
bool AngleCodec_DecodeBcdTenths(uint16_t encoded, uint16_t *tenths)
{
    uint16_t thousands = (encoded >> 12) & 0x0FU;
    uint16_t hundreds = (encoded >> 8) & 0x0FU;
    uint16_t tens = (encoded >> 4) & 0x0FU;
    uint16_t ones = encoded & 0x0FU;

    if (tenths == NULL || thousands > 9U || hundreds > 9U || tens > 9U || ones > 9U) {
        return false;
    }

    *tenths = (uint16_t)(thousands * 1000U + hundreds * 100U + tens * 10U + ones);
    return true;
}
