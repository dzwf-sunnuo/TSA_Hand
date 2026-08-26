#ifndef __ANGLE_CODEC_H
#define __ANGLE_CODEC_H

#include <stdbool.h>
#include <stdint.h>

// AngleCodec_DecodeBcdTenths：将四位压缩BCD角度解码为0.1度单位
// 参数：encoded - BCD编码角度；tenths - 接收解码结果的指针
// 返回值：true表示编码合法，false表示含非法数码或输出指针为空
bool AngleCodec_DecodeBcdTenths(uint16_t encoded, uint16_t *tenths);

#endif
