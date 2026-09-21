
#ifndef __USER_STRING_H__
#define __USER_STRING_H__

#ifndef uint8
#define uint8 unsigned char
#endif

#ifndef uint16
#define uint16 unsigned short
#endif

#ifndef uint32
#define uint32 unsigned long
#endif

#ifndef sint8
#define sint8 char
#endif

#ifndef sint16
#define sint16 short
#endif

#ifndef sint32
#define sint32 long
#endif

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef IN_OUT
#define IN_OUT
#endif

/**
 * @brief  10进展字符串转换为数字
 * @note   数字字符串中间不得有其它符号
 * @param  *str: 需要转换的10进制字符串
 * @param  *rt: 转换后的数字,注意长度必须一致
 * @retval 转换字符串长度   0表示失败
 */
uint8 user_str_DECnumStr2num(IN uint8 *str, OUT sint32 *rt);

/**
 * @brief  将数字转换为10进制字符串
 * @note   
 * @param  num: 需要转换的数字
 * @param  *str: 输出的字符串，最后一位补'\0'
 * @retval 转换后的字符串长度
 */
uint8 user_str_num2DECnumStr(IN sint32 num, OUT uint8 *str);

/**
 * @brief  数字字符串转换为数字
 * @note   数字字符串中间不得有其它符号
 *         0x开头为16进制数
 *         0开头为8进制数
 *         0b开头为2进制数
 *         非0开头为10进制,不可转换小数
 * @param  *str: 需要转换的10进制字符串
 * @param  *rt: 转换后的数字,注意长度必须一致
 * @retval 数字字符串长度  非0-成功(数字字符串长度)  0-失败
 */
uint8 user_str_numStr2num(IN uint8 *str, OUT sint32 *rt);

/**
 * @brief  字符串小写转大写
 * @note   
 * @param  *str: 待转换字符串  -> 输出字符串
 * @retval None
 */
void user_str_low2up(IN_OUT uint8 *str);

/**
 * @brief  字符串比较
 * @note   从头开始比较
 * @param  *s: 源字符串
 * @param  *d: 待比较的字符串
 * @retval 相同的字符数 (不同字符开始下标)
 */
uint8 user_str_cmp(uint8 *s, uint8 *d);

/**
 * @brief  字符串复制
 * @note   将源字符串复制到目标字符串后面
 * @param  *s: 源字符串
 * @param  *d: 目标字符串，需要有足够空间，否则会存在内存溢出bug
 * @param  len: 复制长度，0表示全部复制
 * @retval 目标字符串结束位置指针
 */
uint8 * user_str_copy(uint8 *s, uint8 *d, uint8 len);

/**
 * @brief  查找字符串
 * @note   在源字符串中查找目标字符串
 * @param  *s: 源字符串
 * @param  *d: 需要查找的字符串
 * @retval 出现位置下标，0xFF表示未查找到
 */
uint8 user_str_seek(uint8 *s, uint8 *d);

/**
 * @brief  数组查找指定序列
 * @note   在源数组中查找目标数组
 * @param  *s: 源数组
 * @param  s_len: 源数组长度
 * @param  *d: 需要查找的数组
 * @param  d_len: 查找数组长度
 * @retval 出现位置下标，0xFF表示未查找到
 */
uint8 user_array_seek(uint8 *s, uint8 s_len, uint8 *d, uint8 d_len);

#endif
