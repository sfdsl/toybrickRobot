
#include "user_string.h"

/**
 * @brief  10进展字符串转换为数字
 * @note   数字字符串中间不得有其它符号
 * @param  *str: 需要转换的10进制字符串
 * @param  *rt: 转换后的数字
 * @retval 转换字符串长度   0表示失败
 */
uint8 user_str_DECnumStr2num(IN uint8 *str, OUT sint32 *rt)
{
    // 
    register uint32 i = 0, j = 0;
    register uint8 *p;
    // if (*str == '\0')
    //     return 0;
    // while (*str == ' ')
    //     str++;
    p = str;
    if (*p == '-')
    {
        if ((p[1] - '0') > 9)
            return 0;  // 失败
        while (1)
        {
            p++;
            i = *p - '0';
            if (i <= 9)
            {
                j = j * 10 + i;
            }
            else
            {
                *rt = 0 - j;
                return p - str;
            }
        }
    }
    else // if (i == '+')
    {
        if (*p == '+')
            p++;
        if ((*p - '0') > 9)
            return 0;  // 失败
        while (1)
        {
            i = *p - '0';
            if (i <= 9)
            {
                j = j * 10 + i;
            }
            else
            {
                *rt = j;
                return p - str;
            }
            p++;
        }
    }
}

/**
 * @brief  将数字转换为10进制字符串
 * @note   
 * @param  num: 需要转换的数字
 * @param  *str: 输出的字符串，最后一位补'\0'
 * @retval 转换后的字符串长度
 */
uint8 user_str_num2DECnumStr(IN sint32 num, OUT uint8 *str)
{
    register uint8 j = 0;
    if (num < 0)
    {
        num = 0 - num;
        *str = '-';
        j++;
    }
    else if (num == 0)
    {
        *str++ = '0';
        *str = 0;
        return 1;
    }

    {
        register uint32 i = num;
        while (i != 0)
        {
            i /= 10;
            j++;
        }
    }
    str += j;
    *str = 0;

    while (num != 0)
    {
        str--;
        *str = num % 10 + '0';
        num /= 10;
    }
    return j;
}

/**
 * @brief  数字字符串转换为数字
 * @note   数字字符串中间不得有其它符号
 *         0x开头为16进制数
 *         0开头为8进制数
 *         0b开头为2进制数
 *         非0开头为10进制,不可转换小数
 * @param  *str: 需要转换的10进制字符串
 * @param  *rt: 转换后的数字
 * @retval 数字字符串长度  非0-成功(数字字符串长度)  0-失败
 */
uint8 user_str_numStr2num(IN uint8 *str, OUT sint32 *rt)
{
    // 
    register uint32 i = 0, j;
    register uint8 *p;
    // while (*str == ' ')
    //     str++;
    j = 0;
    if (*str == '-')
    {
        i = 1;
        p = str + 1;
    }
    else if (*str == '+')
    {
        i = 1;
        p = str + 1;
    }
    else
        p = str;
    if (*p == '0')
    {
        i++;
        p++;
        if ((*p == 'x') || (*p == 'X')) // 16进制
        {
            p++;
            i++;
            while (1)
            {
                if (*p >= '0')
                {
                    if (*p <= '9')
                    {
                        j = j * 16 + *p - '0';
                    }
                    else if ((*p >= 'a') && (*p <= 'f'))
                    {
                        j = j * 16 + 10 + *p - 'a';
                    }
                    else if ((*p >= 'A') && (*p <= 'F'))
                    {
                        j = j * 16 + 10 + *p - 'A';
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
                p++;
            }
        }
        else if ((*p == 'b') || (*p == 'B'))  // 2进制
        {
            p++;
            i++;
            while (1)
            {
                if (*p == '0')
                {
                    j = j * 2;
                }
                else if (*p == '1')
                {
                    j = j * 2 + 1;
                }
                else
                {
                    break;
                }
                p++;
            }
        }
        else  // 8进制
        {
            while (1)
            {
                if ((*p >= '0') && (*p <= '7'))
                {
                    j = j * 8 + *p - '0';
                }
                else
                {
                    if (p == str + i)  // 非法字符串  实际为10进制的0
                    {
                        *rt = 0;
                        return 1;
                    }
                    break;
                }
                p++;
            }
        }
    }
    else  // 10进制
    {
        while (1)
        {
            if ((*p >= '0') && (*p <= '9'))
            {
                j = j * 10 + *p - '0';
            }
            else
            {
                break;
            }
            p++;
        }
    }
    if (p == str + i)  // 非法字符串
        return 0;
    if (*str == '-')
        *rt = 0 - j;
    else
        *rt = j;
    return (p - str);
}

/**
 * @brief  字符串小写转大写
 * @note   
 * @param  *str: 待转换字符串  -> 输出字符串
 * @retval None
 */
void user_str_low2up(IN_OUT uint8 *str)
{
    while (*str != '\0')
    {
        if ((*str >= 'a') && (*str <= 'z'))
        {
            *str = *str - 'a' + 'A';
        }
        str++;
    }
}

/**
 * @brief  字符串比较
 * @note   从头开始比较
 * @param  *s: 源字符串
 * @param  *d: 待比较的字符串
 * @retval 相同的字符数 (不同字符开始下标)
 */
uint8 user_str_cmp(uint8 *s, uint8 *d)
{
    // 
    register uint8 i = 0;
    while ((*s != '\0') && (*d != '\0'))
    {
        if (*s++ == *d++)
        {
            i++;
        }
        else
        {
            break;
        }
    }
    return i;
}

/**
 * @brief  字符串复制
 * @note   将源字符串复制到目标字符串后面
 * @param  *s: 源字符串
 * @param  *d: 目标字符串，需要有足够空间，否则会存在内存溢出bug
 * @param  len: 复制长度，0表示全部复制
 * @retval 目标字符串结束位置指针
 */
uint8 * user_str_copy(uint8 *s, uint8 *d, uint8 len)
{
    if (len != 0)
    {
        while (len != 0)
        {
            len--;
            *d++ = *s++;
        }
    }
    else
    {
        while (*s != '\0')
        {
            *d++ = *s++;
        }
    }
    *d = '\0';
    return d;
}

/**
 * @brief  查找字符串
 * @note   在源字符串中查找目标字符串
 * @param  *s: 源字符串
 * @param  *d: 需要查找的字符串
 * @retval 出现位置下标，0xFF表示未查找到
 */
uint8 user_str_seek(uint8 *s, uint8 *d)
{
    register uint8 i, j = 0;
    while (d[j] != '\0')
    {
        j++;
    }
    if (j == 0)
        return 0xFF;
    
    i = 0;
    while (*s != '\0')
    {
        if (*s == *d)
        {
            register uint8 k = 1;
            while (k < j)
            {
                if (s[k] != d[k])
                    break;
                k++;
            }
            if (k == j)
                return i;
        }
        s++;
        i++;
    }
    return 0xFF;
}

/**
 * @brief  数组查找指定序列
 * @note   在源数组中查找目标数组
 * @param  *s: 源数组
 * @param  s_len: 源数组长度
 * @param  *d: 需要查找的数组
 * @param  d_len: 查找数组长度
 * @retval 出现位置下标，0xFF表示未查找到
 */
uint8 user_array_seek(uint8 *s, uint8 s_len, uint8 *d, uint8 d_len)
{
    register uint8 i;
    i = 0;
    while (s_len != 0)
    {
        if (*s == *d)
        {
            register uint8 k = 1;
            while (k < d_len)
            {
                if (s[k] != d[k])
                    break;
                k++;
            }
            if (k == d_len)
                return i;
        }
        s++;
        i++;
        s_len--;
    }
    return 0xFF;
}

/*void u3_printf(char* fmt,...)  
{  
	uint16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	memset(USART3_TX_BUF, 0, USART3_MAX_SEND_LEN);	
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	BSP_Printf("S: %s\r\n", USART3_TX_BUF);
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART3,USART3_TX_BUF[j]); 
	} 
}*/


