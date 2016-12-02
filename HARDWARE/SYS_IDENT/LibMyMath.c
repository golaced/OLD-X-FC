/*********************************************************************************
*                                山猫飞控（Lynx）
*                                   测试版
*
* Version   	: V1.0
* By        	: Lynx@ustc 84693469@qq.com
*
* For       	: Stm32f103VET6
* Mode      	: Thumb2
* Description   : 目前仅有卷积算法的数学库
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include "LibMyMath.h"

/*
 * Name										: fconv
 * Description						: float的卷积与多项式乘法函数，请自行确定好r的长度并完成初始化，防止r溢出
 * Entry                  : float a[]被卷积数组, int m是a的长度, float b[]被卷积数组, int n是b的长度, float r[]输出的数组需用户自行初始化
 * Return                 : void
 * Author									: lynx 84693469@qq.com.
 *
 * History
 * ----------------------
 * Rev										: 0.00
 * Date										: 06/03/2013
 *
 * create.
 * ----------------------
 */
void fconv(float a[], int m, float b[], int n, float r[])
{
	//请自行确定好r的长度并完成初始化，防止r溢出
	int i, j, t = m + n - 1;
	for (i = 0; i < t; i++)
		for (j = 0; j < m && j <= i; j++) 
			if (i - j < n) 
				r[i] += a[j] * b[i - j];
}

