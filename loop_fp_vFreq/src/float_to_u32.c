/*
 * float_to_u32.c
 *
 *  Created on: 27.06.2019
 *      Author: ga85piw
 */

// Only to change data type, bit level data remains same
// Necessary as in memory only u32 type data is written


	unsigned int float_to_u32(float value)
	{
		unsigned int result;
		union float_bytes{
			float fl;
			unsigned int U;
		} data;

		data.fl=value;
		result=data.U;

		return result;
	}
