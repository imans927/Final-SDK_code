/*
 * u32_to_int.c

// Only to change data type, bit level data remains same
// Necessary as in memory only u32 type data is written

 *
 *  Created on: 12.07.2019
 *      Author: ga85piw
 */


	int u32_to_int(unsigned int value)
	{
		float result;
		union float_bytes{
			int il;
			unsigned int U;
		} data;

		data.U=value;
		result=data.il;

		return result;
	}
