/*
 * sbgEcom.c
 *
 *  Created on: 21.07.2018
 *      Author: norman
 */

#include "sbgEcom.h"

int ellipse_d_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum ELLIPSE_D_PARSE_STATE *state, ELLIPSE_MESSAGE *msg)
{
	int ret = -1;

	switch (*state) {
	case ELLIPSE_D_PARSE_STATE0_UNSYNC:
		if (c == SBG_ECOM_SYNC_1) {
			*state = ELLIPSE_D_PARSE_STATE1_SYNC1;
			(*parserbuf_index) = 0;
		}

		break;

	case ELLIPSE_D_PARSE_STATE1_SYNC1:
		if (c == SBG_ECOM_SYNC_2) {
			*state = ELLIPSE_D_PARSE_STATE2_SYNC2;
		}

		break;

	case ELLIPSE_D_PARSE_STATE2_SYNC2:
		*state = ELLIPSE_D_PARSE_STATE3_MSG;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->id = c;

		break;

	case ELLIPSE_D_PARSE_STATE3_MSG:
		*state = ELLIPSE_D_PARSE_STATE4_CLASS;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->cl = c;

		break;

	case ELLIPSE_D_PARSE_STATE4_CLASS:
		*state = ELLIPSE_D_PARSE_STATE5_LEN1;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->length = c<<8;

		break;

	case ELLIPSE_D_PARSE_STATE5_LEN1:
		*state = ELLIPSE_D_PARSE_STATE6_LEN2;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;
		msg->length |= c;

		break;

	case ELLIPSE_D_PARSE_STATE6_LEN2:
		if ((*parserbuf_index)<(msg->length+4))
		{
			msg->data[(*parserbuf_index)] = c;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		}
		else
		{
			*state = ELLIPSE_D_PARSE_STATE7_DATA;
		}

		break;

	case ELLIPSE_D_PARSE_STATE7_DATA:
		*state = ELLIPSE_D_PARSE_STATE8_CRC1;
		msg->crc = c<<8;

		break;

	case ELLIPSE_D_PARSE_STATE8_CRC1:
		msg->crc |= c;
		uint16_t computedCrc;
		computedCrc = calcCRC(parserbuf, *parserbuf_index);
		if (msg->crc == computedCrc)
		{
			*state = ELLIPSE_D_PARSE_STATE9_CRC2;
		}
		else
		{
			*state = ELLIPSE_D_PARSE_STATE0_UNSYNC;
		}


		break;

	case ELLIPSE_D_PARSE_STATE9_CRC2:
		*state = ELLIPSE_D_PARSE_STATE0_UNSYNC;
		if (c==SBG_ECOM_ETX)
		{
			ret = 0;
		}

		break;
	}

#ifdef SF0X_DEBUG
	printf("state: SF0X_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}


/**
 * @brief Calculate CRC checksum for the specified buffer according to the polynom given in the IG500N protocol specification
 * @param pBuffer Pointer to the buffer
 * @param bufferSize Number of bytes in the buffer
 * @retval crc Calculated CRC checksum
 */
uint16_t calcCRC(const char *pBuffer, uint16_t bufferSize)
{
	uint16_t poly = 0x8408;
	uint16_t crc = 0;
	uint8_t carry;
	uint8_t i_bits;
	uint16_t j;
	for (j=0; j<bufferSize; j++)
	{
		crc = crc ^ pBuffer[j];
		for (i_bits=0; i_bits<8; i_bits++)
		{
			carry = crc & 1;
			crc = crc / 2;
			if (carry)
			{
				crc = crc^poly;
			}
		}
	}
	return crc;
}

