#include "sim_parser.h"

/**
 * @brief	Initializes the parser.
 * @param	parser	Parser struct.
 */
void SIM_Parser_init(SIM_Parser_t* parser)
{
	/* All string should be empty at the beginning */
	parser->field1[0] = '\0';
	parser->field2[0] = '\0';
	parser->field3[0] = '\0';
	parser->text[0] = '\0';
	
	(parser->state) = PARSER_WAIT_PLUS;
	(parser->isCompleted) = false;
}

/**
 * @brief	Updates the parser with the last received char.
 * @param	parser	Parser struct.
 * @param	rxChar	Last received char.
 */
void SIM_Parser_update(SIM_Parser_t* parser, char rxChar)
{
	switch(parser->state)
	{
		case PARSER_WAIT_PLUS: //every command starts with a "+"
			if(rxChar == '+')
			{
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_COLON;
			}
			break;
			
		/* Wait for end of command, that is ":" */	
		case PARSER_WAIT_COLON:
			/* If end has come, add a string terminator to the command */
			if(rxChar == ':')
			{
				(parser->cmd[parser->rxCharNumber]) = '\0';
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_FIELD_1;
				break;
			}

			//load the current char in the command string
			(parser->cmd[parser->rxCharNumber]) = rxChar;
			parser->rxCharNumber++;
			
			/* Check for the length correctness */
			if(parser->rxCharNumber > PARSER_MAX_CMD_LENGTH)
			{
				//Reset the FSM
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_PLUS;
			}
			break;
			
		case PARSER_WAIT_FIELD_1:
			if(parser->rxCharNumber == 0) //if the first char
			{
				/* We are looking for a space as first char, that is not part of command;
				if the first char is not a space, there is an error => reset FSM */
				if(rxChar != ' ') 
				{
					parser->rxCharNumber = 0;
					parser->state = PARSER_WAIT_PLUS;
				}
				else //correct
				{
					parser->rxCharNumber++;
				}
				break;
			}
			if(rxChar == ',') //End of field
			{
				(parser->field1[parser->rxCharNumber-1]) = '\0';
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_FIELD_2;
				break;
			}
			
			//load the current char in the field1 string
			(parser->field1[parser->rxCharNumber-1]) = rxChar;
			parser->rxCharNumber++;
			
			/* Check for the length correctness */
			if(parser->rxCharNumber > (PARSER_MAX_FIELD_LENGTH + 1))
			{
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_PLUS;
			}
			break;
			
		case PARSER_WAIT_FIELD_2:
			if(rxChar == ',') //End of field
			{
				(parser->field2[parser->rxCharNumber]) = '\0';
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_FIELD_3;
				break;
			}
			
			//load the current char in the field2 string
			(parser->field2[parser->rxCharNumber]) = rxChar;
			parser->rxCharNumber++;
			
			/* Check for the length correctness */
			if(parser->rxCharNumber > (PARSER_MAX_FIELD_LENGTH))
			{
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_PLUS;
			}
			break;
			
		case PARSER_WAIT_FIELD_3:
			if((rxChar == '\r') || (rxChar == '\n')) //End of field
			{
				(parser->field3[parser->rxCharNumber]) = '\0';
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_TEXT;
				break;
			}
			
			//load the current char in the field3 string
			(parser->field3[parser->rxCharNumber]) = rxChar;
			parser->rxCharNumber++;
			
			/* Check for the length correctness */
			if(parser->rxCharNumber > (PARSER_MAX_FIELD_LENGTH))
			{
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_PLUS;
			}
			break;
			
		case PARSER_WAIT_TEXT:
			/* The new line char can be the first char because of the command format or
			it can be the end of the text. We are assuming that the text doesn't contain any new line char. */
			if((rxChar == '\r') || (rxChar == '\n'))
			{
				if(parser->rxCharNumber == 0) //if the first char, no problem
				{
					break;
				}
				else //completed text catching
				{
					(parser->text[parser->rxCharNumber]) = '\0';
					parser->isCompleted = true;
					break;
				}
				
			}
			
			//load the current char in the text string
			(parser->text[parser->rxCharNumber]) = rxChar;
			parser->rxCharNumber++;
			
			/* Check for the length correctness */
			if(parser->rxCharNumber > (PARSER_MAX_TEXT_LENGTH))
			{
				parser->rxCharNumber = 0;
				parser->state = PARSER_WAIT_PLUS;
			}
			break;
	}
}

