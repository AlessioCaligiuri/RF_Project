#pragma once

#include <stdbool.h>

#define PHONE_NUMBER_LENGTH 11
#define MAX_MESSAGE_LENGTH 160
#define	PARSER_MAX_CMD_LENGTH 4
#define PARSER_MAX_FIELD_LENGTH	25
#define PARSER_MAX_TEXT_LENGTH	MAX_MESSAGE_LENGTH

typedef enum
{
	PARSER_WAIT_PLUS,
	PARSER_WAIT_COLON,
	PARSER_WAIT_FIELD_1,
	PARSER_WAIT_FIELD_2,
	PARSER_WAIT_FIELD_3,
	PARSER_WAIT_TEXT
} SIM_ParserState_t;

typedef struct
{
	SIM_ParserState_t state;
	int rxCharNumber;
	bool isCompleted;
	char cmd[PARSER_MAX_CMD_LENGTH+1];
	char field1[PARSER_MAX_FIELD_LENGTH+1];
	char field2[PARSER_MAX_FIELD_LENGTH+1];
	char field3[PARSER_MAX_FIELD_LENGTH+1];
	char text[PARSER_MAX_TEXT_LENGTH+1];


} SIM_Parser_t;

void SIM_Parser_init(SIM_Parser_t* parser);
void SIM_Parser_update(SIM_Parser_t* parser, char rxChar);
