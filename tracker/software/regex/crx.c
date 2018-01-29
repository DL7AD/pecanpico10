/*
 * name: crx
 * description: Regular Expression Engine (light weight version) for C Language, using double-recursion and function pointers.
 * author: ken (hexabox) seto
 * date: 2009.08~09
 * license: LGPL
 *
 * version: 0.13.13.cpp.1
 */
#include "crx.h"

#include <string.h>
#include <stdlib.h>   // for atoi

#define TYPE_CHAR   1
#define TYPE_CMD    2
#define TYPE_PREFIX 4
#define TYPE_SUFFIX 8
#define TYPE_OPEN   16
#define TYPE_CLOSE  32

typedef struct {
    char    id;
    int     span;
    void*   fcn;
    int     type;
} Cmd;

// function pointers
#define CMD2    (int(*)(char*,char*))
#define CMD3    (int(*)(char*,char*,char*))

// function shortcuts
#define _CMD2(str)  int c_##str(char* pat, char* sam)
#define _CMD3(str)  int c_##str(char* pat, char* sam, char* endp)

int match(char*, char*, char*);

/*
 * rules of commands:
 * 1. return number of consumed characters in sample string
 * 2. TYPE_CLOSE follows TYPE_OPEN immediately in command table
 */
int c_achar(char* pat, char* sam);
int c_any(char* pat, char* sam);
int c_escape(char* pat, char* sam);
int c_group(char* pat, char* sam);
int c_multi(char* pat, char* sam, char* endp);
int c_option(char* pat, char* sam);

Cmd cmd_tbl[] = {
    {'(',    0,    (void*)c_group,    TYPE_OPEN            },
    {')',    1,    (void*)NULL,       TYPE_CLOSE           },
    {'[',    0,    (void*)c_option,   TYPE_OPEN            },
    {']',    1,    (void*)NULL,       TYPE_CLOSE           },
    {'{',    0,    (void*)c_multi,    TYPE_SUFFIX|TYPE_OPEN},
    {'}',    1,    (void*)NULL,       TYPE_CLOSE           },
    {'*',    1,    (void*)c_multi,    TYPE_SUFFIX          },
    {'+',    1,    (void*)c_multi,    TYPE_SUFFIX          },
    {'?',    1,    (void*)c_multi,    TYPE_SUFFIX          },
    {'.',    1,    (void*)c_any,      TYPE_CMD             }, 
    {'\\',   2,    (void*)c_escape,   TYPE_PREFIX          },
    {0,      1,    (void*)c_achar,    TYPE_CHAR            }
};

Cmd* get_cmd(char id)
{
    Cmd* cmd = &cmd_tbl[0];
    while (cmd->id != 0)
    {
        if (id == cmd->id)
            break;
        cmd++;
    }
    return cmd;
}

inline bool is_suffix(char id)
{
    Cmd* cmd = get_cmd(id);
    return (cmd ? (cmd->type & TYPE_SUFFIX) == TYPE_SUFFIX : false);
}

char* find_close(char* init, char stop)
{
    int cnt = 0;
    char* ptr = init;

    while (*ptr)
    {
        if (*ptr == *init)
            cnt++;
        else if (*ptr == stop)
            cnt--;

        if (cnt == 0)
            return ptr;

        ptr++;
    }
    return NULL;
}

char* get_next_pat(char* cur)   // find next unit of pattern
{
    Cmd* cmd = get_cmd(*cur);

    if (cmd->type & TYPE_OPEN)
    {
        cur = find_close(cur, (cmd+1)->id);
        if (cur)
            return (cur + 1);
    }
    return (cur + cmd->span);
}

// -------------- command handlers ----------------

int c_any(char* pat, char* sam) {
	(void)pat;
	(void)sam;
	return 1;
}
int c_achar(char* pat, char* sam) {
	(void)pat;
	(void)sam;
	return (*pat == *sam);
}

int c_group(char* pat, char* sam)    // sub pattern
{
    char *close = find_close(pat, ')');
    if (!close) return false;
    return match(pat+1, sam, close);
}

int c_escape(char* pat, char* sam)
{
    char magic[16] = "";
    
    switch (*++pat)
    {
        // problem 1: inefficient copying everytime
        // problem 2: mem illegal access for long magic
        case 'd':   // digit
            strcpy(magic, "[0-9]");
            break;
        case 'D':   // non-digit
            strcpy(magic, "[^0-9]");
            break;
        case 'x':   // hex digit
            strcpy(magic, "[0-9A-Fa-f]");
            break;
        case 'X':
            strcpy(magic, "[^0-9A-Fa-f]");
            break;
        case 'o':   // octal digit
            strcpy(magic, "[0-7]");
            break;
        case 'O':
            strcpy(magic, "[^0-7]");
            break;
        case 'w':   // word character
            strcpy(magic, "[0-9A-Za-z_]");
            break;
        case 'W':
            strcpy(magic, "[^0-9A-Za-z_]");
            break;
        case 'h':   // head of word character
            strcpy(magic, "[0-9A-Za-z]");
            break;
        case 'H':
            strcpy(magic, "[^0-9A-Za-z]");
            break;
        case 'a':   // alphabetic character
            strcpy(magic, "[A-Za-z]");
            break;
        case 'A':
            strcpy(magic, "[^A-Za-z]");
            break;
        case 'l':   // lowercase character
            strcpy(magic, "[a-z]");
            break;
        case 'L':
            strcpy(magic, "[^a-z]");
            break;
        case 'u':   // uppercase character
            strcpy(magic, "[A-Z]");
            break;
        case 'U':
            strcpy(magic, "[^A-Z]");
            break;
    }

    if (*magic)
        return match(magic, sam, strchr(magic, 0));
    else
        return (*pat == *sam);
}


int c_option(char* pat, char* sam)
{
    bool invert;
    char* from = NULL;
    char* to = NULL;
    char *close;

    close = pat;
    do {
        close = find_close(close, ']');
        if (!close) return false;
    } while (close[-1] == '\\');

    invert = (pat[1] == '^');
    pat += invert ? 2 : 1;

    while (pat < close)
    {
        if (*pat == '-' && from)
        {
            to = pat + 1;
            if (*to == '\\') to ++;
            if (to >= close) break;
            if (*sam >= *from && *sam <= *to)
                return (invert ? 0 : 1);
            pat = to + 1;
            continue;
        }

        from = pat;
        if (*from == '\\') from ++;
        if (from >= close) break;
        if (*sam == *from)
            return (invert ? 0 : 1);

        pat++;
    }

    return (invert ? 1 : 0);
}

int c_multi(char* pat, char* sam, char* endp)
{
    // repeated variables
    Cmd* cmd = get_cmd(*pat);
    int  found;
    char* start_sam = sam;

    // new local variables
    int repeat = 0;
    int good_follows = 0;

    char *ends = strchr(sam, 0);
    char *good_sam = NULL;

    char *multi = get_next_pat(pat);
    char *next_pat = get_next_pat(multi);

    int min = 0, max = 0;

    switch (*multi)
    {
        case '{':  // for range of repetition
            {
                char* comma = strchr(multi, ',');
                char* close = strchr(multi, '}');

                min = atoi(multi + 1);
                if (comma && comma < close)
                    max = atoi(comma + 1);
                else
                    max = min;
            } break;
        case '+':
            min = 1;
            break;
        case '?':
            max = 1;
            break;
    }

    while (sam < ends)
    {
        found = (CMD2 cmd->fcn)(pat, sam);

        if (!found) break;  // can be less than min

        // condition 1
        repeat ++;
        sam += found;   // advance ptr

        if (min && repeat < min)
            continue;

        if (*next_pat)
        {
            if (sam >= ends)
                break;

            found = match(next_pat, sam, endp);
            if (found)    // condition 2
            {
                good_follows = found;
                good_sam = sam;
            }
        }

        if (max && repeat >= max)
            break;
    }

    // return here
    if (repeat < min)
        found = 0;

    else if (!*next_pat)
        found = sam - start_sam;

    else if (good_sam)  // *next_pat > 0
        found = good_sam + good_follows - start_sam;

    else if (!min)
    {
        if (repeat == 0)    // no match before *multi
            found = match(next_pat, sam, endp);

        else if (!good_sam) // repeat > 0 is wrongly match before *multi
            found = match(next_pat, start_sam, endp);
    }

    else found = 0;

    return found;
}

/*
 * return: # found in sam
 */
int match(char* pat, char* sam, char* endp)
{
    Cmd* cmd;
    char* next_pat;
    int  found;
    char* start_sam = sam;


    while (pat < endp)
    {
        if (*pat == 0)  break;
        if (*sam == 0)  return 0;

        next_pat  = get_next_pat(pat);
        cmd = get_cmd(*pat);

        if (next_pat  &&  pat < endp  &&  is_suffix(*next_pat))
        {
            cmd = get_cmd(*next_pat);
            return (sam - start_sam) + (CMD3 cmd->fcn)(pat, sam, endp);
        }
        else
        {
            found = (CMD2 cmd->fcn)(pat, sam);
            if (!found) return 0;

            sam += found;
            pat = next_pat;
        }
    }

    return (sam - start_sam);
}

// -------------- external interface ---------------

char* regex(char* pat, char* sam, int* len)
{
    *len = 0;
    while (*pat && *sam)
    {
        *len = match(pat, sam, strchr(pat, '\0'));
        if (*len > 0)
            break;
        sam++;
    }

    if (*len > 0)
        return sam;
    else
        return NULL;
}

