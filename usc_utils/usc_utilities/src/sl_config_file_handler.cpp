/*
 * sl_config_file_handler.cpp
 *
 *  Created on: Dec 14, 2010
 *      Author: righetti
 */

#include <usc_utilities/sl_config_file_handler.h>

#include <string.h>
#include <stdio.h>

namespace usc_utilities
{

static int TRUE = 1;
static int FALSE = 0;

static void fopen_strip_recursive(const char *filename, FILE *temp);
static FILE *fopen_strip(const char *filename);
static int find_keyword(FILE *fp, char *name);


void fopen_strip_recursive(const char *filename, FILE *temp)
{

  enum Comments {
    C_NONE,
    C_PLAIN,
    C_PLUS
  };

  FILE *infile;
  int	rc,last_rc,j;
  int   skip = C_NONE;
  int   wait = 2;
  char  keyword[100];
  char  fname[100+1];

  infile = fopen(filename,"r");
  if (infile == NULL) {
    printf("Error: fopen_strip couldn't read file >%s<",filename);
    return;
  }


  // clean away comments and add new include files
  last_rc = EOF;
  while ((rc=fgetc(infile)) != EOF) {

    --wait;

    if (skip == C_NONE && rc == '*' && last_rc == '/') {
      skip = C_PLAIN;
    } else if (skip == C_NONE && rc == '/' && last_rc == '/') {
      skip = C_PLUS;
    } else if (skip == C_PLAIN && rc == '/' && last_rc == '*') {
      skip = C_NONE;
      wait = 2;
    } else if (skip == C_PLUS && rc == '\n') {
      skip = C_NONE;
      wait = 2;
    }

    // check for # signs
    if ( rc == '#' && skip == C_NONE && wait <= 0) {
      fscanf(infile,"%s",keyword);
      if (strcmp(keyword,"include")==0) {
	// find the string between double quotes
	while ((rc=fgetc(infile)) != '"') { // find the first double quote
	  if (rc == EOF) {
	    printf("Error: fopen_strip: could not parse include file name (first \")\n");
	    return;
	  }
	}
	j=0;
	while ((rc=fgetc(infile)) != '"') { // read until the next double quote
	  if (rc == EOF) {
	    printf("Error: fopen_strip: could not parse include file name (second \")\n");
	    return;
	  }
	  if (j<100)
	    fname[j++] = rc;
	}
	fname[j]='\0';

	fopen_strip_recursive(fname,temp);
      }
      continue;
    }

    if (skip == C_NONE && last_rc != EOF && wait <= 0) {
      if (last_rc != ';' && last_rc != ',' && last_rc != '=') {
	fputc(last_rc,temp);
      }
    }

    last_rc = rc;

  }

  fputc(last_rc,temp);
  fclose(infile);

}

FILE *fopen_strip(const char *filename)
{
  FILE *temp;

  // open a temp file
  if ((temp = tmpfile())==NULL)
    return NULL;

  // read the files and included files recursively
  fopen_strip_recursive(filename,temp);

  // reset the tempfile to the beginning
  rewind(temp);

  return temp;

}


int find_keyword(FILE *fp, char *name)
{

  int  i;
  int  rc = TRUE;
  char string[strlen(name)*2];
  int  l;
  char sep[]={' ','\n',':',',',';','=','\t','\0'};

  rewind(fp);
  l  = strlen(name);
  i  = 0;

  while (rc != EOF) {

    rc=fgetc(fp);
    if ( rc != EOF ) {

      string[i++] = rc;
      string[i]   = '\0';

      if ( strstr(string,name) != NULL) {
	// wait for one more character to judge whether this string
	// has the correct end delimiters
	if (strchr(sep,string[i-1]) != NULL) {
	  // now check for preceeding delimiter

	  if (i-l-2 < 0) // this means "name" was the first string in file
	    return TRUE;
	  else if (strchr(sep,string[i-l-2]) != NULL) //otherwise check delim
	    return TRUE;
	}
      }

      if (i >= 2*l-1) {
	strcpy(string,&(string[i-l]));
	i = strlen(string);
      }

    }

  }

  return FALSE;

}

bool readSLParameterPoolData(std::string filename, std::string keyword, int n_values, std::vector<double>& values)
{
	int    i,rc;
	FILE  *in;

	char key[keyword.size()];
	strcpy(key, keyword.c_str());

	in = fopen_strip(filename.c_str());
	if (in == NULL) {
		printf("ERROR: Cannot open file >%s<!\n",filename.c_str());
		return FALSE;
	}

	// find keyword
	if (!find_keyword(in, key)) {
		printf("ERROR: cannot find keyword %s\n", key);
		fclose(in);
		return FALSE;
	} else {
		for (i=1; i<=n_values; ++i) {
			rc=fscanf(in,"%lf",&(values[i]));
			if (rc != 1)
			{
				printf("ERROR rc != 1\n");
				return FALSE;
			}
		}
	}

	fclose(in);

	return TRUE;
}

}
