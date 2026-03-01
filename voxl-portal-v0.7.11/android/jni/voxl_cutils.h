/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef VOXL_CUTILS_H
#define VOXL_CUTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#define CLEAR_TERMINAL		"\033c"		// same as typing "clear" in bash
#define DISABLE_WRAP		"\033[?7l"	// disables line wrap, be sure to enable before exiting
#define ENABLE_WRAP			"\033[?7h"	// default terminal behavior
#define CLEAR_LINE			"\033[2K"	// erases line but leaves curser in place
#define GOTO_TOP_LEFT		"\033[f"	// move curser to top left

#define RESET_FONT          "\e[0m"

#define FONT_BOLD           "\e[1m"
#define FONT_DIM            "\e[2m"
#define FONT_UNDERLINE      "\e[4m"
#define FONT_BLINK          "\e[5m"
#define FONT_INVERT         "\e[7m"
#define FONT_HIDDEN         "\e[8m"

#define RESET_BOLD          "\e[21m"
#define RESET_DIM           "\e[22m"
#define RESET_UNDERLINE     "\e[24m"
#define RESET_BLINK         "\e[25m"
#define RESET_INVERT        "\e[27m"
#define RESET_HIDDEN        "\e[28m"

#define RESET_COLOR         "\e[39m"

#define COLOR_BLK           "\e[30m"
#define COLOR_RED           "\e[31m"
#define COLOR_GRN           "\e[32m"
#define COLOR_YLW           "\e[33m"
#define COLOR_BLU           "\e[34m"
#define COLOR_MAG           "\e[35m"
#define COLOR_CYN           "\e[36m"
#define COLOR_LIT_GRY       "\e[37m"

#define COLOR_DRK_GRY       "\e[90m"
#define COLOR_LIT_RED       "\e[91m"
#define COLOR_LIT_GRN       "\e[92m"
#define COLOR_LIT_YLW       "\e[93m"
#define COLOR_LIT_BLU       "\e[94m"
#define COLOR_LIT_MAG       "\e[95m"
#define COLOR_LIT_CYN       "\e[96m"
#define COLOR_WHT           "\e[97m"

#define RESET_COLOR_BKG     "\e[49m"

#define COLOR_BKG_BLK       "\e[40m"
#define COLOR_BKG_RED       "\e[41m"
#define COLOR_BKG_GRN       "\e[42m"
#define COLOR_BKG_YLW       "\e[43m"
#define COLOR_BKG_BLU       "\e[44m"
#define COLOR_BKG_MAG       "\e[45m"
#define COLOR_BKG_CYN       "\e[46m"
#define COLOR_BKG_LIT_GRY   "\e[47m"

#define COLOR_BKG_DRK_GRY   "\e[100m"
#define COLOR_BKG_LIT_RED   "\e[101m"
#define COLOR_BKG_LIT_GRN   "\e[102m"
#define COLOR_BKG_LIT_YLW   "\e[103m"
#define COLOR_BKG_LIT_BLU   "\e[104m"
#define COLOR_BKG_LIT_MAG   "\e[105m"
#define COLOR_BKG_LIT_CYN   "\e[106m"
#define COLOR_BKG_WHT       "\e[107m"

#define GET_COLOR_GT(val, red, ylw) ((val >= red) ? COLOR_RED : ((val >= ylw) ? COLOR_YLW : COLOR_GRN))
#define GET_COLOR_LT(val, red, ylw) ((val <= red) ? COLOR_RED : ((val <= ylw) ? COLOR_YLW : COLOR_GRN))

#define VCU_silent(x) {\
	FILE *silentfd = fopen("/dev/null","w");     \
	int savedstdoutfd = dup(STDOUT_FILENO);      \
	fflush(stdout);                              \
	dup2(fileno(silentfd), STDOUT_FILENO);       \
	x                                            \
	fflush(stdout);                              \
	fclose(silentfd);                            \
	dup2(savedstdoutfd, STDOUT_FILENO);          \
	close(savedstdoutfd);                        \
}

#define VCU_silentErrors(x) {\
	FILE *_silent_err_fd = fopen("/dev/null","w");     \
	int savedstderrfd = dup(STDERR_FILENO);            \
	fflush(stderr);                                    \
	dup2(fileno(_silent_err_fd), STDERR_FILENO);       \
	x                                                  \
	fflush(stderr);                                    \
	fclose(_silent_err_fd);                            \
	dup2(savedstderrfd, STDERR_FILENO);                \
	close(savedstderrfd);                              \
}

int64_t VCU_time_monotonic_ns(void);
int64_t VCU_time_realtime_ns(void);


#ifdef __cplusplus
}
#endif

#endif	// VOXL_CUTILS_H
