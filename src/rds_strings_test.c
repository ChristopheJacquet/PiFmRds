/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2024 Christophe Jacquet, F8FTK

    See https://github.com/ChristopheJacquet/PiFmRds

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <locale.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "rds_strings.h"

void print_hex_bytes(char* s, size_t size) {
    for (int i=0; i < size; i++) {
        printf("%02X ", s[i]);
    }
    printf("\n");
}

void assert_string(char* test_name, char* actual, char* expected, size_t size) {
    bool equal = true;
    for (int i=0; i < size; i++) {
        if (actual[i] != expected[i]) {
            equal = false;
            break;
        }
    }

    printf("Test: %s -> %s\n", test_name, equal ? "PASS" : "FAIL");
    if (equal) return;
    printf("Actual:   "); print_hex_bytes(actual, size);
    printf("Expected: "); print_hex_bytes(expected, size);
}

void test_src_shorter() {
    const size_t dst_size = 7;
    char dst[] = {0, 1, 2, 3, 4, 5, 6, 7};
    char dst_ref[] = {'A', 'B', 'C', 'D', ' ', ' ', ' ', 7};
    fill_rds_string(dst, "ABCD", dst_size);
    assert_string("Copy shorter string", dst, dst_ref, dst_size+1);
}

void test_src_longer() {
    const size_t dst_size = 7;
    char dst[] = {0, 1, 2, 3, 4, 5, 6, 7};
    char dst_ref[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 7};
    fill_rds_string(dst, "ABCDEFGHI", dst_size);
    assert_string("Copy longer string", dst, dst_ref, dst_size+1);
}

void test_same_sizes() {
    const size_t dst_size = 7;
    char dst[] = {0, 1, 2, 3, 4, 5, 6, 7};
    char dst_ref[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 7};
    fill_rds_string(dst, "ABCDEFG", dst_size);
    assert_string("Copy same-size string", dst, dst_ref, dst_size+1);
}

void test_non_ascii() {
    size_t dst_size = 20;
    char dst[dst_size];
    char* dst_ref = "M\x97""beltr\x91""gerf\x99\x8d""e d\x82\x9b""u";
    fill_rds_string(dst, "M\xc3\xb6""beltr\xc3\xa4""gerf\xc3\xbc\xc3\x9f""e "
                         "d\xc3\xa9\xc3\xa7""u", dst_size);
    assert_string("Convert non-ASCII characters", dst, dst_ref, dst_size);
}

void test_skip_invalid() {
    size_t dst_size = 6;
    char dst[dst_size];
    char dst_ref[] = {'A', 'B', 'C', ' ', ' ', ' '};
    fill_rds_string(dst, "A\xc0""B\xc1\xc2""C\xc3\xc4", dst_size);
    assert_string("Skip invalid bytes", dst, dst_ref, dst_size);
}

int main() {
    const char* locale = "C.UTF-8";
    const char* locale_set = setlocale(LC_ALL, locale);
    if (locale_set == NULL || strcmp(locale, locale_set) != 0) {
        printf("Failed to set locale. Locale is '%s', while it should be '%s'. "
               "Tests will probably fail.\n", locale_set, locale);
    }

    test_src_shorter();
    test_src_longer();
    test_same_sizes();
    test_non_ascii();
    test_skip_invalid();
}