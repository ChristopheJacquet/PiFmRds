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

#include <stdlib.h>
#include <string.h>
#include <wchar.h>

// Convert an Unicode code point to a character encoded using the RDS character set.
char codepoint_to_rds_char(wchar_t codepoint) {
    // The following table is sorted by ascending RDS character code.
    switch (codepoint) {
        case 0x000A: return 0x0A;   // LINE FEED
        case 0x000B: return 0x0B;   // END OF HEADLINE
        case 0x000D: return 0x0D;   // CARRIAGE RETURN
        case 0x001F: return 0x1F;   // WORD BREAK - SOFT HIPHEN
        case 0x0020: return 0x20;   // SPACE
        case 0x0021: return 0x21;   // EXCLAMATION MARK
        case 0x0022: return 0x22;   // QUOTATION MARK
        case 0x0023: return 0x23;   // NUMBER SIGN
        case 0x00A4: return 0x24;   // CURRENCY SIGN
        case 0x0025: return 0x25;   // PERCENT SIGN
        case 0x0026: return 0x26;   // AMPERSAND
        case 0x0027: return 0x27;   // APOSTOPHE
        case 0x0028: return 0x28;   // LEFT PARENTHESIS
        case 0x0029: return 0x29;   // RIGHT PARENTHESIS
        case 0x002A: return 0x2A;   // ASTERISK
        case 0x002B: return 0x2B;   // PLUS SIGN
        case 0x002C: return 0x2C;   // COMMA
        case 0x002D: return 0x2D;   // HYPHEN-MINUS
        case 0x002E: return 0x2E;   // FULL STOP
        case 0x002F: return 0x2F;   // SOLIDUS
        case 0x0030: return 0x30;   // DIGIT ZERO
        case 0x0031: return 0x31;   // DIGIT ONE
        case 0x0032: return 0x32;   // DIGIT TWO
        case 0x0033: return 0x33;   // DIGIT THREE
        case 0x0034: return 0x34;   // DIGIT FOUR
        case 0x0035: return 0x35;   // DIGIT FIVE
        case 0x0036: return 0x36;   // DIGIT SIX
        case 0x0037: return 0x37;   // DIGIT SEVEN
        case 0x0038: return 0x38;   // DIGIT EIGHT
        case 0x0039: return 0x39;   // DIGIT NINE
        case 0x003A: return 0x3A;   // COLON
        case 0x003B: return 0x3B;   // SEMICOLON
        case 0x003C: return 0x3C;   // LESS-THAN SIGN
        case 0x003D: return 0x3D;   // EQUALS SIGN
        case 0x003E: return 0x3E;   // GREATER-THAN SIGN
        case 0x003F: return 0x3F;   // QUESTION MARK
        case 0x0040: return 0x40;   // COMMERCIAL AT
        case 0x0041: return 0x41;   // LATIN CAPITAL LETTER A
        case 0x0042: return 0x42;   // LATIN CAPITAL LETTER B
        case 0x0043: return 0x43;   // LATIN CAPITAL LETTER C
        case 0x0044: return 0x44;   // LATIN CAPITAL LETTER D
        case 0x0045: return 0x45;   // LATIN CAPITAL LETTER E
        case 0x0046: return 0x46;   // LATIN CAPITAL LETTER F
        case 0x0047: return 0x47;   // LATIN CAPITAL LETTER G
        case 0x0048: return 0x48;   // LATIN CAPITAL LETTER H
        case 0x0049: return 0x49;   // LATIN CAPITAL LETTER I
        case 0x004A: return 0x4A;   // LATIN CAPITAL LETTER J
        case 0x004B: return 0x4B;   // LATIN CAPITAL LETTER K
        case 0x004C: return 0x4C;   // LATIN CAPITAL LETTER L
        case 0x004D: return 0x4D;   // LATIN CAPITAL LETTER M
        case 0x004E: return 0x4E;   // LATIN CAPITAL LETTER N
        case 0x004F: return 0x4F;   // LATIN CAPITAL LETTER O
        case 0x0050: return 0x50;   // LATIN CAPITAL LETTER P
        case 0x0051: return 0x51;   // LATIN CAPITAL LETTER Q
        case 0x0052: return 0x52;   // LATIN CAPITAL LETTER R
        case 0x0053: return 0x53;   // LATIN CAPITAL LETTER S
        case 0x0054: return 0x54;   // LATIN CAPITAL LETTER T
        case 0x0055: return 0x55;   // LATIN CAPITAL LETTER U
        case 0x0056: return 0x56;   // LATIN CAPITAL LETTER V
        case 0x0057: return 0x57;   // LATIN CAPITAL LETTER W
        case 0x0058: return 0x58;   // LATIN CAPITAL LETTER X
        case 0x0059: return 0x59;   // LATIN CAPITAL LETTER Y
        case 0x005A: return 0x5A;   // LATIN CAPITAL LETTER Z
        case 0x005B: return 0x5B;   // LEFT SQUARE BRACKET
        case 0x005C: return 0x5C;   // REVERSE SOLIDUS
        case 0x005D: return 0x5D;   // RIGHT SQUARE BRACKET
        case 0x2015: return 0x5E;   // HORIZONTAL BAR
        case 0x005F: return 0x5F;   // LOW LINE
        case 0x2551: return 0x60;   // BOX DRAWINGS DOUBLE VERTICAL
        case 0x0061: return 0x61;   // LATIN SMALL LETTER A
        case 0x0062: return 0x62;   // LATIN SMALL LETTER B
        case 0x0063: return 0x63;   // LATIN SMALL LETTER C
        case 0x0064: return 0x64;   // LATIN SMALL LETTER D
        case 0x0065: return 0x65;   // LATIN SMALL LETTER E
        case 0x0066: return 0x66;   // LATIN SMALL LETTER F
        case 0x0067: return 0x67;   // LATIN SMALL LETTER G
        case 0x0068: return 0x68;   // LATIN SMALL LETTER H
        case 0x0069: return 0x69;   // LATIN SMALL LETTER I
        case 0x006A: return 0x6A;   // LATIN SMALL LETTER J
        case 0x006B: return 0x6B;   // LATIN SMALL LETTER K
        case 0x006C: return 0x6C;   // LATIN SMALL LETTER L
        case 0x006D: return 0x6D;   // LATIN SMALL LETTER M
        case 0x006E: return 0x6E;   // LATIN SMALL LETTER N
        case 0x006F: return 0x6F;   // LATIN SMALL LETTER O
        case 0x0070: return 0x70;   // LATIN SMALL LETTER P
        case 0x0071: return 0x71;   // LATIN SMALL LETTER Q
        case 0x0072: return 0x72;   // LATIN SMALL LETTER R
        case 0x0073: return 0x73;   // LATIN SMALL LETTER S
        case 0x0074: return 0x74;   // LATIN SMALL LETTER T
        case 0x0075: return 0x75;   // LATIN SMALL LETTER U
        case 0x0076: return 0x76;   // LATIN SMALL LETTER V
        case 0x0077: return 0x77;   // LATIN SMALL LETTER W
        case 0x0078: return 0x78;   // LATIN SMALL LETTER X
        case 0x0079: return 0x79;   // LATIN SMALL LETTER Y
        case 0x007A: return 0x7A;   // LATIN SMALL LETTER Z
        case 0x007B: return 0x7B;   // LEFT CURLY BRACKET
        case 0x007C: return 0x7C;   // VERTICAL LINE
        case 0x007D: return 0x7D;   // RIGHT CURLY BRACKET
        case 0x00AF: return 0x7E;   // MACRON
        case 0x00E1: return 0x80;   // LATIN SMALL LETTER A WITH ACUTE
        case 0x00E0: return 0x81;   // LATIN SMALL LETTER A WITH GRAVE
        case 0x00E9: return 0x82;   // LATIN SMALL LETTER E WITH ACUTE
        case 0x00E8: return 0x83;   // LATIN SMALL LETTER E WITH GRAVE
        case 0x00ED: return 0x84;   // LATIN SMALL LETTER I WITH ACUTE
        case 0x00EC: return 0x85;   // LATIN SMALL LETTER I WITH GRAVE
        case 0x00F3: return 0x86;   // LATIN SMALL LETTER O WITH ACUTE
        case 0x00F2: return 0x87;   // LATIN SMALL LETTER O WITH GRAVE
        case 0x00FA: return 0x88;   // LATIN SMALL LETTER U WITH ACUTE
        case 0x00F9: return 0x89;   // LATIN SMALL LETTER U WITH GRAVE
        case 0x00D1: return 0x8A;   // LATIN CAPITAL LETTER N WITH TILDE
        case 0x00C7: return 0x8B;   // LATIN CAPITAL LETTER C WITH CEDILLA
        case 0x015E: return 0x8C;   // LATIN CAPITAL LETTER S WITH CEDILLA
        case 0x00DF: return 0x8D;   // LATIN SMALL LETTER SHARP S (German)
        case 0x00A1: return 0x8E;   // INVERTED EXCLAMATION MARK
        case 0x0132: return 0x8F;   // LATIN CAPITAL LIGATURE IJ
        case 0x00E2: return 0x90;   // LATIN SMALL LETTER A WITH CIRCUMFLEX
        case 0x00E4: return 0x91;   // LATIN SMALL LETTER A WITH DIAERESIS
        case 0x00EA: return 0x92;   // LATIN SMALL LETTER E WITH CIRCUMFLEX
        case 0x00EB: return 0x93;   // LATIN SMALL LETTER E WITH DIAERESIS
        case 0x00EE: return 0x94;   // LATIN SMALL LETTER I WITH CIRCUMFLEX
        case 0x00EF: return 0x95;   // LATIN SMALL LETTER I WITH DIAERESIS
        case 0x00F4: return 0x96;   // LATIN SMALL LETTER O WITH CIRCUMFLEX
        case 0x00F6: return 0x97;   // LATIN SMALL LETTER O WITH DIAERESIS
        case 0x00FB: return 0x98;   // LATIN SMALL LETTER U WITH CIRCUMFLEX
        case 0x00FC: return 0x99;   // LATIN SMALL LETTER U WITH DIAERESIS
        case 0x00F1: return 0x9A;   // LATIN SMALL LETTER N WITH TILDE
        case 0x00E7: return 0x9B;   // LATIN SMALL LETTER C WITH CEDILLA
        case 0x015F: return 0x9C;   // LATIN SMALL LETTER S WITH CEDILLA
        case 0x011F: return 0x9D;   // LATIN SMALL LETTER G WITH BREVE
        case 0x0131: return 0x9E;   // LATIN SMALL LETTER DOTLESS I
        case 0x0133: return 0x9F;   // LATIN SMALL LIGATURE IJ
        case 0x00AA: return 0xA0;   // FEMININE ORDINAL INDICATOR
        case 0x03B1: return 0xA1;   // GREEK SMALL LETTER ALPHA
        case 0x00A9: return 0xA2;   // COPYRIGHT SIGN
        case 0x2030: return 0xA3;   // PER MILLE SIGN
        case 0x011E: return 0xA4;   // LATIN CAPITAL LETTER G WITH BREVE
        case 0x011B: return 0xA5;   // LATIN SMALL LETTER E WITH CARON
        case 0x0148: return 0xA6;   // LATIN SMALL LETTER N WITH CARON
        case 0x0151: return 0xA7;   // LATIN SMALL LETTER O WITH DOUBLE ACUTE
        case 0x03C0: return 0xA8;   // GREEK SMALL LETTER PI
        case 0x20AC: return 0xA9;   // EURO SIGN
        case 0x00A3: return 0xAA;   // POUND SIGN
        case 0x0024: return 0xAB;   // DOLLAR SIGN
        case 0x2190: return 0xAC;   // LEFTWARDS ARROW
        case 0x2191: return 0xAD;   // UPWARDS ARROW
        case 0x2192: return 0xAE;   // RIGHTWARDS ARROW
        case 0x2193: return 0xAF;   // DOWNWARDS ARROW
        case 0x00BA: return 0xB0;   // MASCULIN ORDINAL INDICATOR
        case 0x00B9: return 0xB1;   // SUPERSCRIPT ONE
        case 0x00B2: return 0xB2;   // SUPERSCRIPT TWO
        case 0x00B3: return 0xB3;   // SUPERSCRIPT THREE
        case 0x00B1: return 0xB4;   // PLUS-MINUS SIGN
        case 0x0130: return 0xB5;   // LATIN CAPITAL LETTER I WITH DOT ABOVE
        case 0x0144: return 0xB6;   // LATIN SMALL LETTER N WITH ACUTE
        case 0x0171: return 0xB7;   // LATIN SMALL LETTER U WITH DOUBLE ACUTE
        case 0x00B5: return 0xB8;   // MIKRO SIGN
        case 0x00BF: return 0xB9;   // INVERTED QUESTION MARK
        case 0x00F7: return 0xBA;   // DIVISION SIGN
        case 0x00B0: return 0xBB;   // DEGREE SIGN
        case 0x00BC: return 0xBC;   // VULGAR FRACTION ONE QUARTER
        case 0x00BD: return 0xBD;   // VULGAR FRACTION ONE HALF
        case 0x00BE: return 0xBE;   // VULGAR FRACTION THREE QUARTERS
        case 0x00A7: return 0xBF;   // SECTION SIGN
        case 0x00C1: return 0xC0;   // LATIN CAPITAL LETTER A WITH ACUTE
        case 0x00C0: return 0xC1;   // LATIN CAPITAL LETTER A WITH GRAVE
        case 0x00C9: return 0xC2;   // LATIN CAPITAL LETTER E WITH ACUTE
        case 0x00C8: return 0xC3;   // LATIN CAPITAL LETTER E WITH GRAVE
        case 0x00CD: return 0xC4;   // LATIN CAPITAL LETTER I WITH ACUTE
        case 0x00CC: return 0xC5;   // LATIN CAPITAL LETTER I WITH GRAVE
        case 0x00D3: return 0xC6;   // LATIN CAPITAL LETTER O WITH ACUTE
        case 0x00D2: return 0xC7;   // LATIN CAPITAL LETTER O WITH GRAVE
        case 0x00DA: return 0xC8;   // LATIN CAPITAL LETTER U WITH ACUTE
        case 0x00D9: return 0xC9;   // LATIN CAPITAL LETTER U WITH GRAVE
        case 0x0158: return 0xCA;   // LATIN CAPITAL LETTER R WITH CARON
        case 0x010C: return 0xCB;   // LATIN CAPITAL LETTER C WITH CARON
        case 0x0160: return 0xCC;   // LATIN CAPITAL LETTER S WITH CARON
        case 0x017D: return 0xCD;   // LATIN CAPITAL LETTER Z WITH CARON
        case 0x00D0: return 0xCE;   // LATIN CAPITAL LETTER ETH
        case 0x013F: return 0xCF;   // LATIN CAPITAL LETTER L WITH MIDDLE DOT
        case 0x00C2: return 0xD0;   // LATIN CAPITAL LETTER A WITH CIRCUMFLEX
        case 0x00C4: return 0xD1;   // LATIN CAPITAL LETTER A WITH DIAERRESIS
        case 0x00CA: return 0xD2;   // LATIN CAPITAL LETTER E WITH CIRCUMFLEX
        case 0x00CB: return 0xD3;   // LATIN CAPITAL LETTER A WITH DIAERESIS
        case 0x00CE: return 0xD4;   // LATIN CAPITAL LETTER I WITH CIRCUMFLEX
        case 0x00CF: return 0xD5;   // LATIN CAPITAL LETTER I WITH DIAERESIS
        case 0x00D4: return 0xD6;   // LATIN CAPITAL LETTER O WITH CIRCUMFLEX
        case 0x00D6: return 0xD7;   // LATIN CAPITAL LETTER O WITH DIAERRESIS
        case 0x00DB: return 0xD8;   // LATIN CAPITAL LETTER U WITH CIRCUMFLEX
        case 0x00DC: return 0xD9;   // LATIN CAPITAL LETTER U WITH DIAERESIS
        case 0x0159: return 0xDA;   // LATIN SMALL LETTER R WITH CARON
        case 0x010D: return 0xDB;   // LATIN SMALL LETTER C WITH CARON
        case 0x0161: return 0xDC;   // LATIN SMALL LETTER S WITH CARON
        case 0x017E: return 0xDD;   // LATIN SMALL LETTER Z WITH CARON
        case 0x0111: return 0xDE;   // LATIN SMALL LETTER D WITH STROKE
        case 0x0140: return 0xDF;   // LATIN SMALL LETTER L WITH MIDDLE DOT
        case 0x00C3: return 0xE0;   // LATIN CAPITAL LETTER A WITH TILDE
        case 0x00C5: return 0xE1;   // LATIN CAPITAL LETTER A WITH RING ABOVE
        case 0x00C6: return 0xE2;   // LATIN CAPITAL LETTER AE
        case 0x0152: return 0xE3;   // LATIN CAPITAL LIGATURE OE
        case 0x0177: return 0xE4;   // LATIN SMALL LETTER Y WITH CIRCUMFLEX
        case 0x00DD: return 0xE5;   // LATIN CAPITAL LETTER Y WITH ACUTE
        case 0x00D5: return 0xE6;   // LATIN CAPITAL LETTER O WITH TILDE
        case 0x00D8: return 0xE7;   // LATIN CAPITAL LETTER O WITH STROKE
        case 0x00DE: return 0xE8;   // LATIN CAPITAL LETTER THORN
        case 0x014A: return 0xE9;   // LATIN CAPITAL LETTER ENG
        case 0x0154: return 0xEA;   // LATIN CAPITAL LETTER R WITH ACUTE
        case 0x0106: return 0xEB;   // LATIN CAPITAL LETTER C WITH ACUTE
        case 0x015A: return 0xEC;   // LATIN CAPITAL LETTER S WITH ACUTE
        case 0x0179: return 0xED;   // LATIN CAPITAL LETTER Z WITH ACUTE
        case 0x0166: return 0xEE;   // LATIN CAPITAL LETTER T WITH STROKE
        case 0x00F0: return 0xEF;   // LATIN SMALL LETTER ETH
        case 0x00E3: return 0xF0;   // LATIN SMALL LETTER A WITH TILDE
        case 0x00E5: return 0xF1;   // LATIN SMALL LETTER A WITH RING
        case 0x00E6: return 0xF2;   // LATIN SMALL LETTER AE
        case 0x0153: return 0xF3;   // LATIN SMALL LIGATURE OE
        case 0x0175: return 0xF4;   // LATIN SMALL LETTER W WITH CIRCUMFLEX
        case 0x00FD: return 0xF5;   // LATIN SMALL LETTER Y WITH ACUTE
        case 0x00F5: return 0xF6;   // LATIN SMALL LETTER O WITH TILDE
        case 0x00F8: return 0xF7;   // LATIN SMALL LETTER O WITH STROKE
        case 0x00FE: return 0xF8;   // LATIN SMALL LETTER THORN
        case 0x014B: return 0xF9;   // LATIN SMALL LETTER ENG
        case 0x0155: return 0xFA;   // LATIN SMALL LETTER R WITH ACUTE
        case 0x0107: return 0xFB;   // LATIN SMALL LETTER C WITH ACUTE
        case 0x015B: return 0xFC;   // LATIN SMALL LETTER S WITH ACUTE
        case 0x017A: return 0xFD;   // LATIN SMALL LETTER Z WITH ACUTE
        case 0x0167: return 0xFE;   // LATIN SMALL LETTER T WITH STROKE
        default: return 0x20;   // Return a SPACE character for all other code points.
    }
}

void fill_rds_string(char* rds_string, char* src_string, size_t rds_string_size) {
    mbtowc(NULL, 0, 0);   // Reset decoder.

    // First try to copy the source string.
    size_t remaining_src_size = strlen(src_string);
    size_t remaining_rds_size = rds_string_size;
    wchar_t codepoint;
    while (remaining_src_size > 0 && remaining_rds_size > 0) {
        int size = mbtowc(&codepoint, src_string, remaining_src_size);
        if (size == 0) break;   // End of source string.
        if (size < 0) {         // Decode error. Try to skip 1 byte and resync.
            src_string++;
            remaining_src_size--;
            continue;
        }
        *rds_string = codepoint_to_rds_char(codepoint);
        rds_string++;
        remaining_rds_size--;
        src_string += size;
        remaining_src_size -= size;
    }

    // Pad the RDS string with SPACE characters.
    while (remaining_rds_size > 0) {
        *rds_string = 0x20;
        rds_string++;
        remaining_rds_size--;
    }
}
