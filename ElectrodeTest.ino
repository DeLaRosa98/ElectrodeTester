/* Developed by Pablo de la Rosa for Grupo de Fotónica Aplicada from CEMDATIC, Escuela Técnica Superior de Ingeniero de Telecomunicación (ETSIT), Universidad Politécnica de Madrid (UPM)
* Driver for 5 demuxes and 2 7segement displays
* Has the following functionalities:
*  - 72 electrodes automatically: connects one by one and with a 1000ms delay each of the 72 electrodes (uses the two 7seg display to show which electrode is connected)
*  - 72 electrodes manually: manually connects each of the 72 electrodes by pressing a button (uses the two 7seg display to show which electrode is connected)
*  - 24 electrodes automatically: connects one by one and with a 1000ms delay each of the 24 electrodes (uses the two 7seg display to show which electrode is connected)
*  - 24 electrodes manually: manually connects each of the 24 electrodes by pressing a button (uses the two 7seg display to show which electrode is connected)
* The functionalities are chosen through connecting Jumpers
*/

#include <Arduino.h>
#include <math.h>

/*7Segments Decenas*/
#define p_D1 24
#define p_C1 25
#define p_B1 26
#define p_A1 23

/*7Segments Decenas*/
#define p_D0 20
#define p_C0 21
#define p_B0 22
#define p_A0 19

/*Demux control signals*/
#define p_S3 16
#define p_S2 15
#define p_S1 14
#define p_S0 13

/*Demux enables*/
#define p_EN5 12
#define p_EN4 11
#define p_EN3 10
#define p_EN2 9
#define p_EN1 8

/*Jumpers*/
#define p_J72 6
#define p_J24 7

/*Button*/
#define p_Man 5

const int timeThreshold = 150;
long startTime = 0;
bool _J24 = false;
bool _J72 = false;
bool _Manual = false;
volatile bool buttonPress = false;

/* Data for demuxeswith 
    {EN5, EN4, EN3, EN2, EN1, S3, S2, S1, S0} structure */
bool electrodes[72][9] = {       // |Demux|   Channel   |   Pos   |
    {1, 1, 1, 1, 0, 0, 0, 0, 0}, // | EN1 |  0000 (0)   |  [0][j] |
    {1, 1, 1, 1, 0, 0, 0, 0, 1}, // | EN1 |  0001 (1)   |  [1][j] |
    {1, 1, 1, 1, 0, 0, 0, 1, 0}, // | EN1 |  0010 (2)   |  [2][j] |
    {1, 1, 1, 1, 0, 0, 0, 1, 1}, // | EN1 |  0011 (3)   |  [3][j] |
    {1, 1, 1, 1, 0, 0, 1, 0, 0}, // | EN1 |  0100 (4)   |  [4][j] |
    {1, 1, 1, 1, 0, 0, 1, 0, 1}, // | EN1 |  0101 (5)   |  [5][j] |
    {1, 1, 1, 1, 0, 0, 1, 1, 0}, // | EN1 |  0110 (6)   |  [6][j] |
    {1, 1, 1, 1, 0, 0, 1, 1, 1}, // | EN1 |  0111 (7)   |  [7][j] |
    {1, 1, 1, 1, 0, 1, 0, 0, 0}, // | EN1 |  1000 (8)   |  [8][j] |
    {1, 1, 1, 1, 0, 1, 0, 0, 1}, // | EN1 |  1001 (9)   |  [9][j] |
    {1, 1, 1, 1, 0, 1, 0, 1, 0}, // | EN1 |  1010 (10)  | [10][j] |
    {1, 1, 1, 1, 0, 1, 0, 1, 1}, // | EN1 |  1011 (11)  | [11][j] |
    {1, 1, 1, 1, 0, 1, 1, 0, 0}, // | EN1 |  1100 (12)  | [12][j] |
    {1, 1, 1, 1, 0, 1, 1, 0, 1}, // | EN1 |  1101 (13)  | [13][j] |
    {1, 1, 1, 1, 0, 1, 1, 1, 0}, // | EN1 |  1110 (14)  | [14][j] |
    {1, 1, 1, 1, 0, 1, 1, 1, 1}, // | EN1 |  1111 (15)  | [15][j] |
    {1, 1, 1, 0, 1, 0, 0, 0, 0}, // | EN2 |  0000 (0)   | [16][j] |
    {1, 1, 1, 0, 1, 0, 0, 0, 1}, // | EN2 |  0001 (1)   | [17][j] |
    {1, 1, 1, 0, 1, 0, 0, 1, 0}, // | EN2 |  0010 (2)   | [18][j] |
    {1, 1, 1, 0, 1, 0, 0, 1, 1}, // | EN2 |  0011 (3)   | [19][j] |
    {1, 1, 1, 0, 1, 0, 1, 0, 0}, // | EN2 |  0100 (4)   | [20][j] |
    {1, 1, 1, 0, 1, 0, 1, 0, 1}, // | EN2 |  0101 (5)   | [21][j] |
    {1, 1, 1, 0, 1, 0, 1, 1, 0}, // | EN2 |  0110 (6)   | [22][j] |
    {1, 1, 1, 0, 1, 0, 1, 1, 1}, // | EN2 |  0111 (7)   | [23][j] |
    {1, 1, 1, 0, 1, 1, 0, 0, 0}, // | EN2 |  1000 (8)   | [24][j] |
    {1, 1, 1, 0, 1, 1, 0, 0, 1}, // | EN2 |  1001 (9)   | [25][j] |
    {1, 1, 1, 0, 1, 1, 0, 1, 0}, // | EN2 |  1010 (10)  | [26][j] |
    {1, 1, 1, 0, 1, 1, 0, 1, 1}, // | EN2 |  1011 (11)  | [27][j] |
    {1, 1, 1, 0, 1, 1, 1, 0, 0}, // | EN2 |  1100 (12)  | [28][j] |
    {1, 1, 1, 0, 1, 1, 1, 0, 1}, // | EN2 |  1101 (13)  | [29][j] |
    {1, 1, 1, 0, 1, 1, 1, 1, 0}, // | EN2 |  1110 (14)  | [30][j] |
    {1, 1, 1, 0, 1, 1, 1, 1, 1}, // | EN2 |  1111 (15)  | [31][j] |
    {1, 1, 0, 1, 1, 0, 0, 0, 0}, // | EN3 |  0000 (0)   | [32][j] |
    {1, 1, 0, 1, 1, 0, 0, 0, 1}, // | EN3 |  0001 (1)   | [33][j] |
    {1, 1, 0, 1, 1, 0, 0, 1, 0}, // | EN3 |  0010 (2)   | [34][j] |
    {1, 1, 0, 1, 1, 0, 0, 1, 1}, // | EN3 |  0011 (3)   | [35][j] |
    {1, 1, 0, 1, 1, 0, 1, 0, 0}, // | EN3 |  0100 (4)   | [36][j] |
    {1, 1, 0, 1, 1, 0, 1, 0, 1}, // | EN3 |  0101 (5)   | [37][j] |
    {1, 1, 0, 1, 1, 0, 1, 1, 0}, // | EN3 |  0110 (6)   | [38][j] |
    {1, 1, 0, 1, 1, 0, 1, 1, 1}, // | EN3 |  0111 (7)   | [39][j] |
    {1, 1, 0, 1, 1, 1, 0, 0, 0}, // | EN3 |  1000 (8)   | [40][j] |
    {1, 1, 0, 1, 1, 1, 0, 0, 1}, // | EN3 |  1001 (9)   | [41][j] |
    {1, 1, 0, 1, 1, 1, 0, 1, 0}, // | EN3 |  1010 (10)  | [42][j] |
    {1, 1, 0, 1, 1, 1, 0, 1, 1}, // | EN3 |  1011 (11)  | [43][j] |
    {1, 1, 0, 1, 1, 1, 1, 0, 0}, // | EN3 |  1100 (12)  | [44][j] |
    {1, 1, 0, 1, 1, 1, 1, 0, 1}, // | EN3 |  1101 (13)  | [45][j] |
    {1, 1, 0, 1, 1, 1, 1, 1, 0}, // | EN3 |  1110 (14)  | [46][j] |
    {1, 1, 0, 1, 1, 1, 1, 1, 1}, // | EN3 |  1111 (15)  | [47][j] |
    {1, 0, 1, 1, 1, 0, 0, 0, 0}, // | EN4 |  0000 (0)   | [48][j] |
    {1, 0, 1, 1, 1, 0, 0, 0, 1}, // | EN4 |  0001 (1)   | [49][j] |
    {1, 0, 1, 1, 1, 0, 0, 1, 0}, // | EN4 |  0010 (2)   | [50][j] |
    {1, 0, 1, 1, 1, 0, 0, 1, 1}, // | EN4 |  0011 (3)   | [51][j] |
    {1, 0, 1, 1, 1, 0, 1, 0, 0}, // | EN4 |  0100 (4)   | [52][j] |
    {1, 0, 1, 1, 1, 0, 1, 0, 1}, // | EN4 |  0101 (5)   | [53][j] |
    {1, 0, 1, 1, 1, 0, 1, 1, 0}, // | EN4 |  0110 (6)   | [54][j] |
    {1, 0, 1, 1, 1, 0, 1, 1, 1}, // | EN4 |  0111 (7)   | [55][j] |
    {1, 0, 1, 1, 1, 1, 0, 0, 0}, // | EN4 |  1000 (8)   | [56][j] |
    {1, 0, 1, 1, 1, 1, 0, 0, 1}, // | EN4 |  1001 (9)   | [57][j] |
    {1, 0, 1, 1, 1, 1, 0, 1, 0}, // | EN4 |  1010 (10)  | [58][j] |
    {1, 0, 1, 1, 1, 1, 0, 1, 1}, // | EN4 |  1011 (11)  | [59][j] |
    {1, 0, 1, 1, 1, 1, 1, 0, 0}, // | EN4 |  1100 (12)  | [60][j] |
    {1, 0, 1, 1, 1, 1, 1, 0, 1}, // | EN4 |  1101 (13)  | [61][j] |
    {1, 0, 1, 1, 1, 1, 1, 1, 0}, // | EN4 |  1110 (14)  | [62][j] |
    {1, 0, 1, 1, 1, 1, 1, 1, 1}, // | EN4 |  1111 (15)  | [63][j] |
    {0, 1, 1, 1, 1, 0, 0, 0, 0}, // | EN5 |  0000 (0)   | [64][j] |
    {0, 1, 1, 1, 1, 0, 0, 0, 1}, // | EN5 |  0001 (1)   | [65][j] |
    {0, 1, 1, 1, 1, 0, 0, 1, 0}, // | EN5 |  0010 (2)   | [66][j] |
    {0, 1, 1, 1, 1, 0, 0, 1, 1}, // | EN5 |  0011 (3)   | [67][j] |
    {0, 1, 1, 1, 1, 0, 1, 0, 0}, // | EN5 |  0100 (4)   | [68][j] |
    {0, 1, 1, 1, 1, 0, 1, 0, 1}, // | EN5 |  0101 (5)   | [69][j] |
    {0, 1, 1, 1, 1, 0, 1, 1, 0}, // | EN5 |  0110 (6)   | [70][j] |
    {0, 1, 1, 1, 1, 0, 1, 1, 1}  // | EN5 |  0111 (7)   | [71][j] |
};

/* Data for bcd-7seg decoders with 
    {D1, C1, B1, A1, D0, C0, B0, A0} structure */
bool display[72][8] = {       // |   Dec   |   Uni  |   Pos   |
    {0, 0, 0, 0, 0, 0, 0, 1}, // | 0000 (0)|0001 (1)| [0][k]  |
    {0, 0, 0, 0, 0, 0, 1, 0}, // | 0000 (0)|0010 (2)| [1][k]  |
    {0, 0, 0, 0, 0, 0, 1, 1}, // | 0000 (0)|0011 (3)| [2][k]  |
    {0, 0, 0, 0, 0, 1, 0, 0}, // | 0000 (0)|0100 (4)| [3][k]  |
    {0, 0, 0, 0, 0, 1, 0, 1}, // | 0000 (0)|0101 (5)| [4][k]  |
    {0, 0, 0, 0, 0, 1, 1, 0}, // | 0000 (0)|0110 (6)| [5][k]  |
    {0, 0, 0, 0, 0, 1, 1, 1}, // | 0000 (0)|0111 (7)| [6][k]  |
    {0, 0, 0, 0, 1, 0, 0, 0}, // | 0000 (0)|1000 (8)| [7][k]  |
    {0, 0, 0, 0, 1, 0, 0, 1}, // | 0000 (0)|1001 (9)| [8][k]  |
    {0, 0, 0, 1, 0, 0, 0, 0}, // | 0001 (1)|0000 (0)| [9][k]  |
    {0, 0, 0, 1, 0, 0, 0, 1}, // | 0001 (1)|0001 (1)| [10][k] |
    {0, 0, 0, 1, 0, 0, 1, 0}, // | 0001 (1)|0010 (2)| [11][k] |
    {0, 0, 0, 1, 0, 0, 1, 1}, // | 0001 (1)|0011 (3)| [12][k] |
    {0, 0, 0, 1, 0, 1, 0, 0}, // | 0001 (1)|0100 (4)| [13][k] |
    {0, 0, 0, 1, 0, 1, 0, 1}, // | 0001 (1)|0101 (5)| [14][k] |
    {0, 0, 0, 1, 0, 1, 1, 0}, // | 0001 (1)|0110 (6)| [15][k] |
    {0, 0, 0, 1, 0, 1, 1, 1}, // | 0001 (1)|0111 (7)| [16][k] |
    {0, 0, 0, 1, 1, 0, 0, 0}, // | 0001 (1)|1000 (8)| [17][k] |
    {0, 0, 0, 1, 1, 0, 0, 1}, // | 0001 (1)|1001 (9)| [18][k] |
    {0, 0, 1, 0, 0, 0, 0, 0}, // | 0010 (2)|0000 (0)| [19][k] |
    {0, 0, 1, 0, 0, 0, 0, 1}, // | 0010 (2)|0001 (1)| [20][k] |
    {0, 0, 1, 0, 0, 0, 1, 0}, // | 0010 (2)|0010 (2)| [21][k] |
    {0, 0, 1, 0, 0, 0, 1, 1}, // | 0010 (2)|0011 (3)| [22][k] |
    {0, 0, 1, 0, 0, 1, 0, 0}, // | 0010 (2)|0100 (4)| [23][k] |
    {0, 0, 1, 0, 0, 1, 0, 1}, // | 0010 (2)|0101 (5)| [24][k] |
    {0, 0, 1, 0, 0, 1, 1, 0}, // | 0010 (2)|0110 (6)| [25][k] |
    {0, 0, 1, 0, 0, 1, 1, 1}, // | 0010 (2)|0111 (7)| [26][k] |
    {0, 0, 1, 0, 1, 0, 0, 0}, // | 0010 (2)|1000 (8)| [27][k] |
    {0, 0, 1, 0, 1, 0, 0, 1}, // | 0010 (2)|1001 (9)| [28][k] |
    {0, 0, 1, 1, 0, 0, 0, 0}, // | 0011 (3)|0000 (0)| [29][k] |
    {0, 0, 1, 1, 0, 0, 0, 1}, // | 0011 (3)|0001 (1)| [30][k] |
    {0, 0, 1, 1, 0, 0, 1, 0}, // | 0011 (3)|0010 (2)| [31][k] |
    {0, 0, 1, 1, 0, 0, 1, 1}, // | 0011 (3)|0011 (3)| [32][k] |
    {0, 0, 1, 1, 0, 1, 0, 0}, // | 0011 (3)|0100 (4)| [33][k] |
    {0, 0, 1, 1, 0, 1, 0, 1}, // | 0011 (3)|0101 (5)| [34][k] |
    {0, 0, 1, 1, 0, 1, 1, 0}, // | 0011 (3)|0110 (6)| [35][k] |
    {0, 0, 1, 1, 0, 1, 1, 1}, // | 0011 (3)|0111 (7)| [36][k] |
    {0, 0, 1, 1, 1, 0, 0, 0}, // | 0011 (3)|1000 (8)| [37][k] |
    {0, 0, 1, 1, 1, 0, 0, 1}, // | 0011 (3)|1001 (9)| [38][k] |
    {0, 1, 0, 0, 0, 0, 0, 0}, // | 0100 (4)|0000 (0)| [39][k] |
    {0, 1, 0, 0, 0, 0, 0, 1}, // | 0100 (4)|0001 (1)| [40][k] |
    {0, 1, 0, 0, 0, 0, 1, 0}, // | 0100 (4)|0010 (2)| [41][k] |
    {0, 1, 0, 0, 0, 0, 1, 1}, // | 0100 (4)|0011 (3)| [42][k] |
    {0, 1, 0, 0, 0, 1, 0, 0}, // | 0100 (4)|0100 (4)| [43][k] |
    {0, 1, 0, 0, 0, 1, 0, 1}, // | 0100 (4)|0101 (5)| [44][k] |
    {0, 1, 0, 0, 0, 1, 1, 0}, // | 0100 (4)|0110 (6)| [45][k] |
    {0, 1, 0, 0, 0, 1, 1, 1}, // | 0100 (4)|0111 (7)| [46][k] |
    {0, 1, 0, 0, 1, 0, 0, 0}, // | 0100 (4)|1000 (8)| [47][k] |
    {0, 1, 0, 0, 1, 0, 0, 1}, // | 0100 (4)|1001 (9)| [48][k] |
    {0, 1, 0, 1, 0, 0, 0, 0}, // | 0101 (5)|0000 (0)| [49][k] |
    {0, 1, 0, 1, 0, 0, 0, 1}, // | 0101 (5)|0001 (1)| [50][k] |
    {0, 1, 0, 1, 0, 0, 1, 0}, // | 0101 (5)|0010 (2)| [51][k] |
    {0, 1, 0, 1, 0, 0, 1, 1}, // | 0101 (5)|0011 (3)| [52][k] |
    {0, 1, 0, 1, 0, 1, 0, 0}, // | 0101 (5)|0100 (4)| [53][k] |
    {0, 1, 0, 1, 0, 1, 0, 1}, // | 0101 (5)|0101 (5)| [54][k] |
    {0, 1, 0, 1, 0, 1, 1, 0}, // | 0101 (5)|0110 (6)| [55][k] |
    {0, 1, 0, 1, 0, 1, 1, 1}, // | 0101 (5)|0111 (7)| [56][k] |
    {0, 1, 0, 1, 1, 0, 0, 0}, // | 0101 (5)|1000 (8)| [57][k] |
    {0, 1, 0, 1, 1, 0, 0, 1}, // | 0101 (5)|1001 (9)| [58][k] |
    {0, 1, 1, 0, 0, 0, 0, 0}, // | 0110 (6)|0000 (0)| [59][k] |
    {0, 1, 1, 0, 0, 0, 0, 1}, // | 0110 (6)|0001 (1)| [60][k] |
    {0, 1, 1, 0, 0, 0, 1, 0}, // | 0110 (6)|0010 (2)| [61][k] |
    {0, 1, 1, 0, 0, 0, 1, 1}, // | 0110 (6)|0011 (3)| [62][k] |
    {0, 1, 1, 0, 0, 1, 0, 0}, // | 0110 (6)|0100 (4)| [63][k] |
    {0, 1, 1, 0, 0, 1, 0, 1}, // | 0110 (6)|0101 (5)| [64][k] |
    {0, 1, 1, 0, 0, 1, 1, 0}, // | 0110 (6)|0110 (6)| [65][k] |
    {0, 1, 1, 0, 0, 1, 1, 1}, // | 0110 (6)|0111 (7)| [66][k] |
    {0, 1, 1, 0, 1, 0, 0, 0}, // | 0110 (6)|1000 (8)| [67][k] |
    {0, 1, 1, 0, 1, 0, 0, 1}, // | 0110 (6)|1001 (9)| [68][k] |
    {0, 1, 1, 1, 0, 0, 0, 0}, // | 0111 (7)|0000 (0)| [69][k] |
    {0, 1, 1, 1, 0, 0, 0, 1}, // | 0111 (7)|0001 (1)| [70][k] |
    {0, 1, 1, 1, 0, 0, 1, 0}  // | 0111 (7)|0010 (2)| [71][k] |
};

/*Pin array with 
    {EN5, EN4, EN3, EN2, EN1, S3, S2, S1, S0, D1, C1, B1, A1, D0, C0, B0, A0} structure*/
int pin_demux[9] = {p_EN5, p_EN4, p_EN3, p_EN2, p_EN1, p_S3, p_S2, p_S1, p_S0};
int pin_disp[8] = {p_D1, p_C1, p_B1, p_A1, p_D0, p_C0, p_B0, p_A0};

void setup()
{
    begin();
}

void loop()
{
    RunTest();
}

/*!
*   @brief Sets up pin modes, functionality and interruptions
*/
void begin()
{
    /*Jumpers*/
    pinMode(p_J24, INPUT_PULLUP);
    pinMode(p_J72, INPUT_PULLUP);

    /*Button*/
    pinMode(p_Man, INPUT_PULLUP);

    /*Defines which program to run between:
    - 72 electrodes manually (_Manual & _J72)
    - 72 electrodes automatically with 1000ms delay between them (_J72)
    - 24 electrodes manually (_Manual & _J24)
    - 24 electrodes automatically with 1000ms delay between them (_J24)
     */
    if (digitalRead(p_Man) == LOW)
    {
        _Manual = true;
        if (digitalRead(p_J72) == LOW)
        {
            _J72 = true;
        }
        else
        {
            _J72 = false;
            if (digitalRead(p_J24) == LOW)
            {
                _J24 = true;
            }
            else
            {
                _J24 = false;
            }
        }
    }
    else
    {
        _Manual = false;
        if (digitalRead(p_J72) == LOW)
        {
            _J72 = true;
        }
        else
        {
            _J72 = false;
            if (digitalRead(p_J24) == LOW)
            {
                _J24 = true;
            }
            else
            {
                _J24 = false;
            }
        }
    }

    /*Starts interruptions for the pin connected to the button*/
    pinMode(p_Man, INPUT);
    attachInterrupt(digitalPinToInterrupt(p_Man), DebounceCount, RISING);

    /*7Segments*/
    pinMode(p_D1, OUTPUT);
    pinMode(p_C1, OUTPUT);
    pinMode(p_B1, OUTPUT);
    pinMode(p_A1, OUTPUT);
    pinMode(p_D0, OUTPUT);
    pinMode(p_C0, OUTPUT);
    pinMode(p_B0, OUTPUT);
    pinMode(p_A0, OUTPUT);

    /*Demux*/
    pinMode(p_EN5, OUTPUT);
    pinMode(p_EN4, OUTPUT);
    pinMode(p_EN3, OUTPUT);
    pinMode(p_EN2, OUTPUT);
    pinMode(p_EN1, OUTPUT);
    pinMode(p_S3, OUTPUT);
    pinMode(p_S2, OUTPUT);
    pinMode(p_S1, OUTPUT);
    pinMode(p_S0, OUTPUT);
}

/*!
*   @brief ISR function for Button interruption, includes a debouncer
*/
void DebounceCount()
{
    if (millis() - startTime > timeThreshold)
    {
        buttonPress = true;
        startTime = millis();
    }
}

/*!
*   @brief Runs the selected program (selected with Jumpers)
*/
void RunTest()
{
    if (_J72)
    {
        int numElectrode = 0;
        if (_Manual) /*When manual program is selected, each time the button is pressed the next channel is selected*/
        {
            if (buttonPress)
            {
                for (int j = 0; j < 9; j++)
                {
                    digitalWrite(pin_demux[j], electrodes[numElectrode][j]);
                }
                for (int k = 0; k < 8; k++)
                {
                    digitalWrite(pin_disp[k], display[numElectrode][k]);
                }
                if (numElectrode < 71)
                {
                    numElectrode++;
                }
                else
                {
                    numElectrode = 0;
                }
            }
            buttonPress = false;
        }
        else
        {
            for (int i = 0; i < 72; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    digitalWrite(pin_demux[j], electrodes[i][j]);
                }
                for (int k = 0; k < 8; k++)
                {
                    digitalWrite(pin_disp[k], display[i][k]);
                } 
                delay(1000);
            }
        }
    }
    else if (_J24)
    {
        int numElectrode = 24;
        if (_Manual)
        {
            if (buttonPress)
            {
                for (int j = 0; j < 9; j++)
                {
                    digitalWrite(pin_demux[j], electrodes[numElectrode][j]);
                }
                for (int k = 0; k < 8; k++)
                {
                    digitalWrite(pin_disp[k], display[numElectrode - 24][k]);
                }
                if (numElectrode < 47)
                {
                    numElectrode++;
                }
                else
                {
                    numElectrode = 24;
                }
            }
            buttonPress = false;
        }
        else
        {
            for (int i = 24; i < 48; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    digitalWrite(pin_demux[j], electrodes[i][j]);
                }
                for (int k = 0; k < 8; k++)
                {
                    digitalWrite(pin_disp[k], display[i - 24][k]);
                }
                delay(1000);
            }
        }
    }
}